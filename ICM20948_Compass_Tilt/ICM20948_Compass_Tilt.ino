/**
 * @file ICM20948_Compass_Tilt.ino
 * @brief Compass and inclinometer with Mahony AHRS and built-in calibration
 * 
 * Hardware: Arduino Pro Mini, ICM-20948 IMU, OLED 128x32
 * Based on: jremington/ICM_20948-AHRS, Cave Pearl Project
 * Location: Zywiec, Poland (49.6853N, 19.1925E), Declination: 5.5E
 * @license MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "ICM_20948.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// --- OLED Display ---
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     32
#define OLED_RESET        -1
#define OLED_I2C_ADDRESS  0x3C

// --- ICM-20948 ---
#define ICM_AD0_VAL       1
#define ICM_I2C_SPEED     400000

// --- Location (Zywiec, Poland) ---
#define MAGNETIC_DECLINATION  5.5f    // Declination (E = positive)

// --- Parametry filtra Mahony AHRS ---
#define MAHONY_KP             50.0f
#define MAHONY_KI             0.0f

// --- Gyroscope ---
#define GYRO_SCALE            ((M_PI / 180.0f) * 0.00763f)

// --- Auto-calibration ---
#define SHORT_RUN_THRESHOLD_MS    2000
#define SHORT_RUNS_TO_CALIBRATE   3
#define GYRO_CAL_SAMPLES          500

// --- Mag calibration params ---
#define CAL_MIN_TIME_MS       5000
#define CAL_MAX_TIME_MS       60000
#define CAL_MIN_RANGE         100.0f
#define CAL_CHECK_INTERVAL_MS 500

// --- Filters ---
#define EMA_ALPHA             0.10f
#define DISPLAY_UPDATE_MS     250

// --- Mag validation ---
#define MAG_VALID_MAX         5000.0f

// --- EEPROM map ---
#define EEPROM_MAGIC_ADDR         0
#define EEPROM_SHORT_RUNS_ADDR    4
#define EEPROM_CAL_VALID_ADDR     8
#define EEPROM_GYRO_OFF_ADDR      12    // 3 floaty = 12 bajtów
#define EEPROM_MAG_MIN_ADDR       24    // 3 floaty = 12 bajtów  
#define EEPROM_MAG_MAX_ADDR       36    // 3 floaty = 12 bajtów
#define EEPROM_CRC_ADDR           48
#define EEPROM_MAGIC_VALUE        0xCAFE

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ICM_20948_I2C    imu;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct SensorData {
  float roll, pitch, yaw;
  float headingMag, headingGeo;
  float rollFiltered, pitchFiltered;
  float headingMagFiltered, headingGeoFiltered;
  bool filtersReady;
};

/**
 * Calibration struct (Cave Pearl Project method)
 */
struct Calibration {
  float gyroOffset[3];
  float magMin[3];
  float magMax[3];
  bool isValid;
};

struct AHRSState {
  float q[4];
  float eInt[3];
  unsigned long lastUpdate;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

SensorData    g_sensor = {0};
Calibration   g_cal = {{0,0,0}, {-200,-200,-200}, {200,200,200}, false};
AHRSState     g_ahrs = {{1.0f, 0.0f, 0.0f, 0.0f}, {0,0,0}, 0};
unsigned long g_startupTime = 0;
unsigned long g_lastDisplayUpdate = 0;

// ============================================================================
// PROTOTYPES
// ============================================================================

void initDisplay(void);
bool initIMU(void);
void configureIMU(void);
void initEEPROM(void);
void loadCalibration(void);
void saveCalibration(void);
void incrementShortRunCount(void);
bool shouldTriggerCalibration(void);
void resetShortRunCount(void);
uint8_t calculateCRC8(void);

void runGyroCalibration(void);
void runMagCalibration(void);
void applyMagCalibration(float raw[3], float calibrated[3]);

void processSensors(void);
bool validateMagData(void);

float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, 
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float deltat);
void quaternionToEuler(float q[4], float& yaw, float& pitch, float& roll);

float emaFilter(float newVal, float oldVal, float alpha);
float emaFilterAngle(float newAngle, float oldAngle, float alpha);

void updateDisplay(void);
float getDeviationFromNorth(float heading);
char getCardinalChar(float heading);
void showMsg(const char* l1, const char* l2 = nullptr, const char* l3 = nullptr);
void trackRuntime(void);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  g_startupTime = millis();
  
  Wire.begin();
  Wire.setClock(ICM_I2C_SPEED);
  
  initDisplay();
  showMsg("Init...");
  
  if (!initIMU()) {
    showMsg("IMU Err!", "Check conn");
    while (true) delay(100);
  }
  
  configureIMU();
  initEEPROM();
  incrementShortRunCount();
  loadCalibration();
  
  // Check if calibration needed
  if (shouldTriggerCalibration()) {
    showMsg("=CALIBRATION=", "1.Hold still", "2.Rotate slow");
    delay(3000);
    runGyroCalibration();
    runMagCalibration();
    showMsg("Cal done!", "Restart...");
    while (true) delay(100);
  }
  
  // If no calibration, do gyro only
  if (!g_cal.isValid) {
    showMsg("No cal", "Gyro cal...", "Hold still");
    delay(1000);
    runGyroCalibration();
    showMsg("3x restart", "for full cal");
    delay(3000);
  }
  
  g_ahrs.lastUpdate = micros();
  
  showMsg("ICM20948 OK", "Mahony AHRS");
  delay(1500);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  trackRuntime();
  
  if (imu.dataReady()) {
    imu.getAGMT();
    processSensors();
  }
  
  if (millis() - g_lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    g_lastDisplayUpdate = millis();
    updateDisplay();
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initDisplay(void) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

bool initIMU(void) {
  imu.begin(Wire, ICM_AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    imu.begin(Wire, !ICM_AD0_VAL);
  }
  return (imu.status == ICM_20948_Stat_Ok);
}

void configureIMU(void) {
  imu.swReset();
  delay(250);
  
  imu.sleep(false);
  imu.lowPower(false);
  
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                    ICM_20948_Sample_Mode_Continuous);
  
  ICM_20948_fss_t fss;
  fss.a = gpm2;
  fss.g = dps250;
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  ICM_20948_dlpcfg_t dlp;
  dlp.a = acc_d50bw4_n68bw8;
  dlp.g = gyr_d51bw2_n73bw3;
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
  
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  for (int attempt = 0; attempt < 5; attempt++) {
    imu.startupMagnetometer();
    delay(100);
    if (imu.dataReady()) {
      imu.getAGMT();
      if (imu.magX() != 0 || imu.magY() != 0 || imu.magZ() != 0) break;
    }
  }
}

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================

static uint8_t crc8UpdateByte(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}

static uint8_t crc8UpdateFloat(uint8_t crc, float value) {
  uint8_t* ptr = (uint8_t*)&value;
  for (size_t i = 0; i < sizeof(float); i++) {
    crc = crc8UpdateByte(crc, ptr[i]);
  }
  return crc;
}

uint8_t calculateCRC8(void) {
  uint8_t crc = 0x00;
  for (int i = 0; i < 3; i++) crc = crc8UpdateFloat(crc, g_cal.gyroOffset[i]);
  for (int i = 0; i < 3; i++) crc = crc8UpdateFloat(crc, g_cal.magMin[i]);
  for (int i = 0; i < 3; i++) crc = crc8UpdateFloat(crc, g_cal.magMax[i]);
  return crc;
}

void initEEPROM(void) {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC_VALUE) {
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint16_t)EEPROM_MAGIC_VALUE);
    EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
    EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)0);
  }
}

void loadCalibration(void) {
  uint8_t valid;
  EEPROM.get(EEPROM_CAL_VALID_ADDR, valid);
  
  if (valid == 1) {
    int addr = EEPROM_GYRO_OFF_ADDR;
    for (int i = 0; i < 3; i++) {
      EEPROM.get(addr, g_cal.gyroOffset[i]);
      addr += sizeof(float);
    }
    addr = EEPROM_MAG_MIN_ADDR;
    for (int i = 0; i < 3; i++) {
      EEPROM.get(addr, g_cal.magMin[i]);
      addr += sizeof(float);
    }
    addr = EEPROM_MAG_MAX_ADDR;
    for (int i = 0; i < 3; i++) {
      EEPROM.get(addr, g_cal.magMax[i]);
      addr += sizeof(float);
    }
    
    uint8_t storedCRC;
    EEPROM.get(EEPROM_CRC_ADDR, storedCRC);
    
    if (storedCRC == calculateCRC8()) {
      g_cal.isValid = true;
    } else {
      // Reset to defaults
      for (int i = 0; i < 3; i++) {
        g_cal.gyroOffset[i] = 0;
        g_cal.magMin[i] = -200;
        g_cal.magMax[i] = 200;
      }
      g_cal.isValid = false;
      showMsg("CRC Err!", "Recal");
      delay(2000);
    }
  }
}

void saveCalibration(void) {
  int addr = EEPROM_GYRO_OFF_ADDR;
  for (int i = 0; i < 3; i++) {
    EEPROM.put(addr, g_cal.gyroOffset[i]);
    addr += sizeof(float);
  }
  addr = EEPROM_MAG_MIN_ADDR;
  for (int i = 0; i < 3; i++) {
    EEPROM.put(addr, g_cal.magMin[i]);
    addr += sizeof(float);
  }
  addr = EEPROM_MAG_MAX_ADDR;
  for (int i = 0; i < 3; i++) {
    EEPROM.put(addr, g_cal.magMax[i]);
    addr += sizeof(float);
  }
  
  EEPROM.put(EEPROM_CRC_ADDR, calculateCRC8());
  EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)1);
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
  g_cal.isValid = true;
}

void incrementShortRunCount(void) {
  uint8_t count;
  EEPROM.get(EEPROM_SHORT_RUNS_ADDR, count);
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)(count + 1));
}

bool shouldTriggerCalibration(void) {
  uint8_t count;
  EEPROM.get(EEPROM_SHORT_RUNS_ADDR, count);
  if (count >= SHORT_RUNS_TO_CALIBRATE) {
    EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
    return true;
  }
  return false;
}

void resetShortRunCount(void) {
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
}

// ============================================================================
// CALIBRATION
// ============================================================================

/**
 * Gyroscope calibration - averaging offsets
 */
void runGyroCalibration(void) {
  showMsg("GYRO Cal", "Hold still!");
  delay(1500);
  
  long gyroSum[3] = {0, 0, 0};
  int count = 0;
  
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (imu.dataReady()) {
      imu.getAGMT();
      gyroSum[0] += imu.agmt.gyr.axes.x;
      gyroSum[1] += imu.agmt.gyr.axes.y;
      gyroSum[2] += imu.agmt.gyr.axes.z;
      count++;
    }
    delay(5);
    
    // Update display every 100 samples
    if (i % 100 == 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("GYRO "));
      display.print((i * 100) / GYRO_CAL_SAMPLES);
      display.println(F("%"));
      display.display();
    }
  }
  
  if (count > 0) {
    g_cal.gyroOffset[0] = (float)gyroSum[0] / count;
    g_cal.gyroOffset[1] = (float)gyroSum[1] / count;
    g_cal.gyroOffset[2] = (float)gyroSum[2] / count;
  }
  
  showMsg("GYRO OK!");
  delay(800);
}

/**
 * Magnetometer calibration using min/max method
 */
void runMagCalibration(void) {
  showMsg("MAG Cal", "Rotate slowly");
  delay(1500);
  
  // Initialize min/max
  float magMin[3] = {32767.0f, 32767.0f, 32767.0f};
  float magMax[3] = {-32767.0f, -32767.0f, -32767.0f};
  
  unsigned long startTime = millis();
  unsigned long lastCheck = startTime;
  bool complete = false;
  
  while (!complete && (millis() - startTime < CAL_MAX_TIME_MS)) {
    if (imu.dataReady()) {
      imu.getAGMT();
      
      // Axis mapping for AK09916
      float mx = imu.magY();
      float my = -imu.magX();
      float mz = -imu.magZ();
      
      // Update min/max
      if (mx < magMin[0]) magMin[0] = mx;
      if (mx > magMax[0]) magMax[0] = mx;
      if (my < magMin[1]) magMin[1] = my;
      if (my > magMax[1]) magMax[1] = my;
      if (mz < magMin[2]) magMin[2] = mz;
      if (mz > magMax[2]) magMax[2] = mz;
      
      // Check progress
      if (millis() - lastCheck >= CAL_CHECK_INTERVAL_MS) {
        lastCheck = millis();
        
        float rangeX = magMax[0] - magMin[0];
        float rangeY = magMax[1] - magMin[1];
        unsigned long elapsed = millis() - startTime;
        
        bool xOk = (rangeX >= CAL_MIN_RANGE);
        bool yOk = (rangeY >= CAL_MIN_RANGE);
        bool minTime = (elapsed >= CAL_MIN_TIME_MS);
        
        if (xOk && yOk && minTime) complete = true;
        
        // Show compact status
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print(xOk ? F("+") : F("X"));
        display.print(yOk ? F("+") : F("Y"));
        if (!minTime) {
          display.print(F(" "));
          display.print((CAL_MIN_TIME_MS - elapsed) / 1000);
          display.print(F("s"));
        }
        display.setCursor(0, 11);
        display.print((int)rangeX);
        display.print(F("/"));
        display.print((int)rangeY);
        display.display();
      }
    }
    delay(10);
  }
  
  // Save results
  for (int i = 0; i < 3; i++) {
    g_cal.magMin[i] = magMin[i];
    g_cal.magMax[i] = magMax[i];
  }
  
  saveCalibration();
  
  showMsg(complete ? "MAG OK!" : "MAG Timeout", "Saved");
  delay(1500);
}

/**
 * Apply magnetometer calibration (Hard/Soft Iron correction)
 */
void applyMagCalibration(float raw[3], float calibrated[3]) {
  float offset[3], delta[3], scale[3];
  
  for (int i = 0; i < 3; i++) {
    offset[i] = (g_cal.magMax[i] + g_cal.magMin[i]) * 0.5f;
    delta[i] = g_cal.magMax[i] - g_cal.magMin[i];
  }
  
  float avgDelta = (delta[0] + delta[1] + delta[2]) / 3.0f;
  
  for (int i = 0; i < 3; i++) {
    scale[i] = (delta[i] > 1.0f) ? avgDelta / delta[i] : 1.0f;
    calibrated[i] = (raw[i] - offset[i]) * scale[i];
  }
}

// ============================================================================
// VECTOR MATH
// ============================================================================

float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  if (mag > 1e-9f) {
    a[0] /= mag;
    a[1] /= mag;
    a[2] /= mag;
  }
}

// ============================================================================
// MAHONY AHRS
// ============================================================================

/**
 * Mahony quaternion update with Up and West reference vectors
 */
void MahonyQuaternionUpdate(float ax, float ay, float az, 
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float deltat) {
  float q1 = g_ahrs.q[0], q2 = g_ahrs.q[1], q3 = g_ahrs.q[2], q4 = g_ahrs.q[3];
  float norm;
  float hx, hy, hz;
  float ux, uy, uz, wx, wy, wz;
  float ex, ey, ez;
  
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
  float q3q3 = q3 * q3, q3q4 = q3 * q4, q4q4 = q4 * q4;
  
  // Horizon vector = a x m (body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm < 1e-9f) return;
  norm = 1.0f / norm;
  hx *= norm; hy *= norm; hz *= norm;
  
  // Estimated Up direction
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;
  
  // Estimated West direction
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);
  
  // Error = cross product of estimated and measured
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  
  // Integral feedback
  if (MAHONY_KI > 0.0f) {
    g_ahrs.eInt[0] += ex;
    g_ahrs.eInt[1] += ey;
    g_ahrs.eInt[2] += ez;
    gx += MAHONY_KI * g_ahrs.eInt[0];
    gy += MAHONY_KI * g_ahrs.eInt[1];
    gz += MAHONY_KI * g_ahrs.eInt[2];
  }
  
  // Proportional feedback
  gx = gx + MAHONY_KP * ex;
  gy = gy + MAHONY_KP * ey;
  gz = gz + MAHONY_KP * ez;
  
  // Quaternion update
  gx = gx * (0.5f * deltat);
  gy = gy * (0.5f * deltat);
  gz = gz * (0.5f * deltat);
  
  float qa = q1, qb = q2, qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);
  
  // Quaternion normalization
  norm = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  norm = 1.0f / norm;
  g_ahrs.q[0] = q1 * norm;
  g_ahrs.q[1] = q2 * norm;
  g_ahrs.q[2] = q3 * norm;
  g_ahrs.q[3] = q4 * norm;
}

/**
 * Quaternion to Euler angles (NWU orientation)
 */
void quaternionToEuler(float q[4], float& yaw, float& pitch, float& roll) {
  roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5f - (q[1] * q[1] + q[2] * q[2]));
  pitch = asin(2.0f * (q[0] * q[2] - q[1] * q[3]));
  yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5f - (q[2] * q[2] + q[3] * q[3]));
  
  yaw   *= 180.0f / PI;
  pitch *= 180.0f / PI;
  roll  *= 180.0f / PI;
}

// ============================================================================
// SENSOR PROCESSING
// ============================================================================

bool validateMagData(void) {
  float mx = imu.magX();
  float my = imu.magY();
  float mz = imu.magZ();
  
  if (mx == 0 && my == 0 && mz == 0) return false;
  if (fabs(mx) > MAG_VALID_MAX || fabs(my) > MAG_VALID_MAX || fabs(mz) > MAG_VALID_MAX) return false;
  return true;
}

void processSensors(void) {
  float Gxyz[3], Axyz[3], Mxyz[3], McalXyz[3];
  
  // Read gyroscope (rad/s with offset)
  Gxyz[0] = GYRO_SCALE * (imu.agmt.gyr.axes.x - g_cal.gyroOffset[0]);
  Gxyz[1] = GYRO_SCALE * (imu.agmt.gyr.axes.y - g_cal.gyroOffset[1]);
  Gxyz[2] = GYRO_SCALE * (imu.agmt.gyr.axes.z - g_cal.gyroOffset[2]);
  
  // Read and normalize accelerometer
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  vector_normalize(Axyz);
  
  // Read magnetometer with axis mapping for AK09916
  Mxyz[0] = imu.magY();
  Mxyz[1] = -imu.magX();
  Mxyz[2] = -imu.magZ();
  
  // Apply calibration
  applyMagCalibration(Mxyz, McalXyz);
  vector_normalize(McalXyz);
  
  // Align mag axes with accel
  McalXyz[1] = -McalXyz[1];
  McalXyz[2] = -McalXyz[2];
  
  // Calculate delta time
  unsigned long now = micros();
  float deltat = (now - g_ahrs.lastUpdate) * 1.0e-6f;
  g_ahrs.lastUpdate = now;
  
  if (!validateMagData()) return;
  
  // Update Mahony AHRS
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2],
                         Gxyz[0], Gxyz[1], Gxyz[2],
                         McalXyz[0], McalXyz[1], McalXyz[2], deltat);
  
  // Convert to Euler angles
  float yaw, pitch, roll;
  quaternionToEuler(g_ahrs.q, yaw, pitch, roll);
  
  // Apply declination
  yaw = -(yaw + MAGNETIC_DECLINATION);
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
  
  g_sensor.roll = roll;
  g_sensor.pitch = pitch;
  g_sensor.yaw = yaw;
  g_sensor.headingGeo = yaw;
  
  // Magnetic heading (without declination)
  g_sensor.headingMag = yaw - MAGNETIC_DECLINATION;
  if (g_sensor.headingMag < 0) g_sensor.headingMag += 360.0f;
  if (g_sensor.headingMag >= 360.0f) g_sensor.headingMag -= 360.0f;
  
  // EMA filter
  if (!g_sensor.filtersReady) {
    g_sensor.rollFiltered = g_sensor.roll;
    g_sensor.pitchFiltered = g_sensor.pitch;
    g_sensor.headingMagFiltered = g_sensor.headingMag;
    g_sensor.headingGeoFiltered = g_sensor.headingGeo;
    g_sensor.filtersReady = true;
  } else {
    g_sensor.rollFiltered = emaFilter(g_sensor.roll, g_sensor.rollFiltered, EMA_ALPHA);
    g_sensor.pitchFiltered = emaFilter(g_sensor.pitch, g_sensor.pitchFiltered, EMA_ALPHA);
    g_sensor.headingMagFiltered = emaFilterAngle(g_sensor.headingMag, g_sensor.headingMagFiltered, EMA_ALPHA);
    g_sensor.headingGeoFiltered = emaFilterAngle(g_sensor.headingGeo, g_sensor.headingGeoFiltered, EMA_ALPHA);
  }
}

// ============================================================================
// FILTER FUNCTIONS
// ============================================================================

float emaFilter(float newVal, float oldVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

float emaFilterAngle(float newAngle, float oldAngle, float alpha) {
  float newRad = newAngle * PI / 180.0f;
  float oldRad = oldAngle * PI / 180.0f;
  
  float newX = cos(newRad), newY = sin(newRad);
  float oldX = cos(oldRad), oldY = sin(oldRad);
  
  float filtX = alpha * newX + (1.0f - alpha) * oldX;
  float filtY = alpha * newY + (1.0f - alpha) * oldY;
  
  float result = atan2(filtY, filtX) * 180.0f / PI;
  if (result < 0) result += 360.0f;
  return result;
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void updateDisplay(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: Tilt angles
  display.setCursor(0, 0);
  display.print(F("X:"));
  display.print(g_sensor.rollFiltered, 1);
  display.print(F(" Y:"));
  display.print(g_sensor.pitchFiltered, 1);
  
  float magDev = getDeviationFromNorth(g_sensor.headingMagFiltered);
  float geoDev = getDeviationFromNorth(g_sensor.headingGeoFiltered);
  
  // Line 2: Magnetic north
  display.setCursor(0, 11);
  display.print(F("M:"));
  display.print(magDev, 1);
  display.print((char)247);
  display.print(getCardinalChar(g_sensor.headingMagFiltered));
  
  // Line 3: Geographic north
  display.setCursor(0, 22);
  display.print(F("G:"));
  display.print(geoDev, 1);
  display.print((char)247);
  display.print(getCardinalChar(g_sensor.headingGeoFiltered));
  
  display.display();
}

float getDeviationFromNorth(float heading) {
  float deviation = heading;
  if (deviation > 180.0f) deviation -= 360.0f;
  return deviation;
}

char getCardinalChar(float heading) {
  if (heading >= 315.0f || heading < 45.0f)  return 'N';
  if (heading >= 45.0f  && heading < 135.0f) return 'E';
  if (heading >= 135.0f && heading < 225.0f) return 'S';
  return 'W';
}

void showMsg(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(l1);
  if (l2) { display.setCursor(0, 11); display.println(l2); }
  if (l3) { display.setCursor(0, 22); display.println(l3); }
  
  display.display();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void trackRuntime(void) {
  static bool normalOpRecorded = false;
  if (!normalOpRecorded && (millis() - g_startupTime >= SHORT_RUN_THRESHOLD_MS)) {
    normalOpRecorded = true;
    resetShortRunCount();
  }
}
