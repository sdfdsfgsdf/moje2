/*
 * ICM-20948 Compass & Tilt Meter for Arduino Mini
 * Hardware: Arduino Mini Pro, ICM-20948, OLED 128x32
 * Location: Zywiec, Poland (49.68N, 19.19E) - Declination: 5.5E
 * License: MIT
 * 
 * Auto-calibration: If 3 consecutive startups last less than 2 seconds,
 * the device triggers automatic calibration.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <math.h>
#include "ICM_20948.h"

// ============================================
// CONFIGURATION
// ============================================

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1  // Reset pin (-1 if sharing Arduino reset)
#define OLED_ADDRESS 0x3C

// ICM-20948 settings
#define ICM_ADDRESS 0x69  // Default I2C address (0x68 if AD0 is low)

// Auto-calibration settings / Ustawienia automatycznej kalibracji
#define SHORT_RUN_THRESHOLD 2000  // Czas krótkiego uruchomienia (2 sekundy)

// EEPROM addresses / Adresy EEPROM
#define EEPROM_MAGIC_ADDR 0           // Magic number to verify data validity
#define EEPROM_SHORT_RUN_COUNT_ADDR 4 // Counter for short runs
#define EEPROM_CAL_VALID_ADDR 8       // Calibration data valid flag
#define EEPROM_MAG_OFFSET_X_ADDR 12   // Magnetometer offset X
#define EEPROM_MAG_OFFSET_Y_ADDR 16   // Magnetometer offset Y
#define EEPROM_MAG_OFFSET_Z_ADDR 20   // Magnetometer offset Z
#define EEPROM_MAG_SCALE_X_ADDR 24    // Magnetometer scale X
#define EEPROM_MAG_SCALE_Y_ADDR 28    // Magnetometer scale Y
#define EEPROM_MAG_SCALE_Z_ADDR 32    // Magnetometer scale Z

#define EEPROM_MAGIC_VALUE 0xCAFE     // Magic value to check EEPROM validity

// Magnetic declination for Żywiec, Poland (49.6853°N, 19.1925°E)
// Declination is approximately 5.5° East (positive value)
// Update this value periodically as it changes over time
// Source: NOAA World Magnetic Model
#define MAGNETIC_DECLINATION 5.5

// Display update interval (ms) - uśrednianie co 250ms
#define DISPLAY_UPDATE_INTERVAL 250

// Calibration settings
#define CALIBRATION_SAMPLES 100
#define CALIBRATION_MAX_TIME 60000      // Maximum calibration time (60 seconds)
#define CALIBRATION_MIN_RANGE 100.0     // Minimum range required for each axis (uT)
#define CALIBRATION_CHECK_INTERVAL 500  // Check calibration quality every 500ms

// ============================================
// GLOBAL OBJECTS
// ============================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ICM_20948_I2C imu;

// ============================================
// GLOBAL VARIABLES
// ============================================

// Sensor data
float roll = 0;    // Rotation around X axis (degrees)
float pitch = 0;   // Rotation around Y axis (degrees)
float yaw = 0;     // Rotation around Z axis (degrees)
float headingMag = 0;   // Magnetic heading (degrees)
float headingTrue = 0;  // True heading (degrees)

// Averaging buffers / Bufory uśredniania
float rollSum = 0, pitchSum = 0, yawSum = 0;
float headingMagSum = 0, headingTrueSum = 0;
uint8_t avgCount = 0;  // uint8_t max 255 - wystarczy dla 250ms

// Magnetometer calibration offsets (hard iron)
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;

// Magnetometer calibration scale (soft iron)
float magScaleX = 1.0;
float magScaleY = 1.0;
float magScaleZ = 1.0;

// Display mode (tylko 1 tryb teraz - wszystkie dane)
unsigned long lastDisplayUpdate = 0;

// Startup time tracking / Śledzenie czasu uruchomienia
unsigned long startupTime = 0;
bool calibrationMode = false;  // True if we entered calibration mode

// ============================================
// SETUP
// ============================================

void setup() {
  // Record startup time
  startupTime = millis();
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    while (1) {
      delay(100);
    }
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();
  
  // Initialize ICM-20948
  imu.begin(Wire, 1); // AD0 = 1 -> address 0x69
  
  if (imu.status != ICM_20948_Stat_Ok) {
    imu.begin(Wire, 0); // AD0 = 0 -> address 0x68
  }
  
  if (imu.status != ICM_20948_Stat_Ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("IMU Error!"));
    display.println(F("Check wiring"));
    display.display();
    while (1) {
      delay(100);
    }
  }
  
  // Configure IMU
  configureIMU();
  
  // Initialize EEPROM and check for auto-calibration
  initializeEEPROM();
  
  // Increment short run counter (assume this is a short run)
  // Will be reset to 0 if we survive past threshold
  incrementShortRunCounter();
  
  // Load calibration data from EEPROM
  loadCalibrationFromEEPROM();
  
  // Check if we should enter calibration mode (3rd short run)
  if (checkAutoCalibration()) {
    calibrationMode = true;
    calibrateMagnetometer();
    // After calibration, show restart message and halt
    showRestartMessage();
    while (1) {
      delay(100);
    }
  }
  
  // Show startup message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("ICM-20948 Ready"));
  display.println(F("Zywiec, Poland"));
  display.println(F("Decl: 5.5E"));
  display.display();
  delay(2000);
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Track running time for short run detection
  trackRunningTime();
  
  // Read sensor data
  if (imu.dataReady()) {
    imu.getAGMT(); // Get Accel, Gyro, Mag, Temp
    
    // Calculate tilt angles from accelerometer
    calculateTilt();
    
    // Calculate compass heading from magnetometer
    calculateHeading();
    
    // Accumulate for averaging / Zbieraj do uśredniania
    rollSum += roll;
    pitchSum += pitch;
    yawSum += yaw;
    headingMagSum += headingMag;
    headingTrueSum += headingTrue;
    avgCount++;
  }
  
  // Update display at fixed interval (co 250ms)
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    
    // Calculate averages / Oblicz średnie
    if (avgCount > 0) {
      roll = rollSum / avgCount;
      pitch = pitchSum / avgCount;
      yaw = yawSum / avgCount;
      headingMag = headingMagSum / avgCount;
      headingTrue = headingTrueSum / avgCount;
      
      // Reset accumulators / Resetuj akumulatory
      rollSum = pitchSum = yawSum = 0;
      headingMagSum = headingTrueSum = 0;
      avgCount = 0;
    }
    
    updateDisplay();
  }
}

// ============================================
// SENSOR CONFIGURATION
// ============================================

void configureIMU() {
  imu.swReset();
  delay(250);
  
  imu.sleep(false);
  imu.lowPower(false);
  
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  ICM_20948_fss_t fss;
  fss.a = gpm2;
  fss.g = dps250;
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d50bw4_n68bw8;
  dlpcfg.g = gyr_d51bw2_n73bw3;
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  imu.startupMagnetometer();
}

// ============================================
// TILT CALCULATION
// ============================================

void calculateTilt() {
  // Get accelerometer data in g
  float ax = imu.accX() / 1000.0; // Convert mg to g
  float ay = imu.accY() / 1000.0;
  float az = imu.accZ() / 1000.0;
  
  // Calculate roll and pitch from accelerometer
  // Roll: rotation around X axis
  roll = atan2(ay, az) * 180.0 / PI;
  
  // Pitch: rotation around Y axis
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  // Yaw cannot be determined from accelerometer alone
  // It will be calculated from magnetometer heading
}

// ============================================
// COMPASS HEADING CALCULATION
// ============================================

void calculateHeading() {
  // Get magnetometer data
  float mx = imu.magX() - magOffsetX;
  float my = imu.magY() - magOffsetY;
  float mz = imu.magZ() - magOffsetZ;
  
  // Apply soft iron calibration
  mx *= magScaleX;
  my *= magScaleY;
  mz *= magScaleZ;
  
  // Tilt compensation for magnetometer
  // Convert angles to radians
  float rollRad = roll * PI / 180.0;
  float pitchRad = pitch * PI / 180.0;
  
  // Compensate magnetometer readings for tilt
  float cosRoll = cos(rollRad);
  float sinRoll = sin(rollRad);
  float cosPitch = cos(pitchRad);
  float sinPitch = sin(pitchRad);
  
  // Tilt-compensated magnetic field components
  float Xh = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
  float Yh = my * cosRoll - mz * sinRoll;
  
  // Calculate magnetic heading
  headingMag = atan2(-Yh, Xh) * 180.0 / PI;
  
  // Normalize to 0-360 degrees
  if (headingMag < 0) {
    headingMag += 360;
  }
  
  // Calculate true heading by adding magnetic declination
  // For Żywiec, Poland: declination is 5.5° East (positive)
  headingTrue = headingMag + MAGNETIC_DECLINATION;
  
  // Normalize to 0-360 degrees
  if (headingTrue >= 360) {
    headingTrue -= 360;
  }
  if (headingTrue < 0) {
    headingTrue += 360;
  }
  
  // Update yaw with heading
  yaw = headingMag;
}

// ============================================
// MAGNETOMETER CALIBRATION
// ============================================

void calibrateMagnetometer() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("CALIBRATION"));
  display.println(F("Rotate all axes"));
  display.display();
  
  float magMin[3] = {32767, 32767, 32767};
  float magMax[3] = {-32767, -32767, -32767};
  
  unsigned long startTime = millis();
  unsigned long lastCheckTime = startTime;
  bool calibrationComplete = false;
  
  while (!calibrationComplete && (millis() - startTime < CALIBRATION_MAX_TIME)) {
    if (imu.dataReady()) {
      imu.getAGMT();
      
      float mx = imu.magX();
      float my = imu.magY();
      float mz = imu.magZ();
      
      if (mx < magMin[0]) magMin[0] = mx;
      if (mx > magMax[0]) magMax[0] = mx;
      if (my < magMin[1]) magMin[1] = my;
      if (my > magMax[1]) magMax[1] = my;
      if (mz < magMin[2]) magMin[2] = mz;
      if (mz > magMax[2]) magMax[2] = mz;
      
      // Check calibration quality periodically
      if (millis() - lastCheckTime >= CALIBRATION_CHECK_INTERVAL) {
        lastCheckTime = millis();
        
        float rangeX = magMax[0] - magMin[0];
        float rangeY = magMax[1] - magMin[1];
        float rangeZ = magMax[2] - magMin[2];
        
        // Check if all axes have sufficient range
        if (rangeX >= CALIBRATION_MIN_RANGE && 
            rangeY >= CALIBRATION_MIN_RANGE && 
            rangeZ >= CALIBRATION_MIN_RANGE) {
          calibrationComplete = true;
        }
        
        // Update display with progress
        display.clearDisplay();
        display.setCursor(0, 0);
        if (calibrationComplete) {
          display.println(F("CAL OK!"));
        } else {
          display.print(F("CAL: "));
          // Show which axes need more data
          if (rangeX < CALIBRATION_MIN_RANGE) display.print(F("X"));
          if (rangeY < CALIBRATION_MIN_RANGE) display.print(F("Y"));
          if (rangeZ < CALIBRATION_MIN_RANGE) display.print(F("Z"));
        }
        display.setCursor(0, 11);
        display.print(F("X:"));
        display.print((int)rangeX);
        display.print(F(" Y:"));
        display.print((int)rangeY);
        display.setCursor(0, 22);
        display.print(F("Z:"));
        display.print((int)rangeZ);
        display.print(F(" min:"));
        display.print((int)CALIBRATION_MIN_RANGE);
        display.display();
      }
    }
    delay(10);
  }
  
  magOffsetX = (magMax[0] + magMin[0]) / 2.0;
  magOffsetY = (magMax[1] + magMin[1]) / 2.0;
  magOffsetZ = (magMax[2] + magMin[2]) / 2.0;
  
  const float MIN_DELTA = 1.0;
  
  float deltaX = magMax[0] - magMin[0];
  float deltaY = magMax[1] - magMin[1];
  float deltaZ = magMax[2] - magMin[2];
  
  if (fabs(deltaX) < MIN_DELTA) deltaX = MIN_DELTA;
  if (fabs(deltaY) < MIN_DELTA) deltaY = MIN_DELTA;
  if (fabs(deltaZ) < MIN_DELTA) deltaZ = MIN_DELTA;
  
  float avgDelta = (deltaX + deltaY + deltaZ) / 3.0;
  
  magScaleX = avgDelta / deltaX;
  magScaleY = avgDelta / deltaY;
  magScaleZ = avgDelta / deltaZ;
  
  // Save calibration to EEPROM
  saveCalibrationToEEPROM();
  
  display.clearDisplay();
  display.setCursor(0, 0);
  if (calibrationComplete) {
    display.println(F("Cal Done!"));
    display.println(F("Data saved."));
  } else {
    display.println(F("Cal Timeout"));
    display.println(F("Data saved."));
  }
  display.display();
  delay(2000);
}

// ============================================
// DISPLAY UPDATE
// ============================================

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Jeden ekran - wszystkie dane kompaktowo
  // Linia 1: X (Roll), Y (Pitch), Z (Yaw)
  display.setCursor(0, 0);
  display.print(F("X:"));
  display.print(roll, 0);
  display.print(F(" Y:"));
  display.print(pitch, 0);
  display.print(F(" Z:"));
  display.print(yaw, 0);
  
  // Calculate deviations from North
  float magDeviation = getDeviationFromNorth(headingMag);
  float geoDeviation = getDeviationFromNorth(headingTrue);
  
  // Linia 2: Biegun magnetyczny - odchylenie od północy
  display.setCursor(0, 11);
  display.print(F("Mag:"));
  display.print(magDeviation, 1);
  display.print((char)247);
  display.print(getCardinalDirection(headingMag));
  
  // Linia 3: Biegun geograficzny - odchylenie od północy
  display.setCursor(0, 22);
  display.print(F("Geo:"));
  display.print(geoDeviation, 1);
  display.print((char)247);
  display.print(getCardinalDirection(headingTrue));
  
  display.display();
}

// ============================================
// HELPER FUNCTIONS
// ============================================

const char* getCardinalDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5) return "N";
  if (heading >= 22.5 && heading < 67.5) return "NE";
  if (heading >= 67.5 && heading < 112.5) return "E";
  if (heading >= 112.5 && heading < 157.5) return "SE";
  if (heading >= 157.5 && heading < 202.5) return "S";
  if (heading >= 202.5 && heading < 247.5) return "SW";
  if (heading >= 247.5 && heading < 292.5) return "W";
  if (heading >= 292.5 && heading < 337.5) return "NW";
  return "?";
}

// Calculate deviation from North (ideal direction = 0°)
// Returns value from -180 to +180 degrees
// Positive = east of North, Negative = west of North
float getDeviationFromNorth(float heading) {
  float deviation = heading;
  if (deviation > 180) {
    deviation = deviation - 360;
  }
  // Handle exactly 180° as +180° (due South, maximum deviation)
  return deviation;
}

// ============================================
// EEPROM FUNCTIONS
// ============================================

void initializeEEPROM() {
  // Check if EEPROM has valid data
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  if (magic != EEPROM_MAGIC_VALUE) {
    // First run - initialize EEPROM
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint16_t)EEPROM_MAGIC_VALUE);
    EEPROM.put(EEPROM_SHORT_RUN_COUNT_ADDR, (uint8_t)0);
    EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)0);
  }
}

void loadCalibrationFromEEPROM() {
  uint8_t calValid;
  EEPROM.get(EEPROM_CAL_VALID_ADDR, calValid);
  
  if (calValid == 1) {
    EEPROM.get(EEPROM_MAG_OFFSET_X_ADDR, magOffsetX);
    EEPROM.get(EEPROM_MAG_OFFSET_Y_ADDR, magOffsetY);
    EEPROM.get(EEPROM_MAG_OFFSET_Z_ADDR, magOffsetZ);
    EEPROM.get(EEPROM_MAG_SCALE_X_ADDR, magScaleX);
    EEPROM.get(EEPROM_MAG_SCALE_Y_ADDR, magScaleY);
    EEPROM.get(EEPROM_MAG_SCALE_Z_ADDR, magScaleZ);
  }
}

void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_MAG_OFFSET_X_ADDR, magOffsetX);
  EEPROM.put(EEPROM_MAG_OFFSET_Y_ADDR, magOffsetY);
  EEPROM.put(EEPROM_MAG_OFFSET_Z_ADDR, magOffsetZ);
  EEPROM.put(EEPROM_MAG_SCALE_X_ADDR, magScaleX);
  EEPROM.put(EEPROM_MAG_SCALE_Y_ADDR, magScaleY);
  EEPROM.put(EEPROM_MAG_SCALE_Z_ADDR, magScaleZ);
  EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)1);
  
  // Reset short run counter after calibration
  EEPROM.put(EEPROM_SHORT_RUN_COUNT_ADDR, (uint8_t)0);
}

// ============================================
// AUTO-CALIBRATION DETECTION
// ============================================

// Check if we should enter calibration mode
// Returns true if this is the 3rd consecutive short run
bool checkAutoCalibration() {
  uint8_t shortRunCount;
  EEPROM.get(EEPROM_SHORT_RUN_COUNT_ADDR, shortRunCount);
  
  // If we have exactly 3 short runs, trigger calibration
  // (counter was already incremented in setup)
  if (shortRunCount == 3) {
    // Reset counter and trigger calibration
    EEPROM.put(EEPROM_SHORT_RUN_COUNT_ADDR, (uint8_t)0);
    return true;
  }
  
  return false;
}

// Track running time and update short run counter if needed
void trackRunningTime() {
  static bool longRunRecorded = false;
  
  // If we've been running for more than threshold, reset short run counter
  if (!longRunRecorded && (millis() - startupTime >= SHORT_RUN_THRESHOLD)) {
    longRunRecorded = true;
    // This is a long run - reset the short run counter
    EEPROM.put(EEPROM_SHORT_RUN_COUNT_ADDR, (uint8_t)0);
  }
}

void incrementShortRunCounter() {
  uint8_t shortRunCount;
  EEPROM.get(EEPROM_SHORT_RUN_COUNT_ADDR, shortRunCount);
  shortRunCount++;
  EEPROM.put(EEPROM_SHORT_RUN_COUNT_ADDR, shortRunCount);
}

// ============================================
// RESTART MESSAGE
// ============================================

void showRestartMessage() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Calibration"));
  display.println(F("complete!"));
  display.println(F("Please restart."));
  display.display();
}
