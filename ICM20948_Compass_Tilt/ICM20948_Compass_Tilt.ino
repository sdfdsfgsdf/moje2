/**
 * @file ICM20948_Compass_Tilt.ino
 * @brief Professional compass and tilt meter using SparkFun ICM-20948 library
 * 
 * Hardware:
 *   - Arduino Pro Mini (3.3V/8MHz or 5V/16MHz)
 *   - ICM-20948 9-DOF IMU (with AK09916 magnetometer)
 *   - OLED 128x32 I2C display (SSD1306)
 * 
 * Features:
 *   - Tilt measurement on 2 axes (Roll, Pitch) with 0.1° precision
 *   - Magnetic north indication with tilt compensation
 *   - Geographic north indication (corrected for local declination)
 *   - EMA (Exponential Moving Average) filtering for stable readings
 *   - Angular averaging for compass (handles 0°/360° transition)
 *   - Automatic magnetometer calibration (3 short starts trigger)
 *   - EEPROM storage for calibration data with CRC8 integrity check
 *   - AK09916 axis mapping correction
 *   - Magnetometer data validation
 *   - Optional Z-axis calibration requirement for high-tilt accuracy
 * 
 * Location: Żywiec, Poland (49.6853°N, 19.1925°E)
 * Magnetic Declination: 5.5° East (2024)
 * 
 * @author Based on SparkFun ICM-20948 Arduino Library
 * @license MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "ICM_20948.h"

// ============================================================================
// CONFIGURATION SECTION
// ============================================================================

// --- OLED Display Configuration ---
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     32
#define OLED_RESET        -1      // Share Arduino reset pin
#define OLED_I2C_ADDRESS  0x3C

// --- ICM-20948 Configuration ---
#define ICM_AD0_VAL       1       // AD0 pin state (1=0x69, 0=0x68)
#define ICM_I2C_SPEED     400000  // I2C clock speed (400kHz)

// --- Location Configuration (Żywiec, Poland) ---
#define MAGNETIC_DECLINATION  5.5f    // Declination in degrees (East = positive)
#define LATITUDE              49.6853f
#define LONGITUDE             19.1925f

// --- Auto-Calibration Configuration ---
#define SHORT_RUN_THRESHOLD_MS    2000    // Time defining a "short run" (2 seconds)
#define SHORT_RUNS_TO_CALIBRATE   3       // Number of short runs to trigger calibration

// --- Calibration Parameters ---
#define CALIBRATION_MIN_TIME_MS   5000    // Minimum calibration time (5 seconds)
#define CALIBRATION_MAX_TIME_MS   60000   // Maximum calibration time (60 seconds)
#define CALIBRATION_MIN_RANGE_UT  100.0f  // Minimum acceptable range per axis (μT)
#define CALIBRATION_CHECK_MS      500     // Quality check interval during calibration

// --- Filter Configuration ---
#define EMA_ALPHA                 0.10f   // EMA coefficient (0.05-0.15 recommended)
#define DISPLAY_UPDATE_MS         250     // Display refresh interval

// --- Magnetometer Validation ---
#define MAG_VALID_MAX_UT          5000.0f // Maximum valid magnetometer reading (μT)

// --- EEPROM Memory Map ---
#define EEPROM_MAGIC_ADDR         0       // Magic number (2 bytes)
#define EEPROM_SHORT_RUNS_ADDR    4       // Short run counter (1 byte)
#define EEPROM_CAL_VALID_ADDR     8       // Calibration valid flag (1 byte)
#define EEPROM_MAG_OFF_X_ADDR     12      // Mag offset X (4 bytes float)
#define EEPROM_MAG_OFF_Y_ADDR     16      // Mag offset Y (4 bytes float)
#define EEPROM_MAG_OFF_Z_ADDR     20      // Mag offset Z (4 bytes float)
#define EEPROM_MAG_SCL_X_ADDR     24      // Mag scale X (4 bytes float)
#define EEPROM_MAG_SCL_Y_ADDR     28      // Mag scale Y (4 bytes float)
#define EEPROM_MAG_SCL_Z_ADDR     32      // Mag scale Z (4 bytes float)
#define EEPROM_CRC_ADDR           36      // CRC8 checksum (1 byte)
#define EEPROM_MAGIC_VALUE        0xCAFE  // Magic value for EEPROM validation

// --- Calibration Z-axis requirement (for tilted operation) ---
#define CALIBRATION_REQUIRE_Z_AXIS  false // Set to true for better accuracy at high tilt angles

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ICM_20948_I2C    imu;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Structure holding sensor readings and calculated values
 */
struct SensorData {
  // Raw calculated values
  float roll;           // Rotation around X axis (degrees)
  float pitch;          // Rotation around Y axis (degrees)
  float headingMag;     // Magnetic heading (0-360°)
  float headingGeo;     // Geographic heading (0-360°)
  
  // EMA filtered values
  float rollFiltered;
  float pitchFiltered;
  float headingMagFiltered;
  float headingGeoFiltered;
  
  // Filter initialization flag
  bool filtersReady;
};

/**
 * @brief Structure holding magnetometer calibration data
 */
struct MagCalibration {
  // Hard iron offsets
  float offsetX;
  float offsetY;
  float offsetZ;
  
  // Soft iron scale factors
  float scaleX;
  float scaleY;
  float scaleZ;
  
  // Validity flag
  bool isValid;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

SensorData      g_sensor = {0, 0, 0, 0, 0, 0, 0, 0, false};
MagCalibration  g_magCal = {0, 0, 0, 1.0f, 1.0f, 1.0f, false};
unsigned long   g_startupTime = 0;
unsigned long   g_lastDisplayUpdate = 0;
bool            g_calibrationMode = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// Initialization
void initDisplay(void);
bool initIMU(void);
void configureIMU(void);

// EEPROM operations
void initEEPROM(void);
void loadCalibration(void);
void saveCalibration(void);
void incrementShortRunCount(void);
bool shouldTriggerCalibration(void);
void resetShortRunCount(void);
uint8_t calculateCRC8(void);

// Sensor reading and calculations
void readSensors(void);
void calculateTilt(void);
void calculateHeading(void);
bool validateMagData(void);

// Calibration
void runCalibration(void);

// Filtering
float emaFilter(float newVal, float oldVal, float alpha);
float emaFilterAngle(float newAngle, float oldAngle, float alpha);

// Display
void updateDisplay(void);
float getDeviationFromNorth(float heading);
const char* getCardinalDirection(float heading);
void showMessage(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);

// Utility
void trackRuntime(void);

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  g_startupTime = millis();
  
  // Initialize I2C bus
  Wire.begin();
  Wire.setClock(ICM_I2C_SPEED);
  
  // Initialize display
  initDisplay();
  showMessage("Initializing...");
  
  // Initialize IMU
  if (!initIMU()) {
    showMessage("IMU Error!", "Check wiring", "and restart");
    while (true) { delay(100); }
  }
  
  // Configure IMU sensors
  configureIMU();
  
  // Initialize EEPROM and check calibration status
  initEEPROM();
  incrementShortRunCount();
  loadCalibration();
  
  // Check if calibration should be triggered
  if (shouldTriggerCalibration()) {
    g_calibrationMode = true;
    runCalibration();
    showMessage("Calibration", "complete!", "Please restart.");
    while (true) { delay(100); }
  }
  
  // Show startup message
  showMessage("ICM-20948 Ready", "Zywiec, Poland", "Decl: 5.5E");
  delay(2000);
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Track runtime for short run detection
  trackRuntime();
  
  // Read and process sensor data
  if (imu.dataReady()) {
    imu.getAGMT();
    readSensors();
  }
  
  // Update display at fixed interval
  if (millis() - g_lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    g_lastDisplayUpdate = millis();
    updateDisplay();
  }
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Initialize OLED display
 */
void initDisplay(void) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    // Display init failed - halt
    while (true) { delay(100); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

/**
 * @brief Initialize ICM-20948 IMU
 * @return true if initialization successful
 */
bool initIMU(void) {
  // Try primary I2C address
  imu.begin(Wire, ICM_AD0_VAL);
  
  if (imu.status != ICM_20948_Stat_Ok) {
    // Try alternate address
    imu.begin(Wire, !ICM_AD0_VAL);
  }
  
  return (imu.status == ICM_20948_Stat_Ok);
}

/**
 * @brief Configure IMU sensors with optimal settings
 */
void configureIMU(void) {
  // Perform software reset
  imu.swReset();
  delay(250);
  
  // Wake up device
  imu.sleep(false);
  imu.lowPower(false);
  
  // Set sample mode to continuous
  imu.setSampleMode(
    (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
    ICM_20948_Sample_Mode_Continuous
  );
  
  // Configure full scale ranges
  ICM_20948_fss_t fss;
  fss.a = gpm2;     // ±2g for accelerometer (high sensitivity)
  fss.g = dps250;   // ±250°/s for gyroscope
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  // Configure digital low-pass filters
  ICM_20948_dlpcfg_t dlp;
  dlp.a = acc_d50bw4_n68bw8;   // 50Hz 3dB BW for accelerometer
  dlp.g = gyr_d51bw2_n73bw3;   // 51Hz 3dB BW for gyroscope
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
  
  // Enable DLPF
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  // Initialize magnetometer with retries
  bool magOk = false;
  for (int attempt = 0; attempt < 5 && !magOk; attempt++) {
    imu.startupMagnetometer();
    delay(100);
    
    if (imu.dataReady()) {
      imu.getAGMT();
      if (imu.magX() != 0 || imu.magY() != 0 || imu.magZ() != 0) {
        magOk = true;
      }
    }
    delay(50);
  }
  
  if (!magOk) {
    showMessage("Mag warning", "Check sensor");
    delay(2000);
  }
}

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================

/**
 * @brief Calculate CRC8 checksum for calibration data
 * @return CRC8 value
 * 
 * Uses polynomial 0x07 (CRC-8-CCITT)
 */
uint8_t calculateCRC8(void) {
  uint8_t crc = 0x00;
  
  // Calculate CRC over calibration data bytes
  for (uint8_t addr = EEPROM_MAG_OFF_X_ADDR; addr < EEPROM_CRC_ADDR; addr++) {
    uint8_t data = EEPROM.read(addr);
    crc ^= data;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  
  return crc;
}

/**
 * @brief Initialize EEPROM with default values if needed
 */
void initEEPROM(void) {
  uint16_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  if (magic != EEPROM_MAGIC_VALUE) {
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint16_t)EEPROM_MAGIC_VALUE);
    EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
    EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)0);
  }
}

/**
 * @brief Load magnetometer calibration from EEPROM
 * 
 * Validates data integrity using CRC8 checksum before loading
 */
void loadCalibration(void) {
  uint8_t valid;
  EEPROM.get(EEPROM_CAL_VALID_ADDR, valid);
  
  if (valid == 1) {
    // Verify CRC before loading
    uint8_t storedCRC;
    EEPROM.get(EEPROM_CRC_ADDR, storedCRC);
    uint8_t calculatedCRC = calculateCRC8();
    
    if (storedCRC == calculatedCRC) {
      EEPROM.get(EEPROM_MAG_OFF_X_ADDR, g_magCal.offsetX);
      EEPROM.get(EEPROM_MAG_OFF_Y_ADDR, g_magCal.offsetY);
      EEPROM.get(EEPROM_MAG_OFF_Z_ADDR, g_magCal.offsetZ);
      EEPROM.get(EEPROM_MAG_SCL_X_ADDR, g_magCal.scaleX);
      EEPROM.get(EEPROM_MAG_SCL_Y_ADDR, g_magCal.scaleY);
      EEPROM.get(EEPROM_MAG_SCL_Z_ADDR, g_magCal.scaleZ);
      g_magCal.isValid = true;
    } else {
      // CRC mismatch - calibration data corrupted
      showMessage("Cal CRC Error", "Recalibrate!");
      delay(2000);
      EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)0);
    }
  }
}

/**
 * @brief Save magnetometer calibration to EEPROM
 * 
 * Saves calibration data with CRC8 checksum for integrity verification
 */
void saveCalibration(void) {
  EEPROM.put(EEPROM_MAG_OFF_X_ADDR, g_magCal.offsetX);
  EEPROM.put(EEPROM_MAG_OFF_Y_ADDR, g_magCal.offsetY);
  EEPROM.put(EEPROM_MAG_OFF_Z_ADDR, g_magCal.offsetZ);
  EEPROM.put(EEPROM_MAG_SCL_X_ADDR, g_magCal.scaleX);
  EEPROM.put(EEPROM_MAG_SCL_Y_ADDR, g_magCal.scaleY);
  EEPROM.put(EEPROM_MAG_SCL_Z_ADDR, g_magCal.scaleZ);
  
  // Calculate and save CRC8 checksum
  uint8_t crc = calculateCRC8();
  EEPROM.put(EEPROM_CRC_ADDR, crc);
  
  EEPROM.put(EEPROM_CAL_VALID_ADDR, (uint8_t)1);
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
  g_magCal.isValid = true;
}

/**
 * @brief Increment short run counter in EEPROM
 */
void incrementShortRunCount(void) {
  uint8_t count;
  EEPROM.get(EEPROM_SHORT_RUNS_ADDR, count);
  count++;
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, count);
}

/**
 * @brief Check if calibration should be triggered
 * @return true if 3 short runs detected
 */
bool shouldTriggerCalibration(void) {
  uint8_t count;
  EEPROM.get(EEPROM_SHORT_RUNS_ADDR, count);
  
  if (count >= SHORT_RUNS_TO_CALIBRATE) {
    EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
    return true;
  }
  return false;
}

/**
 * @brief Reset short run counter (called when normal operation detected)
 */
void resetShortRunCount(void) {
  EEPROM.put(EEPROM_SHORT_RUNS_ADDR, (uint8_t)0);
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

/**
 * @brief Read sensors and update filtered values
 */
void readSensors(void) {
  calculateTilt();
  
  if (validateMagData()) {
    calculateHeading();
    
    if (!g_sensor.filtersReady) {
      // Initialize filters with first valid readings
      g_sensor.rollFiltered = g_sensor.roll;
      g_sensor.pitchFiltered = g_sensor.pitch;
      g_sensor.headingMagFiltered = g_sensor.headingMag;
      g_sensor.headingGeoFiltered = g_sensor.headingGeo;
      g_sensor.filtersReady = true;
    } else {
      // Apply EMA filters
      g_sensor.rollFiltered = emaFilter(g_sensor.roll, g_sensor.rollFiltered, EMA_ALPHA);
      g_sensor.pitchFiltered = emaFilter(g_sensor.pitch, g_sensor.pitchFiltered, EMA_ALPHA);
      g_sensor.headingMagFiltered = emaFilterAngle(g_sensor.headingMag, g_sensor.headingMagFiltered, EMA_ALPHA);
      g_sensor.headingGeoFiltered = emaFilterAngle(g_sensor.headingGeo, g_sensor.headingGeoFiltered, EMA_ALPHA);
    }
  } else if (g_sensor.filtersReady) {
    // Update tilt even when mag invalid
    g_sensor.rollFiltered = emaFilter(g_sensor.roll, g_sensor.rollFiltered, EMA_ALPHA);
    g_sensor.pitchFiltered = emaFilter(g_sensor.pitch, g_sensor.pitchFiltered, EMA_ALPHA);
  }
}

/**
 * @brief Calculate roll and pitch from accelerometer
 */
void calculateTilt(void) {
  float ax = imu.accX() / 1000.0f;  // Convert mg to g
  float ay = imu.accY() / 1000.0f;
  float az = imu.accZ() / 1000.0f;
  
  // Roll: rotation around X axis
  g_sensor.roll = atan2(ay, az) * 180.0f / PI;
  
  // Pitch: rotation around Y axis
  g_sensor.pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
}

/**
 * @brief Calculate compass heading with tilt compensation
 * 
 * The AK09916 magnetometer has a different axis orientation than the ICM-20948
 * accelerometer/gyroscope. This function applies the necessary transformation:
 *   - Mag X (aligned) = raw Mag Y
 *   - Mag Y (aligned) = -raw Mag X
 *   - Mag Z (aligned) = -raw Mag Z
 */
void calculateHeading(void) {
  // Get raw magnetometer data with axis mapping
  // Transform to match accelerometer coordinate system
  float mx = imu.magY();    // Aligned X = raw Y
  float my = -imu.magX();   // Aligned Y = -raw X
  float mz = -imu.magZ();   // Aligned Z = -raw Z
  
  // Apply hard iron calibration (offset correction)
  mx -= g_magCal.offsetX;
  my -= g_magCal.offsetY;
  mz -= g_magCal.offsetZ;
  
  // Apply soft iron calibration (scale correction)
  mx *= g_magCal.scaleX;
  my *= g_magCal.scaleY;
  mz *= g_magCal.scaleZ;
  
  // Tilt compensation
  float rollRad = g_sensor.roll * PI / 180.0f;
  float pitchRad = g_sensor.pitch * PI / 180.0f;
  
  float cosRoll = cos(rollRad);
  float sinRoll = sin(rollRad);
  float cosPitch = cos(pitchRad);
  float sinPitch = sin(pitchRad);
  
  // Tilt-compensated horizontal components
  float Xh = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
  float Yh = my * cosRoll - mz * sinRoll;
  
  // Calculate magnetic heading
  g_sensor.headingMag = atan2(-Yh, Xh) * 180.0f / PI;
  if (g_sensor.headingMag < 0) {
    g_sensor.headingMag += 360.0f;
  }
  
  // Calculate geographic heading (apply declination)
  g_sensor.headingGeo = g_sensor.headingMag + MAGNETIC_DECLINATION;
  if (g_sensor.headingGeo >= 360.0f) {
    g_sensor.headingGeo -= 360.0f;
  }
  if (g_sensor.headingGeo < 0) {
    g_sensor.headingGeo += 360.0f;
  }
}

/**
 * @brief Validate magnetometer data
 * @return true if data is valid
 */
bool validateMagData(void) {
  float mx = imu.magX();
  float my = imu.magY();
  float mz = imu.magZ();
  
  // Check for all zeros (sensor not ready)
  if (mx == 0 && my == 0 && mz == 0) {
    return false;
  }
  
  // Check for out-of-range values
  if (fabs(mx) > MAG_VALID_MAX_UT || 
      fabs(my) > MAG_VALID_MAX_UT || 
      fabs(mz) > MAG_VALID_MAX_UT) {
    return false;
  }
  
  return true;
}

// ============================================================================
// CALIBRATION FUNCTION
// ============================================================================

/**
 * @brief Run magnetometer calibration routine
 * 
 * Calibration procedure:
 * 1. Collect min/max values for each axis
 * 2. Calculate hard iron offsets (center of ellipsoid)
 * 3. Calculate soft iron scale factors (normalize ellipsoid)
 * 4. Save to EEPROM with CRC8 checksum
 * 
 * Note: If CALIBRATION_REQUIRE_Z_AXIS is true, Z-axis must also reach
 * minimum range for better accuracy at high tilt angles.
 */
void runCalibration(void) {
  showMessage("CALIBRATION", "Rotate all axes", "slowly...");
  delay(1000);
  
  // Initialize min/max tracking
  float magMin[3] = {32767.0f, 32767.0f, 32767.0f};
  float magMax[3] = {-32767.0f, -32767.0f, -32767.0f};
  
  unsigned long startTime = millis();
  unsigned long lastCheck = startTime;
  bool complete = false;
  
  while (!complete && (millis() - startTime < CALIBRATION_MAX_TIME_MS)) {
    if (imu.dataReady()) {
      imu.getAGMT();
      
      // Apply same axis mapping as heading calculation
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
      
      // Periodic quality check
      if (millis() - lastCheck >= CALIBRATION_CHECK_MS) {
        lastCheck = millis();
        
        float rangeX = magMax[0] - magMin[0];
        float rangeY = magMax[1] - magMin[1];
        float rangeZ = magMax[2] - magMin[2];
        unsigned long elapsed = millis() - startTime;
        bool minTimePassed = elapsed >= CALIBRATION_MIN_TIME_MS;
        
        // Check completion (X, Y and optionally Z axes must have sufficient range)
        bool xyComplete = (rangeX >= CALIBRATION_MIN_RANGE_UT) && 
                          (rangeY >= CALIBRATION_MIN_RANGE_UT);
        bool zComplete = !CALIBRATION_REQUIRE_Z_AXIS || 
                         (rangeZ >= CALIBRATION_MIN_RANGE_UT);
        
        if (xyComplete && zComplete && minTimePassed) {
          complete = true;
        }
        
        // Update display
        display.clearDisplay();
        display.setCursor(0, 0);
        
        if (complete) {
          display.println(F("CAL OK!"));
        } else {
          display.print(F("CAL: "));
          if (rangeX < CALIBRATION_MIN_RANGE_UT) display.print(F("X"));
          if (rangeY < CALIBRATION_MIN_RANGE_UT) display.print(F("Y"));
          if (CALIBRATION_REQUIRE_Z_AXIS && rangeZ < CALIBRATION_MIN_RANGE_UT) display.print(F("Z"));
          if (!minTimePassed) {
            display.print(F(" "));
            display.print((CALIBRATION_MIN_TIME_MS - elapsed) / 1000);
            display.print(F("s"));
          }
        }
        
        display.setCursor(0, 11);
        display.print(F("X:"));
        display.print((int)rangeX);
        display.print(F(" Y:"));
        display.print((int)rangeY);
        
        display.setCursor(0, 22);
        if (CALIBRATION_REQUIRE_Z_AXIS) {
          display.print(F("Z:"));
          display.print((int)rangeZ);
          display.print(F(" min:"));
        } else {
          display.print(F("min:"));
        }
        display.print((int)CALIBRATION_MIN_RANGE_UT);
        
        display.display();
      }
    }
    delay(10);
  }
  
  // Calculate hard iron offsets
  g_magCal.offsetX = (magMax[0] + magMin[0]) / 2.0f;
  g_magCal.offsetY = (magMax[1] + magMin[1]) / 2.0f;
  g_magCal.offsetZ = (magMax[2] + magMin[2]) / 2.0f;
  
  // Calculate soft iron scale factors
  float deltaX = magMax[0] - magMin[0];
  float deltaY = magMax[1] - magMin[1];
  float deltaZ = magMax[2] - magMin[2];
  
  // Prevent division by zero
  const float MIN_DELTA = 1.0f;
  if (fabs(deltaX) < MIN_DELTA) deltaX = MIN_DELTA;
  if (fabs(deltaY) < MIN_DELTA) deltaY = MIN_DELTA;
  if (fabs(deltaZ) < MIN_DELTA) deltaZ = MIN_DELTA;
  
  float avgDelta = (deltaX + deltaY + deltaZ) / 3.0f;
  
  g_magCal.scaleX = avgDelta / deltaX;
  g_magCal.scaleY = avgDelta / deltaY;
  g_magCal.scaleZ = avgDelta / deltaZ;
  
  // Save to EEPROM
  saveCalibration();
  
  // Show result
  if (complete) {
    showMessage("Cal Done!", "Data saved.");
  } else {
    showMessage("Cal Timeout", "Data saved.");
  }
  delay(2000);
}

// ============================================================================
// FILTER FUNCTIONS
// ============================================================================

/**
 * @brief Apply EMA filter to linear value
 */
float emaFilter(float newVal, float oldVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

/**
 * @brief Apply EMA filter to angular value (handles 0°/360° wraparound)
 * 
 * Uses vector-based averaging to prevent jumps at North (0°/360°)
 */
float emaFilterAngle(float newAngle, float oldAngle, float alpha) {
  float newRad = newAngle * PI / 180.0f;
  float oldRad = oldAngle * PI / 180.0f;
  
  // Convert to unit vectors
  float newX = cos(newRad);
  float newY = sin(newRad);
  float oldX = cos(oldRad);
  float oldY = sin(oldRad);
  
  // Apply EMA to components
  float filtX = alpha * newX + (1.0f - alpha) * oldX;
  float filtY = alpha * newY + (1.0f - alpha) * oldY;
  
  // Convert back to angle
  float result = atan2(filtY, filtX) * 180.0f / PI;
  
  // Normalize to 0-360
  if (result < 0) result += 360.0f;
  
  return result;
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Update OLED display with current readings
 */
void updateDisplay(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: Tilt angles (Roll, Pitch)
  display.setCursor(0, 0);
  display.print(F("X:"));
  display.print(g_sensor.rollFiltered, 1);
  display.print(F(" Y:"));
  display.print(g_sensor.pitchFiltered, 1);
  
  // Calculate deviations from North
  float magDev = getDeviationFromNorth(g_sensor.headingMagFiltered);
  float geoDev = getDeviationFromNorth(g_sensor.headingGeoFiltered);
  
  // Line 2: Magnetic North deviation
  display.setCursor(0, 11);
  display.print(F("Mag:"));
  display.print(magDev, 1);
  display.print((char)247);  // Degree symbol
  display.print(getCardinalDirection(g_sensor.headingMagFiltered));
  
  // Line 3: Geographic North deviation
  display.setCursor(0, 22);
  display.print(F("Geo:"));
  display.print(geoDev, 1);
  display.print((char)247);
  display.print(getCardinalDirection(g_sensor.headingGeoFiltered));
  
  display.display();
}

/**
 * @brief Calculate deviation from North (-180° to +180°)
 * @param heading Current heading (0-360°)
 * @return Deviation (negative = west, positive = east)
 */
float getDeviationFromNorth(float heading) {
  float deviation = heading;
  if (deviation > 180.0f) {
    deviation -= 360.0f;
  }
  return deviation;
}

/**
 * @brief Get cardinal direction string
 * @param heading Heading in degrees (0-360)
 * @return Cardinal direction string (N, NE, E, SE, S, SW, W, NW)
 */
const char* getCardinalDirection(float heading) {
  if (heading >= 337.5f || heading < 22.5f)  return "N";
  if (heading >= 22.5f  && heading < 67.5f)  return "NE";
  if (heading >= 67.5f  && heading < 112.5f) return "E";
  if (heading >= 112.5f && heading < 157.5f) return "SE";
  if (heading >= 157.5f && heading < 202.5f) return "S";
  if (heading >= 202.5f && heading < 247.5f) return "SW";
  if (heading >= 247.5f && heading < 292.5f) return "W";
  if (heading >= 292.5f && heading < 337.5f) return "NW";
  return "?";
}

/**
 * @brief Show message on display (1-3 lines)
 */
void showMessage(const char* line1, const char* line2, const char* line3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(line1);
  
  if (line2) {
    display.setCursor(0, 11);
    display.println(line2);
  }
  
  if (line3) {
    display.setCursor(0, 22);
    display.println(line3);
  }
  
  display.display();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Track runtime and reset short run counter if normal operation
 */
void trackRuntime(void) {
  static bool normalOpRecorded = false;
  
  if (!normalOpRecorded && (millis() - g_startupTime >= SHORT_RUN_THRESHOLD_MS)) {
    normalOpRecorded = true;
    resetShortRunCount();
  }
}
