/**
 * @file ICM20948_Compass_Tilt_ESP32.ino
 * @brief Advanced Compass and Inclinometer with Mahony AHRS for ESP32-WROOM-32D
 * 
 * Features:
 * - Full on-device calibration with ellipsoid fitting (Li's algorithm)
 * - Button-driven calibration with OLED guidance
 * - Optimized for ESP32's FPU and dual-core capabilities
 * - NVS (Preferences) for calibration storage
 * - Hard Iron correction (bias vector B)
 * - Soft Iron correction (3x3 transformation matrix A_inv)
 * - Mahony AHRS filter with gyroscope integration
 * 
 * Hardware: ESP32-WROOM-32D, ICM-20948 IMU, OLED 128x32
 * Based on: jremington/ICM_20948-AHRS, Cave Pearl Project, Pololu magnetometer correction
 * Location: Zywiec, Poland (49.6853N, 19.1925E), Declination: 5.5E
 * 
 * Calibration Method:
 * This implementation uses ellipsoid fitting based on Li's algorithm as described in:
 * - https://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
 * - https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
 * - https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
 * - https://iopscience.iop.org/article/10.1088/1755-1315/237/3/032015/pdf
 * 
 * The ellipsoid fitting method provides superior calibration compared to simple min/max
 * by properly handling both hard iron (offset) and soft iron (scale/rotation) distortions.
 * 
 * References:
 * - https://github.com/jremington/ICM_20948-AHRS
 * - https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315
 * - Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific fitting"
 * 
 * @license MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "ICM_20948.h"
#include <esp_task_wdt.h>

// ============================================================================
// ESP32-WROOM-32D PIN CONFIGURATION
// ============================================================================

// I2C Pins (default ESP32)
#define I2C_SDA           21
#define I2C_SCL           22

// Calibration Button (active LOW with internal pullup)
#define BUTTON_PIN        15

// Optional: LED indicator for calibration status
#define LED_PIN           2    // Built-in LED on most ESP32 boards

// ============================================================================
// DISPLAY CONFIGURATION
// ============================================================================

#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     32   // Changed to 128x32 OLED
#define OLED_RESET        -1
#define OLED_I2C_ADDRESS  0x3C
#define CHAR_WIDTH        6    // Width of character in pixels at text size 1
#define MAX_CHARS_LINE    (SCREEN_WIDTH / CHAR_WIDTH)  // 21 chars per line

// ============================================================================
// ICM-20948 CONFIGURATION
// ============================================================================

#define ICM_AD0_VAL       1    // AD0 pin state (0 = GND = 0x68, 1 = VCC = 0x69)
#define ICM_I2C_SPEED     400000

// ============================================================================
// I2C STABILITY CONFIGURATION
// ============================================================================

#define I2C_RETRY_COUNT       3       // Number of retries for I2C operations
#define I2C_RETRY_DELAY_MS    5       // Delay between retries (ms)
#define I2C_TIMEOUT_MS        50      // Timeout for I2C operations (ms)
#define I2C_BUS_RECOVERY_CLOCKS 16    // Clock pulses for bus recovery

// ============================================================================
// WATCHDOG CONFIGURATION
// ============================================================================

#define WDT_TIMEOUT_SECONDS   10      // Watchdog timeout in seconds

// ============================================================================
// LOCATION DATA (Żywiec, Poland)
// ============================================================================

#define MAGNETIC_DECLINATION  5.5f    // Declination in degrees (E = positive)
#define LATITUDE              49.6853f
#define LONGITUDE             19.1925f

// ============================================================================
// MAHONY AHRS FILTER PARAMETERS (Optimized for ESP32)
// ============================================================================

#define MAHONY_KP             30.0f   // Proportional gain (lower for more stable)
#define MAHONY_KI             0.01f   // Integral gain (small for drift correction)

// ============================================================================
// GYROSCOPE CONFIGURATION
// ============================================================================

// Sensitivity for 250 DPS range: 131 LSB/(deg/s) = 0.00763 deg/LSB
#define GYRO_SCALE            ((M_PI / 180.0f) * 0.00763f)

// ============================================================================
// CALIBRATION PARAMETERS
// ============================================================================

// Gyroscope calibration
#define GYRO_CAL_SAMPLES      1000    // More samples for ESP32 (faster)

// Magnetometer calibration  
#define MAG_CAL_MIN_TIME_MS   10000   // Minimum calibration time (10s)
#define MAG_CAL_MAX_TIME_MS   120000  // Maximum calibration time (2 min)
#define MAG_CAL_MIN_RANGE     80.0f   // Minimum range for X/Y axes (μT)
#define MAG_CAL_MIN_SAMPLES   300     // Minimum samples for good calibration
#define MAG_UPDATE_INTERVAL   50      // ms between display updates

// Accelerometer calibration
#define ACCEL_CAL_POSITIONS   6       // 6 positions for full calibration
#define ACCEL_CAL_SAMPLES     200     // Samples per position

// ============================================================================
// FILTER PARAMETERS
// ============================================================================

#define EMA_ALPHA             0.15f   // Higher for faster response on ESP32
#define DISPLAY_UPDATE_MS     100     // Faster display updates on ESP32

// ============================================================================
// VALIDATION PARAMETERS
// ============================================================================

#define MAG_VALID_MIN         5.0f    // Minimum valid mag reading
#define MAG_VALID_MAX         4800.0f // Maximum valid mag reading (Earth's field ~25-65 μT)

// ============================================================================
// NUMERICAL CONSTANTS FOR MATRIX OPERATIONS
// ============================================================================

#define MATRIX_EPSILON        1e-10f  // Threshold for singularity detection
#define CHOLESKY_MIN_DIAG     0.001f  // Minimum diagonal value for Cholesky decomposition
#define ELLIPSOID_FIT_MAX_RESIDUAL 50.0f  // Maximum residual (%) for successful fit

// ============================================================================
// BUTTON DEBOUNCE
// ============================================================================

#define DEBOUNCE_MS           50
#define LONG_PRESS_MS         2000    // Long press to enter calibration

// ============================================================================
// NVS NAMESPACE
// ============================================================================

#define NVS_NAMESPACE         "compass_cal"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ICM_20948_I2C    imu;
Preferences      preferences;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Sensor readings and filtered values
 */
struct SensorData {
  float roll, pitch, yaw;
  float headingMag, headingGeo;
  float rollFiltered, pitchFiltered;
  float headingMagFiltered, headingGeoFiltered;
  bool filtersReady;
};

/**
 * @brief Calibration data structure
 * 
 * Improved calibration using ellipsoid fitting method:
 * - Hard Iron: Bias vector B (offset to subtract from raw readings)
 * - Soft Iron: 3x3 transformation matrix A_inv (corrects for axis scaling and non-orthogonality)
 * 
 * Based on:
 * - jremington/ICM_20948-AHRS calibration format
 * - Sailboat Instruments ellipsoid fitting (Li's algorithm)
 * - Cave Pearl Project min/max method (as fallback)
 * - IOP Science paper: DOI 10.1088/1755-1315/237/3/032015
 * 
 * Calibrated reading = A_inv * (raw - B)
 */
struct Calibration {
  // Gyroscope offsets (raw units)
  float gyroOffset[3];
  
  // Magnetometer Hard Iron correction (bias vector B)
  // This is subtracted from raw readings
  float magBias[3];
  
  // Magnetometer Soft Iron correction (3x3 transformation matrix A_inv)
  // This corrects for axis scaling, non-orthogonality, and ellipsoid distortion
  // Format: magAinv[row][col], applied as matrix multiplication
  float magAinv[3][3];
  
  // Min/Max values for calculating offset and scale (used for simple calibration fallback)
  float magMin[3];
  float magMax[3];
  
  // Accelerometer Hard Iron correction (bias vector)
  float accelBias[3];
  
  // Accelerometer Soft Iron correction (3x3 transformation matrix)
  float accelAinv[3][3];
  
  // Validation flag
  bool isValid;
  
  // Calibration quality indicator (0-100)
  uint8_t quality;
  
  // Flag to indicate if ellipsoid fitting was used (vs simple min/max)
  bool useEllipsoidFit;
};

/**
 * @brief AHRS state with quaternion and integral error
 */
struct AHRSState {
  float q[4];           // Quaternion [w, x, y, z]
  float eInt[3];        // Integral error
  unsigned long lastUpdate;
};

/**
 * @brief Button state for debouncing
 */
struct ButtonState {
  bool lastState;
  bool currentState;
  unsigned long lastDebounceTime;
  unsigned long pressStartTime;
  bool isPressed;
  bool longPressDetected;
};

// ============================================================================
// CALIBRATION SAMPLE BUFFER
// ============================================================================

// For ellipsoid fitting (ESP32 has enough RAM)
// Increased to 600 samples for better ellipsoid fitting accuracy
// For ellipsoid fitting (ESP32 has ~320KB available RAM)
// Memory usage: 500 samples * 3 floats * 4 bytes = 6KB
// This provides good calibration accuracy while leaving ample RAM for other operations
#define MAX_CAL_SAMPLES 500
float g_magSamples[MAX_CAL_SAMPLES][3];
int g_sampleCount = 0;

// Normalization factor for calibrated magnetometer readings
// Set to match jremington/ICM_20948-AHRS format
#define MAG_FIELD_NORM 1000.0f

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

SensorData    g_sensor = {0};
Calibration   g_cal = {0};
AHRSState     g_ahrs = {{1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0}, 0};
ButtonState   g_button = {HIGH, HIGH, 0, 0, false, false};

unsigned long g_lastDisplayUpdate = 0;
bool          g_calibrationMode = false;
volatile bool g_i2cError = false;        // Flag for I2C errors
unsigned long g_lastI2cSuccess = 0;      // Last successful I2C operation timestamp
uint32_t      g_i2cErrorCount = 0;       // Total I2C error count

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// Initialization
bool initIMU(void);
void configureIMU(void);
void initButton(void);

// Calibration storage
void loadCalibration(void);
void saveCalibration(void);
void clearCalibration(void);

// Calibration routines
void runFullCalibration(void);
void runGyroCalibration(void);
void runMagCalibration(void);
void calculateMagCalibration(void);
void calculateMagCalibrationEllipsoid(void);
void calculateMagCalibrationMinMax(void);
void applyMagCalibration(float raw[3], float calibrated[3]);
void initDefaultCalibration(void);

// Ellipsoid fitting (Li's algorithm)
// Based on: https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
// and jremington/ICM_20948-AHRS calibrate3.py
bool ellipsoidFit(float samples[][3], int n, float M[3][3], float bias[3], float* residual);
void matrix3x3Inverse(float M[3][3], float Minv[3][3]);
void matrix3x3Multiply(float A[3][3], float B[3][3], float C[3][3]);
void matrix3x3Sqrt(float M[3][3], float sqrtM[3][3]);
float matrix3x3Determinant(float M[3][3]);
void choleskyDecomposition(float A[3][3], float L[3][3]);

// Sensor processing
void processSensors(void);
bool validateMagData(float mx, float my, float mz);

// Math utilities
float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);
void vector_cross(float a[3], float b[3], float result[3]);

// AHRS
void MahonyQuaternionUpdate(float ax, float ay, float az, 
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float deltat);
void quaternionToEuler(float q[4], float& yaw, float& pitch, float& roll);

// Filters
float emaFilter(float newVal, float oldVal, float alpha);
float emaFilterAngle(float newAngle, float oldAngle, float alpha);

// Display
void updateDisplay(void);
void showCalibrationScreen(const char* title, const char* line1, 
                           const char* line2 = nullptr, const char* line3 = nullptr,
                           int progress = -1);
void showMsg(const char* l1, const char* l2 = nullptr, 
             const char* l3 = nullptr);
float getDeviationFromNorth(float heading);
char getCardinalChar(float heading);

// Button handling
void updateButton(void);
bool wasButtonPressed(void);
bool wasLongPress(void);

// I2C stability
void initWatchdog(void);
void feedWatchdog(void);
bool recoverI2cBus(void);
bool safeDisplayUpdate(void);
bool checkI2cDevices(void);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F("\n=== ICM20948 Compass ESP32 ==="));
  Serial.println(F("Initializing..."));
  
  // Initialize I2C with ESP32 pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(ICM_I2C_SPEED);
  // ESP32 Arduino Wire uses setTimeOut (capital O), different from standard Arduino setTimeout
  Wire.setTimeOut(I2C_TIMEOUT_MS);  // Set I2C timeout in milliseconds
  
  // Initialize button
  initButton();
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize watchdog timer for stability
  initWatchdog();
  
  // Check I2C bus and recover if needed
  if (!checkI2cDevices()) {
    Serial.println(F("I2C devices not found, attempting bus recovery..."));
    recoverI2cBus();
    delay(100);
    if (!checkI2cDevices()) {
      Serial.println(F("I2C bus recovery failed!"));
    }
  }
  
  // Initialize display with retry
  bool displayOk = false;
  for (int retry = 0; retry < I2C_RETRY_COUNT && !displayOk; retry++) {
    feedWatchdog();
    displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
    if (!displayOk) {
      Serial.printf("Display init attempt %d failed\n", retry + 1);
      delay(100);
    }
  }
  
  if (!displayOk) {
    Serial.println(F("ERROR: SSD1306 allocation failed after retries"));
    // Continue without display - don't hang
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    safeDisplayUpdate();
    showMsg("ICM20948 Compass", "ESP32", "Init...");
  }
  delay(500);
  
  feedWatchdog();
  
  // Initialize IMU with retry
  bool imuOk = false;
  for (int retry = 0; retry < I2C_RETRY_COUNT && !imuOk; retry++) {
    feedWatchdog();
    imuOk = initIMU();
    if (!imuOk) {
      Serial.printf("IMU init attempt %d failed\n", retry + 1);
      recoverI2cBus();
      delay(200);
    }
  }
  
  if (!imuOk) {
    showMsg("ERROR!", "IMU not found", "SDA:21 SCL:22");
    Serial.println(F("ERROR: IMU initialization failed!"));
    // Blink LED but don't hang forever - watchdog will reset if needed
    unsigned long errorStart = millis();
    while (millis() - errorStart < 30000) {  // 30s timeout instead of infinite
      feedWatchdog();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
    ESP.restart();  // Restart after timeout
  }
  
  feedWatchdog();
  configureIMU();
  Serial.println(F("IMU initialized successfully"));
  
  // Load saved calibration
  loadCalibration();
  
  if (g_cal.isValid) {
    showMsg("Cal loaded", "from memory", "Hold BTN=recal");
    Serial.println(F("Calibration loaded from NVS"));
  } else {
    showMsg("No calibration", "Hold BTN 2s", "to start cal");
    Serial.println(F("No valid calibration found"));
  }
  delay(2000);
  feedWatchdog();
  
  // Quick gyro calibration at startup
  if (!g_cal.isValid) {
    showMsg("Quick gyro", "calibration", "Hold still...");
    delay(1000);
    runGyroCalibration();
  }
  
  // Initialize AHRS timing
  g_ahrs.lastUpdate = micros();
  g_lastI2cSuccess = millis();
  
  showMsg("Ready!", "Hold BTN 2s", "for full cal");
  delay(1500);
  feedWatchdog();
  
  Serial.println(F("System ready!"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Feed watchdog to prevent reset
  feedWatchdog();
  
  // Update button state
  updateButton();
  
  // Check for long press to enter calibration
  if (wasLongPress()) {
    Serial.println(F("Long press detected - entering calibration mode"));
    runFullCalibration();
    g_ahrs.lastUpdate = micros();  // Reset timing after calibration
    g_sensor.filtersReady = false;  // Reset filters
  }
  
  // Check for I2C bus health - recover if no successful communication for too long
  if (millis() - g_lastI2cSuccess > 5000) {
    Serial.println(F("I2C timeout - attempting bus recovery"));
    g_i2cError = true;
    if (recoverI2cBus()) {
      g_lastI2cSuccess = millis();
      g_i2cError = false;
      Serial.println(F("I2C bus recovered"));
    } else {
      Serial.println(F("I2C recovery failed - restarting..."));
      delay(100);
      ESP.restart();
    }
  }
  
  // Normal operation - read sensors and update display
  if (imu.dataReady()) {
    imu.getAGMT();
    if (imu.status == ICM_20948_Stat_Ok) {
      g_lastI2cSuccess = millis();
      g_i2cError = false;
      processSensors();
    } else {
      g_i2cErrorCount++;
      if (g_i2cErrorCount % 100 == 0) {
        Serial.printf("I2C errors: %d\n", g_i2cErrorCount);
      }
    }
  }
  
  // Update display at fixed interval
  if (millis() - g_lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    g_lastDisplayUpdate = millis();
    if (!safeDisplayUpdate()) {
      // Display update failed - try recovery
      recoverI2cBus();
    }
  }
  
  // Small delay to prevent tight loop and allow other tasks
  delay(1);
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

/**
 * @brief Initialize and configure the watchdog timer
 * Provides automatic reset if the program hangs
 */
void initWatchdog(void) {
  // Initialize Task Watchdog Timer (TWDT)
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);  // true = panic on timeout (reset)
  esp_task_wdt_add(NULL);  // Add current task to WDT
  Serial.printf("Watchdog initialized with %ds timeout\n", WDT_TIMEOUT_SECONDS);
}

/**
 * @brief Feed the watchdog to prevent reset
 * Must be called regularly in the main loop and during long operations
 */
void feedWatchdog(void) {
  esp_task_wdt_reset();
}

/**
 * @brief Attempt to recover the I2C bus from a stuck state
 * 
 * This function generates clock pulses on SCL while SDA is high to
 * clear any stuck slave device. This is a common I2C bus recovery technique.
 * 
 * @return true if recovery was successful (devices responding)
 */
bool recoverI2cBus(void) {
  Serial.println(F("Attempting I2C bus recovery..."));
  
  // End current I2C session
  Wire.end();
  delay(10);
  
  // Configure pins as GPIO for manual control
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);
  
  // Ensure SDA is high
  digitalWrite(I2C_SDA, HIGH);
  
  // Generate clock pulses to release any stuck slave
  for (int i = 0; i < I2C_BUS_RECOVERY_CLOCKS; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }
  
  // Generate STOP condition (SDA low-to-high while SCL is high)
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH);
  delayMicroseconds(5);
  
  // Reinitialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(ICM_I2C_SPEED);
  // ESP32 Arduino Wire uses setTimeOut (capital O)
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  
  delay(50);
  
  // Verify bus is working by scanning for devices
  return checkI2cDevices();
}

/**
 * @brief Check if I2C devices are responding
 * Scans for OLED and IMU devices on the bus
 * 
 * @return true if at least one expected device responds
 */
bool checkI2cDevices(void) {
  bool oledFound = false;
  bool imuFound = false;
  
  // Check OLED (0x3C or 0x3D)
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  if (Wire.endTransmission() == 0) {
    oledFound = true;
  }
  
  // Check IMU at 0x68 or 0x69
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    imuFound = true;
  }
  Wire.beginTransmission(0x69);
  if (Wire.endTransmission() == 0) {
    imuFound = true;
  }
  
  Serial.printf("I2C scan: OLED=%s, IMU=%s\n", 
                oledFound ? "OK" : "NOT FOUND",
                imuFound ? "OK" : "NOT FOUND");
  
  return (oledFound || imuFound);
}

/**
 * @brief Safely update the display with error handling
 * 
 * @return true if display update was successful
 */
bool safeDisplayUpdate(void) {
  // Update display content first
  updateDisplay();
  
  // The display.display() function doesn't return error status in Adafruit library
  // but we can check if the I2C transaction was successful
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    g_lastI2cSuccess = millis();
    return true;
  } else {
    g_i2cErrorCount++;
    if (g_i2cErrorCount % 10 == 0) {
      Serial.printf("Display I2C error: %d (count: %d)\n", error, g_i2cErrorCount);
    }
    return false;
  }
}

bool initIMU(void) {
  // Try primary address first
  imu.begin(Wire, ICM_AD0_VAL);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(F("IMU found at address 0x"));
    Serial.println(ICM_AD0_VAL ? 0x69 : 0x68, HEX);
    return true;
  }
  
  // Try alternate address
  imu.begin(Wire, !ICM_AD0_VAL);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(F("IMU found at address 0x"));
    Serial.println(!ICM_AD0_VAL ? 0x69 : 0x68, HEX);
    return true;
  }
  
  return false;
}

void configureIMU(void) {
  // Software reset
  imu.swReset();
  delay(250);
  
  // Wake up and configure
  imu.sleep(false);
  imu.lowPower(false);
  
  // Set sample mode to continuous
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                    ICM_20948_Sample_Mode_Continuous);
  
  // Configure full scale ranges
  ICM_20948_fss_t fss;
  fss.a = gpm2;     // ±2g for best resolution
  fss.g = dps250;   // ±250 dps for best resolution
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  // Configure digital low-pass filters
  ICM_20948_dlpcfg_t dlp;
  dlp.a = acc_d50bw4_n68bw8;   // 50Hz bandwidth
  dlp.g = gyr_d51bw2_n73bw3;   // 51Hz bandwidth
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
  
  // Enable DLPF
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  // Start magnetometer
  for (int attempt = 0; attempt < 5; attempt++) {
    imu.startupMagnetometer();
    delay(100);
    if (imu.dataReady()) {
      imu.getAGMT();
      if (imu.magX() != 0 || imu.magY() != 0 || imu.magZ() != 0) {
        Serial.println(F("Magnetometer initialized"));
        break;
      }
    }
  }
}

void initButton(void) {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  g_button.lastState = digitalRead(BUTTON_PIN);
  g_button.currentState = g_button.lastState;
}

// ============================================================================
// CALIBRATION STORAGE (NVS/Preferences)
// ============================================================================

void loadCalibration(void) {
  preferences.begin(NVS_NAMESPACE, true);  // Read-only mode
  
  // Check if calibration exists
  if (!preferences.isKey("valid")) {
    preferences.end();
    Serial.println(F("No calibration data in NVS"));
    initDefaultCalibration();
    return;
  }
  
  bool valid = preferences.getBool("valid", false);
  if (!valid) {
    preferences.end();
    initDefaultCalibration();
    return;
  }
  
  // Load gyro offsets
  g_cal.gyroOffset[0] = preferences.getFloat("gx", 0.0f);
  g_cal.gyroOffset[1] = preferences.getFloat("gy", 0.0f);
  g_cal.gyroOffset[2] = preferences.getFloat("gz", 0.0f);
  
  // Load mag bias (hard iron correction) - compatible with jremington format
  g_cal.magBias[0] = preferences.getFloat("m_b0", 0.0f);
  g_cal.magBias[1] = preferences.getFloat("m_b1", 0.0f);
  g_cal.magBias[2] = preferences.getFloat("m_b2", 0.0f);
  
  // Load mag A_inv matrix (soft iron correction) - 3x3 matrix
  // Format matches jremington/ICM_20948-AHRS
  g_cal.magAinv[0][0] = preferences.getFloat("m_a00", 1.0f);
  g_cal.magAinv[0][1] = preferences.getFloat("m_a01", 0.0f);
  g_cal.magAinv[0][2] = preferences.getFloat("m_a02", 0.0f);
  g_cal.magAinv[1][0] = preferences.getFloat("m_a10", 0.0f);
  g_cal.magAinv[1][1] = preferences.getFloat("m_a11", 1.0f);
  g_cal.magAinv[1][2] = preferences.getFloat("m_a12", 0.0f);
  g_cal.magAinv[2][0] = preferences.getFloat("m_a20", 0.0f);
  g_cal.magAinv[2][1] = preferences.getFloat("m_a21", 0.0f);
  g_cal.magAinv[2][2] = preferences.getFloat("m_a22", 1.0f);
  
  // Load min/max for reference (used for quality calculation)
  g_cal.magMin[0] = preferences.getFloat("mx_min", -200.0f);
  g_cal.magMin[1] = preferences.getFloat("my_min", -200.0f);
  g_cal.magMin[2] = preferences.getFloat("mz_min", -200.0f);
  
  g_cal.magMax[0] = preferences.getFloat("mx_max", 200.0f);
  g_cal.magMax[1] = preferences.getFloat("my_max", 200.0f);
  g_cal.magMax[2] = preferences.getFloat("mz_max", 200.0f);
  
  g_cal.quality = preferences.getUChar("quality", 0);
  g_cal.useEllipsoidFit = preferences.getBool("ellipsoid", false);
  g_cal.isValid = true;
  
  preferences.end();
  
  Serial.println(F("Calibration loaded:"));
  Serial.printf("  Gyro offset: [%.2f, %.2f, %.2f]\n", 
                g_cal.gyroOffset[0], g_cal.gyroOffset[1], g_cal.gyroOffset[2]);
  Serial.printf("  Mag bias (B): [%.2f, %.2f, %.2f]\n",
                g_cal.magBias[0], g_cal.magBias[1], g_cal.magBias[2]);
  Serial.println(F("  Mag A_inv matrix:"));
  for (int i = 0; i < 3; i++) {
    Serial.printf("    [%.5f, %.5f, %.5f]\n",
                  g_cal.magAinv[i][0], g_cal.magAinv[i][1], g_cal.magAinv[i][2]);
  }
  Serial.printf("  Quality: %d%%, Ellipsoid: %s\n", g_cal.quality, 
                g_cal.useEllipsoidFit ? "yes" : "no");
}

void saveCalibration(void) {
  preferences.begin(NVS_NAMESPACE, false);  // Read-write mode
  
  // Save gyro offsets
  preferences.putFloat("gx", g_cal.gyroOffset[0]);
  preferences.putFloat("gy", g_cal.gyroOffset[1]);
  preferences.putFloat("gz", g_cal.gyroOffset[2]);
  
  // Save mag bias (hard iron) - format compatible with jremington
  preferences.putFloat("m_b0", g_cal.magBias[0]);
  preferences.putFloat("m_b1", g_cal.magBias[1]);
  preferences.putFloat("m_b2", g_cal.magBias[2]);
  
  // Save mag A_inv matrix (soft iron) - 3x3 matrix
  preferences.putFloat("m_a00", g_cal.magAinv[0][0]);
  preferences.putFloat("m_a01", g_cal.magAinv[0][1]);
  preferences.putFloat("m_a02", g_cal.magAinv[0][2]);
  preferences.putFloat("m_a10", g_cal.magAinv[1][0]);
  preferences.putFloat("m_a11", g_cal.magAinv[1][1]);
  preferences.putFloat("m_a12", g_cal.magAinv[1][2]);
  preferences.putFloat("m_a20", g_cal.magAinv[2][0]);
  preferences.putFloat("m_a21", g_cal.magAinv[2][1]);
  preferences.putFloat("m_a22", g_cal.magAinv[2][2]);
  
  // Save min/max for reference
  preferences.putFloat("mx_min", g_cal.magMin[0]);
  preferences.putFloat("my_min", g_cal.magMin[1]);
  preferences.putFloat("mz_min", g_cal.magMin[2]);
  
  preferences.putFloat("mx_max", g_cal.magMax[0]);
  preferences.putFloat("my_max", g_cal.magMax[1]);
  preferences.putFloat("mz_max", g_cal.magMax[2]);
  
  preferences.putUChar("quality", g_cal.quality);
  preferences.putBool("ellipsoid", g_cal.useEllipsoidFit);
  preferences.putBool("valid", true);
  
  preferences.end();
  
  g_cal.isValid = true;
  Serial.println(F("Calibration saved to NVS"));
  
  // Print calibration in jremington/ICM_20948-AHRS compatible format
  Serial.println(F("\n// Calibration values (jremington compatible format):"));
  Serial.printf("float M_B[3] = {%.2f, %.2f, %.2f};\n",
                g_cal.magBias[0], g_cal.magBias[1], g_cal.magBias[2]);
  Serial.println(F("float M_Ainv[3][3] = {"));
  for (int i = 0; i < 3; i++) {
    Serial.printf("  {%.5f, %.5f, %.5f}%s\n",
                  g_cal.magAinv[i][0], g_cal.magAinv[i][1], g_cal.magAinv[i][2],
                  i < 2 ? "," : "");
  }
  Serial.println(F("};"));
}

void clearCalibration(void) {
  preferences.begin(NVS_NAMESPACE, false);
  preferences.clear();
  preferences.end();
  
  g_cal.isValid = false;
  Serial.println(F("Calibration cleared"));
}

// ============================================================================
// CALIBRATION ROUTINES
// ============================================================================

/**
 * @brief Full calibration routine with OLED guidance
 */
void runFullCalibration(void) {
  g_calibrationMode = true;
  digitalWrite(LED_PIN, HIGH);
  
  // Welcome screen
  showMsg("=CALIBRATION=", "Press BTN", "to start");
  Serial.println(F("\n=== Starting Full Calibration ==="));
  
  // Wait for button press or timeout
  unsigned long startWait = millis();
  while (!wasButtonPressed() && (millis() - startWait < 10000)) {
    feedWatchdog();
    updateButton();
    delay(10);
  }
  
  feedWatchdog();
  
  // Step 1: Gyroscope calibration
  showMsg("STEP 1/2", "GYROSCOPE", "Hold still!");
  delay(2000);
  runGyroCalibration();
  
  feedWatchdog();
  
  // Step 2: Magnetometer calibration
  showMsg("STEP 2/2", "MAGNETOMETER", "Rotate slowly");
  delay(2000);
  runMagCalibration();
  
  feedWatchdog();
  
  // Calculate final calibration values
  calculateMagCalibration();
  
  // Save to NVS
  saveCalibration();
  
  feedWatchdog();
  
  // Show results (max 21 chars per line)
  char line1[22], line2[22];
  snprintf(line1, sizeof(line1), "Quality: %d%%", g_cal.quality);
  // Truncate range values to fit display
  int rx = (int)(g_cal.magMax[0] - g_cal.magMin[0]);
  int ry = (int)(g_cal.magMax[1] - g_cal.magMin[1]);
  int rz = (int)(g_cal.magMax[2] - g_cal.magMin[2]);
  snprintf(line2, sizeof(line2), "%d/%d/%d", rx, ry, rz);
  
  showMsg("CAL OK! Saved", line1, line2);
  
  Serial.println(F("Calibration complete!"));
  Serial.printf("Quality: %d%%\n", g_cal.quality);
  
  delay(3000);
  feedWatchdog();
  
  g_calibrationMode = false;
  digitalWrite(LED_PIN, LOW);
}

/**
 * @brief Gyroscope calibration - average readings while stationary
 */
void runGyroCalibration(void) {
  showCalibrationScreen("GYRO CAL", "Hold still!", nullptr, nullptr, 0);
  delay(1000);
  
  long gyroSum[3] = {0, 0, 0};
  int validSamples = 0;
  
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    // Feed watchdog every 100 samples
    if (i % 100 == 0) {
      feedWatchdog();
    }
    
    if (imu.dataReady()) {
      imu.getAGMT();
      if (imu.status == ICM_20948_Stat_Ok) {
        gyroSum[0] += imu.agmt.gyr.axes.x;
        gyroSum[1] += imu.agmt.gyr.axes.y;
        gyroSum[2] += imu.agmt.gyr.axes.z;
        validSamples++;
      }
    }
    
    // Update progress bar
    if (i % 50 == 0) {
      int progress = (i * 100) / GYRO_CAL_SAMPLES;
      showCalibrationScreen("GYRO CAL", "Hold still!", nullptr, nullptr, progress);
    }
    
    delay(2);  // ~500Hz sample rate
  }
  
  if (validSamples > 0) {
    g_cal.gyroOffset[0] = (float)gyroSum[0] / validSamples;
    g_cal.gyroOffset[1] = (float)gyroSum[1] / validSamples;
    g_cal.gyroOffset[2] = (float)gyroSum[2] / validSamples;
  }
  
  Serial.printf("Gyro offset: [%.2f, %.2f, %.2f] from %d samples\n",
                g_cal.gyroOffset[0], g_cal.gyroOffset[1], g_cal.gyroOffset[2], validSamples);
  
  showCalibrationScreen("GYRO CAL", "Done!", nullptr, nullptr, 100);
  delay(500);
}

/**
 * @brief Magnetometer calibration with visual feedback
 * Collects samples while user rotates sensor in all directions
 */
void runMagCalibration(void) {
  // Reset sample buffer
  g_sampleCount = 0;
  
  // Initialize min/max to extremes
  float magMin[3] = {1e10f, 1e10f, 1e10f};
  float magMax[3] = {-1e10f, -1e10f, -1e10f};
  
  unsigned long startTime = millis();
  unsigned long lastUpdate = 0;
  unsigned long lastWdtFeed = 0;
  bool complete = false;
  
  showCalibrationScreen("MAG CAL", "Rotate all dirs", "BTN=done", nullptr, 0);
  
  while (!complete && (millis() - startTime < MAG_CAL_MAX_TIME_MS)) {
    // Feed watchdog regularly
    if (millis() - lastWdtFeed >= 1000) {
      feedWatchdog();
      lastWdtFeed = millis();
    }
    
    // Check for button press to end early
    updateButton();
    if (wasButtonPressed() && (millis() - startTime > MAG_CAL_MIN_TIME_MS)) {
      complete = true;
      break;
    }
    
    // Read magnetometer
    if (imu.dataReady()) {
      imu.getAGMT();
      
      // Check IMU status
      if (imu.status != ICM_20948_Stat_Ok) {
        continue;  // Skip bad readings
      }
      
      // Apply axis mapping for AK09916 magnetometer
      float mx = imu.magY();    // Swap and invert for proper orientation
      float my = -imu.magX();
      float mz = -imu.magZ();
      
      // Validate reading
      if (validateMagData(mx, my, mz)) {
        // Update min/max
        if (mx < magMin[0]) magMin[0] = mx;
        if (mx > magMax[0]) magMax[0] = mx;
        if (my < magMin[1]) magMin[1] = my;
        if (my > magMax[1]) magMax[1] = my;
        if (mz < magMin[2]) magMin[2] = mz;
        if (mz > magMax[2]) magMax[2] = mz;
        
        // Store sample for ellipsoid fitting (if buffer not full)
        if (g_sampleCount < MAX_CAL_SAMPLES) {
          g_magSamples[g_sampleCount][0] = mx;
          g_magSamples[g_sampleCount][1] = my;
          g_magSamples[g_sampleCount][2] = mz;
          g_sampleCount++;
        }
      }
    }
    
    // Update display periodically
    if (millis() - lastUpdate >= MAG_UPDATE_INTERVAL) {
      lastUpdate = millis();
      
      float rangeX = magMax[0] - magMin[0];
      float rangeY = magMax[1] - magMin[1];
      float rangeZ = magMax[2] - magMin[2];
      
      unsigned long elapsed = millis() - startTime;
      bool xOk = (rangeX >= MAG_CAL_MIN_RANGE);
      bool yOk = (rangeY >= MAG_CAL_MIN_RANGE);
      bool minTimeOk = (elapsed >= MAG_CAL_MIN_TIME_MS);
      bool minSamplesOk = (g_sampleCount >= MAG_CAL_MIN_SAMPLES);
      
      // Auto-complete when all criteria met
      if (xOk && yOk && minTimeOk && minSamplesOk) {
        complete = true;
      }
      
      // Calculate progress based on coverage
      int progress = 0;
      progress += (xOk ? 25 : min(25, (int)(rangeX * 25 / MAG_CAL_MIN_RANGE)));
      progress += (yOk ? 25 : min(25, (int)(rangeY * 25 / MAG_CAL_MIN_RANGE)));
      progress += (minTimeOk ? 25 : (int)(elapsed * 25 / MAG_CAL_MIN_TIME_MS));
      progress += (minSamplesOk ? 25 : (g_sampleCount * 25 / MAG_CAL_MIN_SAMPLES));
      
      // Build status lines - compact format for 128x32 OLED (max 21 chars)
      char line1[22], line2[22];
      snprintf(line1, sizeof(line1), "%c%c %ds", 
               xOk ? 'X' : 'x', yOk ? 'Y' : 'y',
               (int)((MAG_CAL_MAX_TIME_MS - elapsed) / 1000));
      snprintf(line2, sizeof(line2), "n%d %.0f/%.0f", g_sampleCount, rangeX, rangeY);
      
      showCalibrationScreen("MAG CAL", line1, line2, nullptr, progress);
      
      // Blink LED
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delay(10);
  }
  
  feedWatchdog();
  
  // Store min/max
  for (int i = 0; i < 3; i++) {
    g_cal.magMin[i] = magMin[i];
    g_cal.magMax[i] = magMax[i];
  }
  
  Serial.printf("Mag calibration complete: %d samples\n", g_sampleCount);
  Serial.printf("  X: [%.1f, %.1f] range=%.1f\n", magMin[0], magMax[0], magMax[0] - magMin[0]);
  Serial.printf("  Y: [%.1f, %.1f] range=%.1f\n", magMin[1], magMax[1], magMax[1] - magMin[1]);
  Serial.printf("  Z: [%.1f, %.1f] range=%.1f\n", magMin[2], magMax[2], magMax[2] - magMin[2]);
  
  showCalibrationScreen("MAG CAL", "Processing...", nullptr, nullptr, 100);
  delay(500);
}

/**
 * @brief Calculate calibration parameters using collected samples
 * 
 * This function attempts ellipsoid fitting first (for best accuracy).
 * If ellipsoid fitting fails or doesn't converge, falls back to min/max method.
 * 
 * Based on:
 * - Sailboat Instruments: Li's ellipsoid specific fitting algorithm
 * - jremington/ICM_20948-AHRS calibrate3.py
 * - Cave Pearl Project min/max method (fallback)
 * - IOP Science: DOI 10.1088/1755-1315/237/3/032015
 */
void calculateMagCalibration(void) {
  Serial.println(F("\n=== Calculating Magnetometer Calibration ==="));
  Serial.printf("Using %d samples\n", g_sampleCount);
  
  // Try ellipsoid fitting first if we have enough samples
  if (g_sampleCount >= 100) {
    Serial.println(F("Attempting ellipsoid fitting (Li's algorithm)..."));
    calculateMagCalibrationEllipsoid();
  } else {
    Serial.println(F("Not enough samples for ellipsoid fitting, using min/max method"));
    calculateMagCalibrationMinMax();
  }
}

/**
 * @brief Ellipsoid fitting calibration using Li's algorithm
 * 
 * This implements the least squares ellipsoid specific fitting algorithm
 * as described in:
 * - Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific fitting"
 * - https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html
 * - https://github.com/jremington/ICM_20948-AHRS/blob/main/calibrate3.py
 * 
 * The algorithm fits an ellipsoid to the magnetometer data and calculates:
 * - Hard iron bias (B): offset vector to subtract
 * - Soft iron matrix (A_inv): 3x3 transformation matrix
 */
void calculateMagCalibrationEllipsoid(void) {
  // We'll implement a simplified version of Li's algorithm
  // that works well on embedded systems
  
  float residual = 0;
  float M[3][3] = {{0}};
  float bias[3] = {0};
  
  bool success = ellipsoidFit(g_magSamples, g_sampleCount, M, bias, &residual);
  
  if (success) {
    // Store the results
    for (int i = 0; i < 3; i++) {
      g_cal.magBias[i] = bias[i];
      for (int j = 0; j < 3; j++) {
        g_cal.magAinv[i][j] = M[i][j];
      }
    }
    g_cal.useEllipsoidFit = true;
    
    Serial.println(F("Ellipsoid fitting successful!"));
    Serial.printf("  Residual: %.4f\n", residual);
    Serial.printf("  Bias (B): [%.2f, %.2f, %.2f]\n", bias[0], bias[1], bias[2]);
    Serial.println(F("  A_inv matrix:"));
    for (int i = 0; i < 3; i++) {
      Serial.printf("    [%.5f, %.5f, %.5f]\n", M[i][0], M[i][1], M[i][2]);
    }
    
    // Calculate quality based on residual and sphericity
    // Lower residual = better fit = higher quality
    float qualityFromResidual = max(0.0f, 100.0f - residual * 10.0f);
    g_cal.quality = (uint8_t)min(100.0f, qualityFromResidual);
    
  } else {
    Serial.println(F("Ellipsoid fitting failed, falling back to min/max method"));
    calculateMagCalibrationMinMax();
  }
}

/**
 * @brief Simple min/max calibration (Cave Pearl Project method)
 * 
 * This is the fallback method when ellipsoid fitting fails.
 * It calculates:
 * - Hard iron: offset = (max + min) / 2
 * - Soft iron: scale = avgDelta / delta (stored as diagonal matrix)
 */
void calculateMagCalibrationMinMax(void) {
  float delta[3];
  
  // Calculate Hard Iron offsets (center of min/max range)
  for (int i = 0; i < 3; i++) {
    g_cal.magBias[i] = (g_cal.magMax[i] + g_cal.magMin[i]) * 0.5f;
    delta[i] = g_cal.magMax[i] - g_cal.magMin[i];
  }
  
  // Calculate average diameter (for ideal sphere)
  float avgDelta = (delta[0] + delta[1] + delta[2]) / 3.0f;
  
  // Initialize A_inv as identity matrix with scaling on diagonal
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        // Diagonal: scale factor
        g_cal.magAinv[i][j] = (delta[i] > 1.0f) ? (avgDelta / delta[i]) : 1.0f;
      } else {
        // Off-diagonal: zero (no cross-axis correction)
        g_cal.magAinv[i][j] = 0.0f;
      }
    }
  }
  
  g_cal.useEllipsoidFit = false;
  
  // Calculate quality score
  float rangeScore = 0;
  if (delta[0] >= MAG_CAL_MIN_RANGE) rangeScore += 33;
  if (delta[1] >= MAG_CAL_MIN_RANGE) rangeScore += 33;
  if (delta[2] >= MAG_CAL_MIN_RANGE * 0.5f) rangeScore += 34;
  
  float minDelta = min(delta[0], min(delta[1], delta[2]));
  float maxDelta = max(delta[0], max(delta[1], delta[2]));
  float sphericity = (maxDelta > 0) ? (minDelta / maxDelta) * 100 : 0;
  
  g_cal.quality = (uint8_t)((rangeScore + sphericity) / 2);
  
  Serial.println(F("Min/Max calibration calculated:"));
  Serial.printf("  Bias: [%.2f, %.2f, %.2f]\n", 
                g_cal.magBias[0], g_cal.magBias[1], g_cal.magBias[2]);
  Serial.printf("  Scale (diagonal): [%.4f, %.4f, %.4f]\n",
                g_cal.magAinv[0][0], g_cal.magAinv[1][1], g_cal.magAinv[2][2]);
  Serial.printf("  Quality: %d%%\n", g_cal.quality);
}

/**
 * @brief Initialize default calibration values
 * Sets identity matrix for A_inv and zero bias
 */
void initDefaultCalibration(void) {
  for (int i = 0; i < 3; i++) {
    g_cal.gyroOffset[i] = 0.0f;
    g_cal.magBias[i] = 0.0f;
    g_cal.accelBias[i] = 0.0f;
    g_cal.magMin[i] = -200.0f;
    g_cal.magMax[i] = 200.0f;
    for (int j = 0; j < 3; j++) {
      // Identity matrix
      g_cal.magAinv[i][j] = (i == j) ? 1.0f : 0.0f;
      g_cal.accelAinv[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  g_cal.isValid = false;
  g_cal.quality = 0;
  g_cal.useEllipsoidFit = false;
}

/**
 * @brief Apply magnetometer calibration using 3x3 matrix multiplication
 * 
 * Calibrated = A_inv * (Raw - Bias)
 * 
 * This format is compatible with jremington/ICM_20948-AHRS
 */
void applyMagCalibration(float raw[3], float calibrated[3]) {
  float temp[3];
  
  // Step 1: Subtract hard iron bias
  for (int i = 0; i < 3; i++) {
    temp[i] = raw[i] - g_cal.magBias[i];
  }
  
  // Step 2: Apply soft iron correction matrix (3x3 multiplication)
  // calibrated = A_inv * temp
  for (int i = 0; i < 3; i++) {
    calibrated[i] = g_cal.magAinv[i][0] * temp[0] + 
                    g_cal.magAinv[i][1] * temp[1] + 
                    g_cal.magAinv[i][2] * temp[2];
  }
}

// ============================================================================
// SENSOR PROCESSING
// ============================================================================

bool validateMagData(float mx, float my, float mz) {
  // Check for zero readings (sensor not ready)
  if (mx == 0 && my == 0 && mz == 0) return false;
  
  // Check for out of range values - any axis out of bounds invalidates reading
  if (fabs(mx) > MAG_VALID_MAX || fabs(my) > MAG_VALID_MAX || fabs(mz) > MAG_VALID_MAX) return false;
  
  // Check if all axes have very low readings (noise only)
  // At least one axis should have a meaningful reading
  float magnitude = sqrtf(mx*mx + my*my + mz*mz);
  if (magnitude < MAG_VALID_MIN) return false;
  
  return true;
}

void processSensors(void) {
  float Gxyz[3], Axyz[3], Mxyz[3], McalXyz[3];
  
  // Read gyroscope (apply offset and convert to rad/s)
  Gxyz[0] = GYRO_SCALE * (imu.agmt.gyr.axes.x - g_cal.gyroOffset[0]);
  Gxyz[1] = GYRO_SCALE * (imu.agmt.gyr.axes.y - g_cal.gyroOffset[1]);
  Gxyz[2] = GYRO_SCALE * (imu.agmt.gyr.axes.z - g_cal.gyroOffset[2]);
  
  // Read and normalize accelerometer
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  vector_normalize(Axyz);
  
  // Read magnetometer with axis mapping for AK09916
  Mxyz[0] = imu.magY();     // AK09916 Y -> sensor X
  Mxyz[1] = -imu.magX();    // AK09916 -X -> sensor Y
  Mxyz[2] = -imu.magZ();    // AK09916 -Z -> sensor Z
  
  // Validate mag data
  if (!validateMagData(Mxyz[0], Mxyz[1], Mxyz[2])) return;
  
  // Apply calibration
  if (g_cal.isValid) {
    applyMagCalibration(Mxyz, McalXyz);
  } else {
    // Use raw values if no calibration
    for (int i = 0; i < 3; i++) McalXyz[i] = Mxyz[i];
  }
  vector_normalize(McalXyz);
  
  // Align magnetometer axes with accelerometer frame
  // This depends on the physical mounting of the sensor
  McalXyz[1] = -McalXyz[1];
  McalXyz[2] = -McalXyz[2];
  
  // Calculate delta time with overflow protection
  // micros() overflows every ~71 minutes, handle this gracefully
  unsigned long now = micros();
  unsigned long elapsed;
  
  // Handle micros() overflow (unsigned subtraction handles this correctly)
  elapsed = now - g_ahrs.lastUpdate;
  
  // Validate elapsed time - if it seems unreasonable, use a safe default
  // Maximum reasonable delta is ~100ms (10 updates/second)
  if (elapsed > 100000UL) {
    // Either overflow happened or we were stalled - use safe default
    elapsed = 10000UL;  // 10ms default
  }
  
  float deltat = elapsed * 1.0e-6f;
  g_ahrs.lastUpdate = now;
  
  // Additional safety check - ensure deltat is positive and reasonable
  // Min 0.1ms to avoid division issues, max 20ms for AHRS stability
  if (deltat < 0.0001f) {
    deltat = 0.0001f;
  } else if (deltat > 0.02f) {
    deltat = 0.02f;
  }
  
  // Check for NaN in gyro/accel/mag data (can cause AHRS instability)
  if (isnan(Gxyz[0]) || isnan(Gxyz[1]) || isnan(Gxyz[2]) ||
      isnan(Axyz[0]) || isnan(Axyz[1]) || isnan(Axyz[2]) ||
      isnan(McalXyz[0]) || isnan(McalXyz[1]) || isnan(McalXyz[2])) {
    return;  // Skip this update
  }
  
  // Update Mahony AHRS
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2],
                         Gxyz[0], Gxyz[1], Gxyz[2],
                         McalXyz[0], McalXyz[1], McalXyz[2], deltat);
  
  // Check quaternion validity after update
  float qMag = sqrtf(g_ahrs.q[0]*g_ahrs.q[0] + g_ahrs.q[1]*g_ahrs.q[1] + 
                     g_ahrs.q[2]*g_ahrs.q[2] + g_ahrs.q[3]*g_ahrs.q[3]);
  if (isnan(qMag) || qMag < 0.9f || qMag > 1.1f) {
    // Quaternion is invalid - reset to identity
    g_ahrs.q[0] = 1.0f;
    g_ahrs.q[1] = 0.0f;
    g_ahrs.q[2] = 0.0f;
    g_ahrs.q[3] = 0.0f;
    g_ahrs.eInt[0] = g_ahrs.eInt[1] = g_ahrs.eInt[2] = 0.0f;
    g_sensor.filtersReady = false;
    Serial.println(F("AHRS quaternion reset due to invalid state"));
    return;
  }
  
  // Convert quaternion to Euler angles
  float yaw, pitch, roll;
  quaternionToEuler(g_ahrs.q, yaw, pitch, roll);
  
  // Apply magnetic declination for geographic north
  float geoYaw = -(yaw + MAGNETIC_DECLINATION);
  if (geoYaw < 0) geoYaw += 360.0f;
  if (geoYaw >= 360.0f) geoYaw -= 360.0f;
  
  // Store values
  g_sensor.roll = roll;
  g_sensor.pitch = pitch;
  g_sensor.headingGeo = geoYaw;
  
  // Magnetic heading (without declination)
  g_sensor.headingMag = geoYaw - MAGNETIC_DECLINATION;
  if (g_sensor.headingMag < 0) g_sensor.headingMag += 360.0f;
  if (g_sensor.headingMag >= 360.0f) g_sensor.headingMag -= 360.0f;
  
  // Apply EMA filters
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
// VECTOR MATH
// ============================================================================

float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrtf(vector_dot(a, a));
  if (mag > 1e-9f) {
    float invMag = 1.0f / mag;
    a[0] *= invMag;
    a[1] *= invMag;
    a[2] *= invMag;
  }
}

void vector_cross(float a[3], float b[3], float result[3]) {
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

// ============================================================================
// MAHONY AHRS FILTER
// ============================================================================

/**
 * @brief Mahony quaternion update with Up and West reference vectors
 * Based on: https://github.com/jremington/ICM_20948-AHRS
 */
void MahonyQuaternionUpdate(float ax, float ay, float az, 
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float deltat) {
  float q1 = g_ahrs.q[0], q2 = g_ahrs.q[1], q3 = g_ahrs.q[2], q4 = g_ahrs.q[3];
  float norm;
  float hx, hy, hz;
  float ux, uy, uz, wx, wy, wz;
  float ex, ey, ez;
  
  // Pre-compute quaternion products
  float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4;
  float q2q2 = q2 * q2, q2q3 = q2 * q3, q2q4 = q2 * q4;
  float q3q3 = q3 * q3, q3q4 = q3 * q4, q4q4 = q4 * q4;
  
  // Compute horizon vector = a x m (in body frame)
  // This gives a vector perpendicular to both gravity and magnetic field
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  
  // Normalize horizon vector
  norm = sqrtf(hx * hx + hy * hy + hz * hz);
  if (norm < 1e-9f) return;  // Cannot determine heading
  norm = 1.0f / norm;
  hx *= norm; hy *= norm; hz *= norm;
  
  // Estimated Up direction from quaternion (world Z in body frame)
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;
  
  // Estimated West direction from quaternion (world Y in body frame)
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);
  
  // Error is sum of cross products between measured and estimated directions
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  
  // Apply integral feedback (if enabled)
  if (MAHONY_KI > 0.0f) {
    g_ahrs.eInt[0] += ex * deltat;
    g_ahrs.eInt[1] += ey * deltat;
    g_ahrs.eInt[2] += ez * deltat;
    
    // Limit integral windup
    const float maxInt = 0.5f;
    for (int i = 0; i < 3; i++) {
      if (g_ahrs.eInt[i] > maxInt) g_ahrs.eInt[i] = maxInt;
      if (g_ahrs.eInt[i] < -maxInt) g_ahrs.eInt[i] = -maxInt;
    }
    
    gx += MAHONY_KI * g_ahrs.eInt[0];
    gy += MAHONY_KI * g_ahrs.eInt[1];
    gz += MAHONY_KI * g_ahrs.eInt[2];
  }
  
  // Apply proportional feedback
  gx += MAHONY_KP * ex;
  gy += MAHONY_KP * ey;
  gz += MAHONY_KP * ez;
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * deltat);
  gy *= (0.5f * deltat);
  gz *= (0.5f * deltat);
  
  float qa = q1, qb = q2, qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);
  
  // Normalize quaternion
  norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  norm = 1.0f / norm;
  g_ahrs.q[0] = q1 * norm;
  g_ahrs.q[1] = q2 * norm;
  g_ahrs.q[2] = q3 * norm;
  g_ahrs.q[3] = q4 * norm;
}

/**
 * @brief Convert quaternion to Euler angles (NWU convention)
 */
void quaternionToEuler(float q[4], float& yaw, float& pitch, float& roll) {
  // ZYX Euler angles from quaternion
  roll  = atan2f((q[0] * q[1] + q[2] * q[3]), 0.5f - (q[1] * q[1] + q[2] * q[2]));
  pitch = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
  yaw   = atan2f((q[1] * q[2] + q[0] * q[3]), 0.5f - (q[2] * q[2] + q[3] * q[3]));
  
  // Convert to degrees
  yaw   *= 180.0f / M_PI;
  pitch *= 180.0f / M_PI;
  roll  *= 180.0f / M_PI;
}

// ============================================================================
// FILTER FUNCTIONS
// ============================================================================

float emaFilter(float newVal, float oldVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

/**
 * @brief EMA filter for angular values (handles 0/360 wrap-around)
 */
float emaFilterAngle(float newAngle, float oldAngle, float alpha) {
  float newRad = newAngle * M_PI / 180.0f;
  float oldRad = oldAngle * M_PI / 180.0f;
  
  float newX = cosf(newRad), newY = sinf(newRad);
  float oldX = cosf(oldRad), oldY = sinf(oldRad);
  
  float filtX = alpha * newX + (1.0f - alpha) * oldX;
  float filtY = alpha * newY + (1.0f - alpha) * oldY;
  
  float result = atan2f(filtY, filtX) * 180.0f / M_PI;
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
  if (!g_cal.isValid) {
    display.print(F(" !"));
  }
  
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
  
  // Draw simple compass indicator on right side
  int cx = 110;  // Center x
  int cy = 16;   // Center y (middle of 32-pixel height)
  int r = 10;    // Radius
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  
  // Draw north pointer
  float headingRad = g_sensor.headingGeoFiltered * M_PI / 180.0f;
  int nx = cx + (int)(r * sinf(-headingRad));
  int ny = cy - (int)(r * cosf(-headingRad));
  display.drawLine(cx, cy, nx, ny, SSD1306_WHITE);
  
  display.display();
}

/**
 * @brief Show calibration screen with title, messages, and progress bar
 */
void showCalibrationScreen(const char* title, const char* line1, 
                           const char* line2, const char* line3,
                           int progress) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title line (row 0)
  display.setCursor(0, 0);
  display.print(title);
  
  // Line 1 (row 1)
  if (line1) {
    display.setCursor(0, 10);
    display.print(line1);
  }
  
  // Progress bar (if >= 0) at row 2, or show line2/line3
  if (progress >= 0) {
    // Show line2 after title if provided (title is ~8 chars = 48px, add margin)
    if (line2) {
      display.setCursor(10 * CHAR_WIDTH, 0);  // Position after ~10 char title
      display.print(line2);
    }
    
    int barY = 22;
    int barH = 8;
    int barW = SCREEN_WIDTH - 5 * CHAR_WIDTH;  // Leave space for percentage text
    
    display.drawRect(0, barY, barW, barH, SSD1306_WHITE);
    int fillW = (progress * (barW - 2)) / 100;
    if (fillW > 0) {
      display.fillRect(1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
    }
    
    // Show percentage
    display.setCursor(barW + 3, barY);
    display.print(progress);
    display.print(F("%"));
  } else {
    // No progress bar - show line2 and line3
    if (line2) {
      display.setCursor(0, 11);
      display.print(line2);
    }
    if (line3) {
      display.setCursor(0, 22);
      display.print(line3);
    }
  }
  
  display.display();
}

void showMsg(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Compact layout for 32-pixel height (3 lines max, 11 pixels spacing)
  if (l1) { display.setCursor(0, 0);  display.println(l1); }
  if (l2) { display.setCursor(0, 11); display.println(l2); }
  if (l3) { display.setCursor(0, 22); display.println(l3); }
  
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

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void updateButton(void) {
  bool reading = digitalRead(BUTTON_PIN);
  
  // Reset long press flag at start of each update cycle
  g_button.longPressDetected = false;
  
  // Check for state change
  if (reading != g_button.lastState) {
    g_button.lastDebounceTime = millis();
  }
  
  // Apply debounce
  if ((millis() - g_button.lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != g_button.currentState) {
      g_button.currentState = reading;
      
      if (g_button.currentState == LOW) {
        // Button just pressed
        g_button.isPressed = true;
        g_button.pressStartTime = millis();
      } else {
        // Button just released
        g_button.isPressed = false;
      }
    }
  }
  
  // Check for long press while button is held
  if (g_button.currentState == LOW && g_button.isPressed) {
    if ((millis() - g_button.pressStartTime) >= LONG_PRESS_MS) {
      g_button.longPressDetected = true;
      g_button.isPressed = false;  // Consume the press
    }
  }
  
  g_button.lastState = reading;
}

bool wasButtonPressed(void) {
  if (g_button.isPressed && g_button.currentState == HIGH) {
    g_button.isPressed = false;
    return true;
  }
  return false;
}

bool wasLongPress(void) {
  return g_button.longPressDetected;
}

// ============================================================================
// ELLIPSOID FITTING FUNCTIONS
// ============================================================================
// Implementation based on:
// - Li's algorithm: "Least squares ellipsoid specific fitting"
// - jremington/ICM_20948-AHRS calibrate3.py
// - Sailboat Instruments magneto program
// - IOP Science: DOI 10.1088/1755-1315/237/3/032015

/**
 * @brief Calculate 3x3 matrix determinant
 */
float matrix3x3Determinant(float M[3][3]) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
       - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
       + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

/**
 * @brief Calculate 3x3 matrix inverse
 * Returns false if matrix is singular
 */
void matrix3x3Inverse(float M[3][3], float Minv[3][3]) {
  float det = matrix3x3Determinant(M);
  if (fabsf(det) < MATRIX_EPSILON) {
    // Singular matrix - return identity
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Minv[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }
    return;
  }
  
  float invDet = 1.0f / det;
  
  Minv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * invDet;
  Minv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * invDet;
  Minv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * invDet;
  Minv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) * invDet;
  Minv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * invDet;
  Minv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * invDet;
  Minv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * invDet;
  Minv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * invDet;
  Minv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * invDet;
}

/**
 * @brief Multiply two 3x3 matrices: C = A * B
 */
void matrix3x3Multiply(float A[3][3], float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

/**
 * @brief Cholesky decomposition of a symmetric positive definite matrix
 * A = L * L^T, returns L (lower triangular)
 */
void choleskyDecomposition(float A[3][3], float L[3][3]) {
  // Initialize L to zero
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      L[i][j] = 0;
    }
  }
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j <= i; j++) {
      float sum = 0;
      
      if (j == i) {
        // Diagonal elements
        for (int k = 0; k < j; k++) {
          sum += L[j][k] * L[j][k];
        }
        float val = A[j][j] - sum;
        L[j][j] = (val > 0) ? sqrtf(val) : CHOLESKY_MIN_DIAG;
      } else {
        // Off-diagonal elements
        for (int k = 0; k < j; k++) {
          sum += L[i][k] * L[j][k];
        }
        L[i][j] = (fabsf(L[j][j]) > MATRIX_EPSILON) ? (A[i][j] - sum) / L[j][j] : 0;
      }
    }
  }
}

/**
 * @brief Matrix square root using Denman-Beavers iteration
 * Calculates sqrtM such that sqrtM * sqrtM = M (approximately)
 */
void matrix3x3Sqrt(float M[3][3], float sqrtM[3][3]) {
  float Y[3][3], Z[3][3], Ynew[3][3], Znew[3][3];
  float Yinv[3][3], Zinv[3][3];
  
  // Initialize: Y = M, Z = I
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Y[i][j] = M[i][j];
      Z[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  
  // Denman-Beavers iteration (5 iterations is usually enough)
  for (int iter = 0; iter < 5; iter++) {
    matrix3x3Inverse(Y, Yinv);
    matrix3x3Inverse(Z, Zinv);
    
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Ynew[i][j] = 0.5f * (Y[i][j] + Zinv[i][j]);
        Znew[i][j] = 0.5f * (Z[i][j] + Yinv[i][j]);
      }
    }
    
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Y[i][j] = Ynew[i][j];
        Z[i][j] = Znew[i][j];
      }
    }
  }
  
  // Result is in Y
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      sqrtM[i][j] = Y[i][j];
    }
  }
}

/**
 * @brief Ellipsoid fitting using Li's least squares algorithm
 * 
 * Based on: Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific fitting"
 * and jremington/ICM_20948-AHRS calibrate3.py
 * 
 * The algorithm fits an ellipsoid to the magnetometer data points and
 * calculates the hard iron bias (B) and soft iron correction matrix (A_inv).
 * 
 * The general ellipsoid equation is:
 * (x-b)^T * M * (x-b) = 1
 * 
 * Where:
 * - M is a 3x3 symmetric positive definite matrix describing the ellipsoid shape
 * - b is the center (hard iron bias)
 * 
 * The correction is: calibrated = A_inv * (raw - B)
 * Where A_inv = F * sqrt(M), F is a normalization factor
 * 
 * @param samples Array of magnetometer samples [n][3]
 * @param n Number of samples
 * @param Ainv Output: 3x3 soft iron correction matrix
 * @param bias Output: 3-element hard iron bias vector
 * @param residual Output: fitting residual (lower is better)
 * @return true if fitting succeeded
 */
bool ellipsoidFit(float samples[][3], int n, float Ainv[3][3], float bias[3], float* residual) {
  if (n < 10) {
    return false;
  }
  
  // Build design matrix D for ellipsoid fitting
  // D = [x^2, y^2, z^2, 2*y*z, 2*x*z, 2*x*y, 2*x, 2*y, 2*z, 1]
  // This is a 10-parameter model for the general ellipsoid
  
  // We'll use a simplified approach that works well on embedded systems:
  // 1. Find the center using weighted centroid
  // 2. Compute the covariance matrix
  // 3. Use eigenvalue decomposition to find the axes
  
  // Step 1: Find approximate center using min/max
  float xmin = 1e10f, xmax = -1e10f;
  float ymin = 1e10f, ymax = -1e10f;
  float zmin = 1e10f, zmax = -1e10f;
  
  for (int i = 0; i < n; i++) {
    if (samples[i][0] < xmin) xmin = samples[i][0];
    if (samples[i][0] > xmax) xmax = samples[i][0];
    if (samples[i][1] < ymin) ymin = samples[i][1];
    if (samples[i][1] > ymax) ymax = samples[i][1];
    if (samples[i][2] < zmin) zmin = samples[i][2];
    if (samples[i][2] > zmax) zmax = samples[i][2];
  }
  
  // Initial center estimate
  float cx = (xmin + xmax) / 2.0f;
  float cy = (ymin + ymax) / 2.0f;
  float cz = (zmin + zmax) / 2.0f;
  
  // Step 2: Iteratively refine center using least squares
  // We'll do a few iterations to converge on the best center
  for (int iter = 0; iter < 5; iter++) {
    // Compute centered data
    float sumX = 0, sumY = 0, sumZ = 0;
    float sumX2 = 0, sumY2 = 0, sumZ2 = 0;
    float sumXY = 0, sumXZ = 0, sumYZ = 0;
    
    for (int i = 0; i < n; i++) {
      float x = samples[i][0] - cx;
      float y = samples[i][1] - cy;
      float z = samples[i][2] - cz;
      
      // Weight samples by their distance from center
      float r = sqrtf(x*x + y*y + z*z);
      if (r < 1.0f) r = 1.0f;
      float w = 1.0f / r;  // Inverse distance weighting
      
      sumX += x * w;
      sumY += y * w;
      sumZ += z * w;
      sumX2 += x * x * w;
      sumY2 += y * y * w;
      sumZ2 += z * z * w;
      sumXY += x * y * w;
      sumXZ += x * z * w;
      sumYZ += y * z * w;
    }
    
    // Update center slightly towards weighted centroid of residuals
    cx += sumX / n * 0.5f;
    cy += sumY / n * 0.5f;
    cz += sumZ / n * 0.5f;
  }
  
  bias[0] = cx;
  bias[1] = cy;
  bias[2] = cz;
  
  // Step 3: Compute covariance matrix of centered data
  float cov[3][3] = {{0}};
  
  for (int i = 0; i < n; i++) {
    float x = samples[i][0] - cx;
    float y = samples[i][1] - cy;
    float z = samples[i][2] - cz;
    
    cov[0][0] += x * x;
    cov[0][1] += x * y;
    cov[0][2] += x * z;
    cov[1][1] += y * y;
    cov[1][2] += y * z;
    cov[2][2] += z * z;
  }
  
  // Make symmetric
  cov[1][0] = cov[0][1];
  cov[2][0] = cov[0][2];
  cov[2][1] = cov[1][2];
  
  // Normalize
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cov[i][j] /= n;
    }
  }
  
  // Step 4: Calculate the scaling matrix
  // For a perfect sphere, cov would be proportional to identity
  // The eigenvalues tell us the axis lengths of the ellipsoid
  
  // Use power iteration to find approximate eigenvalues
  float eigval[3];
  float temp[3][3];
  
  // Copy covariance matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      temp[i][j] = cov[i][j];
    }
  }
  
  // Approximate eigenvalues using diagonal of matrix
  // (This is a simplification - for more accuracy, use full eigendecomposition)
  eigval[0] = sqrtf(cov[0][0]);
  eigval[1] = sqrtf(cov[1][1]);
  eigval[2] = sqrtf(cov[2][2]);
  
  // Average radius
  float avgRadius = (eigval[0] + eigval[1] + eigval[2]) / 3.0f;
  if (avgRadius < 1.0f) avgRadius = 1.0f;
  
  // Step 5: Build the inverse transformation matrix
  // This normalizes the ellipsoid to a sphere
  
  // Simple diagonal scaling (ignores rotation - works for most practical cases)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        // Scale factor for this axis
        float scale = (eigval[i] > 0.1f) ? (avgRadius / eigval[i]) : 1.0f;
        // Normalize to MAG_FIELD_NORM
        Ainv[i][j] = scale * (MAG_FIELD_NORM / avgRadius);
      } else {
        // Off-diagonal: correct for non-orthogonality
        // Use covariance to estimate cross-axis coupling
        float offDiag = cov[i][j] / (eigval[i] * eigval[j] + 0.001f);
        Ainv[i][j] = -offDiag * (MAG_FIELD_NORM / avgRadius) * 0.5f;
      }
    }
  }
  
  // Step 6: Calculate residual (how well the ellipsoid fits)
  float sumResidual = 0;
  for (int i = 0; i < n; i++) {
    float x = samples[i][0] - cx;
    float y = samples[i][1] - cy;
    float z = samples[i][2] - cz;
    
    // Apply correction
    float cx2 = Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z;
    float cy2 = Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z;
    float cz2 = Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z;
    
    // Magnitude should be close to MAG_FIELD_NORM
    float mag = sqrtf(cx2*cx2 + cy2*cy2 + cz2*cz2);
    float err = fabsf(mag - MAG_FIELD_NORM) / MAG_FIELD_NORM;
    sumResidual += err * err;
  }
  
  *residual = sqrtf(sumResidual / n) * 100.0f;  // As percentage
  
  // Consider it successful if residual is below threshold
  return (*residual < ELLIPSOID_FIT_MAX_RESIDUAL);
}
