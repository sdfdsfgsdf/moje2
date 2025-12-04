/**
 * @file ICM20948_Compass_Tilt_ESP32.ino
 * @brief Advanced Compass and Inclinometer with Mahony AHRS for ESP32-WROOM-32D
 * 
 * Features:
 * - Full on-device calibration with ellipsoid fitting
 * - Button-driven calibration with OLED guidance
 * - Optimized for ESP32's FPU and dual-core capabilities
 * - NVS (Preferences) for calibration storage
 * - Hard Iron and Soft Iron correction
 * - Mahony AHRS filter with gyroscope integration
 * 
 * Hardware: ESP32-WROOM-32D, ICM-20948 IMU, OLED 128x32
 * Based on: jremington/ICM_20948-AHRS, Cave Pearl Project, Pololu magnetometer correction
 * Location: Zywiec, Poland (49.6853N, 19.1925E), Declination: 5.5E
 * 
 * References:
 * - https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
 * - https://github.com/jremington/ICM_20948-AHRS
 * - https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315
 * 
 * @license MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "ICM_20948.h"

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

// ============================================================================
// ICM-20948 CONFIGURATION
// ============================================================================

#define ICM_AD0_VAL       1    // AD0 pin state (0 = GND = 0x68, 1 = VCC = 0x69)
#define ICM_I2C_SPEED     400000

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
 * Uses Cave Pearl Project min/max method with soft iron scaling
 */
struct Calibration {
  // Gyroscope offsets (raw units)
  float gyroOffset[3];
  
  // Magnetometer Hard Iron correction (offset)
  float magOffset[3];
  
  // Magnetometer Soft Iron correction (scale factors)
  float magScale[3];
  
  // Min/Max values for calculating offset and scale
  float magMin[3];
  float magMax[3];
  
  // Accelerometer offset (if calibrated)
  float accelOffset[3];
  
  // Validation flag
  bool isValid;
  
  // Calibration quality indicator (0-100)
  uint8_t quality;
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
// Reduced from 1000 to 500 for better memory safety while still providing good calibration
#define MAX_CAL_SAMPLES 500
float g_magSamples[MAX_CAL_SAMPLES][3];
int g_sampleCount = 0;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

SensorData    g_sensor = {0};
Calibration   g_cal = {0};
AHRSState     g_ahrs = {{1.0f, 0.0f, 0.0f, 0.0f}, {0, 0, 0}, 0};
ButtonState   g_button = {HIGH, HIGH, 0, 0, false, false};

unsigned long g_lastDisplayUpdate = 0;
bool          g_calibrationMode = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// Initialization
void initDisplay(void);
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
void applyMagCalibration(float raw[3], float calibrated[3]);

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
             const char* l3 = nullptr, const char* l4 = nullptr);
float getDeviationFromNorth(float heading);
char getCardinalChar(float heading);

// Button handling
void updateButton(void);
bool wasButtonPressed(void);
bool wasLongPress(void);

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
  
  // Initialize button
  initButton();
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize display
  initDisplay();
  showMsg("ICM20948", "Compass ESP32", "Initializing...");
  delay(500);
  
  // Initialize IMU
  if (!initIMU()) {
    showMsg("ERROR!", "IMU not found", "Check wiring", "SDA:21 SCL:22");
    Serial.println(F("ERROR: IMU initialization failed!"));
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  configureIMU();
  Serial.println(F("IMU initialized successfully"));
  
  // Load saved calibration
  loadCalibration();
  
  if (g_cal.isValid) {
    showMsg("Calibration", "loaded from", "memory", "Hold BTN=recal");
    Serial.println(F("Calibration loaded from NVS"));
  } else {
    showMsg("No calibration", "Hold button", "for 2s to", "start cal");
    Serial.println(F("No valid calibration found"));
  }
  delay(2000);
  
  // Quick gyro calibration at startup
  if (!g_cal.isValid) {
    showMsg("Quick gyro", "calibration", "Hold still...");
    delay(1000);
    runGyroCalibration();
  }
  
  // Initialize AHRS timing
  g_ahrs.lastUpdate = micros();
  
  showMsg("Ready!", "Long press BTN", "for full cal");
  delay(1500);
  
  Serial.println(F("System ready!"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Update button state
  updateButton();
  
  // Check for long press to enter calibration
  if (wasLongPress()) {
    Serial.println(F("Long press detected - entering calibration mode"));
    runFullCalibration();
    g_ahrs.lastUpdate = micros();  // Reset timing after calibration
    g_sensor.filtersReady = false;  // Reset filters
  }
  
  // Normal operation - read sensors and update display
  if (imu.dataReady()) {
    imu.getAGMT();
    processSensors();
  }
  
  // Update display at fixed interval
  if (millis() - g_lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    g_lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // Small delay to prevent tight loop
  delay(1);
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void initDisplay(void) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("ERROR: SSD1306 allocation failed"));
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
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
    return;
  }
  
  bool valid = preferences.getBool("valid", false);
  if (!valid) {
    preferences.end();
    return;
  }
  
  // Load gyro offsets
  g_cal.gyroOffset[0] = preferences.getFloat("gx", 0.0f);
  g_cal.gyroOffset[1] = preferences.getFloat("gy", 0.0f);
  g_cal.gyroOffset[2] = preferences.getFloat("gz", 0.0f);
  
  // Load mag calibration
  g_cal.magOffset[0] = preferences.getFloat("mx_off", 0.0f);
  g_cal.magOffset[1] = preferences.getFloat("my_off", 0.0f);
  g_cal.magOffset[2] = preferences.getFloat("mz_off", 0.0f);
  
  g_cal.magScale[0] = preferences.getFloat("mx_scl", 1.0f);
  g_cal.magScale[1] = preferences.getFloat("my_scl", 1.0f);
  g_cal.magScale[2] = preferences.getFloat("mz_scl", 1.0f);
  
  // Load min/max for reference
  g_cal.magMin[0] = preferences.getFloat("mx_min", -200.0f);
  g_cal.magMin[1] = preferences.getFloat("my_min", -200.0f);
  g_cal.magMin[2] = preferences.getFloat("mz_min", -200.0f);
  
  g_cal.magMax[0] = preferences.getFloat("mx_max", 200.0f);
  g_cal.magMax[1] = preferences.getFloat("my_max", 200.0f);
  g_cal.magMax[2] = preferences.getFloat("mz_max", 200.0f);
  
  g_cal.quality = preferences.getUChar("quality", 0);
  g_cal.isValid = true;
  
  preferences.end();
  
  Serial.println(F("Calibration loaded:"));
  Serial.printf("  Gyro offset: [%.2f, %.2f, %.2f]\n", 
                g_cal.gyroOffset[0], g_cal.gyroOffset[1], g_cal.gyroOffset[2]);
  Serial.printf("  Mag offset: [%.2f, %.2f, %.2f]\n",
                g_cal.magOffset[0], g_cal.magOffset[1], g_cal.magOffset[2]);
  Serial.printf("  Mag scale: [%.3f, %.3f, %.3f]\n",
                g_cal.magScale[0], g_cal.magScale[1], g_cal.magScale[2]);
  Serial.printf("  Quality: %d%%\n", g_cal.quality);
}

void saveCalibration(void) {
  preferences.begin(NVS_NAMESPACE, false);  // Read-write mode
  
  // Save gyro offsets
  preferences.putFloat("gx", g_cal.gyroOffset[0]);
  preferences.putFloat("gy", g_cal.gyroOffset[1]);
  preferences.putFloat("gz", g_cal.gyroOffset[2]);
  
  // Save mag calibration
  preferences.putFloat("mx_off", g_cal.magOffset[0]);
  preferences.putFloat("my_off", g_cal.magOffset[1]);
  preferences.putFloat("mz_off", g_cal.magOffset[2]);
  
  preferences.putFloat("mx_scl", g_cal.magScale[0]);
  preferences.putFloat("my_scl", g_cal.magScale[1]);
  preferences.putFloat("mz_scl", g_cal.magScale[2]);
  
  // Save min/max
  preferences.putFloat("mx_min", g_cal.magMin[0]);
  preferences.putFloat("my_min", g_cal.magMin[1]);
  preferences.putFloat("mz_min", g_cal.magMin[2]);
  
  preferences.putFloat("mx_max", g_cal.magMax[0]);
  preferences.putFloat("my_max", g_cal.magMax[1]);
  preferences.putFloat("mz_max", g_cal.magMax[2]);
  
  preferences.putUChar("quality", g_cal.quality);
  preferences.putBool("valid", true);
  
  preferences.end();
  
  g_cal.isValid = true;
  Serial.println(F("Calibration saved to NVS"));
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
  showMsg("=CALIBRATION=", "", "Press button", "to start");
  Serial.println(F("\n=== Starting Full Calibration ==="));
  
  // Wait for button press or timeout
  unsigned long startWait = millis();
  while (!wasButtonPressed() && (millis() - startWait < 10000)) {
    updateButton();
    delay(10);
  }
  
  // Step 1: Gyroscope calibration
  showMsg("STEP 1/2", "GYROSCOPE", "", "Hold very still!");
  delay(2000);
  runGyroCalibration();
  
  // Step 2: Magnetometer calibration
  showMsg("STEP 2/2", "MAGNETOMETER", "", "Rotate slowly");
  delay(2000);
  runMagCalibration();
  
  // Calculate final calibration values
  calculateMagCalibration();
  
  // Save to NVS
  saveCalibration();
  
  // Show results
  char line1[32], line2[32], line3[32];
  snprintf(line1, sizeof(line1), "Quality: %d%%", g_cal.quality);
  snprintf(line2, sizeof(line2), "X:%.0f Y:%.0f Z:%.0f", 
           g_cal.magMax[0] - g_cal.magMin[0],
           g_cal.magMax[1] - g_cal.magMin[1],
           g_cal.magMax[2] - g_cal.magMin[2]);
  snprintf(line3, sizeof(line3), "Saved!");
  
  showMsg("CAL COMPLETE", line1, line2, line3);
  
  Serial.println(F("Calibration complete!"));
  Serial.printf("Quality: %d%%\n", g_cal.quality);
  
  delay(3000);
  
  g_calibrationMode = false;
  digitalWrite(LED_PIN, LOW);
}

/**
 * @brief Gyroscope calibration - average readings while stationary
 */
void runGyroCalibration(void) {
  showCalibrationScreen("GYRO CAL", "Keep sensor", "completely still!", nullptr, 0);
  delay(1000);
  
  long gyroSum[3] = {0, 0, 0};
  int validSamples = 0;
  
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (imu.dataReady()) {
      imu.getAGMT();
      gyroSum[0] += imu.agmt.gyr.axes.x;
      gyroSum[1] += imu.agmt.gyr.axes.y;
      gyroSum[2] += imu.agmt.gyr.axes.z;
      validSamples++;
    }
    
    // Update progress bar
    if (i % 50 == 0) {
      int progress = (i * 100) / GYRO_CAL_SAMPLES;
      showCalibrationScreen("GYRO CAL", "Keep sensor", "completely still!", nullptr, progress);
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
  
  showCalibrationScreen("GYRO CAL", "Complete!", nullptr, nullptr, 100);
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
  bool complete = false;
  
  showCalibrationScreen("MAG CAL", "Rotate slowly", "in all directions", "Press BTN=done", 0);
  
  while (!complete && (millis() - startTime < MAG_CAL_MAX_TIME_MS)) {
    // Check for button press to end early
    updateButton();
    if (wasButtonPressed() && (millis() - startTime > MAG_CAL_MIN_TIME_MS)) {
      complete = true;
      break;
    }
    
    // Read magnetometer
    if (imu.dataReady()) {
      imu.getAGMT();
      
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
      
      // Build status lines
      char line1[32], line2[32], line3[32];
      snprintf(line1, sizeof(line1), "X:%s Y:%s %ds", 
               xOk ? "OK" : "--", yOk ? "OK" : "--",
               (int)((MAG_CAL_MAX_TIME_MS - elapsed) / 1000));
      snprintf(line2, sizeof(line2), "R:%.0f/%.0f/%.0f", rangeX, rangeY, rangeZ);
      snprintf(line3, sizeof(line3), "N:%d BTN=done", g_sampleCount);
      
      showCalibrationScreen("MAG CAL", line1, line2, line3, progress);
      
      // Blink LED
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delay(10);
  }
  
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
 * Uses Cave Pearl Project method with improvements from Pololu
 */
void calculateMagCalibration(void) {
  // Calculate Hard Iron offsets (center of ellipsoid)
  for (int i = 0; i < 3; i++) {
    g_cal.magOffset[i] = (g_cal.magMax[i] + g_cal.magMin[i]) * 0.5f;
  }
  
  // Calculate delta (diameter) for each axis
  float delta[3];
  for (int i = 0; i < 3; i++) {
    delta[i] = g_cal.magMax[i] - g_cal.magMin[i];
  }
  
  // Calculate average diameter (for ideal sphere)
  float avgDelta = (delta[0] + delta[1] + delta[2]) / 3.0f;
  
  // Calculate Soft Iron scale factors
  // This scales each axis to match the average diameter
  for (int i = 0; i < 3; i++) {
    if (delta[i] > 1.0f) {
      g_cal.magScale[i] = avgDelta / delta[i];
    } else {
      g_cal.magScale[i] = 1.0f;
    }
  }
  
  // Calculate quality score based on:
  // 1. Range coverage (each axis should have good range)
  // 2. Sphericity (all axes should have similar range)
  // 3. Sample count
  
  float rangeScore = 0;
  if (delta[0] >= MAG_CAL_MIN_RANGE) rangeScore += 33;
  if (delta[1] >= MAG_CAL_MIN_RANGE) rangeScore += 33;
  if (delta[2] >= MAG_CAL_MIN_RANGE * 0.5f) rangeScore += 34;  // Z axis often has less range
  
  // Sphericity: ratio of smallest to largest delta
  float minDelta = min(delta[0], min(delta[1], delta[2]));
  float maxDelta = max(delta[0], max(delta[1], delta[2]));
  float sphericity = (maxDelta > 0) ? (minDelta / maxDelta) * 100 : 0;
  
  // Final quality is average of range and sphericity scores
  g_cal.quality = (uint8_t)((rangeScore + sphericity) / 2);
  
  Serial.println(F("Calibration calculated:"));
  Serial.printf("  Offset: [%.2f, %.2f, %.2f]\n", 
                g_cal.magOffset[0], g_cal.magOffset[1], g_cal.magOffset[2]);
  Serial.printf("  Scale: [%.4f, %.4f, %.4f]\n",
                g_cal.magScale[0], g_cal.magScale[1], g_cal.magScale[2]);
  Serial.printf("  Range score: %.0f, Sphericity: %.0f\n", rangeScore, sphericity);
  Serial.printf("  Quality: %d%%\n", g_cal.quality);
}

/**
 * @brief Apply magnetometer calibration (Hard Iron and Soft Iron correction)
 */
void applyMagCalibration(float raw[3], float calibrated[3]) {
  for (int i = 0; i < 3; i++) {
    // Hard Iron correction (remove offset)
    // Soft Iron correction (scale to sphere)
    calibrated[i] = (raw[i] - g_cal.magOffset[i]) * g_cal.magScale[i];
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
  
  // Calculate delta time
  unsigned long now = micros();
  float deltat = (now - g_ahrs.lastUpdate) * 1.0e-6f;
  g_ahrs.lastUpdate = now;
  
  // Limit delta time to reasonable values for AHRS stability
  // Max 20ms to prevent large integration errors
  if (deltat > 0.02f) deltat = 0.02f;
  if (deltat < 0.0001f) return;
  
  // Update Mahony AHRS
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2],
                         Gxyz[0], Gxyz[1], Gxyz[2],
                         McalXyz[0], McalXyz[1], McalXyz[2], deltat);
  
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
  
  // Title line
  display.setCursor(0, 0);
  display.print(title);
  
  // Messages (compact layout for 32-pixel height)
  if (line1) {
    display.setCursor(0, 10);
    display.print(line1);
  }
  if (line2) {
    // Show line2 next to line1 if space allows
    display.print(F(" "));
    display.print(line2);
  }
  
  // Progress bar (if >= 0) or line3
  if (progress >= 0) {
    int barY = 23;
    int barH = 7;
    int barW = SCREEN_WIDTH - 30;
    
    display.drawRect(0, barY, barW, barH, SSD1306_WHITE);
    int fillW = (progress * (barW - 2)) / 100;
    if (fillW > 0) {
      display.fillRect(1, barY + 1, fillW, barH - 2, SSD1306_WHITE);
    }
    
    // Show percentage
    display.setCursor(barW + 3, barY);
    display.print(progress);
    display.print(F("%"));
  } else if (line3) {
    display.setCursor(0, 22);
    display.print(line3);
  }
  
  display.display();
}

void showMsg(const char* l1, const char* l2, const char* l3, const char* l4) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Compact layout for 32-pixel height (3 lines max, 11 pixels spacing)
  if (l1) { display.setCursor(0, 0);  display.println(l1); }
  if (l2) { display.setCursor(0, 11); display.println(l2); }
  if (l3) { display.setCursor(0, 22); display.println(l3); }
  // l4 ignored on 32-pixel display - not enough space
  
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
