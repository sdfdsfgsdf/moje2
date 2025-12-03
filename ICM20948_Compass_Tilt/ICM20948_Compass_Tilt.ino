/*
 * ICM-20948 Compass & Tilt Meter for Arduino Mini
 * 
 * ============================================
 * HARDWARE REQUIREMENTS
 * ============================================
 * - Arduino Mini (Pro Mini compatible, 3.3V or 5V)
 * - ICM-20948 9-DOF IMU (I2C) - accelerometer, gyroscope, magnetometer
 * - OLED 128x32 display (I2C, SSD1306 controller)
 * - Push button for calibration (normally open)
 * - 10kΩ pull-up resistor for button (optional if using INPUT_PULLUP)
 * 
 * ============================================
 * PIN CONNECTIONS / WYPROWADZENIE PINÓW
 * ============================================
 * 
 * ARDUINO MINI PRO (3.3V/5V)
 * ┌─────────────────────────────────────────┐
 * │                                         │
 * │  [RAW] ─── Zasilanie 5-12V             │
 * │  [VCC] ─── Zasilanie regulowane 3.3/5V │
 * │  [GND] ─── Masa                         │
 * │                                         │
 * │  [A4/SDA] ─── I2C Data                 │
 * │  [A5/SCL] ─── I2C Clock                │
 * │                                         │
 * │  [D2] ─── Przycisk kalibracji          │
 * │                                         │
 * └─────────────────────────────────────────┘
 * 
 * ICM-20948 (Czujnik 9-DOF)
 * ┌─────────────────────────────────────────┐
 * │  VCC ────── Arduino VCC (3.3V!)        │
 * │  GND ────── Arduino GND                │
 * │  SDA ────── Arduino A4                 │
 * │  SCL ────── Arduino A5                 │
 * │  AD0 ────── GND (adres 0x68)           │
 * │         lub VCC (adres 0x69)           │
 * │  INT ────── (opcjonalnie) Arduino D3   │
 * └─────────────────────────────────────────┘
 * UWAGA: ICM-20948 wymaga zasilania 3.3V!
 *        Dla Arduino 5V użyj konwertera poziomów
 *        lub modułu z wbudowanym regulatorem.
 * 
 * OLED 128x32 (SSD1306)
 * ┌─────────────────────────────────────────┐
 * │  VCC ────── Arduino VCC (3.3V lub 5V)  │
 * │  GND ────── Arduino GND                │
 * │  SDA ────── Arduino A4                 │
 * │  SCL ────── Arduino A5                 │
 * └─────────────────────────────────────────┘
 * 
 * PRZYCISK KALIBRACJI
 * ┌─────────────────────────────────────────┐
 * │                                         │
 * │  Arduino D2 ───┬─── Przycisk ─── GND   │
 * │                │                        │
 * │                └─── (wewnętrzny pullup) │
 * │                                         │
 * │  Przytrzymaj 3 sekundy = kalibracja    │
 * │  Krótkie naciśnięcie = zmiana trybu    │
 * └─────────────────────────────────────────┘
 * 
 * SCHEMAT POŁĄCZEŃ (ASCII)
 * ========================
 * 
 *                    +3.3V
 *                      │
 *          ┌───────────┼───────────┐
 *          │           │           │
 *       [VCC]       [VCC]       [VCC]
 *     ICM-20948    OLED 128x32  Arduino
 *       [GND]       [GND]       [GND]
 *          │           │           │
 *          └───────────┴───────────┴──── GND
 * 
 *     ICM-20948                Arduino Mini
 *       [SDA] ─────────────────── [A4]
 *       [SCL] ─────────────────── [A5]
 * 
 *     OLED 128x32              Arduino Mini
 *       [SDA] ─────────────────── [A4]
 *       [SCL] ─────────────────── [A5]
 * 
 *     Przycisk                 Arduino Mini
 *       [Pin1] ────────────────── [D2]
 *       [Pin2] ────────────────── [GND]
 * 
 * ============================================
 * FEATURES / FUNKCJE
 * ============================================
 * - Pomiar pochylenia na wszystkich 3 osiach (Roll, Pitch, Yaw)
 * - Wskazanie północy magnetycznej
 * - Wskazanie północy geograficznej (skorygowane dla Żywca)
 * - Kalibracja magnetometru (przytrzymaj przycisk 3 sek)
 * - Zmiana trybu wyświetlania (krótkie naciśnięcie)
 * 
 * ============================================
 * LOCATION DATA / DANE LOKALIZACYJNE
 * ============================================
 * Żywiec, Poland coordinates: 49.6853°N, 19.1925°E
 * Magnetic declination: ~5.5°E (as of 2024)
 * 
 * ============================================
 * REQUIRED LIBRARIES / WYMAGANE BIBLIOTEKI
 * ============================================
 * - Adafruit SSD1306 (Manager bibliotek Arduino)
 * - Adafruit GFX (Manager bibliotek Arduino)
 * - SparkFun ICM-20948 (Manager bibliotek Arduino)
 * - Wire (wbudowana)
 * 
 * Author: Arduino Project
 * License: MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

// Button settings / Ustawienia przycisku
#define BUTTON_PIN 2              // Pin przycisku (D2)
#define LONG_PRESS_TIME 3000      // Czas długiego naciśnięcia (3 sekundy) dla kalibracji
#define DEBOUNCE_TIME 50          // Czas debouncingu (ms)

// Magnetic declination for Żywiec, Poland (49.6853°N, 19.1925°E)
// Declination is approximately 5.5° East (positive value)
// Update this value periodically as it changes over time
// Source: NOAA World Magnetic Model
#define MAGNETIC_DECLINATION 5.5

// Display update interval (ms)
#define DISPLAY_UPDATE_INTERVAL 100

// Calibration settings
#define CALIBRATION_SAMPLES 100

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

// Magnetometer calibration offsets (hard iron)
float magOffsetX = 0;
float magOffsetY = 0;
float magOffsetZ = 0;

// Magnetometer calibration scale (soft iron)
float magScaleX = 1.0;
float magScaleY = 1.0;
float magScaleZ = 1.0;

// Display mode (0: Tilt, 1: Compass, 2: All data)
int displayMode = 0;
unsigned long lastDisplayUpdate = 0;

// Button state variables / Zmienne stanu przycisku
bool buttonState = HIGH;           // Aktualny stan przycisku (HIGH = nie naciśnięty)
bool lastButtonState = HIGH;       // Poprzedni stan przycisku
unsigned long buttonPressTime = 0; // Czas naciśnięcia przycisku
unsigned long lastDebounceTime = 0;// Czas ostatniego debouncingu
bool longPressTriggered = false;   // Czy długie naciśnięcie zostało już obsłużone

// ============================================
// SETUP
// ============================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial (timeout 3s)
  
  Serial.println(F("ICM-20948 Compass & Tilt Meter"));
  Serial.println(F("Location: Żywiec, Poland"));
  Serial.println(F("Magnetic Declination: 5.5° East"));
  Serial.println();
  
  // Initialize button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed!"));
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
  // Try default address first
  imu.begin(Wire, 1); // AD0 = 1 -> address 0x69
  
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("Trying alternate I2C address..."));
    imu.begin(Wire, 0); // AD0 = 0 -> address 0x68
  }
  
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.print(F("ICM-20948 init failed! Status: "));
    Serial.println(imu.statusString());
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("IMU Error!"));
    display.println(F("Check wiring"));
    display.display();
    while (1) {
      delay(100);
    }
  }
  
  Serial.println(F("ICM-20948 connected!"));
  
  // Configure IMU
  configureIMU();
  
  // Show startup message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("ICM-20948 Ready"));
  display.println(F("Zywiec, Poland"));
  display.println(F("Decl: 5.5E"));
  display.display();
  delay(2000);
  
  // Optional: Run magnetometer calibration
  // Uncomment the following line to calibrate on startup
  // calibrateMagnetometer();
  
  Serial.println(F("Setup complete!"));
  Serial.println(F("Commands: 'c' = calibrate, 'm' = change mode"));
  Serial.println(F("Button: short press = change mode, hold 3s = calibrate"));
  Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
  // Check for button press (calibration and mode change)
  checkButton();
  
  // Check for serial commands
  checkSerialCommands();
  
  // Read sensor data
  if (imu.dataReady()) {
    imu.getAGMT(); // Get Accel, Gyro, Mag, Temp
    
    // Calculate tilt angles from accelerometer
    calculateTilt();
    
    // Calculate compass heading from magnetometer
    calculateHeading();
  }
  
  // Update display at fixed interval
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = millis();
    updateDisplay();
    
    // Print to serial for debugging
    printDataSerial();
  }
}

// ============================================
// SENSOR CONFIGURATION
// ============================================

void configureIMU() {
  // Reset IMU to known state
  imu.swReset();
  delay(250);
  
  // Wake up IMU
  imu.sleep(false);
  imu.lowPower(false);
  
  // Set sample mode for continuous reading
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  
  // Set full scale ranges
  ICM_20948_fss_t fss;
  fss.a = gpm2;   // Accelerometer: ±2g
  fss.g = dps250; // Gyroscope: ±250 dps
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  
  // Set digital low-pass filter
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d50bw4_n68bw8;  // Accel: 50Hz bandwidth
  dlpcfg.g = gyr_d51bw2_n73bw3; // Gyro: 51Hz bandwidth
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  
  // Enable DLPF
  imu.enableDLPF(ICM_20948_Internal_Acc, true);
  imu.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  // Start magnetometer
  imu.startupMagnetometer();
  
  Serial.println(F("IMU configured"));
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
  Serial.println(F("Starting magnetometer calibration..."));
  Serial.println(F("Rotate the sensor in all directions for 30 seconds"));
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("CALIBRATION"));
  display.println(F("Rotate sensor"));
  display.println(F("in all directions"));
  display.display();
  
  float magMin[3] = {32767, 32767, 32767};
  float magMax[3] = {-32767, -32767, -32767};
  
  unsigned long startTime = millis();
  unsigned long duration = 30000; // 30 seconds
  
  while (millis() - startTime < duration) {
    if (imu.dataReady()) {
      imu.getAGMT();
      
      float mx = imu.magX();
      float my = imu.magY();
      float mz = imu.magZ();
      
      // Track min/max values
      if (mx < magMin[0]) magMin[0] = mx;
      if (mx > magMax[0]) magMax[0] = mx;
      if (my < magMin[1]) magMin[1] = my;
      if (my > magMax[1]) magMax[1] = my;
      if (mz < magMin[2]) magMin[2] = mz;
      if (mz > magMax[2]) magMax[2] = mz;
      
      // Update progress on display
      int progress = (millis() - startTime) * 100 / duration;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("CALIBRATING..."));
      display.print(F("Progress: "));
      display.print(progress);
      display.println(F("%"));
      display.display();
    }
    delay(50);
  }
  
  // Calculate hard iron offsets (center of min/max)
  magOffsetX = (magMax[0] + magMin[0]) / 2.0;
  magOffsetY = (magMax[1] + magMin[1]) / 2.0;
  magOffsetZ = (magMax[2] + magMin[2]) / 2.0;
  
  // Calculate soft iron scale factors
  // If axis delta is too small (sensor wasn't rotated properly on that axis),
  // use default scale of 1.0 to avoid division by zero or unstable results.
  // MIN_DELTA is a small value to handle floating-point precision issues.
  const float MIN_DELTA = 1.0;  // Minimum delta value (1.0 uT)
  
  float deltaX = magMax[0] - magMin[0];
  float deltaY = magMax[1] - magMin[1];
  float deltaZ = magMax[2] - magMin[2];
  
  // Use fabs() for reliable floating-point comparison and ensure positive deltas
  if (fabs(deltaX) < MIN_DELTA) deltaX = MIN_DELTA;
  if (fabs(deltaY) < MIN_DELTA) deltaY = MIN_DELTA;
  if (fabs(deltaZ) < MIN_DELTA) deltaZ = MIN_DELTA;
  
  float avgDelta = (deltaX + deltaY + deltaZ) / 3.0;
  
  magScaleX = avgDelta / deltaX;
  magScaleY = avgDelta / deltaY;
  magScaleZ = avgDelta / deltaZ;
  
  // Print calibration results
  Serial.println(F("Calibration complete!"));
  Serial.print(F("Offsets - X: ")); Serial.print(magOffsetX);
  Serial.print(F(" Y: ")); Serial.print(magOffsetY);
  Serial.print(F(" Z: ")); Serial.println(magOffsetZ);
  Serial.print(F("Scale - X: ")); Serial.print(magScaleX);
  Serial.print(F(" Y: ")); Serial.print(magScaleY);
  Serial.print(F(" Z: ")); Serial.println(magScaleZ);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Calibration Done!"));
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
  
  switch (displayMode) {
    case 0:
      displayTiltMode();
      break;
    case 1:
      displayCompassMode();
      break;
    case 2:
      displayAllDataMode();
      break;
  }
  
  display.display();
}

void displayTiltMode() {
  // Title
  display.setCursor(0, 0);
  display.println(F("TILT (degrees)"));
  
  // Roll
  display.setCursor(0, 10);
  display.print(F("Roll:  "));
  display.print(roll, 1);
  display.print((char)247); // Degree symbol
  
  // Pitch
  display.setCursor(0, 20);
  display.print(F("Pitch: "));
  display.print(pitch, 1);
  display.print((char)247);
  
  // Small compass indicator on right side
  display.setCursor(90, 10);
  display.print(F("N:"));
  display.print((int)headingTrue);
}

void displayCompassMode() {
  // Title with location
  display.setCursor(0, 0);
  display.println(F("COMPASS - Zywiec,PL"));
  
  // Magnetic heading
  display.setCursor(0, 10);
  display.print(F("Mag:  "));
  display.print(headingMag, 1);
  display.print((char)247);
  display.print(F(" "));
  display.print(getCardinalDirection(headingMag));
  
  // True heading
  display.setCursor(0, 20);
  display.print(F("True: "));
  display.print(headingTrue, 1);
  display.print((char)247);
  display.print(F(" "));
  display.print(getCardinalDirection(headingTrue));
}

void displayAllDataMode() {
  // Compact display with all data
  display.setCursor(0, 0);
  display.print(F("R:"));
  display.print(roll, 0);
  display.print(F(" P:"));
  display.print(pitch, 0);
  display.print(F(" Y:"));
  display.print(yaw, 0);
  
  display.setCursor(0, 10);
  display.print(F("Mag N: "));
  display.print(headingMag, 1);
  display.print((char)247);
  
  display.setCursor(0, 20);
  display.print(F("True N:"));
  display.print(headingTrue, 1);
  display.print((char)247);
  display.print(F(" "));
  display.print(getCardinalDirection(headingTrue));
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

// ============================================
// BUTTON HANDLING / OBSŁUGA PRZYCISKU
// ============================================

void checkButton() {
  // Read current button state (LOW = pressed due to INPUT_PULLUP)
  bool reading = digitalRead(BUTTON_PIN);
  
  // Check if button state changed (for debouncing)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed, accept the new state
  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
    // If button state has actually changed
    if (reading != buttonState) {
      buttonState = reading;
      
      // Button was just pressed (LOW = pressed)
      if (buttonState == LOW) {
        buttonPressTime = millis();
        longPressTriggered = false;
        
        // Show visual feedback on display
        display.clearDisplay();
        display.setCursor(0, 10);
        display.println(F("Button pressed..."));
        display.println(F("Hold 3s to calibrate"));
        display.display();
      }
      // Button was just released (HIGH = released)
      else {
        unsigned long pressDuration = millis() - buttonPressTime;
        
        // If it wasn't a long press, treat as short press (change mode)
        if (!longPressTriggered && pressDuration < LONG_PRESS_TIME) {
          // Short press - change display mode
          displayMode = (displayMode + 1) % 3;
          Serial.print(F("Display mode changed to: "));
          Serial.println(displayMode);
        }
      }
    }
    
    // Check for long press while button is still held
    if (buttonState == LOW && !longPressTriggered) {
      unsigned long pressDuration = millis() - buttonPressTime;
      
      // Show countdown on display
      if (pressDuration > 500) { // Start showing after 0.5s
        int secondsRemaining = (LONG_PRESS_TIME - pressDuration) / 1000 + 1;
        if (secondsRemaining > 0 && secondsRemaining <= 3) {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println(F("CALIBRATION IN:"));
          display.setTextSize(2);
          display.setCursor(50, 15);
          display.print(secondsRemaining);
          display.setTextSize(1);
          display.display();
        }
      }
      
      // Long press detected - start calibration
      if (pressDuration >= LONG_PRESS_TIME) {
        longPressTriggered = true;
        Serial.println(F("Long press detected - starting calibration"));
        calibrateMagnetometer();
      }
    }
  }
  
  lastButtonState = reading;
}

void checkSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'c':
      case 'C':
        calibrateMagnetometer();
        break;
      case 'm':
      case 'M':
        displayMode = (displayMode + 1) % 3;
        Serial.print(F("Display mode: "));
        Serial.println(displayMode);
        break;
      case 'r':
      case 'R':
        // Reset calibration
        magOffsetX = 0;
        magOffsetY = 0;
        magOffsetZ = 0;
        magScaleX = 1.0;
        magScaleY = 1.0;
        magScaleZ = 1.0;
        Serial.println(F("Calibration reset"));
        break;
      case '?':
      case 'h':
      case 'H':
        printHelp();
        break;
    }
  }
}

void printHelp() {
  Serial.println(F("\n=== ICM-20948 Compass & Tilt Meter ==="));
  Serial.println(F("Serial Commands:"));
  Serial.println(F("  c - Calibrate magnetometer"));
  Serial.println(F("  m - Change display mode"));
  Serial.println(F("  r - Reset calibration"));
  Serial.println(F("  h - Show this help"));
  Serial.println(F("\nButton (D2):"));
  Serial.println(F("  Short press - Change display mode"));
  Serial.println(F("  Hold 3 sec  - Start calibration"));
  Serial.println(F("\nLocation: Żywiec, Poland"));
  Serial.println(F("Coordinates: 49.6853°N, 19.1925°E"));
  Serial.println(F("Magnetic Declination: 5.5° East"));
  Serial.println();
}

void printDataSerial() {
  Serial.print(F("Roll: "));
  Serial.print(roll, 1);
  Serial.print(F("° | Pitch: "));
  Serial.print(pitch, 1);
  Serial.print(F("° | Mag Heading: "));
  Serial.print(headingMag, 1);
  Serial.print(F("° | True Heading: "));
  Serial.print(headingTrue, 1);
  Serial.print(F("° ("));
  Serial.print(getCardinalDirection(headingTrue));
  Serial.println(F(")"));
}
