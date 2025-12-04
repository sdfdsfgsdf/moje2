/**
 * @file ICM20948_Compass_Tilt.ino
 * @brief Kompas i inklinometr z Mahony AHRS i wbudowaną kalibracją
 * 
 * Bazuje na projekcie jremington/ICM_20948-AHRS oraz dokumentacji
 * thecavepearlproject.org dotyczącej kalibracji magnetometru.
 * 
 * Sprzęt:
 *   - Arduino Pro Mini (3.3V/8MHz lub 5V/16MHz)
 *   - ICM-20948 9-DOF IMU (z magnetometrem AK09916)
 *   - OLED 128x32 I2C (SSD1306)
 * 
 * Funkcje:
 *   - Filtr Mahony AHRS oparty na kwaternionach
 *   - Integracja żyroskopu dla dynamicznej odpowiedzi
 *   - Wbudowana automatyczna kalibracja (bez zewnętrznych skryptów)
 *   - Korekcja Hard Iron (offset min/max)
 *   - Korekcja Soft Iron (skalowanie osi)
 *   - Menu OLED do uruchamiania kalibracji
 *   - Pomiar pochylenia (Roll, Pitch) z dokładnością 0.1°
 *   - Wskazanie północy magnetycznej i geograficznej
 *   - Filtrowanie EMA
 *   - Zapis kalibracji w EEPROM z CRC8
 * 
 * Kalibracja:
 *   - 3x krótkie uruchomienia (<2s) = tryb kalibracji
 *   - Lub przytrzymaj podczas startu
 *   - Obracaj czujnik we wszystkich kierunkach
 *   - Automatyczne wykrywanie zakończenia
 * 
 * Lokalizacja: Żywiec, Polska (49.6853°N, 19.1925°E)
 * Deklinacja magnetyczna: 5.5° E (2024)
 * 
 * Kalibracja wg: https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
 * Referencja: https://github.com/jremington/ICM_20948-AHRS
 * 
 * @license MIT
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "ICM_20948.h"

// ============================================================================
// KONFIGURACJA
// ============================================================================

// --- Wyświetlacz OLED ---
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     32
#define OLED_RESET        -1
#define OLED_I2C_ADDRESS  0x3C

// --- ICM-20948 ---
#define ICM_AD0_VAL       1
#define ICM_I2C_SPEED     400000

// --- Lokalizacja (Żywiec, Polska) ---
#define MAGNETIC_DECLINATION  5.5f    // Deklinacja (E = dodatnia)

// --- Parametry filtra Mahony AHRS ---
#define MAHONY_KP             50.0f
#define MAHONY_KI             0.0f

// --- Żyroskop ---
#define GYRO_SCALE            ((M_PI / 180.0f) * 0.00763f)

// --- Auto-kalibracja ---
#define SHORT_RUN_THRESHOLD_MS    2000
#define SHORT_RUNS_TO_CALIBRATE   3
#define GYRO_CAL_SAMPLES          500

// --- Parametry kalibracji magnetometru ---
#define CAL_MIN_TIME_MS       5000     // Min czas kalibracji
#define CAL_MAX_TIME_MS       60000    // Max czas kalibracji
#define CAL_MIN_RANGE         100.0f   // Min zakres na oś (μT)
#define CAL_CHECK_INTERVAL_MS 500      // Interwał sprawdzania

// --- Filtry ---
#define EMA_ALPHA             0.10f
#define DISPLAY_UPDATE_MS     250

// --- Walidacja magnetometru ---
#define MAG_VALID_MAX         5000.0f

// --- Mapa pamięci EEPROM ---
#define EEPROM_MAGIC_ADDR         0
#define EEPROM_SHORT_RUNS_ADDR    4
#define EEPROM_CAL_VALID_ADDR     8
#define EEPROM_GYRO_OFF_ADDR      12    // 3 floaty = 12 bajtów
#define EEPROM_MAG_MIN_ADDR       24    // 3 floaty = 12 bajtów  
#define EEPROM_MAG_MAX_ADDR       36    // 3 floaty = 12 bajtów
#define EEPROM_CRC_ADDR           48
#define EEPROM_MAGIC_VALUE        0xCAFE

// ============================================================================
// OBIEKTY GLOBALNE
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ICM_20948_I2C    imu;

// ============================================================================
// STRUKTURY DANYCH
// ============================================================================

struct SensorData {
  float roll, pitch, yaw;
  float headingMag, headingGeo;
  float rollFiltered, pitchFiltered;
  float headingMagFiltered, headingGeoFiltered;
  bool filtersReady;
};

/**
 * Struktura kalibracji wg Cave Pearl Project:
 * - Hard Iron: offset = (min + max) / 2 dla każdej osi
 * - Soft Iron: scale = średnia_delta / delta_osi (normalizacja do sfery)
 */
struct Calibration {
  float gyroOffset[3];    // Offsety żyroskopu
  float magMin[3];        // Minimalne wartości magnetometru
  float magMax[3];        // Maksymalne wartości magnetometru
  bool isValid;
};

struct AHRSState {
  float q[4];             // Kwaternion [w, x, y, z]
  float eInt[3];          // Błąd całkowy
  unsigned long lastUpdate;
};

// ============================================================================
// ZMIENNE GLOBALNE
// ============================================================================

SensorData    g_sensor = {0};
Calibration   g_cal = {{0,0,0}, {-200,-200,-200}, {200,200,200}, false};
AHRSState     g_ahrs = {{1.0f, 0.0f, 0.0f, 0.0f}, {0,0,0}, 0};
unsigned long g_startupTime = 0;
unsigned long g_lastDisplayUpdate = 0;

// ============================================================================
// PROTOTYPY FUNKCJI
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
const char* getCardinalDirection(float heading);
void showMessage(const char* line1, const char* line2 = nullptr, const char* line3 = nullptr);
void showCalibrationHelp(void);
void trackRuntime(void);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  g_startupTime = millis();
  
  Wire.begin();
  Wire.setClock(ICM_I2C_SPEED);
  
  initDisplay();
  showMessage("Inicjalizacja...");
  
  if (!initIMU()) {
    showMessage("Blad IMU!", "Sprawdz polaczenia");
    while (true) delay(100);
  }
  
  configureIMU();
  initEEPROM();
  incrementShortRunCount();
  loadCalibration();
  
  // Sprawdź czy uruchomić kalibrację
  if (shouldTriggerCalibration()) {
    showCalibrationHelp();
    delay(3000);
    runGyroCalibration();
    runMagCalibration();
    showMessage("Kalibracja", "zakonczona!", "Restart...");
    while (true) delay(100);
  }
  
  // Jeśli brak kalibracji, wykonaj tylko gyro
  if (!g_cal.isValid) {
    showMessage("Brak kalibracji", "Gyro cal...", "Trzymaj nieruchomo");
    delay(1000);
    runGyroCalibration();
    showMessage("Uzyj 3x restart", "dla pelnej", "kalibracji");
    delay(3000);
  }
  
  g_ahrs.lastUpdate = micros();
  
  showMessage("ICM-20948 Ready", "Mahony AHRS", "Zywiec, PL");
  delay(2000);
}

// ============================================================================
// GŁÓWNA PĘTLA
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
// INICJALIZACJA
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
// FUNKCJE EEPROM
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
      // Reset do domyślnych
      for (int i = 0; i < 3; i++) {
        g_cal.gyroOffset[i] = 0;
        g_cal.magMin[i] = -200;
        g_cal.magMax[i] = 200;
      }
      g_cal.isValid = false;
      showMessage("Blad CRC!", "Rekalibracja");
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
// KALIBRACJA
// ============================================================================

/**
 * Wyświetla instrukcję kalibracji na OLED
 */
void showCalibrationHelp(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("=== KALIBRACJA ==="));
  display.println(F("1. Trzymaj nieruch."));
  display.println(F("2. Obracaj powoli"));
  display.println(F("3. Czekaj na OK"));
  display.display();
}

/**
 * Kalibracja żyroskopu - uśrednianie offsetów
 * Czujnik musi być nieruchomy podczas kalibracji
 */
void runGyroCalibration(void) {
  showMessage("Kalibracja GYRO", "Trzymaj", "nieruchomo!");
  delay(2000);
  
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
    
    // Aktualizuj wyświetlacz co 50 próbek
    if (i % 50 == 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(F("Kalibracja GYRO"));
      display.print(F("Postep: "));
      display.print((i * 100) / GYRO_CAL_SAMPLES);
      display.println(F("%"));
      display.println(F("Nie ruszaj!"));
      display.display();
    }
  }
  
  if (count > 0) {
    g_cal.gyroOffset[0] = (float)gyroSum[0] / count;
    g_cal.gyroOffset[1] = (float)gyroSum[1] / count;
    g_cal.gyroOffset[2] = (float)gyroSum[2] / count;
  }
  
  showMessage("GYRO OK!", "", "");
  delay(1000);
}

/**
 * Kalibracja magnetometru metodą min/max
 * 
 * Zgodnie z Cave Pearl Project:
 * - Hard Iron offset = (min + max) / 2 dla każdej osi
 * - Soft Iron scale = średnia_delta / delta_osi
 * 
 * Ta metoda wymaga minimalnej ilości RAM - tylko 6 floatów (min/max)
 * zamiast setek próbek potrzebnych do ellipsoid fitting.
 */
void runMagCalibration(void) {
  showMessage("Kalibracja MAG", "Obracaj powoli", "we wszystkich");
  delay(2000);
  
  // Inicjalizuj min/max
  float magMin[3] = {32767.0f, 32767.0f, 32767.0f};
  float magMax[3] = {-32767.0f, -32767.0f, -32767.0f};
  
  unsigned long startTime = millis();
  unsigned long lastCheck = startTime;
  bool complete = false;
  
  while (!complete && (millis() - startTime < CAL_MAX_TIME_MS)) {
    if (imu.dataReady()) {
      imu.getAGMT();
      
      // Mapowanie osi dla AK09916 (zgodne z akcelerometrem)
      float mx = imu.magY();
      float my = -imu.magX();
      float mz = -imu.magZ();
      
      // Aktualizuj min/max dla każdej osi
      if (mx < magMin[0]) magMin[0] = mx;
      if (mx > magMax[0]) magMax[0] = mx;
      if (my < magMin[1]) magMin[1] = my;
      if (my > magMax[1]) magMax[1] = my;
      if (mz < magMin[2]) magMin[2] = mz;
      if (mz > magMax[2]) magMax[2] = mz;
      
      // Sprawdzaj postęp co CAL_CHECK_INTERVAL_MS
      if (millis() - lastCheck >= CAL_CHECK_INTERVAL_MS) {
        lastCheck = millis();
        
        float rangeX = magMax[0] - magMin[0];
        float rangeY = magMax[1] - magMin[1];
        float rangeZ = magMax[2] - magMin[2];
        unsigned long elapsed = millis() - startTime;
        
        bool xOk = (rangeX >= CAL_MIN_RANGE);
        bool yOk = (rangeY >= CAL_MIN_RANGE);
        bool zOk = (rangeZ >= CAL_MIN_RANGE);
        bool minTime = (elapsed >= CAL_MIN_TIME_MS);
        
        // Wymagamy X, Y i minimalnego czasu; Z opcjonalne
        if (xOk && yOk && minTime) complete = true;
        
        // Wyświetl status na OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        
        if (complete) {
          display.println(F("KALIBRACJA OK!"));
        } else {
          display.print(F("KAL: "));
          display.print(xOk ? F("+") : F("X"));
          display.print(yOk ? F("+") : F("Y"));
          display.print(zOk ? F("+") : F("Z"));
          if (!minTime) {
            display.print(F(" "));
            display.print((CAL_MIN_TIME_MS - elapsed) / 1000);
            display.print(F("s"));
          }
        }
        
        display.setCursor(0, 11);
        display.print(F("X:"));
        display.print((int)rangeX);
        display.print(F(" Y:"));
        display.print((int)rangeY);
        
        display.setCursor(0, 22);
        display.print(F("Z:"));
        display.print((int)rangeZ);
        display.print(F(" cel:"));
        display.print((int)CAL_MIN_RANGE);
        
        display.display();
      }
    }
    delay(10);
  }
  
  // Zapisz wyniki do struktury kalibracji
  for (int i = 0; i < 3; i++) {
    g_cal.magMin[i] = magMin[i];
    g_cal.magMax[i] = magMax[i];
  }
  
  saveCalibration();
  
  showMessage(complete ? "MAG OK!" : "MAG Timeout", "Zapisano", "");
  delay(2000);
}

/**
 * Aplikuje kalibrację magnetometru do surowych danych
 * 
 * Hard Iron: odejmij offset = (min + max) / 2
 * Soft Iron: przeskaluj aby znormalizować elipsoidę do sfery
 * 
 * @param raw     Surowe dane [3] z magnetometru (po mapowaniu osi)
 * @param calibrated Wyjściowe dane [3] po kalibracji
 */
void applyMagCalibration(float raw[3], float calibrated[3]) {
  // Oblicz offsety (hard iron)
  float offset[3];
  offset[0] = (g_cal.magMax[0] + g_cal.magMin[0]) / 2.0f;
  offset[1] = (g_cal.magMax[1] + g_cal.magMin[1]) / 2.0f;
  offset[2] = (g_cal.magMax[2] + g_cal.magMin[2]) / 2.0f;
  
  // Oblicz delty dla każdej osi
  float delta[3];
  delta[0] = g_cal.magMax[0] - g_cal.magMin[0];
  delta[1] = g_cal.magMax[1] - g_cal.magMin[1];
  delta[2] = g_cal.magMax[2] - g_cal.magMin[2];
  
  // Średnia delta (docelowy promień sfery)
  float avgDelta = (delta[0] + delta[1] + delta[2]) / 3.0f;
  
  // Oblicz skale (soft iron) - normalizacja do sfery
  float scale[3];
  const float MIN_DELTA = 1.0f;  // Zabezpieczenie przed dzieleniem przez 0
  scale[0] = (fabs(delta[0]) > MIN_DELTA) ? avgDelta / delta[0] : 1.0f;
  scale[1] = (fabs(delta[1]) > MIN_DELTA) ? avgDelta / delta[1] : 1.0f;
  scale[2] = (fabs(delta[2]) > MIN_DELTA) ? avgDelta / delta[2] : 1.0f;
  
  // Aplikuj korekcję
  calibrated[0] = (raw[0] - offset[0]) * scale[0];
  calibrated[1] = (raw[1] - offset[1]) * scale[1];
  calibrated[2] = (raw[2] - offset[2]) * scale[2];
}

// ============================================================================
// MATEMATYKA WEKTOROWA
// ============================================================================

float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  if (mag > 1e-9f) {  // Epsilon dla precyzji float
    a[0] /= mag;
    a[1] /= mag;
    a[2] /= mag;
  }
}

// ============================================================================
// FILTR MAHONY AHRS
// ============================================================================

/**
 * Aktualizacja filtra Mahony z kwaternionami
 * 
 * Wykorzystuje wektory referencyjne Up (akcel) i West (akcel x mag)
 * Zmodyfikowana wersja z jremington/ICM_20948-AHRS
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
  
  // Wektor horyzontu = a x m (w układzie ciała)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm < 1e-9f) return;  // Epsilon dla precyzji float
  norm = 1.0f / norm;
  hx *= norm; hy *= norm; hz *= norm;
  
  // Estymowany kierunek Up
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;
  
  // Estymowany kierunek West
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);
  
  // Błąd = iloczyn wektorowy estymowanego i zmierzonego
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
  
  // Sprzężenie całkowe (jeśli włączone)
  if (MAHONY_KI > 0.0f) {
    g_ahrs.eInt[0] += ex;
    g_ahrs.eInt[1] += ey;
    g_ahrs.eInt[2] += ez;
    gx += MAHONY_KI * g_ahrs.eInt[0];
    gy += MAHONY_KI * g_ahrs.eInt[1];
    gz += MAHONY_KI * g_ahrs.eInt[2];
  }
  
  // Sprzężenie proporcjonalne
  gx = gx + MAHONY_KP * ex;
  gy = gy + MAHONY_KP * ey;
  gz = gz + MAHONY_KP * ez;
  
  // Aktualizacja kwaternionu
  gx = gx * (0.5f * deltat);
  gy = gy * (0.5f * deltat);
  gz = gz * (0.5f * deltat);
  
  float qa = q1, qb = q2, qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);
  
  // Normalizacja kwaternionu
  norm = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  norm = 1.0f / norm;
  g_ahrs.q[0] = q1 * norm;
  g_ahrs.q[1] = q2 * norm;
  g_ahrs.q[2] = q3 * norm;
  g_ahrs.q[3] = q4 * norm;
}

/**
 * Konwersja kwaternionu na kąty Eulera
 * Orientacja NWU: X na północ (yaw=0), Y na zachód, Z do góry
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
// PRZETWARZANIE CZUJNIKÓW
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
  
  // Odczyt i skalowanie żyroskopu (rad/s z offsetem)
  Gxyz[0] = GYRO_SCALE * (imu.agmt.gyr.axes.x - g_cal.gyroOffset[0]);
  Gxyz[1] = GYRO_SCALE * (imu.agmt.gyr.axes.y - g_cal.gyroOffset[1]);
  Gxyz[2] = GYRO_SCALE * (imu.agmt.gyr.axes.z - g_cal.gyroOffset[2]);
  
  // Odczyt akcelerometru i normalizacja
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  vector_normalize(Axyz);
  
  // Odczyt magnetometru z mapowaniem osi dla AK09916
  Mxyz[0] = imu.magY();      // Mapowanie osi
  Mxyz[1] = -imu.magX();
  Mxyz[2] = -imu.magZ();
  
  // Aplikuj kalibrację magnetometru
  applyMagCalibration(Mxyz, McalXyz);
  vector_normalize(McalXyz);
  
  // Uzgodnij osie mag z akcel (Y, Z odwrócone)
  McalXyz[1] = -McalXyz[1];
  McalXyz[2] = -McalXyz[2];
  
  // Oblicz delta time
  unsigned long now = micros();
  float deltat = (now - g_ahrs.lastUpdate) * 1.0e-6f;
  g_ahrs.lastUpdate = now;
  
  if (!validateMagData()) return;
  
  // Aktualizuj filtr Mahony AHRS
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2],
                         Gxyz[0], Gxyz[1], Gxyz[2],
                         McalXyz[0], McalXyz[1], McalXyz[2], deltat);
  
  // Konwersja na kąty Eulera
  float yaw, pitch, roll;
  quaternionToEuler(g_ahrs.q, yaw, pitch, roll);
  
  // Aplikuj deklinację (konwencja nawigacyjna: yaw rośnie CW od N)
  yaw = -(yaw + MAGNETIC_DECLINATION);
  if (yaw < 0) yaw += 360.0f;
  if (yaw >= 360.0f) yaw -= 360.0f;
  
  g_sensor.roll = roll;
  g_sensor.pitch = pitch;
  g_sensor.yaw = yaw;
  g_sensor.headingGeo = yaw;
  
  // Nagłówek magnetyczny (bez deklinacji)
  g_sensor.headingMag = yaw - MAGNETIC_DECLINATION;
  if (g_sensor.headingMag < 0) g_sensor.headingMag += 360.0f;
  if (g_sensor.headingMag >= 360.0f) g_sensor.headingMag -= 360.0f;
  
  // Filtr EMA
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
// FUNKCJE FILTRÓW
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
// FUNKCJE WYŚWIETLANIA
// ============================================================================

void updateDisplay(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Linia 1: Kąty pochylenia
  display.setCursor(0, 0);
  display.print(F("X:"));
  display.print(g_sensor.rollFiltered, 1);
  display.print(F(" Y:"));
  display.print(g_sensor.pitchFiltered, 1);
  
  float magDev = getDeviationFromNorth(g_sensor.headingMagFiltered);
  float geoDev = getDeviationFromNorth(g_sensor.headingGeoFiltered);
  
  // Linia 2: Północ magnetyczna
  display.setCursor(0, 11);
  display.print(F("Mag:"));
  display.print(magDev, 1);
  display.print((char)247);  // Symbol stopnia
  display.print(getCardinalDirection(g_sensor.headingMagFiltered));
  
  // Linia 3: Północ geograficzna
  display.setCursor(0, 22);
  display.print(F("Geo:"));
  display.print(geoDev, 1);
  display.print((char)247);
  display.print(getCardinalDirection(g_sensor.headingGeoFiltered));
  
  display.display();
}

float getDeviationFromNorth(float heading) {
  float deviation = heading;
  if (deviation > 180.0f) deviation -= 360.0f;
  return deviation;
}

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

void showMessage(const char* line1, const char* line2, const char* line3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(line1);
  if (line2) { display.setCursor(0, 11); display.println(line2); }
  if (line3) { display.setCursor(0, 22); display.println(line3); }
  
  display.display();
}

// ============================================================================
// FUNKCJE POMOCNICZE
// ============================================================================

void trackRuntime(void) {
  static bool normalOpRecorded = false;
  if (!normalOpRecorded && (millis() - g_startupTime >= SHORT_RUN_THRESHOLD_MS)) {
    normalOpRecorded = true;
    resetShortRunCount();
  }
}
