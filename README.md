# ICM-20948 Compass & Tilt Meter

Projekt do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej z filtrem Mahony AHRS dla **ESP32-WROOM-32D**.

📁 `ICM20948_Compass_Tilt_ESP32/` - Wersja dla **ESP32-WROOM-32D**

## Sprzęt / Hardware

- **ESP32-WROOM-32D** (lub dowolny ESP32 DevKit)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr) - **połączony przez SPI**
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)
- **Przycisk** - do wyzwalania kalibracji (NO, normalnie otwarty)

## Funkcje / Features

### Podstawowe
- ✅ **Filtr Mahony AHRS** - dokładne śledzenie orientacji z kwaternionami
- ✅ **Integracja żyroskopu** - płynna odpowiedź dynamiczna
- ✅ Pomiar pochylenia na 2 osiach (Roll, Pitch) z dokładnością 0.1°
- ✅ Wskazanie północy magnetycznej i geograficznej
- ✅ Filtrowanie EMA z obsługą przejścia 0°/360°
- ✅ Wskaźnik kompasu graficzny na OLED

### Komunikacja
- ✅ **SPI dla IMU** - szybsza i bardziej niezawodna komunikacja z ICM-20948 (4 MHz)
- ✅ **I2C dla OLED** - standardowy interfejs dla wyświetlacza

### Kalibracja na urządzeniu (bez zewnętrznego oprogramowania!)
- ✅ **Jeden przycisk** - przytrzymaj 2s aby rozpocząć kalibrację
- ✅ **Przewodnik na OLED** - instrukcje krok po kroku na ekranie
- ✅ **Pasek postępu** - wizualizacja postępu kalibracji
- ✅ **Automatyczne przetwarzanie** - obliczenia na ESP32, bez komputera!
- ✅ **Wskaźnik jakości** - ocena jakości kalibracji (0-100%)

### Korekcja magnetometru
- ✅ Korekcja Hard Iron (przesunięcie środka elipsoidy)
- ✅ Korekcja Soft Iron (skalowanie osi do sfery) z pełną dekompozycją własną (Jacobi)
- ✅ Zbieranie do 500 próbek dla lepszej dokładności
- ✅ Automatyczne wykrywanie zakończenia kalibracji

### Optymalizacje ESP32
- ✅ Wykorzystanie jednostki FPU dla szybkich obliczeń float
- ✅ Zapis kalibracji w NVS (Preferences) zamiast EEPROM
- ✅ Szybsze próbkowanie (500-1000 próbek żyroskopu z odrzucaniem outlierów)
- ✅ Wyświetlacz OLED 128x32 z kompaktowym UI
- ✅ Aktualizacja wyświetlacza 10Hz

### Stabilność i niezawodność
- ✅ **Watchdog Timer** - automatyczny restart przy zawieszeniu programu
- ✅ **I2C Bus Recovery** - odzyskiwanie magistrali I2C po błędach (dla OLED)
- ✅ **I2C Timeout** - zabezpieczenie przed nieskończonym oczekiwaniem (15ms)
- ✅ **Quaternion Validation** - reset AHRS przy nieprawidłowych stanach
- ✅ **NaN Detection** - ochrona przed błędnymi odczytami czujników
- ✅ **micros() Overflow Handling** - poprawna obsługa przelewania licznika

## Dane lokalizacyjne / Location Data

| Parametr | Wartość |
|----------|---------|
| Miasto | Żywiec, Polska |
| Szerokość geograficzna | 49.6853°N |
| Długość geograficzna | 19.1925°E |
| Deklinacja magnetyczna | 5.5° E (2024) |

## Wyprowadzenie pinów ESP32-WROOM-32D

### Schemat pinów

```
ESP32-WROOM-32D
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  EN ─┐                                          ┌─ GPIO23 ──── SPI MOSI (IMU)
│ VP36 ─┤    ┌──────────────────────────────┐    ├─ GPIO22 ──── SCL (I2C OLED)
│ VN39 ─┤    │                              │    ├─ TX0       │
│  D34 ─┤    │         ESP32-WROOM-32D      │    ├─ RX0       │
│  D35 ─┤    │                              │    ├─ GPIO21 ──── SDA (I2C OLED)
│  D32 ─┤    │          ┌─────────┐         │    ├─ GND       │
│  D33 ─┤    │          │  ESP32  │         │    ├─ GPIO19 ──── SPI MISO (IMU)
│  D25 ─┤    │          │  CHIP   │         │    ├─ GPIO18 ──── SPI SCK (IMU)
│  D26 ─┤    │          └─────────┘         │    ├─ GPIO5 ───── SPI CS (IMU)
│  D27 ─┤    │                              │    ├─ GPIO17    │
│  D14 ─┤    │                              │    ├─ GPIO16    │
│  D12 ─┤    │                              │    ├─ GPIO4 ───── INT (IMU, opcj.)
│  D13 ─┤    └──────────────────────────────┘    ├─ GPIO0     │
│  GND ─┤                                        ├─ GPIO2 ────── LED (wbudowana)
│  VIN ─┤                                        ├─ GPIO15 ───── BUTTON
│ 3V3 ──┤                                        ├─ GND       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Przypisanie pinów

| Funkcja | Pin ESP32 | Opis |
|---------|-----------|------|
| **SPI MOSI** | GPIO23 | Master Out Slave In (do ICM-20948) |
| **SPI MISO** | GPIO19 | Master In Slave Out (z ICM-20948) |
| **SPI SCK** | GPIO18 | Zegar SPI (do ICM-20948) |
| **SPI CS** | GPIO5 | Chip Select dla ICM-20948 |
| **I2C SDA** | GPIO21 | Dane I2C (do OLED) |
| **I2C SCL** | GPIO22 | Zegar I2C (do OLED) |
| **INT** | GPIO4 | Przerwanie z ICM-20948 (opcjonalne) |
| **BUTTON** | GPIO15 | Przycisk kalibracji (do GND) |
| **LED** | GPIO2 | Wbudowana dioda LED (status) |
| **VIN** | VIN | Zasilanie 5V (z USB lub zewn.) |
| **3V3** | 3V3 | Zasilanie 3.3V dla czujników |
| **GND** | GND | Masa |

### Połączenia ICM-20948 (SPI)

Moduł ICM-20948 komunikuje się przez interfejs **SPI** (Serial Peripheral Interface) z prędkością 4 MHz. SPI zapewnia szybszą i bardziej niezawodną komunikację niż I2C, co jest szczególnie ważne dla aplikacji wymagających wysokiej częstotliwości próbkowania.

#### Schemat połączeń SPI

```
ICM-20948 Breakout          ESP32-WROOM-32D
┌────────────────┐         ┌────────────────┐
│                │         │                │
│  VCC (3.3V)    │─────────│ 3V3            │
│  GND           │─────────│ GND            │
│                │         │                │
│  SDA (MOSI)    │─────────│ GPIO23 (VSPI)  │  ← Dane DO czujnika
│  SCL (SCLK)    │─────────│ GPIO18 (VSPI)  │  ← Zegar SPI
│  AD0 (MISO)    │─────────│ GPIO19 (VSPI)  │  ← Dane Z czujnika
│  NCS (CS)      │─────────│ GPIO5          │  ← Chip Select (aktywny LOW)
│                │         │                │
│  INT           │─────────│ GPIO4          │  (opcjonalnie, dla przerwań)
│                │         │                │
└────────────────┘         └────────────────┘
```

#### Mapowanie pinów ICM-20948 (I2C vs SPI)

| Pin na module | Tryb I2C | Tryb SPI | Opis |
|---------------|----------|----------|------|
| **VCC** | Zasilanie 3.3V | Zasilanie 3.3V | Zasilanie modułu |
| **GND** | Masa | Masa | Masa |
| **SDA** | Dane I2C | **MOSI** | Master Out Slave In |
| **SCL** | Zegar I2C | **SCLK** | Zegar SPI |
| **AD0** | Adres I2C (LSB) | **MISO** | Master In Slave Out |
| **NCS** | Nie używany | **CS** | Chip Select (aktywny LOW) |
| **INT** | Przerwanie | Przerwanie | Opcjonalne |

#### Parametry SPI

| Parametr | Wartość | Opis |
|----------|---------|------|
| Prędkość zegara | 4 MHz | Maksymalnie ICM-20948 wspiera 7 MHz |
| Tryb SPI | Mode 0 | CPOL=0, CPHA=0 |
| Kolejność bitów | MSB first | Najpierw najbardziej znaczący bit |
| Magistrala | VSPI | Domyślna magistrala SPI na ESP32 |

#### Ważne uwagi dotyczące SPI

1. **Długość przewodów**: Przewody SPI powinny być jak najkrótsze (< 15cm) dla stabilnej komunikacji przy 4 MHz.

2. **Rezystory pull-up**: Pin CS (NCS) powinien mieć rezystor pull-up 10kΩ do VCC, aby zapobiec przypadkowej aktywacji podczas startu ESP32. Większość modułów breakout ma już wbudowany pull-up.

3. **Separacja magistrali**: IMU używa SPI, a OLED używa I2C - to dwa oddzielne interfejsy, które nie kolidują ze sobą.

4. **Zasilanie**: Moduł ICM-20948 wymaga zasilania **3.3V**. Nie podłączaj do 5V bez konwertera poziomów!

5. **Kolejność włączania**: CS powinien być HIGH przed inicjalizacją SPI, aby uniknąć konfliktów na magistrali.

#### Alternatywne piny SPI

Jeśli domyślne piny VSPI są zajęte, można użyć HSPI:

| Funkcja | VSPI (domyślne) | HSPI (alternatywne) |
|---------|-----------------|---------------------|
| MOSI | GPIO23 | GPIO13 |
| MISO | GPIO19 | GPIO12 |
| SCK | GPIO18 | GPIO14 |
| CS | GPIO5 (dowolny) | GPIO15 (dowolny) |

> **Uwaga:** Zmiana na HSPI wymaga modyfikacji kodu - zmień definicje `SPI_MOSI`, `SPI_MISO`, `SPI_SCK` w pliku .ino.

### Połączenia OLED 128x32 (I2C)

Wyświetlacz OLED pozostaje na magistrali I2C, niezależnie od IMU:

```
OLED SSD1306           ESP32
┌────────┐           ┌────────┐
│  VCC   │───────────│  3V3   │  (lub VIN dla 5V OLED)
│  GND   │───────────│  GND   │
│  SDA   │───────────│ GPIO21 │
│  SCL   │───────────│ GPIO22 │
└────────┘           └────────┘
```

### Podłączenie przycisku kalibracji

```
┌─────────┐
│ BUTTON  │
│   NO    │────────────┐
│         │            │
└─────────┘            │
                       │
ESP32 GPIO15 ──────────┤
                       │
ESP32 GND ─────────────┘

(Wewnętrzny pullup w ESP32 - przycisk do masy)
```

## Pełny schemat połączeń

```
                           +3.3V
                             │
              ┌──────────────┼──────────────┐
              │              │              │
           [3V3]          [VCC]          [VCC]
         ESP32-WROOM     ICM-20948    OLED 128x32
           [GND]          [GND]          [GND]
              │              │              │
              └──────────────┴──────────────┴────── GND

       ESP32-WROOM                    ICM-20948 (SPI)
         [GPIO23] ───────────────────── [SDA/MOSI]
         [GPIO18] ───────────────────── [SCL/SCLK]
         [GPIO19] ───────────────────── [AD0/MISO]
         [GPIO5]  ───────────────────── [NCS]
         [GPIO4]  ───────────────────── [INT] (opcjonalnie)

       ESP32-WROOM                    OLED 128x32 (I2C)
         [GPIO21] ───────────────────── [SDA]
         [GPIO22] ───────────────────── [SCL]

       ESP32-WROOM                    Przycisk
         [GPIO15] ───────────────────── [1]
            [GND] ───────────────────── [2]
```

## Obsługa przycisku

### Tryby działania

| Akcja | Czas trzymania | Funkcja |
|-------|----------------|---------|
| Krótkie naciśnięcie | < 2s | Potwierdzenie (podczas kalibracji) |
| Długie naciśnięcie | ≥ 2s | **Rozpoczęcie pełnej kalibracji** |

### Sekwencja kalibracji

1. **Przytrzymaj przycisk** przez 2 sekundy
2. Wyświetli się ekran powitalny kalibracji
3. **Naciśnij przycisk** aby rozpocząć

#### Krok 1: Ustaw czujnik płasko
```
┌────────────────────────┐
│ FLAT CHECK             │
│                        │
│ Ustaw plasko           │
│ Chip gora              │
│                        │
│ ax:0.02 ay:0.01        │
│ az:0.98 (28s)          │
└────────────────────────┘
```
- Ustaw czujnik **płasko** na stabilnej powierzchni
- Chip musi być skierowany **do góry** (bottom down)
- Przytrzymaj nieruchomo przez ~1.5 sekundy
- Program weryfikuje orientację za pomocą akcelerometru
- Naciśnij przycisk aby pominąć (jeśli jesteś pewien orientacji)

#### Krok 2: Kalibracja żyroskopu
```
┌────────────────────────┐
│▓▓▓▓ GYRO CAL ▓▓▓▓▓▓▓▓▓▓│
│                        │
│ Keep sensor            │
│ completely still!      │
│                        │
│ ████████████░░░░░░  67%│
└────────────────────────┘
```
- Trzymaj urządzenie **nieruchomo**
- Pasek postępu pokazuje zbieranie 1000 próbek
- Trwa około 2-3 sekundy

#### Krok 3: Kalibracja magnetometru
```
┌────────────────────────┐
│▓▓▓▓ MAG CAL ▓▓▓▓▓▓▓▓▓▓│
│                        │
│ X:OK Y:-- 95s          │
│ R:120/45/80            │
│ N:456 BTN=done         │
│ ██████████░░░░░░░  52%│
└────────────────────────┘
```
- **Obracaj powoli** urządzenie we wszystkich kierunkach
- Wykonaj pełne obroty: przód/tył, góra/dół, boki
- Status:
  - `X:OK Y:--` - oś X skalibrowana, Y wymaga więcej danych
  - `R:120/45/80` - zakresy dla osi X/Y/Z
  - `N:456` - liczba zebranych próbek
  - Pozostały czas w sekundach
- **Naciśnij przycisk** gdy skończysz (po minimum 10s)
- Kalibracja kończy się automatycznie gdy zebrane wystarczające dane

#### Krok 4: Wyniki
```
┌────────────────────────┐
│ CAL COMPLETE           │
│                        │
│ Quality: 87%           │
│ X:145 Y:132 Z:98       │
│ Saved!                 │
│                        │
└────────────────────────┘
```

**Uwaga:** Kalibracja używa wyłącznie metody dopasowania elipsoidy (ellipsoid fitting). 
Jeśli kalibracja nie powiedzie się (zbyt mało próbek lub błąd dopasowania), 
wyświetli się komunikat "CAL FAILED!" i kalibracja nie zostanie zapisana.

### Wskaźnik jakości kalibracji

| Jakość | Opis |
|--------|------|
| 90-100% | Doskonała - wszystkie osie dobrze pokryte |
| 70-89% | Dobra - kompas będzie działał prawidłowo |
| 50-69% | Średnia - rozważ powtórzenie kalibracji |
| < 50% | Słaba - powtórz kalibrację z lepszym pokryciem osi |

## Wyświetlanie (Tryb normalny)

```
┌────────────────────────┐
│ Compass ESP32          │
├────────────────────────┤
│ Roll:5.2° Pitch:-2.1°  │
│                        │
│ Mag:  12.5° N          │
│                        │
│ Geo:  18.0° N          │
│                        │
│ Heading: 18°    ◯─     │
└────────────────────────┘
```

- **Roll, Pitch** - kąty pochylenia w stopniach
- **Mag** - odchylenie od północy magnetycznej
- **Geo** - odchylenie od północy geograficznej (z deklinacją)
- **Heading** - kierunek absolutny z graficznym kompasem

## Teoria kalibracji / Calibration Theory

### Ulepszona kalibracja magnetometru (Ellipsoid Fitting)

Projekt implementuje zaawansowaną kalibrację magnetometru opartą na metodzie dopasowania elipsoidy (ellipsoid fitting), zgodnie z artykułami:
- [Improved Magnetometer Calibration (Part 1)](https://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html)
- [Improved Magnetometer Calibration (Part 2)](https://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html)
- [Cave Pearl Project - Calibrating Compass](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/)
- [IOP Science - Ellipsoid Fitting Method](https://iopscience.iop.org/article/10.1088/1755-1315/237/3/032015/pdf)

Implementacja jest kompatybilna z [jremington/ICM_20948-AHRS](https://github.com/jremington/ICM_20948-AHRS).

### Model błędów magnetometru

Surowe odczyty magnetometru są zakłócone przez:
1. **Hard Iron** - stałe przesunięcie środka danych (magnesy, elementy stalowe)
2. **Soft Iron** - zniekształcenie sfery w elipsoidę (materiały ferromagnetyczne)
3. **Błędy skalowania** - różne czułości osi
4. **Nieortogonalność osi** - osie czujnika nie są idealnie prostopadłe

### Metoda Ellipsoid Fitting (Li's Algorithm)

Algorytm dopasowania elipsoidy znajduje parametry transformacji, która przekształca zniekształconą elipsoidę danych z powrotem do sfery:

```
// Równanie ogólnej elipsoidy:
(x - B)^T * M * (x - B) = 1

// Gdzie:
// B - wektor przesunięcia (hard iron bias)
// M - macierz 3x3 opisująca kształt elipsoidy

// Korekcja:
calibrated = A_inv * (raw - B)

// Gdzie A_inv = F * sqrt(M), F - współczynnik normalizacji
```

### Format kalibracji (kompatybilny z jremington)

```cpp
// Hard Iron bias (wektor przesunięcia)
float M_B[3] = {bias_x, bias_y, bias_z};

// Soft Iron correction matrix (macierz transformacji 3x3)
float M_Ainv[3][3] = {
  {a00, a01, a02},
  {a10, a11, a12},
  {a20, a21, a22}
};

// Zastosowanie kalibracji:
float temp[3];
for (int i = 0; i < 3; i++) temp[i] = raw[i] - M_B[i];
calibrated[0] = M_Ainv[0][0]*temp[0] + M_Ainv[0][1]*temp[1] + M_Ainv[0][2]*temp[2];
calibrated[1] = M_Ainv[1][0]*temp[0] + M_Ainv[1][1]*temp[1] + M_Ainv[1][2]*temp[2];
calibrated[2] = M_Ainv[2][0]*temp[0] + M_Ainv[2][1]*temp[1] + M_Ainv[2][2]*temp[2];
```

### Zalety metody Ellipsoid Fitting

Projekt używa wyłącznie metody dopasowania elipsoidy (ellipsoid fitting). Metoda min/max została usunięta.

| Cecha | Ellipsoid Fitting |
|-------|-------------------|
| Korekcja Hard Iron | ✅ |
| Korekcja Soft Iron (skalowanie) | ✅ |
| Korekcja nieortogonalności | ✅ |
| Korekcja pełnej rotacji elipsoidy | ✅ |
| Odporność na outliers | ✅ |
| Dokładność typowa | 90-95% |

## Filtr Mahony AHRS

Projekt wykorzystuje filtr Mahony AHRS oparty na kwaternionach z referencjami wektorów Up i West:

- **Szybka konwergencja** - wykorzystuje wektor horyzontu (a × m)
- **Brak gimbal lock** - reprezentacja kwaternionowa
- **Integracja żyroskopu** - płynna odpowiedź na ruch
- **Kompensacja dryfu** - sprzężenie proporcjonalne i całkowe

Parametry dla ESP32 (zoptymalizowane dla stabilności):
- `Kp = 10.0` - wzmocnienie proporcjonalne (niższe = mniej oscylacji)
- `Ki = 0.005` - wzmocnienie całkowe (kompensacja dryfu)

> **Uwaga:** Przy bardzo dynamicznych ruchach można zwiększyć Kp do 15-20, ale przy spokojnych pomiarach Kp=10 jest bardziej stabilne.

## Wymagane biblioteki

Zainstaluj przez Arduino IDE Library Manager:

1. **Adafruit SSD1306** - obsługa wyświetlacza OLED
2. **Adafruit GFX** - grafika dla wyświetlaczy
3. **SparkFun ICM-20948** - obsługa czujnika IMU (wspiera I2C i SPI)
4. **Preferences** - wbudowana w ESP32 (zapis w NVS)

## Instalacja

### Arduino IDE

1. Dodaj obsługę ESP32:
   - Plik → Preferencje → Dodatkowe adresy URL:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
   - Narzędzia → Płytka → Menadżer płytek → Szukaj "ESP32" → Instaluj

2. Otwórz `ICM20948_Compass_Tilt_ESP32/ICM20948_Compass_Tilt_ESP32.ino`

3. Zainstaluj biblioteki (Narzędzia → Zarządzaj bibliotekami):
   - Adafruit SSD1306
   - Adafruit GFX Library
   - SparkFun ICM-20948

4. Wybierz płytkę: **ESP32 Dev Module** lub **ESP32-WROOM-DA Module**

5. Konfiguracja:
   - Upload Speed: 921600
   - CPU Frequency: 240MHz
   - Flash Mode: QIO
   - Partition Scheme: Default 4MB with spiffs

6. Wgraj program

7. Otwórz Monitor portu szeregowego (115200 baud) dla diagnostyki

8. **Przytrzymaj przycisk 2s** dla kalibracji

### PlatformIO

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps = 
    adafruit/Adafruit SSD1306@^2.5.7
    adafruit/Adafruit GFX Library@^1.11.5
    sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948@^1.2.12
```

## Dlaczego SPI zamiast I2C?

### Porównanie interfejsów

| Cecha | I2C | SPI |
|-------|-----|-----|
| **Prędkość** | 400 kHz (Fast Mode) | 4 MHz (możliwe do 7 MHz) |
| **Liczba przewodów** | 2 (SDA, SCL) | 4 (MOSI, MISO, SCK, CS) |
| **Dzielona magistrala** | Tak (wiele urządzeń) | Tak (oddzielny CS dla każdego) |
| **Niezawodność** | Podatna na zakłócenia | Bardziej odporna |
| **Złożoność** | Prostsza | Więcej przewodów |
| **Opóźnienie** | Wyższe (protokół adresowania) | Niższe (bezpośredni dostęp) |

### Zalety SPI dla IMU

1. **Szybsze próbkowanie** - 10x wyższa prędkość pozwala na częstsze odczyty
2. **Mniejsze opóźnienia** - krytyczne dla filtra AHRS w czasie rzeczywistym
3. **Lepsza stabilność** - brak problemów z adresowaniem i arbitrażem magistrali
4. **Separacja od OLED** - IMU na SPI nie koliduje z wyświetlaczem na I2C

### Kiedy użyć I2C?

- Gdy masz ograniczoną liczbę dostępnych pinów GPIO
- Gdy odległość do czujnika jest bardzo mała (<5cm)
- Gdy nie potrzebujesz maksymalnej wydajności

## Rozwiązywanie problemów

### IMU nie wykryty (SPI)
- Sprawdź połączenia SPI: MOSI (GPIO23), MISO (GPIO19), SCK (GPIO18), CS (GPIO5)
- Sprawdź zasilanie 3.3V na module ICM-20948
- Upewnij się, że CS jest podłączony i nie wisi w powietrzu
- Sprawdź czy przewody nie są za długie (max 15cm dla 4 MHz)
- Zweryfikuj ciągłość połączeń multimetrem

### IMU wykryty ale błędne odczyty
- Sprawdź jakość lutowania/połączeń
- Zmniejsz prędkość SPI do 1 MHz (zmień `SPI_SPEED` w kodzie)
- Dodaj kondensator 100nF między VCC a GND blisko modułu
- Sprawdź czy nie ma zwarć między pinami

### Kompas nieprecyzyjny
- Wykonaj ponowną kalibrację z lepszym pokryciem osi
- Unikaj metalu w pobliżu podczas kalibracji
- Sprawdź jakość kalibracji (cel: > 70%)
- Upewnij się, że deklinacja magnetyczna jest poprawna dla Twojej lokalizacji

### Przycisk nie reaguje
- Sprawdź połączenie do GPIO15 i GND
- Użyj przycisku NO (normalnie otwarty)

### Wyświetlacz nie działa (I2C)
- Sprawdź adres I2C (domyślnie 0x3C)
- Niektóre OLED używają 0x3D
- Sprawdź połączenia SDA (GPIO21) i SCL (GPIO22)

### Program się zawiesza / OLED zamraża obraz
- Sprawdź jakość połączeń I2C dla OLED (kable krótkie i dobrze zamocowane)
- Sprawdź zasilanie - niestabilne zasilanie może powodować problemy
- Program automatycznie wykrywa zawieszenia I2C i próbuje odzyskać magistralę
- Watchdog automatycznie restartuje ESP32 jeśli program nie odpowiada przez 10 sekund
- Sprawdź czy nie ma zakłóceń elektromagnetycznych w pobliżu

### Diagnostyka przez Serial Monitor

Otwórz Serial Monitor (115200 baud) aby zobaczyć komunikaty diagnostyczne:

```
=== ICM20948 Compass ESP32 (SPI) ===
Initializing...
I2C scan: OLED=OK (IMU is on SPI)
IMU found on SPI bus
  CS: GPIO5, SCK: GPIO18, MISO: GPIO19, MOSI: GPIO23
Magnetometer initialized
IMU initialized successfully via SPI
```

## Mechanizmy stabilności

Projekt zawiera kilka mechanizmów zwiększających stabilność:

| Mechanizm | Opis |
|-----------|------|
| **Watchdog Timer** | Automatycznie restartuje ESP32 jeśli program nie odpowiada przez 10 sekund |
| **I2C Bus Recovery** | Automatyczne odzyskiwanie magistrali I2C po wykryciu zawieszenia (dla OLED) |
| **I2C Timeout** | Timeout operacji I2C (15ms) zapobiega nieskończonemu oczekiwaniu |
| **SPI dla IMU** | Szybsza i bardziej niezawodna komunikacja z czujnikiem |
| **Quaternion Validation** | Resetowanie AHRS przy wykryciu nieprawidłowego stanu kwaternionu |
| **micros() Overflow Protection** | Poprawna obsługa przelewania licznika czasu (~71 minut) |
| **NaN Detection** | Wykrywanie i odrzucanie nieprawidłowych odczytów czujników |
| **Gyro Outlier Rejection** | Odrzucanie wartości odstających podczas kalibracji żyroskopu |

## Licencja

MIT License

## Referencje

- [The Cave Pearl Project - Calibrating Compass](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/)
- [jremington/ICM_20948-AHRS](https://github.com/jremington/ICM_20948-AHRS)
- [Pololu - Correcting Magnetometer](https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315)
- [SparkFun ICM-20948 Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)