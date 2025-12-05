# ICM-20948 Compass & Tilt Meter

Projekt do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej z filtrem Mahony AHRS dla **ESP32-WROOM-32D**.

📁 `ICM20948_Compass_Tilt_ESP32/` - Wersja dla **ESP32-WROOM-32D**

## Sprzęt / Hardware

- **ESP32-WROOM-32D** (lub dowolny ESP32 DevKit)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
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

### Kalibracja na urządzeniu (bez zewnętrznego oprogramowania!)
- ✅ **Jeden przycisk** - przytrzymaj 2s aby rozpocząć kalibrację
- ✅ **Przewodnik na OLED** - instrukcje krok po kroku na ekranie
- ✅ **Pasek postępu** - wizualizacja postępu kalibracji
- ✅ **Automatyczne przetwarzanie** - obliczenia na ESP32, bez komputera!
- ✅ **Wskaźnik jakości** - ocena jakości kalibracji (0-100%)

### Korekcja magnetometru
- ✅ Korekcja Hard Iron (przesunięcie środka elipsoidy)
- ✅ Korekcja Soft Iron (skalowanie osi do sfery)
- ✅ Zbieranie do 500 próbek dla lepszej dokładności
- ✅ Automatyczne wykrywanie zakończenia kalibracji

### Optymalizacje ESP32
- ✅ Wykorzystanie jednostki FPU dla szybkich obliczeń float
- ✅ Zapis kalibracji w NVS (Preferences) zamiast EEPROM
- ✅ Szybsze próbkowanie (1000 próbek żyroskopu)
- ✅ Wyświetlacz OLED 128x32 z kompaktowym UI
- ✅ Aktualizacja wyświetlacza 10Hz

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
│  EN ─┐                                          ┌─ GPIO23   │
│ VP36 ─┤    ┌──────────────────────────────┐    ├─ GPIO22 ──── SCL (I2C)
│ VN39 ─┤    │                              │    ├─ TX0       │
│  D34 ─┤    │         ESP32-WROOM-32D      │    ├─ RX0       │
│  D35 ─┤    │                              │    ├─ GPIO21 ──── SDA (I2C)
│  D32 ─┤    │          ┌─────────┐         │    ├─ GND       │
│  D33 ─┤    │          │  ESP32  │         │    ├─ GPIO19    │
│  D25 ─┤    │          │  CHIP   │         │    ├─ GPIO18    │
│  D26 ─┤    │          └─────────┘         │    ├─ GPIO5     │
│  D27 ─┤    │                              │    ├─ GPIO17    │
│  D14 ─┤    │                              │    ├─ GPIO16    │
│  D12 ─┤    │                              │    ├─ GPIO4     │
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
| **I2C SDA** | GPIO21 | Dane I2C (do ICM-20948 i OLED) |
| **I2C SCL** | GPIO22 | Zegar I2C (do ICM-20948 i OLED) |
| **BUTTON** | GPIO15 | Przycisk kalibracji (do GND) |
| **LED** | GPIO2 | Wbudowana dioda LED (status) |
| **VIN** | VIN | Zasilanie 5V (z USB lub zewn.) |
| **3V3** | 3V3 | Zasilanie 3.3V dla czujników |
| **GND** | GND | Masa |

### Połączenia ICM-20948

```
ICM-20948              ESP32
┌────────┐           ┌────────┐
│  VCC   │───────────│  3V3   │
│  GND   │───────────│  GND   │
│  SDA   │───────────│ GPIO21 │
│  SCL   │───────────│ GPIO22 │
│  AD0   │───────────│  3V3   │  ← Adres I2C: 0x69
│  INT   │           │        │  (opcjonalnie)
└────────┘           └────────┘
```

⚠️ **Adres I2C:**
- AD0 → GND = adres 0x68
- AD0 → 3V3 = adres 0x69 (domyślnie w kodzie)

### Połączenia OLED 128x32

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

       ESP32-WROOM                    ICM-20948
         [GPIO21] ───────────────────── [SDA]
         [GPIO22] ───────────────────── [SCL]
                                        [AD0] ── 3V3

       ESP32-WROOM                    OLED 128x32
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

#### Krok 1: Kalibracja żyroskopu
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

#### Krok 2: Kalibracja magnetometru
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

#### Krok 3: Wyniki
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

### Metoda Min/Max (Cave Pearl Project - fallback)

Jeśli dopasowanie elipsoidy nie powiedzie się, używana jest prostsza metoda min/max:

```
// Hard Iron:
offset[i] = (max[i] + min[i]) / 2

// Soft Iron (skalowanie diagonalne):
delta[i] = max[i] - min[i]
avgDelta = (delta[0] + delta[1] + delta[2]) / 3
scale[i] = avgDelta / delta[i]

// Zastosowanie:
calibrated[i] = (raw[i] - offset[i]) * scale[i]
```

### Zalety metody Ellipsoid Fitting

| Cecha | Min/Max | Ellipsoid Fitting |
|-------|---------|-------------------|
| Korekcja Hard Iron | ✅ | ✅ |
| Korekcja Soft Iron (skalowanie) | ✅ | ✅ |
| Korekcja nieortogonalności | ❌ | ✅ |
| Korekcja pełnej rotacji elipsoidy | ❌ | ✅ |
| Odporność na outliers | ❌ | ✅ |
| Dokładność typowa | 70-85% | 90-95% |

## Filtr Mahony AHRS

Projekt wykorzystuje filtr Mahony AHRS oparty na kwaternionach z referencjami wektorów Up i West:

- **Szybka konwergencja** - wykorzystuje wektor horyzontu (a × m)
- **Brak gimbal lock** - reprezentacja kwaternionowa
- **Integracja żyroskopu** - płynna odpowiedź na ruch
- **Kompensacja dryfu** - sprzężenie proporcjonalne i całkowe

Parametry dla ESP32:
- `Kp = 30.0` - wzmocnienie proporcjonalne
- `Ki = 0.01` - wzmocnienie całkowe

## Wymagane biblioteki

Zainstaluj przez Arduino IDE Library Manager:

1. **Adafruit SSD1306** - obsługa wyświetlacza OLED
2. **Adafruit GFX** - grafika dla wyświetlaczy
3. **SparkFun ICM-20948** - obsługa czujnika IMU
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

## Rozwiązywanie problemów

### IMU nie wykryty
- Sprawdź połączenia I2C (SDA/SCL)
- Sprawdź zasilanie 3.3V
- Zmień `ICM_AD0_VAL` na 0 jeśli AD0 podłączony do GND

### Kompas nieprecyzyjny
- Wykonaj ponowną kalibrację z lepszym pokryciem osi
- Unikaj metalu w pobliżu podczas kalibracji
- Sprawdź jakość kalibracji (cel: > 70%)

### Przycisk nie reaguje
- Sprawdź połączenie do GPIO15 i GND
- Użyj przycisku NO (normalnie otwarty)

### Wyświetlacz nie działa
- Sprawdź adres I2C (domyślnie 0x3C)
- Niektóre OLED używają 0x3D

## Licencja

MIT License

## Referencje

- [The Cave Pearl Project - Calibrating Compass](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/)
- [jremington/ICM_20948-AHRS](https://github.com/jremington/ICM_20948-AHRS)
- [Pololu - Correcting Magnetometer](https://forum.pololu.com/t/correcting-the-balboa-magnetometer/14315)
- [SparkFun ICM-20948 Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)