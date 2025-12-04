# ICM-20948 Compass & Tilt Meter

Projekt do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej z filtrem Mahony AHRS.

**Dostępne wersje:**
- 📁 `ICM20948_Compass_Tilt/` - Wersja dla **Arduino Pro Mini** (oryginalna)
- 📁 `ICM20948_Compass_Tilt_ESP32/` - Wersja dla **ESP32-WROOM-32D** (zoptymalizowana) ⭐

---

# 🆕 Wersja ESP32-WROOM-32D (Zalecana)

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
- ✅ Większy wyświetlacz 128x64 z bogatszym UI
- ✅ Aktualizacja wyświetlacza 10Hz (vs 4Hz na Arduino)

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

### Metoda Cave Pearl Project

Kalibracja opiera się na metodzie opisanej w [The Cave Pearl Project](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/):

### Hard Iron (twarde żelazo)
- **Przyczyna:** Stałe zakłócenia magnetyczne w pobliżu czujnika (magnesy, elementy stalowe)
- **Efekt:** Przesunięcie środka elipsoidy danych
- **Korekcja:** `offset = (min + max) / 2` dla każdej osi

### Soft Iron (miękkie żelazo)
- **Przyczyna:** Zakłócenia od materiałów ferromagnetycznych w pobliżu
- **Efekt:** Zniekształcenie sfery w elipsoidę
- **Korekcja:** `scale = średnia_delta / delta_osi`

### Wzory korekcji

```
// Dla każdej osi i = 0, 1, 2 (X, Y, Z):

offset[i] = (max[i] + min[i]) / 2
delta[i] = max[i] - min[i]
avgDelta = (delta[0] + delta[1] + delta[2]) / 3
scale[i] = avgDelta / delta[i]

// Zastosowanie:
calibrated[i] = (raw[i] - offset[i]) * scale[i]
```

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

## Porównanie wersji

| Cecha | Arduino Pro Mini | ESP32-WROOM-32D |
|-------|------------------|-----------------|
| Flash | ~25KB | ~250KB |
| RAM | ~1.2KB | ~520KB |
| Zegar | 8/16 MHz | 240 MHz |
| FPU | Brak | Tak |
| Kalibracja | 3x restart | 1 przycisk |
| Zapis | EEPROM + CRC | NVS (Flash) |
| Wyświetlacz | 128x32 | 128x32 |
| Próbki żyroskopu | 500 | 1000 |
| Próbki magnetometru | Min/Max tylko | Do 500 punktów |
| Aktualizacja wyśw. | 4 Hz | 10 Hz |
| Monitor szeregowy | Ograniczony | Pełna diagnostyka |

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

---

# Wersja Arduino Pro Mini (Oryginalna)

Dokumentacja oryginalnej wersji znajduje się poniżej dla zachowania kompatybilności wstecznej.

## Sprzęt / Hardware

- **Arduino Mini Pro** (3.3V lub 5V)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)

## Wyprowadzenie pinów

### Arduino Mini Pro

```
┌─────────────────────────────────────────┐
│  [RAW] ─── Zasilanie 5-12V             │
│  [VCC] ─── Zasilanie regulowane 3.3/5V │
│  [GND] ─── Masa                         │
│                                         │
│  [A4/SDA] ─── I2C Data                 │
│  [A5/SCL] ─── I2C Clock                │
└─────────────────────────────────────────┘
```

### ICM-20948 (Czujnik 9-DOF)

```
┌─────────────────────────────────────────┐
│  VCC ────── Arduino VCC (3.3V!)        │
│  GND ────── Arduino GND                │
│  SDA ────── Arduino A4                 │
│  SCL ────── Arduino A5                 │
│  AD0 ────── GND (adres 0x68)           │
│         lub VCC (adres 0x69)           │
└─────────────────────────────────────────┘
```

### OLED 128x32 (SSD1306)

```
┌─────────────────────────────────────────┐
│  VCC ────── Arduino VCC (3.3V lub 5V)  │
│  GND ────── Arduino GND                │
│  SDA ────── Arduino A4                 │
│  SCL ────── Arduino A5                 │
└─────────────────────────────────────────┘
```

## Automatyczna kalibracja (Arduino)

Kalibracja uruchamia się poprzez **3 krótkie uruchomienia** (<2 sekundy każde):

| Uruchomienie | Czas działania | Efekt |
|--------------|----------------|-------|
| 1 | < 2 sekundy | Licznik +1 |
| 2 | < 2 sekundy | Licznik +1 |
| 3 | - | **Tryb kalibracji** |

## Instalacja (Arduino)

1. Otwórz plik `ICM20948_Compass_Tilt/ICM20948_Compass_Tilt.ino`
2. Zainstaluj wymagane biblioteki
3. Wybierz płytkę: **Arduino Pro or Pro Mini**
4. Wybierz procesor: **ATmega328P (3.3V, 8MHz)** lub **ATmega328P (5V, 16MHz)**
5. Wgraj program
6. Wykonaj kalibrację (3x szybki restart)