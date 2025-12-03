# ICM-20948 Compass & Tilt Meter

Projekt na Arduino Mini Pro do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej.

## Sprzęt / Hardware

- **Arduino Mini Pro** (3.3V lub 5V)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)
- **Przycisk** - do kalibracji i zmiany trybu

## Funkcje / Features

- ✅ Pomiar pochylenia na 3 osiach (Roll, Pitch, Yaw)
- ✅ Wskazanie północy magnetycznej
- ✅ Wskazanie północy geograficznej (skorygowane dla Żywca, Polska)
- ✅ Kalibracja magnetometru (przycisk lub serial)
- ✅ 3 tryby wyświetlania

## Dane lokalizacyjne / Location Data

| Parametr | Wartość |
|----------|---------|
| Miasto | Żywiec, Polska |
| Szerokość geograficzna | 49.6853°N |
| Długość geograficzna | 19.1925°E |
| Deklinacja magnetyczna | 5.5° E (2024) |

## Wyprowadzenie pinów / Pin Connections

### Arduino Mini Pro

```
┌─────────────────────────────────────────┐
│  [RAW] ─── Zasilanie 5-12V             │
│  [VCC] ─── Zasilanie regulowane 3.3/5V │
│  [GND] ─── Masa                         │
│                                         │
│  [A4/SDA] ─── I2C Data                 │
│  [A5/SCL] ─── I2C Clock                │
│                                         │
│  [D2] ─── Przycisk kalibracji          │
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
│  INT ────── (opcjonalnie) Arduino D3   │
└─────────────────────────────────────────┘
```

⚠️ **UWAGA:** ICM-20948 wymaga zasilania 3.3V! Dla Arduino 5V użyj konwertera poziomów lub modułu z wbudowanym regulatorem.

### OLED 128x32 (SSD1306)

```
┌─────────────────────────────────────────┐
│  VCC ────── Arduino VCC (3.3V lub 5V)  │
│  GND ────── Arduino GND                │
│  SDA ────── Arduino A4                 │
│  SCL ────── Arduino A5                 │
└─────────────────────────────────────────┘
```

### Przycisk kalibracji

```
┌─────────────────────────────────────────┐
│  Arduino D2 ───┬─── Przycisk ─── GND   │
│                │                        │
│                └─── (wewnętrzny pullup) │
│                                         │
│  Przytrzymaj 3 sekundy = kalibracja    │
│  Krótkie naciśnięcie = zmiana trybu    │
└─────────────────────────────────────────┘
```

## Schemat połączeń

```
                    +3.3V
                      │
          ┌───────────┼───────────┐
          │           │           │
       [VCC]       [VCC]       [VCC]
     ICM-20948    OLED 128x32  Arduino
       [GND]       [GND]       [GND]
          │           │           │
          └───────────┴───────────┴──── GND

     ICM-20948                Arduino Mini
       [SDA] ─────────────────── [A4]
       [SCL] ─────────────────── [A5]

     OLED 128x32              Arduino Mini
       [SDA] ─────────────────── [A4]
       [SCL] ─────────────────── [A5]

     Przycisk                 Arduino Mini
       [Pin1] ────────────────── [D2]
       [Pin2] ────────────────── [GND]
```

## Sterowanie

### Przycisk (D2)

| Akcja | Funkcja |
|-------|---------|
| Krótkie naciśnięcie | Zmiana trybu wyświetlania |
| Przytrzymanie 3 sek | Uruchomienie kalibracji magnetometru |

### Komendy Serial (115200 baud)

| Komenda | Funkcja |
|---------|---------|
| `c` | Uruchom kalibrację magnetometru |
| `m` | Zmień tryb wyświetlania |
| `r` | Resetuj kalibrację |
| `h` | Pokaż pomoc |

## Tryby wyświetlania

1. **Tryb pochylenia (Tilt)** - Roll, Pitch + mały kompas
2. **Tryb kompasu (Compass)** - Północ magnetyczna i geograficzna
3. **Tryb wszystkich danych (All)** - Kompaktowy widok wszystkich wartości

## Wymagane biblioteki

Zainstaluj przez Arduino Library Manager:

1. **Adafruit SSD1306** - obsługa wyświetlacza OLED
2. **Adafruit GFX** - grafika dla wyświetlaczy
3. **Adafruit BusIO** - wymagana przez Adafruit SSD1306
4. **SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library** - obsługa czujnika IMU

### Instalacja biblioteki SparkFun ICM-20948

⚠️ **WAŻNE:** Bibliotekę SparkFun ICM-20948 należy zainstalować poprawnie, aby uniknąć błędów linkera.

#### Metoda 1: Arduino Library Manager (zalecana)

1. W Arduino IDE otwórz **Szkic → Dołącz bibliotekę → Zarządzaj bibliotekami...**
2. Wyszukaj: `SparkFun ICM-20948`
3. Zainstaluj bibliotekę **SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library**

#### Metoda 2: Instalacja ręczna z GitHub

1. Pobierz bibliotekę z: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
2. Kliknij **Code → Download ZIP**
3. W Arduino IDE: **Szkic → Dołącz bibliotekę → Dodaj bibliotekę .ZIP...**
4. Wybierz pobrany plik ZIP

#### ⚠️ Rozwiązywanie problemów z linkerem

Jeśli podczas kompilacji pojawią się błędy typu:
```
undefined reference to `ICM_20948_i2c_master_passthrough'
undefined reference to `ICM_20948_get_agmt'
undefined reference to `ICM_20948_check_id'
```

**Przyczyna:** Biblioteka została zainstalowana nieprawidłowo. Folder biblioteki musi mieć poprawną nazwę (nie `src`).

**Rozwiązanie:**

1. Zamknij Arduino IDE
2. Przejdź do folderu bibliotek Arduino:
   - Windows: `C:\Users\<nazwa_użytkownika>\Documents\Arduino\libraries\`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
3. Sprawdź, czy istnieje folder o nazwie `src` bezpośrednio w folderze `libraries` (ścieżka: `libraries/src/ICM_20948.h`). Jeśli tak, to jest niepoprawnie zainstalowana biblioteka ICM-20948 - usuń cały folder `libraries/src/`
4. Upewnij się, że folder biblioteki nazywa się np.:
   - `SparkFun_ICM-20948_ArduinoLibrary` lub
   - `SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library`
5. Sprawdź strukturę folderu biblioteki - powinna zawierać:
   ```
   SparkFun_ICM-20948_ArduinoLibrary/
   ├── library.properties
   ├── src/
   │   ├── ICM_20948.cpp
   │   ├── ICM_20948.h
   │   └── util/
   │       ├── ICM_20948_C.c    ← KRYTYCZNY PLIK!
   │       ├── ICM_20948_C.h
   │       └── ...
   └── examples/
   ```
6. Uruchom ponownie Arduino IDE i skompiluj projekt

## Kalibracja magnetometru

Dla uzyskania dokładnych odczytów kompasu, przeprowadź kalibrację:

1. Przytrzymaj przycisk przez 3 sekundy (lub wyślij `c` przez serial)
2. Obracaj czujnik we wszystkich kierunkach przez 30 sekund
3. Kalibracja zakończy się automatycznie

## Instalacja

1. Otwórz plik `ICM20948_Compass_Tilt.ino` w Arduino IDE
2. Zainstaluj wymagane biblioteki
3. Wybierz płytkę: **Arduino Pro or Pro Mini**
4. Wybierz procesor: **ATmega328P (3.3V, 8MHz)** lub **ATmega328P (5V, 16MHz)**
5. Wgraj program

## Licencja

MIT License