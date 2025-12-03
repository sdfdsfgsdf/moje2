# ICM-20948 Compass & Tilt Meter

Projekt na Arduino Mini Pro do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej.

## Sprzęt / Hardware

- **Arduino Mini Pro** (3.3V lub 5V)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)
- **Przycisk** - do kalibracji i zmiany trybu

## Funkcje / Features

- ✅ Pomiar pochylenia na 3 osiach (X, Y, Z / Roll, Pitch, Yaw)
- ✅ Wskazanie północy magnetycznej
- ✅ Wskazanie północy geograficznej (skorygowane dla Żywca, Polska)
- ✅ Uśrednianie wyników co 0.25s dla większej dokładności
- ✅ Jeden ekran OLED ze wszystkimi danymi
- ✅ Kalibracja magnetometru (przycisk lub serial)

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
| Przytrzymanie 3 sek | Uruchomienie kalibracji magnetometru |

## Wyświetlanie

Jeden ekran ze wszystkimi danymi:
```
X:0 Y:0 Z:0
Mag:45° NE
Geo:50° NE
```
- **X, Y, Z** - kąty pochylenia (Roll, Pitch, Yaw)
- **Mag** - biegun magnetyczny
- **Geo** - biegun geograficzny (skorygowany o deklinację)

## Wymagane biblioteki

Zainstaluj przez Arduino Library Manager:

1. **Adafruit SSD1306** - obsługa wyświetlacza OLED
2. **Adafruit GFX** - grafika dla wyświetlaczy
3. **SparkFun ICM-20948** - obsługa czujnika IMU

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