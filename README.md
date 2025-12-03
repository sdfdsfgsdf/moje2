# ICM-20948 Compass & Tilt Meter

Projekt na Arduino Mini Pro do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej.

## Sprzęt / Hardware

- **Arduino Mini Pro** (3.3V lub 5V)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)

## Funkcje / Features

- ✅ Pomiar pochylenia na 2 osiach (X, Y / Roll, Pitch) z dokładnością 1 miejsca po przecinku
- ✅ Wskazanie północy magnetycznej
- ✅ Wskazanie północy geograficznej (skorygowane dla Żywca, Polska)
- ✅ Filtrowanie EMA (Exponential Moving Average) dla stabilnych odczytów
- ✅ Poprawne uśrednianie kątowe dla kompasu (obsługa przejścia 0°/360°)
- ✅ Jeden ekran OLED ze wszystkimi danymi
- ✅ Automatyczna kalibracja magnetometru dla osi X i Y (wykrywanie krótkich uruchomień)
- ✅ Zapisywanie danych kalibracji w EEPROM
- ✅ Korekcja mapowania osi magnetometru AK09916 (zgodność z układem współrzędnych akcelerometru)
- ✅ Walidacja danych magnetometru (odrzucanie nieprawidłowych odczytów)

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
```

## Automatyczna kalibracja

Kalibracja magnetometru uruchamia się automatycznie poprzez krótkie uruchomienia urządzenia:

| Uruchomienie | Czas działania | Efekt |
|--------------|----------------|-------|
| 1 | < 2 sekundy | Licznik +1 |
| 2 | < 2 sekundy | Licznik +1 |
| 3 | - | **Tryb kalibracji** |

### Jak uruchomić kalibrację:
1. **Uruchom** urządzenie i **wyłącz** przed upływem 2 sekund
2. **Powtórz** krok 1
3. Przy **trzecim uruchomieniu** automatycznie włączy się tryb kalibracji
4. Obracaj czujnik we wszystkich kierunkach
5. Po zakończeniu kalibracji wyświetli się komunikat "Please restart."
6. **Zrestartuj** urządzenie

### Podczas kalibracji:
- Na ekranie wyświetlane są osie wymagające kalibracji (X, Y) oraz aktualne zakresy
- Kalibracja trwa minimum 10 sekund i zakończy się automatycznie gdy osie X i Y osiągną minimalny zakres (100 uT)
- Maksymalny czas kalibracji to 60 sekund
- Dane kalibracji są automatycznie zapisywane w EEPROM

### Reset licznika:
- Jeśli urządzenie działa **dłużej niż 2 sekundy**, licznik krótkich uruchomień jest resetowany

## Wyświetlanie

Jeden ekran ze wszystkimi danymi:
```
X:0.0 Y:0.0
Mag:-5.0° N
Geo:0.0° N
```
- **X, Y** - kąty pochylenia (Roll, Pitch) z dokładnością do 1 miejsca po przecinku
- **Mag** - odchylenie od bieguna magnetycznego (0 = idealnie na północ, wartości ujemne = na zachód, dodatnie = na wschód)
- **Geo** - odchylenie od bieguna geograficznego (skorygowane o deklinację, 0 = idealnie na północ)

## Wymagane biblioteki

Zainstaluj przez Arduino Library Manager:

1. **Adafruit SSD1306** - obsługa wyświetlacza OLED
2. **Adafruit GFX** - grafika dla wyświetlaczy
3. **SparkFun ICM-20948** - obsługa czujnika IMU

## Instalacja

1. Otwórz plik `ICM20948_Compass_Tilt.ino` w Arduino IDE
2. Zainstaluj wymagane biblioteki
3. Wybierz płytkę: **Arduino Pro or Pro Mini**
4. Wybierz procesor: **ATmega328P (3.3V, 8MHz)** lub **ATmega328P (5V, 16MHz)**
5. Wgraj program

## Licencja

MIT License