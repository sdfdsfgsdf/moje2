# ICM-20948 Compass & Tilt Meter

Projekt na Arduino Mini Pro do pomiaru pochylenia i wskazywania północy magnetycznej oraz geograficznej z filtrem Mahony AHRS.

## Sprzęt / Hardware

- **Arduino Mini Pro** (3.3V lub 5V)
- **ICM-20948** - 9-DOF IMU (akcelerometr, żyroskop, magnetometr)
- **OLED 128x32** - wyświetlacz I2C (sterownik SSD1306)

## Funkcje / Features

- ✅ **Filtr Mahony AHRS** - dokładne śledzenie orientacji z kwaternionami
- ✅ **Integracja żyroskopu** - płynna odpowiedź dynamiczna
- ✅ Pomiar pochylenia na 2 osiach (X, Y / Roll, Pitch) z dokładnością 0.1°
- ✅ Wskazanie północy magnetycznej
- ✅ Wskazanie północy geograficznej (skorygowane dla Żywca, Polska)
- ✅ Filtrowanie EMA (Exponential Moving Average) dla stabilnych odczytów
- ✅ Poprawne uśrednianie kątowe dla kompasu (obsługa przejścia 0°/360°)
- ✅ Jeden ekran OLED ze wszystkimi danymi
- ✅ **Wbudowana automatyczna kalibracja** (bez zewnętrznych skryptów!)
- ✅ Korekcja Hard Iron (offset min/max wg Cave Pearl Project)
- ✅ Korekcja Soft Iron (skalowanie osi)
- ✅ Kalibracja żyroskopu przy starcie
- ✅ Zapisywanie danych kalibracji w EEPROM z weryfikacją CRC8
- ✅ Korekcja mapowania osi magnetometru AK09916
- ✅ Walidacja danych magnetometru

## Dane lokalizacyjne / Location Data

| Parametr | Wartość |
|----------|---------|
| Miasto | Żywiec, Polska |
| Szerokość geograficzna | 49.6853°N |
| Długość geograficzna | 19.1925°E |
| Deklinacja magnetyczna | 5.5° E (2024) |

## Teoria kalibracji / Calibration Theory

Kalibracja opiera się na metodzie opisanej w [The Cave Pearl Project](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/):

### Hard Iron (twarde żelazo)
- Stałe zakłócenia magnetyczne w pobliżu czujnika
- Powodują przesunięcie środka elipsoidy danych
- **Korekcja:** offset = (min + max) / 2 dla każdej osi

### Soft Iron (miękkie żelazo)
- Zakłócenia od materiałów ferromagnetycznych
- Powodują zniekształcenie sfery w elipsoidę
- **Korekcja:** skalowanie = średnia_delta / delta_osi

Ta metoda wymaga minimalnej ilości RAM (tylko 6 floatów dla min/max) - idealna dla ATmega328P!

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

### Uruchamianie kalibracji

Kalibracja uruchamia się automatycznie poprzez **3 krótkie uruchomienia** (<2 sekundy każde):

| Uruchomienie | Czas działania | Efekt |
|--------------|----------------|-------|
| 1 | < 2 sekundy | Licznik +1 |
| 2 | < 2 sekundy | Licznik +1 |
| 3 | - | **Tryb kalibracji** |

### Procedura kalibracji

#### Krok 1: Kalibracja żyroskopu
1. Po wejściu w tryb kalibracji wyświetli się komunikat
2. **Trzymaj czujnik nieruchomo** przez kilka sekund
3. Pasek postępu pokaże status
4. Żyroskop zostanie skalibrowany automatycznie

#### Krok 2: Kalibracja magnetometru
1. Wyświetli się komunikat "Obracaj powoli we wszystkich"
2. **Obracaj czujnik** powoli we wszystkich kierunkach
3. Na ekranie pokazany jest status:
   - `X Y Z` - osie wymagające większego zakresu
   - `+` - oś już skalibrowana
   - Aktualne zakresy dla każdej osi
   - Pozostały czas (minimum 5 sekund)
4. Kalibracja kończy się gdy:
   - Osie X i Y osiągną minimalny zakres (100 μT)
   - Minął minimalny czas (5 sekund)

### Podczas kalibracji magnetometru

```
KAL: +Y+  3s      <- X ok, Y potrzebuje pracy, 3s pozostało
X:120 Y:80        <- Aktualne zakresy
Z:45 cel:100      <- Zakres Z i cel
```

### Wskazówki dla najlepszej kalibracji

1. **Obracaj powoli i płynnie** - szybkie ruchy nie pomagają
2. **Pokryj wszystkie orientacje** - przód/tył, góra/dół, boki
3. **Unikaj metalowych przedmiotów** w pobliżu podczas kalibracji
4. **Kalibruj w docelowym środowisku** - zakłócenia magnetyczne są różne
5. Oś Z jest opcjonalna, ale pomaga przy dużych kątach pochylenia

### Reset licznika

Jeśli urządzenie działa **dłużej niż 2 sekundy**, licznik krótkich uruchomień jest automatycznie resetowany.

## Wyświetlanie

Jeden ekran ze wszystkimi danymi:
```
X:0.0 Y:0.0
Mag:-5.0° N
Geo:0.0° N
```
- **X, Y** - kąty pochylenia (Roll, Pitch) z dokładnością do 0.1°
- **Mag** - odchylenie od bieguna magnetycznego (0 = północ)
- **Geo** - odchylenie od bieguna geograficznego (z deklinacją)

## Filtr Mahony AHRS

Projekt wykorzystuje filtr Mahony AHRS (Attitude and Heading Reference System) oparty na kwaternionach:

- **Szybka konwergencja** - wykorzystuje wektory Up i West jako referencje
- **Brak gimbal lock** - reprezentacja kwaternionowa
- **Integracja żyroskopu** - płynna odpowiedź na ruch
- **Fuzja sensorów** - akcelerometr + żyroskop + magnetometr

Bazuje na implementacji z [jremington/ICM_20948-AHRS](https://github.com/jremington/ICM_20948-AHRS).

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
6. Wykonaj kalibrację (3x szybki restart)

## Zużycie pamięci

Projekt jest zoptymalizowany dla ATmega328P:
- **Flash:** ~25KB (z bibliotekami)
- **RAM:** ~1.2KB (min/max zamiast setek próbek)
- **EEPROM:** ~50 bajtów (kalibracja + CRC)

## Licencja

MIT License

## Referencje

- [The Cave Pearl Project - Calibrating Compass](https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/)
- [jremington/ICM_20948-AHRS](https://github.com/jremington/ICM_20948-AHRS)
- [SparkFun ICM-20948 Library](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)