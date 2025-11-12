# Group G (16) Smart Bottle  

This repository contains the firmware for a **smart self-cleaning water bottle** prototype built with an **Arduino Mega 2560** and an **ESP32**.

- The **Arduino Mega** handles all sensing, control, display, keypad input, timing, and self-cleaning logic.
- The **ESP32** receives data from the Mega over UART and uploads it to **ThingSpeak** via Wi-Fi.

> This project is part of the course **Introduction to Embedded System (2025/1), CUEE**.

---

## Key Features

### Arduino Mega (`main_code.ino`)

- **Water level & daily consumption**
  - Ultrasonic sensor measures distance from the sensor to the water surface.
  - Converts distance → water height (cm) → approximate volume (ml) using a simple linear bottle model.
  - Calculates **how much water has been drunk** by tracking decreases in volume over time (with a small threshold to reject sensor noise).
  - Automatically **resets daily consumption at midnight** using the DS3231 RTC.

- **Water temperature monitoring**
  - DS18B20 sensor measures water temperature.
  - Temperature is displayed on the LCD and sent to ESP32.

- **Self-cleaning (manual + scheduled)**
  - Two vibration motors and a “UV” indicator LED are used to represent the cleaning function.
  - **Manual cleaning**: press keypad key `1` to start a cleaning cycle immediately.
  - **Scheduled cleaning**: press key `2` and enter cleaning time in **HHMM** (24-hour) format.
  - A **Timer1 interrupt** runs at 1 Hz and counts down the cleaning duration (`CLEANING_DURATION_SEC`).
  - Buzzer beeps at the start and end of each cleaning cycle.

- **User interface (LCD + keypad)**
  - 16×2 I²C LCD (address `0x27`) displays multiple pages:
    1. **Time & Date** (HH:MM:SS and DD/MM/YYYY).
    2. **Water Temperature** (°C).
    3. **Daily Water Consumed (ml)** and **current water level** (cm + ml).
  - 4×4 keypad:
    - `1` → start manual cleaning.
    - `2` → enter cleaning schedule setup (HHMM).
    - `A` / `B` → switch between LCD pages.

- **Data to ESP32 (UART)**
  - Every 20 seconds, the Mega sends a CSV line to the ESP32:
    ```text
    <waterConsumedTodayMl>,<waterTemp>\n
    ```
  - Only **two values** are sent:
    - **Water consumption today (ml)**.
    - **Water temperature (°C)**.

---

### ESP32 (`esp32.ino`)

- **UART receiver**
  - Uses `HardwareSerial(1)` with:
    - `RX_PIN = 16` (connected to Mega TX1 pin 18 via level-shifter/resistor divider).
    - `TX_PIN = 17` (connected to Mega RX1 pin 19).
  - Builds a string buffer until it encounters `'\n'`, then calls `parseSensorLine()`.

- **CSV parsing (2-field format)**
  - Expected format from Mega:
    ```text
    <waterLevel>,<waterTemp>\n
    ```
  - `parseSensorLine()`:
    - Locates the **first comma**.
    - Splits the string into:
      - `waterLevel` – daily water consumption / level value.
      - `waterTemp`  – water temperature in °C.
    - Sets `haveData = true` once a valid line is parsed.
  - **Room temperature and humidity have been removed** from both parsing and transmission.

- **Wi-Fi & ThingSpeak**
  - Connects to Wi-Fi in **station mode** (`WIFI_STA`).
  - `ensureWiFi()` reconnects as needed before sending data.
  - Every 20 seconds (configurable via `timerDelay`), if `haveData` is `true`:
    - Field 1 → **waterLevel** (daily consumption / level).
    - Field 2 → **waterTemp** (°C).
    - Sends both fields together via `ThingSpeak.writeFields()`.

---

## Repository Structure

```text
.
├── main_code.ino   # Arduino Mega firmware: sensors, UI, cleaning logic, UART TX
└── esp32.ino       # ESP32 firmware: UART RX, Wi-Fi, ThingSpeak upload
```

---

## Hardware Overview

### 1. Microcontrollers

* **Arduino Mega 2560**

  * Handles all **sensors, control logic, display, keypad, and scheduling**.
  * Sends data to ESP32 over `Serial1` (hardware UART).

* **ESP32 DevKit (or equivalent)**

  * Receives data from Mega over UART.
  * Handles **Wi-Fi connection** and **ThingSpeak uploads**.

---

### 2. Sensors & actuators (Mega side)

**Pins below are taken directly from `main_code.ino`:**

* **Ultrasonic sensor (e.g. HC-SR04)**

  * `pingPin` (Trig): **D13**
  * `inPin` (Echo): **D12**

* **Vibration motors / cleaning outputs**

  * `motor1`: **D10**
  * `motor2`: **D11**

* **“UV” LED / cleaning indicator**

  * `UV_LED_PIN`: **D44**

* **Buzzer**

  * `BUZZ_PIN`: **D46**

* **Water temperature sensor (DS18B20)**

  * `ONE_WIRE_BUS`: **D42**

* **RTC module (DS3231)**

  * I²C lines on Arduino Mega:

    * SDA: **D20**
    * SCL: **D21**
  * Used for:

    * Real-time clock display.
    * Daily reset at midnight.
    * Scheduled cleaning trigger.

* **I²C 16×2 LCD**

  * Uses `LiquidCrystal_I2C lcd(0x27, 16, 2);`
  * Address: `0x27`
  * Columns: 16, Rows: 2.

* **4×4 Keypad**

  * Layout:

    ```text
    1 2 3 A
    4 5 6 B
    7 8 9 C
    * 0 # D
    ```

  * Rows: `{2, 3, 4, 5}`

  * Columns: `{6, 7, 8, 9}`

---

### 3. UART connection (Mega ↔ ESP32)

* **Arduino Mega**

  * TX1 (D18) → ESP32 RX pin (16)
  * RX1 (D19) ← ESP32 TX pin (17)

* **ESP32**

  * Uses `HardwareSerial MegaSerial(1);`
  * `RX_PIN = 16`
  * `TX_PIN = 17`

> Ensure **common GND** between Mega and ESP32, and use a **level shifter or resistor divider** from Mega TX (5 V) to ESP32 RX (3.3 V).

---

## Software Requirements

### Arduino Mega (`main_code.ino`)

Install via **Arduino Library Manager**:

* `Keypad`
* `OneWire`
* `DallasTemperature`
* `RTClib`
* `LiquidCrystal_I2C`

Board:

* **Board**: `Arduino Mega 2560`
* **Port**: your Mega’s COM port

---

### ESP32 (`esp32.ino`)

Install:

* **ESP32 board support** (via Boards Manager).
* `ThingSpeak` library.

Board:

* **Board**: your ESP32 variant (e.g. `ESP32 Dev Module`)
* **Port**: your ESP32’s COM port

---

## Configuration

### 1. Bottle geometry (Mega)

In `main_code.ino`:

```cpp
const float BOTTLE_HEIGHT_CM = 20.0;
const float BOTTLE_VOLUME_ML = 500.0;
const float ML_PER_CM        = BOTTLE_VOLUME_ML / BOTTLE_HEIGHT_CM;
```

Adjust:

* `BOTTLE_HEIGHT_CM` → actual bottle internal height (cm)
* `BOTTLE_VOLUME_ML` → usable volume (ml)

This sets a simple **linear** mapping from water height → volume.

---

### 2. Cleaning duration (Mega)

```cpp
const uint16_t CLEANING_DURATION_SEC = 20;
```

Change this to adjust how long each cleaning cycle runs (in seconds).

---

### 3. Wi-Fi & ThingSpeak (ESP32)

In `esp32.ino`, replace with your own settings:

```cpp
// Wi-Fi
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ThingSpeak
unsigned long myChannelNumber = YOUR_CHANNEL_ID;
const char * myWriteAPIKey    = "YOUR_WRITE_API_KEY";
```

Set up a ThingSpeak channel with 2 fields:

1. Water consumed today (ml)
2. Water temperature (°C)

---

## How It Works (Logic Overview)

### 1. Main loop (Mega)

The `loop()` function on the Mega:

1. Reads the current time from **RTC**.
2. Performs a **daily reset** when the day changes:

   * `waterConsumedTodayMl = 0`
   * `lastWaterVolumeMl = currentWaterVolumeMl`
   * `cleanTodayDone = false`
3. Handles the **keypad**:

   * Routes to normal mode or schedule mode.
4. Updates **motor + LED outputs** based on cleaning state.
5. Checks **scheduled cleaning**:

   * If current time matches `cleanHour:cleanMinute` exactly at second 0, and not yet done today, it starts cleaning once and sets `cleanTodayDone = true`.
6. Runs periodic tasks (timed with `millis()`):

   * Every **5 s**: ultrasonic measurement and volume/consumption update.
   * Every **5 s**: water temperature update.
   * Every **1 s**: LCD display update.
   * Every **20 s**: UART send to ESP32.

---

### 2. Ultrasonic → volume → consumption

1. **Ultrasonic measurement** (`measureUltrasonic()`):

   * Triggers the sensor.
   * Uses `pulseIn()` to get echo time.
   * Converts to `distanceCm = duration / 29.0 / 2.0`.
   * Filters out invalid readings (e.g. > 200 cm).

2. **Water height & volume** (`updateWaterFromDistance()`):

   * `waterHeight = BOTTLE_HEIGHT_CM - distanceCm`.
   * Clamps height between `0` and `BOTTLE_HEIGHT_CM`.
   * Converts height → volume:

     ```cpp
     currentWaterVolumeMl = currentWaterHeightCm * ML_PER_CM;
     ```

3. **Daily consumption**:

   * If `lastWaterVolumeMl` is not initial:

     * `diff = lastWaterVolumeMl - currentWaterVolumeMl`.
     * If `diff > 5.0` ml (threshold), adds to `waterConsumedTodayMl`.
   * Updates `lastWaterVolumeMl` to current.

> This approximates how much water has been drunk based on decreases in bottle volume over time.

---

### 3. Temperature measurement

`readWaterTemperature()`:

* Requests temperature from DS18B20.
* Stores in `waterTemp` (°C).
* Prints to Serial and displays on LCD (page 1).

---

### 4. Cleaning control

**Timer1 ISR** (`ISR(TIMER1_COMPA_vect)`):

* Configured at **1 Hz** with `setupTimer1()`.
* When `cleaningActive` is true:

  * Decrements `cleaningSecondsRemaining` each second.
  * When it reaches 0, sets `cleaningActive = false`.

**Start cleaning** (`startCleaning()`):

* Sets:

  ```cpp
  cleaningActive = true;
  cleaningSecondsRemaining = CLEANING_DURATION_SEC;
  ```

**Outputs** (`updateCleaningOutputs()`):

* Reads `cleaningActive` safely (with interrupts off).
* When state changes:

  * On **start**:

    * Beeps the buzzer.
    * Turns on `motor1`, `motor2`, `UV_LED_PIN`.
  * On **stop**:

    * Beeps again.
    * Turns all cleaning outputs off.

---

### 5. LCD pages & keypad controls

**LCD pages** (`NUM_PAGES = 3`, `displayPage`):

* **Page 0 – Time & date**

  * Line 1: `T:HH:MM:SS`
  * Line 2: `D:DD/MM/YYYY`

* **Page 1 – Water temperature**

  * Line 1: `Water Temp:`
  * Line 2: `<temp>°C`

* **Page 2 – Daily consumption & level**

  * Line 1: `Drank:<ml>ml`
  * Line 2: `Lvl:<height>cm <volume>ml`

**Keypad usage** (`handleKeypad()`):

* While **NOT** in schedule mode:

  * `1` → Start **manual cleaning** immediately.
  * `2` → Enter **cleaning schedule setup** (HHMM).
  * `A` → Previous LCD page.
  * `B` → Next LCD page.

* In **schedule mode**:

  * Enter **4 digits** for time `HHMM` (24-hour).
  * On valid time:

    * Saves `cleanHour` and `cleanMinute`.
    * Resets `cleanTodayDone = false`.
    * Shows confirmation on LCD and Serial.
  * On invalid time:

    * Shows error message on LCD.
    * Clears schedule.
  * `*` → **Cancel** schedule setup and return to normal mode.

---

### 6. Data exchange with ESP32

**On Mega** (`sendUartToESP32()`):

* Prepares values:

  ```cpp
  waterLevel = waterConsumedTodayMl;
  // waterTemp already in globals
  ```
* Sends CSV over `Serial1`:

  ```text
  waterLevel,waterTemp\n
  ```

**On ESP32** (`parseSensorLine()`):

* Splits the line at commas into four substrings.
* Converts to floats and sets global variables:

  ```cpp
  waterLevel = sLevel.toFloat();
  waterTemp  = sWTemp.toFloat();
  haveData   = true;
  ```

**ThingSpeak update** (`loop()` in esp32.ino):

* Every `timerDelay` ms (20 s), if `haveData`:

  * Calls `ensureWiFi()` to connect if needed.
  * Fills ThingSpeak fields 1–4.
  * Calls `ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);`
  * Prints status and updates `lastTime`.

---

## Getting Started

1. **Wire all components** according to the pin mappings above.
2. **Install required libraries** in Arduino IDE.
3. **Configure**:

   * Bottle geometry and cleaning duration in `main_code.ino`.
   * Wi-Fi credentials and ThingSpeak channel details in `esp32.ino`.
4. **Upload firmware**:

   * Upload `main_code.ino` to Arduino Mega 2560.
   * Upload `esp32.ino` to the ESP32.
5. **Power up both boards** (ensure common GND).
6. **Watch the LCD**:

   * You should see “Smart Bottle / Initializing”.
   * Then navigate pages with `A` / `B`.
7. **Try features**:

   * Press `1` for manual cleaning.
   * Press `2` and enter `HHMM` to set scheduled cleaning.
   * Check ThingSpeak to see uploaded data.




