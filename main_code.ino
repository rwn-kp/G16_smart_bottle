#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <avr/interrupt.h>
#include <LiquidCrystal_I2C.h>

// Buzzer Pin (piezo buzzer for audio feedback)
const int BUZZ_PIN = 46;

// -------------------- LCD (I2C) --------------------
// 16x2 LCD using I2C interface at address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- Pin definitions --------------------
// Ultrasonic sensor pins
const int pingPin = 13;   // Ultrasonic Trig
const int inPin   = 12;   // Ultrasonic Echo

// Vibration motors for cleaning
const int motor1 = 10;
const int motor2 = 11;

// "UV" LED (indicator LED for cleaning)
const int UV_LED_PIN = 44;

// Dallas DS18B20 (water temperature sensor)
#define ONE_WIRE_BUS 42

// -------------------- Keypad setup --------------------
// 4x4 matrix keypad configuration
const byte ROW_NUM    = 4;
const byte COLUMN_NUM = 4;

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// Keypad row and column pins on the Mega
byte pin_rows[ROW_NUM]      = {2, 3, 4, 5};    // Row pins
byte pin_column[COLUMN_NUM] = {6, 7, 8, 9};    // Column pins

Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

// -------------------- DallasTemperature --------------------
// OneWire + DallasTemperature for DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// -------------------- RTC --------------------
// DS3231 real-time clock
RTC_DS3231 rtc;
uint8_t lastDay = 0;   // Used to detect day change for daily reset

// -------------------- Bottle geometry --------------------
// Simple linear model: volume ≈ height * (ml per cm)
const float BOTTLE_HEIGHT_CM = 20.0;
const float BOTTLE_VOLUME_ML = 500.0;
const float ML_PER_CM        = BOTTLE_VOLUME_ML / BOTTLE_HEIGHT_CM;

float currentWaterHeightCm = 0.0;   // Current water height in cm
float currentWaterVolumeMl = 0.0;   // Current estimated volume in ml
float lastWaterVolumeMl    = -1.0;  // Last volume used to calculate consumption
float waterConsumedTodayMl = 0.0;   // Daily total consumption in ml

// -------------------- Variables for ThingSpeak / UART --------------------
// These values are prepared to be sent to ESP32 (and then to cloud)
float waterLevel = 0.0;   // Used as "water consumed today" for now
float waterTemp  = 0.0;   // Water temperature in °C

// -------------------- Self-cleaning via Timer1 interrupt --------------------
// Timer1 provides a 1-second tick to track cleaning duration
volatile bool     cleaningActive           = false;
volatile uint16_t cleaningSecondsRemaining = 0;
const uint16_t    CLEANING_DURATION_SEC   = 20;  // Default cleaning duration (seconds)

// -------------------- Display pages --------------------
// displayPage: which logical screen is shown in normal mode
int displayPage = 0;
const int NUM_PAGES = 3;
int lastDisplayPage  = -1;   // Last page actually rendered on LCD
int lastLogicalPage  = -1;   // Last logical page index (for clearing when page changes)

// -------------------- Scheduling clean time --------------------
// Variables for daily scheduled cleaning (user inputs HHMM)
bool  scheduleMode   = false;  // true while user is typing HHMM
char  scheduleBuffer[5];
int   scheduleIndex  = 0;
int   cleanHour      = -1;     // 0–23, -1 = not set
int   cleanMinute    = -1;     // 0–59
bool  cleanTodayDone = false;  // true when today's scheduled clean has run

// -------------------- Timing --------------------
// Non-blocking timing using millis()
unsigned long lastUltrasonicMeasure = 0;
const unsigned long ULTRASONIC_INTERVAL_MS = 5000;   // Ultrasonic measurement interval

unsigned long lastTempMeasure = 0;
const unsigned long TEMP_INTERVAL_MS = 5000;          // Temperature measurement interval

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 1000;       // LCD refresh interval

unsigned long lastUartSend = 0;
const unsigned long UART_INTERVAL_MS = 20000;         // UART send interval

// -------------------- Function declarations --------------------
// Timer setup and core logic handlers
void setupTimer1();
void handleKeypad();
void updateCleaningOutputs();
void measureUltrasonic();
void updateWaterFromDistance(float distanceCm);
void readWaterTemperature();
void updateDisplay(const DateTime& now);
void sendUartToESP32();
void startCleaning();

// -------------------- Timer1 ISR: 1 second tick --------------------
// Called every 1 second to update remaining cleaning time
ISR(TIMER1_COMPA_vect) {
  if (cleaningActive && cleaningSecondsRemaining > 0) {
    cleaningSecondsRemaining--;
    if (cleaningSecondsRemaining == 0) {
      cleaningActive = false;
    }
  }
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);   // Serial1: communication to ESP32

  // Ultrasonic pins
  pinMode(pingPin, OUTPUT);
  pinMode(inPin,   INPUT);

  // Cleaning outputs
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(UV_LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);

  // Ensure all outputs are off at startup
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  digitalWrite(UV_LED_PIN, LOW);

  // LCD initialization (compatible with this LiquidCrystal_I2C library)
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Bottle");
  lcd.setCursor(0, 1);
  lcd.print("Initializing");

  // Start DS18B20 temperature sensor
  sensors.begin();

  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RTC error");
  }

  // Configure Timer1 for 1 Hz interrupt
  setupTimer1();

  Serial.println("Smart bottle (Mega) started.");
}

// -------------------- Loop --------------------
void loop() {
  // Get current time from RTC
  DateTime now = rtc.now();

  // Daily reset logic (reset consumption when day changes)
  if (lastDay == 0) {
    lastDay = now.day();
  } else if (now.day() != lastDay) {
    waterConsumedTodayMl = 0.0;
    lastWaterVolumeMl    = currentWaterVolumeMl;
    lastDay              = now.day();
    cleanTodayDone       = false;      // Allow scheduled clean again in new day
  }

  // 1) Handle keypad input (mode switching, page change, manual/scheduled cleaning)
  handleKeypad();

  // 2) Update cleaning outputs (motors, UV LED, buzzer) based on cleaning state
  updateCleaningOutputs();

  // 3) Scheduled cleaning check (only when not in schedule mode)
  if (!scheduleMode && !cleaningActive && cleanHour >= 0 && !cleanTodayDone) {
    // Start cleaning exactly at cleanHour:cleanMinute:00
    if (now.hour() == cleanHour &&
        now.minute() == cleanMinute &&
        now.second() == 0) {
      startCleaning();
      cleanTodayDone = true;
      Serial.println("Scheduled cleaning started.");
    }
  }

  // 4) Periodic tasks using millis()
  unsigned long currentMillis = millis();

  // Ultrasonic measurement
  if (currentMillis - lastUltrasonicMeasure >= ULTRASONIC_INTERVAL_MS) {
    lastUltrasonicMeasure = currentMillis;
    measureUltrasonic();
  }

  // Temperature measurement
  if (currentMillis - lastTempMeasure >= TEMP_INTERVAL_MS) {
    lastTempMeasure = currentMillis;
    readWaterTemperature();
  }

  // LCD update
  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = currentMillis;
    updateDisplay(now);
  }

  // UART send to ESP32
  if (currentMillis - lastUartSend >= UART_INTERVAL_MS) {
    lastUartSend = currentMillis;
    sendUartToESP32();
  }
}

// -------------------- Timer1 configuration --------------------
// Configure Timer1 to generate a 1 Hz interrupt (CTC mode)
void setupTimer1() {
  cli();                   // Disable global interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 15624;                       // Compare value for 1 Hz at 16 MHz with prescaler 1024
  TCCR1B |= (1 << WGM12);              // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);             // Enable Timer1 compare interrupt

  sei();                   // Enable global interrupts
}

// -------------------- Start cleaning (manual + schedule) --------------------
// Arms the cleaning process and sets remaining time
void startCleaning() {
  noInterrupts();
  cleaningActive = true;
  cleaningSecondsRemaining = CLEANING_DURATION_SEC;
  interrupts();
}

// -------------------- Keypad handling --------------------
void handleKeypad() {
  char key = keypad.getKey();
  if (!key) return;

  Serial.print("Key pressed: ");
  Serial.println(key);

  // If we are in schedule mode, focus only on HHMM input
  if (scheduleMode) {
    if (key >= '0' && key <= '9') {
      if (scheduleIndex < 4) {
        scheduleBuffer[scheduleIndex] = key;
        scheduleIndex++;

        // Show digit on LCD (position after "HHMM: ")
        lcd.setCursor(6 + scheduleIndex - 1, 1);
        lcd.print(key);
      }

      // When 4 digits are entered -> parse as time HHMM
      if (scheduleIndex == 4) {
        int HH = (scheduleBuffer[0] - '0') * 10 + (scheduleBuffer[1] - '0');
        int MM = (scheduleBuffer[2] - '0') * 10 + (scheduleBuffer[3] - '0');

        if (HH >= 0 && HH < 24 && MM >= 0 && MM < 60) {
          cleanHour      = HH;
          cleanMinute    = MM;
          cleanTodayDone = false;   // Allow running again today if time has not passed

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Clean time set");
          lcd.setCursor(0, 1);
          if (HH < 10) lcd.print('0');
          lcd.print(HH);
          lcd.print(':');
          if (MM < 10) lcd.print('0');
          lcd.print(MM);

          Serial.print("Cleaning time set to ");
          Serial.print(HH);
          Serial.print(":");
          Serial.println(MM);
        } else {
          // Invalid time entered
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Invalid time");
          lcd.setCursor(0, 1);
          lcd.print("Try again");
          Serial.println("Invalid cleaning time input.");
          cleanHour   = -1;
          cleanMinute = -1;
        }

        // Exit schedule mode after a short display delay
        delay(1500);
        scheduleMode  = false;
        scheduleIndex = 0;
      }
    } else if (key == '*') {
      // Cancel schedule setting
      scheduleMode  = false;
      scheduleIndex = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set time cancel");
      Serial.println("Cleaning time setup canceled.");
      delay(1000);
    }

    // While in scheduleMode, do not process other keys
    return;
  }

  // ---------- Normal mode ----------
  if (key == '1') {
    // Manual clean start
    startCleaning();
  } else if (key == '2') {
    // Enter cleaning schedule setup mode
    scheduleMode  = true;
    scheduleIndex = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Clean Time");
    lcd.setCursor(0, 1);
    lcd.print("HHMM: ");
    Serial.println("Enter cleaning time (HHMM).");
  } else if (key == 'A') {
    // Previous display page
    displayPage--;
    if (displayPage < 0) displayPage = NUM_PAGES - 1;
  } else if (key == 'B') {
    // Next display page
    displayPage++;
    if (displayPage >= NUM_PAGES) displayPage = 0;
  }
}

// -------------------- Motor + LED control --------------------
// Handles transitions into and out of cleaning state (buzzer + motors + UV LED)
void updateCleaningOutputs() {
  static bool lastState = false;

  bool state;
  uint16_t remaining;
  noInterrupts();
  state     = cleaningActive;
  remaining = cleaningSecondsRemaining;
  interrupts();

  // Only act when state changes
  if (state != lastState) {
    if (state) {
      // Cleaning started: beep once and turn everything on
      tone(BUZZ_PIN, 4000);
      delay(1000);
      noTone(BUZZ_PIN);
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(UV_LED_PIN, HIGH);
      Serial.println("Cleaning started.");
    } else {
      // Cleaning finished: beep once and turn everything off
      tone(BUZZ_PIN, 4000);
      delay(1000);
      noTone(BUZZ_PIN);
      digitalWrite(motor1, LOW);
      digitalWrite(motor2, LOW);
      digitalWrite(UV_LED_PIN, LOW);
      Serial.println("Cleaning finished.");
    }
    lastState = state;
  }
}

// -------------------- Ultrasonic --------------------
// Trigger ultrasonic sensor and convert echo time to distance in cm
void measureUltrasonic() {
  long duration;
  float distanceCm;

  // Send trigger pulse
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // Measure echo with timeout (30 ms)
  duration = pulseIn(inPin, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Ultrasonic: no echo");
    return;
  }

  // Convert time to distance (approx. speed of sound in air)
  distanceCm = duration / 29.0 / 2.0;

  // Simple validity check
  if (distanceCm < 0 || distanceCm > 200) {
    Serial.println("Ultrasonic: invalid dist");
    return;
  }

  updateWaterFromDistance(distanceCm);
}

// -------------------- Volume & consumption --------------------
// Convert distance to water height and volume, then update daily consumption
void updateWaterFromDistance(float distanceCm) {
  // Height = bottle height - measured distance
  float waterHeight = BOTTLE_HEIGHT_CM - distanceCm;
  if (waterHeight < 0) waterHeight = 0;
  if (waterHeight > BOTTLE_HEIGHT_CM) waterHeight = BOTTLE_HEIGHT_CM;

  currentWaterHeightCm = waterHeight;
  currentWaterVolumeMl = currentWaterHeightCm * ML_PER_CM;

  // Initialization: first valid reading
  if (lastWaterVolumeMl < 0.0) {
    lastWaterVolumeMl = currentWaterVolumeMl;
    return;
  }

  // Compute volume difference to estimate consumption
  float diff = lastWaterVolumeMl - currentWaterVolumeMl;

  // Only count significant decreases to avoid noise (threshold = 5 ml)
  if (diff > 5.0) {
    waterConsumedTodayMl += diff;
  }

  lastWaterVolumeMl = currentWaterVolumeMl;
}

// -------------------- Temperature --------------------
// Read water temperature from DS18B20 and store in waterTemp
void readWaterTemperature() {
  sensors.requestTemperatures();
  waterTemp = sensors.getTempCByIndex(0);

  Serial.print("Water T: ");
  Serial.print(waterTemp, 1);
  Serial.println(" C");
}

// -------------------- LCD display --------------------
// Update LCD based on current mode and selected display page
void updateDisplay(const DateTime& now) {
  // If we are in schedule mode, let handleKeypad control the LCD
  if (scheduleMode) return;

  // Check cleaning status first
  bool state;
  uint16_t remaining;
  noInterrupts();
  state     = cleaningActive;
  remaining = cleaningSecondsRemaining;
  interrupts();

  if (state) {
    // Cleaning mode: show only remaining time
    if (lastDisplayPage != 100) {
      lcd.clear();
      lastDisplayPage = 100;
    }

    uint16_t mins = remaining / 60;
    uint16_t secs = remaining % 60;

    lcd.setCursor(0, 0);
    lcd.print("Cleaning...     ");
    lcd.setCursor(0, 1);
    lcd.print("Left: ");
    if (mins < 10) lcd.print('0');
    lcd.print(mins);
    lcd.print(':');
    if (secs < 10) lcd.print('0');
    lcd.print(secs);
    lcd.print("   ");

    Serial.print("Cleaning, left=");
    Serial.print(mins);
    Serial.print("m ");
    Serial.print(secs);
    Serial.println("s");
    return;
  }

  // Normal display pages
  if (displayPage != lastLogicalPage) {
    // Page changed: clear LCD once
    lcd.clear();
    lastLogicalPage = displayPage;
    lastDisplayPage = displayPage;
  }

  // Clear both lines before drawing new content
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  if (displayPage == 0) {
    // Page 0: Time + Date (with year)
    lcd.setCursor(0, 0);
    lcd.print("T:");
    if (now.hour() < 10) lcd.print('0');
    lcd.print(now.hour());
    lcd.print(':');
    if (now.minute() < 10) lcd.print('0');
    lcd.print(now.minute());
    lcd.print(':');
    if (now.second() < 10) lcd.print('0');
    lcd.print(now.second());

    lcd.setCursor(0, 1);
    lcd.print("D:");
    if (now.day() < 10) lcd.print('0');
    lcd.print(now.day());
    lcd.print('/');
    if (now.month() < 10) lcd.print('0');
    lcd.print(now.month());
    lcd.print('/');
    lcd.print(now.year());

  } else if (displayPage == 1) {
    // Page 1: Water temperature
    lcd.setCursor(0, 0);
    lcd.print("Water Temp:     ");
    lcd.setCursor(0, 1);
    lcd.print(waterTemp, 1);
    lcd.print((char)223);  // Degree symbol
    lcd.print("C           ");

  } else if (displayPage == 2) {
    // Page 2: Water consumption + current level
    lcd.setCursor(0, 0);
    lcd.print("Drank:");
    lcd.print((int)waterConsumedTodayMl);
    lcd.print("ml      ");

    lcd.setCursor(0, 1);
    lcd.print("Lvl:");
    lcd.print(currentWaterHeightCm, 1);
    lcd.print("cm ");
    lcd.print((int)currentWaterVolumeMl);
    lcd.print("ml");
  }

  Serial.print("Page ");
  Serial.print(displayPage);
  Serial.print(" | Drank=");
  Serial.print(waterConsumedTodayMl, 0);
  Serial.print("ml | T=");
  Serial.print(waterTemp, 1);
  Serial.println("C");
}

// -------------------- UART to ESP32 --------------------
// Send selected data to ESP32 over Serial1
void sendUartToESP32() {
  waterLevel = waterConsumedTodayMl;

  Serial1.print(waterLevel, 1);
  Serial1.print(',');
  Serial1.print(waterTemp, 1);
  Serial1.println();

  Serial.print("UART->ESP32: ");
  Serial.print(waterLevel, 1);  Serial.print(',');
  Serial.print(waterTemp, 1);
}
