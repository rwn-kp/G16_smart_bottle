#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <avr/interrupt.h>
#include <LiquidCrystal_I2C.h>

// Buzzer Pin 
const int BUZZ_PIN = 46;

// -------------------- LCD (I2C) --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- Pin definitions --------------------
const int pingPin = 13;   // Ultrasonic Trig
const int inPin   = 12;   // Ultrasonic Echo

// Vibration motor
const int motor1 = 10;
const int motor2 = 11;

// "UV" LED (indicator LED)
const int UV_LED_PIN = 44;

// Dallas DS18B20
#define ONE_WIRE_BUS 42

// -------------------- Keypad setup --------------------
const byte ROW_NUM    = 4;
const byte COLUMN_NUM = 4;

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte pin_rows[ROW_NUM]      = {2, 3, 4, 5};    // rows
byte pin_column[COLUMN_NUM] = {6, 7, 8, 9};    // columns

Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

// -------------------- DallasTemperature --------------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// -------------------- RTC --------------------
RTC_DS3231 rtc;
uint8_t lastDay = 0;

// -------------------- Bottle geometry --------------------
const float BOTTLE_HEIGHT_CM = 20.0;
const float BOTTLE_VOLUME_ML = 500.0;
const float ML_PER_CM        = BOTTLE_VOLUME_ML / BOTTLE_HEIGHT_CM;

float currentWaterHeightCm = 0.0;
float currentWaterVolumeMl = 0.0;
float lastWaterVolumeMl    = -1.0;
float waterConsumedTodayMl = 0.0;

// -------------------- Variables for ThingSpeak --------------------
float waterLevel = 0.0;
float waterTemp  = 0.0;
float roomTemp   = 0.0;
float humidity   = 0.0;

// -------------------- Self-cleaning via Timer1 interrupt --------------------
volatile bool     cleaningActive           = false;
volatile uint16_t cleaningSecondsRemaining = 0;
const uint16_t    CLEANING_DURATION_SEC   = 20;

// -------------------- Display pages --------------------
int displayPage = 0;
const int NUM_PAGES = 3;
int lastDisplayPage  = -1;
int lastLogicalPage  = -1;

// -------------------- Scheduling clean time --------------------
bool  scheduleMode   = false;  // true while user is typing HHMM
char  scheduleBuffer[5];
int   scheduleIndex  = 0;
int   cleanHour      = -1;     // 0–23, -1 = not set
int   cleanMinute    = -1;     // 0–59
bool  cleanTodayDone = false;  // true when today's scheduled clean has run

// -------------------- Timing --------------------
unsigned long lastUltrasonicMeasure = 0;
const unsigned long ULTRASONIC_INTERVAL_MS = 5000;

unsigned long lastTempMeasure = 0;
const unsigned long TEMP_INTERVAL_MS = 5000;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 1000;

unsigned long lastUartSend = 0;
const unsigned long UART_INTERVAL_MS = 20000;

// -------------------- Function declarations --------------------
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
  Serial1.begin(9600);   // to ESP32

  pinMode(pingPin, OUTPUT);
  pinMode(inPin,   INPUT);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(UV_LED_PIN, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);

  digitalWrite(motor1, LOW);
  digitalWrite(motor2, LOW);
  digitalWrite(UV_LED_PIN, LOW);

  // LCD init (แบบที่คุณใช้ได้)
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Bottle");
  lcd.setCursor(0, 1);
  lcd.print("Initializing");

  sensors.begin();

  if (!rtc.begin()) {
    Serial.println("RTC not found.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RTC error");
  }

  setupTimer1();

  Serial.println("Smart bottle (Mega) started.");
}

// -------------------- Loop --------------------
void loop() {
  DateTime now = rtc.now();

  // Daily reset
  if (lastDay == 0) {
    lastDay = now.day();
  } else if (now.day() != lastDay) {
    waterConsumedTodayMl = 0.0;
    lastWaterVolumeMl    = currentWaterVolumeMl;
    lastDay              = now.day();
    cleanTodayDone       = false;      // allow scheduled clean again in new day
  }

  // 1) Keypad
  handleKeypad();

  // 2) Motor + LED
  updateCleaningOutputs();

  // 3) Scheduled cleaning check (when not in scheduleMode)
  if (!scheduleMode && !cleaningActive && cleanHour >= 0 && !cleanTodayDone) {
    if (now.hour() == cleanHour &&
        now.minute() == cleanMinute &&
        now.second() == 0) {
      startCleaning();
      cleanTodayDone = true;
      Serial.println("Scheduled cleaning started.");
    }
  }

  // 4) Periodic tasks
  unsigned long currentMillis = millis();

  if (currentMillis - lastUltrasonicMeasure >= ULTRASONIC_INTERVAL_MS) {
    lastUltrasonicMeasure = currentMillis;
    measureUltrasonic();
  }

  if (currentMillis - lastTempMeasure >= TEMP_INTERVAL_MS) {
    lastTempMeasure = currentMillis;
    readWaterTemperature();
  }

  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = currentMillis;
    updateDisplay(now);
  }

  if (currentMillis - lastUartSend >= UART_INTERVAL_MS) {
    lastUartSend = currentMillis;
    sendUartToESP32();
  }
}

// -------------------- Timer1 configuration --------------------
void setupTimer1() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 15624;                       // 1 Hz
  TCCR1B |= (1 << WGM12);              // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024
  TIMSK1 |= (1 << OCIE1A);             // interrupt

  sei();
}

// -------------------- Start cleaning (manual + schedule) --------------------
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

  // ถ้าอยู่ในโหมดตั้งเวลา ให้โฟกัสที่การกรอก HHMM เท่านั้น
  if (scheduleMode) {
    if (key >= '0' && key <= '9') {
      if (scheduleIndex < 4) {
        scheduleBuffer[scheduleIndex] = key;
        scheduleIndex++;

        // แสดงตัวเลขบน LCD
        lcd.setCursor(6 + scheduleIndex - 1, 1); // ตำแหน่งหลัง "HHMM: "
        lcd.print(key);
      }

      // กรอกครบ 4 ตัวแล้ว -> parse เวลา
      if (scheduleIndex == 4) {
        int HH = (scheduleBuffer[0] - '0') * 10 + (scheduleBuffer[1] - '0');
        int MM = (scheduleBuffer[2] - '0') * 10 + (scheduleBuffer[3] - '0');

        if (HH >= 0 && HH < 24 && MM >= 0 && MM < 60) {
          cleanHour      = HH;
          cleanMinute    = MM;
          cleanTodayDone = false;   // ให้รันได้อีกครั้งในวันนี้ถ้ายังไม่ถึงเวลา

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
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Invalid time");
          lcd.setCursor(0, 1);
          lcd.print("Try again");
          Serial.println("Invalid cleaning time input.");
          cleanHour   = -1;
          cleanMinute = -1;
        }

        // ออกจากโหมดตั้งเวลา หลังจากแสดงผลชั่วครู่
        delay(1500);
        scheduleMode  = false;
        scheduleIndex = 0;
      }
    } else if (key == '*') {
      // ยกเลิกการตั้งเวลา
      scheduleMode  = false;
      scheduleIndex = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set time cancel");
      Serial.println("Cleaning time setup canceled.");
      delay(1000);
    }

    return; // ไม่ไปทำเคสอื่นเมื่ออยู่ในโหมดตั้งเวลา
  }

  // ---------- โหมดปกติ ----------
  if (key == '1') {
    // Manual clean start
    startCleaning();
  } else if (key == '2') {
    // เข้าโหมดตั้งเวลาทำความสะอาด
    scheduleMode  = true;
    scheduleIndex = 0;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Clean Time");
    lcd.setCursor(0, 1);
    lcd.print("HHMM: ");
    Serial.println("Enter cleaning time (HHMM).");
  } else if (key == 'A') {
    displayPage--;
    if (displayPage < 0) displayPage = NUM_PAGES - 1;
  } else if (key == 'B') {
    displayPage++;
    if (displayPage >= NUM_PAGES) displayPage = 0;
  }
}

// -------------------- Motor + LED control --------------------
void updateCleaningOutputs() {
  static bool lastState = false;

  bool state;
  uint16_t remaining;
  noInterrupts();
  state     = cleaningActive;
  remaining = cleaningSecondsRemaining;
  interrupts();

  if (state != lastState) {
    if (state) {
      tone(BUZZ_PIN, 4000);
      delay(1000);
      noTone(BUZZ_PIN);
      digitalWrite(motor1, HIGH);
      digitalWrite(motor2, HIGH);
      digitalWrite(UV_LED_PIN, HIGH);
      Serial.println("Cleaning started.");
    } else {
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
void measureUltrasonic() {
  long duration;
  float distanceCm;

  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  duration = pulseIn(inPin, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Ultrasonic: no echo");
    return;
  }

  distanceCm = duration / 29.0 / 2.0;

  if (distanceCm < 0 || distanceCm > 200) {
    Serial.println("Ultrasonic: invalid dist");
    return;
  }

  updateWaterFromDistance(distanceCm);
}

// -------------------- Volume & consumption --------------------
void updateWaterFromDistance(float distanceCm) {
  float waterHeight = BOTTLE_HEIGHT_CM - distanceCm;
  if (waterHeight < 0) waterHeight = 0;
  if (waterHeight > BOTTLE_HEIGHT_CM) waterHeight = BOTTLE_HEIGHT_CM;

  currentWaterHeightCm = waterHeight;
  currentWaterVolumeMl = currentWaterHeightCm * ML_PER_CM;

  if (lastWaterVolumeMl < 0.0) {
    lastWaterVolumeMl = currentWaterVolumeMl;
    return;
  }

  float diff = lastWaterVolumeMl - currentWaterVolumeMl;

  if (diff > 5.0) {
    waterConsumedTodayMl += diff;
  }

  lastWaterVolumeMl = currentWaterVolumeMl;
}

// -------------------- Temperature --------------------
void readWaterTemperature() {
  sensors.requestTemperatures();
  waterTemp = sensors.getTempCByIndex(0);

  Serial.print("Water T: ");
  Serial.print(waterTemp, 1);
  Serial.println(" C");
}

// -------------------- LCD display --------------------
void updateDisplay(const DateTime& now) {
  // ถ้ากำลังตั้งเวลาอยู่ ปล่อยให้หน้าจอเป็นของ handleKeypad
  if (scheduleMode) return;

  // เช็คสถานะ cleaning ก่อน
  bool state;
  uint16_t remaining;
  noInterrupts();
  state     = cleaningActive;
  remaining = cleaningSecondsRemaining;
  interrupts();

  if (state) {
    // Cleaning mode: โชว์เวลาเหลืออย่างเดียว
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
    lcd.clear();
    lastLogicalPage = displayPage;
    lastDisplayPage = displayPage;
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  if (displayPage == 0) {
    // Time + Date (with year)
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
    // Water temperature
    lcd.setCursor(0, 0);
    lcd.print("Water Temp:     ");
    lcd.setCursor(0, 1);
    lcd.print(waterTemp, 1);
    lcd.print((char)223);
    lcd.print("C           ");

  } else if (displayPage == 2) {
    // Water consumption + level
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
void sendUartToESP32() {
  waterLevel = waterConsumedTodayMl;

  Serial1.print(waterLevel, 1);
  Serial1.print(',');
  Serial1.print(waterTemp, 1);
  Serial1.print(',');
  Serial1.print(roomTemp, 1);
  Serial1.print(',');
  Serial1.print(humidity, 1);
  Serial1.println();

  Serial.print("UART->ESP32: ");
  Serial.print(waterLevel, 1);  Serial.print(',');
  Serial.print(waterTemp, 1);   Serial.print(',');
  Serial.print(roomTemp, 1);    Serial.print(',');
  Serial.println(humidity, 1);
}

