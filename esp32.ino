#include <WiFi.h>
#include "ThingSpeak.h"
#include <HardwareSerial.h>

// ---------- Wi-Fi ----------
const char* ssid     = "lumberjack";
const char* password = "ymtc5022";

// ---------- ThingSpeak ----------
WiFiClient client;
unsigned long myChannelNumber = 3148064;          // <-- your channel ID
const char * myWriteAPIKey    = "PYHYSTPT6A0BFD51"; // <-- your Write API key

// Timer (like the tutorial)
unsigned long lastTime   = 0;
unsigned long timerDelay = 20000;  // 20 seconds (>= 15s to follow ThingSpeak limit)

// ---------- UART from Arduino Mega ----------
HardwareSerial MegaSerial(1);
const int RX_PIN = 16;   // ESP32 RX  <-- Mega TX1 (18 via level shifter/divider)
const int TX_PIN = 17;   // ESP32 TX  --> Mega RX1 (19)

// Latest sensor values from Mega
float waterLevel = 0;
float waterTemp  = 0;
float roomTemp   = 0;
float humidity   = 0;
bool  haveData   = false;

// ----------------- Parse CSV line from Mega -----------------
void parseSensorLine(String line) {
  Serial.print("Raw line: ");
  Serial.println(line);

  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);

  if (idx1 < 0 || idx2 < 0 || idx3 < 0) {
    Serial.println("Invalid CSV format from Mega");
    return;
  }

  String sLevel = line.substring(0, idx1);
  String sWTemp = line.substring(idx1 + 1, idx2);
  String sRTemp = line.substring(idx2 + 1, idx3);
  String sHum   = line.substring(idx3 + 1);

  waterLevel = sLevel.toFloat();
  waterTemp  = sWTemp.toFloat();
  roomTemp   = sRTemp.toFloat();
  humidity   = sHum.toFloat();
  haveData   = true;

  Serial.print("Parsed -> Level: "); Serial.print(waterLevel);
  Serial.print("  Water T: ");       Serial.print(waterTemp);
  Serial.print("  Room T: ");        Serial.print(roomTemp);
  Serial.print("  Hum: ");           Serial.println(humidity);
}

// ----------------- Wi-Fi connect (same style as RNT) -----------------
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Attempting to connect");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(5000);
    Serial.print(".");
  }
  Serial.println("\nConnected.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // UART with Mega
  MegaSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  Serial.println("ESP32 + ThingSpeak (Mega data) started");
}

void loop() {
  // 1) Read lines from Mega and parse
  static String buffer = "";
  while (MegaSerial.available()) {
    char c = (char)MegaSerial.read();
    if (c == '\n') {
      parseSensorLine(buffer);
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
    }
  }

  // 2) Every timerDelay ms, send latest values to ThingSpeak
  if ((millis() - lastTime) > timerDelay && haveData) {

    ensureWiFi();   // connect/reconnect Wi-Fi like the tutorial

    // Set ThingSpeak fields (map as you like)
    // Field 1: water level
    // Field 2: water temperature
    // Field 3: room temperature
    // Field 4: humidity
    ThingSpeak.setField(1, waterLevel);
    ThingSpeak.setField(2, waterTemp);
    ThingSpeak.setField(3, roomTemp);
    ThingSpeak.setField(4, humidity);

    // Send all fields in one request
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    if (x == 200) {
      Serial.println("Channel update successful.");
    } else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }

    lastTime = millis();
  }
}
