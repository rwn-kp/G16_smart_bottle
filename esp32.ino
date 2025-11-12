#include <WiFi.h>
#include "ThingSpeak.h"
#include <HardwareSerial.h>

// ---------- Wi-Fi ----------
// Replace with your own Wi-Fi credentials
const char* ssid     = "lumberjack";
const char* password = "ymtc5022";

// ---------- ThingSpeak ----------
// Configure your ThingSpeak channel here
WiFiClient client;
unsigned long myChannelNumber = 3148064;            // <-- your channel ID
const char * myWriteAPIKey    = "PYHYSTPT6A0BFD51"; // <-- your Write API key

// Timer for periodic uploads (similar to ThingSpeak examples)
unsigned long lastTime   = 0;
unsigned long timerDelay = 20000;  // 20 seconds (>= 15s to follow ThingSpeak limit)

// ---------- UART from Arduino Mega ----------
// Use HardwareSerial(1) on ESP32 to receive data from Arduino Mega
HardwareSerial MegaSerial(1);
const int RX_PIN = 16;   // ESP32 RX  <-- Mega TX1 (18 via level shifter/divider)
const int TX_PIN = 17;   // ESP32 TX  --> Mega RX1 (19)

// Latest sensor values from Mega
// Mega sends a CSV line in the format: waterLevel,waterTemp\n
float waterLevel = 0;   // Daily water consumption or level value from Mega
float waterTemp  = 0;   // Water temperature from Mega
bool  haveData   = false; // True once at least one valid line has been parsed

// ----------------- Parse CSV line from Mega -----------------
// Expected format: "<waterLevel>,<waterTemp>\n"
void parseSensorLine(String line) {
  Serial.print("Raw line: ");
  Serial.println(line);

  int idx1 = line.indexOf(',');

  // If there is no comma, the format is invalid
  if (idx1 < 0) {
    Serial.println("Invalid CSV format from Mega");
    return;
  }

  // Split into two fields: level and water temperature
  String sLevel = line.substring(0, idx1);
  String sWTemp = line.substring(idx1 + 1);

  waterLevel = sLevel.toFloat();
  waterTemp  = sWTemp.toFloat();
  haveData   = true;

  Serial.print("Parsed -> Level: ");
  Serial.print(waterLevel);
  Serial.print("  Water T: ");
  Serial.println(waterTemp);
}

// ----------------- Wi-Fi connect (same style as RNT) -----------------
// Ensure Wi-Fi is connected before sending data to ThingSpeak
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

  // UART connection with Arduino Mega (9600 baud, 8N1)
  MegaSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Put ESP32 into station mode and initialize ThingSpeak
  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(client);  // Initialize ThingSpeak client

  Serial.println("ESP32 + ThingSpeak (Mega data) started");
}

void loop() {
  // 1) Read lines from Mega and parse them into numeric values
  static String buffer = "";
  while (MegaSerial.available()) {
    char c = (char)MegaSerial.read();
    if (c == '\n') {
      // End of line reached: parse the accumulated buffer
      parseSensorLine(buffer);
      buffer = "";
    } else if (c != '\r') {
      // Build up the current line (ignore carriage return)
      buffer += c;
    }
  }

  // 2) Every timerDelay ms, send latest values to ThingSpeak
  if ((millis() - lastTime) > timerDelay && haveData) {

    // Connect/reconnect Wi-Fi if needed
    ensureWiFi();

    // Set ThingSpeak fields (map as you like)
    // Field 1: water level / consumption
    // Field 2: water temperature
    ThingSpeak.setField(1, waterLevel);
    ThingSpeak.setField(2, waterTemp);

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
