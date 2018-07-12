#include <ArduinoJson.h>         // Version: 5.13.0
#include <Adafruit_NeoPixel.h>   // Version: 1.1.5
#include <EEPROM.h>

// Note: This is intended to be run on an Arduino Uno

// Constants
const int STRIP_LENGTH = 3;
const int STRIP_PIN = 12;
const int SENSOR_PIN = A0;
const int SEED_PIN = A5; // Not connected
const int MAGIC_HEADER_LENGTH = 4;
const byte MAGIC_HEADER[] = {0x12, 0x34, 0x78, 0x90};
const int TRIGGER_RECORD_SIZE = sizeof(int) + sizeof(int);

// Calculated variables
int STRIP_R;
int STRIP_G;
int STRIP_B;
int SENSOR_NORMAL;
int SENSOR_LOW;
int SENSOR_TRIGGER;
int SAVE_LOCATION;

// State/program variables
bool SYS_ERROR = false;
unsigned int TRIGGERS = 0;
bool TRIGGERED = false;
long TRIGGER_DEBOUNCE = 100;
long LAST_STOP = 0;
String COMMAND = "";
Adafruit_NeoPixel STRIP = Adafruit_NeoPixel(STRIP_LENGTH, STRIP_PIN, NEO_GRB + NEO_KHZ800);
StaticJsonBuffer<1024> JSON_BUFFER;

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SEED_PIN, INPUT);
  Serial.begin(9600);
  STRIP.begin();
  STRIP.show();

  randomSeed(analogRead(SEED_PIN));
  STRIP_R = random(20, 255);
  STRIP_G = random(20, 255);
  STRIP_B = random(50, 255);

  delay(500); // Give it a moment to set up
  calibrate();

  initMemory();
  setStripColor();
  
  printInfo();
  printStats();

  while(SYS_ERROR) {
    setStrip(255, 0, 0);
    delay(1000);
    clearStrip();
    delay(1000);
  }
}

void loop() {
  bool wasTriggered = TRIGGERED;
  int v = analogRead(SENSOR_PIN);
  if (v <= SENSOR_TRIGGER) {
    TRIGGERED = true;
  } else {
    TRIGGERED = false;
  }

  if (TRIGGERED != wasTriggered) {
    if (TRIGGERED && (millis() - LAST_STOP) > TRIGGER_DEBOUNCE) {
      trackTriggered();
      LAST_STOP = 0;
    }
    if (!TRIGGERED && LAST_STOP == 0) {
      LAST_STOP = millis();
    }
  }

  if (Serial.available() > 0) {
    COMMAND += char(Serial.read());
  }

  if (COMMAND.length() >= 3) {
    if (COMMAND == "INF") {
      printInfo();
    } else if (COMMAND == "STA") {
      printStats();
    } else if (COMMAND == "CLR") {
      clearMemory();
      printStats();
    } else if (COMMAND == "CAL") {
      calibrate();
      printInfo();
    }

    COMMAND = "";
  }
}

void setStripColor() {
  setStrip(STRIP_R, STRIP_G, STRIP_B);
}

void clearStrip() {
  setStrip(0, 0, 0);
}

void setStrip(int r, int g, int b) {
  for (int i = 0; i < STRIP_LENGTH; i++) {
    STRIP.setPixelColor(i, r, g, b);
  }
  STRIP.show();
}

void calibrate() {
  int samples = 30;
  int betweenSamples = 100;

  clearStrip();
  delay(100);
  setStripColor();

  SENSOR_NORMAL = 0;
  for (int i = 0; i < samples; i++) {
    SENSOR_NORMAL += analogRead(SENSOR_PIN);
    delay(betweenSamples);
  }
  SENSOR_NORMAL = SENSOR_NORMAL / samples;

  clearStrip();

  SENSOR_LOW = 0;
  for (int i = 0; i < samples; i++) {
    SENSOR_LOW += analogRead(SENSOR_PIN);
    delay(betweenSamples);
  }
  SENSOR_LOW = SENSOR_LOW / samples;

  SENSOR_TRIGGER = (SENSOR_LOW + SENSOR_NORMAL) / 2;

  // Do a warm up routine to indicate we're ready
  setStrip(0, 255, 0);
  delay(10000); // 10s
  setStripColor();
  delay(500);
}

void trackTriggered() {
  unsigned int now = int(millis() / 1000); // We can realtively safely do this, unless someone runs this for 56k days
  TRIGGERS++;

  EEPROM.update(SAVE_LOCATION + 0, highByte(now));
  EEPROM.update(SAVE_LOCATION + 1, lowByte(now));
  EEPROM.update(SAVE_LOCATION + 2, highByte(TRIGGERS));
  EEPROM.update(SAVE_LOCATION + 3, lowByte(TRIGGERS));

  printStats();
}

void initMemory() {
  for (int i = 0; i < MAGIC_HEADER_LENGTH; i++) {
    byte v = EEPROM.read(i);
    if (v != MAGIC_HEADER[i]) {
      clearMemory();
      break;
    }
  }

  // Find our save location
  SAVE_LOCATION = 0;
  for (int i = MAGIC_HEADER_LENGTH; i < EEPROM.length(); i++) {
    byte v = EEPROM.read(i);
    if (v == 0xFF) {
      SAVE_LOCATION = i;
      break;
    }
  }
  if (SAVE_LOCATION < MAGIC_HEADER_LENGTH || (EEPROM.length() - SAVE_LOCATION) < TRIGGER_RECORD_SIZE) {
    SYS_ERROR = true;
  }
}

void clearMemory() {
  for (int i = 0; i < EEPROM.length(); i++) {
    if (i < MAGIC_HEADER_LENGTH) {
      EEPROM.write(i, MAGIC_HEADER[i]);
    } else {
      EEPROM.write(i, 0xFF);
    }
  }
}

void readHistorical(int address, unsigned int *record) {
  byte tsHigh = EEPROM.read(address + 0);
  byte tsLow = EEPROM.read(address + 1);
  byte vHigh = EEPROM.read(address + 2);
  byte vLow = EEPROM.read(address + 3);

  if (tsHigh == 0xFF && tsLow == 0xFF && vHigh == 0xFF && vLow == 0xFF) {
    record[0] = 0;
    record[1] = 0;
    return;
  }

  unsigned int ts = (tsHigh * 256) + tsLow;
  unsigned int v = (vHigh * 256) + vLow;

  record[0] = ts;
  record[1] = v;
}

void printInfo() {
  JsonObject& root = JSON_BUFFER.createObject();
  root["schema"] = "INFO";
  
  JsonObject& sensor = root.createNestedObject("sensor");
  sensor["low"] = SENSOR_LOW;
  sensor["normal"] = SENSOR_NORMAL;
  sensor["trigger"] = SENSOR_TRIGGER;
  sensor["is_triggered"] = TRIGGERED;
  sensor["pin"] = SENSOR_PIN;

  JsonObject& strip = root.createNestedObject("strip");
  strip["length"] = STRIP_LENGTH;
  strip["pin"] = STRIP_PIN;
  strip["r"] = STRIP_R;
  strip["g"] = STRIP_G;
  strip["b"] = STRIP_B;

  root.printTo(Serial);
  JSON_BUFFER.clear();
  Serial.println("--END--");
}

void printStats() {
  JsonObject& root = JSON_BUFFER.createObject();
  root["schema"] = "STATS";

  JsonArray& records = root.createNestedArray("records");
  
  for (int i = MAGIC_HEADER_LENGTH; i < EEPROM.length(); i += TRIGGER_RECORD_SIZE) {
    unsigned int record[2];
    readHistorical(i, record);
    if (record[0] == 0) {
      break;
    }

    JsonArray& tuple = records.createNestedArray();
    tuple.add(record[0]);
    tuple.add(record[1]);
  }

  root.printTo(Serial);
  JSON_BUFFER.clear();
  Serial.println("--END--");
}

