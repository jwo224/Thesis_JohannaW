#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "esp_wifi.h"

// =========================
// ROBOT WIFI SETTINGS
// =========================

const char* WIFI_SSID = "GL-AR300M-d28";
const char* WIFI_PASSWORD = "goodlife";

// =========================
// ENERGY SAVING SETTINGS
// =========================

const bool ENERGY_SAVING_MODE = true;

// =========================
// PIN SETTINGS
// =========================

const int SERVO_PIN = 3;
const int FSR_PIN = 4;

// Try GPIO8 first for ESP32-C3 onboard LED.
// If it does not blink, try GPIO10 or LED_BUILTIN.
const int LED_PIN = 8;

// =========================
// SERVO SETTINGS
// =========================

const int START_ANGLE = 180;
const int END_ANGLE = 0;

const int STEP_SIZE = 1;
const int STEP_DELAY = 20;  // ms, increase for slower movement

// =========================
// FSR SETTINGS
// =========================

const int FSR_THRESHOLD = 50;

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

// =========================
// HOLD SETTINGS
// =========================

const int HOLD_TIME_MS = 1000;

// =========================
// WIFI RECONNECT SETTINGS
// =========================

unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 5000;

// =========================
// OBJECTS
// =========================

Servo myServo;
WebServer server(80);

// =========================
// STATE VARIABLES
// =========================

int currentAngle = START_ANGLE;
bool isPressing = false;

int lastFsrRaw = 0;
float lastFsrVoltage = 0.0;
int lastTriggerAngle = START_ANGLE;
bool lastPressReachedThreshold = false;

// =========================
// SETUP
// =========================

void setup() {
  delay(1000);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-C3 FSR Servo Controller - API Mode");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(START_ANGLE);
  currentAngle = START_ANGLE;

  connectToRobotWifi();

  server.on("/", handleRoot);
  server.on("/press", handlePress);
  server.on("/status", handleStatus);
  server.on("/fsr", handleFsr);

  server.begin();

  Serial.println();
  Serial.println("API server started.");
  Serial.print("Base URL: http://");
  Serial.println(WiFi.localIP());
}

// =========================
// MAIN LOOP
// =========================

void loop() {
  maintainWifiConnection();

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // LED always off for battery saving
  digitalWrite(LED_PIN, LOW);

  // No continuous FSR reading here.
  // FSR is only read in /press, /fsr, and /status.

  delay(2);
}

// =========================
// WIFI RECONNECT LOGIC
// =========================

void maintainWifiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();

  if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS) {
    lastReconnectAttempt = now;

    Serial.println("WiFi lost. Trying to reconnect...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// =========================
// CONNECT TO ROBOT WIFI
// =========================

void connectToRobotWifi() {
  Serial.println();
  Serial.println("Connecting ESP32-C3 to robot Wi-Fi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to robot Wi-Fi.");
    Serial.print("Wi-Fi name: ");
    Serial.println(WIFI_SSID);
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi RSSI dBm: ");
    Serial.println(WiFi.RSSI());

    if (ENERGY_SAVING_MODE) {
      WiFi.setSleep(true);
      esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
      Serial.println("Energy saving: WiFi modem sleep enabled.");
    } else {
      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);
      Serial.println("Normal mode: WiFi power save disabled.");
    }

    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("Failed to connect to robot Wi-Fi.");
    digitalWrite(LED_PIN, LOW);
  }
}

// =========================
// API HANDLERS
// =========================

void handleRoot() {
  String response = "";

  response += "ESP32-C3 FSR Servo Controller API\n";
  response += "endpoints=/press,/status,/fsr\n";
  response += "energy_saving_mode=" + String(ENERGY_SAVING_MODE ? "yes" : "no") + "\n";
  response += "wifi_mode=station\n";
  response += "wifi_ssid=" + String(WIFI_SSID) + "\n";
  response += "ip=" + WiFi.localIP().toString() + "\n";
  response += "wifi_connected=" + String(WiFi.status() == WL_CONNECTED ? "yes" : "no") + "\n";
  response += "wifi_rssi_dbm=" + String(WiFi.RSSI()) + "\n";
  response += "fsr_threshold=" + String(FSR_THRESHOLD) + "\n";
  response += "current_angle=" + String(currentAngle) + "\n";

  server.send(200, "text/plain", response);
}

void handlePress() {
  if (isPressing) {
    server.send(409, "text/plain", "Already pressing. Please wait.");
    return;
  }

  isPressing = true;

  Serial.println("# WiFi command received: /press");

  bool reached = pressButtonUntilThreshold();

  Serial.println("# Holding button");
  delay(HOLD_TIME_MS);

  Serial.println("# Returning to start position");
  moveServoSmooth(currentAngle, START_ANGLE);

  isPressing = false;

  // Measure RSSI after the press operation, immediately before response construction
  int wifiRssiDbm = WiFi.RSSI();

  String response = "";
  response += "Press command finished\n";
  response += "energy_saving_mode=" + String(ENERGY_SAVING_MODE ? "yes" : "no") + "\n";
  response += "threshold=" + String(FSR_THRESHOLD) + "\n";
  response += "threshold_reached=" + String(reached ? "yes" : "no") + "\n";
  response += "trigger_angle=" + String(lastTriggerAngle) + "\n";
  response += "fsr_raw=" + String(lastFsrRaw) + "\n";
  response += "fsr_voltage=" + String(lastFsrVoltage, 4) + "\n";
  response += "wifi_rssi_dbm=" + String(wifiRssiDbm) + "\n";

  server.send(200, "text/plain", response);
}

void handleStatus() {
  // Read FSR only when /status is requested
  int fsrRaw = analogRead(FSR_PIN);
  float fsrVoltage = rawToVoltage(fsrRaw);

  lastFsrRaw = fsrRaw;
  lastFsrVoltage = fsrVoltage;

  String response = "";

  response += "status=" + String(isPressing ? "pressing" : "idle") + "\n";
  response += "energy_saving_mode=" + String(ENERGY_SAVING_MODE ? "yes" : "no") + "\n";
  response += "wifi_mode=station\n";
  response += "wifi_ssid=" + String(WIFI_SSID) + "\n";
  response += "ip=" + WiFi.localIP().toString() + "\n";
  response += "wifi_connected=" + String(WiFi.status() == WL_CONNECTED ? "yes" : "no") + "\n";
  response += "wifi_rssi_dbm=" + String(WiFi.RSSI()) + "\n";
  response += "current_angle=" + String(currentAngle) + "\n";
  response += "fsr_threshold=" + String(FSR_THRESHOLD) + "\n";
  response += "last_fsr_raw=" + String(lastFsrRaw) + "\n";
  response += "last_fsr_voltage=" + String(lastFsrVoltage, 4) + "\n";
  response += "last_trigger_angle=" + String(lastTriggerAngle) + "\n";
  response += "last_press_reached_threshold=" + String(lastPressReachedThreshold ? "yes" : "no") + "\n";

  server.send(200, "text/plain", response);
}

void handleFsr() {
  // Read FSR only when /fsr is requested
  int fsrRaw = analogRead(FSR_PIN);
  float fsrVoltage = rawToVoltage(fsrRaw);

  lastFsrRaw = fsrRaw;
  lastFsrVoltage = fsrVoltage;

  String response = "";
  response += "fsr_raw=" + String(fsrRaw) + "\n";
  response += "fsr_voltage=" + String(fsrVoltage, 4) + "\n";
  response += "wifi_rssi_dbm=" + String(WiFi.RSSI()) + "\n";

  server.send(200, "text/plain", response);
}

// =========================
// BUTTON PRESS LOGIC
// =========================

bool pressButtonUntilThreshold() {
  lastPressReachedThreshold = false;
  lastTriggerAngle = END_ANGLE;

  Serial.println("# Starting button press");

  for (int angle = START_ANGLE; angle >= END_ANGLE; angle -= STEP_SIZE) {
    myServo.write(angle);
    currentAngle = angle;

    delay(STEP_DELAY);

    int fsrRaw = analogRead(FSR_PIN);
    float fsrVoltage = rawToVoltage(fsrRaw);

    lastFsrRaw = fsrRaw;
    lastFsrVoltage = fsrVoltage;

    Serial.print("angle=");
    Serial.print(angle);
    Serial.print(", fsr_raw=");
    Serial.print(fsrRaw);
    Serial.print(", fsr_voltage=");
    Serial.println(fsrVoltage, 4);

    if (fsrRaw >= FSR_THRESHOLD) {
      lastTriggerAngle = angle;
      lastPressReachedThreshold = true;

      Serial.print("# Threshold reached at angle ");
      Serial.println(angle);

      return true;
    }
  }

  Serial.println("# Reached 0 degrees without reaching threshold");
  return false;
}

void moveServoSmooth(int fromAngle, int toAngle) {
  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; angle += STEP_SIZE) {
      myServo.write(angle);
      currentAngle = angle;
      delay(STEP_DELAY);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle -= STEP_SIZE) {
      myServo.write(angle);
      currentAngle = angle;
      delay(STEP_DELAY);
    }
  }
}

float rawToVoltage(int rawValue) {
  return (rawValue * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
}