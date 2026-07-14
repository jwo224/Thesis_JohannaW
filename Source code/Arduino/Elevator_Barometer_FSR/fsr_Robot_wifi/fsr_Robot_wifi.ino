#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// =========================
// ROBOT WIFI SETTINGS
// =========================

const char* WIFI_SSID = "GL-AR300M-d28";
const char* WIFI_PASSWORD = "goodlife";

// Fixed Access Point IP:
// http://192.168.4.1

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

const int FSR_THRESHOLD = 25;

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

// =========================
// HOLD SETTINGS
// =========================

const int HOLD_TIME_MS = 1000;

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

// LED blink variables
unsigned long lastLedBlinkTime = 0;
bool ledState = false;
const int LED_BLINK_INTERVAL = 500;

// =========================
// SETUP
// =========================

void setup() {
  delay(1000);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32-C3 FSR Servo Controller - Robot Wi-Fi Mode");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(12);

  myServo.attach(SERVO_PIN);
  myServo.write(START_ANGLE);
  currentAngle = START_ANGLE;

  connectToRobotWifi();

  server.on("/", handleRoot);
  server.on("/press", handlePress);
  server.on("/status", handleStatus);
  server.on("/fsr", handleFsr);

  server.begin();

  Serial.println();
  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());
}

// =========================
// MAIN LOOP
// =========================

void loop() {
  server.handleClient();

  updateWifiLed();

  lastFsrRaw = analogRead(FSR_PIN);
  lastFsrVoltage = rawToVoltage(lastFsrRaw);
}

// =========================
// LED STATUS
// =========================

void updateWifiLed() {
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (millis() - lastLedBlinkTime >= LED_BLINK_INTERVAL) {
      lastLedBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
}

// =========================
// CONNECT TO ROBOTs WIFI
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
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to robot Wi-Fi.");
    Serial.print("Wi-Fi name: ");
    Serial.println(WIFI_SSID);
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_PIN, HIGH);
  } else {
    Serial.println("Failed to connect to robot Wi-Fi.");
    digitalWrite(LED_PIN, LOW);
  }
}

// =========================
// WEB HANDLERS
// =========================

void handleRoot() {
  String html = "";

  html += "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32-C3 Button Press Controller</title>";
  html += "<style>";
  html += "body{font-family:Arial;text-align:center;margin:30px;background:#f4f4f4;}";
  html += ".card{background:white;padding:25px;border-radius:14px;max-width:500px;margin:auto;box-shadow:0 2px 10px rgba(0,0,0,0.2);}";
  html += "button{font-size:26px;padding:18px 35px;margin:15px;border:none;border-radius:10px;background:#4CAF50;color:white;}";
  html += "a{font-size:20px;display:block;margin:12px;}";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class='card'>";
  html += "<h1>ESP32-C3 Button Press Controller</h1>";
  html += "<p><a href='/press'><button>Press Button</button></a></p>";
  html += "<a href='/status'>Status</a>";
  html += "<a href='/fsr'>FSR value</a>";
  html += "<p>Access Point mode active</p>";
  html += "<p>Address: <b>192.168.4.1</b></p>";
  html += "</div>";
  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
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

  String response = "";
  response += "Press command finished\n";
  response += "threshold=" + String(FSR_THRESHOLD) + "\n";
  response += "threshold_reached=" + String(reached ? "yes" : "no") + "\n";
  response += "trigger_angle=" + String(lastTriggerAngle) + "\n";
  response += "fsr_raw=" + String(lastFsrRaw) + "\n";
  response += "fsr_voltage=" + String(lastFsrVoltage, 4) + "\n";

  server.send(200, "text/plain", response);
}

void handleStatus() {
  String response = "";

  response += "status=" + String(isPressing ? "pressing" : "idle") + "\n";
  response += "wifi_mode=station\n";
  response += "wifi_ssid=" + String(WIFI_SSID) + "\n";
  response += "ip=" + WiFi.localIP().toString() + "\n";
  response += "wifi_connected=" + String(WiFi.status() == WL_CONNECTED ? "yes" : "no") + "\n";
  response += "current_angle=" + String(currentAngle) + "\n";
  response += "fsr_threshold=" + String(FSR_THRESHOLD) + "\n";
  response += "last_fsr_raw=" + String(lastFsrRaw) + "\n";
  response += "last_fsr_voltage=" + String(lastFsrVoltage, 4) + "\n";
  response += "last_trigger_angle=" + String(lastTriggerAngle) + "\n";
  response += "last_press_reached_threshold=" + String(lastPressReachedThreshold ? "yes" : "no") + "\n";

  server.send(200, "text/plain", response);
}

void handleFsr() {
  int fsrRaw = analogRead(FSR_PIN);
  float fsrVoltage = rawToVoltage(fsrRaw);

  String response = "";
  response += "fsr_raw=" + String(fsrRaw) + "\n";
  response += "fsr_voltage=" + String(fsrVoltage, 4) + "\n";

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
    updateWifiLed();

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
      updateWifiLed();

      myServo.write(angle);
      currentAngle = angle;
      delay(STEP_DELAY);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle -= STEP_SIZE) {
      updateWifiLed();

      myServo.write(angle);
      currentAngle = angle;
      delay(STEP_DELAY);
    }
  }
}

float rawToVoltage(int rawValue) {
  return (rawValue * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
}