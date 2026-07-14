#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
// Commands: http://10.0.0.164/command?cmd=press floor 1 or http://10.0.0.164/command?cmd=press floor 2

//  WiFi credentials
const char* ssid = "student-FoU";
const char* password = "stud2018";

WebServer server(80);

// Servo setup
static const int servoPin = 13;
Servo servo1;

// Positions for floors
int floor1Pos = 0;
int floor2Pos = 90;

void moveServoSmooth(int target) {
  int current = servo1.read();

  if (current < target) {
    for (int pos = current; pos <= target; pos++) {
      servo1.write(pos);
      delay(15);
    }
  } else {
    for (int pos = current; pos >= target; pos--) {
      servo1.write(pos);
      delay(15);
    }
  }
}

void handleCommand() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    Serial.println("Command: " + cmd);

    if (cmd == "press floor 1") {
      moveServoSmooth(floor1Pos);
      server.send(200, "text/plain", "Moved to Floor 1");
    }
    else if (cmd == "press floor 2") {
      moveServoSmooth(floor2Pos);
      server.send(200, "text/plain", "Moved to Floor 2");
    }
    else {
      server.send(400, "text/plain", "Unknown command");
    }
  } else {
    server.send(400, "text/plain", "No command");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 started");

  servo1.attach(servoPin);

  WiFi.begin(ssid, password);
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/command", handleCommand);
  server.begin();
}

void loop() {
  server.handleClient();
}