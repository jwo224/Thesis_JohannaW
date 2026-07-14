#include <ESP32Servo.h>

const int fsrPin = 34;
const int servoPin = 18;

const int threshold = 600;
const int maxAngle = 180;
const int minAngle = 0;

// Bigger = slower movement
const int stepDelay = 200;

// Choose your start direction
const int startAngle = 180;  // all the way back
const int stopAngle = 0;     // moves toward this angle

Servo myServo;

int angle = startAngle;
bool running = false;
bool stopped = false;

int readFSRAverage() {
  long sum = 0;

  for (int i = 0; i < 20; i++) {
    sum += analogRead(fsrPin);
    delay(2);
  }

  return sum / 20;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(fsrPin, ADC_11db);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);

  angle = startAngle;
  myServo.write(angle);
  delay(1500);

  Serial.println("Ready. Type 'start' to begin.");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "start") {
      angle = startAngle;
      myServo.write(angle);
      delay(1500);

      running = true;
      stopped = false;

      Serial.println("START");
    }

    if (command == "reset") {
      angle = startAngle;
      myServo.write(angle);
      delay(1500);

      running = false;
      stopped = false;

      Serial.println("RESET");
      Serial.println("Ready. Type 'start' to begin.");
    }
  }

  int fsrRaw = readFSRAverage();

  Serial.print("FSR:");
  Serial.print(fsrRaw);
  Serial.print("\tAngle:");
  Serial.print(angle);

  if (!running) {
    Serial.println("\tWAIT");
    delay(200);
    return;
  }

  if (stopped) {
    Serial.println("\tSTOP");
    delay(200);
    return;
  }

  if (fsrRaw >= threshold) {
    stopped = true;
    running = false;
    Serial.println("\tSTOP");
    Serial.println("Threshold reached. Type 'reset' or 'start' to run again.");
    return;
  }

  Serial.println("\tRUN");

  if (angle > stopAngle) {
    angle--;
    myServo.write(angle);
    delay(stepDelay);
  } else {
    stopped = true;
    running = false;
    Serial.println("STOP: end angle reached");
    Serial.println("Type 'reset' or 'start' to run again.");
  }
}