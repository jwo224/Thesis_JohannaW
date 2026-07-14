#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 3;
const int FSR_PIN = 4;

const int START_ANGLE = 160;
const int END_ANGLE = 0;

const int STEP_SIZE = 1;
const int STEP_DELAY = 20;  // ms between angle steps, increase for slower movement

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

int currentAngle = START_ANGLE;

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12); // ESP32 ADC: 0 to 4095

  myServo.attach(SERVO_PIN);
  myServo.write(START_ANGLE);
  currentAngle = START_ANGLE;

  delay(1000);

  Serial.println("# Button force measurement ready");
  Serial.println("# Send any character to start a measurement");
  Serial.println("# CSV format:");
  Serial.println("time_ms,angle,fsr_raw,fsr_voltage");
}

void loop() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      Serial.read();
    }

    runMeasurement();
  }
}

void runMeasurement() {
  Serial.println("# Starting measurement");

  // Move to start position first
  moveServoSmooth(currentAngle, START_ANGLE);
  delay(1000);

  unsigned long startTime = millis();

  // CSV header for each run
  Serial.println("time_ms,angle,fsr_raw,fsr_voltage");

  for (int angle = START_ANGLE; angle >= END_ANGLE; angle -= STEP_SIZE) {
    myServo.write(angle);
    currentAngle = angle;

    delay(STEP_DELAY);

    int fsrRaw = analogRead(FSR_PIN);
    float fsrVoltage = (fsrRaw * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
    unsigned long elapsedTime = millis() - startTime;

    Serial.print(elapsedTime);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(fsrRaw);
    Serial.print(",");
    Serial.println(fsrVoltage, 4);
  }

  Serial.println("# Measurement finished");
  Serial.println("# Returning to 180 degrees");

  delay(1000);
  moveServoSmooth(currentAngle, START_ANGLE);

  Serial.println("# Done");
  Serial.println("# Send any character to start another measurement");
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