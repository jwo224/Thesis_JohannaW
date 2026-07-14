#include <ESP32Servo.h>

Servo myServo;

// =========================
// PIN SETTINGS
// =========================

const int SERVO_PIN = 3;
const int FSR_PIN = 4;

// =========================
// SERVO SETTINGS
// =========================

const int START_ANGLE = 180;
const int END_ANGLE = 0;

const int STEP_SIZE = 1;
const int STEP_DELAY = 20;   // ms, increase for slower movement

// =========================
// FSR SETTINGS
// =========================

const int INITIAL_THRESHOLD = 20;
const float THRESHOLD_FACTOR = 1.25;  // relative increase: +25%
const int MAX_THRESHOLD = 4095;

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

// =========================
// STATE VARIABLES
// =========================

int currentAngle = START_ANGLE;
int currentThreshold = INITIAL_THRESHOLD;
int trialNumber = 0;

bool testFinished = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  myServo.attach(SERVO_PIN);
  myServo.write(START_ANGLE);
  currentAngle = START_ANGLE;

  delay(1000);

  Serial.println("# Adaptive button press test ready");
  Serial.println("# Send any character to start");
  Serial.println("# After each press, answer with y or n");
  Serial.println("# CSV columns:");
  Serial.println("trial,time_ms,phase,threshold,angle,fsr_raw,fsr_voltage");
}

void loop() {
  if (testFinished) {
    return;
  }

  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      Serial.read();
    }

    runAdaptiveTest();
  }
}

void runAdaptiveTest() {
  bool success = false;

  while (!success && currentThreshold <= MAX_THRESHOLD) {
    trialNumber++;

    Serial.println("# ----------------------------------------");
    Serial.print("# Trial ");
    Serial.print(trialNumber);
    Serial.print(" | Threshold: ");
    Serial.println(currentThreshold);

    moveServoSmooth(currentAngle, START_ANGLE, "return_to_start");
    delay(1000);

    bool thresholdReached = pressUntilThreshold(currentThreshold);

    if (!thresholdReached) {
      Serial.println("# Reached 0 degrees without reaching threshold");

      Serial.println("# Moving back to 180 degrees");
      moveServoSmooth(currentAngle, START_ANGLE, "return");

      Serial.println("# Threshold was NOT reached. Try again with the same threshold? Type y or n");

      char tryAgain = waitForYesNoAnswer();

      if (tryAgain == 'y') {
        Serial.println("# Trying again with the same threshold.");
        delay(1000);
        continue;
      } else {
        Serial.println("# Not trying again. Increasing threshold.");

        currentThreshold = calculateNextThreshold(currentThreshold);

        Serial.print("# New threshold: ");
        Serial.println(currentThreshold);

        delay(1000);
        continue;
      }
    }

    // Only happens if threshold WAS reached
    Serial.println("# Holding position for 2 seconds");
    logHoldData(2000, "hold");

    Serial.println("# Moving back to 180 degrees");
    moveServoSmooth(currentAngle, START_ANGLE, "return");

    Serial.println("# Was the button press successful? Type y or n");

    char answer = waitForYesNoAnswer();

    if (answer == 'y') {
      Serial.println("# Success confirmed. Test finished.");
      success = true;
      testFinished = true;
    } else {
      Serial.println("# Not successful. Increasing threshold.");

      currentThreshold = calculateNextThreshold(currentThreshold);

      Serial.print("# New threshold: ");
      Serial.println(currentThreshold);

      delay(1000);
    }
  }

  if (currentThreshold > MAX_THRESHOLD) {
    Serial.println("# Maximum threshold reached. Test stopped.");
    testFinished = true;
  }
}

bool pressUntilThreshold(int threshold) {
  Serial.println("# Pressing button");

  unsigned long startTime = millis();

  for (int angle = START_ANGLE; angle >= END_ANGLE; angle -= STEP_SIZE) {
    myServo.write(angle);
    currentAngle = angle;

    delay(STEP_DELAY);

    int fsrRaw = analogRead(FSR_PIN);
    float fsrVoltage = rawToVoltage(fsrRaw);
    unsigned long elapsedTime = millis() - startTime;

    printCsvRow(
      trialNumber,
      elapsedTime,
      "press",
      threshold,
      angle,
      fsrRaw,
      fsrVoltage
    );

    if (fsrRaw >= threshold) {
      Serial.print("# Threshold reached at angle ");
      Serial.print(angle);
      Serial.print(" with FSR raw ");
      Serial.println(fsrRaw);

      return true;
    }
  }

  return false;
}

void logHoldData(unsigned long holdTimeMs, const char* phase) {
  unsigned long startTime = millis();

  while (millis() - startTime < holdTimeMs) {
    int fsrRaw = analogRead(FSR_PIN);
    float fsrVoltage = rawToVoltage(fsrRaw);
    unsigned long elapsedTime = millis() - startTime;

    printCsvRow(
      trialNumber,
      elapsedTime,
      phase,
      currentThreshold,
      currentAngle,
      fsrRaw,
      fsrVoltage
    );

    delay(50);
  }
}

void moveServoSmooth(int fromAngle, int toAngle, const char* phase) {
  unsigned long startTime = millis();

  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; angle += STEP_SIZE) {
      myServo.write(angle);
      currentAngle = angle;

      delay(STEP_DELAY);

      int fsrRaw = analogRead(FSR_PIN);
      float fsrVoltage = rawToVoltage(fsrRaw);
      unsigned long elapsedTime = millis() - startTime;

      printCsvRow(
        trialNumber,
        elapsedTime,
        phase,
        currentThreshold,
        angle,
        fsrRaw,
        fsrVoltage
      );
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle -= STEP_SIZE) {
      myServo.write(angle);
      currentAngle = angle;

      delay(STEP_DELAY);

      int fsrRaw = analogRead(FSR_PIN);
      float fsrVoltage = rawToVoltage(fsrRaw);
      unsigned long elapsedTime = millis() - startTime;

      printCsvRow(
        trialNumber,
        elapsedTime,
        phase,
        currentThreshold,
        angle,
        fsrRaw,
        fsrVoltage
      );
    }
  }
}

char waitForYesNoAnswer() {
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();

      if (c == 'y' || c == 'Y') {
        clearSerialBuffer();
        return 'y';
      }

      if (c == 'n' || c == 'N') {
        clearSerialBuffer();
        return 'n';
      }
    }
  }
}

void clearSerialBuffer() {
  delay(50);
  while (Serial.available() > 0) {
    Serial.read();
  }
}

int calculateNextThreshold(int oldThreshold) {
  int newThreshold = round(oldThreshold * THRESHOLD_FACTOR);

  // Make sure it always increases at least by 1
  if (newThreshold <= oldThreshold) {
    newThreshold = oldThreshold + 1;
  }

  if (newThreshold > MAX_THRESHOLD) {
    newThreshold = MAX_THRESHOLD + 1;
  }

  return newThreshold;
}

float rawToVoltage(int rawValue) {
  return (rawValue * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
}

void printCsvRow(
  int trial,
  unsigned long timeMs,
  const char* phase,
  int threshold,
  int angle,
  int fsrRaw,
  float fsrVoltage
) {
  Serial.print(trial);
  Serial.print(",");
  Serial.print(timeMs);
  Serial.print(",");
  Serial.print(phase);
  Serial.print(",");
  Serial.print(threshold);
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.print(fsrRaw);
  Serial.print(",");
  Serial.println(fsrVoltage, 4);
}