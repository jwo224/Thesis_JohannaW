#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Servo.h>

// =====================================================
// PIN SETTINGS
// =====================================================

const int FSR_PIN = A3;
const int SERVO_PIN = D10;

// =====================================================
// SERVO SETTINGS
// =====================================================

const int START_ANGLE = 180;
const int END_ANGLE = 0;

const int STEP_SIZE = 1;
const int STEP_DELAY_MS = 20;

// Detach servo when idle to reduce jitter/signal activity.
// Important: if the mechanism needs active holding torque, set this to false.
const bool DETACH_SERVO_WHEN_IDLE = true;

// =====================================================
// FSR SETTINGS
// =====================================================

int fsrThreshold = 50;

const float ADC_REF_VOLTAGE = 3.3;
const int ADC_MAX_VALUE = 4095;

// Number of analog samples when FSR is read.
// Higher = smoother but slightly slower.
const int FSR_SAMPLES = 4;

// =====================================================
// HOLD SETTINGS
// =====================================================

const int HOLD_TIME_MS = 1000;

// =====================================================
// BLE SETTINGS
// =====================================================

const char BLE_NAME[] = "XIAO-FSR-SERVO";

// Lower TX power saves energy but reduces range.
// Valid values usually include: -40, -20, -16, -12, -8, -4, 0, 4, 8
const int BLE_TX_POWER = 0;

// Advertising interval uses units of 0.625 ms.
// 800 = 500 ms, 1600 = 1000 ms.
// Slower advertising saves power but connection discovery takes longer.
const int BLE_ADV_INTERVAL_MIN = 800;
const int BLE_ADV_INTERVAL_MAX = 1600;

// =====================================================
// IDLE POWER SETTINGS
// =====================================================

// Idle delay keeps the loop from constantly spinning.
// BLE is still active and reachable.
const int IDLE_LOOP_DELAY_MS = 50;

// =====================================================
// OBJECTS
// =====================================================

Servo myServo;
BLEUart bleuart;

// =====================================================
// STATE VARIABLES
// =====================================================

int currentAngle = START_ANGLE;
bool isPressing = false;
bool stopRequested = false;
bool servoAttached = false;

int lastFsrRaw = 0;
float lastFsrVoltage = 0.0;
int lastTriggerAngle = START_ANGLE;
bool lastPressReachedThreshold = false;

// =====================================================
// COMMAND BUFFERS
// =====================================================

const int COMMAND_BUFFER_SIZE = 64;

char serialBuffer[COMMAND_BUFFER_SIZE];
char bleBuffer[COMMAND_BUFFER_SIZE];

int serialBufferIndex = 0;
int bleBufferIndex = 0;

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  unsigned long startTime = millis();
  while (!Serial && millis() - startTime < 1500) {
    delay(10);
  }

  analogReadResolution(12);

  attachServoIfNeeded();
  myServo.write(START_ANGLE);
  currentAngle = START_ANGLE;
  delay(300);
  detachServoIfAllowed();

  setupBLE();

  sendSerial("XIAO FSR + Servo BLE low-power controller ready");
  sendSerial("FSR is only read on PRESS, FSR, or STATUS.");
  sendSerial("Commands: OPEN, CLOSE, STOP, GOTO 90, PRESS, STATUS, FSR, THRESH 100");

  sendBLE("READY");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  handleSerialInput();
  handleBLEInput();

  // No continuous FSR reading here.
  // This is intentional for battery saving.

  if (!isPressing) {
    delay(IDLE_LOOP_DELAY_MS);
  }
}

// =====================================================
// BLE SETUP
// =====================================================

void setupBLE() {
  Bluefruit.begin();

  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(BLE_NAME);

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);

  // Slower advertising for lower power consumption.
  Bluefruit.Advertising.setInterval(BLE_ADV_INTERVAL_MIN, BLE_ADV_INTERVAL_MAX);

  // 0 = advertise forever, so it stays reachable.
  Bluefruit.Advertising.start(0);
}

// =====================================================
// INPUT HANDLING
// =====================================================

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    handleCommandCharacter(c, serialBuffer, serialBufferIndex);
  }
}

void handleBLEInput() {
  while (bleuart.available()) {
    char c = bleuart.read();
    handleCommandCharacter(c, bleBuffer, bleBufferIndex);
  }
}

void handleCommandCharacter(char c, char* buffer, int& index) {
  if (c == '\n' || c == '\r') {
    if (index > 0) {
      buffer[index] = '\0';
      processCommand(buffer);
      index = 0;
    }
    return;
  }

  if (index < COMMAND_BUFFER_SIZE - 1) {
    buffer[index++] = c;
  }
}

void uppercaseCommand(char* command) {
  for (int i = 0; command[i] != '\0'; i++) {
    if (command[i] >= 'a' && command[i] <= 'z') {
      command[i] = command[i] - 32;
    }
  }
}

void trimCommand(char* command) {
  int start = 0;
  while (command[start] == ' ' || command[start] == '\t') {
    start++;
  }

  int end = strlen(command) - 1;
  while (end >= start && (command[end] == ' ' || command[end] == '\t')) {
    command[end] = '\0';
    end--;
  }

  if (start > 0) {
    int i = 0;
    while (command[start] != '\0') {
      command[i++] = command[start++];
    }
    command[i] = '\0';
  }
}

// =====================================================
// COMMAND PROCESSING
// =====================================================

void processCommand(char* command) {
  trimCommand(command);
  uppercaseCommand(command);

  if (strlen(command) == 0) {
    return;
  }

  if (strcmp(command, "OPEN") == 0) {
    if (isPressing) {
      sendBLE("BUSY");
      return;
    }

    moveServoSmooth(currentAngle, START_ANGLE);
    detachServoIfAllowed();

    sendSerial("Servo moved to OPEN position.");
    sendBLE("OPENED");
  }

  else if (strcmp(command, "CLOSE") == 0) {
    if (isPressing) {
      sendBLE("BUSY");
      return;
    }

    moveServoSmooth(currentAngle, END_ANGLE);
    detachServoIfAllowed();

    sendSerial("Servo moved to CLOSE position.");
    sendBLE("CLOSED");
  }

  else if (strcmp(command, "STOP") == 0) {
    stopRequested = true;
    isPressing = false;

    detachServoIfAllowed();

    sendSerial("Stop requested.");
    sendBLE("STOP");
  }

  else if (strncmp(command, "GOTO ", 5) == 0) {
    if (isPressing) {
      sendBLE("BUSY");
      return;
    }

    int targetAngle = atoi(command + 5);
    targetAngle = constrain(targetAngle, 0, 180);

    moveServoSmooth(currentAngle, targetAngle);
    detachServoIfAllowed();

    sendSerialStatusLine("GOTO_DONE");
    sendBLE("GOTO_DONE");
    sendBLEAngle(currentAngle);
  }

  else if (strncmp(command, "THRESH ", 7) == 0) {
    if (isPressing) {
      sendBLE("BUSY");
      return;
    }

    int newThreshold = atoi(command + 7);
    newThreshold = constrain(newThreshold, 0, 4095);

    fsrThreshold = newThreshold;

    char msg[32];
    snprintf(msg, sizeof(msg), "THRESH %d", fsrThreshold);

    sendSerial(msg);
    sendBLE(msg);
  }

  else if (strcmp(command, "PRESS") == 0) {
    if (isPressing) {
      sendBLE("BUSY");
      return;
    }

    runPressSequence();
  }

  else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  }

  else if (strcmp(command, "FSR") == 0) {
    printFsr();
  }

  else {
    sendSerial("Unknown command.");
    sendBLE("UNKNOWN");
  }
}

// =====================================================
// PRESS SEQUENCE
// =====================================================

void runPressSequence() {
  isPressing = true;
  stopRequested = false;

  attachServoIfNeeded();

  sendSerial("PRESS_START");
  sendBLE("PRESS");

  bool reached = pressButtonUntilThreshold();

  if (!stopRequested) {
    sendSerial("HOLD_START");
    sendBLE("HOLD");

    unsigned long holdStart = millis();

    while (millis() - holdStart < HOLD_TIME_MS) {
      handleStopOnly();

      if (stopRequested) {
        sendSerial("HOLD_STOPPED");
        sendBLE("STOPPED");
        break;
      }

      delay(10);
    }
  }

  sendSerial("RETURN_START");
  sendBLE("RETURN");

  moveServoSmooth(currentAngle, START_ANGLE);

  detachServoIfAllowed();

  isPressing = false;

  sendSerialPressResult(reached);
  sendBLEPressResult(reached);
}

bool pressButtonUntilThreshold() {
  lastPressReachedThreshold = false;
  lastTriggerAngle = END_ANGLE;

  for (int angle = START_ANGLE; angle >= END_ANGLE; angle -= STEP_SIZE) {
    handleStopOnly();

    if (stopRequested) {
      lastTriggerAngle = angle;
      lastPressReachedThreshold = false;
      sendSerial("PRESS_STOPPED");
      sendBLE("STOPPED");
      return false;
    }

    attachServoIfNeeded();

    myServo.write(angle);
    currentAngle = angle;

    delay(STEP_DELAY_MS);

    readFsr();

    logStepToSerial(angle, lastFsrRaw, lastFsrVoltage);

    if (lastFsrRaw >= fsrThreshold) {
      lastTriggerAngle = angle;
      lastPressReachedThreshold = true;

      sendSerialThresholdReached(angle, lastFsrRaw, lastFsrVoltage);
      sendBLEThresholdReached(angle, lastFsrRaw);

      return true;
    }
  }

  readFsr();

  lastTriggerAngle = END_ANGLE;
  lastPressReachedThreshold = false;

  sendSerialEndReached(lastFsrRaw, lastFsrVoltage);
  sendBLEEndReached(lastFsrRaw);

  return false;
}

// =====================================================
// SERVO MOVEMENT
// =====================================================

void attachServoIfNeeded() {
  if (!servoAttached) {
    myServo.attach(SERVO_PIN);
    servoAttached = true;
    delay(20);
  }
}

void detachServoIfAllowed() {
  if (DETACH_SERVO_WHEN_IDLE && servoAttached && !isPressing) {
    myServo.detach();
    servoAttached = false;
  }
}

void moveServoSmooth(int fromAngle, int toAngle) {
  fromAngle = constrain(fromAngle, 0, 180);
  toAngle = constrain(toAngle, 0, 180);

  attachServoIfNeeded();

  if (fromAngle == toAngle) {
    myServo.write(toAngle);
    currentAngle = toAngle;
    return;
  }

  if (fromAngle < toAngle) {
    for (int angle = fromAngle; angle <= toAngle; angle += STEP_SIZE) {
      handleStopOnly();

      if (stopRequested) {
        break;
      }

      myServo.write(angle);
      currentAngle = angle;

      delay(STEP_DELAY_MS);
    }
  } else {
    for (int angle = fromAngle; angle >= toAngle; angle -= STEP_SIZE) {
      handleStopOnly();

      if (stopRequested) {
        break;
      }

      myServo.write(angle);
      currentAngle = angle;

      delay(STEP_DELAY_MS);
    }
  }

  if (!stopRequested) {
    myServo.write(toAngle);
    currentAngle = toAngle;
  }
}

// =====================================================
// STOP HANDLING DURING MOVEMENT
// =====================================================

void handleStopOnly() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      serialBuffer[serialBufferIndex] = '\0';
      trimCommand(serialBuffer);
      uppercaseCommand(serialBuffer);

      if (strcmp(serialBuffer, "STOP") == 0) {
        stopRequested = true;
      }

      serialBufferIndex = 0;
    } else if (serialBufferIndex < COMMAND_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = c;
    }
  }

  while (bleuart.available()) {
    char c = bleuart.read();

    if (c == '\n' || c == '\r') {
      bleBuffer[bleBufferIndex] = '\0';
      trimCommand(bleBuffer);
      uppercaseCommand(bleBuffer);

      if (strcmp(bleBuffer, "STOP") == 0) {
        stopRequested = true;
      }

      bleBufferIndex = 0;
    } else if (bleBufferIndex < COMMAND_BUFFER_SIZE - 1) {
      bleBuffer[bleBufferIndex++] = c;
    }
  }
}

// =====================================================
// FSR
// =====================================================

void readFsr() {
  long sum = 0;

  for (int i = 0; i < FSR_SAMPLES; i++) {
    sum += analogRead(FSR_PIN);
    delay(2);
  }

  lastFsrRaw = sum / FSR_SAMPLES;
  lastFsrVoltage = rawToVoltage(lastFsrRaw);
}

float rawToVoltage(int rawValue) {
  return (rawValue * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
}

// =====================================================
// STATUS / FSR OUTPUT
// =====================================================

void printStatus() {
  readFsr();

  char msg[160];

  snprintf(
    msg,
    sizeof(msg),
    "STATUS state=%s angle=%d threshold=%d fsr_raw=%d trigger_angle=%d reached=%s servo=%s",
    isPressing ? "pressing" : "idle",
    currentAngle,
    fsrThreshold,
    lastFsrRaw,
    lastTriggerAngle,
    lastPressReachedThreshold ? "yes" : "no",
    servoAttached ? "attached" : "detached"
  );

  sendSerial(msg);

  sendBLE("STATUS");
  sendBLEAngle(currentAngle);
  sendBLEFsr(lastFsrRaw);
  sendBLEThreshold();
  sendBLE(servoAttached ? "S ATTACHED" : "S DETACHED");
}

void printFsr() {
  readFsr();

  char msg[80];

  snprintf(
    msg,
    sizeof(msg),
    "FSR fsr_raw=%d fsr_voltage=%.4f",
    lastFsrRaw,
    lastFsrVoltage
  );

  sendSerial(msg);
  sendBLEFsr(lastFsrRaw);
}

// =====================================================
// SERIAL LOGGING
// =====================================================

void logStepToSerial(int angle, int fsrRaw, float fsrVoltage) {
  Serial.print("angle=");
  Serial.print(angle);
  Serial.print(", fsr_raw=");
  Serial.print(fsrRaw);
  Serial.print(", fsr_voltage=");
  Serial.println(fsrVoltage, 4);
}

void sendSerialThresholdReached(int angle, int fsrRaw, float fsrVoltage) {
  Serial.print("THRESHOLD_REACHED angle=");
  Serial.print(angle);
  Serial.print(", fsr_raw=");
  Serial.print(fsrRaw);
  Serial.print(", fsr_voltage=");
  Serial.print(fsrVoltage, 4);
  Serial.print(", threshold=");
  Serial.println(fsrThreshold);
}

void sendSerialEndReached(int fsrRaw, float fsrVoltage) {
  Serial.print("END_REACHED_WITHOUT_THRESHOLD angle=");
  Serial.print(END_ANGLE);
  Serial.print(", fsr_raw=");
  Serial.print(fsrRaw);
  Serial.print(", fsr_voltage=");
  Serial.print(fsrVoltage, 4);
  Serial.print(", threshold=");
  Serial.println(fsrThreshold);
}

void sendSerialPressResult(bool reached) {
  Serial.print("PRESS_FINISHED threshold=");
  Serial.print(fsrThreshold);
  Serial.print(", threshold_reached=");
  Serial.print(reached ? "yes" : "no");
  Serial.print(", trigger_angle=");
  Serial.print(lastTriggerAngle);
  Serial.print(", fsr_raw=");
  Serial.print(lastFsrRaw);
  Serial.print(", fsr_voltage=");
  Serial.println(lastFsrVoltage, 4);
}

void sendSerialStatusLine(const char* label) {
  Serial.print(label);
  Serial.print(" angle=");
  Serial.print(currentAngle);
  Serial.print(", fsr_raw=");
  Serial.print(lastFsrRaw);
  Serial.print(", fsr_voltage=");
  Serial.println(lastFsrVoltage, 4);
}

// =====================================================
// BLE SHORT MESSAGES
// Keep lines short to avoid BLE UART chunking.
// =====================================================

void sendBLEThresholdReached(int angle, int fsrRaw) {
  sendBLE("HIT");
  sendBLEAngle(angle);
  sendBLEFsr(fsrRaw);
  sendBLEThreshold();
}

void sendBLEEndReached(int fsrRaw) {
  sendBLE("END_NO_HIT");
  sendBLEAngle(END_ANGLE);
  sendBLEFsr(fsrRaw);
  sendBLEThreshold();
}

void sendBLEPressResult(bool reached) {
  sendBLE("DONE");
  sendBLE(reached ? "R 1" : "R 0");
  sendBLEAngle(lastTriggerAngle);
  sendBLEFsr(lastFsrRaw);
  sendBLEThreshold();
}

void sendBLEAngle(int angle) {
  char msg[20];
  snprintf(msg, sizeof(msg), "A %d", angle);
  sendBLE(msg);
}

void sendBLEFsr(int fsrRaw) {
  char msg[20];
  snprintf(msg, sizeof(msg), "F %d", fsrRaw);
  sendBLE(msg);
}

void sendBLEThreshold() {
  char msg[20];
  snprintf(msg, sizeof(msg), "T %d", fsrThreshold);
  sendBLE(msg);
}

// =====================================================
// OUTPUT
// =====================================================

void sendSerial(const char* msg) {
  Serial.println(msg);
}

void sendBLE(const char* msg) {
  if (Bluefruit.connected()) {
    bleuart.println(msg);
  }
}