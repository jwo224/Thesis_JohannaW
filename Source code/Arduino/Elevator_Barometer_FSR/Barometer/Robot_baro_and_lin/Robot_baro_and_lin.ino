#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <LittleFS.h>
#include <math.h>

// =======================================================
// BAROMETER I2C PINS FOR ESP-WROOM-32-U
// =======================================================
#define SDA_PIN 32
#define SCL_PIN 33

// =======================================================
// Motor A
// =======================================================
const int IN1 = 18;
const int IN2 = 21;
const int ENA = 19;  // controls BOTH motors

// =======================================================
// Motor B
// =======================================================
const int IN3 = 18;  // same as IN1, shared pin
const int IN4 = 21;  // same as IN2, shared pin

// PWM for ESP32 Arduino Core 3.x
const int pwmFreq = 2000;
const int pwmResolution = 8;

// Buttons
const int BTN_FWD = 16;
const int BTN_REV = 17;

// Motor control params
const int extendPWM = 210;
const int rampStep = 2;
const int rampInterval = 10;

const unsigned long extendTime = 7000;
const unsigned long retractTime = 8000;

const unsigned long ROBOT_TELEMETRY_INTERVAL_MS = 100;

// =======================================================
// Motor State Machine
// =======================================================
enum MotorState {
  IDLE,
  EXTENDING,
  HOLD,
  RETRACTING
};

MotorState motorState = IDLE;
unsigned long motorStateStartTime = 0;

int currentPWM = 0;
unsigned long lastRampTime = 0;
unsigned long lastRobotTelemetryTime = 0;

// =======================================================
// System Mode
// =======================================================
enum SystemMode {
  MODE_ROBOT,
  MODE_BAROMETER
};

SystemMode systemMode = MODE_ROBOT;

// =======================================================
// Barometer / Floor Detection
// =======================================================
#define MAX_FLOORS 20
#define BARO_PRINT_INTERVAL_MS 1000

#define CALIBRATION_SAMPLES 30
#define LIVE_SAMPLES 10
#define SAMPLE_DELAY_MS 40

#define CSV_FILE "/floor_log.csv"

Adafruit_BMP3XX bmp;

struct FloorPoint {
  int floorNumber;
  float pressurePa;
  float relativeHeightM;
  bool used;
};

struct Reading {
  float temperatureC;
  float pressurePa;
  float relativeHeightM;
  int estimatedFloor;
  float offsetFromEstimatedFloorM;
  bool valid;
};

FloorPoint floors[MAX_FLOORS];

bool barometerInitialized = false;
bool calibrationFinished = false;
bool floor0Set = false;

float floor0PressurePa = 0.0;

unsigned long lastBaroPrintTime = 0;

// =======================================================
// Serial command buffer
// =======================================================
String inputLine = "";

// =======================================================
// MOTOR CONTROL
// =======================================================

void forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, speed);
}

void reverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENA, speed);
}

void stopMotor() {
  ledcWrite(ENA, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void rampForward(int targetPWM) {
  unsigned long now = millis();

  if (now - lastRampTime >= rampInterval) {
    lastRampTime = now;

    if (currentPWM < targetPWM) {
      currentPWM += rampStep;
      if (currentPWM > targetPWM) {
        currentPWM = targetPWM;
      }
    }
  }

  forward(currentPWM);
}

void startDock() {
  if (systemMode != MODE_ROBOT) {
    Serial.println("Robot command rejected: barometer mode active. Type baro_off first.");
    return;
  }

  motorState = EXTENDING;
  motorStateStartTime = millis();
  currentPWM = 0;

  Serial.println("ACK dock");
}

void startUndock() {
  if (systemMode != MODE_ROBOT) {
    Serial.println("Robot command rejected: barometer mode active. Type baro_off first.");
    return;
  }

  motorState = RETRACTING;
  motorStateStartTime = millis();
  currentPWM = 0;

  Serial.println("ACK undock");
}

void emergencyStop() {
  motorState = IDLE;
  currentPWM = 0;
  stopMotor();

  Serial.println("ACK stop");
}

void updateMotorStateMachine() {
  unsigned long now = millis();

  switch (motorState) {
    case IDLE:
      stopMotor();
      currentPWM = 0;
      break;

    case EXTENDING:
      rampForward(extendPWM);

      if (now - motorStateStartTime >= extendTime) {
        stopMotor();
        motorState = HOLD;
      }
      break;

    case HOLD:
      stopMotor();
      break;

    case RETRACTING:
      reverse(255);
      currentPWM = 0;

      if (now - motorStateStartTime >= retractTime) {
        motorState = IDLE;
      }
      break;
  }
}

void handleButtons() {
  if (systemMode != MODE_ROBOT) {
    return;
  }

  static bool lastFwd = false;
  static bool lastRev = false;

  bool fwdPressed = digitalRead(BTN_FWD) == LOW;
  bool revPressed = digitalRead(BTN_REV) == LOW;

  if (fwdPressed && !lastFwd) {
    startDock();
  }

  if (revPressed && !lastRev) {
    startUndock();
  }

  lastFwd = fwdPressed;
  lastRev = revPressed;
}

void printRobotTelemetry() {
  unsigned long now = millis();

  if (now - lastRobotTelemetryTime < ROBOT_TELEMETRY_INTERVAL_MS) {
    return;
  }

  lastRobotTelemetryTime = now;

  if (systemMode == MODE_ROBOT) {
    Serial.print(">pwm:");
    Serial.print(currentPWM);
    Serial.print("\n>state:");
    Serial.println(motorState);
  }
}

// =======================================================
// BAROMETER SENSOR
// =======================================================

bool initBarometer() {
  if (barometerInitialized) {
    return true;
  }

  Wire.begin(SDA_PIN, SCL_PIN);

  bool found = false;

  if (bmp.begin_I2C(0x77, &Wire)) {
    Serial.println("BMP390L found at I2C address 0x77");
    found = true;
  } else if (bmp.begin_I2C(0x76, &Wire)) {
    Serial.println("BMP390L found at I2C address 0x76");
    found = true;
  }

  if (!found) {
    Serial.println("Could not find BMP390L sensor.");
    Serial.println("Check wiring:");
    Serial.println("SEN0423 VCC -> ESP32 3V3");
    Serial.println("SEN0423 GND -> ESP32 GND");
    Serial.println("SEN0423 SDA -> ESP32 GPIO32");
    Serial.println("SEN0423 SCL -> ESP32 GPIO33");
    return false;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  barometerInitialized = true;

  Serial.println("Barometer initialized.");
  return true;
}

bool readSingleSensor(float &temperatureC, float &pressurePa) {
  if (!barometerInitialized) {
    return false;
  }

  if (!bmp.performReading()) {
    return false;
  }

  temperatureC = bmp.temperature;
  pressurePa = bmp.pressure;
  return true;
}

bool readAveragedSensor(float &temperatureC, float &pressurePa, int samples) {
  float tempSum = 0.0;
  float pressureSum = 0.0;
  int goodSamples = 0;

  for (int i = 0; i < samples; i++) {
    float t;
    float p;

    if (readSingleSensor(t, p)) {
      tempSum += t;
      pressureSum += p;
      goodSamples++;
    }

    delay(SAMPLE_DELAY_MS);
  }

  if (goodSamples == 0) {
    return false;
  }

  temperatureC = tempSum / goodSamples;
  pressurePa = pressureSum / goodSamples;
  return true;
}

float heightFromFloor0Pressure(float currentPressurePa) {
  if (!floor0Set || floor0PressurePa <= 0.0) {
    return 0.0;
  }

  return 44330.0 * (1.0 - pow(currentPressurePa / floor0PressurePa, 1.0 / 5.255));
}

// =======================================================
// FLOOR STORAGE
// =======================================================

int findFloorIndex(int floorNumber) {
  for (int i = 0; i < MAX_FLOORS; i++) {
    if (floors[i].used && floors[i].floorNumber == floorNumber) {
      return i;
    }
  }

  return -1;
}

int findFreeFloorSlot() {
  for (int i = 0; i < MAX_FLOORS; i++) {
    if (!floors[i].used) {
      return i;
    }
  }

  return -1;
}

int floorCount() {
  int count = 0;

  for (int i = 0; i < MAX_FLOORS; i++) {
    if (floors[i].used) {
      count++;
    }
  }

  return count;
}

void sortFloorsByNumber() {
  for (int i = 0; i < MAX_FLOORS - 1; i++) {
    for (int j = i + 1; j < MAX_FLOORS; j++) {
      if (floors[i].used && floors[j].used) {
        if (floors[j].floorNumber < floors[i].floorNumber) {
          FloorPoint temp = floors[i];
          floors[i] = floors[j];
          floors[j] = temp;
        }
      }
    }
  }
}

void recalculateFloorHeights() {
  if (!floor0Set) {
    return;
  }

  for (int i = 0; i < MAX_FLOORS; i++) {
    if (floors[i].used) {
      floors[i].relativeHeightM = heightFromFloor0Pressure(floors[i].pressurePa);
    }
  }
}

int estimateFloor(float currentRelativeHeightM) {
  int bestFloor = 0;
  float smallestDifference = 999999.0;

  for (int i = 0; i < MAX_FLOORS; i++) {
    if (floors[i].used) {
      float difference = fabs(currentRelativeHeightM - floors[i].relativeHeightM);

      if (difference < smallestDifference) {
        smallestDifference = difference;
        bestFloor = floors[i].floorNumber;
      }
    }
  }

  return bestFloor;
}

float distanceToEstimatedFloor(float currentRelativeHeightM, int estimatedFloorNumber) {
  int index = findFloorIndex(estimatedFloorNumber);

  if (index < 0) {
    return 0.0;
  }

  return currentRelativeHeightM - floors[index].relativeHeightM;
}

// =======================================================
// CSV LOGGING
// =======================================================

void createCSVHeaderIfNeeded() {
  if (!LittleFS.exists(CSV_FILE)) {
    File file = LittleFS.open(CSV_FILE, FILE_WRITE);

    if (!file) {
      Serial.println("Could not create CSV file.");
      return;
    }

    file.println("type,time_ms,temperature_C,pressure_Pa,pressure_hPa,relative_height_m,estimated_floor,offset_from_estimated_floor_m,actual_floor,correct,calibrated_floor");
    file.close();
  }
}

void appendCSV(
  const char* rowType,
  Reading r,
  bool hasActualFloor,
  int actualFloor,
  bool correct,
  bool hasCalibratedFloor,
  int calibratedFloor
) {
  createCSVHeaderIfNeeded();

  File file = LittleFS.open(CSV_FILE, FILE_APPEND);

  if (!file) {
    Serial.println("Could not open CSV file for append.");
    return;
  }

  file.print(rowType);
  file.print(",");

  file.print(millis());
  file.print(",");

  file.print(r.temperatureC, 2);
  file.print(",");

  file.print(r.pressurePa, 2);
  file.print(",");

  file.print(r.pressurePa / 100.0, 2);
  file.print(",");

  file.print(r.relativeHeightM, 2);
  file.print(",");

  if (strcmp(rowType, "read") == 0 || strcmp(rowType, "test") == 0) {
    file.print(r.estimatedFloor);
  }

  file.print(",");

  if (strcmp(rowType, "read") == 0 || strcmp(rowType, "test") == 0) {
    file.print(r.offsetFromEstimatedFloorM, 2);
  }

  file.print(",");

  if (hasActualFloor) {
    file.print(actualFloor);
  }

  file.print(",");

  if (hasActualFloor) {
    file.print(correct ? "1" : "0");
  }

  file.print(",");

  if (hasCalibratedFloor) {
    file.print(calibratedFloor);
  }

  file.println();
  file.close();
}

void dumpCSV() {
  if (!LittleFS.exists(CSV_FILE)) {
    Serial.println("No CSV file exists yet.");
    return;
  }

  File file = LittleFS.open(CSV_FILE, FILE_READ);

  if (!file) {
    Serial.println("Could not open CSV file.");
    return;
  }

  Serial.println();
  Serial.println("========== CSV START ==========");

  while (file.available()) {
    Serial.write(file.read());
  }

  file.close();

  Serial.println();
  Serial.println("========== CSV END ==========");
  Serial.println();
}

void clearCSV() {
  if (LittleFS.exists(CSV_FILE)) {
    LittleFS.remove(CSV_FILE);
  }

  createCSVHeaderIfNeeded();
  Serial.println("CSV log cleared.");
}

// =======================================================
// BAROMETER READING / TESTING
// =======================================================

Reading getCurrentReading() {
  Reading r;
  r.valid = false;

  float temperatureC;
  float pressurePa;

  if (!readAveragedSensor(temperatureC, pressurePa, LIVE_SAMPLES)) {
    return r;
  }

  r.temperatureC = temperatureC;
  r.pressurePa = pressurePa;
  r.relativeHeightM = heightFromFloor0Pressure(pressurePa);
  r.estimatedFloor = estimateFloor(r.relativeHeightM);
  r.offsetFromEstimatedFloorM = distanceToEstimatedFloor(r.relativeHeightM, r.estimatedFloor);
  r.valid = true;

  return r;
}

void printReading(Reading r) {
  Serial.println("-----------------------------");

  Serial.print("Temperature:       ");
  Serial.print(r.temperatureC, 2);
  Serial.println(" °C");

  Serial.print("Pressure:          ");
  Serial.print(r.pressurePa, 2);
  Serial.println(" Pa");

  Serial.print("Pressure:          ");
  Serial.print(r.pressurePa / 100.0, 2);
  Serial.println(" hPa");

  Serial.print("Relative height:   ");
  Serial.print(r.relativeHeightM, 2);
  Serial.println(" m from floor0");

  Serial.print("Estimated floor:   ");
  Serial.println(r.estimatedFloor);

  Serial.print("Offset from floor: ");
  Serial.print(r.offsetFromEstimatedFloorM, 2);
  Serial.println(" m");
}

void logCalibrationPoint(int floorNumber, float temperatureC, float pressurePa, float relativeHeightM) {
  Reading r;

  r.temperatureC = temperatureC;
  r.pressurePa = pressurePa;
  r.relativeHeightM = relativeHeightM;
  r.estimatedFloor = floorNumber;
  r.offsetFromEstimatedFloorM = 0.0;
  r.valid = true;

  appendCSV("calibration", r, false, 0, false, true, floorNumber);
}

void testActualFloor(int actualFloor) {
  if (!calibrationFinished) {
    Serial.println("Calibration is not finished yet. Type end first.");
    return;
  }

  Reading r = getCurrentReading();

  if (!r.valid) {
    Serial.println("Sensor read failed.");
    return;
  }

  bool correct = (r.estimatedFloor == actualFloor);

  Serial.println();
  Serial.println("========== FLOOR TEST ==========");
  printReading(r);

  Serial.print("Actual floor:      ");
  Serial.println(actualFloor);

  Serial.print("Result:            ");
  Serial.println(correct ? "CORRECT" : "WRONG");

  Serial.println("================================");
  Serial.println();

  appendCSV("test", r, true, actualFloor, correct, false, 0);
}

// =======================================================
// BAROMETER CALIBRATION
// =======================================================

void printCalibrationOverview() {
  recalculateFloorHeights();
  sortFloorsByNumber();

  Serial.println();
  Serial.println("========== CALIBRATION OVERVIEW ==========");

  Serial.print("Configured floors: ");
  Serial.println(floorCount());

  Serial.print("Floor0 pressure: ");
  Serial.print(floor0PressurePa, 2);
  Serial.println(" Pa");

  Serial.println();
  Serial.println("Floor | Pressure [Pa] | Relative height from floor0 [m]");
  Serial.println("--------------------------------------------------------");

  for (int i = 0; i < MAX_FLOORS; i++) {
    if (floors[i].used) {
      Serial.print("  ");
      Serial.print(floors[i].floorNumber);
      Serial.print("   | ");

      Serial.print(floors[i].pressurePa, 2);
      Serial.print("     | ");

      Serial.print(floors[i].relativeHeightM, 2);
      Serial.println();
    }
  }

  Serial.println("==========================================");
  Serial.println();
}

void addOrUpdateFloor(int floorNumber) {
  if (!barometerInitialized) {
    Serial.println("Barometer not initialized.");
    return;
  }

  if (calibrationFinished) {
    Serial.println("Calibration is already finished.");
    Serial.println("Type resetcal to start a new calibration.");
    return;
  }

  Serial.print("Calibrating floor ");
  Serial.print(floorNumber);
  Serial.println(" ... keep sensor still.");

  float temperatureC;
  float pressurePa;

  if (!readAveragedSensor(temperatureC, pressurePa, CALIBRATION_SAMPLES)) {
    Serial.println("Could not read sensor. Floor not saved.");
    return;
  }

  int index = findFloorIndex(floorNumber);

  if (index < 0) {
    index = findFreeFloorSlot();
  }

  if (index < 0) {
    Serial.println("No free floor slots left.");
    return;
  }

  floors[index].floorNumber = floorNumber;
  floors[index].pressurePa = pressurePa;
  floors[index].used = true;

  if (floorNumber == 0) {
    floor0PressurePa = pressurePa;
    floor0Set = true;
  }

  recalculateFloorHeights();

  Serial.println();
  Serial.print("Saved floor ");
  Serial.println(floorNumber);

  Serial.print("Averaged pressure: ");
  Serial.print(pressurePa, 2);
  Serial.println(" Pa");

  Serial.print("Averaged temperature: ");
  Serial.print(temperatureC, 2);
  Serial.println(" °C");

  if (floor0Set) {
    Serial.print("Relative height from floor0: ");
    Serial.print(floors[index].relativeHeightM, 2);
    Serial.println(" m");

    logCalibrationPoint(floorNumber, temperatureC, pressurePa, floors[index].relativeHeightM);
  } else {
    Serial.println("Relative height not calculated yet because floor0 is not set.");
  }

  Serial.println();
}

void finishCalibration() {
  int count = floorCount();

  if (count < 1) {
    Serial.println("No floors configured yet.");
    Serial.println("Set at least floor0 first.");
    return;
  }

  if (!floor0Set) {
    Serial.println("Cannot finish calibration: floor0 is missing.");
    Serial.println("Go to floor 0 and type floor0.");
    return;
  }

  recalculateFloorHeights();
  calibrationFinished = true;

  createCSVHeaderIfNeeded();

  Serial.println();
  Serial.println("Calibration finished.");
  printCalibrationOverview();

  Serial.println("Live barometer output and CSV logging started.");
  Serial.println("Type actual0, actual1, actual2, etc. to compare with estimated floor.");
  Serial.println();
}

void resetCalibration() {
  for (int i = 0; i < MAX_FLOORS; i++) {
    floors[i].used = false;
    floors[i].floorNumber = 0;
    floors[i].pressurePa = 0.0;
    floors[i].relativeHeightM = 0.0;
  }

  calibrationFinished = false;
  floor0Set = false;
  floor0PressurePa = 0.0;

  Serial.println();
  Serial.println("Calibration reset.");
  Serial.println("Set floors again, for example: floor0, floor1, floor2");
  Serial.println("Then type: end");
  Serial.println();
}

void printBarometerHelp() {
  Serial.println();
  Serial.println("Barometer/elevator commands:");
  Serial.println("  floor0       - save current position as floor 0");
  Serial.println("  floor1       - save current position as floor 1");
  Serial.println("  floor2       - save current position as floor 2");
  Serial.println("  floor 3      - save current position as floor 3");
  Serial.println("  end          - finish calibration and start logging");
  Serial.println("  actual0      - compare current estimate with actual floor 0");
  Serial.println("  actual1      - compare current estimate with actual floor 1");
  Serial.println("  actual 2     - compare current estimate with actual floor 2");
  Serial.println("  dumpcsv      - print full CSV log");
  Serial.println("  clearlog     - delete CSV log");
  Serial.println("  overview     - show configured floor values");
  Serial.println("  resetcal     - delete calibration and start over");
  Serial.println("  baro_off     - leave barometer mode");
  Serial.println();
  Serial.println("Global commands:");
  Serial.println("  stop         - stop motor immediately");
  Serial.println("  mode         - print current mode");
  Serial.println();
}

// =======================================================
// MODE SWITCHING
// =======================================================

void enterBarometerMode() {
  if (motorState != IDLE && motorState != HOLD) {
    Serial.println("Cannot enter barometer mode while motor is moving.");
    Serial.println("Send stop first if needed.");
    return;
  }

  stopMotor();
  motorState = IDLE;
  currentPWM = 0;

  if (!initBarometer()) {
    Serial.println("Barometer mode not entered because sensor init failed.");
    return;
  }

  systemMode = MODE_BAROMETER;
  Serial.println("MODE barometer");
  printBarometerHelp();
}

void exitBarometerMode() {
  systemMode = MODE_ROBOT;
  Serial.println("MODE robot");
}

// =======================================================
// COMMAND PARSING
// =======================================================

int parseFloorNumber(String cmd, String prefix) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith(prefix + " ")) {
    return cmd.substring(prefix.length() + 1).toInt();
  }

  if (cmd.startsWith(prefix)) {
    return cmd.substring(prefix.length()).toInt();
  }

  return 9999;
}

void handleRobotCommand(String cmd) {
  if (cmd == "dock") {
    startDock();
  } else if (cmd == "undock") {
    startUndock();
  } else if (cmd == "baro_on" || cmd == "elevator_on" || cmd == "elev_on") {
    enterBarometerMode();
  } else if (cmd == "mode") {
    Serial.println("MODE robot");
  } else if (cmd == "help") {
    Serial.println();
    Serial.println("Robot commands:");
    Serial.println("  dock");
    Serial.println("  undock");
    Serial.println("  stop");
    Serial.println("  baro_on");
    Serial.println("  mode");
    Serial.println("  help");
    Serial.println();
  } else {
    Serial.print("Unknown robot command: ");
    Serial.println(cmd);
  }
}

void handleBarometerCommand(String cmd) {
  if (cmd == "baro_off" || cmd == "elevator_off" || cmd == "elev_off" || cmd == "back") {
    exitBarometerMode();
    return;
  }

  if (cmd == "mode") {
    Serial.println("MODE barometer");
    return;
  }

  if (cmd == "help") {
    printBarometerHelp();
    return;
  }

  if (cmd == "overview") {
    printCalibrationOverview();
    return;
  }

  if (cmd == "resetcal") {
    resetCalibration();
    return;
  }

  if (cmd == "end") {
    finishCalibration();
    return;
  }

  if (cmd == "dumpcsv") {
    dumpCSV();
    return;
  }

  if (cmd == "clearlog") {
    clearCSV();
    return;
  }

  if (cmd.startsWith("floor")) {
    int floorNumber = parseFloorNumber(cmd, "floor");

    if (floorNumber == 9999) {
      Serial.println("Invalid floor command. Use for example: floor0 or floor 2");
      return;
    }

    addOrUpdateFloor(floorNumber);
    return;
  }

  if (cmd.startsWith("actual")) {
    int actualFloor = parseFloorNumber(cmd, "actual");

    if (actualFloor == 9999) {
      Serial.println("Invalid actual command. Use for example: actual0 or actual 2");
      return;
    }

    testActualFloor(actualFloor);
    return;
  }

  Serial.print("Unknown barometer command: ");
  Serial.println(cmd);
  Serial.println("Type help for commands.");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) {
    return;
  }

  // Global highest-priority command
  if (cmd == "stop") {
    emergencyStop();
    return;
  }

  if (systemMode == MODE_ROBOT) {
    handleRobotCommand(cmd);
  } else {
    handleBarometerCommand(cmd);
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}

// =======================================================
// BAROMETER LOOP
// =======================================================

void updateBarometerMode() {
  if (systemMode != MODE_BAROMETER) {
    return;
  }

  if (!calibrationFinished) {
    return;
  }

  unsigned long now = millis();

  if (now - lastBaroPrintTime < BARO_PRINT_INTERVAL_MS) {
    return;
  }

  lastBaroPrintTime = now;

  Reading r = getCurrentReading();

  if (!r.valid) {
    Serial.println("Sensor read failed.");
    return;
  }

  printReading(r);
  appendCSV("read", r, false, 0, false, false, 0);
}

// =======================================================
// SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(BTN_FWD, INPUT_PULLUP);
  pinMode(BTN_REV, INPUT_PULLUP);

  // ESP32 Arduino Core 3.x PWM setup
  ledcAttach(ENA, pwmFreq, pwmResolution);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed. CSV logging will not work.");
  } else {
    Serial.println("LittleFS ready.");
    createCSVHeaderIfNeeded();
  }

  stopMotor();

  Serial.println("READY");
  Serial.println("MODE robot");
  Serial.println("Send baro_on to enable barometer/elevator functions.");
}

void loop() {
  handleSerial();

  // Robot tasks run normally in robot mode.
  handleButtons();
  updateMotorStateMachine();

  // Barometer only works after baro_on and after calibration is finished.
  updateBarometerMode();

  printRobotTelemetry();

  delay(20);
}