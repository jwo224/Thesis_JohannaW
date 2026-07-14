#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>

#define MAX_FLOORS 20
#define PRINT_INTERVAL_MS 1000

#define CALIBRATION_SAMPLES 30
#define LIVE_SAMPLES 10
#define SAMPLE_DELAY_MS 40

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

bool calibrationFinished = false;
bool floor0Set = false;

float floor0PressurePa = 0.0;

unsigned long lastPrintTime = 0;
String inputLine = "";

// ---------------- SENSOR ----------------

bool readSingleSensor(float &temperatureC, float &pressurePa) {
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
    float t, p;

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

// Positive height = above floor0
float heightFromFloor0Pressure(float currentPressurePa) {
  if (!floor0Set || floor0PressurePa <= 0.0) {
    return 0.0;
  }

  return 44330.0 * (1.0 - pow(currentPressurePa / floor0PressurePa, 1.0 / 5.255));
}

// ---------------- FLOOR STORAGE ----------------

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

// ---------------- CSV STREAMING ----------------

void printCSVHeader() {
  Serial.println("CSV_HEADER,type,time_ms,temperature_C,pressure_Pa,pressure_hPa,relative_height_m,estimated_floor,offset_from_estimated_floor_m,actual_floor,correct,calibrated_floor");
}

void printCSVRow(
  const char* rowType,
  Reading r,
  bool hasActualFloor,
  int actualFloor,
  bool correct,
  bool hasCalibratedFloor,
  int calibratedFloor
) {
  Serial.print("CSV,");
  Serial.print(rowType);
  Serial.print(",");

  Serial.print(millis());
  Serial.print(",");

  Serial.print(r.temperatureC, 2);
  Serial.print(",");

  Serial.print(r.pressurePa, 2);
  Serial.print(",");

  Serial.print(r.pressurePa / 100.0, 2);
  Serial.print(",");

  Serial.print(r.relativeHeightM, 2);
  Serial.print(",");

  if (strcmp(rowType, "read") == 0 || strcmp(rowType, "test") == 0) {
    Serial.print(r.estimatedFloor);
  }

  Serial.print(",");

  if (strcmp(rowType, "read") == 0 || strcmp(rowType, "test") == 0) {
    Serial.print(r.offsetFromEstimatedFloorM, 2);
  }

  Serial.print(",");

  if (hasActualFloor) {
    Serial.print(actualFloor);
  }

  Serial.print(",");

  if (hasActualFloor) {
    Serial.print(correct ? "1" : "0");
  }

  Serial.print(",");

  if (hasCalibratedFloor) {
    Serial.print(calibratedFloor);
  }

  Serial.println();
}

void logCalibrationPoint(int floorNumber, float temperatureC, float pressurePa, float relativeHeightM) {
  Reading r;

  r.temperatureC = temperatureC;
  r.pressurePa = pressurePa;
  r.relativeHeightM = relativeHeightM;
  r.estimatedFloor = floorNumber;
  r.offsetFromEstimatedFloorM = 0.0;
  r.valid = true;

  printCSVRow("calibration", r, false, 0, false, true, floorNumber);
}

// ---------------- READING / TESTING ----------------

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

void testActualFloor(int actualFloor) {
  if (!calibrationFinished) {
    Serial.println("Calibration is not finished yet. Type 'end' first.");
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

  printCSVRow("test", r, true, actualFloor, correct, false, 0);
}

// ---------------- CALIBRATION ----------------

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
  if (calibrationFinished) {
    Serial.println("Calibration is already finished.");
    Serial.println("Type 'resetcal' to start a new calibration.");
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
    Serial.println("Go to floor 0 and type: floor0");
    return;
  }

  recalculateFloorHeights();
  calibrationFinished = true;

  Serial.println();
  Serial.println("Calibration finished.");
  printCalibrationOverview();

  Serial.println("CSV logging started.");
  printCSVHeader();

  Serial.println("Type actual0, actual1, actual2, etc. to compare with the estimated floor.");
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

// ---------------- COMMANDS ----------------

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  floor0       - save current position as floor 0");
  Serial.println("  floor1       - save current position as floor 1");
  Serial.println("  floor2       - save current position as floor 2");
  Serial.println("  floor 3      - save current position as floor 3");
  Serial.println("  end          - finish calibration and start logging");
  Serial.println();
  Serial.println("Testing:");
  Serial.println("  actual0      - compare current estimate with actual floor 0");
  Serial.println("  actual1      - compare current estimate with actual floor 1");
  Serial.println("  actual 2     - compare current estimate with actual floor 2");
  Serial.println();
  Serial.println("CSV:");
  Serial.println("  csvheader    - print CSV header");
  Serial.println("  CSV rows are streamed live after calibration.");
  Serial.println();
  Serial.println("Other:");
  Serial.println("  overview     - show configured floor values");
  Serial.println("  resetcal     - delete calibration and start over");
  Serial.println("  help         - show this help");
  Serial.println();
  Serial.println("Important: floor0 must be set before 'end'.");
  Serial.println();
}

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

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "help") {
    printHelp();
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

  if (cmd == "csvheader") {
    printCSVHeader();
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

  Serial.print("Unknown command: ");
  Serial.println(cmd);
  Serial.println("Type 'help' for commands.");
}

void readSerialCommands() {
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

// ---------------- BMP STARTUP ----------------

bool beginBMP() {
  Wire.begin();

  if (bmp.begin_I2C(0x77, &Wire)) {
    Serial.println("BMP390L found at I2C address 0x77");
    return true;
  }

  if (bmp.begin_I2C(0x76, &Wire)) {
    Serial.println("BMP390L found at I2C address 0x76");
    return true;
  }

  return false;
}

// ---------------- SETUP / LOOP ----------------

void setup() {
  Serial.begin(115200);

  // XIAO nRF52840 uses native USB serial.
  // This waits until Serial Monitor or MATLAB connects.
  while (!Serial) {
    delay(10);
  }

  delay(1000);

  Serial.println();
  Serial.println("XIAO nRF52840 Sense Floor Detection + CSV Streaming");
  Serial.println();

  if (!beginBMP()) {
    Serial.println("Could not find BMP390L sensor!");
    Serial.println("Check wiring:");
    Serial.println("SEN0423 VCC -> XIAO 3V3");
    Serial.println("SEN0423 GND -> XIAO GND");
    Serial.println("SEN0423 SDA -> XIAO SDA / D4");
    Serial.println("SEN0423 SCL -> XIAO SCL / D5");

    while (1) {
      delay(1000);
    }
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  Serial.println("Sensor initialized.");
  printHelp();
}

void loop() {
  readSerialCommands();

  if (!calibrationFinished) {
    return;
  }

  unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    Reading r = getCurrentReading();

    if (!r.valid) {
      Serial.println("Sensor read failed.");
      return;
    }

    printReading(r);

    // Live CSV row without manual actual-floor comparison
    printCSVRow("read", r, false, 0, false, false, 0);
  }
}