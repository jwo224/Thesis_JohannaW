#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------- WiFi ----------
const char* ssid = "student-FoU";
const char* password = "stud2018";

WebServer server(80);

// ---------- Pins ----------
const int fsrPin = 4;
const int servoPin = 3;

// ---------- FSR / Servo settings ----------
const int threshold = 600;
const int maxAngle = 180;
const int minAngle = 0;

// Bigger = slower movement
const int stepDelay = 200;

const int startAngle = 180;
const int stopAngle = 0;

Servo myServo;

int angle = startAngle;
bool running = false;
bool stopped = false;

unsigned long lastStepTime = 0;

// Store latest FSR value so webpage can read it
int latestFsrRaw = 0;

// ---------- FSR average ----------
int readFSRAverage() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(fsrPin);
    delay(1);
  }

  return sum / 10;
}

// ---------- Web page ----------
String getStatusPage() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 FSR Servo Control</title>

  <style>
    body {
      font-family: Arial;
      text-align: center;
      margin: 20px;
      background: #f4f4f4;
    }

    .card {
      background: white;
      padding: 20px;
      margin: auto;
      max-width: 700px;
      border-radius: 12px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.15);
    }

    button {
      font-size: 22px;
      padding: 14px 28px;
      margin: 8px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
    }

    .start { background: #4CAF50; color: white; }
    .reset { background: #2196F3; color: white; }
    .stop  { background: #f44336; color: white; }

    canvas {
      width: 100%;
      height: 300px;
      border: 1px solid #ccc;
      background: white;
      margin-top: 20px;
    }

    .value {
      font-size: 24px;
      margin: 10px;
    }
  </style>
</head>

<body>
  <div class="card">
    <h1>ESP32 FSR Servo Control</h1>

    <div class="value">
      FSR: <span id="fsrValue">0</span>
    </div>

    <div class="value">
      Angle: <span id="angleValue">0</span>
    </div>

    <div class="value">
      State: <span id="stateValue">WAIT</span>
    </div>

    <p>
      <button class="start" onclick="sendCommand('/start')">START</button>
      <button class="reset" onclick="sendCommand('/reset')">RESET</button>
      <button class="stop" onclick="sendCommand('/stop')">STOP</button>
    </p>

    <canvas id="chart" width="700" height="300"></canvas>

    <p>Threshold: <b>600</b></p>
  </div>

<script>
  const canvas = document.getElementById("chart");
  const ctx = canvas.getContext("2d");

  const maxPoints = 150;
  const maxAdc = 4095;
  const threshold = 600;

  let values = [];

  function sendCommand(path) {
    fetch(path)
      .then(() => updateData());
  }

  function drawChart() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Background
    ctx.fillStyle = "white";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Grid lines
    ctx.strokeStyle = "#ddd";
    ctx.lineWidth = 1;

    for (let i = 0; i <= 4; i++) {
      let y = canvas.height - (i / 4) * canvas.height;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(canvas.width, y);
      ctx.stroke();

      ctx.fillStyle = "#555";
      ctx.font = "12px Arial";
      ctx.fillText(Math.round((i / 4) * maxAdc), 5, y - 3);
    }

    // Threshold line
    let thresholdY = canvas.height - (threshold / maxAdc) * canvas.height;
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, thresholdY);
    ctx.lineTo(canvas.width, thresholdY);
    ctx.stroke();

    ctx.fillStyle = "red";
    ctx.fillText("threshold 600", 10, thresholdY - 5);

    // FSR line
    if (values.length < 2) return;

    ctx.strokeStyle = "blue";
    ctx.lineWidth = 2;
    ctx.beginPath();

    for (let i = 0; i < values.length; i++) {
      let x = (i / (maxPoints - 1)) * canvas.width;
      let y = canvas.height - (values[i] / maxAdc) * canvas.height;

      if (i === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    }

    ctx.stroke();
  }

  function updateData() {
    fetch("/data")
      .then(response => response.json())
      .then(data => {
        document.getElementById("fsrValue").innerText = data.fsr;
        document.getElementById("angleValue").innerText = data.angle;
        document.getElementById("stateValue").innerText = data.state;

        values.push(data.fsr);

        if (values.length > maxPoints) {
          values.shift();
        }

        drawChart();
      })
      .catch(error => {
        document.getElementById("stateValue").innerText = "CONNECTION LOST";
      });
  }

  setInterval(updateData, 100);
  updateData();
</script>

</body>
</html>
)rawliteral";

  return page;
}

// ---------- Web handlers ----------
void handleRoot() {
  server.send(200, "text/html", getStatusPage());
}

void handleData() {
  String state;

  if (running) {
    state = "RUN";
  } else if (stopped) {
    state = "STOP";
  } else {
    state = "WAIT";
  }

  String json = "{";
  json += "\"fsr\":" + String(latestFsrRaw) + ",";
  json += "\"angle\":" + String(angle) + ",";
  json += "\"state\":\"" + state + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleStart() {
  angle = startAngle;
  myServo.write(angle);
  delay(1000);

  running = true;
  stopped = false;
  lastStepTime = millis();

  server.send(200, "text/plain", "START");
}

void handleReset() {
  angle = startAngle;
  myServo.write(angle);
  delay(1000);

  running = false;
  stopped = false;

  server.send(200, "text/plain", "RESET");
}

void handleStop() {
  running = false;
  stopped = true;

  server.send(200, "text/plain", "STOP");
}

void setup() {
  delay(2000);  // important for ESP32-C3 Serial Monitor

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("BOOTING...");
  Serial.println("Serial is working.");

  analogReadResolution(12);
  analogSetPinAttenuation(fsrPin, ADC_11db);

  Serial.println("Attaching servo...");
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);

  angle = startAngle;
  myServo.write(angle);
  delay(1500);

  Serial.println("Starting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long wifiStartTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - wifiStartTime < 15000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi FAILED.");
    Serial.print("WiFi status code: ");
    Serial.println(WiFi.status());
    Serial.println("Check SSID/password or try phone hotspot.");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/start", handleStart);
  server.on("/reset", handleReset);
  server.on("/stop", handleStop);

  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  server.handleClient();

  latestFsrRaw = readFSRAverage();

  Serial.print("FSR:");
  Serial.print(latestFsrRaw);
  Serial.print("\tAngle:");
  Serial.print(angle);

  if (!running) {
    if (stopped) {
      Serial.println("\tSTOP");
    } else {
      Serial.println("\tWAIT");
    }
    delay(20);
    return;
  }

  if (latestFsrRaw >= threshold) {
    running = false;
    stopped = true;

    Serial.println("\tSTOP");
    Serial.println("Threshold reached.");
    return;
  }

  Serial.println("\tRUN");

  unsigned long now = millis();

  if (now - lastStepTime >= stepDelay) {
    lastStepTime = now;

    if (angle > stopAngle) {
      angle--;
      myServo.write(angle);
    } else {
      running = false;
      stopped = true;
      Serial.println("STOP: end angle reached.");
    }
  }

  delay(20);
}