#include <WiFi.h>
#include <WebServer.h>

// Motor Driver Pins (TB6612FNG)
const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 22;
const int PWMB = 17;

const int STBY = 5;
const int LED_PIN = 2;

// PID and Speed Control Variables
int masterSpeed = 140;          
float Kp = 0.12;                // Increased slightly for quicker response
float Kd = 1.20;                // Increased to dampen overshooting on turn exits

float error = 0;
float lastError = 0;

bool isCalibrated = false;
bool botRunning = false;

const float FACTOR_A = 1.00;
const float FACTOR_B = 170.0 / 200.0;

// QTR-8A Analog Pins (6 ADC1 pins)
const int NUM_SENSORS = 6;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33};

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorValues[NUM_SENSORS];

WebServer server(80);

void setMotors(int dirA, int speedA, int dirB, int speedB);
void stopMotors();
void runPID();

const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 PID Hybrid Line Follower</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #121214; color: white; margin: 0; padding: 15px; }
    .card { background: #1e1e24; padding: 20px; border-radius: 12px; max-width: 360px; margin: auto; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
    h2 { margin-top: 0; color: #007bff; }
    .control-group { margin: 20px 0; text-align: left; }
    label { font-size: 14px; font-weight: bold; color: #ccc; }
    .val { float: right; color: #007bff; font-weight: bold; }
    input[type=range] { width: 100%; height: 8px; margin-top: 8px; }
    .start-btn { width: 100%; padding: 15px; font-size: 18px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; margin-top: 10px; }
    .btn-off { background: #28a745; color: white; }
    .btn-on { background: #dc3545; color: white; }
    #status { font-weight: bold; margin-bottom: 15px; color: #ffc107; }
  </style>
</head>
<body>
  <div class="card">
    <h2>PID + U-Turn Control</h2>
    <div id="status">STATUS: INITIALIZING...</div>

    <button id="toggleBtn" class="start-btn btn-off" onclick="toggleBot()">START BOT</button>

    <div class="control-group">
      <label>Master Speed <span class="val" id="speedVal">140</span></label>
      <input type="range" min="60" max="220" value="140" oninput="updateParams()" id="speedSlider">
    </div>

    <div class="control-group">
      <label>Kp (Proportional) <span class="val" id="kpVal">0.12</span></label>
      <input type="range" min="0.00" max="0.30" step="0.01" value="0.12" oninput="updateParams()" id="kpSlider">
    </div>

    <div class="control-group">
      <label>Kd (Derivative) <span class="val" id="kdVal">1.20</span></label>
      <input type="range" min="0.00" max="3.00" step="0.05" value="1.20" oninput="updateParams()" id="kdSlider">
    </div>
  </div>

  <script>
    function updateParams() {
      let speed = document.getElementById('speedSlider').value;
      let kp = document.getElementById('kpSlider').value;
      let kd = document.getElementById('kdSlider').value;

      document.getElementById('speedVal').innerText = speed;
      document.getElementById('kpVal').innerText = kp;
      document.getElementById('kdVal').innerText = kd;

      fetch(`/setparams?speed=${speed}&kp=${kp}&kd=${kd}`);
    }

    function toggleBot() {
      fetch('/toggle').then(res => res.text()).then(state => {
        let btn = document.getElementById('toggleBtn');
        let status = document.getElementById('status');
        if (state === "RUNNING") {
          btn.innerText = "STOP BOT";
          btn.className = "start-btn btn-on";
          status.innerText = "STATUS: RUNNING";
          status.style.color = "#28a745";
        } else {
          btn.innerText = "START BOT";
          btn.className = "start-btn btn-off";
          status.innerText = "STATUS: READY / PAUSED";
          status.style.color = "#ffc107";
        }
      });
    }

    setInterval(() => {
      fetch('/status').then(res => res.text()).then(msg => {
        if (msg === "CALIBRATED_WAITING" && !window.calibrated) {
          window.calibrated = true;
          document.getElementById('status').innerText = "STATUS: READY / PAUSED";
        }
      });
    }, 1000);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", HTML_CONTENT); }

void handleSetParams() {
  if (server.hasArg("speed") && server.hasArg("kp") && server.hasArg("kd")) {
    masterSpeed = server.arg("speed").toInt();
    Kp = server.arg("kp").toFloat();
    Kd = server.arg("kd").toFloat();
  }
  server.send(200, "text/plain", "OK");
}

void handleToggle() {
  if (!isCalibrated) {
    server.send(200, "text/plain", "NOT_CALIBRATED");
    return;
  }
  botRunning = !botRunning;
  if (!botRunning) stopMotors();
  server.send(200, "text/plain", botRunning ? "RUNNING" : "STOPPED");
}

void handleStatus() {
  if (isCalibrated && !botRunning) server.send(200, "text/plain", "CALIBRATED_WAITING");
  else if (botRunning) server.send(200, "text/plain", "RUNNING");
  else server.send(200, "text/plain", "CALIBRATING");
}

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); pinMode(LED_PIN, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  WiFi.softAP("ESP32_PID_Bot", "12345678");
  server.on("/", handleRoot);
  server.on("/setparams", handleSetParams);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println("Calibration starting in 5 seconds...");
  for (int i = 5; i > 0; i--) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW); delay(800);
    server.handleClient();
  }

  Serial.println("CALIBRATING NOW: SWEEP SENSORS OVER LINE!");
  unsigned long startTime = millis();
  unsigned long lastBlink = 0;
  bool ledState = false;

  while (millis() - startTime < 8000) {
    server.handleClient();
    if (millis() - lastBlink > 100) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(5);
  }

  digitalWrite(LED_PIN, HIGH);
  isCalibrated = true;
  stopMotors();
}

void loop() {
  server.handleClient();
  if (botRunning && isCalibrated) {
    runPID();
  } else {
    stopMotors();
  }
}

// -------------------------------------------------------------
// PID Execution with Dynamic Speed Scaling & U-Turn Overrides
// -------------------------------------------------------------
void runPID() {
  long weightedSum = 0;
  long totalSum = 0;
  int normValues[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normVal = constrain(normVal, 0, 1000);

    if (normVal < 200) normVal = 0; // Filter out surface noise
    normValues[i] = normVal;

    totalSum += normVal;
    weightedSum += (long)normVal * (i * 1000);
  }

  // -------------------------------------------------------------
  // 1. HARD OVERRIDE FOR EXTREME U-TURNS / 90° SHARP CORNERS
  // -------------------------------------------------------------
  if (normValues[0] > 600 && normValues[2] == 0 && normValues[3] == 0) {
    // Sharp Left U-Turn detected on sensor 0
    setMotors(-1, 150 * FACTOR_A, 1, 150 * FACTOR_B);
    lastError = -2500;
    return;
  }
  if (normValues[5] > 600 && normValues[2] == 0 && normValues[3] == 0) {
    // Sharp Right U-Turn detected on sensor 5
    setMotors(1, 150 * FACTOR_A, -1, 150 * FACTOR_B);
    lastError = 2500;
    return;
  }

  // -------------------------------------------------------------
  // 2. ERROR COMPUTATION & LINE LOST RECOVERY
  // -------------------------------------------------------------
  if (totalSum > 300) {
    float position = (float)weightedSum / totalSum; // Center = 2500
    error = position - 2500.0;
  } else {
    // Line lost: spin in direction of last known position
    if (lastError < 0) {
      setMotors(-1, 140 * FACTOR_A, 1, 140 * FACTOR_B);
    } else {
      setMotors(1, 140 * FACTOR_A, -1, 140 * FACTOR_B);
    }
    return;
  }

  // -------------------------------------------------------------
  // 3. DYNAMIC BASE SPEED REDUCTION (Pivoting Factor)
  // -------------------------------------------------------------
  // As error approaches max (2500), forward speed drops by up to 70%
  float speedReduction = 1.0 - (abs(error) / 2500.0) * 0.70;
  float currentBaseSpeed = masterSpeed * speedReduction;

  // -------------------------------------------------------------
  // 4. STANDARD PID CALCULATIONS
  // -------------------------------------------------------------
  float pTerm = Kp * error;
  float dTerm = Kd * (error - lastError);
  float pidOutput = pTerm + dTerm;

  lastError = error;

  float baseSpeedA = currentBaseSpeed * FACTOR_A;
  float baseSpeedB = currentBaseSpeed * FACTOR_B;

  float motorSpeedA = baseSpeedA + pidOutput;
  float motorSpeedB = baseSpeedB - pidOutput;

  int dirA = (motorSpeedA >= 0) ? 1 : -1;
  int dirB = (motorSpeedB >= 0) ? 1 : -1;

  setMotors(dirA, abs((int)motorSpeedA), dirB, abs((int)motorSpeedB));
}

void setMotors(int dirA, int speedA, int dirB, int speedB) {
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  if (dirA == 1) { 
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  } else if (dirA == -1) { 
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else { 
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, speedA);

  if (dirB == 1) { 
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  } else if (dirB == -1) { 
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else { 
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, speedB);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}
