// Runs on the SECOND ESP32. Hosts the web UI and forwards
// speed/Kp/Kd/start-stop to the main (motor) ESP32 over UART.
//
// Wiring (common GND required):
//   This ESP   TX = GPIO19  ---->  Main ESP  RX2 = GPIO4
//   This ESP   RX = GPIO18  <----  Main ESP  TX2 = GPIO27
//
// NOTE: moved off the default UART2 pins (16/17) - on WROVER-class ESP32
// boards those pins are used internally for PSRAM, and contending with them
// can cause exactly the symptom of a crash/reboot loop (blinking LED, AP
// never fully comes up). GPIO18/19 are free general-purpose pins on both
// WROOM and WROVER boards.

#include <WiFi.h>
#include <WebServer.h>

HardwareSerial UARTLink(2); // RX=18, TX=19 (moved off default 16/17 - see note above)

int masterSpeed = 100;
float Kp = 0.18;
float Kd = 0.0;

bool botRunning = false;     // local toggle intent
bool isCalibrated = false;   // last known state from main ESP
bool remoteRunning = false;  // last confirmed state from main ESP

const int NUM_SENSORS = 8;
int sensorValues[NUM_SENSORS] = {0}; // latest normalized (0=white..1000=black) readings from main ESP

// Run history: logs the line-position error (same formula as main ESP's PID)
// for every sensor update received while the bot is running, so a path/turn
// summary can be shown once the run is stopped.
const int MAX_LOG = 600;
float runLog[MAX_LOG];
int runLogLen = 0;
bool wasRunning = false;

char uartBuffer[96];
int uartLen = 0;

char sensorBuf[64];             // reused buffer for /sensors response
char reportBuf[MAX_LOG * 9 + 8]; // reused buffer for /report response ("-3500.0," per sample)

WebServer server(80);

const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 PID Line Follower</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #121214; color: white; margin: 0; padding: 15px; }
    .card { background: #1e1e24; padding: 20px; border-radius: 12px; max-width: 360px; margin: auto; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
    h2 { margin-top: 0; color: #007bff; }
    .control-group { margin: 20px 0; text-align: left; }
    label { font-size: 14px; font-weight: bold; color: #ccc; }
    .val { float: right; color: #007bff; font-weight: bold; }
    input[type=range] { width: 100%; height: 8px; margin-top: 8px; }
    .start-btn { width: 100%; padding: 15px; font-size: 18px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: 0.2s; margin-top: 10px; }
    .btn-off { background: #28a745; color: white; }
    .btn-on { background: #dc3545; color: white; }
    #status { font-weight: bold; margin-bottom: 15px; color: #ffc107; }
    .sensor-strip { display: flex; gap: 4px; }
    .sensor-box {
      flex: 1; height: 60px; border-radius: 6px; background: #ffffff;
      display: flex; flex-direction: column; align-items: center; justify-content: flex-end;
      padding-bottom: 4px; border: 1px solid #333; transition: background 0.1s linear;
    }
    .sensor-box .lbl { font-size: 10px; color: #888; }
    .sensor-box .num { font-size: 10px; font-weight: bold; }
    #reportSection { display: none; }
    #reportCanvas { width: 100%; height: 140px; background: #2a2a32; border-radius: 6px; }
    #reportSummary { font-size: 12px; color: #ccc; margin-top: 8px; display: flex; justify-content: space-between; }
    #reportSummary span.straight { color: #28a745; }
    #reportSummary span.curve { color: #ffc107; }
    #reportSummary span.sharp { color: #dc3545; }
  </style>
</head>
<body>
  <div class="card">
    <h2>PID Control Panel</h2>
    <div id="status">STATUS: INITIALIZING...</div>

    <button id="toggleBtn" class="start-btn btn-off" onclick="toggleBot()">START BOT</button>

    <div class="control-group">
      <label>Master Speed <span class="val" id="speedVal">100</span></label>
      <input type="range" min="60" max="220" value="100" oninput="updateParams()" id="speedSlider">
    </div>

    <div class="control-group">
      <label>Kp (Proportional) <span class="val" id="kpVal">0.18</span></label>
      <input type="range" min="0.00" max="0.30" step="0.01" value="0.18" oninput="updateParams()" id="kpSlider">
    </div>

    <div class="control-group">
      <label>Kd (Derivative) <span class="val" id="kdVal">0.00</span></label>
      <input type="range" min="0.00" max="3.00" step="0.05" value="0.00" oninput="updateParams()" id="kdSlider">
    </div>

    <div class="control-group">
      <label>Live Sensors (left-to-right = D1..D8)</label>
      <div class="sensor-strip" id="sensorStrip"></div>
    </div>

    <div class="control-group" id="reportSection">
      <label>Last Run - What The Bot Saw</label>
      <canvas id="reportCanvas"></canvas>
      <div id="reportSummary">
        <span class="straight" id="repStraight">Straight: 0%</span>
        <span class="curve" id="repCurve">Curves: 0%</span>
        <span class="sharp" id="repSharp">Sharp Turns: 0%</span>
      </div>
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
        } else if (state === "STOPPED") {
          btn.innerText = "START BOT";
          btn.className = "start-btn btn-off";
          status.innerText = "STATUS: READY / PAUSED";
          status.style.color = "#ffc107";
          showRunReport();
        }
      });
    }

    function showRunReport() {
      fetch('/report').then(res => res.text()).then(csv => {
        csv = csv.trim();
        if (!csv) return;
        let errs = csv.split(',').map(Number);
        drawReport(errs);
        document.getElementById('reportSection').style.display = 'block';
      });
    }

    function classify(ratio) {
      if (ratio > 0.80) return 'sharp';
      if (ratio > 0.30) return 'curve';
      return 'straight';
    }

    function drawReport(errs) {
      let canvas = document.getElementById('reportCanvas');
      let dpr = window.devicePixelRatio || 1;
      let w = canvas.clientWidth, h = canvas.clientHeight;
      canvas.width = w * dpr;
      canvas.height = h * dpr;
      let ctx = canvas.getContext('2d');
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      // center reference line (straight ahead)
      ctx.strokeStyle = '#555';
      ctx.beginPath();
      ctx.moveTo(0, h / 2);
      ctx.lineTo(w, h / 2);
      ctx.stroke();

      const colors = { straight: '#28a745', curve: '#ffc107', sharp: '#dc3545' };
      let counts = { straight: 0, curve: 0, sharp: 0 };

      let n = errs.length;
      for (let i = 0; i < n; i++) {
        let ratio = Math.min(Math.abs(errs[i]) / 3500, 1.0);
        let cls = classify(ratio);
        counts[cls]++;
      }

      ctx.lineWidth = 2;
      for (let i = 1; i < n; i++) {
        let x0 = ((i - 1) / (n - 1)) * w;
        let x1 = (i / (n - 1)) * w;
        // errs range -3500..3500 -> y (inverted: positive error = line to the right = plotted below center)
        let y0 = h / 2 + (errs[i - 1] / 3500) * (h / 2 - 6);
        let y1 = h / 2 + (errs[i] / 3500) * (h / 2 - 6);
        let ratio = Math.min(Math.abs(errs[i]) / 3500, 1.0);
        ctx.strokeStyle = colors[classify(ratio)];
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
      }

      let pct = (c) => n ? Math.round((c / n) * 100) : 0;
      document.getElementById('repStraight').innerText = `Straight: ${pct(counts.straight)}%`;
      document.getElementById('repCurve').innerText = `Curves: ${pct(counts.curve)}%`;
      document.getElementById('repSharp').innerText = `Sharp Turns: ${pct(counts.sharp)}%`;
    }

    setInterval(() => {
      fetch('/status').then(res => res.text()).then(msg => {
        let status = document.getElementById('status');
        let btn = document.getElementById('toggleBtn');
        if (msg === "RUNNING") {
          status.innerText = "STATUS: RUNNING";
          status.style.color = "#28a745";
          btn.innerText = "STOP BOT";
          btn.className = "start-btn btn-on";
        } else if (msg === "CALIBRATED_WAITING") {
          status.innerText = "STATUS: READY / PAUSED";
          status.style.color = "#ffc107";
          btn.innerText = "START BOT";
          btn.className = "start-btn btn-off";
        } else {
          status.innerText = "STATUS: CALIBRATING...";
          status.style.color = "#ffc107";
        }
      });
    }, 500);

    let sensorBoxesBuilt = false;
    function buildSensorBoxes(n) {
      let container = document.getElementById('sensorStrip');
      container.innerHTML = '';
      for (let i = 0; i < n; i++) {
        container.innerHTML += `
          <div class="sensor-box" id="box${i}">
            <span class="num" id="num${i}">0</span>
            <span class="lbl">D${i + 1}</span>
          </div>`;
      }
      sensorBoxesBuilt = true;
    }

    setInterval(() => {
      fetch('/sensors').then(res => res.text()).then(csv => {
        let vals = csv.trim().split(',').map(Number);
        if (!sensorBoxesBuilt) buildSensorBoxes(vals.length);
        vals.forEach((v, i) => {
          let box = document.getElementById(`box${i}`);
          let num = document.getElementById(`num${i}`);
          if (box && num) {
            // v: 0 (white) .. 1000 (black) -> map to grayscale box color
            let gray = 255 - Math.round((v / 1000) * 255);
            box.style.background = `rgb(${gray},${gray},${gray})`;
            num.style.color = gray < 128 ? '#fff' : '#000';
            num.innerText = v;
          }
        });
      });
    }, 200);
  </script>
</body>
</html>
)rawliteral";

void sendCommand() {
  UARTLink.print("C,");
  UARTLink.print(masterSpeed);
  UARTLink.print(",");
  UARTLink.print(Kp, 3);
  UARTLink.print(",");
  UARTLink.print(Kd, 3);
  UARTLink.print(",");
  UARTLink.println(botRunning ? 1 : 0);
}

void handleRoot() {
  server.send(200, "text/html", HTML_CONTENT);
}

void handleSetParams() {
  if (server.hasArg("speed") && server.hasArg("kp") && server.hasArg("kd")) {
    masterSpeed = server.arg("speed").toInt();
    Kp = server.arg("kp").toFloat();
    Kd = server.arg("kd").toFloat();
    sendCommand();
  }
  server.send(200, "text/plain", "OK");
}

void handleToggle() {
  if (!isCalibrated) {
    server.send(200, "text/plain", "NOT_CALIBRATED");
    return;
  }
  botRunning = !botRunning;
  sendCommand();
  server.send(200, "text/plain", botRunning ? "RUNNING" : "STOPPED");
}

void handleStatus() {
  if (isCalibrated && !remoteRunning) {
    server.send(200, "text/plain", "CALIBRATED_WAITING");
  } else if (remoteRunning) {
    server.send(200, "text/plain", "RUNNING");
  } else {
    server.send(200, "text/plain", "CALIBRATING");
  }
}

void handleReport() {
  int pos = 0;
  for (int i = 0; i < runLogLen; i++) {
    if (pos >= (int)sizeof(reportBuf) - 12) break; // leave room for one more field
    if (i > 0) reportBuf[pos++] = ',';
    pos += snprintf(reportBuf + pos, sizeof(reportBuf) - pos, "%.1f", runLog[i]);
  }
  reportBuf[pos] = '\0';
  server.send(200, "text/plain", reportBuf);
}

void handleSensors() {
  int pos = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) sensorBuf[pos++] = ',';
    pos += snprintf(sensorBuf + pos, sizeof(sensorBuf) - pos, "%d", sensorValues[i]);
  }
  sensorBuf[pos] = '\0';
  server.send(200, "text/plain", sensorBuf);
}

// Parses one full line ("S,<isCalibrated>,<botRunning>" or "D,<v0>,...,<v7>").
// Modifies `line` in place (strtok_r inserts null terminators at commas).
void handleUARTLine(char* line) {
  if (line[0] == 'S' && line[1] == ',') {
    char* save;
    char* tok = strtok_r(line + 2, ",", &save);
    if (!tok) return;
    int cal = atoi(tok);

    tok = strtok_r(NULL, ",", &save);
    if (!tok) return;
    int run = atoi(tok);

    isCalibrated = (cal == 1);
    remoteRunning = (run == 1);

    if (remoteRunning && !wasRunning) {
      runLogLen = 0; // new run started, clear previous trace
    }
    wasRunning = remoteRunning;
  } else if (line[0] == 'D' && line[1] == ',') {
    char* save;
    char* tok = strtok_r(line + 2, ",", &save);
    int i = 0;
    while (tok && i < NUM_SENSORS) {
      sensorValues[i++] = atoi(tok);
      tok = strtok_r(NULL, ",", &save);
    }

    // Same weighted-centroid math as the main ESP's PID, purely for the
    // post-run trace (does not affect motor control).
    if (remoteRunning && runLogLen < MAX_LOG) {
      long weightedSum = 0;
      long totalSum = 0;
      for (int j = 0; j < NUM_SENSORS; j++) {
        totalSum += sensorValues[j];
        weightedSum += (long)sensorValues[j] * (j * 1000);
      }
      float err;
      if (totalSum > 300) {
        err = ((float)weightedSum / totalSum) - 3500.0;
      } else {
        err = (runLogLen > 0 && runLog[runLogLen - 1] < 0) ? -3500.0 : 3500.0;
      }
      runLog[runLogLen++] = err;
    }
  }
}

// Reads "S,<isCalibrated>,<botRunning>\n" and "D,<v0>,...,<v7>\n" pushed by the main ESP.
void readUARTStatus() {
  while (UARTLink.available()) {
    char c = UARTLink.read();
    if (c == '\n') {
      uartBuffer[uartLen] = '\0';
      handleUARTLine(uartBuffer);
      uartLen = 0;
    } else if (c != '\r') {
      if (uartLen < (int)sizeof(uartBuffer) - 1) {
        uartBuffer[uartLen++] = c;
      } else {
        uartLen = 0; // overflow guard: discard oversized/garbled line
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  UARTLink.begin(115200, SERIAL_8N1, 18, 19); // RX=GPIO18, TX=GPIO19

  WiFi.softAP("ESP32_PID_Bot", "12345678");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/setparams", handleSetParams);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.on("/sensors", handleSensors);
  server.on("/report", handleReport);
  server.begin();
}

void loop() {
  server.handleClient();
  readUARTStatus();
}
