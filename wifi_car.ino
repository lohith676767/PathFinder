#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi Credentials for Access Point Mode
const char* ssid = "ESP32-Car";
const char* password = "12345678";

WebServer server(80);

// Pin Definitions
const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 4;
const int PWMB = 17;

const int STBY = 5;

const int MOTOR_SPEED = 200; // 0 to 255

// HTML & JavaScript Web Page Interface
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 Wi-Fi Car</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #1a1a1a; color: white; margin: 0; padding-top: 30px; }
    h2 { margin-bottom: 20px; }
    .grid { display: inline-grid; grid-template-columns: repeat(3, 90px); grid-gap: 15px; }
    .btn { width: 90px; height: 90px; font-size: 28px; border: none; border-radius: 16px; background: #007bff; color: white; font-weight: bold; cursor: pointer; touch-action: manipulation; }
    .btn:active { background: #0056b3; }
    .stop { background: #dc3545; }
    .stop:active { background: #a71d2a; }
    .empty { visibility: hidden; }
  </style>
</head>
<body>
  <h2>ESP32 Robot Controller</h2>
  <div class="grid">
    <div class="empty"></div>
    <button class="btn" onclick="cmd('forward')">&#9650;</button>
    <div class="empty"></div>
    
    <button class="btn" onclick="cmd('left')">&#9664;</button>
    <button class="btn stop" onclick="cmd('stop')">&#9632;</button>
    <button class="btn" onclick="cmd('right')">&#9654;</button>
    
    <div class="empty"></div>
    <button class="btn" onclick="cmd('reverse')">&#9660;</button>
    <div class="empty"></div>
  </div>

  <script>
    function cmd(direction) {
      fetch('/' + direction);
    }
  </script>
</body>
</html>
)rawliteral";

// Motor Action Functions
void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void moveForward() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, MOTOR_SPEED);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, MOTOR_SPEED);
}

void moveReverse() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, MOTOR_SPEED);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, MOTOR_SPEED);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  analogWrite(PWMB, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  analogWrite(PWMA, MOTOR_SPEED);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, MOTOR_SPEED);
}

void setup() {
  // Pin modes
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Driver active
  digitalWrite(STBY, HIGH);
  stopMotors();

  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);

  // Web routes
  server.on("/", []() { server.send(200, "text/html", HTML_PAGE); });
  server.on("/forward", []() { moveForward(); server.send(200, "text/plain", "OK"); });
  server.on("/reverse", []() { moveReverse(); server.send(200, "text/plain", "OK"); });
  server.on("/left", []() { turnLeft(); server.send(200, "text/plain", "OK"); });
  server.on("/right", []() { turnRight(); server.send(200, "text/plain", "OK"); });
  server.on("/stop", []() { stopMotors(); server.send(200, "text/plain", "OK"); });

  server.begin();
}

void loop() {
  server.handleClient();
}
