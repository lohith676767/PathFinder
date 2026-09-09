// Motor Driver Pins (TB6612FNG)
const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 22;
const int PWMB = 17;

const int STBY = 5;

// Built-in LED Pin
const int LED_PIN = 2;

// PID and Speed Control Variables (defaults; live-updatable over UART from WiFi ESP)
int masterSpeed = 100;          // Optimal base speed
float Kp = 0.18;                // Proportional constant
float Kd = 0;                // Derivative constant

// UART link to WiFi ESP (RX=GPIO4, TX=GPIO27) - keeps 25/26 free for sensors
HardwareSerial UARTLink(2);
char uartBuffer[64];
int uartLen = 0;
unsigned long lastStatusSend = 0;

float error = 0;
float lastError = 0;

// State Control Flags
bool isCalibrated = false;
bool botRunning = false;

// Speed ratio factor (170/200 = 0.85 for Right Motor)
const float FACTOR_A = 1.00;
const float FACTOR_B = 170.0 / 200.0;

// QTR-8A Analog Pins (D1..D8, outer two on ADC2 since WiFi is off)
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {25, 36, 39, 34, 35, 32, 33, 26};

// Calibration boundaries
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorValues[NUM_SENSORS];
int normValues[NUM_SENSORS]; // latest normalized (0=white..1000=black) readings
unsigned long lastSensorSend = 0;

// Function Prototypes
void setMotors(int dirA, int speedA, int dirB, int speedB);
void stopMotors();
void runPID();

// -------------------------------------------------------------
// UART Command Handling (from WiFi ESP)
// Format in:  "C,<masterSpeed>,<Kp>,<Kd>,<run0or1>\n"
// Format out: "S,<isCalibrated0or1>,<botRunning0or1>\n"
// -------------------------------------------------------------
void processUARTCommand(char* line) {
  if (!(line[0] == 'C' && line[1] == ',')) return;

  char* save;
  char* tok = strtok_r(line + 2, ",", &save);
  if (!tok) return;
  int speedVal = atoi(tok);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return;
  float kpVal = atof(tok);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return;
  float kdVal = atof(tok);

  tok = strtok_r(NULL, ",", &save);
  if (!tok) return;
  int runFlag = atoi(tok);

  masterSpeed = speedVal;
  Kp = kpVal;
  Kd = kdVal;

  if (isCalibrated) {
    botRunning = (runFlag == 1);
    if (!botRunning) stopMotors();
  }
}

void handleUART() {
  while (UARTLink.available()) {
    char c = UARTLink.read();
    if (c == '\n') {
      uartBuffer[uartLen] = '\0';
      processUARTCommand(uartBuffer);
      uartLen = 0;
    } else if (c != '\r') {
      if (uartLen < (int)sizeof(uartBuffer) - 1) {
        uartBuffer[uartLen++] = c;
      } else {
        uartLen = 0; // overflow guard: discard oversized/garbled line
      }
    }
  }

  if (millis() - lastStatusSend > 300) {
    lastStatusSend = millis();
    UARTLink.print("S,");
    UARTLink.print(isCalibrated ? 1 : 0);
    UARTLink.print(",");
    UARTLink.println(botRunning ? 1 : 0);
  }
}

void setup() {
  Serial.begin(115200);
  UARTLink.begin(115200, SERIAL_8N1, 4, 27); // RX=GPIO4, TX=GPIO27
  analogSetAttenuation(ADC_11db);

  // Pin Declarations
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  // -------------------------------------------------------------
  // Countdown Phase (5 Seconds - Slow Blinking)
  // -------------------------------------------------------------
  Serial.println("Get ready! Calibration starts in 5 seconds...");
  for (int i = 5; i > 0; i--) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(800);
  }

  // -------------------------------------------------------------
  // Active Calibration Phase (8 Seconds - Fast Blinking)
  // -------------------------------------------------------------
  Serial.println("CALIBRATING NOW: SWEEP SENSORS OVER LINE!");
  unsigned long startTime = millis();
  unsigned long lastBlink = 0;
  bool ledState = false;

  while (millis() - startTime < 8000) {
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
  Serial.println("Calibration Complete! Waiting for START command over UART...");
}

// -------------------------------------------------------------
// Reads all sensors and stores normalized (0=white..1000=black)
// values into normValues[]. Called every loop once calibrated so
// live readings are available whether or not the bot is running.
// -------------------------------------------------------------
void readSensorsNormalized() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normVal = constrain(normVal, 0, 1000);

    if (normVal < 200) normVal = 0; // Filter out background noise

    normValues[i] = normVal;
  }
}

// Sends live sensor readings out over UART ("D,v0,v1,...") and prints
// them to the USB serial monitor for local debugging.
void sendSensorData() {
  if (millis() - lastSensorSend < 150) return;
  lastSensorSend = millis();

  UARTLink.print('D');
  Serial.print('D');
  for (int i = 0; i < NUM_SENSORS; i++) {
    UARTLink.print(',');
    UARTLink.print(normValues[i]);
    Serial.print(',');
    Serial.print(normValues[i]);
  }
  UARTLink.println();
  Serial.println();
}

void loop() {
  handleUART();

  if (isCalibrated) {
    readSensorsNormalized();
    sendSensorData();
  }

  if (botRunning && isCalibrated) {
    runPID();
  } else {
    stopMotors();
  }
}

// -------------------------------------------------------------
// PID Calculation & Execution
// -------------------------------------------------------------
void runPID() {
  long weightedSum = 0;
  long totalSum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int normVal = normValues[i];
    totalSum += normVal;
    weightedSum += (long)normVal * (i * 1000);
  }

  // Calculate position error
  if (totalSum > 300) {
    float position = (float)weightedSum / totalSum; // Position range: 0 to 7000 (Center = 3500)
    error = position - 3500.0;
  } else {
    // Line lost: maintain hard spin in direction of last known error
    if (lastError < 0) error = -3500.0;
    else error = 3500.0;
  }

  // Calculate PID Output
  float pTerm = Kp * error;
  float dTerm = Kd * (error - lastError);
  float pidOutput = pTerm + dTerm;

  if (totalSum > 300) {
    lastError = error;
  }

  // --- Speed Scheduling: slow down proportionally to how sharp the turn is ---
  float errorRatio = fabs(error) / 3500.0; // 0 = straight, 1 = max turn/line-lost
  errorRatio = constrain(errorRatio, 0.0, 1.0);
  const float TURN_SLOWDOWN = 0.65; // up to 65% speed cut on the sharpest turns
  float effectiveSpeed = masterSpeed * (1.0 - TURN_SLOWDOWN * errorRatio);

  // Calculate Base Speeds with Right-Motor Adjustment Factor
  float baseSpeedA = effectiveSpeed * FACTOR_A;
  float baseSpeedB = effectiveSpeed * FACTOR_B;

  // Apply Differential Steering Output
  float motorSpeedA = baseSpeedA + pidOutput;
  float motorSpeedB = baseSpeedB - pidOutput;

  // --- Pivot Turning: on sharp turns, rotate in place instead of a slow differential ---
  const float PIVOT_THRESHOLD = 0.80; // errorRatio above this triggers a point-turn
  const int PIVOT_SPEED = 120;        // fixed rotation speed while pivoting
  if (errorRatio > PIVOT_THRESHOLD) {
    if (error > 0) {
      motorSpeedA = PIVOT_SPEED;   // outer (left) wheel forward
      motorSpeedB = -PIVOT_SPEED;  // inner (right) wheel reverse
    } else {
      motorSpeedA = -PIVOT_SPEED;
      motorSpeedB = PIVOT_SPEED;
    }
  }

  // Direction and Output Determination
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
