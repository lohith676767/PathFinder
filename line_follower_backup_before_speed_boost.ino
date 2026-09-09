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

// Line-loss recovery timing (see runPID()): tracks how long the line has
// been continuously lost, so the recovery behavior can escalate over time
// instead of arcing forward forever.
unsigned long lineLostSince = 0;
bool wasLineLost = false;

// State Control Flags
bool isCalibrated = false;
bool botRunning = false;
bool wasBotRunning = false;    // detects the false->true start transition
unsigned long runStartTime = 0; // millis() when the current run started (for the startup ramp)

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

  // One-shot sensor health report: how much each sensor's reading actually
  // swung between white and black during the calibration sweep. A sensor
  // that barely moved (loose wire, dead sensor, didn't get swept over the
  // line) is caught here instead of silently degrading the run.
  UARTLink.print('H');
  for (int i = 0; i < NUM_SENSORS; i++) {
    UARTLink.print(',');
    UARTLink.print(sensorMax[i] - sensorMin[i]);
  }
  UARTLink.println();

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

  if (botRunning && !wasBotRunning) {
    runStartTime = millis(); // just started - begin the startup ease-in ramp
  }
  wasBotRunning = botRunning;

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

  bool lineFound = totalSum > 300;

  // Track how long the line has been continuously lost, so the recovery
  // behavior below can escalate over time (see the recovery block near the
  // end of this function) instead of arcing forward forever.
  if (!lineFound) {
    if (!wasLineLost) {
      lineLostSince = millis();
      wasLineLost = true;
    }
  } else {
    wasLineLost = false;
  }
  unsigned long lostDuration = wasLineLost ? (millis() - lineLostSince) : 0;

  // Calculate position error
  if (lineFound) {
    float position = (float)weightedSum / totalSum; // Position range: 0 to 7000 (Center = 3500)
    error = position - 3500.0;
  } else {
    // Line lost: maintain hard spin in direction of last known error
    if (lastError < 0) error = -3500.0;
    else error = 3500.0;
  }

  // --- Error Deadband ---
  // Ignores tiny errors caused by sensor/ADC noise right around line-center.
  // Without this, the P-only controller (Kd=0) keeps making small opposing
  // corrections in response to noise that isn't a real deviation, which
  // shows up as a persistent low-amplitude jitter. Only affects the PID
  // correction itself - errorRatio/lastError/pivot logic below still use
  // the real, undamped `error` so turn detection isn't blunted.
  const float ERROR_DEADBAND = 50.0;
  float controlError = (fabs(error) < ERROR_DEADBAND) ? 0.0 : error;

  // Calculate PID Output
  float pTerm = Kp * controlError;
  float dTerm = Kd * (controlError - lastError);
  float pidOutput = pTerm + dTerm;

  if (totalSum > 300) {
    lastError = error;
  }

  // --- Speed Scheduling: slow down proportionally to how sharp the turn is ---
  float errorRatio = fabs(error) / 3500.0; // 0 = straight, 1 = max turn/line-lost
  errorRatio = constrain(errorRatio, 0.0, 1.0);
  const float TURN_SLOWDOWN = 0.65; // up to 65% speed cut on the sharpest turns
  float effectiveSpeed = masterSpeed * (1.0 - TURN_SLOWDOWN * errorRatio);

  // --- Startup Ease-In Ramp ---
  // Ramps speed up from 40% to 100% over the first 500ms after Start is
  // pressed, instead of committing to full masterSpeed instantly - gives the
  // PID loop a moment to settle on real sensor readings before the bot is
  // moving at full commanded speed, reducing the chance of lurching off the
  // line right at the start.
  const unsigned long STARTUP_RAMP_MS = 500;
  float rampFactor = constrain((float)(millis() - runStartTime) / STARTUP_RAMP_MS, 0.0f, 1.0f);
  effectiveSpeed *= (0.4f + 0.6f * rampFactor);

  // Calculate Base Speeds with Right-Motor Adjustment Factor
  float baseSpeedA = effectiveSpeed * FACTOR_A;
  float baseSpeedB = effectiveSpeed * FACTOR_B;

  // Apply Differential Steering Output
  float motorSpeedA = baseSpeedA + pidOutput;
  float motorSpeedB = baseSpeedB - pidOutput;

  // --- Pivot Turning: geometry-derived ARC turn (not a stationary point-turn
  // and not a guessed differential). The sensor sits ~17cm ahead of the axle,
  // so the physical corner is still about that far ahead of the axle at the
  // moment a sharp/lost-line reading triggers this.
  //
  // For a differential-drive robot turning on a circular arc of radius R
  // (measured from the instantaneous center of rotation to the chassis
  // centerline), after rotating through heading angle phi the forward
  // displacement along the ORIGINAL heading is R*sin(phi). For a 90-degree
  // turn (phi = 90 deg), sin(90 deg) = 1, so that displacement is simply R.
  // We want that forward displacement to equal the ~17cm sensor lead
  // distance so the axle actually reaches the corner by the time the
  // chassis has rotated 90 degrees - so target R = 0.17m.
  //
  // Wheel speeds for a track width L (here L = 0.17m, motors at the very
  // ends of the chassis) relate to R via:
  //   v_outer = w*(R + L/2)   v_inner = w*(R - L/2)
  // With R = 0.17m, L/2 = 0.085m:
  //   v_outer ~ (0.17+0.085) = 0.255   v_inner ~ (0.17-0.085) = 0.085
  //   ratio v_outer:v_inner = 3:1, BOTH POSITIVE (inner keeps driving
  //   forward at 1/3 speed - no reversal). A prior guessed 150/-40 split
  //   works out (via the same formula) to R ~= 4.9cm - far short of the
  //   17cm needed, so it was still likely under-shooting the corner.
  const float PIVOT_THRESHOLD = 0.80;   // errorRatio above this triggers the arc turn
  const int PIVOT_OUTER_SPEED = 160;    // outer wheel
  const int PIVOT_INNER_SPEED = 60;     // inner wheel - both forward
  if (errorRatio > PIVOT_THRESHOLD) {
    if (error > 0) {
      motorSpeedA = PIVOT_OUTER_SPEED;  // outer (left) wheel
      motorSpeedB = PIVOT_INNER_SPEED;  // inner (right) wheel - still forward, just slower
    } else {
      motorSpeedA = PIVOT_INNER_SPEED;
      motorSpeedB = PIVOT_OUTER_SPEED;
    }
  }

  // --- Line-Loss Recovery: escalates the longer the line stays lost ---
  //   0-500ms    : the arc-turn above already handles this (most real 90-degree
  //                corners reacquire the line well within this window)
  //   500-900ms  : still nothing - reverse back along the SAME arc, retracing
  //                the path toward where the line was last seen, since
  //                continuing to arc forward clearly isn't finding it
  //   900ms+     : give up and stop, rather than blindly arc/reverse forever
  //                and drift further off-track
  const unsigned long REVERSE_AFTER_MS = 500;
  const unsigned long STOP_AFTER_MS = 900;
  if (wasLineLost && lostDuration >= REVERSE_AFTER_MS) {
    if (lostDuration < STOP_AFTER_MS) {
      // Reverse: same outer/inner split as the arc-turn, both negated, so
      // the chassis retraces the same curve backward instead of forward.
      if (error > 0) {
        motorSpeedA = -PIVOT_OUTER_SPEED;
        motorSpeedB = -PIVOT_INNER_SPEED;
      } else {
        motorSpeedA = -PIVOT_INNER_SPEED;
        motorSpeedB = -PIVOT_OUTER_SPEED;
      }
    } else {
      motorSpeedA = 0;
      motorSpeedB = 0;
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

  // --- Minimum PWM Floor ---
  // Below this duty cycle the N20 motors don't spin cleanly - they cog/
  // stutter in small mechanical jerks instead of turning smoothly (not
  // enough torque to overcome static friction continuously). A wheel that's
  // meant to be fully stopped (speed 0, e.g. from stopMotors()) is left
  // alone; only a wheel commanded to some low-but-nonzero speed gets bumped
  // up into the range where it actually moves smoothly.
  const int MIN_PWM = 40;
  if (speedA > 0 && speedA < MIN_PWM) speedA = MIN_PWM;
  if (speedB > 0 && speedB < MIN_PWM) speedB = MIN_PWM;

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
