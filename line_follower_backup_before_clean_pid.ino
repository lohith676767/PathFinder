// Main line-follower ESP32. QTR-8 style array (8 analog IR sensors, D1..D8
// left-to-right), TB6612FNG dual motor driver, UART link to a second ESP32
// (OLED + 2 buttons) for live tuning and status.
//
// ============================== ARCHITECTURE ==============================
// Sensing: the standard weighted-centroid position estimate used by every
// QTR-array line follower (Pololu's own reference code included) - proven,
// simple, kept as-is.
//
// Control: NOT the classic "PID output added as a flat PWM offset" that
// most hobbyist line followers use. That approach has a real flaw for a
// robot meant to run at variable speed: a flat correction becomes
// proportionally WEAKER relative to forward speed the faster you go, so
// steering authority silently fades exactly when more of it is needed.
// Instead this treats the correction as a CURVATURE DEMAND and mixes it
// with current speed the way real differential-drive robots do:
//   v_left  = v * (1 - k*L/2)
//   v_right = v * (1 + k*L/2)
// (k = curvature, L = track width) - so steering strength scales with
// speed automatically, borrowed straight from mobile-robot kinematics.
//
// Regimes: normal tracking, a committed sharp turn, and line-lost recovery
// are genuinely different behaviors, not just different gains - so they're
// modeled as an explicit finite state machine (a standard, well-known
// robotics software pattern) with debounced transitions, instead of a pile
// of blended thresholds. See RobotState / updateState() / runLineFollower().
//
// Chassis assumptions (17cm x 17cm, motors at the rear, sensor at the very
// front -> ~17cm sensor lead) still drive the sharp-turn arc math exactly
// as derived before - see computeSharpTurnOutput().
// ============================================================================

// ---------------------------------------------------------------
// Pins
// ---------------------------------------------------------------
const int AIN1 = 19, AIN2 = 21, PWMA = 18;
const int BIN1 = 16, BIN2 = 22, PWMB = 17;
const int STBY = 5;
const int LED_PIN = 2;

const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {25, 36, 39, 34, 35, 32, 33, 26}; // D1..D8, outer two on ADC2

HardwareSerial UARTLink(2); // RX=GPIO4, TX=GPIO27 - link to the OLED ESP

// ---------------------------------------------------------------
// Live-tunable parameters (defaults; updated over UART - see processUARTCommand)
// Same numeric meaning/range as before: Kp/Kd are tuned assuming
// REFERENCE_SPEED as the operating point, and now scale correctly at other
// speeds instead of losing relative strength at high speed.
// ---------------------------------------------------------------
int masterSpeed = 100;
float Kp = 0.18;
float Kd = 0.0;

// ---------------------------------------------------------------
// Fixed tuning constants
// ---------------------------------------------------------------
const float REFERENCE_SPEED = 100.0;  // speed Kp/Kd were historically tuned around
const float FACTOR_A = 1.00;          // left motor speed scale
const float FACTOR_B = 0.92;          // right motor speed scale (mechanical trim)
const float ERROR_DEADBAND = 50.0;    // ignore error smaller than this (sensor noise)
const float TURN_SLOWDOWN = 0.35;     // max fractional speed cut on the sharpest turns
const unsigned long STARTUP_RAMP_MS = 500;   // ease-in duration after Start is pressed
const int PIVOT_OUTER_SPEED = 160;    // outer wheel during a committed sharp turn
const int PIVOT_INNER_SPEED = 60;     // inner wheel during a committed sharp turn (~3:1 ratio)
const unsigned long REVERSE_AFTER_MS = 500;  // line lost this long -> start reversing
const unsigned long STOP_AFTER_MS = 900;     // line lost this long -> give up, stop
const int MIN_PWM = 40;               // below this the motors cog instead of spinning smoothly
const int LINE_PRESENT_THRESHOLD = 300; // totalSum above this = line genuinely detected
const int SENSOR_NOISE_FLOOR = 200;   // per-sensor readings below this treated as 0 (white)
const float SENSOR_EMA_ALPHA = 0.5;   // sensor smoothing: higher = more responsive, lower = smoother
const float PREDICTION_HORIZON_S = 0.15; // how far ahead (seconds) to extrapolate the error trend

// State machine thresholds (saturation = |error|/3500, 0=centered, 1=at the
// sensor's edge or line lost). Entry/exit each require the condition to
// hold for a short confirm window, not just one frame - this is the fix for
// the old bug where a single noisy reading during an ordinary curve could
// wrongly trigger the sharp-turn behavior.
const float SHARP_TURN_ENTER = 0.85;
const float SHARP_TURN_EXIT = 0.35;
const unsigned long STATE_CONFIRM_MS = 40;
const unsigned long SHARP_TURN_MIN_COMMIT_MS = 150; // stay committed at least this long once entered

// ---------------------------------------------------------------
// State
// ---------------------------------------------------------------
int sensorMin[NUM_SENSORS], sensorMax[NUM_SENSORS], sensorValues[NUM_SENSORS];
int normValues[NUM_SENSORS]; // latest normalized readings, 0 (white) .. 1000 (black) - raw, for display/telemetry
float filteredNorm[NUM_SENSORS]; // EMA-smoothed version of normValues, used for steering math

bool isCalibrated = false;
bool botRunning = false;
bool wasBotRunning = false;
unsigned long runStartTime = 0;    // for the startup ramp

float lastError = 0;
unsigned long lastVelocityTime = 0; // for computeErrorVelocity's dt

enum RobotState { STATE_TRACKING, STATE_SHARP_TURN, STATE_LINE_LOST };
RobotState robotState = STATE_TRACKING;
unsigned long stateEnteredAt = 0;
unsigned long conditionMetSince = 0; // for debouncing state transitions
bool conditionWasMet = false;
int committedDirection = 1;   // latched turn direction (+1 right, -1 left) for SHARP_TURN/LINE_LOST
unsigned long lineLostSince = 0;

char uartBuffer[64];
int uartLen = 0;
unsigned long lastStatusSend = 0;
unsigned long lastSensorSend = 0;

// ---------------------------------------------------------------
// UART command handling (from the OLED ESP)
//   in:  "C,<masterSpeed>,<Kp>,<Kd>,<run0or1>\n"
//   out: "S,<isCalibrated0or1>,<botRunning0or1>\n"   (every ~300ms)
//        "D,<v0>,...,<v7>\n"                          (every ~150ms, live sensor values)
//        "H,<spread0>,...,<spread7>\n"                (once, right after calibration)
// ---------------------------------------------------------------
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

// ---------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------
void readSensorsNormalized() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normVal = constrain(normVal, 0, 1000);
    if (normVal < SENSOR_NOISE_FLOOR) normVal = 0;

    normValues[i] = normVal; // raw - used for the OLED display/telemetry
    // EMA smoothing - used for the actual steering math (see computeLineError)
    filteredNorm[i] = SENSOR_EMA_ALPHA * normVal + (1.0f - SENSOR_EMA_ALPHA) * filteredNorm[i];
  }
}

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

// ---------------------------------------------------------------
// Setup: pins, then the countdown -> calibration -> ready sequence
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  UARTLink.begin(115200, SERIAL_8N1, 4, 27);
  analogSetAttenuation(ADC_11db);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotors();

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  // Countdown (5s, slow blink) - time to place the bot on the line.
  Serial.println("Get ready! Calibration starts in 5 seconds...");
  for (int i = 5; i > 0; i--) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(800);
  }

  // Calibration (8s, fast blink) - sweep the sensor bar across the line.
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

  // One-shot health report: how much each sensor actually swung between
  // white and black. A sensor that barely moved (loose wire, dead sensor,
  // never got swept over the line) is caught here, not mid-run.
  UARTLink.print('H');
  for (int i = 0; i < NUM_SENSORS; i++) {
    UARTLink.print(',');
    UARTLink.print(sensorMax[i] - sensorMin[i]);
  }
  UARTLink.println();

  Serial.println("Calibration Complete! Waiting for START command over UART...");
}

void loop() {
  handleUART();

  if (isCalibrated) {
    readSensorsNormalized();
    sendSensorData();
  }

  if (botRunning && !wasBotRunning) {
    runStartTime = millis(); // just started - begin the startup ease-in ramp
    robotState = STATE_TRACKING;
    conditionMetSince = 0;
    conditionWasMet = false;
  }
  wasBotRunning = botRunning;

  if (botRunning && isCalibrated) {
    runLineFollower();
  } else {
    stopMotors();
  }
}

// ---------------------------------------------------------------
// Sensing - weighted-centroid position (the standard QTR-array technique).
// ---------------------------------------------------------------

// Signed error, -3500..+3500, center = 0. If the line isn't seen at all,
// holds the last known direction (forces the error to the extreme in
// whichever direction the line last was). Uses the EMA-smoothed sensor
// values, not the raw ones, so a single noisy ADC sample doesn't produce a
// spurious error spike.
float computeLineError(bool &lineFound) {
  float weightedSum = 0, totalSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    totalSum += filteredNorm[i];
    weightedSum += filteredNorm[i] * (i * 1000);
  }

  lineFound = totalSum > LINE_PRESENT_THRESHOLD;

  if (lineFound) {
    float position = weightedSum / totalSum; // 0..7000, center 3500
    return position - 3500.0;
  }
  return (lastError < 0) ? -3500.0 : 3500.0;
}

// How fast the error is changing, in units/second (not just a raw per-loop
// difference) - "what it last saw vs what it sees now," normalized so it
// means the same thing regardless of loop timing or how fast the bot is
// physically moving. Only computed while the line is genuinely tracked.
float computeErrorVelocity(float error, bool lineFound) {
  if (!lineFound) return 0.0;

  unsigned long now = millis();
  float dt = (lastVelocityTime == 0) ? 0.02f : (now - lastVelocityTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  if (dt > 0.3f) dt = 0.3f; // clamp a long gap (e.g. just reacquired after a loss)
  lastVelocityTime = now;

  float velocity = (error - lastError) / dt;
  lastError = error;
  return velocity;
}

// 0 (straight) .. 1 (max turn / line lost), with the deadband subtracted
// first so ordinary straight-line noise costs zero speed, not a small
// continuous cut.
float computeSaturation(float error) {
  float magnitude = fabs(error) - ERROR_DEADBAND;
  if (magnitude < 0.0) magnitude = 0.0;
  return constrain(magnitude / 3500.0, 0.0, 1.0);
}

// Extrapolates PREDICTION_HORIZON_S seconds into the future using the
// current trend, and reports whichever is more urgent - now, or where it's
// headed. At low speed the two are nearly identical (nothing changes much
// in 150ms). At high speed, a fast-changing error means a corner is
// approaching quickly, so downstream logic (speed cut, sharp-turn entry)
// reacts earlier - the same real-time margin regardless of how fast the
// bot is actually moving, instead of a margin that was only correct at
// whatever speed the thresholds happened to be tuned at.
float computeAnticipatedSaturation(float error, float errorVelocity) {
  float predictedError = error + errorVelocity * PREDICTION_HORIZON_S;
  return max(computeSaturation(error), computeSaturation(predictedError));
}

// ---------------------------------------------------------------
// State machine - three genuinely different regimes, debounced transitions.
// ---------------------------------------------------------------

// Tracks how long `condition` has been continuously true; 0 while false.
unsigned long timeConditionHeld(bool condition) {
  if (!condition) {
    conditionWasMet = false;
    return 0;
  }
  if (!conditionWasMet) {
    conditionMetSince = millis();
    conditionWasMet = true;
  }
  return millis() - conditionMetSince;
}

void enterState(RobotState next) {
  robotState = next;
  stateEnteredAt = millis();
  conditionWasMet = false; // reset the debounce tracker for the new state's own condition
}

void updateState(bool lineFound, float error, float anticipatedSaturation) {
  if (!lineFound) {
    if (robotState != STATE_LINE_LOST) {
      committedDirection = (lastError >= 0) ? 1 : -1; // latch direction once, at the moment of loss
      lineLostSince = millis();
      enterState(STATE_LINE_LOST);
    }
    return;
  }

  switch (robotState) {
    case STATE_LINE_LOST:
    case STATE_TRACKING:
      // (LINE_LOST always exits back to TRACKING the instant the line is
      // seen again - no debounce needed here, re-detecting a real signal
      // is trustworthy immediately, unlike the noisy "about to lose it"
      // direction upstream.)
      if (robotState == STATE_LINE_LOST) {
        enterState(STATE_TRACKING);
        return;
      }
      if (timeConditionHeld(anticipatedSaturation > SHARP_TURN_ENTER) >= STATE_CONFIRM_MS) {
        committedDirection = (error >= 0) ? 1 : -1; // latch once, at entry
        enterState(STATE_SHARP_TURN);
      }
      break;

    case STATE_SHARP_TURN: {
      bool committedLongEnough = (millis() - stateEnteredAt) >= SHARP_TURN_MIN_COMMIT_MS;
      if (committedLongEnough &&
          timeConditionHeld(anticipatedSaturation < SHARP_TURN_EXIT) >= STATE_CONFIRM_MS) {
        enterState(STATE_TRACKING);
      }
      break;
    }
  }
}

// ---------------------------------------------------------------
// Per-state motor output
// ---------------------------------------------------------------

// Normal tracking: curvature-based steering, kinematically mixed with
// current speed (see the file header) instead of added as a flat offset -
// this is the fix for steering weakening at high speed. Speed itself eases
// off with anticipated saturation and ramps in after a fresh start.
void computeTrackingOutput(float error, float errorVelocity, float anticipatedSaturation,
                            float &motorSpeedA, float &motorSpeedB) {
  float controlError = (fabs(error) < ERROR_DEADBAND) ? 0.0 : error;
  float curvatureDemand = Kp * controlError + Kd * errorVelocity;

  float rampFactor = constrain((float)(millis() - runStartTime) / STARTUP_RAMP_MS, 0.0f, 1.0f);
  float speed = masterSpeed * (1.0f - TURN_SLOWDOWN * anticipatedSaturation) * (0.4f + 0.6f * rampFactor);

  // Kinematic mixing: the correction scales with current speed, so it
  // keeps proportionally the same steering authority whether crawling or
  // going flat out - normalized against REFERENCE_SPEED so existing Kp/Kd
  // values keep feeling the same as before at the speed they were tuned at.
  float turnComponent = curvatureDemand * (speed / REFERENCE_SPEED);

  motorSpeedA = (speed * FACTOR_A) + turnComponent;
  motorSpeedB = (speed * FACTOR_B) - turnComponent;
}

// Committed sharp turn: geometry-derived arc turn (not a stationary point-
// turn). The sensor sits ~17cm ahead of the axle, so the physical corner
// is still about that far ahead of the axle at the moment this triggers.
// For a differential-drive turn of radius R, forward displacement after
// rotating 90 degrees is exactly R, so setting R = 0.17m (the sensor lead)
// makes the chassis actually reach the corner as it completes the turn.
// With track width L = 0.17m, wheel speeds v_outer=(R+L/2), v_inner=(R-L/2)
// work out to a ~3:1 ratio, both positive (PIVOT_OUTER_SPEED / _INNER_SPEED).
void computeSharpTurnOutput(float &motorSpeedA, float &motorSpeedB) {
  motorSpeedA = (committedDirection > 0) ? PIVOT_OUTER_SPEED : PIVOT_INNER_SPEED;
  motorSpeedB = (committedDirection > 0) ? PIVOT_INNER_SPEED : PIVOT_OUTER_SPEED;
}

// Line lost: escalates the longer it stays lost. 0-500ms keeps arcing in
// the committed direction (usually enough for a real corner to reacquire);
// 500-900ms reverses along the same arc, retracing back toward where the
// line was last seen; 900ms+ gives up and stops, rather than drifting
// further off-track on a blind guess.
void computeLineLostOutput(float &motorSpeedA, float &motorSpeedB) {
  unsigned long lostDuration = millis() - lineLostSince;

  if (lostDuration < REVERSE_AFTER_MS) {
    motorSpeedA = (committedDirection > 0) ? PIVOT_OUTER_SPEED : PIVOT_INNER_SPEED;
    motorSpeedB = (committedDirection > 0) ? PIVOT_INNER_SPEED : PIVOT_OUTER_SPEED;
  } else if (lostDuration < STOP_AFTER_MS) {
    motorSpeedA = (committedDirection > 0) ? -PIVOT_OUTER_SPEED : -PIVOT_INNER_SPEED;
    motorSpeedB = (committedDirection > 0) ? -PIVOT_INNER_SPEED : -PIVOT_OUTER_SPEED;
  } else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
}

void runLineFollower() {
  bool lineFound;
  float error = computeLineError(lineFound);
  float errorVelocity = computeErrorVelocity(error, lineFound); // also latches lastError
  float anticipatedSaturation = computeAnticipatedSaturation(error, errorVelocity);

  updateState(lineFound, error, anticipatedSaturation);

  float motorSpeedA, motorSpeedB;
  switch (robotState) {
    case STATE_TRACKING:
      computeTrackingOutput(error, errorVelocity, anticipatedSaturation, motorSpeedA, motorSpeedB);
      break;
    case STATE_SHARP_TURN:
      computeSharpTurnOutput(motorSpeedA, motorSpeedB);
      break;
    case STATE_LINE_LOST:
      computeLineLostOutput(motorSpeedA, motorSpeedB);
      break;
  }

  int dirA = (motorSpeedA >= 0) ? 1 : -1;
  int dirB = (motorSpeedB >= 0) ? 1 : -1;
  setMotors(dirA, abs((int)motorSpeedA), dirB, abs((int)motorSpeedB));
}

// ---------------------------------------------------------------
// Motor output
// ---------------------------------------------------------------
void setMotors(int dirA, int speedA, int dirB, int speedB) {
  speedA = constrain(speedA, 0, 255);
  speedB = constrain(speedB, 0, 255);

  // Below MIN_PWM the motors cog/stutter instead of spinning smoothly (not
  // enough torque to overcome static friction continuously). A wheel meant
  // to be fully stopped (speed 0) is left alone.
  if (speedA > 0 && speedA < MIN_PWM) speedA = MIN_PWM;
  if (speedB > 0 && speedB < MIN_PWM) speedB = MIN_PWM;

  if (dirA == 1) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else if (dirA == -1) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); }
  analogWrite(PWMA, speedA);

  if (dirB == 1) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else if (dirB == -1) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); }
  analogWrite(PWMB, speedB);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}
