
// ============================================================
// MAIN LINE FOLLOWER ESP32
// ============================================================
//
// Hardware:
//   ESP32
//   TB6612FNG motor driver
//   QTR-8A / 8-channel analog line sensor
//   OLED ESP32 connected through UART
//
// UART:
//   Main ESP32 RX = GPIO4
//   Main ESP32 TX = GPIO27
//
// OLED protocol:
//
//   Commands FROM OLED:
//      CAL
//      START
//      STOP
//      C,speed,kp,kd,run
//
//   Data TO OLED:
//      S,calibrated,running
//      D,s1,s2,s3,s4,s5,s6,s7,s8
//      H,h1,h2,h3,h4,h5,h6,h7,h8
//
// IMPORTANT:
//   Calibration is NON-BLOCKING.
//   UART continues working during calibration.
//
// ============================================================


// ============================================================
// MOTOR DRIVER PINS - TB6612FNG
// ============================================================

const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 22;
const int PWMB = 17;

const int STBY = 5;


// Built-in LED
const int LED_PIN = 2;


// ============================================================
// LINE SENSOR PINS
// ============================================================

const int NUM_SENSORS = 8;

const int sensorPins[NUM_SENSORS] = {
  25,
  36,
  39,
  34,
  35,
  32,
  33,
  26
};


// ============================================================
// UART
// ============================================================
//
// Main ESP32:
//     RX = GPIO4
//     TX = GPIO27
//
// OLED ESP32:
//     connected to the opposite RX/TX.
//
// ============================================================

HardwareSerial UARTLink(2);

const int UART_RX = 4;
const int UART_TX = 27;


// ============================================================
// CONTROL PARAMETERS
// ============================================================

int masterSpeed = 100;

float Kp = 0.18;
float Kd = 0.0;


// ============================================================
// SENSOR DATA
// ============================================================

int sensorRaw[NUM_SENSORS] = {0};

// Normalized:
//     0    = white
//     1000 = black

int sensorValue[NUM_SENSORS] = {0};


// Calibration limits
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];


// ============================================================
// CALIBRATION STATE
// ============================================================

bool isCalibrated = false;

bool calibrationActive = false;

unsigned long calibrationStartTime = 0;

unsigned long lastCalibrationBlink = 0;

bool calibrationLEDState = false;


// Calibration duration
const unsigned long CALIBRATION_TIME = 8000;

// Minimum required sensor range
const int MIN_SENSOR_SPREAD = 300;


// ============================================================
// SENSOR WEIGHTS
// ============================================================
//
// D1 ---------------- D8
//
// -3500 -2500 -1500 -500
// +500  +1500 +2500 +3500
//
// ============================================================

const int sensorWeight[NUM_SENSORS] = {
  -3500,
  -2500,
  -1500,
  -500,
   500,
   1500,
   2500,
   3500
};


// ============================================================
// LINE FOLLOWING VARIABLES
// ============================================================

float position = 0;

float error = 0;

float previousError = 0;

float derivative = 0;

float correction = 0;


// ============================================================
// LINE DETECTION
// ============================================================

const int LINE_THRESHOLD = 200;

bool lineDetected = false;


// ============================================================
// LAST KNOWN LINE DIRECTION
// ============================================================
//
// -1 = left
// +1 = right
//  0 = unknown
//
// ============================================================

int lastKnownDirection = 0;


// ============================================================
// SHARP TURN / CORNER DETECTION
// ============================================================
//
// Two independent signals feed into "this loss was a sharp/right-angle
// turn, not a dashed-line gap" (lostWasSharpTurn in updateLineLostState,
// below) - either one firing is enough:
//
// (1) CONE SIGNATURE - the WIDTH of the black patch shrinks over
//     consecutive samples as the sensor bar approaches the corner:
//
//       WWBBBBWW  ->  WWBBBWWW  ->  WWWBBWWW  ->  WWWWWWWW
//         (4 lit)        (3 lit)      (2 lit)       (lost)
//
//     sampleSharpTurnHistory()/wasNarrowingBeforeLoss() below detect this
//     from a rolling window of lit-sensor counts.
//
// (2) SUDDEN IMBALANCE - the robot was tracking normally (roughly
//     balanced), the reading suddenly skews hard to one side (even just
//     briefly/slightly - at higher speed the in-between balanced-ish
//     reading can be skipped almost entirely between control loops), and
//     the very next moment the line disappears completely. This is
//     exactly what lastKnownDirection (below) already captures: it's
//     sticky, only updated when the reading skews past
//     IMBALANCE_THRESHOLD, holding its last value otherwise. So at the
//     instant the line is lost, lastKnownDirection != 0 means it was
//     skewed one way right before going white.
//
// (1) needs several consecutive narrowing samples and can miss very fast
// corners; (2) can fire off a single skewed reading (or even a stale one
// from just before) but says nothing about *how* the loss happened, only
// that it was lopsided. Between them they cover both the "gradual corner"
// and the "blink and it's a right angle" cases.
//
// ============================================================

const float IMBALANCE_THRESHOLD = 80.0;

const unsigned long SHARP_SAMPLE_INTERVAL = 20;

unsigned long lastSharpSampleTime = 0;

int activeCountHistory[4] = { 8, 8, 8, 8 };


// ============================================================
// LINE LOST RECOVERY
// ============================================================
//
// Losing the line briefly (a dashed/gapped line) should NOT trigger a
// pivot search - the bot should just coast straight through the gap.
// Losing it for longer means it actually left the track (or the pivot
// didn't fully complete the turn), so escalate:
//
//   0 - DOTTED_LINE_GRACE_MS........ coast straight (bridge the gap)
//   .. - LINE_LOST_RECOVERY_MS...... pivot toward lastKnownDirection
//   .. - +RECOVERY_REVERSE_MS....... back up toward where line was last seen
//   .. - LINE_LOST_ABORT_MS......... wider/harder pivot sweep
//   beyond LINE_LOST_ABORT_MS....... give up, stop (avoid wandering off
//                                     forever with no line in sight)
//
// A confirmed sharp turn (lostWasSharpTurn) skips the bridge phase
// entirely and pivots immediately, since it's clearly not a dash gap.
//
// ============================================================

const unsigned long DOTTED_LINE_GRACE_MS = 300;

const unsigned long LINE_LOST_RECOVERY_MS = 2500;

const unsigned long RECOVERY_REVERSE_MS = 400;

const unsigned long LINE_LOST_ABORT_MS = 6000;


unsigned long lineLostStartTime = 0;

bool lineWasDetectedLastLoop = true;

bool lostWasSharpTurn = false;


enum RecoveryPhase {
  RECOVERY_NONE,
  RECOVERY_BRIDGE,
  RECOVERY_SEARCH,
  RECOVERY_REVERSE,
  RECOVERY_WIDE_SEARCH,
  RECOVERY_ABORTED
};

RecoveryPhase recoveryPhase = RECOVERY_NONE;


// ============================================================
// RUN STATE
// ============================================================

bool botRunning = false;


// ============================================================
// UART BUFFER
// ============================================================

char uartBuffer[96];

int uartLen = 0;


// ============================================================
// TIMING
// ============================================================

// Line-control interval
unsigned long lastControlTime = 0;

const unsigned long CONTROL_INTERVAL = 2;


// OLED update interval
unsigned long lastStatusTime = 0;

const unsigned long STATUS_INTERVAL = 100;


// ============================================================
// SENSOR DATA UPDATE
// ============================================================

unsigned long lastSensorSend = 0;

const unsigned long SENSOR_SEND_INTERVAL = 100;


// ============================================================
// MOTOR PWM
// ============================================================
//
// Uses the current ESP32 Arduino API:
//
//     ledcAttach(pin, frequency, resolution)
//     ledcWrite(pin, duty)
//
// No ledcSetup() is used.
//
// ============================================================

const int PWM_FREQ = 20000;

const int PWM_RESOLUTION = 8;


// ============================================================
// MOTOR DIRECTION
// ============================================================
//
// These values describe the ELECTRICAL orientation of
// the motors on your current wiring.
//
// If the robot drives backwards when commanded forward,
// change the corresponding value.
//
// ============================================================

const bool MOTOR_A_INVERT = true;

const bool MOTOR_B_INVERT = true;


// ============================================================
// MINIMUM MOTOR PWM
// ============================================================
//
// Small N20 motors may not start properly at very low PWM.
//
// A commanded non-zero speed below this value is raised
// to this value.
//
// ============================================================

const int MIN_PWM = 40;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void stopMotors();

void setMotorA(int speed);

void setMotorB(int speed);

void setMotors(int leftSpeed, int rightSpeed);

void readSensors();

void normalizeSensors();

void calculatePosition();

int countActiveSensors();

void sampleSharpTurnHistory();

bool wasNarrowingBeforeLoss();

void updateLineLostState();

void lineFollow();

void startCalibration();

void updateCalibration();

void finishCalibration();

void processCommand(char *line);

void readUART();

void sendStatus();

void sendSensorData();

void sendHealthData();


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(300);


  Serial.println();
  Serial.println("==========================================");
  Serial.println("       MAIN LINE FOLLOWER ESP32");
  Serial.println("==========================================");


  // ----------------------------------------------------------
  // UART
  // ----------------------------------------------------------

  UARTLink.begin(
    115200,
    SERIAL_8N1,
    UART_RX,
    UART_TX
  );

  Serial.println("UART initialized.");

  Serial.print("UART RX = GPIO");
  Serial.println(UART_RX);

  Serial.print("UART TX = GPIO");
  Serial.println(UART_TX);


  // ----------------------------------------------------------
  // MOTOR PINS
  // ----------------------------------------------------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  pinMode(LED_PIN, OUTPUT);


  // Enable TB6612FNG
  digitalWrite(STBY, HIGH);


  // ----------------------------------------------------------
  // PWM
  // ----------------------------------------------------------

  bool pwmAOK = ledcAttach(
    PWMA,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  bool pwmBOK = ledcAttach(
    PWMB,
    PWM_FREQ,
    PWM_RESOLUTION
  );


  Serial.print("PWM A attach: ");
  Serial.println(pwmAOK ? "OK" : "FAILED");

  Serial.print("PWM B attach: ");
  Serial.println(pwmBOK ? "OK" : "FAILED");


  // ----------------------------------------------------------
  // INITIAL MOTOR STOP
  // ----------------------------------------------------------

  stopMotors();


  // ----------------------------------------------------------
  // SENSOR INITIALIZATION
  // ----------------------------------------------------------

  for (int i = 0; i < NUM_SENSORS; i++) {

    pinMode(
      sensorPins[i],
      INPUT
    );

    sensorMin[i] = 4095;

    sensorMax[i] = 0;
  }


  // ----------------------------------------------------------
  // INITIAL STATE
  // ----------------------------------------------------------

  isCalibrated = false;

  botRunning = false;

  calibrationActive = false;


  digitalWrite(
    LED_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // START CALIBRATION
  // ----------------------------------------------------------

  Serial.println();
  Serial.println("------------------------------------------");
  Serial.println("Starting calibration.");
  Serial.println("Move the sensor array over WHITE + BLACK.");
  Serial.println("------------------------------------------");


  startCalibration();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {


  // ==========================================================
  // ALWAYS HANDLE UART
  // ==========================================================
  //
  // This is extremely important.
  //
  // UART is processed even while calibration is running.
  //
  // Therefore an OLED button command cannot get blocked
  // behind a delay() or while-loop.
  //
  // ==========================================================

  readUART();


  // ==========================================================
  // CALIBRATION STATE
  // ==========================================================

  if (calibrationActive) {

    updateCalibration();

    // Motors MUST remain stopped
    stopMotors();

  }


  // ==========================================================
  // NORMAL OPERATION
  // ==========================================================

  else {

    // --------------------------------------------------------
    // Fast line-following control loop
    // --------------------------------------------------------

    if (
      millis() - lastControlTime
      >= CONTROL_INTERVAL
    ) {

      lastControlTime = millis();


      if (
        botRunning &&
        isCalibrated
      ) {

        readSensors();

        normalizeSensors();

        sampleSharpTurnHistory();

        calculatePosition();

        updateLineLostState();

        lineFollow();

      }

      else {

        // Robot not running
        stopMotors();


        // Still update sensor readings so OLED
        // can display live sensor values.

        if (isCalibrated) {

          readSensors();

          normalizeSensors();
        }
      }
    }
  }


  // ==========================================================
  // PERIODIC OLED DATA
  // ==========================================================

  if (
    millis() - lastStatusTime
    >= STATUS_INTERVAL
  ) {

    lastStatusTime = millis();


    sendSensorData();

    sendStatus();
  }
}


// ============================================================
// START CALIBRATION
// ============================================================

void startCalibration() {

  // ----------------------------------------------------------
  // Stop robot
  // ----------------------------------------------------------

  botRunning = false;

  stopMotors();


  // ----------------------------------------------------------
  // Calibration state
  // ----------------------------------------------------------

  calibrationActive = true;

  isCalibrated = false;


  calibrationStartTime = millis();

  lastCalibrationBlink = millis();

  calibrationLEDState = false;


  digitalWrite(
    LED_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // Reset calibration data
  // ----------------------------------------------------------

  for (int i = 0; i < NUM_SENSORS; i++) {

    sensorMin[i] = 4095;

    sensorMax[i] = 0;
  }


  // ----------------------------------------------------------
  // Reset line-following state
  // ----------------------------------------------------------

  position = 0;

  error = 0;

  previousError = 0;

  derivative = 0;

  correction = 0;

  lineDetected = false;

  lastKnownDirection = 0;

  lineLostStartTime = 0;

  lineWasDetectedLastLoop = true;

  lostWasSharpTurn = false;

  recoveryPhase = RECOVERY_NONE;

  activeCountHistory[0] = 8;
  activeCountHistory[1] = 8;
  activeCountHistory[2] = 8;
  activeCountHistory[3] = 8;


  Serial.println("CALIBRATION ACTIVE");


  // Immediately tell OLED
  sendStatus();
}


// ============================================================
// UPDATE CALIBRATION
// ============================================================
//
// This function is intentionally NON-BLOCKING.
//
// It performs a small amount of work and immediately returns.
//
// ============================================================

void updateCalibration() {

  // ----------------------------------------------------------
  // Read every sensor
  // ----------------------------------------------------------

  for (int i = 0; i < NUM_SENSORS; i++) {

    int value =
      analogRead(
        sensorPins[i]
      );


    if (
      value <
      sensorMin[i]
    ) {

      sensorMin[i] = value;
    }


    if (
      value >
      sensorMax[i]
    ) {

      sensorMax[i] = value;
    }
  }


  // ----------------------------------------------------------
  // Calibration LED
  // ----------------------------------------------------------

  if (
    millis() - lastCalibrationBlink
    >= 100
  ) {

    lastCalibrationBlink = millis();

    calibrationLEDState =
      !calibrationLEDState;


    digitalWrite(
      LED_PIN,
      calibrationLEDState
    );
  }


  // ----------------------------------------------------------
  // Calibration timeout
  // ----------------------------------------------------------

  if (
    millis() - calibrationStartTime
    >= CALIBRATION_TIME
  ) {

    finishCalibration();
  }
}


// ============================================================
// FINISH CALIBRATION
// ============================================================

void finishCalibration() {

  calibrationActive = false;

  botRunning = false;


  // ----------------------------------------------------------
  // Validate calibration
  // ----------------------------------------------------------

  bool valid = true;


  Serial.println();
  Serial.println("------------------------------------------");
  Serial.println("CALIBRATION RESULTS");
  Serial.println("------------------------------------------");


  for (int i = 0; i < NUM_SENSORS; i++) {

    int spread =
      sensorMax[i] -
      sensorMin[i];


    Serial.print("Sensor ");
    Serial.print(i + 1);

    Serial.print("  Min=");
    Serial.print(sensorMin[i]);

    Serial.print("  Max=");
    Serial.print(sensorMax[i]);

    Serial.print("  Spread=");
    Serial.println(spread);


    if (
      spread <
      MIN_SENSOR_SPREAD
    ) {

      valid = false;
    }
  }


  // ----------------------------------------------------------
  // Calibration result
  // ----------------------------------------------------------

  if (valid) {

    isCalibrated = true;

    digitalWrite(
      LED_PIN,
      HIGH
    );


    Serial.println();
    Serial.println("==========================================");
    Serial.println("CALIBRATION SUCCESSFUL");
    Serial.println("Waiting for START command.");
    Serial.println("==========================================");
  }

  else {

    isCalibrated = false;

    digitalWrite(
      LED_PIN,
      LOW
    );


    Serial.println();
    Serial.println("==========================================");
    Serial.println("CALIBRATION FAILED");
    Serial.println("At least one sensor had insufficient range.");
    Serial.println("Send CAL to try again.");
    Serial.println("==========================================");
  }


  stopMotors();


  // Send result immediately
  sendHealthData();

  sendStatus();
}


// ============================================================
// MOTOR A
// ============================================================

void setMotorA(int speed) {

  speed =
    constrain(
      speed,
      -255,
      255
    );


  // ----------------------------------------------------------
  // Apply motor inversion
  // ----------------------------------------------------------

  if (MOTOR_A_INVERT) {

    speed = -speed;
  }


  // ----------------------------------------------------------
  // Forward
  // ----------------------------------------------------------

  if (speed > 0) {

    int pwm =
      constrain(
        speed,
        MIN_PWM,
        255
      );


    digitalWrite(
      AIN1,
      HIGH
    );

    digitalWrite(
      AIN2,
      LOW
    );


    ledcWrite(
      PWMA,
      pwm
    );
  }


  // ----------------------------------------------------------
  // Reverse
  // ----------------------------------------------------------

  else if (speed < 0) {

    int pwm =
      constrain(
        -speed,
        MIN_PWM,
        255
      );


    digitalWrite(
      AIN1,
      LOW
    );

    digitalWrite(
      AIN2,
      HIGH
    );


    ledcWrite(
      PWMA,
      pwm
    );
  }


  // ----------------------------------------------------------
  // STOP
  // ----------------------------------------------------------

  else {

    digitalWrite(
      AIN1,
      LOW
    );

    digitalWrite(
      AIN2,
      LOW
    );


    ledcWrite(
      PWMA,
      0
    );
  }
}


// ============================================================
// MOTOR B
// ============================================================

void setMotorB(int speed) {

  speed =
    constrain(
      speed,
      -255,
      255
    );


  // ----------------------------------------------------------
  // Apply motor inversion
  // ----------------------------------------------------------

  if (MOTOR_B_INVERT) {

    speed = -speed;
  }


  // ----------------------------------------------------------
  // Forward
  // ----------------------------------------------------------

  if (speed > 0) {

    int pwm =
      constrain(
        speed,
        MIN_PWM,
        255
      );


    digitalWrite(
      BIN1,
      HIGH
    );

    digitalWrite(
      BIN2,
      LOW
    );


    ledcWrite(
      PWMB,
      pwm
    );
  }


  // ----------------------------------------------------------
  // Reverse
  // ----------------------------------------------------------

  else if (speed < 0) {

    int pwm =
      constrain(
        -speed,
        MIN_PWM,
        255
      );


    digitalWrite(
      BIN1,
      LOW
    );

    digitalWrite(
      BIN2,
      HIGH
    );


    ledcWrite(
      PWMB,
      pwm
    );
  }


  // ----------------------------------------------------------
  // STOP
  // ----------------------------------------------------------

  else {

    digitalWrite(
      BIN1,
      LOW
    );

    digitalWrite(
      BIN2,
      LOW
    );


    ledcWrite(
      PWMB,
      0
    );
  }
}


// ============================================================
// SET BOTH MOTORS
// ============================================================

void setMotors(
  int leftSpeed,
  int rightSpeed
) {

  leftSpeed =
    constrain(
      leftSpeed,
      -255,
      255
    );


  rightSpeed =
    constrain(
      rightSpeed,
      -255,
      255
    );


  setMotorA(
    leftSpeed
  );

  setMotorB(
    rightSpeed
  );
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors() {

  setMotorA(0);

  setMotorB(0);
}


// ============================================================
// READ SENSORS
// ============================================================

void readSensors() {

  for (
    int i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    sensorRaw[i] =
      analogRead(
        sensorPins[i]
      );
  }
}


// ============================================================
// NORMALIZE SENSOR VALUES
// ============================================================
//
// Converts:
//
//   sensorMin -> 0
//   sensorMax -> 1000
//
// Therefore:
//
//   0    = white
//   1000 = black
//
// ============================================================

void normalizeSensors() {

  for (
    int i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    int range =
      sensorMax[i] -
      sensorMin[i];


    // Prevent division by zero / bad calibration

    if (range < 20) {

      sensorValue[i] = 0;

      continue;
    }


    long value =
      (
        (long)(
          sensorRaw[i] -
          sensorMin[i]
        )
        * 1000L
      )
      / range;


    value =
      constrain(
        value,
        0,
        1000
      );


    // Small noise rejection

    if (
      value <
      LINE_THRESHOLD
    ) {

      value = 0;
    }


    sensorValue[i] =
      (int)value;
  }
}


// ============================================================
// CALCULATE LINE POSITION
// ============================================================

void calculatePosition() {

  long weightedSum = 0;

  long total = 0;


  for (
    int i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    int value =
      sensorValue[i];


    if (
      value <
      LINE_THRESHOLD
    ) {

      value = 0;
    }


    weightedSum +=
      (long)value *
      sensorWeight[i];


    total += value;
  }


  // ----------------------------------------------------------
  // LINE DETECTED
  // ----------------------------------------------------------

  if (total > 0) {

    lineDetected = true;


    position =
      (float)weightedSum /
      (float)total;


    // Remember direction

    if (position < -IMBALANCE_THRESHOLD) {

      lastKnownDirection = -1;
    }

    else if (position > IMBALANCE_THRESHOLD) {

      lastKnownDirection = 1;
    }
  }


  // ----------------------------------------------------------
  // LINE LOST
  // ----------------------------------------------------------

  else {

    lineDetected = false;


    if (
      lastKnownDirection < 0
    ) {

      position = -3500;
    }

    else if (
      lastKnownDirection > 0
    ) {

      position = 3500;
    }

    else {

      position = 0;
    }
  }
}


// ============================================================
// COUNT ACTIVE (BLACK) SENSORS
// ============================================================

int countActiveSensors() {

  int count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {

    if (sensorValue[i] > 0) {

      count++;
    }
  }

  return count;
}


// ============================================================
// SAMPLE SENSOR WIDTH HISTORY (for sharp-turn "cone" detection)
// ============================================================

void sampleSharpTurnHistory() {

  if (
    millis() - lastSharpSampleTime
    < SHARP_SAMPLE_INTERVAL
  ) {

    return;
  }

  lastSharpSampleTime = millis();


  activeCountHistory[0] = activeCountHistory[1];
  activeCountHistory[1] = activeCountHistory[2];
  activeCountHistory[2] = activeCountHistory[3];
  activeCountHistory[3] = countActiveSensors();
}


// ============================================================
// WAS THE LINE NARROWING BEFORE IT WAS LOST?
// ============================================================
//
// True when the last few width samples shrank step by step down to (near)
// zero - the "cone" signature of an approaching sharp/right-angle turn -
// rather than dropping out abruptly, which is what a dashed-line gap
// looks like.
//
// ============================================================

bool wasNarrowingBeforeLoss() {

  // Must have actually started reasonably wide.
  if (activeCountHistory[0] < 2) {

    return false;
  }


  // Must be non-increasing across the whole window...
  for (int i = 0; i < 3; i++) {

    if (activeCountHistory[i + 1] > activeCountHistory[i]) {

      return false;
    }
  }


  // ...and actually have shrunk, not just stayed flat.
  return activeCountHistory[0] > activeCountHistory[2];
}


// ============================================================
// UPDATE LINE-LOST RECOVERY STATE
// ============================================================
//
// Non-blocking state machine - see the RECOVERY_* comment block near the
// top of the file for the phase timeline.
//
// ============================================================

void updateLineLostState() {

  if (lineDetected) {

    lineLostStartTime = 0;

    lineWasDetectedLastLoop = true;

    lostWasSharpTurn = false;

    recoveryPhase = RECOVERY_NONE;

    return;
  }


  // ----------------------------------------------------------
  // Line NOT detected this loop
  // ----------------------------------------------------------

  if (lineWasDetectedLastLoop) {

    // Just lost it this instant - decide what caused it. Either signal
    // (gradual cone narrowing, or a hard skew right before whiteout) is
    // enough to call it a sharp turn - see the comment block above.

    lineLostStartTime = millis();

    bool coneNarrowed = wasNarrowingBeforeLoss();

    bool suddenImbalance = (lastKnownDirection != 0);

    lostWasSharpTurn = coneNarrowed || suddenImbalance;

    lineWasDetectedLastLoop = false;
  }


  unsigned long elapsed =
    millis() - lineLostStartTime;


  if (elapsed >= LINE_LOST_ABORT_MS) {

    recoveryPhase = RECOVERY_ABORTED;
  }

  else if (elapsed >= LINE_LOST_RECOVERY_MS) {

    if (
      elapsed - LINE_LOST_RECOVERY_MS
      < RECOVERY_REVERSE_MS
    ) {

      recoveryPhase = RECOVERY_REVERSE;
    }

    else {

      recoveryPhase = RECOVERY_WIDE_SEARCH;
    }
  }

  else if (
    !lostWasSharpTurn &&
    elapsed < DOTTED_LINE_GRACE_MS
  ) {

    recoveryPhase = RECOVERY_BRIDGE;
  }

  else {

    recoveryPhase = RECOVERY_SEARCH;
  }
}


// ============================================================
// LINE FOLLOWER
// ============================================================
//
// This is the NEW controller.
//
// The old movement / locomotion logic is NOT used here.
//
// ============================================================

void lineFollow() {

  // ----------------------------------------------------------
  // ERROR
  // ----------------------------------------------------------

  error = position;


  // ----------------------------------------------------------
  // ERROR DEADBAND
  // ----------------------------------------------------------
  //
  // Prevents tiny ADC noise around the center from causing
  // constant small corrections.
//
// ----------------------------------------------------------

  const float ERROR_DEADBAND = 50.0;

  float controlError;


  if (
    fabs(error) <
    ERROR_DEADBAND
  ) {

    controlError = 0;
  }

  else {

    controlError = error;
  }


  // ----------------------------------------------------------
  // DERIVATIVE
  // ----------------------------------------------------------

  derivative =
    controlError -
    previousError;


  // ----------------------------------------------------------
  // PD
  // ----------------------------------------------------------

  correction =
    (
      Kp *
      controlError
    )
    +
    (
      Kd *
      derivative
    );


  previousError =
    controlError;


  // ----------------------------------------------------------
  // LIMIT CORRECTION
  // ----------------------------------------------------------

  correction =
    constrain(
      correction,
      -255,
      255
    );


  // ----------------------------------------------------------
  // BASE SPEED
  // ----------------------------------------------------------

  float effectiveSpeed =
    masterSpeed;


  // ----------------------------------------------------------
  // SHARP TURN SPEED REDUCTION
  // ----------------------------------------------------------

  float errorRatio =
    fabs(error) /
    3500.0;


  errorRatio =
    constrain(
      errorRatio,
      0.0,
      1.0
    );


  const float TURN_SLOWDOWN = 0.65;


  effectiveSpeed =
    masterSpeed *
    (
      1.0 -
      TURN_SLOWDOWN *
      errorRatio
    );


  // ----------------------------------------------------------
  // MOTOR MIXING
  // ----------------------------------------------------------

  float leftSpeed =
    effectiveSpeed +
    correction;


  float rightSpeed =
    effectiveSpeed -
    correction;


  // ----------------------------------------------------------
  // LINE LOST
  // ----------------------------------------------------------
  //
  // See the RECOVERY_* phase machine (updateLineLostState) near
  // the top of the file for how recoveryPhase gets decided.
  //
  // ----------------------------------------------------------

  if (!lineDetected) {

    switch (recoveryPhase) {


      // ------------------------------------------------------
      // Likely just a gap in a dashed line - coast straight
      // through it instead of pivoting.
      // ------------------------------------------------------

      case RECOVERY_BRIDGE:

        leftSpeed = masterSpeed;

        rightSpeed = masterSpeed;

        break;


      // ------------------------------------------------------
      // Pivot toward the side the line was last seen on.
      // A confirmed sharp turn (cone narrowing and/or a sudden
      // skew right before loss - see lostWasSharpTurn) gets a
      // full tank-turn pivot so a 90 actually completes instead
      // of arcing too wide; an ordinary loss gets a gentler one.
      // ------------------------------------------------------

      case RECOVERY_SEARCH: {

        int insideSpeed =
          lostWasSharpTurn
            ? -masterSpeed
            : -60;


        if (lastKnownDirection < 0) {

          leftSpeed = insideSpeed;

          rightSpeed = masterSpeed;
        }

        else if (lastKnownDirection > 0) {

          leftSpeed = masterSpeed;

          rightSpeed = insideSpeed;
        }

        else {

          leftSpeed = 0;

          rightSpeed = 0;
        }

        break;
      }


      // ------------------------------------------------------
      // Lost for a while now - back up toward where the line
      // was last seen before widening the search.
      // ------------------------------------------------------

      case RECOVERY_REVERSE:

        leftSpeed = -80;

        rightSpeed = -80;

        break;


      // ------------------------------------------------------
      // Still not found after backing up - sweep harder/wider.
      // ------------------------------------------------------

      case RECOVERY_WIDE_SEARCH:

        if (lastKnownDirection < 0) {

          leftSpeed = -(int)(masterSpeed * 0.8);

          rightSpeed = masterSpeed;
        }

        else if (lastKnownDirection > 0) {

          leftSpeed = masterSpeed;

          rightSpeed = -(int)(masterSpeed * 0.8);
        }

        else {

          leftSpeed = 0;

          rightSpeed = 0;
        }

        break;


      // ------------------------------------------------------
      // Given up - stop rather than wander off-track forever.
      // ------------------------------------------------------

      case RECOVERY_ABORTED:
      default:

        leftSpeed = 0;

        rightSpeed = 0;

        botRunning = false;

        break;
    }
  }


  // ----------------------------------------------------------
  // LIMIT MOTOR SPEED
  // ----------------------------------------------------------

  leftSpeed =
    constrain(
      (int)leftSpeed,
      -255,
      255
    );


  rightSpeed =
    constrain(
      (int)rightSpeed,
      -255,
      255
    );


  // ----------------------------------------------------------
  // OUTPUT
  // ----------------------------------------------------------

  setMotors(
    (int)leftSpeed,
    (int)rightSpeed
  );
}


// ============================================================
// PROCESS UART COMMAND
// ============================================================
//
// Supported commands:
//
//   CAL
//   START
//   STOP
//
//   C,speed,kp,kd,run
//
// Examples:
//
//   CAL
//   START
//   STOP
//   C,120,0.18,0.50,1
//
// ============================================================

void processCommand(
  char *line
) {

  // ----------------------------------------------------------
  // Ignore empty command
  // ----------------------------------------------------------

  if (
    line == nullptr ||
    line[0] == '\0'
  ) {

    return;
  }


  Serial.print("UART RX: ");

  Serial.println(line);


  // ==========================================================
  // CALIBRATION COMMAND
  // ==========================================================

  if (
    strcmp(
      line,
      "CAL"
    ) == 0
  ) {

    Serial.println(
      "Command: CAL"
    );


    startCalibration();

    return;
  }


  // ==========================================================
  // START COMMAND
  // ==========================================================

  if (
    strcmp(
      line,
      "START"
    ) == 0
  ) {

    Serial.println(
      "Command: START"
    );


    if (
      isCalibrated &&
      !calibrationActive
    ) {

      botRunning = true;

      previousError = 0;

      error = 0;

      lastKnownDirection = 0;

          lineLostStartTime = 0;

      lineWasDetectedLastLoop = true;

      lostWasSharpTurn = false;

      recoveryPhase = RECOVERY_NONE;


      Serial.println(
        "BOT STARTED"
      );
    }

    else {

      botRunning = false;

      Serial.println(
        "START rejected: not calibrated."
      );
    }


    sendStatus();

    return;
  }


  // ==========================================================
  // STOP COMMAND
  // ==========================================================

  if (
    strcmp(
      line,
      "STOP"
    ) == 0
  ) {

    Serial.println(
      "Command: STOP"
    );


    botRunning = false;

    stopMotors();


    previousError = 0;

    error = 0;


    Serial.println(
      "BOT STOPPED"
    );


    sendStatus();

    return;
  }


  // ==========================================================
  // C,speed,kp,kd,run COMMAND
  // ==========================================================

  if (
    line[0] != 'C' ||
    line[1] != ','
  ) {

    Serial.println(
      "Unknown UART command."
    );

    return;
  }


  char *save = nullptr;


  // ----------------------------------------------------------
  // SPEED
  // ----------------------------------------------------------

  char *token =
    strtok_r(
      line + 2,
      ",",
      &save
    );


  if (!token) {

    return;
  }


  int newSpeed =
    atoi(token);


  // ----------------------------------------------------------
  // KP
  // ----------------------------------------------------------

  token =
    strtok_r(
      nullptr,
      ",",
      &save
    );


  if (!token) {

    return;
  }


  float newKp =
    atof(token);


  // ----------------------------------------------------------
  // KD
  // ----------------------------------------------------------

  token =
    strtok_r(
      nullptr,
      ",",
      &save
    );


  if (!token) {

    return;
  }


  float newKd =
    atof(token);


  // ----------------------------------------------------------
  // RUN
  // ----------------------------------------------------------

  token =
    strtok_r(
      nullptr,
      ",",
      &save
    );


  if (!token) {

    return;
  }


  int run =
    atoi(token);


  // ----------------------------------------------------------
  // VALIDATE
  // ----------------------------------------------------------

  newSpeed =
    constrain(
      newSpeed,
      0,
      255
    );


  newKp =
    constrain(
      newKp,
      0.0f,
      10.0f
    );


  newKd =
    constrain(
      newKd,
      0.0f,
      10.0f
    );


  // ----------------------------------------------------------
  // UPDATE PARAMETERS
  // ----------------------------------------------------------

  masterSpeed =
    newSpeed;

  Kp =
    newKp;

  Kd =
    newKd;


  // ----------------------------------------------------------
  // RUN STATE
  // ----------------------------------------------------------

  if (
    run == 1 &&
    isCalibrated &&
    !calibrationActive
  ) {

    botRunning = true;
  }

  else {

    botRunning = false;
  }


  // ----------------------------------------------------------
  // Reset controller state
  // ----------------------------------------------------------

  previousError = 0;

  error = 0;

  lastKnownDirection = 0;

  lineLostStartTime = 0;

  lineWasDetectedLastLoop = true;

  lostWasSharpTurn = false;

  recoveryPhase = RECOVERY_NONE;


  if (!botRunning) {

    stopMotors();
  }


  // ----------------------------------------------------------
  // Debug
  // ----------------------------------------------------------

  Serial.print("Speed = ");
  Serial.println(masterSpeed);

  Serial.print("Kp = ");
  Serial.println(Kp, 4);

  Serial.print("Kd = ");
  Serial.println(Kd, 4);

  Serial.print("Running = ");
  Serial.println(
    botRunning ? 1 : 0
  );


  sendStatus();
}


// ============================================================
// READ UART
// ============================================================
//
// UART is intentionally processed on EVERY loop iteration.
//
// This is what prevents the OLED button from becoming
// unresponsive during calibration.
//
// ============================================================

void readUART() {

  while (
    UARTLink.available()
  ) {

    char c =
      UARTLink.read();


    // --------------------------------------------------------
    // END OF LINE
    // --------------------------------------------------------

    if (
      c == '\n'
    ) {

      uartBuffer[uartLen] =
        '\0';


      if (
        uartLen > 0
      ) {

        processCommand(
          uartBuffer
        );
      }


      uartLen = 0;
    }


    // --------------------------------------------------------
    // IGNORE CR
    // --------------------------------------------------------

    else if (
      c == '\r'
    ) {

      // Ignore carriage return
    }


    // --------------------------------------------------------
    // NORMAL CHARACTER
    // --------------------------------------------------------

    else {

      if (
        uartLen <
        (
          (int)sizeof(uartBuffer)
          - 1
        )
      ) {

        uartBuffer[uartLen++] =
          c;
      }

      else {

        // Overflow protection

        uartLen = 0;

        Serial.println(
          "UART buffer overflow."
        );
      }
    }
  }
}


// ============================================================
// SEND STATUS
// ============================================================
//
// OLED:
//
//   S,0,0
//      ^ ^
//      | running
//      calibrated
//
//   S,1,0 = calibrated, stopped
//
//   S,1,1 = calibrated, running
//
// ============================================================

void sendStatus() {

  UARTLink.print("S,");

  UARTLink.print(
    isCalibrated ? 1 : 0
  );

  UARTLink.print(",");

  UARTLink.println(
    botRunning ? 1 : 0
  );
}


// ============================================================
// SEND SENSOR DATA
// ============================================================
//
// Format:
//
// D,s1,s2,s3,s4,s5,s6,s7,s8
//
// Values:
// 0 - 1000
//
// ============================================================

void sendSensorData() {

  if (
    millis() - lastSensorSend
    <
    SENSOR_SEND_INTERVAL
  ) {

    return;
  }


  lastSensorSend =
    millis();


  UARTLink.print("D");


  for (
    int i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    UARTLink.print(",");

    UARTLink.print(
      sensorValue[i]
    );
  }


  UARTLink.println();
}


// ============================================================
// SEND SENSOR HEALTH
// ============================================================
//
// Format:
//
// H,spread1,spread2,...spread8
//
// Healthy sensor:
//
// spread >= 300
//
// ============================================================

void sendHealthData() {

  UARTLink.print("H");


  for (
    int i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    int spread =
      sensorMax[i] -
      sensorMin[i];


    UARTLink.print(",");

    UARTLink.print(
      spread
    );
  }


  UARTLink.println();
}

