// Motor Driver Pins (TB6612FNG)
const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 4;
const int PWMB = 17;

const int STBY = 5;

// Built-in LED Pin
const int LED_PIN = 2;

// Base Motor Speeds (With inherent hardware compensation)
const int BASE_SPEED_A = 200; // Left Motor
const int BASE_SPEED_B = 170; // Right Motor
const int MAX_SPEED = 255;

// QTR-8A Analog Pins
const int NUM_SENSORS = 8;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// Calibration boundaries
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

// PID Tuning Constants
float Kp = 0.04;    // Proportional gain
float Ki = 0.0001;  // Integral gain
float Kd = 0.25;    // Derivative gain

// PID Internal Variables
int lastError = 0;
float integral = 0;
int lastPosition = 3500; // Center position default (0 to 7000)

void setMotors(int speedA, int speedB);
int getLinePosition();
void stopMotors();

void setup() {
  Serial.begin(115200);

  // Set ESP32 ADC attenuation to read full 0 - 3.3V range
  analogSetAttenuation(ADC_11db);

  // Motor Pin Setup
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

  // Calibration Countdown
  Serial.println("Get ready! Calibration starts in 5 seconds...");
  for (int i = 5; i > 0; i--) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(800);
  }

  // Active Calibration Phase (8 Seconds)
  Serial.println("CALIBRATING: SWEEP SENSORS BACK AND FORTH OVER LINE!");
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
  Serial.println("Calibration Complete!");
  delay(2000);
}

void loop() {
  int position = getLinePosition();

  // Error ranges from -3500 (Far Left) to +3500 (Far Right). 0 = Centered
  int error = position - 3500;

  // PID Calculations
  float P = error;
  integral += error;
  integral = constrain(integral, -10000, 10000); // Prevent integral windup
  float D = error - lastError;
  lastError = error;

  float correction = (Kp * P) + (Ki * integral) + (Kd * D);

  // Apply correction relative to compensated base speeds
  int motorSpeedA = BASE_SPEED_A + correction; // Left Motor
  int motorSpeedB = BASE_SPEED_B - correction; // Right Motor

  setMotors(motorSpeedA, motorSpeedB);

  delay(5);
}

// -------------------------------------------------------------
// Calculate Line Position using Weighted Average (0 to 7000)
// -------------------------------------------------------------
int getLinePosition() {
  unsigned long weightedSum = 0;
  unsigned long sum = 0;
  bool lineDetected = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = analogRead(sensorPins[i]);
    
    // Normalize raw reading from 0 (white) to 1000 (black)
    int scaledVal = map(val, sensorMin[i], sensorMax[i], 0, 1000);
    scaledVal = constrain(scaledVal, 0, 1000);

    if (scaledVal > 200) {
      lineDetected = true;
    }

    weightedSum += (unsigned long)scaledVal * (i * 1000);
    sum += scaledVal;
  }

  // Memory fallback if line is completely lost
  if (!lineDetected || sum == 0) {
    if (lastPosition < 3500) {
      return 0;      // Hold hard-left position
    } else {
      return 7000;   // Hold hard-right position
    }
  }

  lastPosition = weightedSum / sum;
  return lastPosition;
}

// -------------------------------------------------------------
// Motor Control with Reverse Differential Steering
// -------------------------------------------------------------
void setMotors(int speedA, int speedB) {
  speedA = constrain(speedA, -MAX_SPEED, MAX_SPEED);
  speedB = constrain(speedB, -MAX_SPEED, MAX_SPEED);

  // Left Motor (A)
  if (speedA > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, speedA);
  } else if (speedA < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(speedA));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }

  // Right Motor (B)
  if (speedB > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, speedB);
  } else if (speedB < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, abs(speedB));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void stopMotors() {
  setMotors(0, 0);
}
