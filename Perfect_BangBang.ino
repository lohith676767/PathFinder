// Motor Driver Pins (TB6612FNG)
const int AIN1 = 19;
const int AIN2 = 21;
const int PWMA = 18;

const int BIN1 = 16;
const int BIN2 = 22; // Rewired from GPIO 4 to 22
const int PWMB = 17;

const int STBY = 5;

// Built-in LED Pin
const int LED_PIN = 2;

// Motor Speeds
const int SPEED_A = 200;       // Left Motor base speed
const int SPEED_B = 170;       // Right Motor base speed
const int TURN_SPEED_A = 150;  // Left Motor turning speed
const int TURN_SPEED_B = 120;  // Right Motor turning speed

// QTR-8A Analog Pins (Using the 6 ADC1 pins only)
const int NUM_SENSORS = 6;
const int sensorPins[NUM_SENSORS] = {36, 39, 34, 35, 32, 33};

// Calibration boundaries
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorValues[NUM_SENSORS];

// Function Prototypes
void setMotors(int dirA, int speedA, int dirB, int speedB);
void moveForward();
void turnLeft();
void turnRight();
void gentleLeft();
void gentleRight();
void stopMotors();

void setup() {
  Serial.begin(115200);

  // Set ESP32 ADC attenuation to read full 0 - 3.3V range
  analogSetAttenuation(ADC_11db);

  // Motor & LED Setup
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
  // SWEEP SENSORS MANUALLY ACROSS BOTH BLACK LINE AND WHITE SURFACE!
  // -------------------------------------------------------------
  Serial.println("CALIBRATING NOW: SWEEP SENSORS BACK AND FORTH OVER LINE!");
  
  unsigned long startTime = millis();
  unsigned long lastBlink = 0;
  bool ledState = false;

  while (millis() - startTime < 8000) {
    // Fast blink LED to signal active calibration
    if (millis() - lastBlink > 100) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }

    // Capture min and max analog readings across all 6 sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(5);
  }

  digitalWrite(LED_PIN, HIGH); // Solid ON when calibration ends
  Serial.println("Calibration Complete!");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" Min: "); Serial.print(sensorMin[i]);
    Serial.print(" | Max: "); Serial.print(sensorMax[i]);
    Serial.print(" | Diff: "); Serial.println(sensorMax[i] - sensorMin[i]);
  }

  delay(2000);
}

void loop() {
  bool onBlack[NUM_SENSORS];
  String sensorPattern = "";

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    int range = sensorMax[i] - sensorMin[i];

    // Check that calibration captured a valid difference (> 300 ADC units)
    if (range > 300) {
      int threshold = sensorMin[i] + (range / 2);
      onBlack[i] = (sensorValues[i] > threshold);
    } else {
      onBlack[i] = false; 
    }
    
    sensorPattern += onBlack[i] ? "B" : "W";
  }

  Serial.print("[");
  Serial.print(sensorPattern);
  Serial.print("] - ");

  // -------------------------------------------------------------
  // Decision Matrix (6 Sensors: S0 to S5)
  // -------------------------------------------------------------
  if (onBlack[2] || onBlack[3]) {
    Serial.println("STRAIGHT");
    moveForward();
  }
  else if (onBlack[0]) {
    Serial.println("SHARP LEFT");
    turnLeft();
  }
  else if (onBlack[1]) {
    Serial.println("GENTLE LEFT");
    gentleLeft();
  }
  else if (onBlack[5]) {
    Serial.println("SHARP RIGHT");
    turnRight();
  }
  else if (onBlack[4]) {
    Serial.println("GENTLE RIGHT");
    gentleRight();
  }
  else {
    Serial.println("LINE LOST - STOPPING");
    stopMotors();
  }

  delay(10);
}

// -------------------------------------------------------------
// Motor Functions
// -------------------------------------------------------------

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

void moveForward()  { setMotors(1, SPEED_A, 1, SPEED_B); }
void turnLeft()     { setMotors(-1, TURN_SPEED_A, 1, SPEED_B); }
void turnRight()    { setMotors(1, SPEED_A, -1, TURN_SPEED_B); }
void gentleLeft()   { setMotors(1, TURN_SPEED_A, 1, SPEED_B); }
void gentleRight()  { setMotors(1, SPEED_A, 1, TURN_SPEED_B); }
void stopMotors()   { setMotors(0, 0, 0, 0); }
