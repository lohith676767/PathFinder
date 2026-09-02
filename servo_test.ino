#include <ESP32Servo.h>

// Pin definitions
const int ledPin = 2;       // Onboard LED pin on standard 30-pin ESP32 boards
const int servoPin = 18;    // Digital PWM pin connected to servo signal wire

Servo myServo;              // Create servo object

void setup() {
  // Initialize serial monitor for debugging
  Serial.begin(115200);

  // Initialize the onboard LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Allow allocation of all timers for the ESP32Servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Standard 50Hz servo configuration (min/max pulse widths in microseconds)
  myServo.setPeriodHertz(50); 
  myServo.attach(servoPin, 500, 2400); 

  Serial.println("ESP32 Setup Complete: LED and Servo ready.");
}

void loop() {
  // Turn LED ON and sweep servo to 0 degrees
  digitalWrite(ledPin, HIGH);
  myServo.write(0);
  Serial.println("LED ON | Servo at 0°");
  delay(1000);

  // Move servo to 90 degrees
  myServo.write(90);
  Serial.println("Servo at 90°");
  delay(1000);

  // Turn LED OFF and sweep servo to 180 degrees
  digitalWrite(ledPin, LOW);
  myServo.write(180);
  Serial.println("LED OFF | Servo at 180°");
  delay(1000);

  // Move servo back to 90 degrees before looping
  myServo.write(90);
  delay(1000);
}
