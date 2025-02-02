//This repository is designed to work with in the Arduino IDE with the LiquidCrystal extension

// Variables
volatile int distanceEncoderCount = 0; // # of rotations of the driveshaft
volatile int speedEncoderCount = 0; // # of rotations to calculate speed
volatile int potentiometerValue; // Dial to select target distance 
volatile float motorSpeed = 0; // PWM control value for motor
volatile float actualSpeed = 0; // Measured motor speed in RPM
volatile int standbySwitch; // Toggle starter switch
int accelerateIncrement = 15; // Controls rate of speed change

// Constants
const int pulsesPerRotation = 408; // Encoder pulses per full wheel rotation
const float wheelDiameter = 11.3; // Wheel diameter in cm
const float wheelCircumference = wheelDiameter * 3.14159; // Circumference calculation
const int maxSpeed = 200; // Max speed limit
const float crawlSpeed = 15;
const int minDecelDistance = 50; // Minimum cm to start slowing
const float speedScalingFactor = 0.6; // Reduce speed as target approaches
const float distanceThreshold = 0.05; // Rotations precision
const unsigned long updateInterval = 100; // Interval for loop updates
const unsigned long LCDupdateInterval = 250; // Interval for LCD updates
const unsigned long speedCalcInterval = 100; // Interval for speed calculations
unsigned long lastLCDUpdate = 0; // Track last LCD update time

unsigned long lastSpeedCalc = 0;
unsigned long lastUpdateTime = 0;

// Set motion states
enum MotorState {
  STANDBY, // Waiting for activation
  ACCELERATING, // Increasing speed
  MAINTAINING, // Holding speed steady
  DECELERATING, // Slowing down
  CRAWLING, // Maintains slow speed until target
  STOPPED // Complete stop after run
};
MotorState currentState = STANDBY;

// LCD Setup
#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 13, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// New: Calculate deceleration point based on target distance
float calculateDecelPoint(float targetRotations) {
    float minDecelRotations = minDecelDistance / wheelCircumference;
    return targetRotations - minDecelRotations;
}

// LCD updates
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  switch (currentState) {
    case STANDBY:
      lcd.print("Ready - Set Dist");
      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(map(potentiometerValue, 0, 1023, 700, 1000));
      lcd.print(" cm");
      break;
      
    case ACCELERATING:
      lcd.print("Accelerating");
      lcd.setCursor(0, 1);
      lcd.print("RPM: ");
      lcd.print((int)actualSpeed);
      break;
      
    case MAINTAINING:
      lcd.print("Max Speed");
      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distanceEncoderCount / pulsesPerRotation * wheelCircumference);
      break;
      
    case DECELERATING:
      lcd.print("Slowing Down");
      lcd.setCursor(0, 1);
      lcd.print("RPM: ");
      lcd.print((int)actualSpeed);
      break;

    case CRAWLING:
      lcd.print("Crawling");
      lcd.setCursor(0,1);
      lcd.print("Dist: ");
      lcd.print(distanceEncoderCount / pulsesPerRotation * wheelCircumference);
      break;
      
    case STOPPED:
      lcd.print("Stopped");
      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distanceEncoderCount / pulsesPerRotation * wheelCircumference);
      break;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT); // Potentiometer Input
  pinMode(2, INPUT); // Quad Encoder A
  pinMode(3, INPUT); // Quad Encoder B
  pinMode(7, INPUT_PULLUP); // Toggle Switch for standby
  pinMode(8, OUTPUT); // IN1 Motor direction pin
  pinMode(9, OUTPUT); // IN2 Motor direction pin
  pinMode(10, OUTPUT); // Motor Control
 
  attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
  lcd.begin(16, 2);
  lcd.print("Ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    updateMotorSpeed();
  }
  
  if (currentTime - lastLCDUpdate >= LCDupdateInterval) {
    lastLCDUpdate = currentTime;
    updateLCD();
  }
}

void updateMotorSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - lastSpeedCalc >= speedCalcInterval) {
    lastSpeedCalc = currentTime;
    actualSpeed = ((float)speedEncoderCount / pulsesPerRotation) * (60000.0 / speedCalcInterval);
    speedEncoderCount = 0;
  }

  standbySwitch = digitalRead(7);
  potentiometerValue = analogRead(A0);
  int threshold = constrain(map(potentiometerValue, 0, 1023, 700, 1000), 700, 1000);
  float targetRotations = threshold / wheelCircumference;
  float maintainingStartRotations = 2.82; // 100/wheel circumference
  
  // Determine deceleration point
  float decelStartRotations = calculateDecelPoint(targetRotations);

  switch (currentState) {
    case STANDBY:
      if (standbySwitch == LOW) {
        distanceEncoderCount = 0;
        currentState = ACCELERATING;
      }
      break;

    case ACCELERATING:
      if (motorSpeed < maxSpeed) {
        motorSpeed += accelerateIncrement;
        setMotorSpeed(motorSpeed);
      }
      if (distanceEncoderCount / pulsesPerRotation >= maintainingStartRotations) {
        currentState = MAINTAINING;
      }
      break;

    case MAINTAINING:
      setMotorSpeed(maxSpeed);
      if (distanceEncoderCount / pulsesPerRotation >= decelStartRotations) {
        motorSpeed = maxSpeed * 0.9;
        currentState = DECELERATING;
      }
      break;

    case DECELERATING:
      float remainingRotations = targetRotations - (distanceEncoderCount / pulsesPerRotation);
      if (remainingRotations > 0) {  // Make sure we haven't passed target
        motorSpeed = max(crawlSpeed, motorSpeed - 2); // Fixed deceleration rate
        setMotorSpeed(motorSpeed);
      } else {
        motorSpeed = 0;  // Emergency stop if we pass target
        setMotorSpeed(motorSpeed);
        currentState = STOPPED;
      }
      break;

    case CRAWLING:
      if (distanceEncoderCount / pulsesPerRotation < targetRotations) {
        setMotorSpeed(crawlSpeed);
      } else {
        currentState = STOPPED;
      }
      break;

    case STOPPED:
      motorSpeed = 0;
      setMotorSpeed(motorSpeed);
      break;
  }
}

void encoderISR() {
  distanceEncoderCount += 1;
  speedEncoderCount += 1;
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, maxSpeed);
 
  if (speed == 0) {
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
  } else {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }
 
  analogWrite(10, speed);
}
