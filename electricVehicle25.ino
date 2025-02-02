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
const int PULSES_PER_ROTATION = 408; // Encoder pulses per full wheel rotation
const float WHEEL_DIAMETER = 11.3; // Wheel diameter in cm
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159; // Circumference calculation
const int MAX_SPEED = 200; // Max speed limit
const float CRAWL_SPEED = 15;
const int MIN_DECEL_DISTANCE = 50; // Minimum cm to start slowing
const float SPEED_SCALING_FACTOR = 0.6; // Reduce speed as target approaches
const float DISTANCE_THRESHOLD = 0.05; // Rotations precision
const unsigned long UPDATE_INTERVAL = 100; // Interval for loop updates
const unsigned long LCD_UPDATE_INTERVAL = 250; // Interval for LCD updates
const unsigned long SPEED_CALC_INTERVAL = 100; // Interval for speed calculations
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
    float minDecelRotations = MIN_DECEL_DISTANCE / WHEEL_CIRCUMFERENCE;
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
      lcd.print(distanceEncoderCount / PULSES_PER_ROTATION * WHEEL_CIRCUMFERENCE);
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
      lcd.print(distanceEncoderCount / PULSES_PER_ROTATION * WHEEL_CIRCUMFERENCE);
      break;
      
    case STOPPED:
      lcd.print("Stopped");
      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distanceEncoderCount / PULSES_PER_ROTATION * WHEEL_CIRCUMFERENCE);
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
  
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    updateMotorSpeed();
  }
  
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    lastLCDUpdate = currentTime;
    updateLCD();
  }
}

void updateMotorSpeed() {
  unsigned long currentTime = millis();
  if (currentTime - lastSpeedCalc >= SPEED_CALC_INTERVAL) {
    lastSpeedCalc = currentTime;
    actualSpeed = ((float)speedEncoderCount / PULSES_PER_ROTATION) * (60000.0 / SPEED_CALC_INTERVAL);
    speedEncoderCount = 0;
  }

  standbySwitch = digitalRead(7);
  potentiometerValue = analogRead(A0);
  int threshold = constrain(map(potentiometerValue, 0, 1023, 700, 1000), 700, 1000);
  float targetRotations = threshold / WHEEL_CIRCUMFERENCE;
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
      if (motorSpeed < MAX_SPEED) {
        motorSpeed += accelerateIncrement;
        setMotorSpeed(motorSpeed);
      }
      if (distanceEncoderCount / PULSES_PER_ROTATION >= maintainingStartRotations) {
        currentState = MAINTAINING;
      }
      break;

    case MAINTAINING:
      setMotorSpeed(MAX_SPEED);
      if (distanceEncoderCount / PULSES_PER_ROTATION >= decelStartRotations) {
        motorSpeed = MAX_SPEED * 0.9;
        currentState = DECELERATING;
      }
      break;

    case DECELERATING:
      float remainingRotations = targetRotations - (distanceEncoderCount / PULSES_PER_ROTATION);
      if (remainingRotations > 0) {  // Make sure we haven't passed target
        motorSpeed = max(CRAWL_SPEED, motorSpeed - 2); // Fixed deceleration rate
        setMotorSpeed(motorSpeed);
      } else {
        motorSpeed = 0;  // Emergency stop if we pass target
        setMotorSpeed(motorSpeed);
        currentState = STOPPED;
      }
      break;

    case CRAWLING:
      if (distanceEncoderCount / PULSES_PER_ROTATION < targetRotations) {
        setMotorSpeed(CRAWL_SPEED);
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
  speed = constrain(speed, 0, MAX_SPEED);
 
  if (speed == 0) {
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
  } else {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }
 
  analogWrite(10, speed);
}