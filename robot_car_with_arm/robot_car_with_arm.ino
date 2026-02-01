/*
 * HACKATHON COMPETITION CODE - FULLY AUTONOMOUS
 * No serial dependency - runs on battery power
 */

#include <Servo.h>

// ==================== PIN DEFINITIONS ====================

// Color Sensor
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8
#define OE A2

// IR Sensors (for ramp detection)
#define IR_LEFT A0
#define IR_RIGHT A1

// Ultrasonic Sensor
#define TRIG_PIN 9
#define ECHO_PIN 17  // A3 as digital pin

// L298N Motor Driver
#define ENA 3
#define IN1 2
#define IN2 11
#define ENB 10
#define IN3 12
#define IN4 13

// Lift Arm Servo
#define LIFT_ARM_PIN 18  // A4 as digital pin

// ==================== GLOBAL VARIABLES ====================

Servo liftArm;

// Arm positions
const int ARM_HOME = 90;
const int ARM_HOOK_DOWN = 65;
const int ARM_LIFT_UP = 110;

// Motor speeds
int motorSpeed = 150;
int turnSpeed = 100;
int slowSpeed = 80;
int rampClimbSpeed = 180;

// Ultrasonic
long duration;
int distance;

// Timing variables
const unsigned long TURN_90_TIME = 650;
const unsigned long TURN_180_TIME = 1300;
const unsigned long MOVE_BOX_OFFSET = 300;

// State tracking
bool hasBox = false;
enum RobotState {
  SEARCHING_FOR_PATH,
  FOLLOWING_PATH,
  HANDLING_BOX,
  TARGET_SHOOTING,
  OBSTACLE_COURSE,
  COMPLETING_COURSE
};

RobotState currentState = SEARCHING_FOR_PATH;

// Color sensor readings
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Color ranges - UPDATE WITH YOUR VALUES!
struct ColorRange {
  int rMin, rMax;
  int gMin, gMax;
  int bMin, bMax;
};

// CALIBRATE THESE!
ColorRange pathRed = {20, 50, 80, 150, 80, 150};
ColorRange pathGreen = {80, 150, 20, 50, 80, 150};
ColorRange pathBlue = {80, 150, 80, 150, 20, 50};

// ==================== SETUP ====================

void setup() {
  // NO SERIAL = FASTER STARTUP!
  // Serial.begin(9600);  // REMOVED for competition
  
  // Initialize everything quickly
  initializeRobot();
  
  // Wait 2 seconds for robot placement
  delay(2000);
  
  // Start autonomous operation immediately
  currentState = SEARCHING_FOR_PATH;
}

void initializeRobot() {
  // Color Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(OE, OUTPUT);

  digitalWrite(OE, LOW);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // IR Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor Driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Servo
  liftArm.attach(LIFT_ARM_PIN);
  liftArm.write(ARM_HOME);
}

// ==================== MAIN LOOP ====================

void loop() {
  // Read sensors
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);
  distance = getDistance();
  String color = detectPathColor();
  
  // Main autonomous state machine
  switch (currentState) {
    case SEARCHING_FOR_PATH:
      searchForPath(irLeft, irRight, color);
      break;
      
    case FOLLOWING_PATH:
      followPath(irLeft, irRight, distance, color);
      break;
      
    case HANDLING_BOX:
      handleBoxSequence();
      break;
      
    case TARGET_SHOOTING:
      executeTargetShooting(irLeft, irRight, color);
      break;
      
    case OBSTACLE_COURSE:
      executeObstacleCourse(irLeft, irRight, distance);
      break;
      
    case COMPLETING_COURSE:
      completeCourse();
      break;
  }
  
  delay(40);
}

// ==================== AUTONOMOUS DECISION MAKING ====================

void searchForPath(int irLeft, int irRight, String color) {
  // Move forward slowly while looking for a colored path
  moveForwardSlow();
  
  if (color == "RED") {
    // Found red path - obstacle course
    stopMotors();
    delay(500);
    currentState = OBSTACLE_COURSE;
  }
  else if (color == "GREEN") {
    // Found green path - target shooting
    stopMotors();
    delay(500);
    currentState = TARGET_SHOOTING;
  }
  else if (color == "BLUE" && !hasBox) {
    // Found blue tape - box pickup
    stopMotors();
    delay(500);
    currentState = HANDLING_BOX;
  }
  else {
    // Keep searching
    currentState = FOLLOWING_PATH;
  }
}

void followPath(int irLeft, int irRight, int distance, String color) {
  // Simple obstacle avoidance
  if (distance < 15 && distance > 0) {
    avoidObstacle();
    return;
  }
  
  // Check for color decisions
  if (color == "RED") {
    // Switch to obstacle course
    currentState = OBSTACLE_COURSE;
    return;
  }
  else if (color == "GREEN") {
    // Switch to target shooting
    currentState = TARGET_SHOOTING;
    return;
  }
  else if (color == "BLUE" && !hasBox) {
    // Box pickup
    currentState = HANDLING_BOX;
    return;
  }
  
  // Basic forward movement
  moveForward();
}

// ==================== BOX HANDLING ====================

void handleBoxSequence() {
  static int boxStep = 0;
  static unsigned long boxTimer = 0;
  
  switch (boxStep) {
    case 0: // Turn to face box
      turnRight();
      delay(TURN_90_TIME);
      stopMotors();
      boxStep = 1;
      boxTimer = millis();
      break;
      
    case 1: // Approach box
      moveForwardSlow();
      if (millis() - boxTimer > MOVE_BOX_OFFSET) {
        stopMotors();
        boxStep = 2;
        boxTimer = millis();
      }
      break;
      
    case 2: // Lower hook
      liftArm.write(ARM_HOOK_DOWN);
      delay(1000);
      boxStep = 3;
      boxTimer = millis();
      break;
      
    case 3: // Lift box
      liftArm.write(ARM_LIFT_UP);
      delay(1000);
      hasBox = true;
      boxStep = 4;
      boxTimer = millis();
      break;
      
    case 4: // Back up
      moveBackwardSlow();
      if (millis() - boxTimer > 300) {
        stopMotors();
        boxStep = 5;
        boxTimer = millis();
      }
      break;
      
    case 5: // Turn to place
      turnLeft();
      delay(TURN_180_TIME);
      stopMotors();
      boxStep = 6;
      boxTimer = millis();
      break;
      
    case 6: // Move to place position
      moveForwardSlow();
      if (millis() - boxTimer > MOVE_BOX_OFFSET) {
        stopMotors();
        boxStep = 7;
        boxTimer = millis();
      }
      break;
      
    case 7: // Place box
      liftArm.write(ARM_HOOK_DOWN);
      delay(1000);
      boxStep = 8;
      boxTimer = millis();
      break;
      
    case 8: // Return arm to home
      liftArm.write(ARM_HOME);
      delay(500);
      hasBox = false;
      boxStep = 9;
      boxTimer = millis();
      break;
      
    case 9: // Return to path
      turnRight();
      delay(TURN_90_TIME);
      stopMotors();
      currentState = FOLLOWING_PATH;
      boxStep = 0;
      break;
  }
}

// ==================== TARGET SHOOTING ====================

void executeTargetShooting(int irLeft, int irRight, String color) {
  static int targetStep = 0;
  static unsigned long targetTimer = 0;
  
  switch (targetStep) {
    case 0: // Climb ramp using IR sensors
      climbRamp(irLeft, irRight);
      if (irLeft == 1 && irRight == 1) { // At top
        stopMotors();
        targetStep = 1;
        targetTimer = millis();
      }
      break;
      
    case 1: // Navigate to center (simplified)
      // For competition: Just drive forward to push ball
      moveForwardSlow();
      if (millis() - targetTimer > 2000) {
        stopMotors();
        targetStep = 2;
        targetTimer = millis();
      }
      break;
      
    case 2: // Descend ramp
      turnRight();
      delay(1600);
      stopMotors();
      moveForwardSlow();
      delay(1000);
      currentState = COMPLETING_COURSE;
      targetStep = 0;
      break;
  }
}

void climbRamp(int irLeft, int irRight) {
  if (irLeft == 0 && irRight == 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, rampClimbSpeed);
    analogWrite(ENB, rampClimbSpeed);
  }
  else if (irLeft == 0 && irRight == 1) {
    analogWrite(ENA, rampClimbSpeed - 50);
    analogWrite(ENB, rampClimbSpeed);
  }
  else if (irLeft == 1 && irRight == 0) {
    analogWrite(ENA, rampClimbSpeed);
    analogWrite(ENB, rampClimbSpeed - 50);
  }
  else {
    // At top - handled by calling function
  }
}

// ==================== OBSTACLE COURSE ====================

void executeObstacleCourse(int irLeft, int irRight, int distance) {
  // Simple obstacle course: forward + avoid
  if (distance < 20 && distance > 0) {
    avoidObstacle();
  } else {
    moveForward();
  }
  
  // After some time, consider it complete
  static unsigned long courseStart = millis();
  if (millis() - courseStart > 45000) { // 45 seconds
    currentState = COMPLETING_COURSE;
  }
}

void completeCourse() {
  // Drive forward to finish
  moveForward();
}

// ==================== COLOR DETECTION ====================

String detectPathColor() {
  // Read color sensor
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  delay(2);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  delay(2);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  delay(2);

  // Check ranges
  if (isInRange(redFrequency, greenFrequency, blueFrequency, pathRed)) {
    return "RED";
  }
  if (isInRange(redFrequency, greenFrequency, blueFrequency, pathGreen)) {
    return "GREEN";
  }
  if (isInRange(redFrequency, greenFrequency, blueFrequency, pathBlue)) {
    return "BLUE";
  }
  
  return "NONE";
}

bool isInRange(int r, int g, int b, ColorRange range) {
  return (r >= range.rMin && r <= range.rMax &&
          g >= range.gMin && g <= range.gMax &&
          b >= range.bMin && b <= range.bMax);
}

// ==================== MOTOR FUNCTIONS ====================

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveForwardSlow() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, slowSpeed);
  analogWrite(ENB, slowSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackwardSlow() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, slowSpeed);
  analogWrite(ENB, slowSpeed);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ==================== ULTRASONIC ====================

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = duration * 0.034 / 2;

  if (distance == 0 || distance > 400) return 400;
  return distance;
}

// ==================== OBSTACLE AVOIDANCE ====================

void avoidObstacle() {
  stopMotors();
  delay(300);
  moveBackward();
  delay(500);
  turnRight();
  delay(700);
  stopMotors();
}
