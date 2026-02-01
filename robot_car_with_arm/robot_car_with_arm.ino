/*
 * Autonomous Robot Car with Robotic Arm
 * Arduino Uno - Robotics Hackathon
 *
 * HARDWARE:
 * - V575 Color Sensor (TCS3200-based)
 * - 2x MH Sensor Series Flying Fish IR Sensors
 * - HY-SRF05 Ultrasonic Sensor
 * - 2x SG90 Servos
 * - DC Motors + L298N
 */

#include <Servo.h>

// ==================== PIN DEFINITIONS ====================

// V575 Color Sensor
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8
#define OE A2  // Active LOW

// IR Sensors
#define IR_LEFT A0
#define IR_RIGHT A1

// Ultrasonic Sensor
#define TRIG_PIN 9
#define ECHO_PIN 9

// L298N Motor Driver
#define ENA 3
#define IN1 2
#define IN2 11
#define ENB 10
#define IN3 12
#define IN4 13

// Servos
#define SERVO_BASE A4
#define SERVO_GRIPPER A5

// ==================== GLOBAL VARIABLES ====================

Servo baseServo;
Servo gripperServo;

// Color sensor readings
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Debug settings
bool DEBUG_COLOR = true;
unsigned long lastColorPrint = 0;
const unsigned long COLOR_PRINT_INTERVAL = 300;

// Motor speeds
int motorSpeed = 150;
int turnSpeed = 100;

// Ultrasonic
long duration;
int distance;

// ==================== STATE MACHINE ====================

enum RobotState {
  FOLLOWING_LINE,
  OBJECT_PICKUP,
  OBJECT_PLACE,
  AVOIDING_OBSTACLE
};

RobotState currentState = FOLLOWING_LINE;

// ==================== COLOR RANGES (CALIBRATE!) ====================

struct ColorRange {
  int rMin, rMax;
  int gMin, gMax;
  int bMin, bMax;
};

ColorRange redColor   = {20, 50, 80, 150, 80, 150};
ColorRange greenColor = {80, 150, 20, 50, 80, 150};
ColorRange blueColor  = {80, 150, 80, 150, 20, 50};

// ==================== ARM POSITIONS ====================

struct ArmPosition {
  int base;
  int gripper;
};

ArmPosition homePos       = {90, 90};
ArmPosition pickupPos     = {90, 10};
ArmPosition holdPos       = {90, 170};
ArmPosition placeLeftPos  = {45, 170};
ArmPosition releasePos    = {90, 10};

// ==================== SETUP ====================

void setup() {
  Serial.begin(9600);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(OE, OUTPUT);

  digitalWrite(OE, LOW);      // Enable color sensor
  digitalWrite(S0, HIGH);     // 20% scaling
  digitalWrite(S1, LOW);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  baseServo.attach(SERVO_BASE);
  gripperServo.attach(SERVO_GRIPPER);

  moveArmToPosition(homePos);

  Serial.println("Robot Initialized");
  Serial.println("Printing COLOR SENSOR OUTPUT...");
  delay(1500);
}

// ==================== MAIN LOOP ====================

void loop() {
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);

  distance = getDistance();
  String color = readColor();

  switch (currentState) {

    case FOLLOWING_LINE:
      if (distance < 15 && distance > 0) {
        currentState = AVOIDING_OBSTACLE;
        break;
      }

      if (color == "RED") {
        stopMotors();
        currentState = OBJECT_PICKUP;
        break;
      }

      if (color == "GREEN") {
        stopMotors();
        currentState = OBJECT_PLACE;
        break;
      }

      followLine(irLeft, irRight);
      break;

    case AVOIDING_OBSTACLE:
      avoidObstacle();
      currentState = FOLLOWING_LINE;
      break;

    case OBJECT_PICKUP:
      performPickup();
      currentState = FOLLOWING_LINE;
      break;

    case OBJECT_PLACE:
      performPlace();
      currentState = FOLLOWING_LINE;
      break;
  }

  delay(40);
}

// ==================== MOTOR FUNCTIONS ====================

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
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
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ==================== LINE FOLLOWING ====================

void followLine(int left, int right) {
  if (left == 0 && right == 0) moveForward();
  else if (left == 0 && right == 1) {
    analogWrite(ENA, motorSpeed - 50);
    analogWrite(ENB, motorSpeed);
  }
  else if (left == 1 && right == 0) {
    analogWrite(ENA, motorSpeed);
    analogWrite(ENB, motorSpeed - 50);
  }
  else {
    analogWrite(ENA, motorSpeed / 2);
    analogWrite(ENB, motorSpeed / 2);
  }
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

// ==================== COLOR SENSOR ====================

String readColor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  delay(5);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  delay(5);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  if (DEBUG_COLOR && millis() - lastColorPrint > COLOR_PRINT_INTERVAL) {
    lastColorPrint = millis();
    Serial.print("R: "); Serial.print(redFrequency);
    Serial.print(" | G: "); Serial.print(greenFrequency);
    Serial.print(" | B: "); Serial.print(blueFrequency);
    Serial.print(" => ");
  }

  if (isInRange(redFrequency, greenFrequency, blueFrequency, redColor)) {
    if (DEBUG_COLOR) Serial.println("RED");
    return "RED";
  }
  if (isInRange(redFrequency, greenFrequency, blueFrequency, greenColor)) {
    if (DEBUG_COLOR) Serial.println("GREEN");
    return "GREEN";
  }
  if (isInRange(redFrequency, greenFrequency, blueFrequency, blueColor)) {
    if (DEBUG_COLOR) Serial.println("BLUE");
    return "BLUE";
  }

  if (DEBUG_COLOR) Serial.println("NONE");
  return "NONE";
}

bool isInRange(int r, int g, int b, ColorRange c) {
  return (r >= c.rMin && r <= c.rMax &&
          g >= c.gMin && g <= c.gMax &&
          b >= c.bMin && b <= c.bMax);
}

// ==================== ARM ====================

void moveArmToPosition(ArmPosition p) {
  baseServo.write(p.base);
  delay(300);
  gripperServo.write(p.gripper);
  delay(300);
}

void performPickup() {
  moveArmToPosition(pickupPos);
  moveArmToPosition(holdPos);
  moveArmToPosition(homePos);
}

void performPlace() {
  moveArmToPosition(placeLeftPos);
  moveArmToPosition(releasePos);
  moveArmToPosition(homePos);
}

void avoidObstacle() {
  stopMotors();
  delay(300);
  moveBackward();
  delay(500);
  turnRight();
  delay(700);
}
