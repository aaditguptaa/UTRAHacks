#include <Servo.h>

/* ===================== SERVO ===================== */
Servo liftArm;

/* ===================== PIN ===================== */
const int ARM_PIN = 9;

/* ===================== POSITIONS (TUNE THESE) ===================== */
const int ARM_UP = 120;     // box lifted
const int ARM_DOWN = 45;    // arm under box

/* ===================== STATES ===================== */
enum ArmState {
  ARM_RAISED,
  ARM_LOWERED
};

ArmState armState = ARM_RAISED;

/* ===================== TIMING ===================== */
unsigned long stateStart = 0;
const int MOVE_TIME = 600;

/* ===================== SETUP ===================== */
void setup() {
  liftArm.attach(ARM_PIN);
  liftArm.write(ARM_UP);
  stateStart = millis();
}

/* ===================== LOOP ===================== */
void loop() {

  switch (armState) {

    case ARM_RAISED:
      // default driving state
      // call lowerArm() when ready to pick up or drop
      break;

    case ARM_LOWERED:
      // arm is down under box
      // call raiseArm() after driving forward
      break;
  }

  // Example demo automation (REMOVE for competition)
  demoSequence();
}

/* ===================== CONTROL FUNCTIONS ===================== */

void lowerArm() {
  liftArm.write(ARM_DOWN);
  armState = ARM_LOWERED;
  stateStart = millis();
}

void raiseArm() {
  liftArm.write(ARM_UP);
  armState = ARM_RAISED;
  stateStart = millis();
}

/* ===================== DEMO SEQUENCE ===================== */
/* This just shows the motion without driving */
void demoSequence() {
  static int step = 0;

  if (millis() - stateStart < MOVE_TIME) return;

  if (step == 0) {
    lowerArm();  // prepare to load
    step++;
  } 
  else if (step == 1) {
    raiseArm();  // lift box
    step++;
  }
  else if (step == 2) {
    lowerArm();  // drop box
    step = 0;
  }
}
