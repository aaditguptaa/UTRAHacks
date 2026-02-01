#include <Servo.h>
long duration distance;

// define pins for sensor
const int trigPin = ;
const int echoPin = ;

// define pins for motors
const int mtl1Pin = 2;
const int mtl2pin = 11;
const int mtr1pin = 12;
const int mtr2pin = 13;

// Cheatsheet for motors params
// Definition on pinA: '1st pin'
// Definition on pinB : '2nd pin'
// 1/A/Front:LOW 2/B/Back:HIGH -> Rotate Forward
// 1/A/Front:High 2/B/Back:LOW -> Rotate Backward
// 1/A/Front:LOW 2/B/Back:LOW -> Stop/Brake

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serail.begin(115200)
}

void loop() {
  // reset and stabilize everything
  digitalWrite(trigPin, LOW);
  // delay 2 ms 
  delayMicroseconds(2);

  // trigger pulse
  digitalWrite(trigPin, HIGH)
  // delay 10 ms
  delayMicroseconds(10);
  // end pulse
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.prrint(distance);
  delay(10);
  // if no obstacle or obstacle is far
  if (distance > 15)
  {
    digitalWrite(mtl2pin, HIGH);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, HIGH);
    digitalWrite(mtr1pin, LOW);
  }
  else if ((distance <10)&&(distance > 0))
  {
    // Stop
    digitalWrite(mtl2pin, LOW);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, LOW);
    digitalWrite(mtr1pin LOW);
    delay(100);

    delay(500);
    Myservo.write(180);
    delay(500);
    Myservo.write(90);
    delay(500);

    // Move Backward
    digitalWrite(mtl2pin, LOW);
    digitalWrite(mtl1pin, HIGH);
    digitalWrite(mtr2pin, LOW);
    digitalWrite(mtr2pin, HIGH);
    delay(500); // time needs test
    // Stop
    digitalWrite(mtl2pin, LOW);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, LOW);
    digitalWrite(mtr1pin, LOW);
    delay(100); // time needs test
    // Turn Left
    digitalWrite(mtl2pin, LOW);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, HIGH);
    digitalWrite(mtr1pin, LOW);
    delay(500); // time needs test
    // Move Forward
    digitalWrite(mtl2pin, HIGH);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, HIGH;
    digitalWrite(mtr1pin, LOW);
    delay(500); // time needs test
    // Turn to Straight
    digitalWrite(mtl2pin, HIGH);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, LOW);
    digitalWrite(mtr1pin, LOW);
    delay(500); // time needs test
    // Turn Right
    digitalWrite(mtl2pin, HIGH);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, LOW);
    digitalWrite(mtr1pin, LOW);
    delay(500) // time needs test
    // Move Forward
    digitalWrite(mtl2pin, HIGH);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, HIGH;
    digitalWrite(mtr1pin, LOW);
    delay(500); // time needs test
    // Turn to Straight
    digitalWrite(mtl2pin, LOW);
    digitalWrite(mtl1pin, LOW);
    digitalWrite(mtr2pin, HIGH);
    digitalWrite(mtr1pin, LOW);
    delay(500) // time needs test
  }
}
