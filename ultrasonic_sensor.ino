long duration;
int distance;

const int trigPin = ;
const int echoPin = ;

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
}