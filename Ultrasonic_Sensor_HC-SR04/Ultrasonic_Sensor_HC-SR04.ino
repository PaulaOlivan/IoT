const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, distance;
float wall;
float range = 5.0;

void setup() {
  // put your setup code here, to run once:
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Distance to the wall
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  Serial.print("Distance to the wall = ");
  Serial.print(wall);
  Serial.println(" cm");
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  distance = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  if (distance > (wall+range) || distance < (wall-range)){
    Serial.print("Intruder detected at = ");
    Serial.print(distance);
    Serial.println(" cm");
    wall = distance;
    Serial.print("New wall measure = ");
    Serial.print(wall);
    Serial.println(" cm");
  }

  delay(3000);
}
