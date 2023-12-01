const int buzzer = 4; //buzzer to arduino digital pin 0
bool intruder = false;

void setup(){
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 0 as an output
  digitalWrite(buzzer, HIGH);
}

void loop(){
  if (intruder){
    activeAlarm();
  }
  delay(5000);
  intruder = true;
}

// intruderLevel is an integer between 1 and 3
// which indicates different infractions
void activeAlarm(){    
  //tone(buzzer, 100); // Frecuencia para el tono más alto
  digitalWrite(buzzer, LOW);
  delay(1000);        // ...for 1 sec
  //noTone(buzzer);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  intruder = false;
}


