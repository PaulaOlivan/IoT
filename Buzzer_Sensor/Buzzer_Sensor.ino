const int buzzer = 4; //buzzer to arduino digital pin 0
int intruder = 0;

void setup(){
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 0 as an output
  digitalWrite(buzzer, HIGH);
}

void loop(){
  Serial.print ("Intruder value: ");
  Serial.println(intruder);
  if (intruder == 1){
    Serial.println("Intruder detected");
    activeAlarm();
  }
}

// intruderLevel is an integer between 1 and 3
// which indicates different infractions
void activeAlarm(int intruderLevel){
    tone(buzzer, 10);   // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzer);     // reset to get a new sound...
    delay(1000);        // ...for 1sec
    
    if (intruderLevel == 1) {
      Serial.println("Level 1 intruder detected");
      tone(buzzerPin, 10); // Frecuencia para el tono más bajo
      delay(1000);        // ...for 1 sec
      noTone(buzzer);     // reset to get a new sound...
      delay(1000);        // ...for 1sec
    } 
    else if (intruderLevel == 2) {
      Serial.println("Level 2 intruder detected");
      tone(buzzerPin, 100); // Frecuencia para el tono medio
      delay(1000);        // ...for 1 sec
      noTone(buzzer);     // reset to get a new sound...
      delay(1000);        // ...for 1sec
    } 
    else if (intruderLevel == 3) {
      Serial.println("Level 3 intruder detected");
      tone(buzzerPin, 500); // Frecuencia para el tono más alto
      delay(1000);        // ...for 1 sec
      noTone(buzzer);     // reset to get a new sound...
      delay(1000);        // ...for 1sec
    }
    intruder = 0; // Restablecer la variable intruso a 0
}


