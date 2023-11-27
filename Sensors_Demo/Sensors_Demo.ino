
// Variables for ultrasonic sensor
const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, US_distance;
float US_wall;
float US_range = 5.0;

// Variables for Infrared sensor 
const int IF_Pin = A0; // Analog pin connected to the sensor

// Variables for Keypad
const int keyVals[16] = {809, 827, 924, 0, 
                         166, 219, 503, 0,
                         269, 313, 562, 0,
                         0,   647, 0,   994};
const char keys[16] = {'1', '2', '3', 'A', 
                       '4', '5', '6', 'B',
                       '7', '8', '9', 'C',
                       '*', '0', '#', 'D'};
int KEY_range = 5; // Tolerancia arriba o abajo del valor numérico
int lastButton = -1; // Último botón presionado
unsigned long lastPressTime = 0; // Último momento en el que se presionó un botón
bool buttonReleased = true; // Indica si el botón fue liberado

// Variables for Buzzer
const int buzzer = 4; //buzzer to arduino digital pin 0
int intruder = 0;

void setup() {
  // Setup for all the sensors and actuators
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Distance to the wall for the Ultrasonic sensor
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  Serial.print("Distance to the wall = ");
  Serial.print(wall);
  Serial.println(" cm");
  delay(500);

  // Setup for the Buzzer
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 0 as an output
  digitalWrite(buzzer, HIGH); // Deactivated the buzzer pin

}

void loop() {
  // Ultrasonic Sensor Demo
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  distance = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  if (distance > (wall+range) || distance < (wall-range)){
    Serial.print("Intruder detected at ");
    Serial.print(distance);0
    Serial.println(" cm");
    wall = distance;
    Serial.print("New wall measure: "); // Update the wall measure
    Serial.print(wall);
    Serial.println(" cm");
  }

  delay(3000);


  // Infrared Sensore Demo
  int IF_value = analogRead(IF_Pin); // Read the analog value from the sensor
  
  // Convert the analog value to distance (in centimeters)
  float IF_voltage = IF_value * (3.3 / 1023.0); // Convert the sensor reading to voltage
  float IF_distance = 13.0 * pow(IF_voltage, -1.0); // Formula to convert voltage to distance
  
  Serial.print("Distance: ");
  Serial.print(IF_distance);
  Serial.println(" cm");

  delay(500); // Wait for a short duration before taking the next reading


  //Keypad
    unsigned long currentTime = millis(); // Obtener el tiempo actual

  int keyIn = analogRead(A1); // Read the keypad input
  int detectedButton = detectButton(keyIn); // Find with button is detected

  if (detectedButton != -1) {
    if (buttonReleased) {
      Serial.print("New key pulsed: ");
      Serial.println(keys[detectedButton]);
      lastButton = detectedButton;
      lastPressTime = currentTime;
      buttonReleased = false;
      delay(100);
    }
  } 
  
  else if (detectedButton == -1) {
    buttonReleased = true;
  }
  delay (500); 


  // Buzzer
  Serial.print ("Intruder value: ");
  Serial.println(intruder);
  if (intruder == 1){
    Serial.println("Intruder detected");
    activeAlarm();
  }
}

// Function for the Keypad Demo
int detectButton(int inputValue) {
  for (int i = 0; i < 16; i++) {
    if (inputValue >= keyVals[i] - range && inputValue <= keyVals[i] + range) {
      return i; // Return the pulsed key
      }
  }
  return -1; // No key pulsed
}

// Function for the Buzzer activation
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