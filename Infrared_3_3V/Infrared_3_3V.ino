const int if_Pin = A0; // Analog pin connected to the sensor
float light;
float if_range = 50.0;

void setup() {
  Serial.begin(9600);
  light = lights_measure();

}

void loop() {
  int sensorValue = analogRead(if_Pin); // Read the analog value from the sensor
  
  // Convert the analog value to distance (in centimeters)
  float voltage = sensorValue * (3.3 / 1023.0); // Convert the sensor reading to voltage
  float distance = 13.0 * pow(voltage, -1.0); // Formula to convert voltage to distance
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > (light+if_range) || distance < (light-if_range)){
    Serial.print("Intruder detected at: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  else{
    Serial.print("Normal measure: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(1000); // Wait for a short duration before taking the next reading
}


float lights_measure(){

  int sum = 0;
  for (int i=0; i<5; i++){
    int value = analogRead(if_Pin);

    float voltage = value * (3.3 / 1023.0);
    float distance = 13.0 * pow(voltage, -1.0);

    sum = sum + distance;
  }
  sum = sum / 5;
  return sum;

}
