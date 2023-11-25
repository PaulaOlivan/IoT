const int sensorPin = A0; // Analog pin connected to the sensor

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Read the analog value from the sensor
  
  // Convert the analog value to distance (in centimeters)
  float voltage = sensorValue * (3.3 / 1023.0); // Convert the sensor reading to voltage
  float distance = 13.0 * pow(voltage, -1.0); // Formula to convert voltage to distance
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500); // Wait for a short duration before taking the next reading
}
