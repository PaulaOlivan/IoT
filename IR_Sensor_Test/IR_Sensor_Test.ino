// Arduino Code to measure distance with a Sharp GP2D12 sensor
// www.swanrobotics.com

const int infraredPin = A5; // Sensor is connected to the analog A0
int IR_distance; //Sensor result
int IR_range = 500; //Diference between distance change
int IR_wall; //Original distance

const int buzzerPin = 7;

float fltSensorCalc = 0; //Calculated value

void setup(){
  Serial.begin(9600); // Setup communication with computer to present results serial monitor
  startInfrared(); 
}

void loop(){
  // read the value from the ir sensor
  readInfrared();
  delay(1000); //Wait
}

void startInfrared(){
  Serial.print("Initializing the infrared sensor ");
  IR_wall = analogRead(infraredPin);
  Serial.print("Infrared value ");
  Serial.println(IR_wall);
}

void readInfrared(){
  IR_distance = analogRead(infraredPin); //Get sensor value
  if (IR_distance > (IR_wall + IR_range) || IR_distance < (IR_wall - IR_range)){
    Serial.print("Intruder detected on the infrared sensor at = ");
    Serial.println(IR_distance);
    IR_wall = IR_distance;
    Serial.print("New wall measure = "); // Update the wall measure
    Serial.println(IR_wall);
    //intruderDetected();
    // Send the value to Andreas board
    /*String temp = String(IR_distance, 2); // 2 say how many decimals we want
    client.publish(infraredSensorTopic, temp.c_str());*/
  }
  else{
    Serial.println("All good");
  }
}