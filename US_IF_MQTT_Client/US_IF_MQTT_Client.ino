/**
  First install appropriate libraries library in IDE 
**/

// turn on debug messages
// #define VERBOSE //Uncomment for full information about camera detection
#include <WiFi.h>
#include <PubSubClient.h>

char ssid[] = "COSMOTE-166950";
char pass[] = "52752766413729464236";
const char* mqttServer = "test.mosquitto.org";  // Uses the Mosquitto public broker
const int mqttPort = 1883;
const char* mqttClientId = "mkr1010-client";
const char* ultrasonicSensorTopic = "motionDetection/ultrasonicSensor";
const char* infraredSensorTopic = "motionDetection/infraredSensor";
const char* keypadTopic = "motionDetection/keypad";

WiFiClient espClient;
PubSubClient client(espClient);

// Variables for infrared sensor
const int sensorPin = A0; // Analog pin connected to the sensor
float if_light = 0.0;

// Variables for ultrasonic sensor
const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, us_distance;
float us_wall;
float us_range = 5.0;

// Variables for keypad
const int keyVals[16] = {809, 827, 924, 0, 
                         166, 219, 503, 0,
                         269, 313, 562, 0,
                         0,   647, 0,   994};
const char keys[16] = {'1', '2', '3', 'A', 
                       '4', '5', '6', 'B',
                       '7', '8', '9', 'C',
                       '*', '0', '#', 'D'};
int key_range = 5;
int lastButton = -1;
unsigned long lastPressTime = 0;
bool buttonReleased = true;


void setup() {

  Serial.begin(115200);//Initialize serial connection
  delay(5000);//Time for user to open serial monitor on Arduino IDE

  //debug("INFO", "Init");

  /** Setting Ultrasonic sensor **/
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  us_wall = us_measure();

  /** Setting Infrared sensor **/
  if_light = if_measure();

  /** Setting Keypad **/

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // MQTT setup
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  reconnect();

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  // Capture and send data if motion is detected on ultrasonic sensor
  if (ultrasonicMovement()){
    client.publish(ultrasonicSensorTopic, "1");
  }

  // Capture and send data if motion is detected on infrared sensor
  if (infraredMovement()){
    client.publish(infraredSensorTopic, "1");
  }

  // Capture and send data if a key is pressed
  char key = keyPressed();
  if (key != -1){
    client.publish(keypadTopic, key);
  }

  Serial.println("| MICROWAVE |    CAM    | ULTRASONIC|  INFRARED |   KEYPAD  |");
  Serial.println("|     " + String(microwave) + "     |     " + String(cam) + "     |     " + String(ultrasonic) + "     |     " + String(infrared) + "    |     " + String(keypad) +"     |");

  microwave = 0;
  cam = 0;
  ultrasonic = 0;
  infrared = 0;
  keypad = 0;

  delay(100);
}

float us_measure(){

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  us_wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  
  Serial.print("Distance to the wall = ");
  Serial.print(us_wall);
  Serial.println(" cm");
  delay(500);
}

bool ultrasonicMovement() {
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  pulse_width = pulseIn(echoPin, HIGH);
  us_distance = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  
  if (us_distance > (us_wall+us_range) || us_distance < (us_wall-us_range)){
    Serial.print("Intruder detected at = ");
    Serial.print(distance);0
    Serial.println(" cm");
    return true;
  }
  else {
    return false;
  }
}

float if_measure(){
  float sum = 0.0;
  for (i=0; i<5; i++){
    int sensorValue = analogRead(sensorPin); // Read the analog value from the sensor
    float voltage = sensorValue * (3.3 / 1023.0); // Convert the sensor reading to voltage
    float distance = 13.0 * pow(voltage, -1.0); // Formula to convert voltage to distance
    sum = sum + distance
  }
  sum = sum / 5;
  return sum;
}

bool infraredMovement() {
  int sensorValue = analogRead(sensorPin); // Read the analog value from the sensor
  
  // Convert the analog value to distance (in centimeters)
  float voltage = sensorValue * (3.3 / 1023.0); // Convert the sensor reading to voltage
  float distance = 13.0 * pow(voltage, -1.0); // Formula to convert voltage to distance
  
  if (distance>(if_light+50) || distance<(if_light-50)){
    return true;
  }
  else{
    return false;
  }
}

int keyPressed(){
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

  return detectedButton;
}

int detectButton(int inputValue) {
  for (int i = 0; i < 16; i++) {
    if (inputValue >= keyVals[i] - key_range && inputValue <= keyVals[i] + key_range) {
      return i;
      }
  }
  return -1;
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(mqttClientId)) {
      Serial.println("Connected to MQTT");

      // Subscribe to specified topics
      client.subscribe(microwaveSensorTopic);
      client.subscribe(esp32camTopic);
      client.subscribe(ultrasonicSensorTopic);
      client.subscribe(infraredSensorTopic);
      client.subscribe(keypadTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {

  if (String(topic).equals(String(microwaveSensorTopic))) {
    microwave = 1;

  } else if (String(topic).equals(String(esp32camTopic))) {
    cam = 1;
  } else if (String(topic).equals(String(ultrasonicSensorTopic))) {
    ultrasonic = 1;
  } else if (String(topic).equals(String(infraredSensorTopic))) {
    infrared = 1;
  } else if (String(topic).equals(String(keypadTopic))) {
    keypad = 1;
  }

}