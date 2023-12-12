/**
  First install appropriate libraries library in IDE 
**/

// turn on debug messages
// #define VERBOSE //Uncomment for full information about camera detection
#include <WiFi.h>
#include <PubSubClient.h>

char ssid[] = "dlink";
//char ssid[] = "COSMOTE-166950";
char pass[] = "";
//char pass[] = "52752766413729464236";
const char* mqttServer = "test.mosquitto.org";  // Uses the Mosquitto public broker
const int mqttPort = 1883;
const char* mqttClientId = "mkr1010-client";

const char* microwaveSensorTopic = "motionDetection/microwaveSensor";
const char* esp32camTopic = "motionDetection/esp32cam";
const char* ultrasonicSensorTopic = "motionDetection/ultrasonicSensor";
const char* infraredSensorTopic = "motionDetection/infraredSensor";
const char* keypadTopic = "motionDetection/keypad";

WiFiClient espClient;
PubSubClient client(espClient);

int microwave = 0;
int cam = 0;
int ultrasonic = 0;

/******* Variables for ultrasonic sensor *******/
const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, us_distance;
float us_wall;
float us_range = 10.0;


/******* Variables for the keypad *******/
const int keyVals[16] = {809, 827, 924, 0, 
                         166, 219, 503, 0,
                         269, 313, 562, 0,
                         0,   647, 0,   994};
const char keys[16] = {'1', '2', '3', 'A', 
                       '4', '5', '6', 'B',
                       '7', '8', '9', 'C',
                       '*', '0', '#', 'D'};
int key_range = 8;
int lastButton = -1;
unsigned long lastPressTime = 0;
bool buttonReleased = true;

int buzzerPin = 4;

void setup() {

  Serial.begin(115200);//Initialize serial connection
  delay(5000);//Time for user to open serial monitor on Arduino IDE

  //debug("INFO", "Init");
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);

  /** Setting Ultrasonic sensor **/
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  us_wall = us_measure();

  // Connect to Wi-Fi
  WiFi.begin(ssid, pass);
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
    String temp = String(us_distance, 2); // 2 say how many decimals we want
    client.publish(ultrasonicSensorTopic, temp.c_str());
  }

  Serial.println("| MICROWAVE |    CAM    | ULTRASONIC|   KEYPAD  |");
  Serial.print("|     " + String(microwave) + "     |     " + String(cam));
  Serial.println("     |     " + String(us_distance) + "     |     ");

  if (microwave == 1){
    intruderDetected();
  }

  if (cam == 1){
    intruderDetected();
  }

  microwave = 0;
  cam = 0;
  ultrasonic = 0;

  delay(500);
}

void intruderDetected(){
  for (int i=0; i<10; i++){ //10 normal tones
      digitalWrite(buzzerPin, LOW);
      delay(150);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
  }
}

float us_measure(){
  Serial.println("Initial measure for the Ultrasonic Sensor \n\n");
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  float wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  
  Serial.print("Distance to the wall = ");
  Serial.print(wall);
  Serial.println(" cm");

  return wall;
}

bool ultrasonicMovement() {
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  pulse_width = pulseIn(echoPin, HIGH);
  us_distance = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  
  if (us_distance > (us_wall+us_range) || us_distance < (us_wall-us_range)){
    Serial.print("Intruder detected at = ");
    Serial.print(us_distance);
    Serial.println(" cm");
    us_wall = us_distance;
    Serial.print("New wall measure at = ");
    Serial. print(us_wall);
    Serial.println(" cm");
    return true;
  }
  else {
    return false;
  }
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
  }
}

