#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <WiFi.h>
#include <Keypad.h>
#include <PubSubClient.h>

#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

/******** Variables for the IoT Cloud conection ********/
const char THING_ID[] = "Home_Alarm_v2";

String activation_Way;
bool alarm;
int tries;
bool intruder;
bool cam_Detection;
bool infrared_Detection;
bool microwave_Detection;
bool ultrasonic_Detection;

/******** Variables for the MQTT server ********/
//char ssid[] = "COSMOTE-166950";
//char pass[] = "52752766413729464236";
char ssid[] = "dlink";
char pass[] = "";
const char* mqttServer = "test.mosquitto.org";  // Uses the Mosquitto public broker
const int mqttPort = 1883;
const char* mqttClientId = "mkr1010-client";

const char* microwaveSensorTopic = "motionDetection/microwaveSensor";
const char* esp32camTopic = "motionDetection/esp32cam";
const char* ultrasonicSensorTopic = "motionDetection/ultrasonicSensor";
const char* infraredSensorTopic = "motionDetection/infraredSensor";

WiFiClient espClient;
PubSubClient client(espClient);

int microwave = 0;
int cam = 0;
int ultrasonic = 0;
int infrared = 0;

/******** Variables for the RFID sensor ********/
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
uint8_t storedUID[] = {0xC7, 0xFE, 0x66, 0x39};
uint8_t storedUIDLength = sizeof(storedUID);
uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID
bool cardDetected = false;

/******** Variables for ultrasonic sensor ********/
const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, US_distance;
float US_wall;
float US_range = 10.0;

/******** Variables for infrared sensor ********/
const int infraredPin = A5; // Sensor is connected to the analog A0
int IR_distance; //Sensor result
int IR_range = 75; //Diference between distance change
int IR_wall; //Original distance

/******** Variables for the keypad ********/
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

const char keys[ROWS][COLS] = {'1', '2', '3', 'A', 
                               '4', '5', '6', 'B',
                               '7', '8', '9', 'C',
                               '*', '0', '#', 'D'};

byte rowPins[ROWS] = {A0, A1, A2, A3}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {2, 3, 4, 5}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS); //Create an object of keypad

const int passwdSize = 4;
char inputPassword[passwdSize] = {'\0', '\0' , '\0', '\0'}; // Array to store the entered password
const char storedPassword[passwdSize] = {'1', '9', '9', '1'}; // Stored password
static int pos = 0; 

/******** Variables for the buzzer and LED ********/
const int buzzerPin = 7; //buzzer to arduino digital pin 6
const int ledPin = A6; 

/*****************************************/
void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  delay(1500);
  while (!Serial) delay(10);
  keypad.addEventListener(keypadEvent); //add an event listener for this keypad
  alarm  = false; // false = non active || true = active
  intruder = false; // false = any intruder || true = a intruder
  tries = 0;

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ssid, pass);
  
  // Define the Thing Properties
  initProperties();
  
  
  /* The following function allows you to obtain more information related to the state 
  of network and IoT Cloud connection and errors the higher number the more granular
  information you’ll get. The default is 0 (only errors). Maximum is 4 */

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  // Setup for the alarm's sound system
  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin D6 as an output
  digitalWrite(buzzerPin, HIGH); // Deactivated the buzzer pin
  pinMode(ledPin, OUTPUT); // Set led - pin A5 as an output
  digitalWrite(ledPin, LOW);
  
  //Setup for the Ultrasonic Sensor
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
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

  /*nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);*/
}

/*****************************************/
void loop() {
  ArduinoCloud.update();
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (alarm == false){ // Alarm deactivated only reads the keypad and the sensor
    readKeypad(0); // 0 If the alarm is deactivated
    if (alarm == false){ //The alarm continues deactivated so read the RFID
      /*cardDetected = readRFID();
      if (cardDetected){
        if (uidLength == storedUIDLength && memcmp(uid, storedUID, uidLength) == 0){
          activateAlarm();
          Serial.println("Card validated, alarm activated");
          activation_Way = "Activated via RFID card";
        }
        else{ //The card set closer is not correct
          alarmIncorrectTry();
          Serial.println("Incorrect card, try again with the correct card");
        }
      }*/
    }
  }
 
  else{ // The alarm is active
    readUltrasonic();
    readInfrared();
    // Lets see if the alarm turn off
    readKeypad(1); // 1 if the alarm is actived
    if (alarm == true){ //The alarm continues activated so read the RFID
      /*cardDetected = readRFID();
      if (cardDetected){
        if (uidLength == storedUIDLength && memcmp(uid, storedUID, uidLength) == 0){
          Serial.println("Card validated, alarm desactivated");
          activation_Way = "Deactivated via RFID card";
          desactivateAlarm();
        }
        else{ //The card set closer is not correct
          Serial.println("Incorrect card, try again with the correct card");
          alarmIncorrectTry();
        }
      }*/
    }  
  }

  ArduinoCloud.update();
}

/****** Function for inicialize Arduino IoT Cloud ******/
void initProperties() {
  ArduinoCloud.setThingId(THING_ID);
  ArduinoCloud.addProperty(activation_Way, READ, NULL);
  ArduinoCloud.addProperty(alarm, READWRITE, ON_CHANGE, onAlarmChange);
  ArduinoCloud.addProperty(tries, READ, NULL);
  ArduinoCloud.addProperty(intruder, READ, NULL);
  ArduinoCloud.addProperty(cam_Detection, READ, NULL);
  ArduinoCloud.addProperty(infrared_Detection, READ, NULL);
  ArduinoCloud.addProperty(microwave_Detection, READ, NULL);
  ArduinoCloud.addProperty(ultrasonic_Detection, READ, NULL);
}

/****** Functions for the IoT Cloud dashboard ******/
void onAlarmChange() {
  if (alarm) {
    activation_Way = "Activated via dashboard";
    activateAlarm();
  } 
  else {
    desactivateAlarm();
    activation_Way = "Deactivated via dashboard";
  }

  ArduinoCloud.update();
}

/****** Functions related to turning the alarm on and off ******/ 
void readKeypad(int alarmValue) {
  char key = keypad.getKey();// Read the key
  // Print if key pressed
  if (key){
    digitalWrite(buzzerPin, LOW); // Activate the passive buzzer
    delay(250); // Wait for a short duration
    digitalWrite(buzzerPin, HIGH); // Turn off the buzzer

    // Store the detected button in the password array
    if (key != 'A'){
      storePassword(key);
    }
    else{
      comparePasswords(alarmValue);
    }
  }

  if (key == 'D'){
    resetPassword();
  }
} 

bool readRFID() {
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100); //Dont move the timeout
  if (success) {
    return true;
  }
  else{
    return false;
  }
}

/****** Functions related with the alarm and buzzer ******/
void activateAlarm(){
  alarm = true;
  tries = 0; //Reset tries if we active the alarm
  digitalWrite(buzzerPin, LOW); // 1 large tone
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(ledPin, LOW);
  delay(100);
  for (int i=0; i<2; i++){ //2 normal tones
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  
  intruder = false;

  // Read the Ultrasonic sensor to measure the initial wall
  startUltrasonic();
  // Read the Infrared sensor to measure the initial wall
  startInfrared();
  
}

void desactivateAlarm(){
  alarm = false;
  tries = 0;
  for (int i=0; i<2; i++){ //2 normal tones
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  digitalWrite(buzzerPin, LOW); // 1 large tone
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, HIGH);
  digitalWrite(ledPin, LOW);
  delay(100);
  intruder = false;
}

void alarmIncorrectTry(){
  tries++;
  if (tries >= 3){ //Maximum number of attempts reached
    intruderDetected();
  }
  else{
    digitalWrite(buzzerPin, LOW); //turn on the buzzer
    digitalWrite(ledPin, HIGH); // turn on the led
    delay(1500);
    digitalWrite(buzzerPin, HIGH); //turn off the buzzer
    digitalWrite(ledPin, LOW); // turn off the led
  }
}

void intruderDetected(){
  intruder = true;
  for (int i=0; i<10; i++){ //10 normal tones
      digitalWrite(buzzerPin, LOW);
      digitalWrite(ledPin, HIGH);
      delay(150);
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(ledPin, LOW);
      delay(100);
  }
}

/********* Functions for detect intruder *********/
void startUltrasonic(){
  Serial.println("Initializing the ultrasonic sensor");
  digitalWrite(triggerPin, HIGH);   // Distance to the wall
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  US_wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  Serial.print("Ultrasonic distance ");
  Serial.print(US_wall);
  Serial.println(" cm");
}

void startInfrared(){
  Serial.print("Initializing the infrared sensor ");
  IR_wall = analogRead(infraredPin);
  Serial.print("Infrared value ");
  Serial.println(IR_wall);
}

void readUltrasonic(){
  digitalWrite(triggerPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  US_distance = (pulse_width*.0343)/2;

  if (US_distance > (US_wall+US_range) || US_distance < (US_wall-US_range)){
    Serial.print("Intruder detected on the ultrasonic sensor at = ");
    Serial.print(US_distance);
    Serial.println(" cm");
    US_wall = US_distance;
    Serial.print("New wall measure = "); // Update the wall measure
    Serial.print(US_wall);
    Serial.println(" cm");
    intruderDetected();
    // Send the value to Andreas board
    String temp = String(US_distance, 2); // 2 say how many decimals we want
    client.publish(ultrasonicSensorTopic, temp.c_str());
  }

  microwave = 0;
  microwave_Detection = false;
  cam = 0;
  cam_Detection = false;
  ultrasonic = 0;
  ultrasonic_Detection = false;
  infrared = 0;
  infrared_Detection = false;
}

void readInfrared(){
  IR_distance = analogRead(infraredPin); //Get sensor value
  if (IR_distance > (IR_wall+IR_range) || IR_distance < (IR_wall-IR_range)){
    Serial.print("Intruder detected on the infrared sensor at = ");
    Serial.println(IR_distance);
    IR_wall = IR_distance;
    Serial.print("New wall measure = "); // Update the wall measure
    Serial.println(IR_wall);
    intruderDetected();
    // Send the value to Andreas board
    String temp = String(IR_distance, 2); // 2 say how many decimals we want
    client.publish(infraredSensorTopic, temp.c_str());
  }
  
  microwave = 0;
  microwave_Detection = false;
  cam = 0;
  cam_Detection = false;
  ultrasonic = 0;
  ultrasonic_Detection = false;
  infrared = 0;
  infrared_Detection = false;
}

/********* Functions for the Keypad *********/
void storePassword(char key) {
  /*if (pos < passwdSize) {
    inputPassword[pos++] = key;
  }*/
  
  inputPassword[0] = inputPassword[1];
  inputPassword[1] = inputPassword[2];
  inputPassword[2] = inputPassword[3];
  inputPassword[3] = key;
  
}

void comparePasswords(int alarmValue) {
  bool pass_equal = true;
  for (int i = 0; i < passwdSize; i++) {
    if (inputPassword[i] != storedPassword[i]) {
        pass_equal = false;
    }
  }
  if (pass_equal && alarmValue == 0) {
    Serial.println("Password validated, alarm activated");
    activation_Way = "Activated via keypad password";
    activateAlarm();
  }
  else if (pass_equal && alarmValue == 1){
    Serial.println("Password validated, alarm desactivated");
    activation_Way = "Deactivated via keypad password";
    desactivateAlarm();
  }
  else {
    Serial.println("Incorrect password, try again");
    alarmIncorrectTry();
  }
  resetPassword();
}

void resetPassword() {
  for (int i = 0; i < passwdSize; i++) {
    inputPassword[i] = '\0';
  }
  Serial.println("Restarting the input password");
  pos = 0; // Restart the index for store the password correctly
}

void keypadEvent(KeypadEvent eKey){
  switch (keypad.getState()){
  case PRESSED:
    Serial.println(eKey);
  }
} 

/****** Functions for the MQTT server ******/
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
    microwave_Detection = true;
    if (alarm == true) { //The alarm is activated
      Serial.println("Intruder detected on the microwave sensor");
      intruderDetected();
    } 
  } 
  else if (String(topic).equals(String(esp32camTopic))) {
    cam = 1;
    cam_Detection = true;
    if(alarm == true){
      Serial.println("Intruder detected on the camera");
      intruderDetected();
    }
  } 
  else if (String(topic).equals(String(ultrasonicSensorTopic))) {
    ultrasonic = 1;
    ultrasonic_Detection = true;
    // Send to Andreas microcontroller
  }
  else if (String(topic).equals(String(infraredSensorTopic))) {
    infrared = 1;
    infrared_Detection = true;
    // Send to Andreas microcontroller
  }
}
