#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <Keypad.h>

#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
uint8_t storedUID[] = {0xC7, 0xFE, 0x66, 0x39};
uint8_t storedUIDLength = sizeof(storedUID);
uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID
bool cardDetected = false;

bool alarm = false; // false = non active || true = active
int tries = 0;
String password = "1234";

// Variables for ultrasonic sensor
const int triggerPin = 9;
const int echoPin = 10;
float pulse_width, US_distance;
float US_wall;
float US_range = 5.0;

// Variables for Keypad
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

const char keys[ROWS][COLS] = {'1', '2', '3', 'A', 
                               '4', '5', '6', 'B',
                               '7', '8', '9', 'C',
                               '*', '0', '#', 'D'};

byte rowPins[ROWS] = {14, 13, 8, 7}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {6, 2, 3, 5}; //connect to the column pinouts of the keypad
//Create an object of keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

const int passwdSize = 4;
char inputPassword[passwdSize] = {'\0', '\0' , '\0', '\0'}; // Array to store the entered password
const char storedPassword[passwdSize] = {'1', '9', '9', '1'}; // Stored password
static int pos = 0;

// Variables for Buzzer
const int buzzerPin = 4; //buzzer to arduino digital pin 0
int intruder = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) delay(10);
  keypad.addEventListener(keypadEvent); //add an event listener for this keypad

  // Setup for the Buzzer
  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 0 as an output
  digitalWrite(buzzerPin, HIGH); // Deactivated the buzzer pin

  //Setup for the Ultrasonic Sensor
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
}

void loop() {
  //char key = keypad.getKey();// Read the key
  if (alarm == false){ // Alarm deactivated only reads the keypad and the sensor
    for (int i=0; i<1000; i++){
      readKeypad();
      if (inputPassword[passwdSize - 1] != '\0') {
        bool passwordsMatch = comparePasswords();
        if (passwordsMatch) {
          Serial.println("Password validated, alarm activated");
          activateAlarm();
        } else {
          Serial.println("Incorrect password, try again");
          alarmIncorrectTry();
        }
        resetPassword();
      }
    }
    if (alarm == false){ //The alarm continues deactivated so read the RFID
      cardDetected = readRFID();
      if (cardDetected){
        if (uidLength == storedUIDLength && memcmp(uid, storedUID, uidLength) == 0){
          activateAlarm();
          Serial.println("Card validated, alarm activated");
        }
        else{ //The card set closer is not correct
          alarmIncorrectTry();
          Serial.println("Incorrect card, try again with the correct card");
        }
      }
    }
  }
  else{ // The alarm is active
    readUltrasonic();
    // Lets see if the alarm turn off
    readKeypad();
    if (inputPassword[passwdSize - 1] != '\0') {
      bool passwordsMatch = comparePasswords();
      if (passwordsMatch) {
        desactivateAlarm();
        Serial.println("Password validated, alarm desactivated");
      } else {
        alarmIncorrectTry();
        Serial.println("Incorrect password, try again");
      }
      resetPassword();
    }
    if (alarm == true){ //The alarm continues activated so read the RFID
      cardDetected = readRFID();
      if (cardDetected){
        if (uidLength == storedUIDLength && memcmp(uid, storedUID, uidLength) == 0){
          desactivateAlarm();
          Serial.println("Card validated, alarm desactivated");
        }
        else{ //The card set closer is not correct
          alarmIncorrectTry();
          Serial.println("Incorrect card, try again with the correct card");
        }
      }
    }  
  }
}

/****** Functions related to turning the alarm on and off ******/
void readKeypad() {
  char key = keypad.getKey();// Read the key
  // Print if key pressed
  if (key){
    Serial.print("Key Pressed : ");
    Serial.println(key);
    digitalWrite(buzzerPin, LOW); // Activate the passive buzzer
    delay(250); // Wait for a short duration
    digitalWrite(buzzerPin, HIGH); // Turn off the buzzer

    // Store the detected button in the password array
    storePassword(key);
  }

  if (key == 'D'){
    resetPassword();
  }
}

bool readRFID() {
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100); //Dont move the timeout
  if (success) {
    Serial.println("Found an ISO14443A card");
    // Display UID information
    Serial.print("  UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");
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
  delay(500);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  for (int i=0; i<2; i++){ //2 normal tones
    digitalWrite(buzzerPin, LOW);
    delay(250);
    digitalWrite(buzzerPin, HIGH);
    delay(100);
  }

  // Read the Ultrasonic sensor to measure the initial wall
  Serial.println("Initializing the ultrasonic sensor");
  digitalWrite(triggerPin, HIGH);   // Distance to the wall
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  US_wall = (pulse_width*.0343)/2; //measure in cm, divided by 2 cause sounds goes and comes back
  Serial.print("Measured distance ");
  Serial.print(US_wall);
  Serial.println(" cm");
}

void desactivateAlarm(){
  alarm = false;
  tries = 0;
  for (int i=0; i<2; i++){ //2 normal tones
    digitalWrite(buzzerPin, LOW);
    delay(250);
    digitalWrite(buzzerPin, HIGH);
    delay(100);
  }
  digitalWrite(buzzerPin, LOW); // 1 large tone
  delay(500);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
}

void alarmIncorrectTry(){
  tries++;
  if (tries >= 3){ //Maximum number of attempts reached
    intruderDetected();
  }
  else{
    digitalWrite(buzzerPin, LOW);
    delay(1500);
    digitalWrite(buzzerPin, HIGH);
  }
}

void intruderDetected(){
  for (int i=0; i<10; i++){ //10 normal tones
      digitalWrite(buzzerPin, LOW);
      delay(250);
      digitalWrite(buzzerPin, HIGH);
      delay(100);
  }
}

/********* Functions for detect intruder *********/
void readUltrasonic(){
  digitalWrite(triggerPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pulse_width = pulseIn(echoPin, HIGH);
  US_distance = (pulse_width*.0343)/2;

  if (US_distance > (US_wall+US_range) || US_distance < (US_wall-US_range)){
    Serial.print("Intruder detected at = ");
    Serial.print(US_distance);
    Serial.println(" cm");
    US_wall = US_distance;
    Serial.print("New wall measure = "); // Update the wall measure
    Serial.print(US_wall);
    Serial.println(" cm");
    intruderDetected();
  }
}

/********* Functions for the Keypad *********/
void storePassword(char key) {
  if (pos < passwdSize) {
    inputPassword[pos++] = key;
  }
}

bool comparePasswords() {
  for (int i = 0; i < passwdSize; i++) {
    Serial.print(i);
    Serial.print(": input -> ");
    Serial.print(inputPassword[i]);
    Serial.print(" | stored -> ");
    Serial.println(storedPassword[i]);
    if (inputPassword[i] != storedPassword[i]) {
      return false;
    }
  }
  return true;
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
  //   if (passwd_pos - 15 >= 5) { 
  //     return ;
  //   }
  //   lcd.setCursor((passwd_pos++),0);
  //   switch (eKey){
  //   case '#':                 //# is to validate password 
  //     passwd_pos  = 15;
  //     checkPassword(); 
  //     break;
  //   case '*':                 //* is to reset password attempt
  //     password.reset(); 
  //     passwd_pos = 15;
  //  // TODO:
//     break;
  //   default: 
  //     password.append(eKey);
  //     lcd.print("*");
  //   }
  }
  // Serial.print("Keypressed: ");
  // Serial.println(eKey);


}




