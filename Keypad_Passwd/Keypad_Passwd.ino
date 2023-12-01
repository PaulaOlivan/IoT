const int keyVals[16] = {809, 827, 924, 0, 
                         166, 219, 503, 0,
                         269, 313, 562, 0,
                         0,   647, 0,   0}; // 0 Acts like the enter key
const char keys[16] = {'1', '2', '3', 'A', 
                       '4', '5', '6', 'B',
                       '7', '8', '9', 'C',
                       '*', '0', '#', 'D'};
int range = 5; // Tolerance above or below the numeric value
int lastButton = -1; // Last pressed button
unsigned long lastPressTime = 0; // Last time a button was pressed
bool buttonReleased = true; // Indicates if the button was released
const int passwdSize = 4;
char inputPassword[passwdSize] = {'\0', '\0' , '\0', '\0'}; // Array to store the entered password
const char storedPassword[passwdSize] = {'1', '2', '3', '4'}; // Stored password
static int pos = 0;
const int buzzerPin = 4; // Pin for the passive buzzer

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(buzzerPin, OUTPUT); // Set the buzzer pin as output
  digitalWrite(buzzerPin, HIGH);
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time

  int keyIn = analogRead(A1); // Read the keypad input
  int detectedButton = detectButton(keyIn); // Find which button is detected

  if (detectedButton != -1 && buttonReleased) {
    if (detectedButton != lastButton && (currentTime - lastPressTime) > 200) {
      /*Serial.print("New key pulsed: ");
      Serial.print(keys[detectedButton]); // Show the button on the serial monitor
      Serial.print(" | Voltage A1: ");
      Serial.println(keyIn);*/
      lastButton = detectedButton;
      lastPressTime = currentTime; // Update the time of the last pressed button
      buttonReleased = false; // Disable detection until the button is released
      
      digitalWrite(buzzerPin, LOW); // Activate the passive buzzer
      delay(250); // Wait for a short duration
      digitalWrite(buzzerPin, HIGH); // Turn off the buzzer

      // Store the detected button in the password array
      storePassword(keys[detectedButton]);
      
      // Check if all four digits have been entered
      if (inputPassword[passwdSize-1] != '\0') {
        // Compare the entered password with the stored password
        bool passwordsMatch = comparePasswords();
        
        // Display the result based on the comparison
        if (passwordsMatch) {
          Serial.println("Passwords match!");
        } else {
          Serial.println("Passwords are different!");
        }
        
        // Reset the entered password array
        resetPassword();
      }
    }
  } else if (detectedButton == -1) {
    buttonReleased = true; // Enable detection if no button is detected
  }
  delay(500);
}

int detectButton(int inputValue) {
  for (int i = 0; i < 16; i++) {
    if (inputValue >= keyVals[i] - range && inputValue <= keyVals[i] + range) {
      return i; // Return the index of the detected button
    }
  }
  return -1; // Return -1 if no button is detected
}

void storePassword(int buttonIndex) {
  if (pos < passwdSize) {
    inputPassword[pos++] = buttonIndex;
  }
}

bool comparePasswords() {
  for (int i = 0; i < passwdSize; i++) {
    Serial.print(i);
    Serial.print(": input -> ");
    Serial.print(inputPassword[i]);
    Serial.print(" | output -> ");
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
  Serial.println("Restarting the password");
  pos = 0; // Restart the index for store the password correctly
}

