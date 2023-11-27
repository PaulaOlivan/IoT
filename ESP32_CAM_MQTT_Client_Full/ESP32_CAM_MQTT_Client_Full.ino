/**
  First install appropriate libraries in IDE
**/

// turn on debug messages
// #define VERBOSE //Uncomment for full information about camera detection
#include "EloquentSurveillance.h"

/**
 * Instantiate motion detector
 */
EloquentSurveillance::Motion motion;


#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"

const char* ssid = "RedmiNote9Pro";
const char* password = "12345678";
const char* mqttServer = "test.mosquitto.org";  // Uses the Mosquitto public broker
const int mqttPort = 1883;
const char* mqttClientId = "esp32cam-client";

const char* microwaveSensorTopic = "motionDetection/microwaveSensor";
const char* esp32camTopic = "motionDetection/esp32cam";
const char* ultrasonicSensorTopic = "motionDetection/ultrasonicSensor";
const char* infraredSensorTopic = "motionDetection/infraredSensor";
const char* keypadTopic = "motionDetection/keypad";

WiFiClient espClient;
PubSubClient client(espClient);

int microwaveSensorPin = 16;  // pin where Microwave Motion Detection Sensor is attached to
// volatile bool motionDetected = false;

unsigned long startTime = micros();

int microwave = 0;
int cam = 0;
int ultrasonic = 0;
int infrared = 0;
char* keypad;

void setup() {

  Serial.begin(115200);//Initialize serial connection
  delay(5000);//Time for user to open serial monitor on Arduino IDE

  debug("INFO", "Init");


  // ------------------ ESP32 CAM SETUP ----------------------------
  // ---------------------------------------------------------------

  /**
    * Configure camera model
    * You have access to the global variable `camera`
    * Allowed values are:
    *  - aithinker()
    *  - m5()
    *  - m5wide()
    *  - eye()
    *  - wrover()
    */
  camera.aithinker();

  /**
    * Configure camera resolution
    * Allowed values are:
    * - _96x96()
    * - qqvga()
    * - qcif()
    * - hqvga()
    * - _240x240()
    * - qvga()
    * - cif()
    * - hvga()
    * - vga()
    * - svga()
    * - xga()
    * - hd()
    * - sxga()
    * - uxga()
    * - fhd()
    * - p_hd()
    * - p_3mp()
    * - qxga()
    * - qhd()
    * - wqxga()
    * - p_fhd()
    * - qsxga()
    */
  camera.qvga();

  /**
    * Configure JPEG quality
    * Allowed values are:
    *  - lowQuality()
    *  - highQuality()
    *  - bestQuality()
    *  - setQuality(quality), ranging from 10 (best) to 64 (lowest)
    */
  camera.bestQuality();

  /**
    * Configure motion detection
    *
    * > setMinChanges() accepts a number from 0 to 1 (percent) or an integer
    *   At least the given number of pixels must change from one frame to the next
    *   to trigger the motion.
    *   For example the line "motion.setMinChanges(0.01)" translates to "Trigger motion if at least 1% of the pixels
    *   in the image changed value"
    */
  motion.setMinChanges(0.0);

  /**
    * > setMinPixelDiff() accepts an integer
    *   Each pixel value must differ at least of the given amount from one frame to the next
    *   to be considered as different.
    *   The following line translates to "Consider a pixel as changed if its value increased
    *   or decreased by 1 (out of 255)"
    */
  motion.setMinPixelDiff(1);

  /**
    * > setMinSizeDiff() accepts a number from 0 to 1 (percent) or an integer
    *   To speed up the detection, you can exit early if the image size is almost the same.
    *   This is an heuristic that says: "If two consecutive frames have a similar size, they
    *   probably have the same contents". This is by no means guaranteed, but can dramatically
    *   reduce the computation cost.
    *   The following line translates to "Check for motion if the filesize of the current image
    *   differs by more than 5% from the previous".
    *
    *   If you don't feel like this heuristic works for you, delete this line.
    */
  // motion.setMinSizeDiff(0.05);

  /**
    * Turn on debouncing.
    * It accepts the number of milliseconds between two consecutive events.
    * The following line translates to "Don't trigger a new motion event if
    * 10 seconds didn't elapsed from the previous"
  */
  // motion.debounce(10000L);
  motion.debounce(200L);

  /**
    * Initialize the camera
    * If something goes wrong, print the error message
    */
  while (!camera.begin())
      debug("ERROR", camera.getErrorMessage());

  debug("SUCCESS", "Camera OK");

  // ---------------------------------------------------------------
  // ---------------------------------------------------------------


  /** Setting microwaveSensorPin pin (pin 16) as INPUT for reading motion detection
    from Microwave Motion Sensor
  **/
  pinMode(microwaveSensorPin, INPUT);

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

  // Capture and send data if motion is detected
  captureAndSendIfMotion();

  // Serial.print("MICROWAVE: ");
  // Serial.println(microwave);
  // microwave = 0;


  // Serial.print("CAM: ");
  // Serial.println(cam);
  // cam = 0;

  Serial.println("| MICROWAVE |    CAM    | ULTRASONIC|  INFRARED |   KEYPAD  |");
  Serial.println("|     " + String(microwave) + "     |     " + String(cam) + "     |     " + String(ultrasonic) + "     |     " + String(infrared) + "    |     " + String(keypad) +"     |");

  microwave = 0;
  cam = 0;
  ultrasonic = 0;
  infrared = 0;
  keypad = 0;


  Serial.println("| MICROWAVE |    CAM    | ULTRASONIC|  INFRARED |   KEYPAD  |");
  Serial.println("|     " + String(microwave) + "     |     " + String(cam) + "     |     " + String(ultrasonic) + "     |     " + String(infrared) + "    |     " + String(keypad) +"     |");


  delay(500);
}


void captureAndSendIfMotion() {

  // ---------- MICROWAVE Sensor detection and publishing to MQTT --------
  // ---------------------------------------------------------------------
  /** 
    * Only if 2.5 seconds have past, then take another measure from Microwave Motion Sensor
    * (for not overwhelming MQTT server)
  **/
  if(micros() - startTime > 2500000) {
    startTime = micros();

    int microwaveSensorMotionValue = digitalRead(microwaveSensorPin);

    // If motion from MICROWAVE Sensor is detected, send "1" to the corresponding MQTT topic (topic: "motionDetection/microwaveSensor")
    if (microwaveSensorMotionValue == HIGH) {
      // Serial.println("** MICROWAVE motion: DETECTED");
      // Serial.println("----");
      client.publish(microwaveSensorTopic, "MICROWAVE MOTION DETECTION");
      // Serial.println("\nMotion detected from MICROWAVE sensor! Message sent to MQTT.\n");
      microwaveSensorMotionValue = LOW;
    }
    // else {
    //   Serial.println("---");
    // }

  }
  // ---------------------------------------------------------------------
  // ---------------------------------------------------------------------



  // ------------ ESP32 CAM detection and publishing to MQTT -------------
  // ---------------------------------------------------------------------
  /**
    * Try to capture a frame
    * If something goes wrong, print the error message
  */
  int esp32camMotionValue = LOW;

  if (!camera.capture()) {
      debug("ERROR", camera.getErrorMessage());
      return;
  }

  /**
    * Look for motion.
    * In the `true` branch, you can handle a motion event.
    * For the moment, just print the processing time for motion detection.
    */
  if (!motion.update())
      return;

  if (motion.detect()) {
      // debug("INFO", String("Motion detected in ") + motion.getExecutionTimeInMicros() + " us");
      esp32camMotionValue = HIGH;
      delay(300);
  }
  else if (!motion.isOk()) {
    /**
      * Something went wrong.
      * This part requires proper handling if you want to integrate it in your project
      * because you can reach this point for a number of reason.
      * For the moment, just print the error message
      */
    debug("ERROR", motion.getErrorMessage());
  }

  // If motion from ESP32 CAM is detected, send "1" to the corresponding MQTT topic (topic: "motionDetection/esp32cam")
  if (esp32camMotionValue == HIGH) {

    client.publish(esp32camTopic, "CAM MOTION DETECTION");
    esp32camMotionValue = LOW;

  }
  else {
    esp32camMotionValue = LOW;
  }

  // ---------------------------------------------------------------------
  // ---------------------------------------------------------------------
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
    // Serial.println("hi");
    microwave = 1;

  } else if (String(topic).equals(String(esp32camTopic))) {
    cam = 1;
  } else if (String(topic).equals(String(ultrasonicSensorTopic))) {
    cam = 1;
  } else if (String(topic).equals(String(infraredSensorTopic))) {
    cam = 1;
  } else if (String(topic).equals(String(keypadTopic))) {
    cam = 1;
  }

}