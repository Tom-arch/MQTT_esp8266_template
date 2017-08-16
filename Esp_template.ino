//Template of firmware for ESP 8266 to control an automated system with MQTT protocol
//The example device has a relay connected at pin 12 and a sensor connected at PIN 14 that a binary value

//Global includes
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>


//Define constants: pin numbers and delays. Warning: do not use PIN GPIO 16 (D0) since it is LED_BUILTIN
//Delays in milliseconds
#define NO_WIFI   100
#define NO_MQTT   1000
#define WORKING   0
#define PUB_DELAY 30000 //This should be the sensor publishing interval
#define SENSOR_DELAY 1000 //This should be the actual sensor reading delay. The difference helps keeping low bandwidth occupation.
#define ERR       4
#define VALID_ADDR 0
#define STATUS_ADDR 1
#define RELAY_PIN  12
#define SENSOR_PIN 14


//Global constants
// Wifi
const PROGMEM char* WL_SSID = "your_ssid";
const PROGMEM char* WL_PWD = "your_password";

// MQTT
const PROGMEM char* SERVER = "192.168.1.4";
const PROGMEM uint16_t PORT = 1883;
const PROGMEM char* CLIENT_ID = "your_client_id";
const PROGMEM char* USERNAME = "test";
const PROGMEM char* PASSWORD = "password";
const PROGMEM char* STATE_TOPIC = "test/status";
const PROGMEM char* COMMAND_TOPIC = "test/commands";
const PROGMEM char* SENSOR_TOPIC = "test/sensor";

// Payload
const char* PAYLOAD_ON = "ON";
const char* PAYLOAD_OFF = "OFF";


//Global variables
  //The wifi has been requested?
bool wifiRequested = false;

  //Led blink global variables -> led status, time of the next blink, time between two blinks, boolean that tells id we should try reconnecting.
bool ledStatus = false;
unsigned long nextBlinkMillis = 0;
unsigned long blinkInterval = NO_WIFI;
//unsigned long blinkTemp = 0; //Debug
bool reconnect = false;

  //Sensor reading variables -> sensor value, time of the next publishing
bool sensorValue = false;
unsigned long nextSensorMillis = 0;

  //Periodic status publishing variables -> publishStatus tells the system if it should publish at the current loop, the other is the time of the next publishing. RelayStatus is the current status of the relay
bool relayStatus = false;
bool publishStatus = false;
unsigned long nextStatusMillis = 0;


  //Holds the command to execute
volatile uint8_t command = ERR;


WiFiClient wfClient;
PubSubClient mqttClient;



//Set up the sketch components
void setup() {
  
  //Serial setup. Serial can be left in the final sketch since the ESP has lots of memory
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  //Pins initialization
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(SENSOR_PIN, INPUT);
  /* Please complete with modes and initial states for output pins */

  // Init the mqtt client parameters
  mqttClient.setClient(wfClient);
  mqttClient.setServer(SERVER, PORT);
  mqttClient.setCallback(callback);

  //EEPROM initialization. The EEPROM is used to save the status in case the board resets. Nothing should be written in the EEPROM in this stage.
  EEPROM.begin(10);
  bool valid = EEPROM.read(VALID_ADDR);
  if(valid) {
    bool is_on = EEPROM.read(STATUS_ADDR);
    relayStatus = is_on;
    Serial.println("EEPROM value: "); Serial.println(is_on);
    digitalWrite(RELAY_PIN, is_on);
  } else {
    //Else leave the default values
    Serial.println("Invalid EEPROM");
  }

  //Led initialization pattern. Please note that in the ESP8266 NodeMcu board the LED_BUILTIN is on when the value is LOW.
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
  }
  Serial.println("READY");

  //Init timer global variables
  nextBlinkMillis = millis() + blinkInterval;
  nextStatusMillis = millis() + PUB_DELAY;
  nextSensorMillis = millis() + SENSOR_DELAY;
  
}



//Code to run repeatedly
void loop() {

  //Execute the command if registered
  executeCommand();
  
  //Check the timers
  checkTimers();
  
  //Check the connectivity
  checkConnectivity();
/* Debug
  if(blinkInterval != blinkTemp) {
    Serial.println("BLINK CHANGE");
    Serial.println(blinkInterval);
  }
  blinkTemp = blinkInterval;
*/
}



//Execute a command
void executeCommand() {

  //Copy the command variable to avoid incoherent results caused by its modification while executing this method
  uint8_t cmd = command;
  command = ERR;
  if (cmd == 0) {
    digitalWrite(RELAY_PIN, HIGH);
    relayStatus = true;
    EEPROM.write(VALID_ADDR, true);
    EEPROM.write(STATUS_ADDR, true);
    EEPROM.commit();
    Serial.println("Executed command 0");
    publishStatus = true; //Publish the status immediately to let Home Assistant know of the change
  } else if (cmd == 1) {
    digitalWrite(RELAY_PIN, LOW);
    relayStatus = false;
    EEPROM.write(VALID_ADDR, true);
    EEPROM.write(STATUS_ADDR, false);
    EEPROM.commit();
    Serial.println("Executed command 1");
    publishStatus = true;
  }
  
  /* Please complete with other commands */
  
}



//This method checks if the Millis timers have expired. The timers work by saving the Millis value of the event. This way the Millis overflow problem is solved, as stated in the Arduino wiki.
void checkTimers() {
  
  //Sample the current time
  unsigned long currentMillis = millis();

  //Blink and MQTT connecting timer handling
  if (blinkInterval == WORKING) { //If the system is working leave the led on
    digitalWrite(LED_BUILTIN, LOW);
    ledStatus = true;
  } else if(currentMillis >= nextBlinkMillis) {
    triggerLed();
    reconnect = true;
    nextBlinkMillis = currentMillis + blinkInterval;
  }

    //Sensor reading timer handling
  if(currentMillis >= nextSensorMillis) {
    bool tempValue = digitalRead(SENSOR_PIN);
    if(tempValue != sensorValue) { //If there has been a change of the sensor value, publish the status to alert the server. Generally for int values, check if abs(difference) is greater than a threshold value
      publishStatus = true;
      Serial.println("Change of sensor value");
    }
    sensorValue = tempValue;
    nextSensorMillis = currentMillis + SENSOR_DELAY;
  }

  //Publish status timer handling
  if(currentMillis >= nextStatusMillis) {
    publishStatus = true;
    nextStatusMillis = currentMillis + PUB_DELAY;
  }
  
}



//This method checks that the wifi is connected and sets the led blink time if there is anything wrong
void checkConnectivity() {

  //Check if the wifi is not connected
  if(WiFi.status() != WL_CONNECTED) {
    if(!wifiRequested) {
      WiFi.begin(WL_SSID, WL_PWD);
      blinkInterval = NO_WIFI;
      wifiRequested = true;
    }
  } else {
    wifiRequested = false;
    //Check if MQTT is not connected
    if (!mqttClient.connected()) {
      if (reconnect) {
        Serial.println(" MQTT RECONNECT");
        if(mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD)) {
          mqttClient.subscribe(COMMAND_TOPIC);
          /* Please add topics */
        }
        reconnect = false;
      }
      blinkInterval = NO_MQTT;
    } else {
      blinkInterval = WORKING;
      //Publish the status
      if(publishStatus) {
        publishStatus = false;
        
        if(relayStatus) {
          mqttClient.publish(STATE_TOPIC, PAYLOAD_ON);
        } else {
          mqttClient.publish(STATE_TOPIC, PAYLOAD_OFF);
        }
        
        if(sensorValue) {
          mqttClient.publish(SENSOR_TOPIC, PAYLOAD_ON);
        } else {
          mqttClient.publish(SENSOR_TOPIC, PAYLOAD_OFF);
        }
        mqttClient.publish(STATE_TOPIC, " "); //Debug only
        
        /* Please add publish statements here */
      }
      //Loop the MQTT client. This is mandatory.
      mqttClient.loop();
    }
  }
  
}



//Callback to receive MQTT commands. This only sets the command voltaile variable.
void callback(char* topic, byte* payload, unsigned int len) {
  
  //Parse the message
  char message_buff[len+1];
  int i = 0;
  for(i=0; i<len; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  //Check the topic and the command
  if(strcmp(topic, COMMAND_TOPIC) == 0) {
    if(strcmp(message_buff, PAYLOAD_ON) == 0) {
      command = 0;
    } else if(strcmp(message_buff, PAYLOAD_OFF) == 0) {
      command = 1;
    } else {
      Serial.println("Command not accepted");
    }
  }
  
}



//This method triggers the LED_BUILTIN. Note that the LED is on when LOW is set.
void triggerLed() {
  if(ledStatus) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledStatus = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    ledStatus = true;
  }
  
}

