#include <SPI.h>
#include <cstring> 
#include <WiFi.h>
#include <ESP32Servo.h>
#include <mqtt_client.h>
#include <PubSubClient.h>
#include <time.h>

Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
 
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
#define servoPin 18
int lSensor = 4;
int blue = 2;

String flags = "00000000";
int heartBeatInt = 60 * 5; //default heartbeat 1 minute
unsigned long int hbCount = 0; // remove before actually releasing the product
unsigned long int lastHB = 0;
int on = 0;
int off = 0;

const char* devID = "1234";
const char* ssid = "StorageDefender";
const char* password = "BigRockCandy1";

const char *mqtt_broker = "sensorxperience.com";
const char *upTopic = "esp32/uplink/1";
const char *downTopic = "esp32/downlink/1";
const char *mqtt_username = "admin";
const char *mqtt_password = "password";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);


void setup() {
	// Allow allocation of all timers


	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  myservo.write(0);
  Serial.begin(115200);
  pinMode(blue, OUTPUT);
  digitalWrite(blue, LOW);
  WifiConnect();
  configTime(0, 0, "pool.ntp.org");
  pinMode(lSensor, INPUT);
  

	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
}

void WifiConnect(){

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        digitalWrite(blue, HIGH);
        delay(250);
        digitalWrite(blue, LOW);
        delay(250);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = devID;
        Serial.printf("The client %s connecting to mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to SensorXperience");
        } 
        else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
        client.subscribe(downTopic);
    }
}

void callback(char *topic, byte *payload, unsigned int length) {
 Serial.print("Message arrived in topic: ");
 Serial.println(upTopic);
 Serial.print("Message:");
 String payloadApp;
 for (int i = 0; i < length; i++) {
     Serial.print((char) payload[i]);
     payloadApp += (char)payload[i];
 }
 flags = payloadApp;
 Serial.println();
 Serial.println(flags);
 Serial.println("-----------------------");
}

//returns true if the timeout interval has been reached
bool timeOut(unsigned long int originalTs, int interval) {
  unsigned long int ts = getTime();
  if((ts - originalTs) > interval) {
    return true;
  }
  else {
    return false;
  }
}

//Toggles the servo motor and keeps trying until the light status function returns false
void servoToggle() {
  unsigned long int beginning = getTime();
  myservo.write(0);
  delay(10);
  while(lightStatus() && !timeOut(beginning, 10)) {
    for(int i = 0; i < 180; i++) {
      myservo.write(i);
      delay(10);
    }
    delay(500);
    myservo.write(0);
    delay(1000);
  }
}

//Used to calubrate the photo resistor so that the parameters are more flexible
int lightCalibrate() {
  int conf = 0;
  while(1) {
    Serial.println("Please turn light on\n");
    if(Serial.parseInt() == 1) {

      on = analogRead(lSensor);
      conf++;
      break;
    }
    delay(100);
  }
  while(1) {
    Serial.println("Please turn light off\n");
    if(Serial.parseInt() == 2) {
      off = analogRead(lSensor);
      conf++;
      break;
    }
    delay(100);
  }
  return conf;
}

//returns true if light is on
bool lightStatus() {
  int ret = false;
  int currState = analogRead(lSensor);
  int mean = (on + off)/2;
  if(currState >= mean) {
    ret = true;
  }
  return ret;
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
 
void loop() {
  if(timeOut(lastHB, heartBeatInt)) {
    flags[3] = '1';
  }
  if(Serial.available()) {
      flags = Serial.readString();
  }
  if (flags[0] == '1') {
    if(lightCalibrate() == 2) {
      Serial.print("onval, offval:");
      Serial.println(on);
      Serial.println(off);
    }
    else {
      Serial.println("error setting light parameters");
    }
    
  }
  if (flags[1] == '1') {
    servoToggle();
  }
  if (flags[2] == '1') {
    Serial.println(lightStatus());
    Serial.println(analogRead(lSensor));
  }
  if (flags[3] == '1') {
    hbCount++;
    String deviceID = devID;
    String packetStr = "{\"deviceID\": \""+ deviceID +"\", \"timestamp\": \""+ getTime() +"\", \"count\": \""+ hbCount +"\"}";

    const char* packet = packetStr.c_str(); 
    //Serial.println(packet);
    client.publish(upTopic, packet);
    lastHB = getTime();
  }
 flags = "00000000";
 client.loop();
 delay(100);
}