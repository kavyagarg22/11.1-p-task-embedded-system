#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Arduino.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>

// Replace with your Wi-Fi credentials
const char* ssid = "";
const char* password = "";

// Replace with your MQTT broker details
const char* mqtt_server = "";
const int mqtt_port = ;

// Initialize the Wi-Fi client
WiFiClient wifiClient;

// Initialize the MQTT client
PubSubClient client(wifiClient);

// Creating Object of TinyGPSPlus
TinyGPSPlus gps;

//Defining Hardwares
#define USUP_ECHO 4
#define USUP_TRIG 5
#define Water_Sen A0
#define Fall_Btn 6
#define Emer_Swth 3


//Defining Variables
long Distance;
int currentState;
int fallCurrentState;
int lastState = HIGH;
int fallLastState = HIGH;
float longTi;
float laTi;
int mFlag=0;
int eFlag=0;
int fFlag=0;


void setup() {
	
  Serial.begin(9600);

  // GPS and Blueetooth Initializing
  Serial1.begin(9600);  
  // Connect to Wi-Fi
  connectToWiFi();
  // Set MQTT server and port
  client.setServer(mqtt_server, mqtt_port);

  delay(3000);
  
  // Defining PINs of Hardwares
  pinMode(Fall_Btn, INPUT_PULLUP);
  pinMode(USUP_ECHO, INPUT);
  pinMode(USUP_TRIG, OUTPUT);
  pinMode(Emer_Swth, INPUT_PULLUP);

}


void loop() {
  if (!client.connected()) {
    // Reconnect to MQTT broker if the connection is lost
    reconnect();
  }

  // Calling UltraDist Function to calculate distance
  Distance = UltraDist();

  // Distance threshhold check
  if (Distance < 401) {
    Serial.print("Distance: ");
    Serial.println(Distance);
  } else {
  }


  // Water Resistance
  SenseAcitivity();


  //Fall Detection
  fallCurrentState = digitalRead(Fall_Btn);
  if (fallLastState == LOW && fallCurrentState == HIGH) {
    Serial.println("Fall Detected");
	fFlag = 1;
  }
  else{
    fFlag = 0;
  }
  fallLastState = fallCurrentState;


  // Switch of emergency and GPS
  currentState = digitalRead(Emer_Swth);
  if (lastState == LOW && currentState == HIGH) {
	eFlag = 1;

  //GPS
    gps_locator();
    Serial.print(longTi);
    Serial.print(", ");
    Serial.print(laTi);


    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println("No GPS");
	}
	  // Emergency message send over bluetooth
      String emergency = "*-E"+String(eFlag)+"-"+String(longTi)+"-"+String(laTi)+"-#";
	  // *-E0-longti-lati-#
      Serial1.println(emergency);
      delay(2000);
      String Mes= "I am in Emergency My Location: "+String(longTi)+","+String(laTi);
      publishMessage("SmartShoe/Emergency",Mes);

    
  }
  else{
	eFlag=0;
  }
  
  lastState = currentState;
  
  //Final Message send over Bluetooth
  String sendMessage = "*-M"+String(mFlag)+"-D"+String(Distance)+"-T"+String(fFlag)+"-#";
  // *-M0-D45-T0-#
  Serial1.println(sendMessage);
  delay(2000);
  client.loop();

}



// Calculating Distance
int UltraDist() {
  long duration, distance;
  digitalWrite(USUP_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(USUP_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(USUP_TRIG, LOW);
  duration = pulseIn(USUP_ECHO, HIGH);
  distance = (duration / 2) / 29.1;
  return distance;
}


// Reading Moisture Reading

void SenseAcitivity() {
  int Value = analogRead(Water_Sen);
  if (Value < 950) {
    mFlag = 1;
  } else {
	mFlag = 0;
  }
}


// Getting Cordinates
void getCordinate()
{
   
  if (gps.location.isValid())
  {
    laTi = gps.location.lat();
    longTi = gps.location.lng();
  }
  
}

void gps_locator(){
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      getCordinate();

  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS detected: check wiring."));
    
  }
}



void connectToWiFi() {
  // Attempt to connect to Wi-Fi
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(1000);
  }

   Serial.println("Connected to WiFi");
}

void reconnect() {
  // Loop until we're reconnected to the MQTT broker
  while (!client.connected()) {
    // Attempt to connect to the MQTT broker
    if (client.connect("ArduinoClient")) {
      
    } else {
      // Wait before retrying
      delay(5000);
    }
  }
}

void publishMessage(const char* topic, const String& message) {
  // Publish a message to the specified MQTT topic
  client.publish(topic, message.c_str());
  
}