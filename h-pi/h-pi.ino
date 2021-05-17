#include "config.h"
// #include "connection_status.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// MQTT config imported from config.h
const char* mqtt_server = mqtt_server_config;  // IP of the MQTT broker
const char* mqtt_username = mqtt_username_config; // MQTT username
const char* mqtt_password = mqtt_password_config; // MQTT password
const char* data_topic = data_topic_name; 
const char* clientID = thing_id; 


WiFiClient wifi_client;
const char* ssid     = ssid_config;
const char* password = password_config;

PubSubClient client(mqtt_server, 1883, wifi_client);


const int capacity = JSON_OBJECT_SIZE(6);
StaticJsonDocument<capacity> doc;

/*
 * Pump configurations
 */
const int pump1 = 14;
const int pump2 = 27;
const int pump3 = 26;
const int pump4 = 25;
const int moisture_sensor1 = 32;
const int moisture_sensor2 = 35;
const int moisture_sensor3 = 34;
const int AirValue1 = 470;  
const int AirValue2 = 550; 
const int WaterValue = 210;
int soilmoisturepercent=0;
int val = 0;


/*
 * 
 */

const int push_button = 12;
const int indicator_led = 21;
int relay_flag = 0;

void setup() {

  Serial.begin(115200);
  Serial.println(clientID);
  
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);
  pinMode(pump1, OUTPUT);
  pinMode(indicator_led, OUTPUT);

  pinMode(push_button, INPUT_PULLUP);

  pinMode(moisture_sensor1, INPUT);
  pinMode(moisture_sensor2, INPUT);
  pinMode(moisture_sensor3, INPUT);

  attachInterrupt(digitalPinToInterrupt(push_button), handlePushButton, FALLING); // trigger when button pressed, but not when released.

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

}

void handlePushButton() {
  /* 
   *  Handle the interrupt for user pressing the push button
   */
   
  if (relay_flag == 0) {
    relay_flag = 1;
  }
}

void pumps_off(){
  digitalWrite(pump1, LOW);
  digitalWrite(pump2, LOW);
  digitalWrite(pump3, LOW);
  digitalWrite(pump4, LOW);
  digitalWrite(indicator_led, LOW);

}

void handlePumps_dummy(){
    digitalWrite(indicator_led, HIGH);
    Serial.println("pump");
    delay(2000);
    Serial.println("pump");
    timeClient.update();
    String cur_time = timeClient.getFormattedTime();
    doc["humidity"].set(100);
    doc["time"].set(cur_time);

    char JSONmessageBuffer[capacity];
    serializeJson(doc, JSONmessageBuffer);
    
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      delay(50); // This delay ensures that client.publish doesn't clash with the client.connect call
      client.publish(data_topic, JSONmessageBuffer);
    }
    pumps_off();
}

void handlePumps(){
  digitalWrite(pump1, HIGH);
  digitalWrite(pump2, HIGH);
  delay(5000);
  pumps_off();
  digitalWrite(pump3, HIGH);
  digitalWrite(pump4, HIGH);
  delay(5000);
  pumps_off();
}

void loop() {
  if (relay_flag == 1){
    handlePumps_dummy();
    relay_flag = 0;
  }

  soilmoisturepercent = map(analogRead(moisture_sensor1), AirValue1, WaterValue, 0, 100);
  Serial.print(soilmoisturepercent);
  Serial.print(",");
  
  soilmoisturepercent = map(analogRead(moisture_sensor2), AirValue2, WaterValue, 0, 100);
  Serial.print(soilmoisturepercent);
  Serial.print(",");
  
  soilmoisturepercent = map(analogRead(moisture_sensor3), AirValue2, WaterValue, 0, 100);
  Serial.println(soilmoisturepercent);

}
