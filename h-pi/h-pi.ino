#include "config.h"
// #include "connection_status.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"

#include "message.pb.h"

#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// MQTT config imported from config.h
const char* mqtt_server = mqtt_server_config;  // IP of the MQTT broker
const char* mqtt_username = mqtt_username_config; // MQTT username
const char* mqtt_password = mqtt_password_config; // MQTT password
const char* data_topic = data_topic_name; 
const char* config_topic = config_topic_name; 
const char* measure_topic = measure_topic_name; 
const char* pump_topic = pump_topic_name; 
const char* clientID = thing_id; 
const char* region = thing_region; 
const char* measurement = thing_measurement; 
const int interr_time_rate = interr_time_rate_config;  // IP of the MQTT broker


WiFiClient wifi_client;
const char* ssid     = ssid_config;
const char* password = password_config;


PubSubClient client(mqtt_server, 1883, wifi_client);

//PubSubClient client(mqtt_server, 1883, wifi_client);

const int capacity = JSON_OBJECT_SIZE(12);
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
uint8_t conn_stat = 0;
unsigned long waitCount = 0;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int interrupt_flag = 1;

void IRAM_ATTR onTimer() {
  /*
   * Handle timer interrupt for reading photoresistor
   */
  interrupt_flag = 1;
  
  }

void setup() {

  Serial.begin(115200);
  Serial.println(F("H-PI - Human-Plant Interface"));
  Packet message = Packet_init_zero;
  message.measurement =  thing_measurement;
  
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

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, interr_time_rate, true);
  pumps_off();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  Serial.print(messageTemp);
}

void connection_status() {
  /* 
   *  Handle different WIFI statuses, and MQTT connection.
   */
  if ((WiFi.status() != WL_CONNECTED) && (conn_stat != 1)) {
    conn_stat = 0;
  }
  if ((WiFi.status() == WL_CONNECTED) && (conn_stat <= 1)) {
    conn_stat = 2;
  }
  switch (conn_stat) {
    case 0:                                                       // MQTT and WiFi down: start WiFi
      Serial.println("WiFi down: start WiFi");
      WiFi.begin(ssid, password);
      conn_stat = 1;
      break;
    case 1:                                                       // WiFi starting, do nothing here
      Serial.println("WiFi starting, wait : " + String(waitCount));
      waitCount++;
      if (waitCount % 1000 == 0) {
        conn_stat = 0;
      }
      break;
    case 2:  // WiFi up, MQTT down: start MQTT
      if (!client.connected()){
        Serial.println("WiFi up, MQTT down: start MQTT");
        if (client.connect(clientID, mqtt_username, mqtt_password)) {
          conn_stat = 3;
          client.setServer(mqtt_server, 1883);
          client.setCallback(callback);
          client.subscribe(config_topic);
          client.subscribe(measure_topic);
          client.subscribe(pump_topic);

          timeClient.update();
          Serial.print("Formatted time configured: ");
          Serial.println(timeClient.getFormattedTime());
          waitCount = 0;
        }
      }
      else {
        conn_stat = 3;
      }
      break;
    case 3:
      Serial.println("WiFi up, MQTT up: finished MQTT configuration");
      conn_stat = 5;
      timerAlarmEnable(timer);
      break;
  }
  delay(10);
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
  digitalWrite(pump1, HIGH);
  digitalWrite(pump2, HIGH);
  digitalWrite(pump3, HIGH);
  digitalWrite(pump4, HIGH);
}

void handlePumps_dummy(){
    digitalWrite(indicator_led, HIGH);
    Serial.println("pump");
    delay(2000);
    Serial.println("pump");
    timeClient.update();
    String cur_time = timeClient.getFormattedTime();
    pumps_off();
}

void handlePumps(){
  digitalWrite(pump1, LOW);
  delay(10000);
  pumps_off();
  digitalWrite(pump2, LOW);
  delay(10000);
  pumps_off();
  digitalWrite(pump3, LOW);
  delay(2000);
  pumps_off();
  digitalWrite(pump4, LOW);
  delay(3000);
  pumps_off();
}

void loop() {
  connection_status();
  client.loop();
  if (relay_flag == 1){
    handlePumps();
    Serial.println("Pump");
    relay_flag = 0;
  }
  if (interrupt_flag == 1){
    JsonObject tags = doc.createNestedObject("tags");
    JsonObject fields = doc.createNestedObject("fields");
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      timeClient.update();
      String cur_time = timeClient.getFormattedTime();
      Serial.println(cur_time);
      doc["measurement"].set(measurement);
      JsonObject tags = doc.createNestedObject("tags");
      tags["host"].set(thing_id);
      tags["region"].set(region);
      doc["time"].set(cur_time);
            
      soilmoisturepercent = map(analogRead(moisture_sensor1), AirValue1, WaterValue, 0, 100);
      Serial.print(soilmoisturepercent);
      Serial.print(",");
      fields["soil_sensor_1"].set(soilmoisturepercent);
      
      soilmoisturepercent = map(analogRead(moisture_sensor2), AirValue2, WaterValue, 0, 100);
      Serial.print(soilmoisturepercent);
      Serial.print(",");
      fields["soil_sensor_2"].set(soilmoisturepercent);
      
      soilmoisturepercent = map(analogRead(moisture_sensor3), AirValue2, WaterValue, 0, 100);
      Serial.println(soilmoisturepercent);
      fields["soil_sensor_3"].set(soilmoisturepercent);
  
      char JSONmessageBuffer[capacity];
      serializeJson(doc, JSONmessageBuffer);
      if (client.publish(data_topic, JSONmessageBuffer)) {
        Serial.println("Data sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Data failed to send. Reconnecting to MQTT Broker and trying again");
        if (client.connect(clientID, mqtt_username, mqtt_password)) {
          delay(50); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(data_topic, JSONmessageBuffer);
        }
        else {
          Serial.println("Data failed to send. MQTT broker unreachable");
        }
      }
      doc.clear();
    }
        
    interrupt_flag = 0;
  }

}
