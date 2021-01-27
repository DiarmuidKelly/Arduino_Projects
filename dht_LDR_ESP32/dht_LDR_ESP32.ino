#include "DHT.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>                // needed for the WiFi communication
#include <WiFiUdp.h>
#include "PubSubClient.h" // Connect and publish to the MQTT broker
#include "config.h"

#define DHTPIN 2
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// MQTT
const char* mqtt_server = mqtt_server_config;  // IP of the MQTT broker
const char* data_topic = "apartment/sensor1/data";
const char* mqtt_username = mqtt_username_config; // MQTT username
const char* mqtt_password = mqtt_password_config; // MQTT password
const char* clientID = "sensor1"; // MQTT client ID

// WIFI
WiFiClient wifi_client;
const char* ssid     = ssid_config;
const char* password = password_config;
unsigned long waitCount = 0;                 // counter
uint8_t conn_stat = 0;
unsigned long lastStatus = 0;                // counter in example code for conn_stat == 5
unsigned long lastTask = 0;                  // counter in example code for conn_stat <> 5

// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifi_client); 
//Set up some global variables for the light level an initial value.
//cosntants for the pins where sensors are plugged into.
const int sensorPin = 34;
const int R_LED = 21;
const int G_LED = 22;
const int B_LED = 23;
int lightVal;   // light reading
int relay_pin = 15;
int push_button = 4;

int relay_flag = 0;

const int capacity = JSON_OBJECT_SIZE(6);
StaticJsonDocument<capacity> doc;


void setup() {
  
  Serial.begin(115200);
  Serial.println(F("DHTxx test!"));
  // Begin lights
  
  ledcAttachPin(R_LED, 1);
  ledcAttachPin(G_LED, 2);
  ledcAttachPin(B_LED, 3);
  ledcSetup(1, 12000, 8);
  ledcSetup(2, 12000, 8);
  ledcSetup(3, 12000, 8);
  pinMode(relay_pin, OUTPUT);
  pinMode(push_button, INPUT);


  WiFi.mode(WIFI_STA);

//  Serial.print("Connecting to ");
//  Serial.println(ssid);
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//    WiFi.begin(ssid, password);
//  }
//  // Print local IP address and start web server
//  Serial.println("");
//  Serial.println("WiFi connected.");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());

  // Initialize a NTPClient to get time
  timeClient.begin();

  dht.begin();

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
//  if (client.connect(clientID, mqtt_username, mqtt_password)) {
//    Serial.println("Connected to MQTT Broker!");
//  }
//  else {
//    Serial.println("Connection to MQTT Broker failed...");
//  }


}

void loop() {
  if ((WiFi.status() != WL_CONNECTED) && (conn_stat != 1)) { conn_stat = 0; }
//  if ((WiFi.status() == WL_CONNECTED) && !client.connected() && (conn_stat != 3))  { conn_stat = 2; }
  if ((WiFi.status() == WL_CONNECTED) && (conn_stat != 5)) { conn_stat = 4;}
  switch (conn_stat) {
    case 0:                                                       // MQTT and WiFi down: start WiFi
      ledcWrite(1, 10);
      Serial.println("MQTT and WiFi down: start WiFi");
      WiFi.begin(ssid, password);
      conn_stat = 1;      
      break;
    case 1:                                                       // WiFi starting, do nothing here
      ledcWrite(2, 10);
      ledcWrite(1, 10);
      Serial.println("WiFi starting, wait : "+ String(waitCount));
      waitCount++;
      if (waitCount % 1000 == 0){
        conn_stat = 0;
      }
      break;
    case 2:                                                       // WiFi up, MQTT down: start MQTT
      ledcWrite(2, 10);
      ledcWrite(1, 10);
      Serial.println("WiFi up, MQTT down: start MQTT");
      client.connect(clientID, mqtt_username, mqtt_password);
      conn_stat = 3;
      waitCount = 0;
      break;
    case 3:
      ledcWrite(2, 10);
      ledcWrite(1, 10);// WiFi up, MQTT starting, do nothing here
      Serial.println("WiFi up, MQTT starting, wait : "+ String(waitCount));
      waitCount++;
      if (waitCount % 1000 == 0){
        conn_stat = 2;
      }
      break;
    case 4:                                                       // WiFi up, MQTT up: finish MQTT configuration
      Serial.println("WiFi up, MQTT up: finish MQTT configuration");
      client.connect(clientID, mqtt_username, mqtt_password);
      //mqttClient.subscribe(output_topic);
      //mqttClient.publish(input_topic, Version);
      conn_stat = 5;      
      ledcWrite(1, 0);              
      ledcWrite(2, 0);              
      break;
  }
  if (conn_stat == 5){        
    delay(500);
    int push_button_state = digitalRead(push_button);
    // If the button is pressed
    if(push_button_state == HIGH){
      // If the flag is 0 set it to skip the light sensor, else set to 0 to hit light_sensor
      Serial.println("pushing button");
      if(relay_flag == 0){
        relay_flag = 1;
        digitalWrite(relay_pin, HIGH);
      } else{
        relay_flag = 0;
       }
      Serial.println(relay_flag); 
    }
    ledcWrite(2, 0);
    lightVal = analogRead(sensorPin); // read the current light levels
    Serial.println(lightVal);
    if(relay_flag == 0){
      if(lightVal>=3500) //If it's dark turn off relay. Open circuit
        digitalWrite(relay_pin, LOW);
      if(lightVal<3000)
        digitalWrite(relay_pin, HIGH); // If it's bright. Close circuit
    }
    if(relay_flag == 1){
      if(lightVal<3000) // If it's getting light set flag 2
        relay_flag = 2;
    }
    if(relay_flag == 2){ // If it's getting dark set flag 0
      if(lightVal>=3500)
        relay_flag = 0;
    }

    // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
    if (millis() - lastTask > 30000) {                                 // Print message every 30 seconds
      Serial.println("print this every 30 seconds");
      lastTask = millis();
      ledcWrite(2, 10);
      ledcWrite(1, 10);
      client.connect(clientID, mqtt_username, mqtt_password);
      delay(100);
      
      timeClient.update();
      String cur_time = timeClient.getFormattedTime();
      Serial.println(cur_time);
      doc["time"].set(cur_time);
      
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float h = dht.readHumidity();
      // Read temperature as Celsius (the default)
      float t = dht.readTemperature();
      // Read temperature as Fahrenheit (isFahrenheit = true)
      float f = dht.readTemperature(true);
    
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
    
      // Compute heat index in Fahrenheit (the default)
      float hif = dht.computeHeatIndex(f, h);
      // Compute heat index in Celsius (isFahreheit = false)
      float hic = dht.computeHeatIndex(t, h, false);
    
      doc["humidity"].set(h);
      doc["temp"].set(t);
      doc["heat_i"].set(hic);
      doc["light"].set(lightVal);
      
      char JSONmessageBuffer[capacity];
      serializeJson(doc, JSONmessageBuffer);
      
      if (client.publish(data_topic, JSONmessageBuffer)) {
        Serial.println("Data sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else {
        Serial.println("Data failed to send. Reconnecting to MQTT Broker and trying again");
        client.connect(clientID, mqtt_username, mqtt_password);
        delay(100); // This delay ensures that client.publish doesn't clash with the client.connect call
        client.publish(data_topic, JSONmessageBuffer);
      }
      client.disconnect();  // disconnect from the MQTT broker
      ledcWrite(1, 0);
      ledcWrite(2, 0);    
      
      Serial.print(F("Humidity: "));
      Serial.print(h);
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.print(F("°C "));
      Serial.print(f);
      Serial.print(F("°F  Heat index: "));
      Serial.print(hic);
      Serial.print(F("°C "));
      Serial.print(hif);
      Serial.println(F("°F"));
      Serial.println(lightVal);
    }
  }
}