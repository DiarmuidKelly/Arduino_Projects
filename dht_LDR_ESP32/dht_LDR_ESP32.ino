#include "DHT.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
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
WiFiClient wifiClient;
const char* ssid     = ssid_config;
const char* password = password_config;

// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient); 
//Set up some global variables for the light level an initial value.
//cosntants for the pins where sensors are plugged into.
const int sensorPin = 34;
const int R_LED = 21;
const int G_LED = 22;
const int B_LED = 23;
int lightVal;   // light reading


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
  
  ledcWrite(1, 10);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  ledcWrite(2, 10);

  // Initialize a NTPClient to get time
  timeClient.begin();

  dht.begin();

  // Connect to MQTT Broker
  // client.connect returns a boolean value to let us know if the connection was successful.
  // If the connection is failing, make sure you are using the correct MQTT Username and Password (Setup Earlier in the Instructable)
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
  ledcWrite(1, 0);


}

void loop() {
  delay(2000);
  timeClient.update();
  String cur_time = timeClient.getFormattedTime();
  Serial.println(cur_time);
  doc["time"].set(cur_time);
  
  // Wait a few seconds between measurements.
  lightVal = analogRead(sensorPin); // read the current light levels
  ledcWrite(2, 0);


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


  // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
  ledcWrite(2, 10);
  ledcWrite(1, 10);
  
  char JSONmessageBuffer[capacity];
  serializeJson(doc, JSONmessageBuffer);
  
  client.connect(clientID, mqtt_username, mqtt_password);
  if (client.publish(data_topic, JSONmessageBuffer)) {
    Serial.println("Data sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Data failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(data_topic, JSONmessageBuffer);
  }
  client.disconnect();  // disconnect from the MQTT broker
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  delay(1000*30);       // print new values every 30 seconds


  
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
