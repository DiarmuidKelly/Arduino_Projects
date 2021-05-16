#include "DHT.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"
#include "config.h"


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// MQTT config imported from config.h
const char* mqtt_server = mqtt_server_config;  // IP of the MQTT broker
const char* data_topic = "apartment/sensor1/data";
const char* mqtt_username = mqtt_username_config; // MQTT username
const char* mqtt_password = mqtt_password_config; // MQTT password
const char* clientID = "sensor1"; // MQTT client ID

// WIFI
WiFiClient wifi_client;
const char* ssid     = ssid_config;
const char* password = password_config;
unsigned long waitCount = 0;
uint8_t conn_stat = 0;
int connection_flag = 0;
unsigned long lastTask = 0;

// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifi_client);

#define DHTPIN 2
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);
const int photoresistor_pin = 34;
const int R_LED = 21;
const int G_LED = 22;
const int B_LED = 23;
const int relay_pin = 15;
const int push_button = 4;


int lightVal;
int relay_flag = 0;

const int capacity = JSON_OBJECT_SIZE(6);
StaticJsonDocument<capacity> doc;

hw_timer_t * timer = NULL;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int interrupt_flag = 0;


void IRAM_ATTR onTimer() {
  /*
   * Handle timer interrupt for reading photoresistor
   */
  interrupt_flag = 1;
  lightVal = analogRead(photoresistor_pin); // read the current light levels
}
void IRAM_ATTR connectionTimer() {
  /*
   * Handle timer to call wifi and MQTT setup connection_status() function
   */
  connection_flag = 1;
}

void setup() {
  /*
   * Initialise configuration
   */
  Serial.begin(115200);
  Serial.println(F("Starting DHT & Light thing"));

  ledcAttachPin(R_LED, 1);
  ledcAttachPin(G_LED, 2);
  ledcAttachPin(B_LED, 3);
  ledcSetup(1, 12000, 8);
  ledcSetup(2, 12000, 8);
  ledcSetup(3, 12000, 8);
  pinMode(relay_pin, OUTPUT);
  pinMode(push_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button), handlePushButton, FALLING); // trigger when button pressed, but not when released.

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &connectionTimer, true);
  timerAlarmWrite(timer1, 500000, true);
  timerAlarmEnable(timer1);

  WiFi.mode(WIFI_STA);
  // Initialize a NTPClient to get time
  timeClient.begin();

  dht.begin();
}

void led_control(int colour) {
  /*
      Sets the LED controller to print
      case 1: Red
      case 2: Green
      case 3: Blue
      case 4: Yellow

  */
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  switch (colour) {
    case 1:
      ledcWrite(1, 10);
      break;
    case 2:
      ledcWrite(2, 10);
      break;
    case 3:
      ledcWrite(3, 10);
      break;
    case 4:
      ledcWrite(1, 10);
      ledcWrite(2, 10);
      break;
  }
  return;
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
      led_control(1);
      Serial.println("WiFi down: start WiFi");
      WiFi.begin(ssid, password);
      conn_stat = 1;
      break;
    case 1:                                                       // WiFi starting, do nothing here
      led_control(4);
      Serial.println("WiFi starting, wait : " + String(waitCount));
      waitCount++;
      if (waitCount % 1000 == 0) {
        conn_stat = 0;
      }
      break;
    case 2:  // WiFi up, MQTT down: start MQTT
      led_control(3);
      Serial.println("WiFi up, MQTT down: start MQTT");
      if (client.connect(clientID, mqtt_username, mqtt_password)) {
        conn_stat = 3;
        timeClient.update();
        Serial.print("Formatted time configured: ");
        Serial.println(timeClient.getFormattedTime());
        waitCount = 0;
      }
      break;
    case 3:
      Serial.println("WiFi up, MQTT up: finished MQTT configuration");
      conn_stat = 5;
      led_control(2);
      break;
  }
}

void handlePushButton() {
  /* 
   *  Handle the interrupt for user pressing the push button
   */
  //  Serial.println("pushing button");
  if (relay_flag == 0) {
    relay_flag = 1;
    digitalWrite(relay_pin, HIGH);
  } else {
    relay_flag = 0;
    digitalWrite(relay_pin, LOW);
  }
  //  Serial.println(relay_flag);

}

void loop() {
  /* 
   *  Check if the connection needs to be checked. Handle this function
   *  Check if the Light sensor needs to be read based on timer interrupt flag
   *  Inside the latter if loop, check if 30 seconds has passed since sending data
   *  to MQTT. If so, send the data JSON to the MQTT broker.
   */
  if (connection_flag == 1){
    connection_status();
    connection_flag = 0;
  }

  if (interrupt_flag == 1) {
    if (conn_stat == 5) {
      led_control(0);
      Serial.println(lightVal);
      if (relay_flag == 0) {
        if (lightVal >= 3500) //If it's dark turn off relay. Open circuit
          digitalWrite(relay_pin, LOW);
        if (lightVal < 2800)
          digitalWrite(relay_pin, HIGH); // If it's bright. Close circuit
      }
      if (relay_flag == 1) {
        if (lightVal < 2800) // If it's getting light set flag 2
          relay_flag = 2;
      }
      if (relay_flag == 2) { // If it's getting dark set flag 0
        if (lightVal >= 3500)
          relay_flag = 0;
      }

      // PUBLISH to the MQTT Broker (topic = Temperature, defined at the beginning)
      if (millis() - lastTask > 30000) {   // Print message every 30 seconds
        lastTask = millis();
        led_control(4);
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


        if (client.connect(clientID, mqtt_username, mqtt_password)) {
          timeClient.update();
          String cur_time = timeClient.getFormattedTime();
          Serial.println(cur_time);

          doc["humidity"].set(h);
          doc["temp"].set(t);
          doc["heat_i"].set(hic);
          doc["light"].set(lightVal);
          doc["time"].set(cur_time);

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
              led_control(3);
            }
          }
          client.disconnect();  // disconnect from the MQTT broker
        }
        led_control(0);

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
    interrupt_flag = 0;
  }
}
