#include "config.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h> 
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"


//---------------------------------------

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", (int)time_zone, 60000);

// MQTT config imported from config.h
const char *mqtt_server = mqtt_server_config;     
const char *mqtt_username = mqtt_username_config;
const char *mqtt_password = mqtt_password_config; 
const char *data_topic = data_topic_name;
const char *config_topic = config_topic_name;
const char *clientID = thing_id;
const char *region = thing_region;
char *measurement = thing_measurement;
int interr_time_rate = interr_time_rate_config;


/*
* Wireless config
*/
WiFiClient wifi_client;
const char *ssid = ssid_config;
const char *password = password_config;
uint8_t conn_stat = 0;
unsigned long waitCount = 0;

PubSubClient client(mqtt_server, 1883, wifi_client);


/*
* JSON objects
*/
const int capacity = JSON_OBJECT_SIZE(16);
StaticJsonDocument<capacity> doc;
char JSONmessageBuffer[capacity];
JsonObject tags = doc.createNestedObject("tags");
JsonObject fields = doc.createNestedObject("fields");

/*
 * Sensor/Actuator configurations
 */
const int PIN_TO_SENSOR =  26;
const int LED =  19;
int pinStateCurrent   = LOW;
int pinStatePrevious  = LOW;


/*
* Interrupts
*/
String cur_time;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int interrupt_flag = 1;
int send_flag = 0;

void IRAM_ATTR onTimer()
{
  /*
   * Handle interrupt flag for reading photoresistor
   */
  interrupt_flag = 1;
}

void setup() {
  Serial.begin(115200);
  /*
  * Set tags and measurement fields
  */
  tags["host"].set(thing_id);
  tags["region"].set(region);

  doc["measurement"] = measurement;

  /*
  * Configure Pins
  */

  pinMode(PIN_TO_SENSOR, INPUT);
  pinMode(LED, OUTPUT);


  /*
  * Configure Timer
  */
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, interr_time_rate, true);
  timerAlarmEnable(timer);

  /*
  * Configure Wifi
  */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

}

void message_callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
  DynamicJsonDocument in_doc(1024);
  deserializeJson(in_doc, messageTemp);
  JsonObject obj = in_doc.as<JsonObject>();
  String received = obj["fields"];
  deserializeJson(in_doc, received);
  obj = in_doc.as<JsonObject>();
  /*
  * Set config from message
  */
  int test = obj["test"];
  Serial.println(test);
  interr_time_rate = obj["UF"]; 
}
void connection_status()
{
  /* 
   *  Handle different WIFI statuses, and MQTT connection.
   */
  if ((WiFi.status() != WL_CONNECTED) && (conn_stat != 1))
  {
    conn_stat = 0;
  }
  if ((WiFi.status() == WL_CONNECTED) && (conn_stat <= 1))
  {
    conn_stat = 2;
  }
  switch (conn_stat)
  {
  case 0: // MQTT and WiFi down: start WiFi
    Serial.println("WiFi down: start WiFi");
    WiFi.begin(ssid, password);
    conn_stat = 1;
    break;
  case 1: // WiFi starting, do nothing here
    Serial.println("WiFi starting, wait : " + String(waitCount));
    waitCount++;
    if (waitCount % 1000 == 0)
    {
      conn_stat = 0;
    }
    break;
  case 2: // WiFi up, MQTT down: start MQTT
    if (!client.connected())
    {
      Serial.println("WiFi up, MQTT down: start MQTT");
      if (client.connect(clientID, mqtt_username, mqtt_password))
      {
        conn_stat = 3;
        client.setServer(mqtt_server, 1883);
        client.setCallback(message_callback);
        client.subscribe(config_topic);

        timeClient.update();
        Serial.print("Formatted time configured: ");
        Serial.println(timeClient.getFormattedTime());
        waitCount = 0;
      }
    }
    else
    {
      conn_stat = 3;
    }
    break;
  case 3:
    Serial.println("WiFi up, MQTT up: finished MQTT configuration");
    conn_stat = 5;
    break;
  }
  delay(10);
}

void loop() {
  connection_status();
  client.loop();

  pinStatePrevious = pinStateCurrent;
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);
  Serial.println(pinStateCurrent);
  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {
    Serial.println("Motion detected!");
    fields["motion"].set(1);
    fields["time"].set(timeClient.getFormattedTime());
    digitalWrite(LED, HIGH);
    send_flag = 1;
  }
  else if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {  
    Serial.println("Motion stopped!");
    fields["motion"].set(0);
    fields["  "].set(timeClient.getFormattedTime());
    digitalWrite(LED, LOW);
    send_flag = 1;
  }
  if (send_flag == 1)
  {
    if (client.connect(clientID, mqtt_username, mqtt_password))
    {
      timeClient.update();
      cur_time = timeClient.getFormattedTime();
      doc["time"] = cur_time;

      serializeJson(doc, JSONmessageBuffer);
      if (client.publish(data_topic, JSONmessageBuffer))
      {
        Serial.println("Data sent!");
      }
      // Again, client.publish will return a boolean value depending on whether it succeeded or not.
      // If the message failed to send, we will try again, as the connection may have broken.
      else
      {
        Serial.println("Data failed to send. Reconnecting to MQTT Broker and trying again");
        if (client.connect(clientID, mqtt_username, mqtt_password))
        {
          delay(50); // This delay ensures that client.publish doesn't clash with the client.connect call
          client.publish(data_topic, JSONmessageBuffer);
        }
        else
        {
          Serial.println("Data failed to send. MQTT broker unreachable");
        }
      }
      fields["motion"].set(-1);
      fields["time"].set(-1);
      doc["time"] = 0;
    }
    interrupt_flag = 0;
    send_flag = 0;
  }
}
