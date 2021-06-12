#include "config.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include "PubSubClient.h"

/*
* Enable Test mode
*/

int test_mode = 0;

//---------------------------------------

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", time_zone, 60000);

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
int pump_timeout = pump_timeout_config;

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
const int pump1 = 14;
const int pump2 = 27;
const int pump3 = 26;
const int pump4 = 25;
int pump1_timeout = 0;
int pump2_timeout = 0;
int pump3_timeout = 0;
int pump4_timeout = 0;
int pump_threshold = 60;
const int moisture_sensor1 = 32;
const int moisture_sensor2 = 35;
const int moisture_sensor3 = 34;
int AirValue1 = 3300;
int AirValue2 = 3200;
int AirValue3 = 2800;
int WaterValue1 = 1200;
int WaterValue2 = 1000;
int WaterValue3 = 1000;
int p1_delay = 10000; // Pump delays (milliseconds)
int p2_delay = 10000;
int p3_delay = 2000;
int p4_delay = 4000;
int soilmoisturepercent = 0;
const int push_button = 12;
const int indicator_led = 21;

/*
* Interrupts
*/
String cur_time;
hw_timer_t *timer = NULL;
hw_timer_t *timer_pump_timeout = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int interrupt_flag = 1;
int relay_flag = 0;

void IRAM_ATTR onTimer()
{
  /*
   * Handle interrupt flag for reading photoresistor
   */
  interrupt_flag = 1;
}

void IRAM_ATTR pumpTimer()
{
  /*
   * 
   */
  if(pump1_timeout > 0){
    pump1_timeout--;
  }
  if(pump2_timeout > 0){
    pump2_timeout--;
  }
  if(pump3_timeout > 0){
    pump3_timeout--;
  }
  if(pump4_timeout > 0){
    pump4_timeout--;
  }
}

void setup()
{

  Serial.begin(115200);
  Serial.println(F("H-PI - Human-Plant Interface"));
  Serial.println(clientID);

  /*
  * Set tags and measurement fields
  */
  tags["host"].set(thing_id);
  tags["region"].set(region);
  serializeJson(tags, JSONmessageBuffer);

  doc["measurement"] = measurement;

  /*
  * Configure pump pins
  */
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(pump3, OUTPUT);
  pinMode(pump4, OUTPUT);
  pinMode(pump1, OUTPUT);
  pinMode(indicator_led, OUTPUT);

  /*
  * Configure sensors pins
  */
  pinMode(moisture_sensor1, INPUT);
  pinMode(moisture_sensor2, INPUT);
  pinMode(moisture_sensor3, INPUT);

  /*
  * Push button pin 
  */
  pinMode(push_button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(push_button), handlePushButton, FALLING); // trigger when button pressed (pullup), but not when released.

  /*
  * Configure Timer
  */
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, interr_time_rate, true);
  timerAlarmEnable(timer);

  timer_pump_timeout = timerBegin(1, 80, true);
  timerAttachInterrupt(timer_pump_timeout, &pumpTimer, true);
  timerAlarmWrite(timer_pump_timeout, 1000000, true); // Every second
  timerAlarmEnable(timer_pump_timeout);

  /*
  * Configure Wifi
  */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  /*
  * Misc.
  */
  pumps_off();
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
  AirValue1 = obj["AT1"];
  AirValue2 = obj["AT2"];
  AirValue3 = obj["AT3"];
  WaterValue1 = obj["LT1"];
  WaterValue2 = obj["LT2"];
  WaterValue3 = obj["LT3"];
  relay_flag = obj["Pump_on"];
  p1_delay = obj["PD1"];
  p2_delay = obj["PD2"];
  p3_delay = obj["PD3"];
  p4_delay = obj["PD4"];
  interr_time_rate = obj["UF"];
  pump_timeout = obj["PTO"];
  pump_threshold = obj["PTV"];
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

void handlePushButton()
{
  /* 
   *  Handle the interrupt for user pressing the push button
   */

  if (relay_flag == 0)
  {
    relay_flag = 1;
  }
}

void pumps_off()
{
  digitalWrite(pump1, HIGH);
  digitalWrite(pump2, HIGH);
  digitalWrite(pump3, HIGH);
  digitalWrite(pump4, HIGH);
}

void handlePumps_dummy()
{
  digitalWrite(indicator_led, HIGH);
  Serial.println("pump_dummy");
  delay(2000);
  pump1_timeout = pump_timeout;
  pump2_timeout = pump_timeout;
  pump3_timeout = pump_timeout;
  pump4_timeout = pump_timeout;
  fields["p1"].set(1);
  fields["p2"].set(1);
  fields["p3"].set(1);
  fields["p4"].set(1);
  Serial.println("pump_dummy");
  pumps_off();
}

void handlePumps()
{
  Serial.println("pump starting");
  pump_select(pump1, p1_delay, &pump1_timeout);
  pump_select(pump2, p2_delay, &pump2_timeout);
  pump_select(pump3, p3_delay, &pump3_timeout);
  pump_select(pump4, p4_delay, &pump4_timeout);
  fields["p1"].set(1);
  fields["p2"].set(1);
  fields["p3"].set(1);
  fields["p4"].set(1);
  Serial.println("pump finished");
  pumps_off();
}

void pump_select(int32_t pump, int32_t pump_time, int* pump_timeout_val)
{
  digitalWrite(pump, LOW);
  delay(pump_time);
  *pump_timeout_val = pump_timeout;
  pumps_off();
}

void loop()
{
  connection_status();
  client.loop();
  if (relay_flag == 1)
  {
    if (test_mode == 1)
    {
      handlePumps_dummy();
    }
    else
    {
      handlePumps();
    }
    relay_flag = 0;
  }
  if (interrupt_flag == 1)
  {
    if (client.connect(clientID, mqtt_username, mqtt_password))
    {
      timeClient.update();
      cur_time = timeClient.getFormattedTime();
      doc["time"] = cur_time;

      soilmoisturepercent = map(analogRead(moisture_sensor1), AirValue1, WaterValue1, 0, 100);
      Serial.print(soilmoisturepercent);
      Serial.print(",");
      fields["ss1"].set(soilmoisturepercent);

      soilmoisturepercent = map(analogRead(moisture_sensor2), AirValue2, WaterValue2, 0, 100);
      Serial.print(soilmoisturepercent);
      Serial.print(",");
      fields["ss2"].set(soilmoisturepercent);

      soilmoisturepercent = map(analogRead(moisture_sensor3), AirValue3, WaterValue3, 0, 100);
      Serial.println(soilmoisturepercent);
      fields["ss3"].set(soilmoisturepercent);

      fields["p1"].set(int(fields["p1"]));
      fields["p2"].set(int(fields["p2"]));
      fields["p3"].set(int(fields["p3"]));
      fields["p4"].set(int(fields["p4"]));

      serializeJson(doc, JSONmessageBuffer);
      if (client.publish(data_topic, JSONmessageBuffer))
      {
        Serial.println("Data sent!");
        int t = fields["p1"];
        Serial.println(t);
        fields["p1"].set(0);
        fields["p2"].set(0);
        fields["p3"].set(0);
        fields["p4"].set(0);
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
        fields["p1"].set(0);
        fields["p2"].set(0);
        fields["p3"].set(0);
        fields["p4"].set(0);
      }

      if (fields["ss1"] < pump_threshold && pump3_timeout == 0){
        if (test_mode == 1)
        {
          handlePumps_dummy();
        }
        else
        {
            pump_select(pump3, p3_delay, &pump3_timeout);
        }
        fields["p3"].set(1);
      }
      if (fields["ss2"] < pump_threshold && pump4_timeout == 0){
        if (test_mode == 1)
        {
          handlePumps_dummy();
        }
        else
        {
        pump_select(pump4, p4_delay, &pump4_timeout);
        }
        fields["p4"].set(1);
      }
      if (fields["ss3"] < pump_threshold && pump1_timeout == 0 && pump2_timeout == 0){
        if (test_mode == 1)
        {
          handlePumps_dummy();
        }
        else
        {
        pump_select(pump1, p1_delay, &pump1_timeout);
        pump_select(pump2, p2_delay, &pump2_timeout);
        }
        fields["p1"].set(1);
        fields["p2"].set(1);
      }
      Serial.println(pump1_timeout);
      Serial.println(pump2_timeout);
      Serial.println(pump3_timeout);
      Serial.println(pump4_timeout);


      doc["time"] = 0;
    }
    interrupt_flag = 0;
  }
}
