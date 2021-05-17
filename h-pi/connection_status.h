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
