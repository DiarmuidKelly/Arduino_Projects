# Arduino_Projects

See related MQTT project for InfluxDB time series data records: [Project Repo](https://github.com/DiarmuidKelly/MQTT)

Repository of various Arduino projects mostly written for home automation.

| Project | Description |
| ----------- | ----------- |
| DHT_LDR | Read DHT11 sensor for temperature and humidity readings. Read Photosensor and control power relay based on cutoff threshold. Report readings to MQTT broker |
| H-PI | Read capacitive soil moisture sensors and control pump actuators. Report moisture % and pump statuses to MQTT broker. |

## Contents

* [Requirements](#Requirements)
* [H-PI Specific Requirements](#H-PI-Specific)
* [DHT_LDR Specific Requirements](#DHT_LDR-Specific)

## Requirements

ESP32 board support

```bash
https://dl.espressif.com/dl/package_esp32_index.json
```

### ArduinoJson

```bash
https://github.com/bblanchon/ArduinoJson
```

### NTPClient

```bash
https://github.com/arduino-libraries/NTPClient
```

### pubsubclient

```bash
https://github.com/knolleary/pubsubclient
```

#### Configuration

```PubSubClient.h```

```cpp
// MQTT_MAX_PACKET_SIZE : Maximum packet size. Override with setBufferSize().
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 1024
#endif
```

## H-PI Specific

### Misc

Reference for state managment in industrial control systems

```bash
https://en.wikipedia.org/wiki/Finite-state_machine
```

## DHT_LDR Specific

### DHT sensor Library - Adafruit

```bash
https://github.com/adafruit/DHT-sensor-library
```

**Dependency**: Adafruit Unified Sensor Driver

```bash
https://github.com/adafruit/Adafruit_Sensor
```
