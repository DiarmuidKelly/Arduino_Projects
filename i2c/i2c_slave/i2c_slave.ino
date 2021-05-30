#include <Wire.h>

#define I2C_SDA 23
#define I2C_SCL 22

void setup(){
  Serial.begin(15200);
  Serial.println("I2C test");
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.onRequest(requestEvent);
  
}

void loop(){
  delay(100);
}

void requestEvent(){
  Wire.write("hello ");
}