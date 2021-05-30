#include <Wire.h>

#define I2C_SDA 23
#define I2C_SCL 22

void setup(){
  Serial.begin(115200);
  Serial.println("I2C test");
  Wire.begin(I2C_SDA, I2C_SCL);
  
}

void loop(){
  Wire.requestFrom(I2C_SDA, I2C_SCL);
  while (Wire.available()){
    char c = Wire.read();
    Serial.print(c);
  }
  Serial.println("Hi :)");
  delay(500);
}
