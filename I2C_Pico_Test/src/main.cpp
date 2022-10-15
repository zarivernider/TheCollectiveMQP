#include <Arduino.h>
#include "I2C_Primary.h"
I2C_M i2c_m;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(20000);
}

void loop() {
  Serial.print("Read val ");
  int out = i2c_m.readI2CInt(1, 0x13);
  Serial.print(out);
  Serial.print("\t Write 452\t");
  i2c_m.writeI2CInt(4548, 1, 0x13);
  
  out = i2c_m.readI2CInt(1, 0x13);
  Serial.print("written ");
  Serial.println(out);
  delay(5000);
  i2c_m.writeI2CInt(0, 1, 0x13);
  Serial.println("Cleared");
 
}

