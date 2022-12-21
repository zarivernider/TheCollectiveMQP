#include <Arduino.h>
#include "I2C_Primary.h"
I2C_M i2c_m;
void setup() {
  // put your setup code here, to run once:
  i2c_m.init();
  Serial.begin(9600);
  delay(10000);
  Serial.println("INIT DONE");
}

void loop() {
  // Serial.print("Read val at Addr 1 ");
  // int out = i2c_m.readInt(1, 0x13);
  // Serial.println(out);
  Serial.println("\t Write 4548 to addr 1\t");
  i2c_m.writeInt(4548, 1, 0x13);
  
  // out = i2c_m.readInt(1, 0x13);
  // Serial.print("written ");
  // Serial.print(out);

  // out = i2c_m.readInt(0, 0x13);
  // Serial.print("Addr 0:  ");
  // Serial.println(out);

  // delay(5000);
  // i2c_m.writeInt(0, 1, 0x13);
  // Serial.println("Cleared");


  delay(5000);
}

