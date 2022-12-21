#include <Arduino.h>
#include "I2C_Peripheral.h"

I2C_P i2c;
void setup() {
  // put your setup code here, to run once:
  i2c.init(0x13);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(i2c.isRead()) i2c.read();
  if(i2c.isWrite()) i2c.write(0x23);
}