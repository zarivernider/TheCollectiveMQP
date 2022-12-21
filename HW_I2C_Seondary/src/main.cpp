#include <Arduino.h>
#include "I2C_Peripheral.h"

I2C_P i2c;
void setup() {
  // put your setup code here, to run once:
  i2c.init(0x13);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_SAR_OFFSET), HEX);
  // Serial.println(irq_is_enabled(23));
  delay(3000);
}