#include <Arduino.h>
#include "I2C_Peripheral.h"

I2C_P i2c_p;
int baseAngle = 0;
int forceSensor = 0x3476;

void setup() {
  i2c_p.arrMap[0] = &baseAngle; // Set baseAngle as the 0 address
  i2c_p.arrMap[1]  = &forceSensor; // Set forceSensor as the 1 address
  // put your setup code here, to run once:
  i2c_p.init(0x15);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_SAR_OFFSET), HEX);
  // Serial.println(irq_is_enabled(23));
  if(i2c_p.mailBox) {
    Serial.print(i2c_p.status, HEX);
    Serial.print("\t Address: ");
    Serial.print(i2c_p.globAddr);
    Serial.print("\t receive: ");
    Serial.print(i2c_p.rec);
    Serial.print("\t Force sensor: ");
    Serial.println(forceSensor);

    i2c_p.mailBox = false;
  }
  delay(3000);
}