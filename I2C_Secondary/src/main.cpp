#include <Arduino.h>
#include <Wire.h>
#include "system_pi_pico.h"
#include "I2C_Peripheral.h"

I2C_P i2c_p;
int baseAngle = 0;
int forceSensor = 20;

void setup() {
  i2c_p.arrMap[0] = &baseAngle; // Set baseAngle as the 0 address
  i2c_p.arrMap[1]  = &forceSensor; // Set forceSensor as the 1 address
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

