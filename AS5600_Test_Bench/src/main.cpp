#include <Arduino.h>
#include <Wire.h>

// In order to set registers of the AS5600, set the PGO pin low  and SCL pin to GP5 and SDA to GP4 of the pi pico
// Setting the PGO pin high will make the output active


#define secAddr 0x36

void writeReg(uint8_t regAddr, uint8_t data) {
  Wire.beginTransmission(secAddr);
  Wire.write(regAddr);
  Wire.write(data);
  Wire.endTransmission();
}

byte readReg(uint8_t regAddr) {
  Wire.beginTransmission(secAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(secAddr, 1);
  byte value = Wire.read();
  Wire.endTransmission();
  return value;
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  delay(1000);
  byte currValue = readReg(0x8);
  currValue &= 0x0F;
  currValue |= 0xA0;
  writeReg(0x8, currValue);
}

void loop() {
  // put your main code here, to run repeatedly:
}