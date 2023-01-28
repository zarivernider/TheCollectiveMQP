#include <Arduino.h>
#include "AS5600.h"
AS5600 pwmRead(15, 0);
void setup() {
  // put your setup code here, to run once:
  pwmRead.init();
}

void loop() {
  // // put your main code here, to run repeatedly:
  // Serial.print("Count: ");
  Serial.println(pwmRead.getCtr());

  // Serial.print((uint32_t)&pwmRead.rawAngle, HEX);
  // Serial.print("\t");
  // Serial.print(*(io_rw_32*)(DMA_CH0_WRITE_ADDR + pwmRead.DMAoffset), HEX);
  // Serial.print("\t");
  // Serial.print(*(io_rw_32*)(DMA_CH0_READ_ADDR + pwmRead.DMAoffset), HEX);
  // Serial.print("\t");
  // Serial.print((PWM_BASE | (0x8 + pwmRead.pwmOffset)), HEX);
  // Serial.print("\t");
  // Serial.print(*(io_rw_32*)(PWM_BASE | (0x8 + pwmRead.pwmOffset)));
  // Serial.print("\t");
  // Serial.print(pwmRead.rawAngle);
  // Serial.print("\t");
  // Serial.println(*(io_rw_32*)(DMA_CH0_CTRL_TRIG + pwmRead.DMAoffset), HEX);
  delay(1000);
}