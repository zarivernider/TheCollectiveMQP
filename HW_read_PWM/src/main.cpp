#include <Arduino.h>
#include "AS5600.h"
AS5600 pwmRead(15, 15);
void setup() {
  // put your setup code here, to run once:
  pwmRead.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pwmRead.getCtr());
  delay(1000);
}