#include <Arduino.h>
#include "servo.h"
#include "system_pi_pico.h"
Servo pwmTest(28);
void setup() {
  // put your setup code here, to run once:
  pwmTest.attach();

}

void loop() {
  // put your main code here, to run repeatedly:
  pwmTest.setServo(45);

}

