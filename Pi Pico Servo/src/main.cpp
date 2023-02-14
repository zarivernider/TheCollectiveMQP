#include <Arduino.h>
#include "servo.h"
#include "system_pi_pico.h"
Servo pwmTest(3);
void setup() {
  // put your setup code here, to run once:
  pwmTest.attach();

}

void loop() {
  // put your main code here, to run repeatedly:
  pwmTest.setServo(0);
  delay(5000);
  pwmTest.setServo(135); // Max angle for the gripper to rotate
  delay(5000);
}

