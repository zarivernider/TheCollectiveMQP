#include <Arduino.h>
#include <stepper.h>

Stepper motor(1, 2, 3);

void setup() {
  // put your setup code here, to run once:
  motor.init();
  // delay(5000);
  motor.setSpeed(375);
  // delay(5000);
  // motor.brakeStop();
  // delay(5000);
  // motor.setDirection(true);
}

void loop() {
  // // put your main code here, to run repeatedly:
  // motor.enable(true);
  // // motor.setDirection(true);
  // delay(1000);
  // motor.enable(false);
  // delay(1000);
}