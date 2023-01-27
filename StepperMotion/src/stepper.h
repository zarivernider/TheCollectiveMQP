#ifndef stepper_h
#define stepper_h
#include <Arduino.h>
#include "system_pi_pico.h"
#include <DIO.h>
/*
  Servo class to manipulate servos where the max pulse width time in microseconds is less than 5000 uS. Angle range is modifiable. 
  Works for any GPIO pin
*/
class Stepper
{
private:
  #define microSteps 2 // The amount of micro steps that is configured for the driver
  #define stepsperRev 200 // The amount of steps it takes for one revolution 1 / degrees per step
  uint8_t pwmPin = 0;
//   uint8_t dirPin = 0;
//   uint8_t enPin = 0;
  DIO dirGPIO; 
  DIO enGPIO;
  uint16_t pwmOffset;
  uint16_t pwmioOffset;
  uint16_t dirioOffset;
  uint16_t enioOffset;
  uint32_t systemFreq;


public:
Stepper(uint8_t pwmPin, uint8_t directionPin, uint8_t enablePin) { // Set up servo for 500 - 2400 uS pulse width
    // Stepper::dirPin = directionPin; // set direction pin
    // Stepper::enPin = enablePin; // set enable pin
    dirGPIO = DIO(directionPin);
    enGPIO = DIO(enablePin);
    Stepper::pwmPin = pwmPin; // set pwm pin
  } 
  
//   ~Stepper() {
//     detach();
//   }
  void init();
  void enable(bool isEnable);
  void setSpeed(int16_t RPMspeed);
  void setDirection(bool isFwd);
  void brakeStop();


};

#endif