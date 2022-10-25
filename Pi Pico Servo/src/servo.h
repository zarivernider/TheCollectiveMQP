#ifndef servo_h
#define servo_h
#include <Arduino.h>
#include "system_pi_pico.h"
/*
  Servo class to manipulate servos where the max pulse width time in microseconds is less than 5000 uS. Angle range is modifiable. 
  Works for any GPIO pin
*/
class Servo
{
private:
  uint16_t minCount = 0; // min count for pulse width
  uint16_t maxCount = 0; // max count for pulse width
  float maxAngle = 180; // max angle for servo
  float minAngle = 0; // min angle for servo
  bool isattach = false; // Is servo attached
  uint8_t pwmPin = 0;


public:
Servo(uint8_t pwmPin) { // Set up servo for 500 - 2400 uS pulse width
    // Clock is 105 nS per tick. Convert input from uS to nS and get clock tick. 
    minCount = 4762; // set minimum time for 500 uS
    maxCount = 22857; // set maximum time for 2400 uS
    Servo::pwmPin = pwmPin; // set pwm pin
  } 
  
  Servo(uint8_t pwmPin, float usMin, float usMax) { // Set up the servo a custom pulse width time
    // Clock is 105 nS per tick. Convert input from uS to nS and get clock tick. 
    minCount = ceil((usMin*1000) / 105); // ceil to not be less than minimum
    maxCount = floor((usMax*1000) / 105); // floor to not be more than maximum
    Servo::pwmPin = pwmPin; // set pwm pin
  } 
  Servo(uint8_t pwmPin, float usMin, float usMax, float minAngle, float maxAngle) { // Set up the servo for a custom pulse width and angle range
    // Clock is 105 nS per tick. Convert input from uS to nS and get clock tick. 
    minCount = ceil((usMin*1000) / 105); // ceil to not be less than minimum
    maxCount = floor((usMax*1000) / 105); // floor to not be more than maximum
    Servo::pwmPin = pwmPin; // set pwm pin
    Servo::minAngle = minAngle; // set min servo angle
    Servo::maxAngle = maxAngle; // set max servo angle
  } 
  ~Servo() {
    detach();
  }
  void attach();
  void detach();
  void setMinMax(uint16_t usMin, uint16_t usMax);
  void setServo(uint16_t degrees);
};

#endif