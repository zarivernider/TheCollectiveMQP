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
  #define clockDivider 10
  #define clockTOP 62499
  #define clockTickuS ((float)clockDivider / 125) // Calculate clock tick period in uS
  #define presetMinuS 500 // Set undefined minimum time in microseconds
  #define presetMaxuS 2400 // Set undefined max time in microseconds
public:
Servo(uint8_t pwmPin) { // Set up servo for 500 - 2400 uS pulse width
    minCount = ceil( (float)presetMinuS / clockTickuS); // ceil to not be less than minimum
    maxCount = floor( (float)presetMaxuS / clockTickuS); // floor to not be more than maximum
    Servo::pwmPin = pwmPin; // set pwm pin
  } 
  
  Servo(uint8_t pwmPin, float usMin, float usMax) { // Set up the servo a custom pulse width time
    minCount = ceil( (float)usMin / clockTickuS); // ceil to not be less than minimum
    maxCount = floor( (float)usMax / clockTickuS); // floor to not be more than maximum
    Servo::pwmPin = pwmPin; // set pwm pin
  } 
  Servo(uint8_t pwmPin, float usMin, float usMax, float minAngle, float maxAngle) { // Set up the servo for a custom pulse width and angle range
    minCount = ceil( (float)usMin / clockTickuS); // ceil to not be less than minimum
    maxCount = floor( (float)usMax / clockTickuS); // floor to not be more than maximum
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