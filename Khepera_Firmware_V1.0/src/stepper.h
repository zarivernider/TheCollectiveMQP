#ifndef stepper_h
#define stepper_h
#include <Arduino.h>
#include "system_pi_pico.h"
#include <DIO.h>
/*
  Servo class to manipulate servos where the max pulse width time in microseconds is less than 5000 uS. Angle range is modifiable. 
  Works for any GPIO pin
*/
#define maxFreq 17000
#define minFreq 16
#define systemFreq 1000000
class Stepper
{
private:
  #define microSteps 2 // The amount of micro steps that is configured for the driver
  #define stepsperRev 200 // The amount of steps it takes for one revolution 1 / degrees per step
  #define startMicroSteps 16 // Initial setting for the amount of microsteps
  #define regOTPaddress 0x4 // Address for the flash memory in the TMC 
  #define TMCaddress 0x0  // Address to access the TMC peripheral 
  #define transmissionBytes 8 // number of bytes in UART transmission
  #define regConfaddress 0x0
  uint8_t pwmPin = 0;
//   uint8_t dirPin = 0;
//   uint8_t enPin = 0;
  DIO dirGPIO; 
  DIO enGPIO;
  DIO MS1;
  DIO MS2;
  uint16_t pwmOffset;
  uint16_t pwmioOffset;
  uint16_t dirioOffset;
  uint16_t enioOffset;
  uint8_t UARTpin;


public:
Stepper(uint8_t pwmPin, uint8_t directionPin, uint8_t enablePin, uint8_t MS1pin, uint8_t MS2pin, uint8_t TXpin) { // Set up servo for 500 - 2400 uS pulse width
    // Stepper::dirPin = directionPin; // set direction pin
    // Stepper::enPin = enablePin; // set enable pin
    dirGPIO = DIO(directionPin);
    enGPIO = DIO(enablePin);
    MS1 = DIO(MS1pin);
    MS2 = DIO(MS2pin);
    UARTpin = TXpin;
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
  void setDirFreq(int16_t freq);
  void setFreq(uint16_t freq);
  void setMicroSteps(uint8_t setting);
  void writeAddress(uint8_t regAddres, uint32_t message);
  uint8_t calcCRC(uint8_t* datagram, uint8_t datagramLength);
  void readAddress(uint8_t regAddress);
};

#endif