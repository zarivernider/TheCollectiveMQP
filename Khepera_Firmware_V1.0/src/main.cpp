#include <Arduino.h>
#include "ADC.h"
#include "AS5600.h"
#include "I2C_Peripheral.h"
#include "servo.h"
#include "stepper.h"
#include "APA102_Custom.h"

// Global definitions
#define numbLEDsRing 8 // Number of LEDs in the ring
#define I2C_Address 0x24 // Address of the I2C module 
#define openGripperAngle 90 // degrees
#define closeGripperAngle 0 // degrees
// Pin assignments
#define stepperDirPin 0
#define stepperPWMPin 1
#define stepperEnablePin 2
#define I2C0SCLPin 4
#define I2C0SDAPin 5
#define SPICLKPin 6
#define SPIMOSIPin 7
#define testLEDPin 14
#define encPWMReadPin 15
#define stepperUARTPin 21
#define stepperS1Pin 21
#define stepperS2Pin 20
#define groundADCchannel 1
#define yforceADCchannel 0
#define xforceADCchannel 2
#define gripperPin 3
// I2C Register map


// Class initialization
APA102 stringLEDs(numbLEDsRing);
ADC adc(0b111, 0); // Activate all channels and DMA channel 0
AS5600 absoluteEncoder(encPWMReadPin);
DIO extLED(testLEDPin);
I2C_P i2c_p;
Servo gripper(gripperPin);
Stepper motor(stepperPWMPin, stepperDirPin, stepperEnablePin); 
// ToDo: Stepper programmer. Sets SS pins and UART for controlling TMC


// Global Variables

void setup() {
  // Initialize classes 
  stringLEDs.Init(); // LEDs first. 4 SPI pins are assigned, but 2 are needed. Others will be reassigned. 
  gripper.attach();
  motor.init();
  extLED.init();
  absoluteEncoder.init();
  i2c_p.init(I2C_Address);
  adc.initMulti();

}

void loop() {
  // put your main code here, to run repeatedly:
  
}