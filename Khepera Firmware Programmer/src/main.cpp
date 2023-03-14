#include <Arduino.h>
#include "ADC.h"
#include "AS5600.h"
#include "I2C_Peripheral.h"
#include "servo.h"
#include "stepper.h"
#include "APA102_Custom.h"
#include "I2C_Primary.h"
#include <Wire.h>

/*
ToDo List:
  - Set interrupt priorities
  - Set SPI class to reset proper pinouts
  - Manipulate brightness of the LEDs
  - 


*/


// Global definitions
#define numbLEDsRing 27 // Number of LEDs in the ring
#define LEDsPerReg 8 // Number of LEDs in one register
#define bitsPerLED 2
#define I2C_Sec_Address 0x24 // Address of the I2C module 
// #define openGripperAngle 90 // degrees
// #define closeGripperAngle 0 // degrees
#define BACKDRIVE 0
#define POSITION 1
#define SPEED 2
#define STOP 3
// Pin assignments
#define stepperDirPin 0
#define stepperPWMPin 1
#define stepperEnablePin 2
#define I2C0SCLPin 4
#define I2C0SDAPin 5
#define I2C1SCLPin 19
#define I2C1SDAPin 18
#define SPICLKPin 6
#define SPIMOSIPin 7
#define testLEDPin 14
#define encPWMReadPin 15
#define stepperUARTPin 16
#define stepperS1Pin 22
#define stepperS2Pin 21
#define groundADCchannel 1
#define yforceADCchannel 0
#define xforceADCchannel 2
#define gripperPin 13
// I2C Register map
uint16_t turretPosition = 0; // 0x00: Desired position
uint16_t turretEncoder = 0; // 0x01: Actual position
uint16_t turretSpeed = 0; // 0x02: Current speed the turret is moving (was a signed int, does it need to be typecasted?)
uint16_t turretMaxSpeed = 0; // 0x03: Max speed the turret can move
uint16_t turretMaxTolerance = 0; // 0x04: Tolerance of PID function
uint16_t turretKp = 0; // 0x05: Q15 representation of the proportional constant
uint16_t turretKi = 0; // 0x06: Q15 representation of the integral constant
uint16_t turretKd = 0; // 0x07: Q15 representation of the derivative constant
uint16_t turretState = 0; //0x08: State representation of the turret. Only bottom two bits significant
                          // 0: Backdriveable
                          // 1: Goto position
                          // 2: Set speed
                          // 3: Hold position

uint16_t isGripper = 0; //0x09: Boolean for gripper being open or closed. 1 is Open
uint16_t gripperPresets = 0x5A00; // 0x0A: set open and close position for gripper in degrees. Upper byte is open

uint16_t forceSensorParallel = 0; //0xB: Force sensor reading of the parallel sensor
uint16_t forceSensorPerpendicular = 0; // 0x0C: Force sensor reading of the perpendicular sensor

uint16_t LEDbrightness = 0; // 0x0D: set LED brightness
uint16_t LEDcolorPresets[8] =  {0x0, // 0x0E: set preset color 0 red and green (8 bits) green is upper byte
                                0x0, // 0x0F: set preset color 0 blue (8 bits) lower byte
                                0x00FF, // 0x10: set preset color 1 red and green (8 bits) green is upper byte
                                0x0, // 0x11: set preset color 1 blue (8 bits) lower byte
                                0xFF00, // 0x12: set preset color 2 red and green (8 bits) green is upper byte
                                0x0, // 0x13: set preset color 2 blue (8 bits) lower byte
                                0x0000, // 0x14: set preset color 3 red and green (8 bits) green is upper byte
                                0xFF}; // 0x15: set preset color 3 blue (8 bits) lower byte
uint16_t LEDcolors[4] = {0,  //0x16: set LEDs to preset colors. 2 bits per LED. 0 is LSB 7 is MSB
                         0,  //0x17: set LEDs to preset colors. 2 bits per LED. 8 is LSB 15 is MSB
                         0,  //0x18: set LEDs to preset colors. 2 bits per LED. 16 is LSB 23 is MSB
                         0}; //0x19: set LEDs to preset colors. 2 bits per LED. 24 is LSB. 26 is [5:4]
uint16_t LEDflush = 0; // 0x1A: Boolean for LED being set. 1 is set them. Auto rewrites to 0

uint16_t writeEEPROM = 0x0; // 0x1B: Boolean for EEPROM being written. 1 is write it. Auto rewrites to 0
// Class initialization
APA102 stringLEDs(numbLEDsRing);
ADC adc(0b111, 0); // Activate all channels and DMA channel 0
AS5600 absoluteEncoder(encPWMReadPin);
DIO extLED(testLEDPin);
I2C_P i2c_p;
Servo gripper(gripperPin);
Stepper motor(stepperPWMPin, stepperDirPin, stepperEnablePin, stepperS1Pin, stepperS2Pin, stepperUARTPin); 
I2C_M i2cMain(I2C1SDAPin, I2C1SCLPin, i2c1);
// ToDo: Stepper programmer. Sets SS pins and UART for controlling TMC


// //Temp var
// DIO MS1(22);
// DIO MS2(21);

// Global Variables
uint16_t forceSensor = 20;
void setup() {
  // Initialize variables memory address to I2C registers
  i2c_p.arrMap[0] = &turretPosition;
  i2c_p.arrMap[1] = &turretEncoder;
  i2c_p.arrMap[2] = &turretSpeed;
  i2c_p.arrMap[3] = &turretMaxSpeed;
  i2c_p.arrMap[4] = &turretMaxTolerance;
  i2c_p.arrMap[5] = &turretKp;
  i2c_p.arrMap[6] = &turretKi;
  i2c_p.arrMap[7] = &turretKd;
  i2c_p.arrMap[8] = &turretState;
  i2c_p.arrMap[9] = &isGripper;
  i2c_p.arrMap[10] = &gripperPresets;
  i2c_p.arrMap[11] = &forceSensorParallel;
  i2c_p.arrMap[12] = &forceSensorPerpendicular;
  i2c_p.arrMap[13] = &LEDbrightness;
  i2c_p.arrMap[14] = &LEDcolorPresets[0];
  i2c_p.arrMap[15] = &LEDcolorPresets[1]; 
  i2c_p.arrMap[16] = &LEDcolorPresets[2]; 
  i2c_p.arrMap[17] = &LEDcolorPresets[3]; 
  i2c_p.arrMap[18] = &LEDcolorPresets[4]; 
  i2c_p.arrMap[19] = &LEDcolorPresets[5]; 
  i2c_p.arrMap[20] = &LEDcolorPresets[6]; 
  i2c_p.arrMap[21] = &LEDcolorPresets[7]; 
  i2c_p.arrMap[22] = &LEDcolors[0];
  i2c_p.arrMap[23] = &LEDcolors[1];
  i2c_p.arrMap[24] = &LEDcolors[2];
  i2c_p.arrMap[25] = &LEDcolors[3];
  i2c_p.arrMap[26] = &LEDflush;
  i2c_p.arrMap[27] = &writeEEPROM;



  // Initialize classes 
  // stringLEDs.Init(); // LEDs first. 4 SPI pins are assigned, but 2 are needed. Others will be reassigned. 
  // gripper.attach();
  motor.init();
  motor.enable(false);
  // extLED.init();
  // absoluteEncoder.init();
  // adc.initMulti();
  // i2c_p.init(I2C_Sec_Address);
  // i2cMain.sdkInit();
  delay(1000);
 
}

void loop() {

  // uint32_t setOTPmessage = 0x6 | (0x0 << 4) | (0xbd << 8);
  // motor.writeAddress(regOTPaddress, setOTPmessage); // Set OTP to use internal memorys

  // delay(4000);
// Read status 0x6F
motor.readAddress(0x6F);
delay(4000);
// Read total status 0x00
motor.readAddress(0x00);
delay(4000);
// Read inputs 0x06
motor.readAddress(0x06);
delay(4000);

// motor.enable(true);
// motor.setFreq(1000);
  
}