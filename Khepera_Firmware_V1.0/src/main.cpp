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
  - 


*/


// Global definitions
#define numbLEDsRing 27 // Number of LEDs in the ring
#define LEDsPerReg 8 // Number of LEDs in one register
#define bitsPerLED 2
#define I2C_Sec_Address 0x24 // Address of the I2C module 
#define openGripperAngle 90 // degrees
#define closeGripperAngle 0 // degrees
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
int16_t turretSpeed = 0; // 0x02: Current speed the turret is moving
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

uint16_t isGripper = 1; //0x09: Boolean for gripper being open or closed. 1 is Open
uint16_t gripperPresets = 0x0091; // 0x0A: set open and close position for gripper in degrees. Upper byte is open

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
  // Initialize classes 
  stringLEDs.Init(); // LEDs first. 4 SPI pins are assigned, but 2 are needed. Others will be reassigned. 
  // gripper.attach();
  delay(100);
  motor.init();

  // extLED.init();
  // absoluteEncoder.init();
  adc.initMulti();
  // i2c_p.init(I2C_Sec_Address);
  // i2c_p.arrMap[1] = &forceSensor;
  // i2cMain.sdkInit();
  delay(5000);
      motor.writeAddress(regConfaddress, 0x103); // Enable internal Rsense resistors
      uint32_t setOTPmessage = 0x6 | (0x0 << 4) | (0xbd << 8);
      motor.writeAddress(regOTPaddress, setOTPmessage); // Set OTP to use internal memorys
  motor.enable(true);
}

void loop() {
  // Execute gripper
  uint8_t gripperPos = isGripper & 0x1 ? (gripperPresets >> 8) : gripperPresets & 0xFF;
  gripper.setServo(gripperPos);

  // Update LEDs
  if(LEDflush & 0x1) {
    for(uint8_t regNumb = 0; regNumb < numbLEDsRing / LEDsPerReg; regNumb++) { // Get the specific register
      for (uint8_t LEDNumb = 0; LEDNumb < LEDsPerReg; LEDNumb++) { // LED number in the register
        uint8_t LEDnumber = regNumb*LEDsPerReg + LEDNumb; // LED number globally
        if(LEDnumber >= numbLEDsRing - 1) break; // Leave loop if last LED has been set
        uint8_t getPreset = (LEDcolors[regNumb] >> LEDnumber) & 0xFF;
        uint8_t presetIndex = getPreset * 2;
        uint8_t green = LEDcolorPresets[presetIndex] >> 8;
        uint8_t red = LEDcolorPresets[presetIndex];
        uint8_t blue = LEDcolorPresets[presetIndex + 1];
        stringLEDs[LEDnumber] = stringLEDs.MakeColor(red, green, blue);
      }
    }
    stringLEDs.Show();
    LEDflush = 0x0;
  }

// Set encoder position
turretEncoder = absoluteEncoder.getCtr();

if(writeEEPROM & 0x1) {
  // Make diagnostic LED bright
  extLED.assertIO(true);
  // ToDo: clear and write the EEPROM
  // Make diagnostic LED low
  extLED.assertIO(false);
  writeEEPROM = 0x0;
}

switch (turretState)
{
case BACKDRIVE:
  motor.enable(false);
  break;
case POSITION:
  // ToDo: Write control loop
  break;
case SPEED:
    motor.enable(true);
    turretSpeed = abs(turretSpeed) > turretMaxSpeed ? turretMaxSpeed : turretSpeed;
    if(turretSpeed > 0) motor.setDirection(true);
    else motor.setDirection(false);
    motor.setSpeed(turretSpeed);
    break;
case STOP:
  motor.enable(true);
  motor.brakeStop();
  break;
}

// ToDo: Add ADC calibration
forceSensorParallel = adc.getADCMulti(0); // Not sure if proper ADC (ToDo)
forceSensorPerpendicular =  adc.getADCMulti(2); // Not sure if proper ADC (ToDo)
}