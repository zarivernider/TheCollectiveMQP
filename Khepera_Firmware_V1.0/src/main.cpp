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
#define I2C_Sec_Address 0x24 // Address of the I2C module 
#define openGripperAngle 90 // degrees
#define closeGripperAngle 0 // degrees
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
  // adc.initMulti();
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
  // // delay(1000);true
  // gripper.setServo(180);
  // stringLEDs.showUniform(0,255, 0);
  // delay(2000);
  // Serial.println("Sending data");
  // Serial.println(*(io_rw_32*)(PADS_BANK0_BASE + 0x14), HEX);
  // i2cMain.sdkwriteData(I2C_Sec_Address, 0x1, 0x1234);
  // uint16_t returnValue;
  // uint16_t read = i2cMain.sdkreadData(I2C_Sec_Address, 0x1, &returnValue);
  // stringLEDs.showUniform(255, 0, 0);
  // gripper.setServo(90);
  // Serial.print(adc.getrawADCMulti(0));
  // Serial.print("\t");
  // Serial.print(adc.getrawADCMulti(1));
  // Serial.print("\t");
  // Serial.print(adc.getrawADCMulti(2));
  // Serial.print("\t");
  // Serial.print(read, HEX);
  // Serial.print("\t");
  // Serial.print(returnValue, HEX);
  // Serial.print("\t");
  // Serial.println(forceSensor, HEX);
  
  // delay(2000);
                        // bit 6   byte 0     enable write

  // motor.readAddress(0x13);
  // int byteNumb = 0;
  // if (Serial1.available()) {
    
  //   uint8_t read = Serial1.read();
  //   Serial.print("Byte number ");
  //   Serial.print(byteNumb);
  //   Serial.print(": ");
  //   Serial.println(read);
  //   byteNumb++;

  // }
  motor.setFreq(1000);
  // // gripper.setServo(0);
  // Serial.println("Show Red");
  // stringLEDs.showUniform(255, 0, 0);
  delay(4000);
  // motor.readAddress(0x6);
  // stringLEDs.showUniform(0, 255, 0);
  // delay(4000);
  motor.readAddress(0x6f);
  
  delay(4000);
  // stringLEDs.showUniform(0, 0, 255);
  motor.readAddress(0x00);
  delay(4000);
  // motor.readAddress(0x05);
  // // // // // gripper.setServo(180);
  // // // delay(4000);
  // // // motor.readAddress(0x6);
  // delay(4000);

}