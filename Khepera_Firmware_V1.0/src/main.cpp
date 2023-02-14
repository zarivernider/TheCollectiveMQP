#include <Arduino.h>
#include "ADC.h"
#include "AS5600.h"
#include "I2C_Peripheral.h"
#include "servo.h"
#include "stepper.h"
#include "APA102_Custom.h"
#include "I2C_Primary.h"
#include <Wire.h>

// Global definitions
#define numbLEDsRing 8 // Number of LEDs in the ring
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
I2C_M i2cMain(I2C1SDAPin, I2C1SCLPin, i2c1);
// ToDo: Stepper programmer. Sets SS pins and UART for controlling TMC


// Global Variables
uint16_t forceSensor = 20;
void setup() {
  // Initialize classes 
  // stringLEDs.Init(); // LEDs first. 4 SPI pins are assigned, but 2 are needed. Others will be reassigned. 
  gripper.attach();
  // gripper.setMinMax(0,180);
  // motor.init();
  // extLED.init();
  // absoluteEncoder.init();
  i2c_p.init(I2C_Sec_Address);
  i2c_p.arrMap[1] = &forceSensor;
  // // adc.initMulti();
  // delay(6000);
  // Serial.println("Pre Wire");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);


  // i2cMain.init();
  // // Wire.begin();
  // delay(2000);
  // Serial.println("Post Wire");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);
  i2cMain.sdkInit();
  
}

void loop() {
  // // put your main code here, to run repeatedly:
  // Serial.println("Begin transaction");


  for(int i = 0; i < 180; i++) {
    gripper.setServo(i);
    delay(50);
  }
      gripper.setServo(0);
      delay(500);

  // gripper.setServo(0);
  // delay(2000);
  // gripper.setServo(180);
  // delay(2000);
  // i2cMain.beginTransmission(I2C_Sec_Address);
  // Serial.println("Begin");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);
  // i2cMain.write(0x12);
  // Serial.println("Write");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);

  // i2cMain.write(0x34);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);

  // i2cMain.endTransmission();
  // Serial.println("End");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);

  // Serial.println(*(io_rw_32*)(I2C1_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C1_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C1_BASE | I2C_IC_TAR_OFFSET), HEX);
  // delay(2000);

  // delay(1000);
  // Wire.beginTransmission(0x15);
  // Serial.println("Begin transmission");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);


  // Wire.write(0x24);
  // Serial.println("Write");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);
  //   Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_DATA_CMD_OFFSET), HEX);

  // Wire.endTransmission();
  //   Serial.println("End Transmission");
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_STATUS_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_TAR_OFFSET), HEX);
  // Serial.println(*(io_rw_32*)(I2C0_BASE + I2C_IC_ENABLE_OFFSET), HEX);

  // delay(1000);
  // delay(1000);
  // Serial.println("Sending data");
  // Serial.println(*(io_rw_32*)(PADS_BANK0_BASE + 0x50), HEX);

  // i2cMain.sdkwriteData(I2C_Sec_Address, 0x1, 0x1234);
  // uint16_t returnValue;
  // uint16_t read = i2cMain.sdkreadData(I2C_Sec_Address, 0x1, &returnValue);
  // Serial.print(read, HEX);
  // Serial.print("\t");
  // Serial.print(returnValue, HEX);
  // Serial.print("\t");
  // Serial.println(forceSensor, HEX);
  
  delay(2000);

}