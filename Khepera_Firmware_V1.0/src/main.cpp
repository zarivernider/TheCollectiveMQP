#include <Arduino.h>
#include "ADC.h"
#include "AS5600.h"
#include "I2C_Peripheral.h"
#include "servo.h"
#include "stepper.h"
#include "APA102_Custom.h"
#include "I2C_Primary.h"
#include <Wire.h>
#include "EEPROM.h"
#include "DSP.h"
/*
Backburner List:
  - Set interrupt priorities
  - Set SPI class to reset proper pinouts
*/
// 

// Global definitions
#define numbLEDsRing 27 // Number of LEDs in the ring
#define LEDsPerReg 8 // Number of LEDs in one register
#define bitsPerLED 2
#define I2C_Sec_Address 0x35 // Address of the I2C module 
#define BACKDRIVE 0
#define POSITION 1
#define SPEED 2
#define STOP 3

#define PUSH 4
#define PUSH_THRESHOLD 15
#define maxEncoder 44500
#define minEncoder 1350
#define deltaEncoder (maxEncoder - minEncoder)
#define halfEncoder (deltaEncoder >> 1)
#define PID_BAUD 16
#define PID_KI_TOLERANCE 5586 // Disable KI when more than 1/8 of circle away 
#define Q_VALUE 12 // Integers are set up as Q10 values
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
#define parallelforceADCchannel 2
#define perpendicularforceADCchannel 0
#define gripperPin 13
#define writeProtectIO 20
// I2C Register map
uint16_t turretDesPosition = halfEncoder >> 2; // 0x00: Desired position
uint16_t turretEncoder = 0; // 0x01: Actual position. Raw output from the AS5600
uint16_t turretSpeed = 1000; // 0x02: Current speed the turret is moving 
uint16_t turretMaxSpeed = 0xFFFF; // 0x03: Max speed the turret can move
uint16_t turretMaxTolerance = 141; // 0x04: Tolerance of PID function
uint16_t turretKp = 5325; // 0x05: Q12 representation of the proportional constant
uint16_t turretKi = 0; // 0x06: Q12 representation of the integral constant
uint16_t turretKd = 0; // 0x07: Q12 representation of the derivative constant
uint16_t turretState = 3; //0x08: State representation of the turret. Only bottom two bits significant
                          // 0: Backdriveable
                          // 1: Goto position
                          // 2: Set speed
                          // 3: Hold position
                          // 4: Push

uint16_t isGripper = 0; //0x09: Boolean for gripper being open or closed. 1 is Open
uint16_t gripperPresets = 0x5A00; // 0x0A: set open and close position for gripper in degrees. Upper byte is open

uint16_t forceSensorParallel = 0; //0xB: Force sensor reading of the parallel sensor
uint16_t forceSensorPerpendicular = 0; // 0x0C: Force sensor reading of the perpendicular sensor

uint16_t LEDbrightness = 0x1F; // 0x0D: set LED brightness
uint16_t LEDcolorPresets[8] =  {0x0, // 0x0E: set preset color 0 red and green (8 bits) green is upper byte
                                0x0, // 0x0F: set preset color 0 blue (8 bits) lower byte
                                0x00FF, // 0x10: set preset color 1 red and green (8 bits) green is upper byte // CHANGED FROM 0x00FF
                                0x0, // 0x11: set preset color 1 blue (8 bits) lower byte
                                0xFF00, // 0x12: set preset color 2 red and green (8 bits) green is upper byte
                                0x0, // 0x13: set preset color 2 blue (8 bits) lower byte
                                0x0000, // 0x14: set preset color 3 red and green (8 bits) green is upper byte
                                0xFF}; // 0x15: set preset color 3 blue (8 bits) lower byte
uint16_t LEDcolors[4] = {0,  //0x16: set LEDs to preset colors. 2 bits per LED. 0 is LSB 7 is MSB
                         0,  //0x17: set LEDs to preset colors. 2 bits per LED. 8 is LSB 15 is MSB
                         0,  //0x18: set LEDs to preset colors. 2 bits per LED. 16 is LSB 23 is MSB
                         0}; //0x19: set LEDs to preset colors. 2 bits per LED. 24 is LSB. 26 is [5:4]
uint16_t LEDflush = 0;       // 0x1A: Boolean for LED being set. 1 is set them. Auto rewrites to 0
uint16_t writeEEPROM = 0x0;  // 0x1B: Boolean for EEPROM being written. 1 is write it. Auto rewrites to 0

uint16_t turretEncoderTrim = 28912; // 0x1C Current offset for the encoder
uint16_t writeturretTrim = 0x0;     // 0x1D Boolean for encoder orientation to be set. 1 captures current orientation. Auto rewrites to 0
uint16_t turretPosition = 0x0;      // 0x1E encoder position translated from 0 - maxEnc - minEnc. Convert by multiplying with 360 / 43150
uint16_t calibrateADC = 0x0;        // 0x1F Boolean to calibrate the ADCs. 1 calibrates ADCs. Auto rewrites to 0

uint16_t steadyStateADC;
uint16_t pushKp = 27000;             // 0x20
uint16_t parallelFiltered;          // 0x21
uint16_t perpendicularFiltered;     // 0x22
// Debug Registers only - Not included in documentation. Set variables based on default values and not the EEPROM. Require a restart. 
uint16_t runTestSequence = 0x0;     // 0x25 Boolean to test the system. 1 Runs test sequence. Auto rewrites to 0. 







// Class initialization
APA102 stringLEDs(numbLEDsRing);
ADC adc(0b111, 0); // Activate all channels and DMA channel 0
AS5600 absoluteEncoder(encPWMReadPin);
DIO extLED(testLEDPin);
I2C_P i2c_p;
Servo gripper(gripperPin);
Stepper motor(stepperPWMPin, stepperDirPin, stepperEnablePin, stepperS1Pin, stepperS2Pin, stepperUARTPin); 
I2C_M i2cMain(I2C1SDAPin, I2C1SCLPin, i2c1);
EEPROM eeprom(&i2cMain, writeProtectIO);

//Test functions
void testLED(); // Test the LEDs by setting the colors to alternate. Call in setup
void testADC(); // Test both ADC's and print the return values. Call in loop.
void testPrint(); // Random function. Modify whenever to print whatever is desired
void globalStop(); // Stop everything
void testSystems(); // Test all subsystems but ADC
void printVal(uint16_t testVal); // Test sending a specific value
void testForceSensors();

// Global Variables
int32_t sumError = 0;


void setup() {
  // Initialize variables memory address to I2C registers
  i2c_p.arrMap[0] = &turretDesPosition;
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
  i2c_p.arrMap[28] = &turretEncoderTrim;
  i2c_p.arrMap[29] = &writeturretTrim;
  i2c_p.arrMap[30] = &turretPosition;
  i2c_p.arrMap[31] = &calibrateADC;
  i2c_p.arrMap[32] = &pushKp;
  i2c_p.arrMap[33] = &parallelFiltered;          
  i2c_p.arrMap[34] = &perpendicularFiltered;    
  i2c_p.arrMap[35] = &runTestSequence;
 

  // Initialize classes 
  stringLEDs.Init(); // LEDs first. 4 SPI pins are assigned, but 2 are needed. Others will be reassigned. 
  gripper.attach();
  motor.init();
  extLED.init();
  absoluteEncoder.init();
  adc.initMulti();
  i2c_p.init(I2C_Sec_Address);
  eeprom.init();
  extLED.assertIO(true);

  eeprom.readArray(i2c_p.arrMap, numbReg);
  adc.calibrateMulti(400);
  steadyStateADC = adc.getADCMulti(perpendicularforceADCchannel); // Set steady state 
  delay(1000);
  extLED.assertIO(false);



}

void loop() {

  // Execute gripper
  uint8_t gripperPos = isGripper & 0x1 ? (gripperPresets >> 8) : gripperPresets & 0xFF;
  gripper.setServo(gripperPos);

  // Update LEDs
  if(LEDflush & 0x1) {
    for(uint8_t regNumb = 0; regNumb < ceil((float)numbLEDsRing / LEDsPerReg); regNumb++) { // Get the specific register (ceil((float)numbLEDsRing / LEDsPerReg)
      for (uint8_t LEDNumb = 0; LEDNumb < LEDsPerReg; LEDNumb++) { // LED number in the register
        uint8_t LEDnumber = regNumb*LEDsPerReg + LEDNumb; // LED number globally
        if(LEDnumber >= numbLEDsRing) break; // Leave loop if last LED has been set
        uint8_t getPreset = (LEDcolors[regNumb] >> (LEDNumb*2) ) & 0x03; // Get the color of specific LED
        uint8_t presetIndex = getPreset * 2; // Two registers per each preset
        uint8_t green = LEDcolorPresets[presetIndex] >> 8;
        uint8_t red = LEDcolorPresets[presetIndex];
        uint8_t blue = LEDcolorPresets[presetIndex + 1];
        stringLEDs[LEDnumber] = stringLEDs.MakeColor(red, green, blue);
      }
    }
    stringLEDs.Show(LEDbrightness & 0x1F);
    LEDflush = 0x0;
  }
  // Calibrate ADCs
  if(calibrateADC & 0x1) {
    // Make diagnostic LED bright
    extLED.assertIO(true);
    adc.calibrateMulti(400);
    steadyStateADC = adc.getADCMulti(perpendicularforceADCchannel);
    // Make diagnostic LED low
    extLED.assertIO(false);
    // Reset flag
    calibrateADC = 0x0;
  }
  // Set encoder position
  turretEncoder = absoluteEncoder.getCtr();
  uint16_t constrainEncoder = constrain(turretEncoder, minEncoder, maxEncoder); // Set the encoder between min and max. Constrain to prevent ovf
  uint16_t tempTrim = turretEncoderTrim - minEncoder; // Get the distance between the trim value and the minimum recorded encoder value
  uint32_t transPosition = constrainEncoder > turretEncoderTrim ? // Check if the position is not currently between the determined zero and minimum
                           constrainEncoder : // Output is exactly as expected
                           constrainEncoder - minEncoder + maxEncoder; // Shift the value to be after the max to set the new heading
  turretPosition = transPosition - turretEncoderTrim; // Normalize everything so that the scale starts with the encoder at 0
  turretPosition = map(transPosition, turretEncoderTrim, maxEncoder + tempTrim, 0, maxEncoder - minEncoder); // Linearly map so 0 represents 0 and max is resolution

  // Set encoder head
  if(writeturretTrim & 0x1) {
    // Store current location as 0 degrees
    turretEncoderTrim = constrainEncoder;
    // Reset flag to 0
    writeturretTrim = 0x0;
  }

  if(writeEEPROM & 0x1) {
    // Make diagnostic LED bright
    extLED.assertIO(true);
    i2c_p.isInterrupt(false); // Disable I2C communication interrupts while writing EEPROM
    writeEEPROM = 0x0; // Clear first so it is not written as 1 in EEPROM
    globalStop(); // Disable settings so that the turret does not move on startup
    // Write the EEPROM
    eeprom.writeArray(i2c_p.arrMap, numbReg);
    i2c_p.isInterrupt(true);
    // Make diagnostic LED low
    extLED.assertIO(false);
    
  }
  switch (turretState)
  {
  case BACKDRIVE:
    // Disable the stepper driver
    motor.enable(false);
    motor.brakeStop();
    extLED.assertIO(false);
    break;
  case POSITION: {
    
    // ToDo: Write control loop
    static uint32_t oldTime = 0;
    uint32_t currentTime = millis();
    if(currentTime - oldTime > PID_BAUD) {
      int16_t translatedPosition; // New position based on target
      static int prevError; // Previous error for D calculation
      // Point to the oppposite side of the circle
      int16_t half_position = turretDesPosition >= halfEncoder  ? turretDesPosition - halfEncoder : turretDesPosition + halfEncoder;
      // Check if the opposite side of the circle is on the left side of the circle (greater than 180 degrees). True if yes
      bool leftHalfCircle = half_position >= halfEncoder ? true : false;
      // Check if the current position is on the left side (greater than 180 degrees) true if yes
      bool currentPositionLeft = turretPosition >= half_position ? true : false;

      // If both attributes are in the left side of the circle then subtract a full circle
      if(leftHalfCircle && currentPositionLeft) translatedPosition = turretPosition - deltaEncoder;
      // If both attributes are in the right side of the circle then add a full circle
      else if(!leftHalfCircle && !currentPositionLeft) translatedPosition = turretPosition + deltaEncoder;
      // If neither conditions are true then keep the position the same
      else translatedPosition = turretPosition;

      // Define an error
      int16_t error = (translatedPosition - turretDesPosition) >> 1; // Lose a bit of precision for fixed point arithmetic
      static bool PIDisenable = true;
      if(abs(error) > turretMaxTolerance >> 1) PIDisenable = true;
      else if (abs(error) < PID_KI_TOLERANCE >> 2) PIDisenable = false;

      if(PIDisenable) {
        motor.enable(true);
      // Find the delta error. delta T is combined in the derivative constant
      int16_t deltaError = error - prevError;
      
      // Zero the Ki band when the error is too large
      if(abs(error) > PID_KI_TOLERANCE) {
          sumError = 0;
      }
      else sumError += error; // sum the errors
      // Fixed point calculation to calculate the proper output
      int16_t motorOut = ((int32_t)turretKp*error  >> Q_VALUE)+ 
                    ((int32_t)turretKd*deltaError >> Q_VALUE) + 
                    ((int32_t)turretKi*sumError >> Q_VALUE); 
      // Limit max speed
      motorOut = abs(motorOut) > turretMaxSpeed ? turretMaxSpeed : motorOut;
      turretSpeed = motorOut; // Set speed equal to the current motor setting
      motor.setDirFreq(motorOut);
      oldTime = currentTime;
      prevError = error;
      }
      else {
        motor.enable(false);
      }
    }

    break;
    }
  case SPEED: {
    motor.enable(true);
    int negTurretSpeed = (int16_t)turretSpeed;
    if(negTurretSpeed > 0) motor.setDirection(true);
    else motor.setDirection(false);
    negTurretSpeed = abs(negTurretSpeed) > turretMaxSpeed ? turretMaxSpeed : abs(negTurretSpeed);
    motor.setFreq(negTurretSpeed);
    break;
    }
  case STOP:
    motor.enable(true);
    motor.brakeStop();
    break;
  case PUSH: {
    motor.enable(true);
    int16_t error = perpendicularFiltered - steadyStateADC;
    error = abs(error) < PUSH_THRESHOLD ? 0 : error;
    int16_t motorOut = (int32_t)pushKp*error  >> Q_VALUE;
    motor.setDirFreq(motorOut*8);
    break;
    }
  }

  // Get the current ADC values
  forceSensorParallel = adc.getADCMulti(parallelforceADCchannel); 
  forceSensorPerpendicular =  adc.getADCMulti(perpendicularforceADCchannel); 

  // Update force sensor through the filter periodically. Data is updated every 3 mS
  static uint32_t oldTime_DSP = 0;
  if(millis() - oldTime_DSP > 4) {
    addFilter(forceSensorParallel, &parallelFilter);
    addFilter(forceSensorPerpendicular, &perpendicularFilter);
    parallelFiltered = getAvgFilter(parallelFilter);
    perpendicularFiltered = getAvgFilter(perpendicularFilter);
    oldTime_DSP = millis();
  }

  // testADC();
  // testPrint();
  if(runTestSequence & 0x1) testSystems();
  else if(runTestSequence & 0x2) testForceSensors();

  }

void testLED() { 
  LEDcolors[0] = 0x6DB6;
  LEDcolors[1] = 0xDB6D;
  LEDcolors[2] = 0xB6DB;
  LEDcolors[3] = 0x6DB6;
  LEDflush = 1;
}

void testADC() {
  static uint32_t oldTime = 0;
  if(millis() - oldTime > 1500) {
    Serial.print("Zero: ");
    Serial.print(adc.getrawADCMulti(1));
    Serial.print("\t Parallel: ");
    Serial.print(forceSensorParallel);
    Serial.print("\t Perpendicular: ");
    Serial.println(forceSensorPerpendicular);
    oldTime = millis();
  }

}

void testPrint() {
  static uint32_t oldTime = 0;
  if(millis() - oldTime > 1000) {
    Serial.println(writeEEPROM);
    oldTime = millis();
  }
}

void printVal(uint16_t testVal) {
  static uint32_t oldTime = 0;
  if(millis() - oldTime > 1000) {
    Serial.println(testVal);
    oldTime = millis();
  }
}

void globalStop() {
  turretState = 0; // Turn off turret
  isGripper = 1; // Open Gripper1
  writeEEPROM = 0;
  writeturretTrim = 0;


}

void testSystems() {
  static uint16_t testCounter = 0;
  static uint32_t oldTime = 0;
  extLED.assertIO(true);
  if(millis() - oldTime > 2000) {
    testCounter++;
    oldTime = millis();
  }
  if(testCounter == 1) isGripper = 0; // Open gripper
  else if(testCounter == 3) isGripper = 1; // Close gripper
  else if(testCounter == 5) { // Start rotation
    turretState = SPEED;
    turretMaxSpeed = 0xFFFF;
    turretSpeed = 1000;
  }
  else if(testCounter == 8) { // Turn on LEDs
    LEDcolorPresets[0] = 0x0;
    LEDcolorPresets[1] = 0x0;
    LEDcolorPresets[2] = 0x00FF;
    LEDcolorPresets[3] = 0x0;
    LEDcolorPresets[4] = 0xFF00;
    LEDcolorPresets[5] = 0x0;
    LEDcolorPresets[6] = 0x0000;
    LEDcolorPresets[7] = 0xFF;
    testLED();
  }
  else if(testCounter == 9) { // Change rotation 
    turretSpeed = (int16_t)-1000;
  }
  else if(testCounter == 13) {
    turretState = BACKDRIVE;
  }
  else if(testCounter == 14) {
    LEDcolors[0] = 0x0;
    LEDcolors[1] = 0x0;
    LEDcolors[2] = 0x0;
    LEDcolors[3] = 0x0;
    LEDflush = 1;
  }
  else if(testCounter == 15) {
    turretMaxTolerance = 141;
    turretState = POSITION;
    turretDesPosition = 0;
    turretKp = 5325;
    turretKi = 0; 
    turretKd = 0;
  }
  else if(testCounter == 20) {
    turretDesPosition = halfEncoder;
  }
  else if(testCounter == 26) {
    turretState = BACKDRIVE;
    runTestSequence = 0x0;
    testCounter = 0;
    extLED.assertIO(false);
  }
  
}

#define parallelFS 1000
#define perpendicularFS 1000
#define parallelThresh (parallelFS / (int)(numbLEDsRing >> 1))
#define perpendicularThresh (perpendicularFS / (int)(numbLEDsRing >> 1))
#define halfRing ((int)(numbLEDsRing >> 1))
void testForceSensors() {
  turretState = 3;
  static uint32_t oldTime = 0;
  if(millis() - oldTime > 25) {
    int16_t parallelError = parallelFiltered - steadyStateADC;
    int16_t perpendicularError = perpendicularFiltered - steadyStateADC;
    bool parallelNegative = parallelError > 0 ? false : true;
    bool perpendicularNegative = perpendicularError > 0 ? false : true;
    int parallelnumbLEDs = parallelError / parallelThresh;
    int perpendicularnumbLEDs = perpendicularError / perpendicularThresh; 
    int16_t maxPerpendicular = perpendicularNegative ? halfRing : halfRing + perpendicularnumbLEDs;
    int16_t minPerpendicular = perpendicularNegative ? halfRing + perpendicularnumbLEDs : halfRing;
    int16_t maxParallel = parallelNegative ? halfRing : halfRing + parallelnumbLEDs;
    int16_t minParallel  = parallelNegative ? halfRing + parallelnumbLEDs : halfRing;

    for(int i = 1; i < numbLEDsRing + 1; i++) {
      
      uint8_t red = (i > minPerpendicular && i < maxPerpendicular) ? 255 : 0;
      uint8_t blue = (i > minParallel&& i < maxParallel) ? 255 : 0;
      stringLEDs[i - 1] = stringLEDs.MakeColor(red, 0, blue);
    }
    stringLEDs[12] = stringLEDs.MakeColor(0, 255, 0);
    stringLEDs.Show(LEDbrightness);

    oldTime = millis();
  }
}

