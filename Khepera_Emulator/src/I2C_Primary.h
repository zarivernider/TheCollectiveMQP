#ifndef I2C_Primary_h
#define I2C_Primary_h
#include <Arduino.h>
#include "system_pi_pico.h"
/*
  Class to establish secondary I2C control of a raspberry pi pico. 
  Steps to use:
    Create instance of class
    Set variable memory address equal to one of 30 memory address spots in arrMap(<obj_name>.arrMap[<desired address>] = &<desired variable>)
    The desired variables will now change when sent with proper messages
  Proper messages:
    Read:
      Send a byte of address. Address is index of arrMap
      Request only 2 bytes of data
    Write:
      Send byte 1: address for the variable. same as the index of arrMap
      Send byte 2: desired least significant byte
      Send byte 3: desired most signigicant byte
*/
class I2C_M {
  private: 
    i2c_inst_t *i2cBase;
    uint16_t SCLioOffset = 0;
    uint16_t SDAioOffset = 0;
    uint16_t SDApadOffset = 0;
    uint16_t SCLpadOffset = 0;
    uint32_t I2C_BASE_OFFSET = 0;
    uint32_t maxPollvalue = 12500; // Clock counts for 10x i2c signal. (0.1 mS)
    #define baudRate 1000000
  
  public:    
    // Constructor for reg
    I2C_M(uint8_t SDApin, uint8_t SCLpin, uint32_t I2C_Base) {
        SCLioOffset = SCLpin * 0x08;
        SDAioOffset = SDApin * 0x08;
        I2C_BASE_OFFSET = I2C_Base;
    }
    // constructor for SDK
    I2C_M(uint8_t SDApin, uint8_t SCLpin, i2c_inst_t *i2c) {
        SCLioOffset = SCLpin * 0x08;
        SDAioOffset = SDApin * 0x08;
        SDApadOffset = (SDApin * 0x04) + 0x04;
        SCLpadOffset = (SCLpin * 0x04) + 0x04;

        i2cBase = i2c;
    }
    
    bool mailBox = false;
    // Functions
    void init();
    void beginTransmission(uint8_t address);
    void write(uint8_t data);
    void requestFrom(uint8_t address, uint8_t numbBytes);
    uint8_t read();
    void endTransmission();
    bool isAvailable();
    void blockUntilData();
    
    void sdkInit();
    void sdkwriteData(uint8_t address, uint8_t reg, uint16_t data);
    uint16_t sdkreadData(uint8_t address, uint8_t reg, uint16_t *memData);


};

#endif