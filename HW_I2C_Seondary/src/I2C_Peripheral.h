#ifndef I2C_Peripheral_h
#define I2C_Peripheral_h
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
class I2C_P {
  private: 
    // Storage variables
    uint8_t I2C_Addr = 0; // store secondary I2C address
    
  
  public:    
  // Variables for array
    int* arrMap[30]; // Hold memory addresses for the desired variables to change
    byte globAddr = 0; // Keep track of current global address to connect send and recieve functions
    bool mailBox = false;
    uint32_t status = true;
  // Functions
    void init(uint8_t address);

};

extern I2C_P i2c_p;
#endif