#ifndef I2C_Peripheral_h
#define I2C_Peripheral_h
#include <Arduino.h>
#include <Wire.h>
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

    Code is optimized to only write 2 byte data types. WILL NOT WORK WITH DATA TYPES LARGER THAN 16 bits
*/
class I2C_M {
  private: 
    // Storage variables
    // uint8_t I2C_Addr = 0; // store secondary I2C address
    
  
  public:    
  // Functions
    void init();
    void writeInt(int data, byte reg, byte I2C_Addr);
    int readInt(byte reg, byte I2C_Addr);

};

#endif