#ifndef DIO_h
#define DIO_h
#include <Arduino.h>
#include "system_pi_pico.h"
/*
 
*/
class DIO
{
private:
  uint8_t ioPin;
  uint16_t ioOffset;

public:
DIO(uint8_t IOPin) { 
    ioPin = IOPin;
  }
DIO (){}
  

  void init();
  void assertIO(bool isHigh);


};

#endif