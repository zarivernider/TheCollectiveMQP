#ifndef __APA102_H
#define __APA102_H

#include <SPI.h>
#include <vector_uC.h>

#define Int32Vector TVector<uint32_t>

class LEDStrip {};

class APA102 : public LEDStrip
{
private:
  Int32Vector pixels;

public:
  APA102(uint16_t count) : pixels(count) {} 
  
  void Init(void)
  {
    SPI.begin();
    for(uint16_t i = 0; i < pixels.Length(); i++) pixels[i] = MakeColor(0, 0, 0);
    
    Show(); //should clear all
  }

  uint16_t CountPixels(void) {return pixels.Length();}

  void Show(void)
  {
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    for(uint16_t i = 0; i < pixels.Length(); i++)
    {
      for(int j = 24; j >= 0; j-=8) SPI.transfer(pixels[i] >> j);
    }
    
    SPI.endTransaction();
  }

  static uint32_t MakeColor(uint32_t red, uint32_t grn, uint32_t blu) //uint32, since I'll need to shift them
  {
    uint32_t color = 0xff000000; //first byte is all 1's (lower 5 bits are intensity, but best to leave them 1's)
    color |= (blu << 16) | (grn << 8) | (red << 0);

    return color;
  }
//
//  uint32_t SetPixel(uint16_t i, uint32_t color)
//  {
//    return pixels[i] = color;
//  }

  uint32_t& operator [] (int i)
  {
    return pixels[i];
  }
};

#endif
