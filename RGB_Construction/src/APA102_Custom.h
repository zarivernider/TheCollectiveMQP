#pragma once

#include <SPI.h>
// arduino::MbedSPI SPI1(NC, 3, 2);

class APA102
{
private:
  
  struct pixel {
    uint8_t r = 0;
    uint8_t b = 0;
    uint8_t g = 0;
  };
  pixel *pixels;

public:
  APA102(uint16_t count) {
    length = count;
    pixels = new pixel[count];
  } 
  ~APA102() {
    delete[] pixels;
  }
  int length = 0;
  void Init(void);
  int CountPixels(void) {return length;}
  pixel MakeColor(uint8_t red, uint8_t green, uint8_t blue);
  void Show(void);
  void showUniform(uint8_t red, uint8_t green, uint8_t blue);
  pixel& operator [] (int i)
  {
    return pixels[i];
  }
};
  



