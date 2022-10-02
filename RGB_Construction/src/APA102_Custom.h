#ifndef APA102_Cust_h
#define APA102_Cust_hS

#include <SPI.h>
// arduino::MbedSPI SPI1(NC, 3, 2);

class APA102_Cust
{
private:
  
  struct pixel {
    uint8_t r = 0;
    uint8_t b = 0;
    uint8_t g = 0;
  };
  pixel *pixels;

public:
  APA102_Cust(uint16_t count) {
    length = count;
    pixels = new pixel[count];
  } 
  ~APA102_Cust() {
    delete[] pixels;
  }
  unsigned int length = 0;
  
  void Init(void)
  {
    SPI.begin();
    for(uint16_t i = 0; i < length; i++) pixels[i] = MakeColor(0, 0, 0);
    Show(); //should clear all
  }
  uint16_t CountPixels(void) {return length;}

  pixel MakeColor(uint8_t red, uint8_t green, uint8_t blue) {
    pixel out;
    out.r = red;
    out.g = green;
    out.b = blue;
    return out;
  }
  void Show(void)
  {
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    for(uint16_t i = 0; i < length; i++)
    {
        SPI.transfer(0xFF);
        SPI.transfer(pixels[i].b);
        SPI.transfer(pixels[i].g);
        SPI.transfer(pixels[i].r);
    
    
    }
    
    SPI.endTransaction();
  }
  void showUniform(uint8_t red, uint8_t green, uint8_t blue)
  {
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    for(uint16_t i = 0; i < length; i++)
    {

      SPI.transfer(0xFF);
      SPI.transfer(blue);
      SPI.transfer(green);
      SPI.transfer(red);

    }
    // for(int i = 1; i <= length / 15; i++)
    // {
    //     SPI.transfer(0);
    // }
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.transfer(0xFF);
    SPI.endTransaction();
  }


//
//  uint32_t SetPixel(uint16_t i, uint32_t color)
//  {
//    return pixels[i] = color;
//  }

//   uint32_t& operator [] (int i)
//   {
//     return pixels[i];
//   }
// };
};
  




#endif
