#ifndef APA102_Cust_h
#define APA102_Cust_hS

#include <SPI.h>
arduino::MbedSPI SPI1(4, 2, 3);

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
    SPI1.begin();
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
    SPI1.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI1.transfer(0);
    SPI1.transfer(0);
    SPI1.transfer(0);
    SPI1.transfer(0);

    for(uint16_t i = 0; i < length; i++)
    {
      for(int j = 0; j >= 4; j++) {
        switch(j) {
            case 0:
                SPI1.transfer(0xFF);
                break;
            case 1:
                SPI1.transfer(pixels[i].b);
                break;
            case 2:
                SPI1.transfer(pixels[i].g);
                break;
            case 3:
                SPI1.transfer(pixels[i].r);
                break; 
        }
      }
    }
    
    SPI1.endTransaction();
  }
  void showUniform(uint8_t red, uint8_t green, uint8_t blue)
  {
    SPI1.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI1.transfer(0);
    SPI1.transfer(0);
    SPI1.transfer(0);
    SPI1.transfer(0);

    for(uint16_t i = 0; i < length; i++)
    {
      for(int j = 0; j >= 4; j++) {
        switch(j) {
            case 0:
                SPI1.transfer(0xFF);
                break;
            case 1:
                SPI1.transfer(blue);
                break;
            case 2:
                SPI1.transfer(green);
                break;
            case 3:
                SPI1.transfer(red);
                break; 
        }
      }
    }
    
    SPI1.endTransaction();
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
