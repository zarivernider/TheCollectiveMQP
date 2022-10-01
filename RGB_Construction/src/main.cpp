#include <Arduino.h>
#include <SPI.h>
 void Show(void)
  {
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    // for(uint16_t i = 0; i < pixels.Length(); i++)
    // {
    //   for(int j = 24; j >= 0; j-=8) SPI.transfer(pixels[i] >> j);
    // }
    
    SPI.endTransaction();
  }
 void Init(void)
  {
    SPI.begin();
    // for(uint16_t i = 0; i < pixels.Length(); i++) pixels[i] = MakeColor(0, 0, 0);
    
    Show(); //should clear all
  }

  // uint16_t CountPixels(void) {return pixels.Length();}

struct pixel {
    unsigned char r = 0;
    unsigned char b = 0;
    unsigned char g = 0;
  }; 
pixel *pixels;
void setup() {
  // put your setup code here, to run once:

  
  int count = 10;
  pixels = (pixel*)realloc(pixels, count * sizeof(pixel));
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(sizeof(pixels));
  delay(1000);
}