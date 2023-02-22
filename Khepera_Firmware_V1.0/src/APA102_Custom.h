#pragma once
/*
  Class to control APA102 light strips with the RP-2040. This class implements SPI lines and uses the default pins. 
  Colors are manipulated in RGB format
*/
#include <SPI.h>
#include "system_pi_pico.h"
// arduino::MbedSPI SPI1(NC, 3, 2);

class APA102
{
private:
  #define MOSIpin 7
  #define SCLKpin 6
  // Data type to define a pixel in RGB
  struct pixel {
    uint8_t r = 0;
    uint8_t b = 0;
    uint8_t g = 0;
  };
  pixel *pixels; // Amount of pixels is a variable array of pixels datatype / struct

public:
  APA102(uint16_t count) { // Set the size of the light strip and create the housing array
    length = count;
    pixels = new pixel[count];
  } 
  ~APA102() { 
    delete[] pixels; // Delete the array once the class is gone to prevent memory leaks
  }
  int length = 0; // Set initial length to a size of 0
  void Init(void); // Initialize the Pico for transmission
  int CountPixels(void) {return length;} // Return the amount of pixels in the transmission line
  pixel MakeColor(uint8_t red, uint8_t green, uint8_t blue); // Format RGB colors to proper data type
  void Show(void); // Upload all values from internal buffer to the light strips
  void showUniform(uint8_t red, uint8_t green, uint8_t blue); // Set all LEDs the same color
  pixel& operator [] (int i) // Make the class index-able
  {
    return pixels[i];
  }
};
  



