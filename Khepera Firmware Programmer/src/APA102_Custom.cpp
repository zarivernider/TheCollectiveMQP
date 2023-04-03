#include <APA102_Custom.h>

void APA102::Init(void)
  {
    SPI.begin();
    // Enable proper GPIO pins
    uint8_t MOSIoffset = MOSIpin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    uint8_t SCLKoffset = SCLKpin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS

    writeReg(IO_BANK0_BASE | (0x04 + MOSIoffset), 5, 0, 1); // func select -> 1 = SPI
    writeReg(IO_BANK0_BASE | (0x04 + MOSIoffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    writeReg(IO_BANK0_BASE | (0x04 + SCLKoffset), 5, 0, 1); // func select -> 1 = SPI
    writeReg(IO_BANK0_BASE | (0x04 + SCLKoffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    // Zero all pixels
    for(uint16_t i = 0; i < length; i++) pixels[i] = MakeColor(0, 0, 0); // Clear color of all LEDs
    Show(); //should clear all
  }
  APA102::pixel APA102::MakeColor(uint8_t red, uint8_t green, uint8_t blue) {
    pixel out; // Create a blank data type
    out.r = red; // Set red value
    out.g = green; // Set green value
    out.b = blue; // Set blue value
    return out;
  }

  void APA102::Show(void)
  {
    // Begin transmission with 8 Meg clock
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    // Set pixel colors based on data stored in array
    for(uint16_t i = 0; i < length; i++)
    {
        SPI.transfer(0xFF);
        SPI.transfer(pixels[i].b);
        SPI.transfer(pixels[i].g);
        SPI.transfer(pixels[i].r);
    }
    //End frame of 32 bits, adjusted for length
    for(int i = 0; i < length / 20 +4; i++)
    {
        SPI.transfer(0);
    }
    // End transmission
    SPI.endTransaction();
  }

   void APA102::showUniform(uint8_t red, uint8_t green, uint8_t blue)
  {
    // Begin transmission with 8 Meg clock
    SPI.beginTransaction(SPISettings(8000000ul, MSBFIRST, SPI_MODE0));

    //32 bits of zeros to init xfer
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    // Make all LEDs uniform color
    for(uint16_t i = 0; i < length; i++)
    {

      SPI.transfer(0xFF);
      SPI.transfer(blue);
      SPI.transfer(green);
      SPI.transfer(red);

    }
    //End frame of 32 bits, adjusted for length
    for(int i = 0; i < length / 20 +4; i++)
    {
        SPI.transfer(0);
    }
    // End transmission
    SPI.endTransaction();
  }