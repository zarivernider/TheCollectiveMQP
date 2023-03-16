#include <Wire.h>
#include "I2C_Primary.h"
#include "DIO.h"

#define i2cAddress 0x50 // Address with no modification
#define writeDelay 6 // Delay in milliseconds between each write
class EEPROM
{
private:
    I2C_M i2c_comm;
    DIO writeProtect;
    void isWrite(bool isEnable);
    // arduino::MbedI2C Wire1(18, 19);
    // TwoWire i2cWIRE = TwoWire(1);

public:
    // EEPROM(uint8_t SDApin, uint8_t SCLpin, i2c_inst_t *i2c, uint8_t writeIO) {
    //     // i2c_comm = I2C_M(SDApin, SCLpin, i2c);
    //     writeProtect = DIO(writeIO);
    //     Wire1 = MbedI2C(SDApin, SCLpin);
    // }
        EEPROM(uint8_t SDApin, uint8_t SCLpin, uint8_t writeIO) {
        // i2c_comm = I2C_M(SDApin, SCLpin, i2c);
        writeProtect = DIO(writeIO);
        // Wire1 = arduino::MbedI2C(SDApin, SCLpin);
    }

    EEPROM(I2C_M *i2c_object, uint8_t writeIO) {
        i2c_comm = *i2c_object;
        writeProtect = DIO(writeIO);
    }
    void read(uint16_t address, uint16_t numbElements);
    void write(uint16_t address, uint16_t numbElements);
    void writeArray(uint16_t* arrayMap[], uint16_t numbElements);
    void readArray(uint16_t* arrayMap[], uint16_t numbElements);
    void init();
    void initWIRE();
    void writeArrayWIRE(uint16_t* arrayMap[], uint16_t numbElements);
};


