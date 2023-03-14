#include "I2C_Primary.h"
#include "DIO.h"
#define i2cAddress 0x50 // Address with no modification

class EEPROM
{
private:
    I2C_M i2c_comm;
    DIO writeProtect;

    void isWrite(bool isEnable);
public:
    EEPROM(uint8_t SDApin, uint8_t SCLpin, i2c_inst_t *i2c, uint8_t writeIO) {
        i2c_comm = I2C_M(SDApin, SCLpin, i2c);
        writeProtect = DIO(writeIO);
    }

    EEPROM(I2C_M *i2c_object, uint8_t writeIO) {
        i2c_comm = *i2c_object;
        writeProtect = DIO(writeIO);
    }

    void writeArray(uint16_t* arrayMap[], uint16_t numbElements);
    void readArray(uint16_t* arrayMap[], uint16_t numbElements);
    void init();
};


