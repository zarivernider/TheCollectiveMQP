#include "EEPROM.h"

void EEPROM::writeArray(uint16_t* arrayMap[], uint16_t numbElements) {
    EEPROM::isWrite(true); // Enable write control
    uint8_t sendArray[66]; // Size is max page size of 64 bytes and 16 bit address
    sendArray[0] = 0x0; // Initial address is 0
    sendArray[1] = 0x0;
    // assume all words end up on the same page
    for (int iter = 2; iter < numbElements + 2; iter++) {
        // EEPROM structured as little endian (MSB at the lower address)
        sendArray[2*iter] = *arrayMap[iter] >> 8;
        sendArray[2*iter + 1] = *arrayMap[iter] & 0xFF;
    }
    EEPROM::i2c_comm.sdkwriteArray(sendArray, numbElements*2, i2cAddress);
    EEPROM::isWrite(false); // Disable write
}


void EEPROM::readArray(uint16_t* arrayMap[], uint16_t numbElements) {
    uint8_t returnArray[65];
    EEPROM::i2c_comm.sdkreadArray(returnArray, 0x0, numbElements*2, i2cAddress);
    for(int iter = 0; iter < numbElements; iter++) {
        *arrayMap[iter] = (returnArray[2*iter] << 8) + returnArray[2*iter + 1];
    }

}

void EEPROM::init() {
    EEPROM::i2c_comm.sdkInit();
    writeProtect.init();
    EEPROM::isWrite(false); // Disable write feature
}

void EEPROM::isWrite(bool isEnable) {
    writeProtect.assertIO(!isEnable);
}