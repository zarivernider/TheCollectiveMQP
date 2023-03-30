#include "EEPROM.h"

void EEPROM::writeArray(uint16_t* arrayMap[], uint16_t numbElements) {
    EEPROM::isWrite(true); // Enable write control
    for (int iter = 0; iter < numbElements; iter++) {
        // EEPROM structured as little endian (MSB at the lower address)
        if(iter == 0) Serial.println(*arrayMap[0]);
        uint8_t MSB = *arrayMap[iter] >> 8;
        uint8_t LSB = *arrayMap[iter] & 0xFF;
        i2c_comm.sdkwriteAddress(i2cAddress, 2*iter, MSB, writeDelay);
        i2c_comm.sdkwriteAddress(i2cAddress, 2*iter + 1, LSB, writeDelay);
    }
    EEPROM::isWrite(false); // Disable write
}


void EEPROM::readArray(uint16_t* arrayMap[], uint16_t numbElements) {
    uint8_t returnArray[100]; // Arbitrarily large
    EEPROM::i2c_comm.sdkreadArray(returnArray, 0x0, numbElements*2, i2cAddress); // Read all of the EEPROM values
    for(int iter = 0; iter < numbElements; iter++) {
        // Set each of the values from the EEPROM
        *arrayMap[iter] = (returnArray[2*iter] << 8) + returnArray[2*iter + 1];
    }

}

void EEPROM::init() {
    // Wire1.begin();
    EEPROM::i2c_comm.sdkInit();
    writeProtect.init();
    
    EEPROM::isWrite(false); // Disable write feature
}

void EEPROM::isWrite(bool isEnable) {
    writeProtect.assertIO(!isEnable);
}

void EEPROM::read(uint16_t address, uint16_t numbElements) {
    uint8_t returnInt[64];
    i2c_comm.sdkreadArray(returnInt, address, numbElements, i2cAddress);
    for(int iter = 0; iter < numbElements; iter++) {
        Serial.print("Address: ");
        Serial.print(iter / 2 + address/2, HEX);
        Serial.print("\t Value: ");
        Serial.println(returnInt[iter], HEX);
    }
}

// void EEPROM::writeArrayWIRE(uint16_t* arrayMap[], uint16_t numbElements) {
//     EEPROM::isWrite(true); // Enable write control
//     uint8_t sendArray[2]; // Size is max page size of 64 bytes and 16 bit address
//     Wire1.beginTransmission(i2cAddress);
//     Wire1.write(0x0);
//     Wire1.write(0x0);
//     delay(6);
//     // assume all words end up on the same page
//     for (int iter = 0; iter < numbElements; iter++) {
//         // EEPROM structured as little endian (MSB at the lower address)
//         sendArray[2*iter] = *arrayMap[iter] >> 8;
//         sendArray[2*iter + 1] = *arrayMap[iter] & 0xFF;
//                     // EEPROM::i2c_comm.sdkwriteArray(sendArray, 1, i2cAddress, true);
//         // Put first element in TX buffer
//         Wire1.write(sendArray[0]);
//         delay(6);
//         Wire1.write(sendArray[1]); // Put second element in TX buffer
//         delay(6);
//     }
//     // EEPROM::i2c_comm.sdkwriteArray(sendArray, numbElements*2 + 2, i2cAddress);
//     Wire1.endTransmission();
//     EEPROM::isWrite(false); // Disable write
// }