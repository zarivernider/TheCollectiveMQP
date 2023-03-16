#include "I2C_Primary.h"

void I2C_M::init() {    
  
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_DISABLED);
    // Write IC_CON for speed mode and desired speed
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_CON_OFFSET, 2, 1, 1); // Standard 100 kHz
    // Write IC_CON 7 biy addressing
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_CON_OFFSET, 1, 4, 0); 
    // Write IC_CON disable bit 6
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_CON_OFFSET, 1, 6, 1); // Disable secondary
    // Write IC_CON enable Primary mode
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_CON_OFFSET, 1, 0, 1); // Enable Primary

    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SCLioOffset), 5, 0, 3); // func select -> 3 = SCL
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SCLioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SDAioOffset), 5, 0, 3); // func select -> 3 = SDA
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SDAioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_ENABLED); // Enable I2C
    // hw_write_masked((io_rw_32*)(I2C_M::I2C_BASE_OFFSET  | I2C_IC_INTR_MASK_OFFSET),
    //                 0x0260, // 0x0400 is strt
    //                 I2C_IC_INTR_MASK_BITS); // enable TX Abort and RD req and start
    // // Set interrupt handler at vector table
    // irq_set_exclusive_handler(I2C0_IRQ, i2cHandle);
    // irq_set_enabled(I2C0_IRQ, true); // enable interrupt
}

void I2C_M::beginTransmission(uint8_t address) {
    // Disable I2C
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_DISABLED); // Enable I2C
    // Write to IC_TAR
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_TAR_OFFSET, 8, 0, address); // Write 7 bit address
    // Transfer direction
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_TAR_OFFSET, 1, 10, 1); // Gen start bit
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_TAR_OFFSET, 1, 11, 1); // Enable start bit
    // Enable I2C
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_ENABLED); // Enable I2C
}

void I2C_M::write(uint8_t data) {
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_DATA_CMD_OFFSET, 1, 8, 0);
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_DATA_CMD_OFFSET, 8, 0, data);
}

void I2C_M::endTransmission() {
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_DATA_CMD_OFFSET, 1, 9, 1);
    // Disable I2C register
    writeReg(I2C_M::I2C_BASE_OFFSET | I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_DISABLED);
    // Clear poll value
    uint32_t pollValue = 0;
    // Block until either I2C status is off or exceed limit
    while((*(io_rw_32*)(I2C_M::I2C_BASE_OFFSET | I2C_IC_ENABLE_STATUS_OFFSET) & 0x1) && (pollValue <= I2C_M::maxPollvalue)) {
        pollValue++;
    }
}
uint8_t I2C_M::read() {
    // Read cmd 
    uint32_t readFifo = *(io_rw_32*)(I2C_M::I2C_BASE_OFFSET + I2C_IC_DATA_CMD_OFFSET);
    // return data
    return (uint8_t)readFifo;
}

void I2C_M::requestFrom(uint8_t address, uint8_t numbBytes) {
    // Write to IC_TAR
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_TAR_OFFSET, 8, 0, address); // Write 7 bit address
    // Enable I2C
    writeReg(I2C_M::I2C_BASE_OFFSET | I2C_IC_ENABLE_OFFSET, 1, 0, I2C_IC_ENABLE_ENABLE_VALUE_ENABLED); // Enable I2C
    // Transfer direction
    writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_TAR_OFFSET, 1, 10, 1); // Gen start bit

    for(int iter = 0; iter < numbBytes; iter++) writeReg(I2C_M::I2C_BASE_OFFSET + I2C_IC_DATA_CMD_OFFSET, 1, 8, 1);

}

bool I2C_M::isAvailable() {
    uint32_t fifoStatus = *(io_rw_32*)(I2C_M::I2C_BASE_OFFSET | I2C_IC_STATUS_OFFSET);
    fifoStatus = (fifoStatus & 0x8) >> 3;
    return fifoStatus;
}

void I2C_M::blockUntilData() {
    while(!I2C_M::isAvailable()) {}

}

void I2C_M::sdkInit() {
    _i2c_init(I2C_M::i2cBase, baudRate);
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SCLioOffset), 5, 0, 3); // func select -> 3 = SCL
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SCLioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SDAioOffset), 5, 0, 3); // func select -> 3 = SDA
    writeReg(IO_BANK0_BASE | (0x04 + I2C_M::SDAioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    writeReg(PADS_BANK0_BASE + I2C_M::SDApadOffset, 1, 3, 1); // Enable pull-up
    writeReg(PADS_BANK0_BASE + I2C_M::SCLpadOffset, 1, 3, 1); // Enable pull-up

    writeReg(PADS_BANK0_BASE + I2C_M::SDApadOffset, 1, 2, 0); // Disable pull-down
    writeReg(PADS_BANK0_BASE + I2C_M::SCLpadOffset, 1, 2, 0); // Disable pull-down
}
void I2C_M::sdkwriteData(uint8_t address, uint8_t reg, uint16_t data) {
    uint8_t upperByte = data >> 8;
    uint8_t dataArray[3] = {reg, (uint8_t)data, upperByte};

    i2c_write_blocking(I2C_M::i2cBase, address, &dataArray[0], 3, false);

}
// Potentially make a write that has an input array as mem address
uint16_t I2C_M::sdkreadData(uint8_t address, uint8_t reg, uint16_t *memData) {
    uint8_t returnInt[2];
    // i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop
    i2c_write_blocking(I2C_M::i2cBase, address, &reg, 1, true);
    uint16_t numbRead  = i2c_read_blocking(I2C_M::i2cBase, address, returnInt, 2, false);
    *memData =  (returnInt[1] << 8) + returnInt[0];

    return numbRead;
}

// // Potentially make a write that has an input array as mem address
// void I2C_M::sdkreadData16(uint8_t address, uint8_t reg, uint16_t *memData) {
//     uint8_t returnInt[2];
//     // i2c_read_blocking_until(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop
//     i2c_write_blocking(I2C_M::i2cBase, address, &reg, 1, true);
//     uint16_t numbRead  = i2c_read_blocking(I2C_M::i2cBase, address, returnInt, 2, false);
//     *memData =  (returnInt[1] << 8) + returnInt[0];
// }

void I2C_M::sdkwriteAddress(uint8_t i2cAddress, uint16_t memAddress, uint8_t data, uint8_t delaymS)  {
    uint8_t sendArray[3];
    sendArray[0] = memAddress >> 8; // MSB of address first
    sendArray[1] = memAddress & 0xFF; // LSB of address second
    sendArray[2] = data; // Data last
    i2c_write_blocking(I2C_M::i2cBase, i2cAddress, sendArray, 3, false);
    delay(delaymS);
}


void I2C_M::sdkreadArray(uint8_t* returnArray, uint16_t reg, uint8_t length, uint8_t address) {
    uint8_t regSplit[2];
    regSplit[0] = reg >> 8; // Assume MSB first? Datasheet is vague
    regSplit[1] = reg & 0xFF; // LSB second
    i2c_write_blocking(I2C_M::i2cBase, address, regSplit, 2, true);
    i2c_read_blocking(I2C_M::i2cBase, address, returnArray, length, false);
}

