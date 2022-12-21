#include "I2C_Peripheral.h"
uint32_t finish = 0;


void I2C_P::init(uint8_t address) {
    // I2C Memory addresses
    // uint32_t I2C_Con = I2C0_BASE | 0x00; // I2C Control
    // uint32_t I2C_En = I2C0_BASE | 0x6c; // I2C Enable

    I2C_P::I2C_Addr = address; // Store its own address
    // Wire.begin(I2C_P::I2C_Addr); // StartI2C at address

    // Wire.onReceive(receiveEvent); // Set on recieve function
    // Wire.onRequest(requestEvent); // Set on request function

    // writeRe    // Wire.begin(I2C_P::I2C_Addr); // StartI2C at address

    // Wire.    // Wire.begin(I2C_P::I2C_Addr); // StartI2C at address

    // Wire.onReceive(receiveEvent); // Set on recieve function
    // Wire.onRequest(requestEvent); // Set on request functiononReceive(receiveEvent); // Set on recieve function
    // Wire.onRequest(requestEvent); // Set on request functiong(I2C_En, 1, 0, 0x0); // Disable I2C, mandatory to write to registers
    // writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
    // writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable 
    // writeReg(I2C0_BASE | 0x7c, 16, 0, 40); // IC_SDA_HOLD ~400 nS
    // writeReg(I2C0_BASE | 0x7c, 16, 16, 40); // IC_SDA_HOLD ~400 nS
    // // Add GPIO funct
    // byte SCL_offset = 0x28; // offset for GPIO5
    // byte SDA_offset = 0x20; // offset fo GPIO4
    // writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 5, 0, 3); // func select -> 3 = SCL
    // writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    // writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 5, 0, 3); // func select -> 3 = SDA
    // writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    // writeReg(I2C_En, 1, 0, 0x1); // Enable I2C

    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_ENABLE_OFFSET),
                    I2C_IC_ENABLE_ENABLE_VALUE_DISABLED,
                    I2C_IC_ENABLE_ENABLE_BITS); // disable I2C
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_SAR_OFFSET),
                    I2C_P::I2C_Addr & 0x7F, // Only keep 7 bits
                    I2C_IC_SAR_BITS); // set secondary address
    
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET),
                    I2C_IC_CON_IC_10BITADDR_SLAVE_VALUE_ADDR_7BITS,
                    I2C_IC_CON_IC_10BITADDR_SLAVE_BITS); // set 7 bit address
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET),
                    I2C_IC_CON_SPEED_VALUE_STANDARD,
                    I2C_IC_CON_SPEED_BITS); // set speed
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET),
                    I2C_IC_CON_MASTER_MODE_VALUE_DISABLED,
                    I2C_IC_CON_MASTER_MODE_BITS); // disable primary
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CON_OFFSET),
                    I2C_IC_CON_IC_SLAVE_DISABLE_VALUE_SLAVE_ENABLED,
                    I2C_IC_CON_IC_SLAVE_DISABLE_BITS); // enable secondary
  


    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_ENABLE_OFFSET),
                    I2C_IC_ENABLE_ENABLE_VALUE_ENABLED,
                    I2C_IC_ENABLE_ENABLE_BITS); // enable I2C
// Set interrupt handler at vector table
// DAT (7:0) of CMD register hold data

}