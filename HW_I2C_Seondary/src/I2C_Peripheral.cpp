#include "I2C_Peripheral.h"
uint32_t finish = 0;

static void i2cHandle() {
    // Serial.print("A: ");
    // original attempt to read registers
    // Serial.println(*(io_rw_32*)(I2C0_BASE | I2C_IC_RAW_INTR_STAT_OFFSET));
    // uint32_t strtClr = *(io_rw_32*)(I2C0_BASE | I2C_IC_CLR_START_DET_OFFSET);
    // uint32_t abtClr = *(io_rw_32*)(I2C0_BASE | I2C_IC_CLR_TX_ABRT_OFFSET);
    // uint32_t reqClr = *(io_rw_32*)(I2C0_BASE | I2C_IC_CLR_RD_REQ_OFFSET);

    // Trial #2 to read register
    uint32_t status = i2c0->hw->intr_stat;
    uint32_t clr = i2c0->hw->clr_start_det;
    clr = i2c0->hw->clr_tx_abrt;
    clr = i2c0->hw->clr_rd_req;

    // Last ditch attempt to write a reset to the clr registers
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CLR_START_DET_OFFSET),
                    I2C_IC_CLR_START_DET_BITS,
                    I2C_IC_CLR_START_DET_RESET); 

    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CLR_TX_ABRT_OFFSET),
                    I2C_IC_CLR_TX_ABRT_BITS,
                    I2C_IC_CLR_TX_ABRT_RESET); 
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_CLR_RD_REQ_OFFSET),
                    I2C_IC_CLR_RD_REQ_BITS,
                    I2C_IC_CLR_RD_REQ_RESET); 

}
void I2C_P::init(uint8_t address) {

    I2C_P::I2C_Addr = address; // Store its own address
    

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

    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_INTR_MASK_OFFSET),
                    I2C_IC_INTR_MASK_BITS,
                    0x0460); // enable TX Abort and RD req and start
    
    // // Add GPIO funct. Pull-up resistors are external
    byte SCL_offset = 0x28; // offset for GPIO5
    byte SDA_offset = 0x20; // offset fo GPIO4
    writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 5, 0, 3); // func select -> 3 = SCL
    writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 5, 0, 3); // func select -> 3 = SDA
    writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    hw_write_masked((io_rw_32*)(I2C0_BASE | I2C_IC_ENABLE_OFFSET),
                    I2C_IC_ENABLE_ENABLE_VALUE_ENABLED,
                    I2C_IC_ENABLE_ENABLE_BITS); // enable I2C
    // Set interrupt handler at vector table
    irq_set_exclusive_handler(I2C0_IRQ, i2cHandle);
    irq_set_enabled(I2C0_IRQ, true); // enable interrupt
// DAT (7:0) of CMD register hold data
uint32_t status = i2c1->hw->intr_stat;

}

