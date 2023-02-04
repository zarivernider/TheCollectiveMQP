#include "I2C_Peripheral.h"
uint32_t finish = 0;

static inline void i2cHandle() {
    uint32_t Tempstatus = i2c0->hw->intr_stat;
    i2c0->hw->clr_start_det;
    i2c0->hw->clr_tx_abrt;
    if(Tempstatus & 0x200) // if stop
    {
        uint16_t message = 0;
        i2c_p.rec = i2c0->hw->rxflr;
        for(uint32_t i = 0; i < i2c_p.rec; i++) {
            uint32_t receive = i2c0->hw->data_cmd & 0xFF;
            if(i == 0)  i2c_p.globAddr = receive;
            else message |= receive << ((i - 1)*8);
        }
        if(i2c_p.rec > 1) {
            uint16_t* valSave = i2c_p.arrMap[i2c_p.globAddr]; // point to the specific memory address of the intended variable
            *valSave = message; // Rewrite variable
        }
        i2c_p.mailBox = true;
        i2c0->hw->clr_stop_det; // clear stop bit

    }
    // if(!i2c_p.mailBox) {
    //     i2c_p.status = Tempstatus;
    //     i2c_p.mailBox = true;
    // }
    else if(Tempstatus & 0x20) {
        uint16_t* valRead = i2c_p.arrMap[i2c_p.globAddr]; // Point to the specific memory address of the intended variable
        i2c0->hw->data_cmd = (byte)(*valRead & 0x00FF); // send LSB first
        i2c0->hw->data_cmd = (byte)(*valRead >> 8); // send MSB second
        i2c0->hw->clr_rd_req;
    }

}
void I2C_P::init(uint8_t address) {

    I2C_P::I2C_Addr = address; // Store its own address
    
    hw_write_masked((io_rw_32*)(WATCHDOG_BASE),
                    0x0, // Get bit 30
                    0x40000000); // Set to 0 (Set ctrl to 0)
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
                    0x0260, // 0x0400 is strt
                    I2C_IC_INTR_MASK_BITS); // enable TX Abort and RD req and start
    
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

}


