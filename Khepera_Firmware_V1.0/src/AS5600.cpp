#include <AS5600.h>
uint32_t staticPWM_CTR_Addr;
volatile uint16_t registerValue = 0;

static inline void GPIOhandler() {
    writeReg(IO_BANK_INTR1, 1, 30, 1);
    registerValue = *(io_rw_32*)(staticPWM_CTR_Addr) & 0xFFFF; // Save value
    hw_write_masked((io_rw_32*)staticPWM_CTR_Addr,
                    0x0,
                    0xFFFF); // Clear top

}
void AS5600::init() {
    // Calculate register offsets for the pin
    AS5600::pwmScale = (AS5600::pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
    AS5600::pwmSlice = (int)(pwmScale/2);
    AS5600::pwmOffset = AS5600::pwmSlice * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    AS5600::ioOffset = AS5600::pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    staticPWM_CTR_Addr = (PWM_BASE | (0x8 + AS5600::pwmOffset));
    // Set GPIO as PWM 
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    // Set CSR Div Mode to 1. Enables input and free running
    writeReg(PWM_BASE | (0x00 + pwmOffset), 2, 4, 1); // CHx_CSR 
    // Set DIV as 6. fsys -> 20.83 MHz 
    writeReg(PWM_BASE | (0x04 + pwmOffset), 8, 4, 6); // CHx_DIV. 
    // Set top -> max to never roll over. max time is ~3mS
    writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, 0xFFFF); // CHx_TOP
    // Enable PWM
    writeReg(PWM_BASE | (0x00 + pwmOffset), 1, 0, 1); // CHx_CSR off enable on
    // Enable interrupt
    writeReg(IO_BANK_PROC0_INTE1, 1, 30, 1);
    irq_set_exclusive_handler(IO_IRQ_BANK0, GPIOhandler);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

uint16_t AS5600::getCtr() {
    // return *(io_rw_32*)(PWM_BASE | (0x8 + pwmOffset)) & 0xFF;

    return registerValue;

}