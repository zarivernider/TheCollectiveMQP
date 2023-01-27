#include <AS5600.h>

void AS5600::init() {
    // Calculate register offsets for the pin
    AS5600::pwmScale = (AS5600::pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
    AS5600::pwmSlice = (int)(pwmScale/2);
    AS5600::pwmOffset = AS5600::pwmSlice * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    AS5600::ioOffset = AS5600::pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    // Set GPIO as PWM 
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    // Set CSR Div Mode to 1. Enables input and free running
    writeReg(PWM_BASE | (0x00 + pwmOffset), 2, 4, 1); // CHx_CSR 
    // Set DIV as 16. fsys -> 7,812,500 Hz
    writeReg(PWM_BASE | (0x04 + pwmOffset), 8, 4, 16); // CHx_DIV. 
    // Set top -> 16,982. fpwm -> ~460 Hz
    writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, 16982); // CHx_TOP
    // Enable PWM
    writeReg(PWM_BASE | (0x00 + pwmOffset), 1, 0, 1); // CHx_CSR off enable on
    // After test:
    // Enable DMA for PWM Wrap (1 transfer)

    
}

uint16_t AS5600::getCtr() {
    return *(io_rw_32*)(PWM_BASE | (0x8 + pwmOffset)) & 0xFF;

}