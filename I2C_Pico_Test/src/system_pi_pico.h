#ifndef system_pico_h
#define system_pico_h
#include <Arduino.h>
static const uint32_t I2C_Con = I2C0_BASE | 0x00;
static const uint32_t I2C_En = I2C0_BASE | 0x6c;
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);

#endif