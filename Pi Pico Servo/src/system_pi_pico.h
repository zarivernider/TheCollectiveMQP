#ifndef system_pico_h
#define system_pico_h
#include <Arduino.h>

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);

#endif