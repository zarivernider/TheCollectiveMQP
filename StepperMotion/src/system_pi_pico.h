#ifndef system_pico_h
#define system_pico_h
#include <Arduino.h>
/*
    Included are system level commands that communicate to the hardware of the pi pico directly

*/

// Write registers at an address for a specified number of bits, the proper offset, and the data to be set for the registers
// It is imperative that "data" is the same length as specified by "numbBits"
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data); 

#endif