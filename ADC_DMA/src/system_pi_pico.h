#ifndef system_pico_h
#define system_pico_h
#include <Arduino.h>
/*
    Included are system level commands that communicate to the hardware of the pi pico directly

*/

// Write registers at an address for a specified number of bits, the proper offset, and the data to be set for the registers
// It is imperative that "data" is the same length as specified by "numbBits"
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data); 
#define DMA_CH0_READ_ADDR (DMA_BASE | 0x0)
#define DMA_CH0_WRITE_ADDR (DMA_BASE | 0x4)
#define DMA_CH0_TRANS_COUNT (DMA_BASE | 0x8)
#define DMA_CH0_CTRL_TRIG (DMA_BASE | 0xc)
#define DMA_INTE0 (DMA_BASE | 0x404)
#define DMA_INTS0 (DMA_BASE | 0x40c)
#define DREQ_PWM_WRAP_BASE 24
#define DMA_READ_TO_READ_OFFSET 0x40
#define DREQ_ADC 36
#endif