#include "system_pi_pico.h"

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  // Write registers at an address for a specified number of bits, the proper offset, and the data to be set for the registers
  // It is imperative that "data" is the same length as specified by "numbBits"
  
  io_rw_32* addrPnt = (io_rw_32*)(addr); // Type cast input 32 bit address to datatype readable by the manipulating register command
  uint32_t clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1; // Create a mask that is the size of the input number of bits
  hw_clear_bits(addrPnt, clrMask << offset); // Clear the specific set of bits that is offset by an input amount and has a variable size of input length
  hw_set_bits(addrPnt, (data & clrMask) << offset); // Mask the input data to be the proper size and shift it to be properly offset
}