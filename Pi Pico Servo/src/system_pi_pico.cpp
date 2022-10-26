#include "system_pi_pico.h"

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1;
  hw_clear_bits(addrPnt, clrMask << offset);
  hw_set_bits(addrPnt, (data & clrMask) << offset);
}