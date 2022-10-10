#include <Arduino.h>
#include <Wire.h>
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);
uint32_t I2C_Con = I2C0_BASE | 0x00;
uint32_t I2C_En = I2C0_BASE | 0x6c;
void setup() {
  // put your setup code here, to run once:
  Wire.begin(0x13);
  writeReg(I2C_En, 1, 0, 0x0);
  writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
  writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable (BOTH)
  writeReg(I2C0_BASE | 0x7c, 16, 0, 40); // IC_SDA_HOLD ~400 nS
  writeReg(I2C0_BASE | 0x7c, 16, 16, 40); // IC_SDA_HOLD ~400 nS
  writeReg(I2C_En, 1, 0, 0x1);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1;
  hw_clear_bits(addrPnt, clrMask << offset);
  hw_set_bits(addrPnt, data << offset);
}