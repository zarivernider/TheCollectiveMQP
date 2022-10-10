#include <Arduino.h>
#include<Wire.h>
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);
void writeI2CInt(int data, byte addr);
int readI2CInt(byte reg);
uint32_t I2C_Con = I2C0_BASE | 0x00;
uint32_t I2C_En = I2C0_BASE | 0x6c;
void setup() {
  // put your setup code here, to run once:
  // I2C0_BASE & I2C_IC_CON_BITS &= !0x6;
  Serial.begin(9600);
  Wire.begin();
  writeReg(I2C_En, 1, 0, 0x0); // Disable I2C
  // writeReg(I2C_Con, 2, 1, 0x1); // Turn on fast mode, but it is not
  // writeReg(I2C0_BASE | 0x1c, 16, 0, 34); // SCL_HCNT
  // writeReg(I2C0_BASE | 0x20, 16, 0, 50); // SCL_LCNT
  // writeReg(I2C0_BASE | 0xa0, 8, 0, 20); // SPKLEN chose at guess :/
  // writeReg(I2C0_BASE | 0x94, 8, 0, 7); // SDA Enable (BOTH)
  writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
  writeReg(I2C0_BASE | 0x1c, 16, 0, 89); // SCL_HCNT
  writeReg(I2C0_BASE | 0x20, 16, 0, 135+25); // SCL_LCNT
  writeReg(I2C0_BASE | 0xa0, 8, 0, 20); // SPKLEN chose at guess :/
  writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable 


  // writeReg(I2C0_BASE | 0x7c, 16, 0, 40);
  // writeReg(I2C0_BASE | 0x7c, 16, 16, 40);

  writeReg(I2C_En, 1, 0, 0x1); // Enable I2C
  delay(20000);
}

void loop() {
  Serial.print("Read val ");
  int out = readI2CInt(1);
  Serial.print(out);
  Serial.print("\t Write 452\t");
  writeI2CInt(4548, 1);
  
  out = readI2CInt(1);
  Serial.print("written ");
  Serial.println(out);
  delay(5000);
  writeI2CInt(0, 1);
  Serial.println("Cleared");
 
}
//IO_BANK0_BASE | 0x004)

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1;
  hw_clear_bits(addrPnt, clrMask << offset);
  hw_set_bits(addrPnt, (data & clrMask) << offset);
}
void writeI2CInt(int data, byte reg) {
  Wire.beginTransmission(0x13);
  Wire.write(reg << 1);
  Wire.write((byte)(data & 0x00FF));
  Wire.write((byte)(data >> 8));
  Wire.endTransmission();
}

int readI2CInt(byte reg) {
  Wire.beginTransmission(0x13);
  Wire.write((reg << 1) | 0x1);
  Wire.endTransmission();
  Wire.requestFrom(0x13, 2);
  int output = Wire.read();
  output |= (Wire.read() << 8);
  return output;
}
// Init
// hw_set_bits(TEST, 0x3 << 12); // Set bits enable output

// hw_clear_bits(TEST, 0x3 << 8); // Clear output pins
// hw_set_bits(TEST, 0x2 << 8); // Set pins low

// Loop to alternate IO pin
// delay(2000);
// hw_clear_bits(TEST, 0x3 << 8); // Clear output pins
// hw_set_bits(TEST, 0x2 << 8); // Set pins low
// delay(2000);
// hw_clear_bits(TEST, 0x3 << 8); // Clear output pins
// hw_set_bits(TEST, 0x3 << 8); // Set pins high

  // writeReg(IO_BANK0_BASE | 0x004, 2, 12, 0x3);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);
  //  delay(2000);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x3);
  // delay(2000);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);