#include "I2C_Primary.h"

void I2C_M::init() {
    Wire.begin();
    byte SDA_offset = 0x20; // offset fo GPIO4
    byte SCL_offset = 0x28; // offset for GPIO5
    writeReg(I2C_En, 1, 0, 0x0); // Disable I2C
    writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
    writeReg(I2C0_BASE | 0x1c, 16, 0, 89); // SCL_HCNT
    writeReg(I2C0_BASE | 0x20, 16, 0, 135+25); // SCL_LCNT
    writeReg(I2C0_BASE | 0xa0, 8, 0, 20); // SPKLEN 
    writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable 
    // Add GPIO funct
    writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 5, 0, 3); // func select -> 3 = SCL
    writeReg(IO_BANK0_BASE | (0x04 + SCL_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 5, 0, 3); // func select -> 3 = SDA
    writeReg(IO_BANK0_BASE | (0x04 + SDA_offset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    
    writeReg(I2C_En, 1, 0, 0x1); // Enable I2C

}


void I2C_M::writeInt(int data, byte reg, byte I2C_Addr) {
  Wire.beginTransmission(I2C_Addr); // Begin transmission at specific address
  Wire.write(reg); // Write register address
  Wire.write((byte)(data & 0x00FF)); // Send LSB
  Wire.write((byte)(data >> 8)); // Send MSB
  Wire.endTransmission(); // End transmission
}

int I2C_M::readInt(byte reg, byte I2C_Addr) {
    Wire.beginTransmission(I2C_Addr); // Begin transmission at specific address
    Wire.write(reg); // Write register address
    Wire.endTransmission(); // Temp end transmission
    Wire.requestFrom(I2C_Addr, 2); // Get the new data at specified address
    int output; // Create temp variable to hold output
    while(Wire.available() < 2) { delay(10); } // Wait until 2 bytes are received

    output = Wire.read(); // Get LSB 
    output |= (Wire.read() << 8); // get MSB
    return output;
}



// Bit shifted R/W (DEPRECATED)
// void writeI2CInt(int data, byte reg, byte I2C_Addr) {
//   Wire.beginTransmission(I2C_Addr);
//   Wire.write(reg << 1);
//   Wire.write((byte)(data & 0x00FF));
//   Wire.write((byte)(data >> 8));
//   Wire.endTransmission();
// }

// int readI2CInt(byte reg, byte I2C_Addr) {
//   Wire.beginTransmission(I2C_Addr);
//   Wire.write((reg << 1) | 0x1);
//   Wire.endTransmission();
//   Wire.requestFrom(I2C_Addr, 2);
  
//   int output = Wire.read();
//   output |= (Wire.read() << 8);
//   return output;
// }