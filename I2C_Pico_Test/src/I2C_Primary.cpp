#include "I2C_Primary.h"

void I2C_M::init() {
    Wire.begin();
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
