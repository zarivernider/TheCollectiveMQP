#include "I2C_Peripheral.h"

static void receiveEvent(int howMany)
{
    i2c_p.globAddr = Wire.read(); // First byte is always address

    if(Wire.available() == 2) { // If 16 bit word passed -> write value
        byte tempBuffer[2]; // temp place to store data
        for(int i = 0; i < 2; i++) { // Get the data
            tempBuffer[i] = Wire.read();
        }
        int* valSave = i2c_p.arrMap[i2c_p.globAddr]; // point to the specific memory address of the intended variable
        int value = tempBuffer[0] + (tempBuffer[1] << 8); // Build message
        *valSave = value; // Rewrite variable
    }
    else while (Wire.available() > 0) Wire.read(); // Clear memory buffer if not cleared
    
}
static void requestEvent() {
  int* valRead = i2c_p.arrMap[i2c_p.globAddr]; // Point to the specific memory address of the intended variable
  Wire.write((byte)(*valRead & 0x00FF)); // send LSB first
  Wire.write((byte)(*valRead >> 8)); // send MSB second
}

// static void receiveEventwShift(int howMa.ny)
// {
//   globAddr = Wire.read();
//   bool isRead = globAddr & 0x1 ? true : false;
//   globAddr = globAddr >> 1;
//   if(isRead) return;
//   else {
//     byte tempBuffer[2];
//     if(Wire.available() >= 2) {
//       for(int i = 0; i < 2; i++) {
//         tempBuffer[i] = Wire.read();
//       }
//     }
//     int* valSave = arrMap[globAddr];
//     int value = tempBuffer[0] + (tempBuffer[1] << 8);
//     *valSave = value;
//   }
// }



void I2C_P::I2C_init() {
    Wire.begin(I2C_Addr);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    writeReg(I2C_En, 1, 0, 0x0); // Disable I2C, mandatory to write to registers
    writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
    writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable 
    writeReg(I2C0_BASE | 0x7c, 16, 0, 40); // IC_SDA_HOLD ~400 nS
    writeReg(I2C0_BASE | 0x7c, 16, 16, 40); // IC_SDA_HOLD ~400 nS
    writeReg(I2C_En, 1, 0, 0x1); // Enable I2C
    // Add GPIO funct
    byte SCL_offset = 0x28; // offset for GPIO5
    byte SDA_offset = 0x20; // offset fo GPIO4
    writeReg(IO_BANK0_BASE | 0x04 | SCL_offset, 5, 0, 4); // func select -> 3 = SCL
    writeReg(IO_BANK0_BASE | 0x04 | SCL_offset, 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | 0x04 | SDA_offset, 5, 0, 3); // func select -> 3 = SDA
    writeReg(IO_BANK0_BASE | 0x04 | SDA_offset, 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
}

