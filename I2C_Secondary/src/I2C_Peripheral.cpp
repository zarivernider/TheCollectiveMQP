#include "I2C_Peripheral.h"

static void receiveEvent(int howMany)
{

    i2c_p.globAddr = Wire.read(); // First byte is always address
    if(Wire.available()) { // If 16 bit word passed -> write value
        byte tempBuffer[2]; // temp place to store data
        for(int i = 0; i < 2; i++) { // Get the data
            tempBuffer[i] = Wire.read();
        }
        int* valSave = i2c_p.arrMap[i2c_p.globAddr]; // point to the specific memory address of the intended variable
        int value = tempBuffer[0] + (tempBuffer[1] << 8); // Build message
        *valSave = value; // Rewrite variable
    }
    if(Wire.available()) receiveEvent(Wire.available()); // If the buffer is not clear, re-run the function

}
static void requestEvent() {
  int* valRead = i2c_p.arrMap[i2c_p.globAddr]; // Point to the specific memory address of the intended variable
  Wire.write((byte)(*valRead & 0x00FF)); // send LSB first
  Wire.write((byte)(*valRead >> 8)); // send MSB second
}



void I2C_P::init(uint8_t address) {
    // I2C Memory addresses
    uint32_t I2C_Con = I2C0_BASE | 0x00; // I2C Control
    uint32_t I2C_En = I2C0_BASE | 0x6c; // I2C Enable

    I2C_P::I2C_Addr = address; // Store its own address
    Wire.begin(I2C_P::I2C_Addr); // StartI2C at address

    Wire.onReceive(receiveEvent); // Set on recieve function
    Wire.onRequest(requestEvent); // Set on request function

}

