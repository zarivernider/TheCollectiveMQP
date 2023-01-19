#include <DIO.h>

void DIO::init() {
   // Init gpio dir
    DIO::ioOffset = DIO::ioPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + DIO::ioOffset ), 2, 12, 3); // Pin output -> 3 = enable output, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + DIO::ioOffset ), 2, 8, 2); // Pin output -> 3 = drive output high, 2 = drive output low
}

void DIO::assertIO(bool isHigh) {
    uint8_t dirVal = isHigh ? 0x3 : 0x2; 
    writeReg(IO_BANK0_BASE | (0x04 + DIO::ioOffset), 2, 8, dirVal); // Pin output -> 3 = drive output high, 2 = drive output low
}