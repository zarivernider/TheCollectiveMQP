#include <Arduino.h>
#include<Wire.h>
void writeReg(long addr, uint8_t numbBits, uint8_t offset, long data);
void setup() {
  // put your setup code here, to run once:
  // I2C0_BASE & I2C_IC_CON_BITS &= !0x6;
  
  writeReg(IO_BANK0_BASE | 0x004, 2, 12, 0x3);

  writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);
  // Wire.begin();
}

void loop() {
  delay(2000);
  writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x3);
  delay(2000);
  writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);
  // byte error, address;
  // int nDevices;
 
  // Serial.println("Scanning...");
 
  // nDevices = 0;
  // for(address = 1; address <= 127; address++)
  // {
  //   // The i2c_scanner uses the return value of
  //   // Wire.endTransmission to see if
  //   // a device acknowledged the address.
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();
 
  //   if (error == 0)
  //   {
  //     Serial.print("I2C device found at address 0x");
  //     if (address<16)
  //       Serial.print("0");
  //     Serial.print(address,HEX);
  //     Serial.println("  !");
 
  //     nDevices++;
  //   }
  //   else if (error==4)
  //   {
  //     Serial.print("Unknown error at address 0x");
  //     if (address<16)
  //       Serial.print("0");
  //     Serial.println(address,HEX);
  //   }    
  // }
  // if (nDevices == 0)
  //   Serial.println("No I2C devices found\n");
  // else
  //   Serial.println("done\n");
 
  // delay(5000);           // wait 5 seconds for next scan
 
}
//IO_BANK0_BASE | 0x004)

void writeReg(long addr, uint8_t numbBits, uint8_t offset, long data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFF : (0x1 << (numbBits)) - 1;
  hw_clear_bits(addrPnt, clrMask << offset);
  hw_set_bits(addrPnt, data << offset);
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