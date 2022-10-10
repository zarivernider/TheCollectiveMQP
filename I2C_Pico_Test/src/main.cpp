#include <Arduino.h>
#include<Wire.h>
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);
uint32_t I2C_Con = I2C0_BASE | 0x00;
uint32_t I2C_En = I2C0_BASE | 0x6c;
void setup() {
  // put your setup code here, to run once:
  // I2C0_BASE & I2C_IC_CON_BITS &= !0x6;
  
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
}

void loop() {
 
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address <= 127; address++)
  {
    // The i2c_scanner uses the return value of
    // Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
 
}
//IO_BANK0_BASE | 0x004)

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1;
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

  // writeReg(IO_BANK0_BASE | 0x004, 2, 12, 0x3);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);
  //  delay(2000);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x3);
  // delay(2000);
  // writeReg(IO_BANK0_BASE | 0x004, 2, 8, 0x2);