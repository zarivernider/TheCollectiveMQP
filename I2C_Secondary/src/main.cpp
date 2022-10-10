#include <Arduino.h>
#include <Wire.h>
//LSB first
int* arrMap[30];
int testVal = 0;
byte globAddr = 0;
void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data);
uint32_t I2C_Con = I2C0_BASE | 0x00;
uint32_t I2C_En = I2C0_BASE | 0x6c;
void receiveEvent(int howMany)
{
  globAddr = Wire.read();
  bool isRead = globAddr & 0x1 ? true : false;
  globAddr = globAddr >> 1;
  if(isRead) return;
  else {
    byte tempBuffer[2];
    if(Wire.available() >= 2) {
      for(int i = 0; i < 2; i++) {
        tempBuffer[i] = Wire.read();
      }
    }
    int* valSave = arrMap[globAddr];
    int value = tempBuffer[0] + (tempBuffer[1] << 8);
    *valSave = value;
  }
}
void requestEvent() {
  int* valRead = arrMap[globAddr];
  int value = *valRead;
  Wire.write((byte)(*valRead & 0x00FF)); // LSB first
  Wire.write((byte)(*valRead >> 8)); // MSB second
}
void setup() {
  arrMap[1] = &testVal;
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(0x13);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  writeReg(I2C_En, 1, 0, 0x0);
  writeReg(I2C_Con, 2, 1, 0x2); // Turn on fast mode
  writeReg(I2C0_BASE | 0x94, 8, 0, 12); // SDA Enable (BOTH)
  writeReg(I2C0_BASE | 0x7c, 16, 0, 40); // IC_SDA_HOLD ~400 nS
  writeReg(I2C0_BASE | 0x7c, 16, 16, 40); // IC_SDA_HOLD ~400 nS
  writeReg(I2C_En, 1, 0, 0x1);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}

void writeReg(uint32_t addr, uint8_t numbBits, uint8_t offset, uint32_t data) {
  io_rw_32* addrPnt = (io_rw_32*)(addr);
  long clrMask = numbBits == 32 ? 0xFFFFFFFF : (0x1 << (numbBits)) - 1;
  hw_clear_bits(addrPnt, clrMask << offset);
  hw_set_bits(addrPnt, (data & clrMask) << offset);
}
