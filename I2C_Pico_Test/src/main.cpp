#include <Arduino.h>
#include "I2C_Primary.h"
I2C_M i2c_m;
#define I2C_Sec_Address 0x24 // Address of the I2C module 

void testStepper() { // Make the stepper turn
  i2c_m.writeInt(2500, 0x03, I2C_Sec_Address); // Set max speed as 2500
  delay(10);
  i2c_m.writeInt(1000, 0x2, I2C_Sec_Address);
  delay(10);
  i2c_m.writeInt(2, 0x08, I2C_Sec_Address);
  }

void stopStepper() {
  i2c_m.writeInt(0, 0x08, I2C_Sec_Address);
  delay(10);
}
void testALTLED() {
  i2c_m.writeInt(0x6DB6, 0x16, I2C_Sec_Address); // Set max speed as 2500
  delay(10);
  i2c_m.writeInt(0xDB6D, 0x17, I2C_Sec_Address);
  delay(10);
  i2c_m.writeInt(0XB6DB, 0x18, I2C_Sec_Address);
  delay(10);
  i2c_m.writeInt(0X6DB6, 0x19, I2C_Sec_Address); // Set max speed as 2500
  delay(10);
  i2c_m.writeInt(0X1, 0x1A, I2C_Sec_Address);
  delay(10);
}
void testLED() {
  i2c_m.writeInt(0x5555, 0x16, I2C_Sec_Address); // Set max speed as 2500
  delay(10);
  i2c_m.writeInt(0x5555, 0x17, I2C_Sec_Address);
  delay(10);
  i2c_m.writeInt(0x5555, 0x18, I2C_Sec_Address);
  delay(10);
  i2c_m.writeInt(0x5555, 0x19, I2C_Sec_Address); // Set max speed as 2500
  delay(10);
  i2c_m.writeInt(0X1, 0x1A, I2C_Sec_Address);
  delay(10);

}

void openGripper() {
  i2c_m.writeInt(0x1, 0x09, I2C_Sec_Address);
  delay(10);
}


void closeGripper() {
  i2c_m.writeInt(0x0, 0x09, I2C_Sec_Address);
  delay(10);
}

void readForceSensor() {
  uint16_t parallel = i2c_m.readInt(0xB, I2C_Sec_Address);
  delay(10);
  uint16_t perp = i2c_m.readInt(0xC, I2C_Sec_Address);
  Serial.print("Parallel Force: ");
  Serial.print(parallel);
  Serial.print("\t Perpendicular Force: ");
  Serial.println(perp);
}
void setup() {
  // put your setup code here, to run once:
  i2c_m.init();
  Serial.begin(9600);
  delay(4000);
  Serial.println("INIT DONE");
  // testALTLED();
}

void loop() {
  for(int i = 0; i < 3; i++) {
  delay(2000);
  closeGripper();
  delay(2000);
  openGripper();
  }
  testStepper();
  delay(4000);
  stopStepper();
  testALTLED();
  delay(4000);

  // readForceSensor();
  // delay(500);

}



