#include <Arduino.h>
char packetSize = 3;
byte tempBuffer[3]; 
unsigned long memOffset = 0x20000000;
// int testVal = MEM(memOffset + 0x1, unsigned);
int testVal = 0;
int* arrMap[30];
void setup() {
  // put your setup code here, to run once:
  arrMap[1] = &testVal;
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() >= packetSize && (Serial.available() % packetSize == 0))  { // Make sure there is an even 3 amount of data points and at least 3 exist
    Serial.readBytes(tempBuffer, packetSize); // Read 3 and place them in a temp buffer
    Serial.print("Address: ");
    byte address = tempBuffer[0] >> 1;
    Serial.print(address);
    Serial.print("\t");
    Serial.print("Value: ");
    int value = tempBuffer[1] + (tempBuffer[2] << 8);
    Serial.println(value);
    int* valSave = arrMap[address];
    *valSave = value;
    //Handle buffer
  }
  Serial.print("testVal: ");
  Serial.println(testVal);
  delay(2000);
  // else if(Serial.available() < 3 && Serial.available() != 0) while(Serial.available()) Serial.read(); // If buffer is not 0 and not 3 packets, clear it
  


  // while(Serial.available()) {
  //   byte output = Serial.read(); 
  //   Serial.println(output);
  // }
  // Serial.println("Buffer clear");
  // delay(2000);

}