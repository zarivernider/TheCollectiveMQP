#include <Arduino.h>
byte tempBuffer[6]; 
char packetSize = 3;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() >= packetSize && (Serial.available() % packetSize == 0))  {
    Serial.readBytes(tempBuffer, packetSize);
    Serial.print("Address: ");
    Serial.print(tempBuffer[0]);
    Serial.print("\t");
    Serial.print("Value: ");
    Serial.println(tempBuffer[1] + (tempBuffer[2] << 8));
    //Handle buffer
  }
  // else if(Serial.available() < 3 && Serial.available() != 0) while(Serial.available()) Serial.read(); // If buffer is not 0 and not 3 packets, clear it
  


  // while(Serial.available()) {
  //   byte output = Serial.read(); 
  //   Serial.println(output);
  // }
  // Serial.println("Buffer clear");
  // delay(2000);

}