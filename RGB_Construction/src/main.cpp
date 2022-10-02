#include <Arduino.h>
#include <SPI.h>
#include <APA102_Custom.h>
APA102_Cust ledStrip(170);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ledStrip.Init();
  ledStrip.Show();

}

void loop() {
  Serial.println("Clear");
  ledStrip.showUniform(0, 0, 0);
  delay(2000);
  Serial.println("Red");
  ledStrip.showUniform(255, 0, 0);
  delay(2000);
  Serial.println("Green");
  ledStrip.showUniform(0, 255, 0);
  delay(2000);
  Serial.println("Blue");
  ledStrip.showUniform(0, 0, 255);
  delay(2000);
}