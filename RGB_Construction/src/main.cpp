#include <Arduino.h>
#include <SPI.h>
#include <APA102_Custom.h>
APA102_Cust ledStrip(10);
void setup() {
  // put your setup code here, to run once:
  ledStrip.Init();
  ledStrip.Show();

}

void loop() {
  ledStrip.showUniform(0, 0, 0);
  delay(1000);
  ledStrip.showUniform(0, 0, 255);
  delay(1000);
  ledStrip.showUniform(0, 255, 0);
  delay(1000);
  ledStrip.showUniform(255, 0, 0);
  delay(1000);
}