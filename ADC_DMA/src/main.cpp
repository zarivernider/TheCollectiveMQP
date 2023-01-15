#include <Arduino.h>
#include "ADC.h"

ADC adc(2, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  adc.init();
  adc.calibrateADC(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Raw ADC: ");
  Serial.print(adc.getRawADC());
  Serial.print("\t Raw Volts: ");
  Serial.print(adc.getVolt_float(false));
  Serial.print("\t Calibrated ADC: ");
  Serial.print(adc.getADC());
  Serial.print("\t Calibrated voltage: ");
  Serial.println(adc.getVolt_float(true));
  delay(1000);
}