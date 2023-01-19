#include <Arduino.h>
#include "ADC.h"
ADC adc(0b110, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  adc.initMulti();
  
  // adc.calibrateADC_Single(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("ADC0: ");
  Serial.print(adc.rawADC[0]);
  Serial.print("\t ADC1: ");
  Serial.print(adc.rawADC[1]);
  Serial.print("\t ADC2: ");
  Serial.print(adc.rawADC[2]);
  Serial.print("\t TEST: ");
  // Serial.println(*(io_rw_32*)(DMA_CH0_CTRL_TRIG + adc.DMAoffset), HEX); // 
  // Serial.println(*(io_rw_32*)(ADC_BASE + ADC_CS_OFFSET), HEX);
  // Serial.println(adc.DMAoffset, HEX);
  // Serial.println(adc.numbChannels);
  Serial.println(*(io_rw_32*)(ADC_BASE + ADC_FCS_OFFSET), HEX);
  delay(1000);

}

// For single operation
// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   adc.initSingle();
//   adc.calibrateADC_Single(2000);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   Serial.print("Raw ADC: ");
//   Serial.print(adc.getRawADC_Single());
//   Serial.print("\t Raw Volts: ");
//   Serial.print(adc.getVolt_float_Single(false));
//   Serial.print("\t Calibrated ADC: ");
//   Serial.print(adc.getADC_Single());
//   Serial.print("\t Calibrated voltage: ");
//   Serial.println(adc.getVolt_float_Single(true));
//   delay(1000);
// }