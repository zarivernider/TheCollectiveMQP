#include <Arduino.h>
#include "ADC.h"
ADC adc(0b110, 0);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  adc.initMulti();
  
  adc.calibrateMulti(2000);
  Serial.println("Calibration complete");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.print("Raw 0: ");
  // Serial.print(adc.getrawADCMulti(0));
  // Serial.print("\t Calib 0: ");
  // Serial.print(adc.getADCMulti(0));
  // Serial.print("\t Conv 0: ");
  // Serial.print(adc.getVoltMulti(true, 0));

  // Serial.print("\t Raw 1: ");
  // Serial.print(adc.getrawADCMulti(1));
  // Serial.print("\t Calib 1: ");
  // Serial.print(adc.getADCMulti(1));
  // Serial.print("\t Conv 1: ");
  // Serial.print(adc.getVoltMulti(true, 1));

  // Serial.print("\t Raw 2: ");
  // Serial.print(adc.getrawADCMulti(2));
  // Serial.print("\t Calib 2: ");
  // Serial.print(adc.getADCMulti(2));
  // Serial.print("\t Conv 2: ");
  // Serial.println(adc.getVoltMulti(true, 2));

  // Serial.print("Raw 2: ");
  // Serial.print(adc.getrawADCMulti(2));
  // Serial.print("\t Calib 2: ");channelTRL_TRIG + adc.DMAoffset), HEX); // 
  // Serial.println(*(io_rw_32*)(ADC_BASE + ADC_CS_OFFSET), HEX);
  // Serial.println(adc.DMAoffset, HEX);
  // Serial.println(adc.numbChannels);
  // Serial.println(*(io_rw_32*)(ADC_BASE + ADC_RESULT_OFFSET));
  // Serial.println(adc.getrawADCMulti(2));
  Serial.print("Conv 0: ");
  Serial.print(adc.getVoltMulti(true, 0));
  Serial.print("\t Conv 1: ");
  Serial.print(adc.getVoltMulti(true, 1));
  Serial.print("\t Conv 2: ");
  Serial.println(adc.getVoltMulti(true, 2));

  // Serial.print("Conv 0: ");
  // Serial.print(adc.getADCMulti(0));
  // Serial.print("\t Conv 1: ");
  // Serial.print(adc.getADCMulti(1));
  // Serial.print("\t Conv 2: ");
  // Serial.println(adc.getADCMulti(2));

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