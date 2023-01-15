#include "ADC.h"

void ADC::init() {
    uint8_t gpioPin = ADC::GPIO_ADC_OFFSET + ADC::ain_sel;
    // Enable gpio
    uint8_t ioOffset = gpioPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 1, 16, 0); // Enable as output and do not invert signal

    writeReg(ADC_BASE + ADC_CS_OFFSET, 3, 12, ADC::ain_sel);
    // Set the ADC freq to 1000 samples per second
    writeReg(ADC_BASE + ADC_DIV_OFFSET, 8, 0, 0x0); // Set frac for 0 -> Clock is 48 MHz
    writeReg(ADC_BASE + ADC_DIV_OFFSET, 16, 8, 0xBB7F); // Set int for 47,999
    writeReg(ADC_BASE + ADC_CS_OFFSET, 1, 3, 1); // Set ADC to start many
    writeReg(ADC_BASE + ADC_CS_OFFSET, 1, 0, 1); // Set ADC to enable
}

uint16_t ADC::getADC() {
    // return (ADC::getRawADC() - ADC::calib) & 0xFFF; // Remove calibration 
    uint16_t rawADC = ADC::getRawADC();
    return rawADC < ADC::calib ? 0 : rawADC - ADC::calib;

}

uint16_t ADC::getRawADC() {
    return *(io_rw_32*)(ADC_BASE + ADC_RESULT_OFFSET) & 0xFFF;
}

void ADC::calibrateADC(uint16_t numbValues) {
    uint32_t totalSum = 0;
    for(int iter = 0; iter < numbValues; iter++) {
        totalSum += ADC::getRawADC();
        delay(1);
    }
    ADC::calib = totalSum / numbValues;
}

float ADC::getVolt_float(bool isCalib) {
    uint16_t ADCval = isCalib ? ADC::getADC() : ADC::getRawADC();
    return (float)(3.3 * ADCval) / 4096;
}