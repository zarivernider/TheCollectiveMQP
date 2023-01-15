#ifndef ADC_Driver_h
#define ADC_Driver_h
#include <Arduino.h>
#include "system_pi_pico.h"

class ADC
{
    private:
        uint8_t ain_sel;
        uint8_t DMAnumber;
        uint32_t DMAoffset;
        float fsVolt = 3.3; // full scale voltage
        uint8_t GPIO_ADC_OFFSET = 26; // GPIO 26 represents ADC 0
        uint16_t calib = 0;
    public:
        uint32_t ADCresult = 0;
        ADC(uint8_t ADC_Channel, uint8_t DMA_Channel) {
            if(ADC_Channel > 2) ain_sel = 2;
            else ain_sel = ADC_Channel;
            DMAnumber = DMA_Channel;
        }
        void init();
        float getVolt_float(bool isCalib);
        uint32_t getVolt_fix(bool isCalib);
        uint8_t getVolt_khepera(bool isCalib);
        uint16_t getADC();
        uint16_t getRawADC();
        void calibrateADC(uint16_t numbValues);


};

#endif