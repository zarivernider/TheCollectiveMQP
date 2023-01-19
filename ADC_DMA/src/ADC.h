#ifndef ADC_Driver_h
#define ADC_Driver_h
#include <Arduino.h>
#include "system_pi_pico.h"

class ADC
{
    private:
        #define numbADCchannels 3
        uint8_t ain_sel;
        
        
        float fsVolt = 3.3; // full scale voltage
        uint8_t GPIO_ADC_OFFSET = 26; // GPIO 26 represents ADC 0
        uint16_t calib = 0;
        uint8_t activeChannels = 0;
        // uint8_t numbChannels;
        
        // uint16_t *ADC0Val = &rawADC[0];
        // uint16_t *ADC1Val = 
        
    public:
        uint8_t enterInt = 0; // TEST 
        uint8_t numbChannels;
        uint8_t DMAnumber;
        uint16_t rawADC[numbADCchannels];
        uint8_t firstChannel;
        uint32_t DMAoffset;
        uint32_t ADCresult = 0;
        ADC(uint8_t ADC_Channel) {
            if(ADC_Channel > 2) ain_sel = 2;
            else ain_sel = ADC_Channel;
            
        }
        ADC(uint8_t channels, uint8_t DMA_Channel) {
            DMAnumber = DMA_Channel;
            activeChannels = channels;
            // activeChannels &= 0b111; // FIX LATER 
        }
        void initSingle();
        float getVolt_float_Single(bool isCalib);
        uint16_t getADC_Single();
        uint16_t getRawADC_Single();
        void calibrateADC_Single(uint16_t numbValues);

        void initMulti();



};

extern ADC adc;

#endif