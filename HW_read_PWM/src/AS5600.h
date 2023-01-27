#ifndef AS5600_h
#define AS5600_h
#include <Arduino.h>
#include "system_pi_pico.h"
/*
    Class to use the ADC of the Pi Pico (RP 2040)
    Please note that with consecutive channels, DMA cannot jump meaning only channels 0 and 2 will not work and all three channels must be active in this case
    **Potential bug: The ADC channel not corresponding to the array value. May be resolved, but keep an eye out
*/


class AS5600
{
    private:

        uint8_t pwmPin = 0;
        uint8_t pwmScale;
        uint8_t pwmOffset;
        uint8_t ioOffset;
        uint8_t pwmSlice;
        //15
    public:
        uint8_t DMAnumber;
        uint32_t DMAoffset;

        AS5600(uint8_t GPIO, uint8_t DMA_Channel) {
            DMAnumber = DMA_Channel;
            pwmPin = GPIO;
        }
        void init();
        uint16_t getCtr();




};


#endif