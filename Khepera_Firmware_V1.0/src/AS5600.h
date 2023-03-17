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
        
        uint8_t ioOffset;
        uint8_t pwmSlice;
        
        //15
    public:
    uint16_t rawAngle;
        uint8_t pwmOffset;

        AS5600(uint8_t GPIO) {
            pwmPin = GPIO;
        }
        void init();
        uint16_t getCtr();
};


#endif