#include "ADC.h"
static inline void dmaHandler() {
    // // Get interrupt status
    // uint32_t status = *(io_rw_32*)DMA_INTS0;
    // Clear interrupt
    hw_write_masked((io_rw_32*)(DMA_INTS0),
                    0x1 << adc.DMAnumber, 
                    0xffff); // clear interrupt
    // Rewrite start address
    // *(io_rw_32*)(DMA_CH0_WRITE_ADDR ) |= (uint32_t)&adc.rawADC[adc.firstChannel];
    hw_write_masked((io_rw_32*)(DMA_CH0_WRITE_ADDR + adc.DMAoffset), 
                    (uint32_t)&adc.rawADC[adc.firstChannel],
                    0xFFFFFFFF); // set write address as rawADC array
    writeReg(ADC_BASE + ADC_CS_OFFSET, 3, 12, adc.firstChannel); // Set starting channel to the first channel
    // Restart DMA
    // *(io_rw_32*)(DMA_CH0_CTRL_TRIG ) |= 0x1;
    hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + adc.DMAoffset), 
                0x1,
                0x1); // enable DMA 

}
void ADC::initSingle() {
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

uint16_t ADC::getADC_Single() {
    // return (ADC::getRawADC() - ADC::calib) & 0xFFF; // Remove calibration 
    uint16_t rawADC = ADC::getRawADC_Single();
    return rawADC < ADC::calib ? 0 : rawADC - ADC::calib;

}

uint16_t ADC::getRawADC_Single() {
    return *(io_rw_32*)(ADC_BASE + ADC_RESULT_OFFSET) & 0xFFF;
}

void ADC::calibrateADC_Single(uint16_t numbValues) {
    uint32_t totalSum = 0;
    for(int iter = 0; iter < numbValues; iter++) {
        totalSum += ADC::getRawADC_Single();
        delay(1);
    }
    ADC::calib = totalSum / numbValues;
}

float ADC::getVolt_float_Single(bool isCalib) {
    uint16_t ADCval = isCalib ? ADC::getADC_Single() : ADC::getRawADC_Single();
    return (float)(3.3 * ADCval) / 4096;
}

 void ADC::initMulti() {
    ADC::DMAoffset = DMA_READ_TO_READ_OFFSET * ADC::DMAnumber; 
    ADC::firstChannel = numbADCchannels + 10; // Set firstChannel to an arbitrary number that will never be reached (less than numbADCchannels)
    ADC::numbChannels = 0; // record the amount of active channels
    // For the input signal, check active channels and assign GPIO pin as outputs
    for(int iter = 0; iter < numbADCchannels; iter++) {
        bool isActive = (ADC::activeChannels >> iter) & 0x1;
        if(isActive) {
            ADC::firstChannel = ADC::firstChannel == numbADCchannels + 10 ? iter : ADC::firstChannel; // Check if first channel and set value to the channel that is first
            uint8_t gpioPin = ADC::GPIO_ADC_OFFSET + iter;
            // Enable gpio
            uint8_t ioOffset = gpioPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
            writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 1, 16, 0); // Enable as output and do not invert signal
            uint8_t padOffset = (gpioPin * 0x04) + 0x04;
            // Set IE of GPIO Pad low
            writeReg(PADS_BANK0_BASE | (padOffset), 1, 6, 0);
            // Set OO / OD of GPIO Pad High
            writeReg(PADS_BANK0_BASE | (padOffset), 1, 7, 1);
            ADC::numbChannels++; // account for current channel
        }
    }
    
    // Enable ADC DREQ
    writeReg(ADC_BASE | ADC_FCS_OFFSET, 1, 3, 0x1);
    // Set THRESH to 1
    writeReg(ADC_BASE | ADC_FCS_OFFSET, 4, 24, 0x1);
    // Set DMA transfersize to 8 and preshift fifo samples (FCS) to 8 bits. Not used; entire 12 bit word is kept
    // writeReg(ADC_BASE | ADC_FCS_OFFSET, 1, 1, 0x1);
    // Enable FIFO
    writeReg(ADC_BASE | ADC_FCS_OFFSET, 1, 0, 0x1);
    // Configure ADC for all except start many pin
    writeReg(ADC_BASE + ADC_CS_OFFSET, 5, 16, ADC::activeChannels); // Set proper channels active
    // Set the ADC freq to 1000 samples per second
    writeReg(ADC_BASE + ADC_DIV_OFFSET, 8, 0, 0x0); // Set frac for 0 -> Clock is 48 MHz
    writeReg(ADC_BASE + ADC_DIV_OFFSET, 16, 8, 0xBB7F); // Set int for 47,999

    writeReg(ADC_BASE + ADC_CS_OFFSET, 1, 0, 1); // Set ADC to enable

    // Configure DMA
    hw_write_masked((io_rw_32*)(DMA_CH0_WRITE_ADDR + ADC::DMAoffset), 
                    (uint32_t)&ADC::rawADC[ADC::firstChannel],
                    0xFFFFFFFF); // set write address as rawADC array
        
    hw_write_masked((io_rw_32*)(DMA_CH0_READ_ADDR + ADC::DMAoffset), 
                    ADC_BASE | ADC_FIFO_OFFSET,
                    0xFFFFFFFF); // set read address as fifo

    writeReg((DMA_CH0_CTRL_TRIG + ADC::DMAoffset), 6, 15, DREQ_ADC); // set dreq on pwm wrap

    hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + ADC::DMAoffset),                     
                    0x24, //0x20 is quarter word
                    0x3f); // increment only the write, set quarter word, not high priority
    
    // Enable corresponding DMA 
    writeReg(DMA_INTE0, 1, ADC::DMAnumber, 0x1);
    // Enable interrupt for DMA
    irq_set_exclusive_handler(DMA_IRQ_0, dmaHandler);
    irq_set_enabled(DMA_IRQ_0, true);
    // // Set fifo length -> global saved value (transcount)
    hw_write_masked((io_rw_32*)(DMA_CH0_TRANS_COUNT + ADC::DMAoffset), 
                    ADC::numbChannels,
                    0xFFFFFFFF);
    // Enable DMA in CTRL
    hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + ADC::DMAoffset), 
                    0x1,
                    0x1); // enable DMA 
    
    writeReg(ADC_BASE + ADC_CS_OFFSET, 3, 12, ADC::firstChannel); // Set starting channel to the first channel
    // Start many pin of ADC
    writeReg(ADC_BASE + ADC_CS_OFFSET, 1, 3, 1); // Set ADC to start many


 }


void ADC::calibrateMulti(uint16_t numbValues) {
    uint32_t totalSum[numbADCchannels];
    // Clear all values
    for(int iter = 0; iter < numbADCchannels; iter++) totalSum[iter] = 0;
    for(int iter = 0; iter < numbValues; iter++) {
        for(int iter = 0; iter < numbADCchannels; iter++) totalSum[iter] += ADC::getrawADCMulti(iter);
        delay(numbADCchannels);
    }
    for(int iter = 0; iter < numbADCchannels; iter++) ADC::calibMulti[iter] = totalSum[iter] / numbValues;
}

uint16_t ADC::getADCMulti(uint8_t channel) { 
    // uint8_t ovfChannel = channel < numbADCchannels ? channel : numbADCchannels - 1; // prevent index out of bounds error
    uint16_t rawADC = ADC::getrawADCMulti(channel);
    return rawADC < ADC::calibMulti[channel] ? 0 : rawADC - ADC::calibMulti[channel];
}

uint16_t ADC::getrawADCMulti(uint8_t channel) { 
    // channel = channel < numbADCchannels ? channel : numbADCchannels - 1; // prevent index out of bounds error
    return ADC::rawADC[channel] & 0xFFF;
}

float ADC::getVoltMulti(bool isCalib, uint8_t channel) {
    uint16_t ADCval = isCalib ? ADC::getADCMulti(channel) : ADC::getrawADCMulti(channel);
    return (float)(3.3 * ADCval) / 4096;
}
