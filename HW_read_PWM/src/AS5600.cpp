#include <AS5600.h>
uint8_t staticDMAnumber;
uint32_t staticDMAoffset;
uint32_t staticPWM_CTR_Addr;
volatile uint16_t registerValue = 0;
static inline void dma2Handler() {
    // // // Get interrupt status
    // uint32_t status = *(io_rw_32*)DMA_INTS0;
    // Clear interrupt
    hw_write_masked((io_rw_32*)(DMA_INTS0),
                    0x1 << staticDMAnumber, 
                    0xffff); // clear interrupt

    // Restart DMA
    hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + staticDMAoffset), 
                0x1,
                0x1); // enable DMA 
}
static inline void GPIOhandler() {
    writeReg(IO_BANK_INTR1, 1, 30, 1);
    registerValue = *(io_rw_32*)(staticPWM_CTR_Addr) & 0xFFFF; // Save value
    hw_write_masked((io_rw_32*)staticPWM_CTR_Addr,
                    0x0,
                    0xFFFF); // Clear top

}
void AS5600::init() {
    // Calculate register offsets for the pin
    AS5600::pwmScale = (AS5600::pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
    AS5600::pwmSlice = (int)(pwmScale/2);
    AS5600::pwmOffset = AS5600::pwmSlice * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    AS5600::ioOffset = AS5600::pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    staticPWM_CTR_Addr = (PWM_BASE | (0x8 + AS5600::pwmOffset));
    // Set GPIO as PWM 
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output

    // Set CSR Div Mode to 1. Enables input and free running
    writeReg(PWM_BASE | (0x00 + pwmOffset), 2, 4, 1); // CHx_CSR 
    // Set DIV as 16. fsys -> 7,812,500 Hz 
    writeReg(PWM_BASE | (0x04 + pwmOffset), 8, 4, 16); // CHx_DIV. 
    // Set top -> 16,982. fpwm -> ~460 Hz
    writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, 16982); // CHx_TOP
    // Enable PWM
    writeReg(PWM_BASE | (0x00 + pwmOffset), 1, 0, 1); // CHx_CSR off enable on

    // DMA is deprecated
        // // Enable DMA for PWM Wrap (1 transfer)
        // // Calculate register offsets for the channel
        // AS5600::DMAoffset = DMA_READ_TO_READ_OFFSET * AS5600::DMAnumber; 
        // staticDMAoffset = AS5600::DMAoffset;
        // staticDMAnumber = AS5600::DMAnumber;
        // hw_write_masked((io_rw_32*)(DMA_CH0_WRITE_ADDR + AS5600::DMAoffset), 
        //                 (uint32_t)&rawAngle,
        //                 0xFFFFFFFF); // set write address as internal angle value
            
        // hw_write_masked((io_rw_32*)(DMA_CH0_READ_ADDR + AS5600::DMAoffset), 
        //                 (PWM_BASE | (0x8 + pwmOffset)),
        //                 0xFFFFFFFF); // set read address as PWM counter register

        // writeReg((DMA_CH0_CTRL_TRIG + AS5600::DMAoffset), 6, 15, DREQ_PWM_WRAP_BASE  + AS5600::pwmSlice); // set dreq on pwm wrap

        // hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + AS5600::DMAoffset),                     
        //                 0x4,
        //                 0x3f); // set half word
        
        // writeReg(DMA_INTE1, 1, AS5600::DMAnumber, 0x1);
        // // Enable interrupt for DMA
        // irq_set_exclusive_handler(DMA_IRQ_1, dma2Handler);
        // irq_set_enabled(DMA_IRQ_1, true);
        // // Set transfer size
        // // Set fifo length -> global saved value (transcount)
        // hw_write_masked((io_rw_32*)(DMA_CH0_TRANS_COUNT + AS5600::DMAoffset), 
        //                 1,
        //                 0xFFFFFFFF);
        // // Enable DMA in CTRL
        // hw_write_masked((io_rw_32*)(DMA_CH0_CTRL_TRIG + AS5600::DMAoffset), 
        //                 0x1,
        //                 0x1); // enable DMA 
        // Enable interrupt
    writeReg(IO_BANK_PROC0_INTE1, 1, 30, 1);
    irq_set_exclusive_handler(IO_IRQ_BANK0, GPIOhandler);
    irq_set_enabled(IO_IRQ_BANK0, true);

}

uint16_t AS5600::getCtr() {
    // return *(io_rw_32*)(PWM_BASE | (0x8 + pwmOffset)) & 0xFF;

    return registerValue;

}