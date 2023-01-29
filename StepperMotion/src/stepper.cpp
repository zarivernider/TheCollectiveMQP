#include "stepper.h"

// void Stepper::attach() {
//     uint8_t pwmScale = (pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
//     uint8_t pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
//     uint8_t ioOffset = pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
//     writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 5, 0, 4); // func select -> 4 = PWM
//     writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
//     writeReg(PWM_BASE | (0x04 + pwmOffset), 8, 4, 14); // CHx_DIV. Set the divider to 14. Clock becomes 133 MHz -> 9.5 MHz
//     writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, 47499); // CHx_TOP. fPWM = 9.5 MHz / (TOP + 1) = 200 Hertz (5 mS period)
//     writeReg(PWM_BASE | (0x00 + pwmOffset), 1, 0, 1); // CHx_CSR off enable on
// }

// void stepper::detach() {
//     uint8_t ioOffset = pwmPin * 0x08;
//     writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 2); // Pin output -> 0 = func select, 2 = disable output 
//     isattach = false; // set flag false
// }

// void stepper::setMinMax(uint16_t usMin, uint16_t usMax) {
//     if(usMax > 5000) return; // Do not allow any value larger than 5000 uS 
//     // Clock is 105 nS per tick. Convert input from uS to nS and get clock tick. 
//     minCount = ceil((usMin*1000) / 105); // ceil to not be less than minimum
//     maxCount = floor((usMax*1000) / 105); // floor to not be more than maximum
// }

// void stepper::setServo(uint16_t degrees) {
//     if (!isattach) attach();
//     uint8_t Aoffset = pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
//     unsigned int linCount = map(degrees, minAngle, maxAngle, minCount, maxCount); // Linearily map the number of ticks across set angle
//     uint8_t pwmScale = (pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
//     uint8_t pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
//     writeReg(PWM_BASE | (pwmOffset + 0x0c), 16, Aoffset, linCount); // Set CC the amount of clock ticks the cycle will be high
// }


void Stepper::init() {
    Stepper::enGPIO.init();
    Stepper::dirGPIO.init();

    //enable pwm 
    uint8_t pwmScale = (Stepper::pwmPin > 15) ? Stepper::pwmPin - 16 : Stepper::pwmPin; // Channels wrap so pin 0 and 16 on same channel
    Stepper::pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    Stepper::pwmioOffset = Stepper::pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 8, 0); // Pin output -> 0 = func select, 2 = drive output low
    writeReg(PWM_BASE | (0x04 + Stepper::pwmOffset ), 8, 4, 125); // CHx_DIV. Set the divider to 125. Clock becomes 133 MHz -> 1 MHz
    Stepper::systemFreq = 1000000;
    writeReg(PWM_BASE | (0x04 + Stepper::pwmOffset ), 4, 0, 0); // CHx_DIV. Remove fractional bits
    writeReg(PWM_BASE | (0x00 + Stepper::pwmOffset ), 1, 0, 1); // CHx_CSR off enable on
}

void Stepper::setSpeed(int16_t RPMspeed) { // TEST: 375 RPM -> 5000 Hz @ 4 usteps
    // If speed is very low then just shunt the output
    if(RPMspeed < 20) brakeStop();
    else{
        // Find output freq
        float outFreq = (float)(RPMspeed * microSteps * stepsperRev) / 60; 
        // Find TOP value
        uint16_t TOPvalue = (systemFreq / outFreq) - 1;
        // Set TOP value
        writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, TOPvalue); // CHx_TOP. fPWM = 9.5 MHz / (TOP + 1) 
        // Set CC
        uint8_t Aoffset = Stepper::pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
        writeReg(PWM_BASE | (Stepper::pwmOffset + 0x0c), 16, Aoffset, TOPvalue / 2); // Set CC the amount of clock ticks the cycle will be high
    }
}

void Stepper::enable(bool isEnabled) {
    enGPIO.assertIO(!isEnabled);
}

void Stepper::setDirection(bool isFwd) {
    dirGPIO.assertIO(isFwd);
}
    
void Stepper::brakeStop() {
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 8, 2); // Pin output -> 0 = func select, 2 = drive output low
}
