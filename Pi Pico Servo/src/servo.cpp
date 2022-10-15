#include <servo.h>

void Servo::attach() {
    uint8_t pwmOffset = (int)(pwmPin / 2) << 4; // Off set is set by most sig byte where 0 = Ch0
    uint8_t ioOffset = pwmPin * 0x08;
    writeReg(IO_BANK0_BASE | 0x04 | ioOffset, 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | 0x04 | ioOffset, 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(PWM_BASE | 0x04 | pwmOffset, 8, 4, 14); // CH0_DIV. Set the divider to 14. Clock becomes 133 MHz -> 9.5 MHz
    writeReg(PWM_BASE | 0x10 | pwmOffset, 16, 0, 47499); // CH0_TOP. fPWM = 9.5 MHz / (TOP + 1) = 200 Hertz (5 mS period)
    writeReg(PWM_BASE | 0x00 | pwmOffset, 1, 0, 1); // CH0_CSR off enable on
    isattach = true; // set flag true
}

void Servo::detach() {
    uint8_t ioOffset = pwmPin * 0x08;
    writeReg(IO_BANK0_BASE | 0x04 | ioOffset, 2, 12, 2); // Pin output -> 0 = func select, 2 = disable output 
    isattach = false; // set flag false
}

void Servo::setMinMax(uint16_t usMin, uint16_t usMax) {
    if(usMax > 5000) return; // Do not allow any value larger than 5000 uS 
    // Clock is 105 nS per tick. Convert input from uS to nS and get clock tick. 
    minCount = ceil((usMin*1000) / 105); // ceil to not be less than minimum
    maxCount = floor((usMax*1000) / 105); // floor to not be more than maximum
}

void Servo::setServo(uint16_t degrees) {
    if (!isattach) attach();
    uint8_t Aoffset = pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
    unsigned int linCount = map(degrees, minAngle, maxAngle, minCount, maxCount); // Linearily map the number of ticks across set angle
    writeReg(PWM_BASE | 0x0c, 16, Aoffset, linCount); // Set CC the amount of clock ticks the cycle will be high
}