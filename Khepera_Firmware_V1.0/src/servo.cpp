#include <servo.h>

void Servo::attach() {
    uint8_t pwmScale = (pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
    uint8_t pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    uint8_t ioOffset = pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(PWM_BASE | (0x04 + pwmOffset), 8, 4, clockDivider); // CHx_DIV. Set the divider to 14. Clock becomes 125 MHz -> 12.5 MHz
    writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, clockTOP); // CHx_TOP. fPWM = 12.5 MHz / (TOP + 1) = 200 Hertz (5 mS period)
    writeReg(PWM_BASE | (0x00 + pwmOffset), 1, 0, 1); // CHx_CSR off enable on
    isattach = true; // set flag true
}

void Servo::detach() {
    uint8_t ioOffset = pwmPin * 0x08;
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset), 2, 12, 2); // Pin output -> 0 = func select, 2 = disable output 
    isattach = false; // set flag false
}

void Servo::setMinMax(uint16_t usMin, uint16_t usMax) {
    if(usMax > 5000) return; // Do not allow any value larger than 5000 uS 
    minCount = ceil( (float)usMin  / clockTickuS ); // ceil to not be less than minimum
    maxCount = floor( (float)usMax / clockTickuS); // floor to not be more than maximum
}

void Servo::setServo(uint16_t degrees) {
    if (!isattach) attach();
    uint8_t Aoffset = pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
    degrees = constrain(degrees, minAngle, maxAngle); // Constrain from 0 to 180 degrees
    unsigned int linCount = map(degrees, minAngle, maxAngle, minCount, maxCount); // Linearily map the number of ticks across set angle
    uint8_t pwmScale = (pwmPin > 15) ? pwmPin - 16 : pwmPin; // Channels wrap so pin 0 and 16 on same channel
    uint8_t pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    writeReg(PWM_BASE | (pwmOffset + 0x0c), 16, Aoffset, linCount); // Set CC the amount of clock ticks the cycle will be high
}