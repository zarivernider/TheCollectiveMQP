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
    // Enable internal sense resistors
    Serial1.begin(9600); // Initialize UART at 9600
    // Enable the correct UART pint
    uint16_t ioOffset = Stepper::UARTpin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset ), 5, 0, 2); // func select -> 2 = UART
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset ), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + ioOffset ), 2, 8, 0); // Pin output -> 0 = func select, 2 = disable output
    uint16_t padOffset = (Stepper::UARTpin * 0x04) + 0x04;
    writeReg(PADS_BANK0_BASE + padOffset, 1, 3, 1); // Enable pull-up
    writeReg(PADS_BANK0_BASE + padOffset, 1, 2, 0); // Disable pull-down

    
    // Stepper::writeAddress(regConfaddress, 0x103); // Enable internal Rsense resistors
    
    // Initialize IO
    Stepper::enGPIO.init();
    Stepper::dirGPIO.init();
    Stepper::MS1.init();
    Stepper::MS2.init();

    // Set initial condition
    Stepper::enGPIO.assertIO(true); // Stop stepper on start-up
    Stepper::setMicroSteps(startMicroSteps);
    Stepper::dirGPIO.assertIO(true);

    
    //enable pwm 
    uint8_t pwmScale = (Stepper::pwmPin > 15) ? Stepper::pwmPin - 16 : Stepper::pwmPin; // Channels wrap so pin 0 and 16 on same channel
    Stepper::pwmOffset = (int)(pwmScale/2) * 0x14; // Offset repeats every 0x14 register addresses. Gets to CHx_CSR
    Stepper::pwmioOffset = Stepper::pwmPin * 0x08; // Offset repeats every 0x8 register addresses. Gets to GPIOx_STATUS
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 5, 0, 4); // func select -> 4 = PWM
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 12, 0); // Pin output -> 0 = func select, 2 = disable output
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 8, 0); // Pin output -> 0 = func select, 2 = drive output low
    writeReg(PWM_BASE | (0x04 + Stepper::pwmOffset ), 8, 4, 125); // CHx_DIV. Set the divider to 125. Clock becomes 125 MHz -> 1 MHz
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
        writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, TOPvalue); // CHx_TOP. fPWM = 1 MHz / (TOP + 1) 
        // Set CC
        uint8_t Aoffset = Stepper::pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
        writeReg(PWM_BASE | (Stepper::pwmOffset + 0x0c), 16, Aoffset, TOPvalue >> 1); // Set CC the amount of clock ticks the cycle will be high
    }
}
void Stepper::setFreq(uint16_t freq) { // TEST: 
    // If speed is very low then just shunt the output
    if(freq < minFreq) freq = minFreq;
    else if(freq >= maxFreq) freq = maxFreq;
    // Find TOP value
    uint16_t TOPvalue = (systemFreq / freq) - 1;
    // Set TOP value
    writeReg(PWM_BASE | (0x10 + pwmOffset), 16, 0, TOPvalue); // CHx_TOP. fPWM = 1 MHz / (TOP + 1) 
    // Set CC
    uint8_t Aoffset = Stepper::pwmPin % 2 == 1 ? 16 : 0; // if even then A side controller, if odd then B controller
    writeReg(PWM_BASE | (Stepper::pwmOffset + 0x0c), 16, Aoffset, TOPvalue >> 1); // Set CC the amount of clock ticks the cycle will be high
}

void Stepper::setDirFreq(int16_t freq) {
    if(freq > 0) {
        Stepper::setDirection(false);
        setFreq(freq);
    }
    else {
        Stepper::setDirection(true);
        setFreq(-freq);
    }
}
void Stepper::enable(bool isEnabled) {
    if(isEnabled) writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 8, 0); // Pin output -> 0 = func select, 0 = drive output funtion select
    enGPIO.assertIO(!isEnabled);
}

void Stepper::setDirection(bool isFwd) {
    dirGPIO.assertIO(isFwd);
}
    
void Stepper::brakeStop() {
    writeReg(IO_BANK0_BASE | (0x04 + Stepper::pwmioOffset), 2, 8, 2); // Pin output -> 0 = func select, 2 = drive output low
}

void Stepper::setMicroSteps(uint8_t setting) {
    if(setting == 2) {
        Stepper::MS1.assertIO(true);
        Stepper::MS2.assertIO(false);
    }
    else if(setting == 4) {
        Stepper::MS1.assertIO(false);
        Stepper::MS2.assertIO(true);
    }
    else if(setting == 8) {
        Stepper::MS1.assertIO(false);
        Stepper::MS2.assertIO(false);
    }
    else if(setting == 16) {
        Stepper::MS1.assertIO(true);
        Stepper::MS2.assertIO(true);
    }
}

void Stepper::writeAddress(uint8_t regAddres, uint32_t message) {
    uint8_t regBytes[transmissionBytes];
    regBytes[0] = 0x5; // synchronization bit
    regBytes[1] = TMCaddress; // addressing byte
    regBytes[2] = regAddres | 0x80; // register address, set to write
    regBytes[3] = (message >> 24) & 0xFF; // MSB of data
    regBytes[4] = (message >> 16) & 0xFF;
    regBytes[5] = (message >> 8) & 0xFF;
    regBytes[6] = message & 0xFF; // LSB of data
    regBytes[7] = Stepper::calcCRC(regBytes, transmissionBytes);

    for(int iter = 0; iter < transmissionBytes; iter++){
        Serial1.write(regBytes[iter]);
    }

}

uint8_t Stepper::calcCRC(uint8_t* datagram, uint8_t datagramLength) {
    int i,j;
    uint8_t crc = 0;
    for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
        uint8_t currentByte = datagram[i]; // Retrieve a byte to be sent from Array
            for (j=0; j<8; j++) {
            if ((crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
            } // for CRC bit
    } // for message byte
    return crc;
}

void Stepper::readAddress(uint8_t regAddres) {
    uint8_t regBytes[transmissionBytes - 4]; // Same size as write without message
    regBytes[0] = 0x5; // synchronization bit
    regBytes[1] = TMCaddress; // addressing byte
    regBytes[2] = regAddres; // register address
    regBytes[3] = Stepper::calcCRC(regBytes, transmissionBytes - 4);

    for(int iter = 0; iter < transmissionBytes - 4; iter++){
        Serial1.write(regBytes[iter]);
    }
}

