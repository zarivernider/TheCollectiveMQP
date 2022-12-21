#include "I2C_Peripheral.h"
// #include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
uint32_t finish = 0;
#define I2C0_SDA 6
#define I2C0_SCL 7

void I2C_P::init(uint8_t address) {
    I2C_P::I2C_Addr = address; // Store its own address
    _i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C0_SDA, I2C0_SCL, GPIO_FUNC_I2C));

    i2c_set_slave_mode(i2c0, true, I2C_P::I2C_Addr & 0x7F); // set mode and keep it 7 bits

}

bool I2C_P::isRead() {
    return i2c_get_read_available(i2c0);
}

bool I2C_P::isWrite() {
    return i2c_get_write_available(i2c0);
}

void I2C_P::write(int value) {
    uint8_t upper = 0x34;
    uint8_t lower = 0x23;
    i2c_write_raw_blocking(i2c0, &upper, 1);
    i2c_write_raw_blocking(i2c0, &lower, 1);
}

void I2C_P::read() {
    uint8_t destination = 0;
    i2c_read_raw_blocking(i2c0, &destination, 1);
    Serial.print("Read: ");
    Serial.println(destination);
}