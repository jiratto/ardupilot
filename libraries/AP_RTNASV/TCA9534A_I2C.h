#pragma once

#include <AP_HAL/I2CDevice.h>

class TCA9534A_I2C
{
public:

    // initialise the driver
    bool init(uint8_t addr, uint8_t config);

    void update(void);

    bool read(int command, uint8_t &data);

    bool write(int command, uint8_t value);

    bool read_word(uint8_t reg, uint8_t &data);

    void set_output_relay(int input);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
