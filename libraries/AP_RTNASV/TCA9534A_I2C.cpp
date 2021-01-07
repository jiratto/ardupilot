#include "TCA9534A_I2C.h"
#include <utility>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define TCA9534A_I2C_ADDR   0x20
#define TCA9534A_I2C_BUS    1

#define INPUT_PORT_BYTE     0x00
#define OUTPUT_PORT_BYTE    0x01
#define POL_INVERT_BYTE     0x02
#define CONFIG_BYTE         0X03
#define ALL_HIGH            0xFF
#define ALL_LOW             0x00
#define DEFAULT_CONFIG      0xFF

bool TCA9534A_I2C::init(uint8_t addr, uint8_t config)
{
    _dev = std::move(hal.i2c_mgr->get_device(TCA9534A_I2C_BUS, addr));

    // take i2c bus sempahore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // Configure the role of each IO
    bool ret = _dev->write_register(CONFIG_BYTE, config);

    // Configure polarity inversion
    ret &= _dev->write_register(POL_INVERT_BYTE, ALL_LOW);

    // give back i2c semaphore
    _dev->get_semaphore()->give();

    return ret;
}

// Read status from the Register
// 0 = Input Port Reg, 1 = Output Port Reg, 2 = Pol Inv Reg, 3 = Config Reg
bool TCA9534A_I2C::read(int command, uint8_t &data)
{
    bool ret;

    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    switch(command){
        case 0:
            ret = read_word(INPUT_PORT_BYTE, data);
            break;
        case 1:
            ret = read_word(OUTPUT_PORT_BYTE, data);
            break;
        case 2:
            ret = read_word(POL_INVERT_BYTE, data);
            break;
        case 3: 
            ret = read_word(CONFIG_BYTE, data);
            break;
        default:
            ret = false;
            break;
    }

    _dev->get_semaphore()->give();

    return ret;
}

// Write to register
// 0 = Input Port Reg, 1 = Output Port Reg, 2 = Pol Inv Reg, 3 = Config Reg
bool TCA9534A_I2C::write(int command, uint8_t value)
{
    bool ret;
    
    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    switch(command){
        case 1:
        {
            ret = _dev->write_register(OUTPUT_PORT_BYTE,value);
            break;
        }
        case 2:
        {
            ret = _dev->write_register(POL_INVERT_BYTE,value);
            break;
        }
        case 3:
        {
            ret = _dev->write_register(CONFIG_BYTE,value);
            break;
        }
        default:
        {
            ret = false;
            break;
        }
    }

    _dev->get_semaphore()->give();

    return ret;
}

// read word from register
// returns true if read was successful, false if failed
bool TCA9534A_I2C::read_word(uint8_t reg, uint8_t& data)
{
    const uint8_t read_size = 1;
    uint8_t buff[read_size];    // buffer to hold results

    // read the appropriate register from the device
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return false;
    }

    // convert buffer to word
    data = (uint8_t)buff[0];

    // return success
    return true;
}

void TCA9534A_I2C::update()
{
    // Read the input port register
    uint8_t data;
    if (read(0, data)) {
        //gcs
        // if((data & 0x70) == 0x00)
        //     write(1, 0x00); // ALL ON
        // else if ((data & 0x70) == 0x10)
        //     write(1, 0x01); // 1 OFF 2 ON 3 ON
        // else if ((data & 0x70) == 0x20)
        //     write(1, 0x02); // 1 ON 2 OFF 3 ON
        // else if ((data & 0x70) == 0x30)
        //     write(1, 0x03); // 1 OFF 2 oFF 3 ON
        // else if ((data & 0x70) == 0x40)
        //     write(1, 0x04); // 1 ON 2 ON 3 OFF
        // else if ((data & 0x70) == 0x50)
        //     write(1, 0x05); // 1 OFF 2 ON 3 OFF
        // else if ((data & 0x70) == 0x60)
        //     write(1, 0x06); // 1 ON 2 OFF 3 OFF
        // else if ((data & 0x70) == 0x70)
        //     write(1, 0x07); //ALL OFF
    }
}

void TCA9534A_I2C::set_output_relay(int input)
{
    if (input == 0)
        write(1, 0x06);
    else if (input == 1)
        write(1, 0x04);
    else if (input == 2)
        write(1, 0x00);
    else if (input == 3)
        write(1, 0x07);
}
