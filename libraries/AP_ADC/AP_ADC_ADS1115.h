#pragma once

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define ADS1115_DEFAULT_ADDRESS 0x48

struct adc_report_s
{
    uint8_t id;
    float data;
};

class AP_ADC_ADS1115
{
public:
    AP_ADC_ADS1115(uint8_t address = ADS1115_DEFAULT_ADDRESS);
    ~AP_ADC_ADS1115();

    bool init();
    size_t read(adc_report_s *report, size_t length) const;

    uint8_t get_channels_number() const
    {
        return _channels_number;
    }

private:
    static const uint8_t _channels_number;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint8_t             _address;
    uint16_t            _gain;
    int                 _channel_to_read;
    adc_report_s        *_samples;

    void _update();
    bool _start_conversion(uint8_t channel);

    float _convert_register_data_to_mv(int16_t word) const;
};
