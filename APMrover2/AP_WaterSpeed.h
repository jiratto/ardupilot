#pragma once

#include "AP_NMEA_Driver.h"

class AP_WaterSpeed : public AP_NMEA_Driver
{
public: 
    AP_WaterSpeed();

    AP_WaterSpeed(const AP_WaterSpeed &other) = delete;
    AP_WaterSpeed &operator=(const AP_WaterSpeed&) = delete;

    static AP_WaterSpeed *get_singleton() {
        return singleton_;
    }

    void init(const AP_SerialManager &serial_manager) override;
    
private:
    static AP_WaterSpeed *singleton_;
    bool decode_latest_term() override;

    enum sentence_types {
        NMEA_SENTENCE_VBW = 32,
        NMEA_SENTENCE_OTHER = 0
    };
    uint16_t sentence_type_;

    float water_speed_long_; // Longitudinal water speed, "-" means astern
    float water_speed_tran_; // Transverse water speed, "-" means port
    bool water_speed_valid_;
    float ground_speed_long_; // Longitudinal ground speed, "-" means astern
    float ground_speed_tran_; // Transverse ground speed, "-" means port
    bool ground_speed_valid_;
};

namespace AP {
    AP_WaterSpeed &waterspeed();
};
