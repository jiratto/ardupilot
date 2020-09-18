#pragma once

#include "AP_NMEA_Driver.h"

class AP_WaterSpeed : public AP_NMEA_Driver
{
public: 
    AP_WaterSpeed();

    /* Do not allow copies */
    AP_WaterSpeed(const AP_WaterSpeed &other) = delete;
    AP_WaterSpeed &operator=(const AP_WaterSpeed&) = delete;

    // Get the singleton
    static AP_WaterSpeed *get_singleton();

    // Return true if device is enabled
    bool enabled() const;

    void update();

    // Initialize the device and prepare it for use
    void init();
    void init(const AP_SerialManager &serial_manager) override;

    // send mavlink message to GCS
    void send(mavlink_channel_t chan);
    
private:
    static AP_WaterSpeed *_singleton;

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
