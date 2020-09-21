#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_WaterSpeed
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

    // Device update loop
    void update();

    // Initialize the device and prepare it for use
    void init();

    // Send mavlink message to GCS
    void send(mavlink_channel_t chan);
    
private:
    AP_HAL::UARTDriver *_uart;
    char _term[50];             // buffer for the current term within the current sentence
    uint8_t _term_offset;       // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;       // term index within the current sentence
    uint8_t _checksum;          // checksum accumulator
    bool _term_is_checksum;     // current term is the checksum
    bool _sentence_valid;       // is current sentence valid so far
    char _sentence_begin;       // sentence begin char

    enum sentence_types {
        NMEA_SENTENCE_VHW = 32,
        NMEA_SENTENCE_OTHER = 0
    };
    uint16_t sentence_type_;

    // Try and decode NMEA message
    bool decode(char c);

    // Decode each term
    bool decode_latest_term();

    // Convert from char to hex value for checksum
    int16_t char_to_hex(char a);
    
    // Parses the as a decimal number with up to 3 decimal digits
    static int32_t _parse_decimal_100(const char *p);

    struct waterspeed_vehicle_t {
        mavlink_water_speed_t info;
        uint32_t last_update_ms;    // last time this was refreshed, allows timeouts
        uint32_t last_send_ms;      // last time this message was sent via mavlink, stops us spamming the link
    };
    waterspeed_vehicle_t _result;

    int32_t _new_water_speed_true;
    int32_t _new_water_speed_relative;

    static AP_WaterSpeed *_singleton;
};

namespace AP {
    AP_WaterSpeed *waterspeed();
};
