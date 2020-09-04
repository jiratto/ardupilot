#pragma once

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_AIS
{
public:
    AP_AIS();

    /* Do not allow copies */
    AP_AIS(const AP_AIS &other) = delete;
    AP_AIS &operator=(const AP_AIS&) = delete;

    static AP_AIS *get_singleton() {
        return singleton_;
    }

    void init(const AP_SerialManager &serial_manager);

    void update(void);

private:

    bool decode(char c);

    bool decode_latest_term();

    int16_t char_to_hex(char a);

private:
    static AP_AIS *singleton_;
    AP_HAL::UARTDriver *uart_;

    char _term[50];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _sentence_valid;      // is current sentence valid so far
};

namespace AP {
    AP_AIS &ais();
};
