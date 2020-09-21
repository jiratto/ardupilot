#pragma once

#include <AP_HAL/AP_HAL.h>
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

class AP_NMEA_Driver
{
public:
    
    AP_NMEA_Driver();
    
    virtual ~AP_NMEA_Driver(void) {}

protected:

    virtual bool decode_latest_term(void) = 0;

    bool decode(char c);

    int16_t char_to_hex(char a);   

protected:
    AP_HAL::UARTDriver *_uart;
    char _term[50];             // buffer for the current term within the current sentence
    uint8_t _term_offset;       // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;       // term index within the current sentence
    uint8_t _checksum;          // checksum accumulator
    bool _term_is_checksum;     // current term is the checksum
    bool _sentence_valid;       // is current sentence valid so far
    char _sentence_begin;       // sentence begin char
};
