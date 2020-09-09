#include "AP_NMEA_Driver.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <climits>
#include <AP_Logger/AP_Logger.h>

AP_NMEA_Driver::AP_NMEA_Driver()
{
}

void AP_NMEA_Driver::update(void)
{
    if (uart_ == nullptr) {
        return;
    }
    int16_t nbytes = uart_->available();
    while (nbytes-- > 0) {
        char c = uart_->read();
        if (decode(c)) {
        }
    }
}

bool AP_NMEA_Driver::decode(char c)
{
    switch (c) {
    case ',':
        _checksum ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
    {
        _term[_term_offset] = 0;
        bool valid_sentence = decode_latest_term();
        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '$':
    case '!':
        _sentence_valid = false;
        _term_number = 0;
        _term_offset = 0;
        _checksum = 0;
        _term_is_checksum = false;
        return false;
    }

    if (_term_offset < sizeof(_term) - 1) {
        _term[_term_offset++] = c;
    }
    if (!_term_is_checksum) {
        _checksum ^= c;
    }

    return false;
}

int16_t AP_NMEA_Driver::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

