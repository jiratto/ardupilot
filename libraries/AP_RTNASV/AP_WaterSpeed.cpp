#include "AP_WaterSpeed.h"
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

#ifndef DIGIT_TO_VAL
#define DIGIT_TO_VAL(_x)    (_x - '0')
#endif

AP_WaterSpeed::AP_WaterSpeed()
    : _new_water_speed_true(0),
      _new_water_speed_relative(0)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("WaterSpeed must be singleton");
    }
#endif
    _singleton = this;

    _result.last_update_ms = 0;
    _result.last_send_ms = 0;
}

AP_WaterSpeed *AP_WaterSpeed::get_singleton()
{
    return _singleton;
}

bool AP_WaterSpeed::enabled() const
{
    return true;
}

void AP_WaterSpeed::init()
{
    if (!enabled()) {
        return;
    }

    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_WaterSpeed, 0);
    if (_uart == nullptr) {
        return;
    }
    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_WaterSpeed, 0));   
}

void AP_WaterSpeed::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            const uint32_t now = AP_HAL::millis();
            _result.last_update_ms = now;
            _result.info.water_speed_true = _new_water_speed_true * 0.01f;
            _result.info.water_speed_relative = _new_water_speed_relative * 0.01f;
        }
    }
}

void AP_WaterSpeed::send(mavlink_channel_t chan)
{
    if (!enabled()) {
        return;
    }
       
    const uint32_t now = AP_HAL::millis();
    if (_result.last_update_ms != 0) {
        if (_result.last_send_ms < _result.last_update_ms) {
            _result.last_send_ms = now;
            mavlink_msg_water_speed_send_struct(chan, &_result.info);
        }
    }
}

int32_t AP_WaterSpeed::_parse_decimal_100(const char *p)
{
    char *endptr = nullptr;
    long ret = 100 * strtol(p, &endptr, 10);
    int sign = ret < 0 ? -1 : 1;

    if (ret >= (long)INT32_MAX) {
        return INT32_MAX;
    }
    if (ret <= (long)INT32_MIN) {
        return INT32_MIN;
    }
    if (endptr == nullptr || *endptr != '.') {
        return ret;
    }

    if (isdigit(endptr[1])) {
        ret += sign * 10 * DIGIT_TO_VAL(endptr[1]);
        if (isdigit(endptr[2])) {
            ret += sign * DIGIT_TO_VAL(endptr[2]);
            if (isdigit(endptr[3])) {
                ret += sign * (DIGIT_TO_VAL(endptr[3]) >= 5);
            }
        }
    }
    return ret;
}

bool AP_WaterSpeed::decode_latest_term()
{
    if (_term_is_checksum) {
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        return ((checksum == _checksum) && _sentence_valid);
    }

    if (_term_number == 0) {
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
             sentence_type_ = NMEA_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "VHW") == 0) {
            _sentence_valid = true;
            sentence_type_ = NMEA_SENTENCE_VHW;
        } else {
            sentence_type_ = NMEA_SENTENCE_OTHER;
        }
        return false;
    }

    if (!_sentence_valid) {
        return false;
    }

    if ((sentence_type_ != NMEA_SENTENCE_OTHER) && _term[0]) {
        switch (sentence_type_ + _term_number) {
            case NMEA_SENTENCE_VHW + 1: // heading in degrees True
                break;
            case NMEA_SENTENCE_VHW + 2: // T = True
                break;
            case NMEA_SENTENCE_VHW + 3: // Heading in degrees Magnetic
                break;
            case NMEA_SENTENCE_VHW + 4: // M = Magnetic
                break;
            case NMEA_SENTENCE_VHW + 5: // speed in knots
                _new_water_speed_true = _parse_decimal_100(_term);
                break;
            case NMEA_SENTENCE_VHW + 6: // N = knots
                break;
            case NMEA_SENTENCE_VHW + 7: // speed relative to water in Kilometers/hr
                _new_water_speed_relative = _parse_decimal_100(_term);
                break;
            case NMEA_SENTENCE_VHW + 8: // K = kilometres
                break;
        }
    }

    return false;
}

bool AP_WaterSpeed::decode(char c)
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

int16_t AP_WaterSpeed::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

AP_WaterSpeed *AP_WaterSpeed::_singleton = nullptr;

namespace AP 
{
    AP_WaterSpeed *waterspeed()
    {
        return AP_WaterSpeed::get_singleton();
    }
};
