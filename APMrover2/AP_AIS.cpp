#include "AP_AIS.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <climits>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;
AP_AIS *AP_AIS::singleton_;

AP_AIS::AP_AIS()
{
    if (singleton_ != nullptr) {
        AP_HAL::panic("AP_AIS must be singleton");
    }
    singleton_ = this;
}

void AP_AIS::init(const AP_SerialManager &serial_manager)
{
    uart_ = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AIS, 0);
    if (uart_ != nullptr) {
        uart_->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_AIS, 0));
    }

    if (uart_ == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AIS initialized failed");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "AIS initialized");
    }
}

void AP_AIS::update(void)
{
    if (uart_ == nullptr) {
        return;
    }
    int16_t nbytes = uart_->available();

    //gcs().send_text(MAV_SEVERITY_INFO, "ais have %d bytes", (int)nbytes);

    while (nbytes-- > 0) {
        char c = uart_->read();
        if (decode(c)) {

        }
    }
}

bool AP_AIS::decode(char c)
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

        // if (valid_sentence) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "checksum %c%c", _term[0], _term[1]);
        // }

        _term_number++;
        _term_offset = 0;
        _term_is_checksum = (c == '*');
        return valid_sentence;
    }

    case '!': // sentence begin
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

bool AP_AIS::decode_latest_term()
{
    if (_term_is_checksum) {
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        return ((checksum == _checksum) && _sentence_valid);
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        // the first two letters of the NMEA term are the talker ID.
        // we accept any two characters here.
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
             // unknown ID (we are actually expecting II)
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "VDO") == 0) {
            // we found the sentence type for wind
            _sentence_valid = true;
        }
        return false;
    }

    // if this is not the sentence we want then wait for another
    if (!_sentence_valid) {
        return false;
    }

    if (_term_number == 5) {
        gcs().send_text(MAV_SEVERITY_INFO, "AIS data : %s", _term);
    }

    return false;
}

int16_t AP_AIS::char_to_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}


namespace AP {

AP_AIS &ais()
{
    return *AP_AIS::get_singleton();
}

};
