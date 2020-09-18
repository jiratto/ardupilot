#include "AP_WaterSpeed.h"

AP_WaterSpeed::AP_WaterSpeed()
    : AP_NMEA_Driver(),
      water_speed_long_(0.0f),
      water_speed_tran_(0.0f),
      water_speed_valid_(false),
      ground_speed_long_(0.0f),
      ground_speed_tran_(0.0f),
      ground_speed_valid_(false)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("WaterSpeed must be singleton");
    }
#endif
    _singleton = this;
}

AP_WaterSpeed *AP_WaterSpeed::get_singleton()
{
    return _singleton;
}

bool AP_WaterSpeed::enabled() const
{
    return false;
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

void AP_WaterSpeed::init(const AP_SerialManager &serial_manager)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_WaterSpeed, 0);
    if (_uart != nullptr) {
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_WaterSpeed, 0));
    }

    if (_uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "WaterSpeed initialized failed");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "WaterSpeed initialized");
    }
}

void AP_WaterSpeed::update()
{
    AP_NMEA_Driver::update();
}

void AP_WaterSpeed::send(mavlink_channel_t chan)
{
    if (!enabled()) {
        return;
    }
    // TODO
}

bool AP_WaterSpeed::decode_latest_term()
{
    if (_term_is_checksum) {
        uint8_t checksum = 16 * char_to_hex(_term[0]) + char_to_hex(_term[1]);
        if ((checksum == _checksum) && _sentence_valid) {
            if (water_speed_valid_)
                gcs().send_text(MAV_SEVERITY_INFO, "water speed : %f,%f", water_speed_long_, water_speed_tran_);
            if (ground_speed_valid_)
                gcs().send_text(MAV_SEVERITY_INFO, "ground speed : %f,%f", ground_speed_long_, ground_speed_tran_);
        }
        return ((checksum == _checksum) && _sentence_valid);
    }

    if (_term_number == 0) {
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
             sentence_type_ = NMEA_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "VTG") == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "VTG");
            _sentence_valid = true;
            sentence_type_ = NMEA_SENTENCE_VBW;
        } else {
            sentence_type_ = NMEA_SENTENCE_OTHER;
        }
        return false;
    }

    if (!_sentence_valid) {
        return false;
    }

    // if ((sentence_type_ != NMEA_SENTENCE_OTHER) && _term[0]) {
    //     switch (sentence_type_ + _term_number) {
    //         case NMEA_SENTENCE_VBW + 1:
    //             water_speed_long_ = strtof(_term, NULL);
    //             break;
    //         case NMEA_SENTENCE_VBW + 2:
    //             water_speed_tran_ = strtof(_term, NULL);
    //             break;
    //         case NMEA_SENTENCE_VBW + 3:
    //             if (_term[0] >= 'A' && _term[0] <= 'Z') {
    //                 water_speed_valid_ = true;
    //             }
    //             break;
    //         case NMEA_SENTENCE_VBW + 4: 
    //             ground_speed_long_ = strtof(_term, NULL);
    //             break;
    //         case NMEA_SENTENCE_VBW + 5: 
    //             ground_speed_tran_ = strtof(_term, NULL);
    //             break;
    //         case NMEA_SENTENCE_VBW + 6: 
    //             if (_term[0] >= 'A' && _term[0] <= 'Z') {
    //                 ground_speed_valid_ = true;
    //             }
    //             break;
    //     }
    // }

    return false;
}

AP_WaterSpeed *AP_WaterSpeed::_singleton = nullptr;

namespace AP {
    AP_WaterSpeed &waterspeed()
    {
        return *AP_WaterSpeed::get_singleton();
    }
};
