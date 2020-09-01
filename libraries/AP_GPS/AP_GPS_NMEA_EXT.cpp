/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//

/// @file	AP_GPS_NMEA_EXT.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <AP_Common/AP_Common.h>

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "AP_GPS_NMEA_EXT.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// optionally log all NMEA data for debug purposes
// #define NMEA_LOG_PATH "nmea.log"

#ifdef NMEA_LOG_PATH
#include <stdio.h>
#endif

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)        (_x - '0')
#define hexdigit(x) ((x)>9?'A'+((x)-10):'0'+(x))

AP_GPS_NMEA_EXT::AP_GPS_NMEA_EXT(AP_GPS &_gps, 
                                 AP_GPS::GPS_State &_state, 
                                 AP_GPS::Weather_State &_weather, 
                                 AP_HAL::UARTDriver *_port) : AP_GPS_Backend(_gps, _state, _port),
    weather(_weather)
{
}

bool AP_GPS_NMEA_EXT::read(void)
{
    int16_t numc;
    bool parsed = false;

    numc = port->available();
    while (numc--) {
        char c = port->read();
#ifdef NMEA_LOG_PATH
        static FILE *logf = nullptr;
        if (logf == nullptr) {
            logf = fopen(NMEA_LOG_PATH, "wb");
        }
        if (logf != nullptr) {
            ::fwrite(&c, 1, 1, logf);
        }
#endif
        if (_decode(c)) {
            parsed = true;
        }
    }
    return parsed;
}

bool AP_GPS_NMEA_EXT::_decode(char c)
{
    bool valid_sentence = false;

    _sentence_length++;
        
    switch (c) {
    case ',': // term terminators
        _parity ^= c;
        FALLTHROUGH;
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        _sentence_length = 1;
        return valid_sentence;
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;

    return valid_sentence;
}

int32_t AP_GPS_NMEA_EXT::_parse_decimal_100(const char *p)
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

/*
  parse a NMEA latitude/longitude degree value. The result is in degrees*1e7
 */
uint32_t AP_GPS_NMEA_EXT::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    float frac_min = 0;
    int32_t ret = 0;

    // scan for decimal point or end of field
    for (p = _term; *p && isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees
    while ((p - q) > 2 && *q) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }

    // convert minutes
    while (p > q && *q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }

    // convert fractional minutes
    if (*p == '.') {
        q = p + 1;
        float frac_scale = 0.1f;
        while (*q && isdigit(*q)) {
            frac_min += DIGIT_TO_VAL(*q) * frac_scale;
            q++;
            frac_scale *= 0.1f;
        }
    }
    ret = (deg * (int32_t)10000000UL);
    ret += (min * (int32_t)10000000UL / 60);
    ret += (int32_t) (frac_min * (1.0e7f / 60.0f));
    return ret;
}

/*
  see if we have a new set of NMEA messages
 */
bool AP_GPS_NMEA_EXT::_have_new_message()
{
    if (_new_weather_message) {
        return true;
    }

    if (_last_RMC_ms == 0 ||
        _last_GGA_ms == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _last_RMC_ms > 150 ||
        now - _last_GGA_ms > 150) {
        return false;
    }
    if (_last_VTG_ms != 0 && 
        now - _last_VTG_ms > 150) {
        return false;
    }
    // prevent these messages being used again
    if (_last_VTG_ms != 0) {
        _last_VTG_ms = 1;
    }

    if (now - _last_HDT_ms > 300) {
        // we have lost GPS yaw
        state.have_gps_yaw = false;
    }

    _last_GGA_ms = 1;
    _last_RMC_ms = 1;
    return true;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA_EXT::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        uint8_t nibble_high = 0;
        uint8_t nibble_low  = 0;
        if (!hex_to_uint8(_term[0], nibble_high) || !hex_to_uint8(_term[1], nibble_low)) {
            return false;
        }
        const uint8_t checksum = (nibble_high << 4u) | nibble_low;
        if (checksum == _parity) {
            if (_gps_data_good) {
                uint32_t now = AP_HAL::millis();
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                    _last_RMC_ms = now;
                    //time                        = _new_time;
                    //date                        = _new_date;
                    state.location.lat     = _new_latitude;
                    state.location.lng     = _new_longitude;
                    state.ground_speed     = _new_speed*0.01f;
                    state.ground_course    = wrap_360(_new_course*0.01f);
                    make_gps_time(_new_date, _new_time * 10);
                    set_uart_timestamp(_sentence_length);
                    state.last_gps_time_ms = now;
                    fill_3d_velocity();
                    break;
                case _GPS_SENTENCE_GGA:
                    _last_GGA_ms = now;
                    state.location.alt  = _new_altitude;
                    state.location.lat  = _new_latitude;
                    state.location.lng  = _new_longitude;
                    state.num_sats      = _new_satellite_count;
                    state.hdop          = _new_hdop;
                    switch(_new_quality_indicator) {
                    case 0: // Fix not available or invalid
                        state.status = AP_GPS::NO_FIX;
                        break;
                    case 1: // GPS SPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    case 2: // Differential GPS, SPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                        break;
                    case 3: // GPS PPS Mode, fix valid
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    case 4: // Real Time Kinematic. System used in RTK mode with fixed integers
                        state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                        break;
                    case 5: // Float RTK. Satellite system used in RTK mode, floating integers
                        state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                        break;
                    case 6: // Estimated (dead reckoning) Mode
                        state.status = AP_GPS::NO_FIX;
                        break;
                    default://to maintain compatibility with MAV_GPS_INPUT and others
                        state.status = AP_GPS::GPS_OK_FIX_3D;
                        break;
                    }
                    break;
                case _GPS_SENTENCE_VTG:
                    _last_VTG_ms = now;
                    state.ground_speed  = _new_speed*0.01f;
                    state.ground_course = wrap_360(_new_course*0.01f);
                    fill_3d_velocity();
                    // VTG has no fix indicator, can't change fix status
                    break;
                case _GPS_SENTENCE_HDT:
                    _last_HDT_ms = now;
                    state.gps_yaw = wrap_360(_new_gps_yaw*0.01f);
                    state.have_gps_yaw = true;
                    break;
                }
            } else {
                switch (_sentence_type) {
                case _GPS_SENTENCE_RMC:
                case _GPS_SENTENCE_GGA:
                    // Only these sentences give us information about
                    // fix status.
                    state.status = AP_GPS::NO_FIX;
                }
            }

            // Check each weather message type and assign it's
            switch (_sentence_type) {
            case _WIN_SENTENCE_MWV:
                if (_mwv_data_valid) {
                    weather.wind_ang_bow = _wind_ang_bow;
                    weather.wind_spd_rel = _wind_spd_rel;
                    weather.wind_spd_the = _wind_spd_the;
                    _mwv_data_valid = false;
                    _new_weather_message = true;
                }
                break;
            case _WIN_SENTENCE_MDA:
                // if (_have_MDA_message) {
                //     weather.pressure_bar = _pressure_bar;
                //     weather.temp_celcius = _temp_celcius;
                //     weather.humid_rel = _humid_rel;
                //     weather.wind_ang_north = _wind_ang_north;
                //     weather.wind_ang_mag = _wind_ang_mag;
                //     weather.wind_spd_knot = _wind_spd_knot;
                //     _have_MDA_message = false;
                //     _new_weather_message = true;
                // }
                break;
            case _SON_SENTENCE_DPT:
                // if (_have_DPT_message) {
                //     weather.water_depth = _water_depth;
                //     _have_DPT_message = false;
                //     _new_weather_message = true;
                // }
                break;
            case _SON_SENTENCE_MTW:
                // if (_have_MTW_message) {
                //     weather.water_temp = _water_temp;
                //     _have_MTW_message = false;
                //     _new_weather_message = true;
                // }
                break;
            case _SON_SENTENCE_VHW:
                // if (_have_VHW_message) {
                //     weather.water_speed = _water_speed;
                //     _have_VHW_message = false;
                //     _new_weather_message = true;
                // }
                break;
            case _SON_SENTENCE_VLW:
                // if(_have_VLW_message) {
                //     weather.total_miles = _total_miles;
                //     weather.miles_since_reset = _miles_since_reset;
                //     _have_VLW_message = false;
                //     _new_weather_message = true;
                // }
                break;
            }
            // see if we got a good message
            return _have_new_message();
        }
        // we got a bad message, ignore it
        return false;
    }

    // the first term determines the sentence type
    if (_term_number == 0) {
        /*
          The first two letters of the NMEA term are the talker
          ID. The most common is 'GP' but there are a bunch of others
          that are valid. We accept any two characters here.
         */
        if (_term[0] < 'A' || _term[0] > 'Z' ||
            _term[1] < 'A' || _term[1] > 'Z') {
            _sentence_type = _GPS_SENTENCE_OTHER;
            return false;
        }
        const char *term_type = &_term[2];
        if (strcmp(term_type, "RMC") == 0) {
            _sentence_type = _GPS_SENTENCE_RMC;
        } else if (strcmp(term_type, "GGA") == 0) {
            _sentence_type = _GPS_SENTENCE_GGA;
        } else if (strcmp(term_type, "HDT") == 0) {
            _sentence_type = _GPS_SENTENCE_HDT;
            // HDT doesn't have a data qualifier
            _gps_data_good = true;
        } else if (strcmp(term_type, "VTG") == 0) {
            _sentence_type = _GPS_SENTENCE_VTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
        } else if (strcmp(term_type, "MDA") == 0) {
            _sentence_type = _WIN_SENTENCE_MDA;
        } else if (strcmp(term_type, "MWV") == 0) {
            _sentence_type = _WIN_SENTENCE_MWV;
        } else if (strcmp(term_type, "DPT") == 0) {
            _sentence_type = _SON_SENTENCE_DPT;
        } else if (strcmp(term_type, "MTW") == 0) {
            _sentence_type = _SON_SENTENCE_MTW;
        } else if (strcmp(term_type, "VHW") == 0) {
            _sentence_type = _SON_SENTENCE_VHW;
        } else if (strcmp(term_type, "VLW") == 0) {
            _sentence_type = _SON_SENTENCE_VLW;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }

    // 32 = RMC, 64 = GGA, 96 = VTG, 128 = HDT
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
        // operational status
        //
        case _GPS_SENTENCE_RMC + 2: // validity (RMC)
            _gps_data_good = _term[0] == 'A';
            break;
        case _GPS_SENTENCE_GGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
            _new_quality_indicator = _term[0] - '0';
            break;
        case _GPS_SENTENCE_VTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
            break;
        case _GPS_SENTENCE_GGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GGA + 8: // HDOP (GGA)
            _new_hdop = (uint16_t)_parse_decimal_100(_term);
            break;

        // time and date
        //
        case _GPS_SENTENCE_RMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GGA + 1: // Time (GGA)
            _new_time = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

        // location
        //
        case _GPS_SENTENCE_RMC + 3: // Latitude
        case _GPS_SENTENCE_GGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 4: // N/S
        case _GPS_SENTENCE_GGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_RMC + 5: // Longitude
        case _GPS_SENTENCE_GGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_RMC + 6: // E/W
        case _GPS_SENTENCE_GGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal_100(_term);
            break;

        // course and speed
        //
        case _GPS_SENTENCE_RMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_VTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal_100(_term) * 514) / 1000;       // knots-> m/sec, approximiates * 0.514
            break;
        case _GPS_SENTENCE_HDT + 1: // Course (HDT)
            _new_gps_yaw = _parse_decimal_100(_term);
            break;
        case _GPS_SENTENCE_RMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_VTG + 1: // Course (VTG)
            _new_course = _parse_decimal_100(_term);
            break;

        // wind Message
        // 
        case _WIN_SENTENCE_MWV + 1: // Wind angle relative to vessel bow
            _wind_ang_bow = _parse_decimal_100(_term);
            break;
        case _WIN_SENTENCE_MWV + 2: // Wind speed reference
            if (_term[0] == 'T')
                _the_selector = true;
            else
                _the_selector = false;
            break;
        case _WIN_SENTENCE_MWV + 3: // Wind speed
            if (_the_selector) {
                _wind_spd_the = _parse_decimal_100(_term);
            } else {
                _wind_spd_rel = _parse_decimal_100(_term);
            }
            break;
        case _WIN_SENTENCE_MWV + 5: // Validitiy check
            if (_term[0] == 'A')
                _mwv_data_valid = true;
            else
                _mwv_data_valid = false;
            break;

        }
    }

    return false;
}

/*
  detect a NMEA GPS. Adds one byte, and returns true if the stream
  matches a NMEA string
 */
bool
AP_GPS_NMEA_EXT::_detect(struct NMEA_detect_state &state, uint8_t data)
{
	switch (state.step) {
	case 0:
		state.ck = 0;
		if ('$' == data) {
			state.step++;
		}
		break;
	case 1:
		if ('*' == data) {
			state.step++;
		} else {
			state.ck ^= data;
		}
		break;
	case 2:
		if (hexdigit(state.ck>>4) == data) {
			state.step++;
		} else {
			state.step = 0;
		}
		break;
	case 3:
		if (hexdigit(state.ck&0xF) == data) {
            state.step = 0;
			return true;
		}
		state.step = 0;
		break;
    }
    return false;
}
