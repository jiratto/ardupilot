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
//

/// @file	AP_GPS_NMEA_EXT.h
/// @brief	NMEA protocol parser (Extension)
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

/// NMEA parser
///
class AP_GPS_NMEA_EXT : public AP_GPS_Backend
{
    friend class AP_GPS_NMEA_EXT_Test;

public:
    AP_GPS_NMEA_EXT(AP_GPS &_gps, 
                    AP_GPS::GPS_State &_state, 
                    AP_GPS::Weather_State &_weather, 
                    AP_HAL::UARTDriver *_port);
    
    using AP_GPS_Backend::AP_GPS_Backend;

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read() override;

	static bool _detect(struct NMEA_detect_state &state, uint8_t data);

    const char *name() const override { return "NMEA_EXT"; }

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,     // [13] $GPRMC,044911,A,1333.4681,N,10034.9158,E,0.0,293.4,040121,0.7,W,D*02
        _GPS_SENTENCE_GGA = 64,     // [15] $GPGGA,044910,1333.4682,N,10034.9157,E,2,12,1.0,-1.0,M,-26.4,M,,*4C
        _GPS_SENTENCE_VTG = 96,     // [10] $GPVTG,237.8,T,,,0.0,N,0.0,K,A*4E
        _GPS_SENTENCE_HDT = 128,    // [3]  **$GPHDT,123.456,T*00
        /// extension
        _GPS_SENTENCE_HDG = 138,    // [6]  $GPHDG,165.4,0.0,E,0.7,W*4D
        _GPS_SENTENCE_ZDA = 148,    // [7]  $GPZDA,044910,04,01,2021,,*44
        _GPS_SENTENCE_MWD = 158,    // [9]  $GPMWD,311.6,T,,,3.4,N,1.7,M*34
        _GPS_SENTENCE_MDA = 178,    // [21] $GPMDA,,,,,29.7,C,,,,,,,,,,,,,,*0E
        _GPS_SENTENCE_MWV = 208,    // [6]  $GPMWV,145.9,R,3.4,N,A*24
        _GPS_SENTENCE_VHW = 218,    // [9]  $GPVHW,,,165.4,M,,,,*3B
        _GPS_SENTENCE_DPT = 228,    // [3]  $SDDPT,3.6,0.0*52
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);

    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[15];                                                     ///< buffer for the current term within the current sentence
    uint16_t _sentence_type;                                    ///< the sentence type currently being processed
    uint8_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _gps_data_good;                                        ///< set when the sentence indicates data is good

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms = 0;
    uint32_t _last_GGA_ms = 0;
    uint32_t _last_VTG_ms = 0;
    uint32_t _last_HDT_ms = 0;  
    // rtnasv
    uint32_t _last_HDG_ms = 0;
    uint32_t _last_ZDA_ms = 0;
    uint32_t _last_MWD_ms = 0;
    uint32_t _last_MDA_ms = 0;
    uint32_t _last_MWV_ms = 0;
    uint32_t _last_VHW_ms = 0;   

    // weather station
    AP_GPS::Weather_State &weather;
    float _new_wind_angle;
    char _new_wind_reference;
    float _new_wind_speed;
    char _new_wind_speed_units;
    char _new_wind_status;
    float _new_barometric_pressure;
    float _new_air_temperature;
    float _new_relative_humidity;
    float _new_water_speed;
    float _new_water_depth;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _MTK_init_string[];                  ///< init string for MediaTek units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];
};
