#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_ExpandingArray.h>

#define AIVDM_BUFFER_SIZE 10

class AP_AIS
{
public:
    AP_AIS();

    /* Do not allow copies */
    AP_AIS(const AP_AIS &other) = delete;
    AP_AIS &operator=(const AP_AIS&) = delete;

    // Get the singleton
    static AP_AIS *get_singleton();

    // Return true if device is enabled
    bool enabled() const;

    // Initialize the AIS object and prepare it for use
    void init();

    // Device update loop
    void update();

    // send mavlink message to GCS
    void send(mavlink_channel_t chan);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_AIS *_singleton;

    // parameters
    AP_Int8 _type;             // type of AIS recever
    AP_Int16 _max_list;        // maximum number of vessels to track at once
    AP_Int16 _time_out;        // time in seconds that a vessel will be dropped from the list
    AP_Int16 _options;         // options bitmask

    enum AISType {
        AIS_NONE   = 0,
        AIS_NMEA   = 1,
    };

    enum options {
        AIS_OPTIONS_LOG_ALL_RAW         = 1<<0,
        AIS_OPTIONS_LOG_UNSUPPORTED_RAW = 1<<1,
        AIS_OPTIONS_LOG_DECODED         = 1<<2,
    };

    struct AIVDM {
        uint8_t num;
        uint8_t total;
        uint8_t ID;
        char payload[65];
    };
    AIVDM _incoming;
    AIVDM _AIVDM_buffer[AIVDM_BUFFER_SIZE];

    struct ais_vehicle_t {
        mavlink_ais_vessel_t info;
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
        uint32_t last_send_ms; // last time this message was sent via mavlink, stops us spamming the link
    };

    // list of the vessels that are being tracked
    AP_ExpandingArray<ais_vehicle_t> _list;

    AP_HAL::UARTDriver *_uart;

    uint16_t _send_index; // index of the last vessel send over mavlink

    // removed the given index from the AIVDM buffer shift following elements
    void buffer_shift(uint8_t i);

    // find vessel in existing list, if not then return new index if possible
    bool get_vessel_index(uint32_t mmsi, uint16_t &index, uint32_t lat = 0, uint32_t lon = 0);
    void clear_list_item(uint16_t index);

    // decode the payload
    bool payload_decode(const char *payload);

    // decode specific message types
    bool decode_position_report(const char *payload, uint8_t type);
    bool decode_static_and_voyage_data(const char *payload);

    // read the specified bits from the char array each char giving 6 bits
    void get_char(const char *payload, char *array, uint16_t low, uint16_t high);
    uint32_t get_bits(const char *payload, uint16_t low, uint16_t high);
    int32_t get_bits_signed(const char *payload, uint16_t low, uint16_t high);
    // un-encode the ASCII payload armoring
    uint8_t payload_char_decode(const char c);

    // log a raw AIVDM message
    void log_raw(const AIVDM *msg);

    // try and decode NMEA message
    bool decode(char c);

    // decode each term
    bool decode_latest_term();

    // convert from char to hex value for checksum
    int16_t char_to_hex(char a);

    // varables for decoding NMEA sentence
    char _term[65];            // buffer for the current term within the current sentence
    uint8_t _term_offset;      // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;      // term index within the current sentence
    uint8_t _checksum;         // checksum accumulator
    bool _term_is_checksum;    // current term is the checksum
    bool _sentence_valid;      // is current sentence valid so far
};

namespace AP {
    AP_AIS *ais();
};

