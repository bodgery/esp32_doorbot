#ifndef ESP32_DOORBOT_H
#  define ESP32_DOORBOT_H


void init_wifi();
void init_wiegand();
void check_serial_commands();
void handle_serial_command( String cmd );
void check_wiegand();
void check_tag( String tag );
bool check_tag_remote( String tag );
bool log_tag_remote( String tag );
void open_door();
void close_door();
void check_door_status();
void do_success();
void do_fail();
void wiegand_pin_state_change();
void wiegand_state_change( bool plugged, const char* message );
void wiegand_receive(
    uint8_t* data
    ,uint8_t bits
    ,const char* message
);
void wiegand_error(
    Wiegand::DataError error
    ,uint8_t* rawData
    ,uint8_t rawBits
    ,const char* message
);
void check_cache_build_time();
void rebuild_cache();
void rebuild_cache(
    unsigned int tries_left
);


#endif /* ifndef ESP32_DOORBOT_H */

