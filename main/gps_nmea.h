#ifndef gps_nmea_h
#define gps_nmea_h

#include <stdint.h>
#include <stdbool.h>

typedef struct QueueDefinition *QueueHandle_t;


// setups reply-to-host callback
void gps_nmea_init(bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q));

// waits for the whole NMEA sentence and delegates it to 'process_nmea_sentence' callback
void gps_nmea_read_sentence(const uint32_t timeoutMs, void (*process_nmea_sentence)(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q), QueueHandle_t* (*tx_queue)());

// default NMEA sentences processor (currently considers only RMC)
void gps_nmea_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q);

// Debug NMEA sentences processor (simply prints all GPS replies)
void gps_nmea_debug_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q);

// sends list of commands (separated by '\r') via UART to GPS
void gps_nmea_send_commands(const char* commands, QueueHandle_t* q);


#endif // gps_nmea_h
