#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct QueueDefinition *QueueHandle_t;


// Setups reply-to-host callback
void gps_nmea_init(bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q));

// Waits for the whole NMEA sentence and delegates it to 'process_nmea_sentence' callback.
// NMEA format (ASCII string up to 83 symbols) is like '$TTSSS,data1,data2,...,dataN*CS<CR><LF>'.
void gps_nmea_read_sentence(
    const uint32_t timeoutMs,
    bool (*process_nmea_sentence)(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q),
    QueueHandle_t* (*tx_queue)(),
    const bool debugUnknowSentences
);

// Default NMEA sentences processor (currently considers only RMC)
bool gps_nmea_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q);

// Debug NMEA sentences processor (simply prints all GPS replies)
bool gps_nmea_debug_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q);

// Sends list of commands separated by 'Start Delimiter' ('$' or '!') via UART to GPS
void gps_nmea_send_commands(const char* commands, QueueHandle_t* q);
