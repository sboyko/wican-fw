#ifndef gps_common_h
#define gps_common_h

#include <stdint.h>
#include <stdbool.h>

typedef struct QueueDefinition *QueueHandle_t;


// Инициализация работы с UART.
void gps_serial_init(QueueHandle_t *tx_queue, QueueHandle_t *rx_queue);

// Очистка буфера UART.
void gps_serial_flush();

// Чтение байта из буфера UART.
bool gps_serial_get_byte(uint8_t* symbol);

// Отправка строки в буфер UART.
bool gps_serial_send_buffer(const char* str, const uint16_t length);


// Разрешение на использование GPS
void gps_wait_enabled(const uint32_t xTicksToWait);

void gps_set_enabled(const bool isEnabledNotDisabled);


#endif // gps_common_h
