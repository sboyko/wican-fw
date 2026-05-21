#include "gps_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "types.h"

#define TAG  __func__


static QueueHandle_t *uart_tx_queue = NULL, *uart_rx_queue = NULL;
static xdev_buffer rx_buffer;
static xdev_buffer tx_buffer;

static uint8_t serial_buffer[DEV_BUFFER_LENGTH * 2];
static int serial_buffer_len = 0;
static int serial_buffer_pos = 0;

static EventGroupHandle_t s_gps_event_group = NULL;
static const int GPS_ENABLED_BIT = BIT0;

// Инициализация работы с UART.
void gps_serial_init(QueueHandle_t *tx_queue, QueueHandle_t *rx_queue)
{
	uart_tx_queue = tx_queue;
	uart_rx_queue = rx_queue;

	if (s_gps_event_group == NULL) {
		s_gps_event_group = xEventGroupCreate();
	}
}

// Проверка готовности UART.
static bool gps_serial_ready()
{
	return uart_tx_queue != NULL && uart_rx_queue != NULL;
}

// Очистка буфера UART.
void gps_serial_flush()
{
	if (!gps_serial_ready()) {
		return;
	}

	serial_buffer_len = 0;
	serial_buffer_pos = 0;

	while (xQueueReceive(*uart_rx_queue, &rx_buffer, 0)) {
		// cleanup incoming queue
	}
}

// Чтение байта из буфера UART.
bool gps_serial_get_byte(uint8_t* symbol)
{
	if (!gps_serial_ready()) {
		return false;
	}

	if (serial_buffer_len == 0) {
		while (xQueuePeek(*uart_rx_queue, &rx_buffer, 5)) {
			if (serial_buffer_len + rx_buffer.usLen >= sizeof(serial_buffer)) {
				break;
			}

			xQueueReceive(*uart_rx_queue, &rx_buffer, 0);
			memcpy(&serial_buffer[serial_buffer_len], rx_buffer.ucElement, rx_buffer.usLen);
			serial_buffer_len += rx_buffer.usLen;
		}
	}

	if (serial_buffer_pos < serial_buffer_len) {
		*symbol = serial_buffer[serial_buffer_pos];
		serial_buffer_pos += 1;
		return true;
	}

	serial_buffer_len = 0;
	serial_buffer_pos = 0;
	return false;
}

// Отправка строки в буфер UART.
bool gps_serial_send_buffer(const char* str, const uint16_t length)
{
	if (!gps_serial_ready()) {
		return false;
	}

	const int totalLength = (length == 0 ? strlen(str) : length);
	int offset = 0;

	while (offset < totalLength) {
		tx_buffer.usLen = MIN(totalLength - offset, sizeof(tx_buffer.ucElement));
		memcpy(tx_buffer.ucElement, str + offset, tx_buffer.usLen);

		if (xQueueSend(*uart_tx_queue, &tx_buffer, 10) != pdTRUE) {
			ESP_LOGE(TAG, "xQueueSend() fails");
			//assert(false);
			return false;
		}
		offset += tx_buffer.usLen;
	
#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG, tx_buffer.ucElement, tx_buffer.usLen, ESP_LOG_INFO);
#endif
	}

	return true;
}

void gps_wait_enabled(const uint32_t xTicksToWait)
{
	if (s_gps_event_group != NULL) {
		xEventGroupWaitBits(s_gps_event_group, GPS_ENABLED_BIT,
			pdFALSE, pdFALSE, xTicksToWait); // portMAX_DELAY
	} else {
		vTaskDelay(xTicksToWait);
	}
}

void gps_set_enabled(const bool isEnabledNotDisabled)
{
	if (s_gps_event_group != NULL) {
		if (isEnabledNotDisabled) {
			xEventGroupSetBits(s_gps_event_group, GPS_ENABLED_BIT);
		} else {
			xEventGroupClearBits(s_gps_event_group, GPS_ENABLED_BIT);
		}
	}
}
