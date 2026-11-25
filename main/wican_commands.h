#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>


typedef struct QueueDefinition *QueueHandle_t;

void wican_commands_process(
	const uint8_t* const cipherdata,
	const size_t cipherdata_len,
	const bool isExport,
	char* const response,
	bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q),
	QueueHandle_t *q
);
