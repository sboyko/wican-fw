#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

void wican_commands_process(const uint8_t* const cipherdata, const size_t cipherdata_len, const bool isExport, char* const response);
