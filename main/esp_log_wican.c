#include "esp_log_wican.h"

#include <mbedtls/error.h>

/**
 * Uses appropriate logging for:
 *  esp_err_t
 *  mbedTLS (int codes)
 */
void wican_log_error(char* const buffer, int err_code, const wican_log_error_t err_type)
{
	static const int ERR_DESCRIPTION_MAX_SIZE = 30;

	if (err_type == ERR_ESP_TYPE) {
		const char* err_text = esp_err_to_name(err_code);
		if (err_text != NULL) {
			strlcpy(buffer + strlen(buffer), err_text, ERR_DESCRIPTION_MAX_SIZE);
		}
		sprintf(buffer + strlen(buffer), "_esp_%x", err_code);
	}
	else if (err_type == ERR_MBEDTLS_TYPE) {
		mbedtls_strerror(err_code, buffer, ERR_DESCRIPTION_MAX_SIZE);
		sprintf(buffer + strlen(buffer), "_mbed_%04x", err_code); // note that actually 'err_code' is negative
	}
}