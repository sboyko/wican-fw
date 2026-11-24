#include <freertos/FreeRTOS.h>
#include <cJSON.h>
#include <esp_ota_ops.h>
#include <mbedtls/platform_util.h>

#include "wican_settings.h"
#include "wican_commands.h"
#include "esp_log_wican.h"
#include "ble.h"
#include "can.h"
#include "config_server.h"
#include "aes_gcm.h"

#define TAG  __func__

static const esp_partition_t* ota_update_partition = NULL;
static esp_ota_handle_t ota_update_handle = 0; // set by esp_ota_begin(), must be freed via esp_ota_end()

/*
* First part of OTA process (Start-upload-finish)
*/
static esp_err_t startOta(const size_t image_size, char* const response)
{
	if((settings_get_int(CFG_WIC_HSI) & WIC_HSI_BLE) == WIC_HSI_BLE) {
		ble_disable();
	}

	if ((settings_get_int(CFG_WIC_HSI) & (WIC_HSI_WIFI | WIC_HSI_WEB)) != 0) {
		config_server_stop();
		//wifi_network_deinit();
	}

	can_disable();

	ESP_LOGI(TAG, "Starting OTA");

	const esp_partition_t *configured = esp_ota_get_boot_partition();
	const esp_partition_t *running = esp_ota_get_running_partition();

	if (configured != NULL && running != NULL) {
		if (configured != running) {
			ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08lx, but running from offset 0x%08lx",
					configured->address, running->address);
			ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
		}
		ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08lx)",
				running->type, running->subtype, running->address);
	}

	ota_update_partition = esp_ota_get_next_update_partition(NULL);
	if (ota_update_partition == NULL) {
		return ESP_ERR_OTA_SELECT_INFO_INVALID;
	}
	ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx",
				ota_update_partition->subtype, ota_update_partition->address);

	esp_err_t ret = esp_ota_begin(ota_update_partition, image_size, &ota_update_handle);
	if (ret != ESP_OK) {
		ota_update_partition = NULL;
	}
	return ret;
}

/*
* Second (repeated) part of OTA process (start-Upload-finish)
*/
static esp_err_t uploadOta(const uint8_t* data, const size_t data_size, char* const response)
{
	if (ota_update_partition == NULL) {
		return ESP_FAIL;
	}

	esp_err_t ret = esp_ota_write(ota_update_handle, data, data_size);
	if (ret != ESP_OK) {
		esp_ota_abort(ota_update_handle);
		ota_update_partition = NULL;
	} else {
		//vTaskDelay(1);
	}
	return ret;
}

/*
* Final part of OTA process (start-upload-Finish)
*/
static esp_err_t finishOta(char* const response)
{
	if (ota_update_partition == NULL) {
		return ESP_FAIL;
	}

	esp_err_t ret = esp_ota_end(ota_update_handle);
	if (ret != ESP_OK) {
		ota_update_partition = NULL;
		return ret;
	}

	ret = esp_ota_set_boot_partition(ota_update_partition);
	ota_update_partition = NULL;
	
	return ret;
}

/*
* Handler for WiCAN commands.
*/
static const uint16_t CMD_PING = 10; // "OK" reply
static const uint16_t CMD_RESTART = 11; // "OK" reply

static const uint16_t CMD_SETTING_GET = 12; // sprintf("%s for 'str'/%d for 'int'", (const char*)value)
static const uint16_t CMD_SETTING_SET = 13; // sscanf("%s for 'str'/%d for 'int'", (const char*)value); "OK" reply
static const uint16_t CMD_SETTING_RESET = 14; // in = (0=ram_cache/1=nvs)(uint8_t); "OK" reply

static const uint16_t CMD_WEB_STATUS = 20; // "%d" reply
static const uint16_t CMD_WEB_SET_CUSTOMER = 21; // in = CustomerUIN(16) +  CustomerAES(16) + CustomerPrK(32); "OK" reply

static const uint16_t CMD_OTA_START = 30; // in = image_size(uint32_t); "OK" reply
static const uint16_t CMD_OTA_UPLOAD = 31; // in = data; "OK" reply
static const uint16_t CMD_OTA_FINISH = 32; // "OK" reply

static int dispatchSetting(const int command, const uint8_t* const data, const size_t data_len, char* const response, const size_t response_len)
{
	switch (command) {

	case CMD_PING: {
		break; // OK
	}

	case CMD_RESTART: {
		config_server_graceful_restart(1);
		break; // OK
	}

	case CMD_SETTING_GET: {
		const char* setting_name = (const char*)data;

		const int result_len = settings_get_by_name(setting_name, response, response_len);
		if (result_len < 0) {
			strcat(response, "failed_get_");
			strcat(response, setting_name);
			return ESP_FAIL;
		}
		return result_len;
	}

	case CMD_SETTING_SET: {
		const char* setting_name = (const char*)data;
		const char* setting_value = strchr(setting_name, '\0') + 1;

		if (settings_set_by_name(setting_name, setting_value) != ESP_OK) {
			strcat(response, "failed_set_");
			strcat(response, setting_name);
			return ESP_FAIL;
		}
		break; // OK
	}

	case CMD_SETTING_RESET: {
		uint8_t clearNvs = 0;
		if (data_len >= sizeof(clearNvs)) {
			memcpy(&clearNvs, data, sizeof(clearNvs));
		}

		const esp_err_t ret = settings_reset(clearNvs == 1);
		if (ret != ESP_OK) {
			strcat(response, "failed_reset_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}
		break; // OK
	}

	case CMD_WEB_STATUS: {
		return sprintf(response, "%d", config_server_get_ws_status());
	}

	case CMD_WEB_SET_CUSTOMER: { // WiCAN initialization data (Customer UIN (16), Customer AES (16), Customer PrK (32))
		if (data_len != 16 + 16 + 32) {
			strcat(response, "Cmd_malformed");
			return ESP_FAIL;
		}

		esp_err_t ret = settings_set_blob(CFG_WS_AES, data + 16);
		if (ret != ESP_OK) {
			strcat(response, "failed_aes_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}

		ret = settings_set_blob(CFG_WS_PRK, data + 16 + 16);
		if (ret != ESP_OK) {
			strcat(response, "failed_prk_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}

		((char*)data)[17] = 0;
		ret = settings_set_by_name("ws_uin", (const char*)data);
		if (ret != ESP_OK) {
			strcat(response, "failed_uin_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}
		
		break; // OK
	}

	case CMD_OTA_START: {
		uint32_t image_size = 0;
		if (data_len >= sizeof(image_size)) {
			memcpy(&image_size, data, sizeof(image_size));
		}
		
		const esp_err_t ret = startOta(image_size, response);
		if (ret != ESP_OK) {
			strcat(response, "ota_start_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}
		break; // OK
	}

	case CMD_OTA_UPLOAD: {
		const esp_err_t ret = uploadOta(data, data_len, response);
		if (ret != ESP_OK) {
			strcat(response, "ota_upload_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}
		break; // OK
	}

	case CMD_OTA_FINISH: {
		const esp_err_t ret = finishOta(response);
		if (ret != ESP_OK) {
			strcat(response, "ota_finish_");
			wican_log_error(response, ret, ERR_ESP_TYPE);
			return ESP_FAIL;
		}
		break; // OK
	}

	default: {
		strcat(response, "Cmd_unknown");
		return ESP_FAIL;
	}

	}

	strcpy(response, "OK"); // same to 'ok_str'
	return 1000; // indicates plain text
}

void wican_commands_process(const uint8_t* const cipherdata, const size_t cipherdata_len, const bool isExport, char* const response)
{
	static uint16_t g_command_counter = 0;

	if (!aes_gcm_init(cipherdata, cipherdata_len, isExport, response)) {
		if (response[0] != 0) {
			strcat(response, "CANERROR");
		}
		g_command_counter = 0;
		return;
	}

	static const size_t CMD_PDU_MAX_SIZE = 233;
	uint8_t* data = malloc(CMD_PDU_MAX_SIZE);
	char* outdata = malloc(CMD_PDU_MAX_SIZE);
	bool processed = false;

	while (true) {
		const int data_len = aes_gcm_decrypt(cipherdata, cipherdata_len, data, response);
		if (data_len < 0) {
			break;
		}

		// PDU Payload = cmd(2) + cmd_counter(2) + data(up to 228)
		if (data_len < 4) {
			strcat(response, "Pdu_malformed");
			break;
		}
		const uint16_t cmd = (data[0] + (data[1] << 8));
		const uint16_t cmd_counter = (data[2] + (data[3] << 8));

		if (cmd_counter != g_command_counter) {
			strcat(response, "Invalid_session");
			break;
		}

		outdata[0] = 0;
		const int reply_len = dispatchSetting(cmd, data + 4, data_len - 4, outdata, CMD_PDU_MAX_SIZE - 1);

		if (reply_len < 0) { // error text
			strcat(response, outdata);
		} else if (reply_len > CMD_PDU_MAX_SIZE - 1) { // plain text
			strcpy(response, outdata);
			processed = true;
		} else { // reply for enciphering
			const int result_len = aes_gcm_encrypt((const uint8_t*)outdata, reply_len, data, response);
			if (result_len < 0) {
				break;
			}

			int offset = strlen(response);
			for (int i = 0; i < result_len; i++) {
				offset += sprintf(response + offset, "%02X", data[i]);
			}
			processed = true;
		}

		break;
	}

	mbedtls_platform_zeroize(data, CMD_PDU_MAX_SIZE);
	free(data);
	mbedtls_platform_zeroize(outdata, CMD_PDU_MAX_SIZE);
	free(outdata);

	if (processed) {
		g_command_counter += 1;
	} else {
		strcat(response, "CANERROR");
	}
}
