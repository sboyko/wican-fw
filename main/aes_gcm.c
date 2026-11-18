#include "aes_gcm.h"

#include "esp_log_wican.h"
#include "config_server.h"

#include <mbedtls/sha256.h>
#include <mbedtls/gcm.h>
#include <esp_random.h>
#include <esp_mac.h>
#include <esp_hmac.h>

#define TAG  __func__


// Константы для размеров (в байтах)
#define AES_KEY_BITS   256  // Поддерживается 128, 192 или 256
#define IV_LENGTH      12   // Рекомендуемая длина IV для GCM составляет 12 байт
#define AES_TAG_LENGTH 16   // Длина тега аутентификации (обычно 12, 14 или 16)

static mbedtls_gcm_context* g_gcm = NULL;

/*
* Initialize AES GCM context with
*  key = esp_hmac_calculate(BLOCK_KEY4 ($NvsKey HMAC_UP), iv + esp_efuse_mac_get_default() + (isExport ? Admin_passw : ''))
*/
static bool _aes_gcm_create(const uint8_t* const iv, const bool isExport, char* const response)
{
	bool result = false;
	uint8_t *hmac_out = NULL;
	uint8_t* message = NULL;

	// Выделяем 32 байта в куче (Heap) под результат (SHA-256)
	const size_t hmac_len = AES_KEY_BITS / 8;
	hmac_out = (uint8_t *)calloc(1, hmac_len);
	if (hmac_out == NULL) {
		ESP_LOGE(TAG, "Heap allocation error %d bytes", hmac_len);
		strcat(response, "heap_allocation_error");
		goto cleanup;
	}

	// Установка ключа для шифрования
	const size_t message_len = IV_LENGTH + 6 + (!isExport ? strlen(config_server_get_wic_pass()) : 0);
	message = (uint8_t *)calloc(1, message_len);
	if (message == NULL) {
		ESP_LOGE(TAG, "Heap allocation error %d bytes", message_len);
		strcat(response, "heap_allocation_error");
		goto cleanup;
	}

	memcpy(message, iv, IV_LENGTH);
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(message + IV_LENGTH));
	if (!isExport) {
		memcpy(message + IV_LENGTH + 6, config_server_get_wic_pass(), strlen(config_server_get_wic_pass()));
	}

	// Инициализация структуры mbedTLS
	mbedtls_gcm_init(g_gcm);

	int ret = esp_hmac_calculate(!isExport ? HMAC_KEY4 : HMAC_KEY5, message, message_len, hmac_out);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "esp_hmac_calculate() failed: %s", esp_err_to_name(ret));
		strcat(response, "hmac_calculate_error_");
		strcat(response, esp_err_to_name(ret));
		goto cleanup;
	}

	ret = mbedtls_gcm_setkey(g_gcm, MBEDTLS_CIPHER_ID_AES, hmac_out, AES_KEY_BITS);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "mbedtls_gcm_setkey (enc) failed: -0x%04X", -ret);
		strcat(response, "gcm_setkey_error_");
		strcat(response, esp_err_to_name(ret));
		goto cleanup;
	}

	result = true;

cleanup:
	if (message != NULL) {
		mbedtls_platform_zeroize(message, message_len);
		free(message);
		message = NULL;
	}
	if (hmac_out != NULL) {
		mbedtls_platform_zeroize(hmac_out, hmac_len);
		free(hmac_out);
		hmac_out = NULL;
	}

	return result;
}

/*
* API
*/
bool aes_gcm_init(const uint8_t* const cipherdata, const size_t cipherdata_len, const bool isExport, char* const response)
{
	if (cipherdata_len < IV_LENGTH + AES_TAG_LENGTH) { // at least 'IV(12) + AuthTag(16)' must present
		strcat(response, "Insufficient_input_data");
		return false;
	}

	if (cipherdata_len == IV_LENGTH + AES_TAG_LENGTH) { // empty command
		aes_gcm_cleanup();
		response[0] = 0; // skip reply
		return false;
	}

	if (g_gcm == NULL) {
		g_gcm = malloc(sizeof(mbedtls_gcm_context));
		if (g_gcm == NULL) {
			strcat(response, "gcm_context_allocation_error");
			return false;
		}

		if (!_aes_gcm_create(cipherdata, isExport, response)) {
			aes_gcm_cleanup();
			return false;
		}
	}

	return true;
}

/*
* API
*/
void aes_gcm_cleanup()
{
	if (g_gcm != NULL) {
		mbedtls_gcm_free(g_gcm);
		free(g_gcm);
		g_gcm = NULL;
	}
}

/*
* API
*/
int aes_gcm_decrypt(const uint8_t* const cipherdata, const size_t cipherdata_len, uint8_t* const plain_output, char* const response)
{
	if (g_gcm == NULL) {
		ESP_LOGE(TAG, "AES GCM context not initialized.");
		strcat(response, "gcm_context_absent");
		return -1;
	}
	if (cipherdata_len <= IV_LENGTH + AES_TAG_LENGTH) {
		ESP_LOGE(TAG, "Insufficient data length.");
		strcat(response, "Insufficient_input_data");
		return -1;
	}

	// Расшифрование с одновременной проверкой тега целостности
	const uint8_t* iv = cipherdata;
	const uint8_t* auth_tag = cipherdata + IV_LENGTH;
	const uint8_t* ciphertext = cipherdata + IV_LENGTH + AES_TAG_LENGTH;
	const size_t plaindata_len = cipherdata_len - IV_LENGTH - AES_TAG_LENGTH;

	const int ret = mbedtls_gcm_auth_decrypt(g_gcm, plaindata_len, iv, IV_LENGTH, NULL, 0,
			auth_tag, AES_TAG_LENGTH, ciphertext, plain_output);

	if (ret == MBEDTLS_ERR_GCM_AUTH_FAILED) {
		ESP_LOGE(TAG, "Authentication FAILED! Data is corrupted or tampered.");
		strcat(response, "Authentication_Failed");
		return -1;
	} else if (ret != ESP_OK) {
		ESP_LOGE(TAG, "mbedtls_gcm_auth_decrypt failed: -0x%04X", -ret);
		strcat(response, "gcm_decrypt_failed_");
		strcat(response, esp_err_to_name(ret));
		return -1;
	}

	plain_output[plaindata_len] = 0; // so 'data' can be interpreted as null-terminated string

	return plaindata_len;
}

/*
* API
*/
int aes_gcm_encrypt(const uint8_t* const plaindata, const size_t plaindata_len, uint8_t* const cipher_output, char* const response)
{
	if (g_gcm == NULL) {
		ESP_LOGE(TAG, "AES GCM context not initialized.");
		strcat(response, "gcm_context_absent");
		return -1;
	}

	// Шифрование и генерация тега аутентификации
	uint8_t* iv = cipher_output;
	uint8_t* auth_tag = cipher_output + IV_LENGTH;
	uint8_t* ciphertext = cipher_output + IV_LENGTH + AES_TAG_LENGTH;

	esp_fill_random(iv, IV_LENGTH);

	const int ret = mbedtls_gcm_crypt_and_tag(g_gcm, MBEDTLS_GCM_ENCRYPT, plaindata_len, iv, IV_LENGTH, NULL, 0,
			plaindata, ciphertext, AES_TAG_LENGTH, auth_tag);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "mbedtls_gcm_crypt_and_tag failed: -0x%04X", -ret);
		strcat(response, "gcm_encrypt_failed_");
		strcat(response, esp_err_to_name(ret));
		return -1;
	}

	return plaindata_len + IV_LENGTH + AES_TAG_LENGTH;
}
