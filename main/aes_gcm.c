#include "aes_gcm.h"

#include "esp_log_wican.h"
#include "config_server.h"

#include <string.h>
#include <mbedtls/sha256.h>
#include <esp_random.h>
#include <esp_mac.h>

#define TAG  __func__


// Константы для размеров (в байтах)
#define AES_KEY_BITS   256  // Поддерживается 128, 192 или 256
#define IV_LENGTH      12   // Рекомендуемая длина IV для GCM составляет 12 байт
#define AES_TAG_LENGTH 16   // Длина тега аутентификации (обычно 12, 14 или 16)

// First 6 bytes are overridden with MAC address
static uint8_t gcm_aad[AES_TAG_LENGTH] = { 0xda, 0x15, 0x53, 0x9c, 0x6b, 0x70, 0x6b, 0xf9, 0x7a, 0x16, 0xc0, 0x2b, 0x63, 0x1a, 0x4b, 0x12 };

/*
* API
*/
bool aes_gcm_init(mbedtls_gcm_context* gcm)
{
	// Инициализация структуры mbedTLS
	mbedtls_gcm_init(gcm);

	// Установка ключа для шифрования
	const char* psw = config_server_get_wic_pass();

	uint8_t psw_digest[AES_KEY_BITS / 8];
	int ret = mbedtls_sha256((const uint8_t*) psw, strlen(psw), psw_digest, 0); // '0' for SHA-256
	if (ret != 0) {
		ESP_LOGE(TAG, "mbedtls_sha256() failed, reason = %d", ret);
		return false;
	}

	ret = mbedtls_gcm_setkey(gcm, MBEDTLS_CIPHER_ID_AES, psw_digest, AES_KEY_BITS);
	if (ret != 0) {
		ESP_LOGE(TAG, "mbedtls_gcm_setkey (enc) failed: -0x%04X", -ret);
		return false;
	}

	uint8_t derived_mac_addr[6] = {0};
	ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_SOFTAP));
	for (size_t i = 0, in = sizeof(derived_mac_addr); i < in; ++i) {
		gcm_aad[i] = derived_mac_addr[i];
	}

	return true;
}

/*
* API
*/
int aes_gcm_decrypt(mbedtls_gcm_context* gcm, const uint8_t* const cipherdata, const size_t cipherdata_len, uint8_t* const plain_output)
{
	// Расшифрование с одновременной проверкой тега целостности
	const uint8_t* iv = cipherdata;
	const uint8_t* auth_tag = cipherdata + IV_LENGTH;
	const uint8_t* ciphertext = cipherdata + IV_LENGTH + AES_TAG_LENGTH;
	const size_t plaindata_len = cipherdata_len - IV_LENGTH - AES_TAG_LENGTH;

	const int ret = mbedtls_gcm_auth_decrypt(gcm, plaindata_len, iv, IV_LENGTH, gcm_aad, sizeof(gcm_aad),
			auth_tag, AES_TAG_LENGTH, ciphertext, plain_output);

	if (ret == MBEDTLS_ERR_GCM_AUTH_FAILED) {
		ESP_LOGE(TAG, "Authentication FAILED! Data is corrupted or tampered.");
		return -2;
	} else if (ret != 0) {
		ESP_LOGE(TAG, "mbedtls_gcm_auth_decrypt failed: -0x%04X", -ret);
		return -1;
	}

	return plaindata_len;
}

/*
* API
*/
int aes_gcm_encrypt(mbedtls_gcm_context* gcm, const uint8_t* const plaindata, const size_t plaindata_len, uint8_t* const cipher_output)
{
	// Шифрование и генерация тега аутентификации
	uint8_t* iv = cipher_output;
	uint8_t* auth_tag = cipher_output + IV_LENGTH;
	uint8_t* ciphertext = cipher_output + IV_LENGTH + AES_TAG_LENGTH;

	esp_fill_random(iv, IV_LENGTH);

	const int ret = mbedtls_gcm_crypt_and_tag(gcm, MBEDTLS_GCM_ENCRYPT, plaindata_len, iv, IV_LENGTH, gcm_aad, sizeof(gcm_aad),
			plaindata, ciphertext, AES_TAG_LENGTH, auth_tag);

	if (ret != 0) {
		ESP_LOGE(TAG, "mbedtls_gcm_crypt_and_tag failed: -0x%04X", -ret);
		return -1;
	}

	return plaindata_len + IV_LENGTH + AES_TAG_LENGTH;
}
