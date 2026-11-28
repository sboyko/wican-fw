#include <freertos/FreeRTOS.h>
#include <nvs_flash.h>
#include <esp_flash.h>

#include "wican_settings.h"
#include "esp_log_wican.h"

#define TAG  __func__
static const char* NVS_NAMESPACE = "app_cfg";

// global vars
static bool s_initialized = false;

static StaticSemaphore_t s_mutex_buf;
static SemaphoreHandle_t s_mutex = NULL;

// Смещения в RAM-пуле (только для кэшируемых строк)
enum {
    OFF_WIC_NAME = 1,
    OFF_WS_UIN = OFF_WIC_NAME + LEN_WS_ADDR,
    OFF_GPS_SETT = OFF_WS_UIN + LEN_WS_UIN,
    OFF_GPS_INIT = OFF_GPS_SETT + LEN_GPS_SETT,
    TOTAL_STR_POOL = OFF_GPS_INIT + LEN_GPS_INIT,
};

typedef enum {
    CFG_TYPE_I32 = 1,
    CFG_TYPE_STR,
	CFG_TYPE_BLOB,
} cfg_type_t;

// Единый дескриптор (хранится во Flash)
typedef struct {
    const char *nvs_key; // Имя ключа в NVS (макс. 15 символов)
    cfg_type_t type; // Тип ключа
    union {
        struct {
            int32_t default_val; // Значение по умолчанию
        } i32;
        struct {
            const char *default_val; // Значение по умолчанию
            size_t max_len; // Максимальная длина строки с '\0'
            size_t pool_offset; // Смещение в пуле (если = 0, значит не кешируем)
        } str;
        struct {
            size_t max_len; // Максимальная длина данных
        } blob;
    };
} cfg_desc_t;

// Вспомогательные макросы для читаемости таблицы
#define DESC_I32(key, d) \
    { .nvs_key = (key), .type = CFG_TYPE_I32, .i32 = { .default_val = (d) } }

#define DESC_STR(key, d, len, off) \
    { .nvs_key = (key), .type = CFG_TYPE_STR, .str = { .default_val = (d), .max_len = (len), .pool_offset = (off) } }

#define DESC_BLOB(key, len) \
    { .nvs_key = (key), .type = CFG_TYPE_BLOB, .blob = { .max_len = (len) } }

// Все настройки перечислены ниже.
// Начальное значение всех настроек задаётся из программы WiCAN Admin.
// Если ничего не задано, адаптер поддерживает только WIC_HSI_USB интерфейс, авторизироваться при этом невозможно.
//
static const cfg_desc_t s_descriptors[CFG_SETTINGS_COUNT] = {
	[CFG_WIC_HSI] = DESC_I32("wic_hsi", 0),
	[CFG_BLE_PASS] = DESC_I32("ble_pass", 0),
	[CFG_WIFI_AP_CH] = DESC_I32("ap_ch", 0),
	[CFG_WIFI_AP_PROTO] = DESC_I32("ap_proto", 0),
	[CFG_WIFI_AP_PORT] = DESC_I32("ap_port", 0),
	[CFG_UART_BAUD] = DESC_I32("uart_baud", 921600), // единственная ненулевая настройка

	[CFG_TEST_CANARY] = DESC_STR("canary", "", LEN_TEST_CANARY, 0),
	[CFG_WIC_PASS] = DESC_STR("wic_pass", "", LEN_WIC_PASS, 0),
	[CFG_WIC_NAME] = DESC_STR("wic_name", "", LEN_WIC_NAME, OFF_WIC_NAME),
	[CFG_WIFI_AP_PASS] = DESC_STR("ap_pass", "", LEN_WIFI_AP_PASS, 0),
	[CFG_WIFI_AP_IP] = DESC_STR("ap_ip", "", LEN_WIFI_AP_IP, 0),
	[CFG_WIFI_STA_SSID] = DESC_STR("sta_ssid", "", LEN_WIFI_STA_SSID, 0),
	[CFG_WIFI_STA_PASS] = DESC_STR("sta_pass", "", LEN_WIFI_STA_PASS, 0),
	[CFG_WS_ADDR] = DESC_STR("ws_addr", "", LEN_WS_ADDR, 0),
	[CFG_WS_UIN] = DESC_STR("ws_uin", "", LEN_WS_UIN, OFF_WS_UIN),
	[CFG_GPS_SETT] = DESC_STR("gps_sett", "", LEN_GPS_SETT, OFF_GPS_SETT),
	[CFG_GPS_INIT] = DESC_STR("gps_init", "", LEN_GPS_INIT, OFF_GPS_INIT),

	[CFG_WS_AES] = DESC_BLOB("ws_aes", LEN_WS_AES),
	[CFG_WS_PRK] = DESC_BLOB("ws_prk", LEN_WS_PRK),
};

// Состояние кэша в RAM
static int32_t s_i32_cache[CFG_TEST_CANARY] = {0}; // CFG_TEST_CANARY это первый 'str' параметр, то есть все 'i32' идут до него
static char s_str_pool[TOTAL_STR_POOL] = {0};
static bool s_loaded[CFG_SETTINGS_COUNT] = {false};


/**
 * Opens default (that is 'nvs' named) NVS partions.
 * Works both for non-enciphered and enciphered partions.
 */
esp_err_t settings_init()
{
	esp_err_t ret = nvs_flash_init();
	if (ret != ESP_OK) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	s_initialized = (ret == ESP_OK);

	if (s_mutex == NULL) {
		s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_buf);
	}
	return ret;
}

/**
 * Проверяет (без записи во Flash), что секрет не хранится в открытом виде ни на одной странице раздела "nvs".
 */
bool settings_enciphered()
{
	if (!s_initialized) {
		return false;
	}

	// first check that ordinal read returns 'canary'
	char secret[LEN_TEST_CANARY];
	if (settings_get_string(CFG_TEST_CANARY, secret, sizeof(secret)) != ESP_OK
		|| memcmp(secret, s_descriptors[CFG_TEST_CANARY].nvs_key, sizeof(secret)) != 0
		) {
		return false;
	}

	return true;
	
	// Lookup for 'canary' via raw flash reading.
	// Actually code below should be run only once, no need to have in firmware.
	/*

	// 1. Находим раздел "nvs"
	const esp_partition_t* nvs_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");
	if (nvs_part == NULL) {
		ESP_LOGE(TAG, "No 'nvs' partition");
		return false;
	}

	// 2. Выделяем буфер в куче под одну страницу (4096 байт)
	const size_t page_size = 4096;
	uint8_t* page_buf = (uint8_t *)malloc(page_size);
	if (page_buf == NULL) {
		ESP_LOGE(TAG, "Allocation of %d bytes fails", page_size);
		return false;
	}

	bool is_encrypted = true;

	// 3. Читаем все 8 страниц (32 КБ) напрямую с SPI Flash по физическим адресам
	for (size_t offset = 0; offset < nvs_part->size; offset += page_size) {
		esp_err_t err = esp_flash_read(NULL, page_buf, nvs_part->address + offset, page_size);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "SPI Flash read error (offset 0x%x): %s", offset, esp_err_to_name(err));
			is_encrypted = false;
			break;
		}

		// Проверяем заголовок страницы (первые 4 байта)
		uint32_t page_state;
		memcpy(&page_state, page_buf, sizeof(uint32_t));
		
		// Пропускаем стертые (пустые) страницы
		if (page_state == 0xFFFFFFFF) {
			continue;
		}

		// Ищем совпадение секрета в области записей (со смещения 0x40 до конца страницы)
		for (size_t i = 0x40; i <= page_size - sizeof(secret); i++) {
			if (memcmp(page_buf + i, secret, sizeof(secret)) == 0) {
				ESP_LOGE(TAG, "LEAK: Secret found in plain view on page 0x%x (offset 0x%x)!", offset, i);
				is_encrypted = false;
				break;
			}
		}

		if (!is_encrypted) {
			break;
		}
	}

	free(page_buf);

	if (is_encrypted) {
		ESP_LOGI(TAG, "Audit completed: no plaintext found in Flash.");
	}
	return is_encrypted;
	*/
}

/**
 * If 'eraseNvs == true' then the NVS partition is physically erased, otherwise only its cache are cleared.
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_reset(const bool eraseNvs)
{
	configASSERT(s_mutex != NULL);

	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	esp_err_t err = ESP_OK;
	if (eraseNvs) {
		nvs_handle_t handle;
		err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
		if (err == ESP_OK) {
			err = nvs_erase_all(handle);
			if (err == ESP_OK) {
				err = nvs_commit(handle);
			}
			nvs_close(handle);
		}
	}

	if (err == ESP_OK) {
		memset(s_loaded, 0, sizeof(s_loaded));
		memset(s_str_pool, 0, sizeof(s_str_pool));
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return err;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
int32_t settings_get_int(const settings_id_t id)
{
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_I32) {
		ESP_LOGE(TAG, "Wrong ID for nvs_get_i32: %d", id);
		return 0;
	}
	const cfg_desc_t *desc = &s_descriptors[id];

	configASSERT(s_mutex != NULL);
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	if (!s_loaded[id]) {
		int32_t val = desc->i32.default_val;
		nvs_handle_t handle;
		if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
			if (nvs_get_i32(handle, desc->nvs_key, &val) == ESP_OK) {
				s_loaded[id] = true;
			}
			nvs_close(handle);
		}
		s_i32_cache[id] = val;
	}

	const int32_t result = s_i32_cache[id];
	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return result;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_set_int(const settings_id_t id, const int32_t value)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_I32) {
		return ESP_ERR_INVALID_ARG;
	}
	configASSERT(s_mutex != NULL);

	const cfg_desc_t *desc = &s_descriptors[id];
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	if (s_loaded[id] && s_i32_cache[id] == value) {
		if (s_mutex != NULL) {
			xSemaphoreGive(s_mutex);
		}
		return ESP_OK;
	}

	nvs_handle_t handle;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
	if (err == ESP_OK) {
		err = nvs_set_i32(handle, desc->nvs_key, value);
		if (err == ESP_OK) {
			err = nvs_commit(handle);
		}
		nvs_close(handle);
	}

	if (err == ESP_OK) {
		s_i32_cache[id] = value;
		s_loaded[id] = true;
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return err;
}

static esp_err_t local_copy_safe(char *dst, const char *src, size_t dst_size)
{
	if (dst_size == 0 || strlen(src) >= dst_size) {
		return ESP_ERR_NVS_INVALID_LENGTH;
	}

	strcpy(dst, src);
	return ESP_OK;
}

static esp_err_t local_read_str_from_nvs(const cfg_desc_t *desc, char *target_buf, const size_t target_size)
{
	nvs_handle_t handle;
	esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
	if (result == ESP_OK) {
		size_t len = target_size;
		result = nvs_get_str(handle, desc->nvs_key, target_buf, &len);
		if (result != ESP_OK) {
			//local_copy_safe(target_buf, desc->str.default_val, target_size);
		}
		nvs_close(handle);
	} else {
		//local_copy_safe(target_buf, desc->str.default_val, target_size);
	}
	return result;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_get_string(const settings_id_t id, char* out_value, const size_t value_size)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_STR || out_value == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	const cfg_desc_t *desc = &s_descriptors[id];

	configASSERT(s_mutex != NULL);
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	esp_err_t result = ESP_OK;
	if (desc->str.pool_offset == 0) {
		result = local_read_str_from_nvs(desc, out_value, value_size);
	} else {
		char *slot = &s_str_pool[desc->str.pool_offset];
		if (!s_loaded[id]) {
			result = local_read_str_from_nvs(desc, slot, value_size);
			if (result == ESP_OK) {
				s_loaded[id] = true;
			}
		}
		if (s_loaded[id]) {
			result = local_copy_safe(out_value, slot, value_size);
		}
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return result;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_set_string(const settings_id_t id, const char* value)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_STR || value == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	const cfg_desc_t *desc = &s_descriptors[id];

	if (strlen(value) >= desc->str.max_len) {
		return ESP_ERR_INVALID_SIZE;
	}

	configASSERT(s_mutex != NULL);
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	if (s_loaded[id] && desc->str.pool_offset != 0) {
		char *slot = &s_str_pool[desc->str.pool_offset];
		if (strcmp(slot, value) == 0) {
			if (s_mutex != NULL) {
				xSemaphoreGive(s_mutex);
			}
			return ESP_OK;
		}
	}

	nvs_handle_t handle;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
	if (err == ESP_OK) {
		err = nvs_set_str(handle, desc->nvs_key, value);
		if (err == ESP_OK) {
			err = nvs_commit(handle);
		}
		nvs_close(handle);
	}

	if (err == ESP_OK && desc->str.pool_offset != 0) {
		char *slot = &s_str_pool[desc->str.pool_offset];
		local_copy_safe(slot, value, desc->str.max_len);
		s_loaded[id] = true;
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return err;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_get_blob(const settings_id_t id, void* out_value)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_BLOB || out_value == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	const cfg_desc_t *desc = &s_descriptors[id];

	configASSERT(s_mutex != NULL);
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	nvs_handle_t handle;
	esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
	if (result == ESP_OK) {
		size_t length = desc->blob.max_len;
		result = nvs_get_blob(handle, desc->nvs_key, out_value, &length);
		nvs_close(handle);
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return result;
}

/**
 * Multi-threaded (protected by mutex 's_mutex').
 */
esp_err_t settings_set_blob(const settings_id_t id, const void* value)
{
	if (!s_initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (id < 0 || id >= CFG_SETTINGS_COUNT || s_descriptors[id].type != CFG_TYPE_BLOB || value == NULL) {
		return ESP_ERR_INVALID_ARG;
	}
	const cfg_desc_t *desc = &s_descriptors[id];

	configASSERT(s_mutex != NULL);
	if (s_mutex != NULL) {
		xSemaphoreTake(s_mutex, portMAX_DELAY);
	}

	nvs_handle_t handle;
	esp_err_t result = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
	if (result == ESP_OK) {
		const size_t length = desc->blob.max_len;
		result = nvs_set_blob(handle, desc->nvs_key, value, length);
		if (result == ESP_OK) {
			result = nvs_commit(handle);
		}
		nvs_close(handle);
	}

	if (s_mutex != NULL) {
		xSemaphoreGive(s_mutex);
	}
	return result;
}

/**
 * - for string-valued setting fill 'out_value' with value as '%s'.
 * - for int-valued setting fill 'out_value' with value as '%d'.
 * - blob-valued setting not supported
 */
int settings_get_by_name(const char* name, char* out_value, const size_t value_size)
{
	for (int i = 0; i < CFG_SETTINGS_COUNT; ++i) {
		const cfg_desc_t *desc = &s_descriptors[i];
		if (strcmp(desc->nvs_key, name) != 0) {
			continue;
		}

		if (desc->type == CFG_TYPE_STR) {
			if (settings_get_string(i, out_value, value_size) == ESP_OK) {
				return strlen(out_value);
			}
		} else if (desc->type == CFG_TYPE_I32) {
			return snprintf(out_value, value_size, "%d", (int)settings_get_int(i));
		}
		break;
	}
	return ESP_FAIL;
}

/**
 * - for string-valued setting store value as '%s' (including '\0').
 * - for int-valued setting store value as atoi('%s').
 * - for blob-valued setting store value as byte-buffer of exact size (defined by appropriate 'LEN_..' constant).
 */
esp_err_t settings_set_by_name(const char* name, const char* value)
{
	for (int i = 0; i < CFG_SETTINGS_COUNT; ++i) {
		const cfg_desc_t *desc = &s_descriptors[i];
		if (strcmp(desc->nvs_key, name) != 0) {
			continue;
		}

		if (desc->type == CFG_TYPE_STR) {
			return settings_set_string(i, value);
		} else if (desc->type == CFG_TYPE_I32) {
			return settings_set_int(i, (uint32_t)atoi(value));
		} else if (desc->type == CFG_TYPE_BLOB) {
			return settings_set_blob(i, value);
		}
		break;
	}
	return ESP_ERR_NOT_FOUND;
}
