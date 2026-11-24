#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

// WiCAN HSI (Hardware/Software Interface)
typedef enum {
	WIC_HSI_USB = 0x01, // Always set for now
	WIC_HSI_BLE = 0x02,
	WIC_HSI_WIFI = 0x04, // WIFI_MODE_AP
	WIC_HSI_WEB = 0x08,  // WIFI_MODE_STA
} settings_wican_iterface_t;

typedef enum {
	// Числовые параметры (i32)
	CFG_WIC_HSI = 0, // HSI bit-field (settings_wican_iterface_t), for now WIC_HSI_USB is always on
	CFG_BLE_PASS, // BLE pairing passwords (PIN code), exactly 6 numeric digits (000000 to 999999)
	CFG_WIFI_AP_CH, // Soft-AP channel (1..13, 0 - let the driver choose automatically)
	CFG_WIFI_AP_PROTO, // Soft-AP protocol (0=TCP / 1=UDP)
	CFG_WIFI_AP_PORT, // Soft-AP port (0..65535)
	CFG_UART_BAUD, // UART USB baudrate (460800 by default, equal to the one used for ESP32 flashing)

	// Строковые параметры (str)
	CFG_TEST_CANARY, // Always contains 'canary' value to test NVS encryption via raw flash read
	CFG_WIC_PASS, // Master (admin) password (6..16 characters)
	CFG_WIC_NAME, // Name of WiCAN adapter (0..14 characters), full name is 'WiC_<wic_name>.<MAC_address>' like 'WiC_MyName.562e5fd68549'
	CFG_WIFI_AP_PASS, // Soft-AP WiFi password (for WPA2/WPA3 is 8..63 characters)
	CFG_WIFI_AP_IP, // Soft-AP IP address (IPv4 address, up to INET_ADDRSTRLEN(16))
	CFG_WIFI_STA_SSID, // Target-AP SSID (1..31 characters), empty value means 'WiCANabitabit' SSID name is used
	CFG_WIFI_STA_PASS, // Target-AP password (8..63 characters), empty value means 'airabit123' password is used
	CFG_WS_ADDR, // URL of WebSocket server (up to 230 bytes), like 'ws://212.24.43.2:80' or 'wss://akm02.abit.spb.ru'
	CFG_WS_UIN, // Remote Customer UIN (3..16 characters)
	CFG_GPS_SETT, // GPS COM setting (format 'baud_parity_dataBits_stopBits'), like '9600_0_8_0'
	CFG_GPS_INIT, // GPS initialization command ('Start Delimiter' ('$' or '!') are used as separators, up to 230 bytes)

	// Бинарные данные (blob)
	CFG_WS_AES, // Remote Customer AES (exactly 16 raw bytes)
	CFG_WS_PRK, // Remote Customer PrK (exactly 32 raw bytes)

	CFG_SETTINGS_COUNT
} settings_id_t;

// Максимальные размеры строковых параметров (с учётом '\0')
#define LEN_TEST_CANARY    7 // no cache (always 'canary' value)
#define LEN_WIC_PASS       17 // no cache
#define LEN_WIC_NAME       15
#define LEN_WIFI_AP_PASS   64 // no cache
#define LEN_WIFI_AP_IP     17 // no cache
#define LEN_WIFI_STA_SSID  32 // no cache
#define LEN_WIFI_STA_PASS  64 // no cache
#define LEN_WS_ADDR        231 // no cache
#define LEN_WS_UIN         17
#define LEN_GPS_SETT       14
#define LEN_GPS_INIT       231
#define LEN_WS_AES         16 // no cache
#define LEN_WS_PRK         32 // no cache


// Open NVS partition to read/write settings.
esp_err_t settings_init();

// Determine whether NVS partition is really enciphered.
bool settings_enciphered();

// Clear settings cache.
esp_err_t settings_reset(const bool eraseNvs);


// In most cases default value (which is returned on any error) is 0.
int32_t settings_get_int(const settings_id_t id);

// Store integer value (NVS 'i32' type).
esp_err_t settings_set_int(const settings_id_t id, const int32_t value);


// Terminating '\0' is always copied to 'out_value'.
esp_err_t settings_get_string(const settings_id_t id, char* out_value, const size_t value_size);

// Return error in case 'value' exceeds appropriate 'LEN_..' constant.
esp_err_t settings_set_string(const settings_id_t id, const char* value);


// Exact 'out_value' size is defined by appropriate 'LEN_..' constant.
esp_err_t settings_get_blob(const settings_id_t id, void* out_value);

// Exact 'value' size is defined by appropriate 'LEN_..' constant.
esp_err_t settings_set_blob(const settings_id_t id, const void* value);


// Lookup and update settings by 'name'.
// Return:
// - ESP_FAIL on error
// - positive string length otherwise
int settings_get_by_name(const char* name, char* out_value, const size_t value_size);

// Lookup settings by 'name'.
esp_err_t settings_set_by_name(const char* name, const char* value);
