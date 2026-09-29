/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#pragma once

#define AP_MODE				0
#define APSTA_MODE			1

#define CAN_NORMAL			0
#define CAN_SILENT			1

#define UDP_PORT			0
#define TCP_PORT			1

#define SLCAN				0
#define REALDASH			1
#define SAVVYCAN			2
#define OBD_ELM327			3

// WiCAN HSI (Hardware/Software Interface)
typedef enum {
	WIC_HSI_USB = 0x01, // Always set for now
	WIC_HSI_BLE = 0x02,
	WIC_HSI_WIFI = 0x04, // WIFI_MODE_AP
	WIC_HSI_WEB = 0x08,  // WIFI_MODE_STA
} config_server_wican_iterface_t;

typedef struct _device_config
{
	char can_datarate[65];
	char can_mode[65];
	char protocol[65];
	char sleep_status[32];
	char sleep_volt[32];
	char batt_alert[32];
	char batt_alert_ssid[65];
	char batt_alert_pass[65];
	char batt_alert_volt[32];
	char batt_alert_protocol[65];
	char batt_alert_url[256];
	char batt_alert_port[32];
	char batt_alert_topic[256];
	char batt_alert_time[16];
	char batt_mqtt_user[64];
	char batt_mqtt_pass[64];
	char mqtt_en[10];
	char mqtt_url[256];
	char mqtt_port[32];
	char mqtt_user[64];
	char mqtt_pass[64];
	char mqtt_elm327_log[10];
	char mqtt_tx_topic[64];
	char mqtt_rx_topic[64];
	char mqtt_status_topic[64];

	// Master (admin) password, up to 16 characters.
	char wic_pass[18];

	// Visible adapter name is 'WiC_<wic_name>.<MAC_address>' like 'WiC_MyName.562e5fd68549'
	// So maximum length of 'wic_name' is:  32 (Wifi SSID / BLE Device Name) - 4('WiC_') - 6*2(MAC) - 1('.') - 1('\0') = 14
	char wic_name[16];

	// Unsigned integer value (stored as hex string) which corresponds to 'config_server_wican_iterface_t' enumeration.
	// Note: for now WIC_HSI_USB is always set.
	char wic_hsi[10];

	// BLE pairing passwords (passkeys or PIN codes) must be exactly 6 numeric digits (000000 to 999999).
	char ble_pass[8];

	// WiFi password of soft-AP (for modern WPA2 and WPA3 networks must be between 8 and 63 characters).
	char ap_pass[65];

	// Channel of soft-AP (1..13, 0 - let the driver choose automatically)
	char ap_ch[3];

	// Protocol ('tcp' / 'udp') of soft-AP 
	char ap_proto[5];

	// IP address of soft-AP protocol (like '192.168.80.1')
	char ap_ip[18];

	// Port of soft-AP protocol (0..65535)
	char ap_port[7];

	// SSID of target-AP (1..32 characters)
	// Empty value means Abit-specific SSID name is used.
	char sta_ssid[34];

	// Password of target-AP (8..63 characters)
	// Empty value means Abit-specific SSID password is used.
	char sta_pass[65];

	// URL of WebSocket server (up to 230 bytes, like 'ws://212.24.43.2:80' or 'wss://akm02.abit.spb.ru').
	char ws_addr[232];

	// UART USB baudrate
	char uart_baud[10];

	// GPS COM setting (format 'baud_parity_dataBits_stopBits' , i.e. '9600_0_8_0')
	char gps_sett[15];

	// GPS initialization command (i.e. '$PMTK220,1000\\r$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\\r')
	char gps_init[232];

} device_config_t;

typedef struct QueueDefinition *QueueHandle_t;

void config_server_start(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, char * did);
void config_server_restart(void);
void config_server_stop(void);

int8_t config_server_get_can_rate(void);
int8_t config_server_get_can_mode(void);
void config_server_wifi_connected(bool flag);
//bool config_server_get_wifi_connected(void);
void config_server_set_sta_ip(char* ip);
void config_server_get_sta_ip(char* ip);
int8_t config_server_protocol(void);
int8_t config_server_get_sleep_config(void);
void config_server_set_ble_config(uint8_t b);
bool config_server_ws_connected(void);
bool config_server_get_sleep_volt(float *sleep_volt);
int8_t config_server_get_battery_alert_config(void);
int32_t config_server_get_alert_port(void);
char *config_server_get_alert_ssid(void);
char *config_server_get_alert_pass(void);
char *config_server_get_alert_protocol(void);
char *config_server_get_alert_url(void);
char *config_server_get_alert_topic(void);
int8_t config_server_get_alert_volt(float *alert_volt);
int config_server_get_alert_time(void);
char *config_server_get_alert_mqtt_user(void);
char *config_server_get_alert_mqtt_pass(void);
int8_t config_server_mqtt_en_config(void);
char *config_server_get_mqtt_url(void);
int32_t config_server_get_mqtt_port(void);
char *config_server_get_mqtt_user(void);
char *config_server_get_mmqtt_pass(void);
char *config_server_get_mqtt_canflt(void);
int8_t config_server_mqtt_elm327_log(void);
char *config_server_get_mqtt_tx_topic(void);
char *config_server_get_mqtt_rx_topic(void);
char *config_server_get_mqtt_status_topic(void);

// Master password (6..16 characters)
const char *config_server_get_wic_pass();

// Name of WiCAN adapter (0..14 characters)
const char *config_server_get_wic_name();

// Bit-field of used HSI (config_server_wican_iterface_t)
uint32_t config_server_get_wic_hsi();

// BLE standard specifies a 6-digit passkey (0-9) for standard pairing, i.e BLE relies on a strict 6-digit number.
uint32_t config_server_get_ble_pass();

// Пароль для режима точки доступа (Access Point или soft-AP), в котором ESP32 сам создает собственную Wi-Fi сеть,
// к которой могут подключаться другие устройства.
// В этом режиме устройство само назначает IP-адреса подключающимся клиентам (работает встроенный DHCP-сервер).
// Чип транслирует SSID (имя сети, задаётся в fill_adapter_name()) и этот пароль.
const char *config_server_get_wifi_ap_pass();

// Channel of soft-AP (integer in between 1..13)
int8_t config_server_get_wifi_ap_ch();

// Protocol (TCP_PORT / UDP_PORT) of soft-AP 
int8_t config_server_get_wifi_ap_proto();

// IP address of soft-AP protocol
const char *config_server_get_wifi_ap_ip();

// Port of soft-AP protocol (0..65535)
uint16_t config_server_get_wifi_ap_port();

// SSID of target-AP (1..32 characters or empty)
const char *config_server_get_wifi_sta_ssid();

// Password of target-AP (8..63 characters or empty)
const char *config_server_get_wifi_sta_pass();

// URL of WebSocket server (up to 230 bytes)
const char *config_server_get_ws_addr();

// UART USB baudrate
uint32_t config_server_get_uart_baud();

// GPS COM setting (format 'baud_parity_dataBits_stopBits')
const char* config_server_get_gps_sett();

// GPS initialization command (i.e. '$PMTK220,1000\\r$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\\r')
const char* config_server_get_gps_init();

void config_server_processSetting(const uint8_t* const ciphered_setting, const size_t setting_length, char* const response);
