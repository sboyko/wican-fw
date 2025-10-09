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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_gatt_common_api.h"
#include "types.h"
#include "ble.h"
#include "comm_server.h"
#include "config_server.h"
#include "wifi_network.h"
#include "elm327.h"

/* Attributes State Machine */
enum
{
    IDX_SERVICE_COMM,
    
	IDX_CHAR_COMM_TX,
    IDX_VALUE_COMM_TX, // connects to 'xBle_RX_Queue'

    IDX_CHAR_COMM_RX,
    IDX_VALUE_COMM_RX, // connects to 'xBle_TX_Queue'
    //IDX_CHAR_DESC_COMM_RX,

    IDX_CHAR_BLE_STATUS,
    IDX_VALUE_BLE_STATUS,

    HRS_IDX_NB,
};

#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)

static uint8_t adv_config_done = 0;

#define GATTS_TABLE_TAG "BLE"

#define HEART_PROFILE_NUM                         1
#define HEART_PROFILE_APP_IDX                     0
#define ESP_HEART_RATE_APP_ID                     0x55
#define HEART_RATE_SVC_INST_ID                    0
#define EXT_ADV_HANDLE                            0
#define NUM_EXT_ADV_SET                           1
#define EXT_ADV_DURATION                          0
#define EXT_ADV_MAX_EVENTS                        0

// see https://punchthrough.com/maximizing-ble-throughput-part-3-data-length-extension-dle-2/
//
// MIN: In Bluetooth 4.0, BLE was introduced with a maximum payload of 33 bytes (not including Access Address and CRC fields). 
// Each layer in the protocol stack takes its cut:
//  2 bytes for packet header (type and length),
//  4 bytes for MIC (when encryption is enabled),
//  4 bytes for L2CAP header (channel ID and packet length),
// ATT protocol is left with 23 bytes, which is the default and minimal MTU for ATT protocol.
// With an ATT write request (or notification), 3 bytes are used by command type and attribute ID, _20_ bytes are left for the attribute data.
//
// MAX: Using packet length extension introduced in Bluetooth 4.2:
// Up to 251 bytes at the radio level (255 with MIC), so _242_ bytes available for attribute data.
//
#define BLE_ATT_DATA_MIN_SIZE                     20
#define BLE_ATT_DATA_MAX_SIZE                     240 // intentionally smaller then 242 to prevents Message Integrity Check (MIC) failure (reason = 0x3d)

static uint8_t dev_name[32] = {0};
static uint8_t manufacturer[]="MeatPi";

static uint16_t profile_handle_table[HRS_IDX_NB];
static TaskHandle_t xble_handle = NULL;
//static uint8_t *ext_adv_raw_data;
//static uint8_t ext_adv_raw_data[64] = {
//        0x02, 0x01, 0x06,
//        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd,
//        0x11, 0X09,0x00
//};

//static esp_ble_gap_ext_adv_t ext_adv[1] = {
//    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
//};

static esp_ble_adv_params_t heart_rate_adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t sec_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
//    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x18, 0x0D, 0x00, 0x00,
	0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xe0, 0xfe, 0x00, 0x00,
};

static esp_ble_adv_data_t heart_rate_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = ESP_BLE_CONN_INT_MIN, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = ESP_BLE_CONN_INT_MIN, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// config scan response data
static esp_ble_adv_data_t heart_rate_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(manufacturer),
    .p_manufacturer_data = manufacturer,
};

static uint8_t conn_led = 0;
static EventGroupHandle_t s_ble_event_group = NULL;
#define BLE_CONNECTED_BIT 			BIT0
#define BLE_CONGEST_BIT				BIT1
esp_ble_gap_ext_adv_params_t ext_adv_params_2M = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = 0x20,
    .interval_max = 0x20,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[HEART_PROFILE_NUM] = {
    [HEART_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};

static QueueHandle_t *xBle_TX_Queue = NULL, *xBle_RX_Queue = NULL;
static esp_ble_conn_update_params_t conn_params = {0};
/* Service */
static const uint16_t GATTS_UUID_COMM_SERVICE      = 0xfee0;
static const uint16_t GATTS_UUID_COMM_CHAR_TX      = 0xfee1;
static const uint16_t GATTS_UUID_COMM_CHAR_RX      = 0xfee2;
static const uint16_t GATTS_UUID_BLE_STATUS        = 0xfee3;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_notify_ind          = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
//#define SVC_INST_ID                 0
static const uint8_t no_value = 0;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
// If the client sends a larger MTU size, the ble_max_data_size will be set to
// the minium of BLE_ATT_DATA_MAX_SIZE and (spp_mtu_size - 3).
// Since the default MTU size is 23 this is initially set to 20
static uint16_t ble_max_data_size = BLE_ATT_DATA_MIN_SIZE;
//static bool is_connected = false;

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
	    // Service Declaration
	    [IDX_SERVICE_COMM] =
	    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
			sizeof(GATTS_UUID_COMM_SERVICE), sizeof(GATTS_UUID_COMM_SERVICE), (uint8_t *)&GATTS_UUID_COMM_SERVICE}},

	    /* Characteristic Declaration */
	    [IDX_CHAR_COMM_TX] =
	    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	    	CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

	    /* Characteristic Value */
	    [IDX_VALUE_COMM_TX] =
	    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_UUID_COMM_CHAR_TX, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			BLE_ATT_DATA_MIN_SIZE, 0, NULL}},

	    /* Characteristic Declaration */
	    [IDX_CHAR_COMM_RX] =
	    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
	    	CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify_ind}},

	    /* Characteristic Value */
	    [IDX_VALUE_COMM_RX] =
	    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_UUID_COMM_CHAR_RX, ESP_GATT_PERM_READ,
			BLE_ATT_DATA_MIN_SIZE, 0, NULL}},

	    // /* Client Characteristic Configuration Descriptor */
	    // [IDX_CHAR_DESC_COMM_RX]  =
	    // {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	    //   sizeof(heart_measurement_ccc), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

		/* Characteristic Declaration */
		[IDX_CHAR_BLE_STATUS] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
			CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

		/* Characteristic Value */
		[IDX_VALUE_BLE_STATUS] =
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_UUID_BLE_STATUS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&no_value}},

};



static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
	case ESP_LE_KEY_NONE:
		key_str = "ESP_LE_KEY_NONE";
		break;
	case ESP_LE_KEY_PENC:
		key_str = "ESP_LE_KEY_PENC";
		break;
	case ESP_LE_KEY_PID:
		key_str = "ESP_LE_KEY_PID";
		break;
	case ESP_LE_KEY_PCSRK:
		key_str = "ESP_LE_KEY_PCSRK";
		break;
	case ESP_LE_KEY_PLK:
		key_str = "ESP_LE_KEY_PLK";
		break;
	case ESP_LE_KEY_LLK:
		key_str = "ESP_LE_KEY_LLK";
		break;
	case ESP_LE_KEY_LENC:
		key_str = "ESP_LE_KEY_LENC";
		break;
	case ESP_LE_KEY_LID:
		key_str = "ESP_LE_KEY_LID";
		break;
	case ESP_LE_KEY_LCSRK:
		key_str = "ESP_LE_KEY_LCSRK";
		break;
	default:
		key_str = "INVALID BLE KEY TYPE";
		break;

   }

   return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
	case ESP_LE_AUTH_NO_BOND:
		auth_str = "ESP_LE_AUTH_NO_BOND";
		break;
	case ESP_LE_AUTH_BOND:
		auth_str = "ESP_LE_AUTH_BOND";
		break;
	case ESP_LE_AUTH_REQ_MITM:
		auth_str = "ESP_LE_AUTH_REQ_MITM";
		break;
	case ESP_LE_AUTH_REQ_BOND_MITM:
		auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_ONLY:
		auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
		break;
	case ESP_LE_AUTH_REQ_SC_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
		break;
	case ESP_LE_AUTH_REQ_SC_MITM_BOND:
		auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
		break;
	default:
		auth_str = "INVALID BLE AUTH REQ";
		break;
   }

   return auth_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);

    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices list : %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)&dev_list[i].bond_key, sizeof(esp_ble_bond_key_info_t));
    }
	ESP_LOGI(GATTS_TABLE_TAG, "");

    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

static void logBtDeviceAddress(const char* logPrefix, const esp_bd_addr_t bd_addr)
{
	ESP_LOGI(GATTS_TABLE_TAG, "%s: %08x%04x", logPrefix,
		(bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
		(bd_addr[4] << 8) + bd_addr[5]);
}

static bool ble_tx_ready()
{
	if(ble_connected()) {
		return esp_ble_get_cur_sendable_packets_num(spp_conn_id) > 0;
	}
	return false;
}
static void ble_send(uint8_t* buf, uint8_t buf_len, int charactValueIndex)
{
	const esp_err_t result = esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, profile_handle_table[charactValueIndex], buf_len, buf, false);
	if (result != ESP_OK) {
		ESP_LOGE(GATTS_TABLE_TAG, "esp_ble_gatts_send_indicate() fails: %d", result);
	} else {
#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(GATTS_TABLE_TAG, buf, buf_len, ESP_LOG_INFO);
#endif
	}

	vTaskDelay(pdMS_TO_TICKS(3)); // prevents Message Integrity Check (MIC) failure (reason = 0x3d)
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_event = %d", event);

    switch (event) {
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&heart_rate_adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(GATTS_TABLE_TAG, "The passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
		logBtDeviceAddress("remote BD_ADDR", param->ble_security.auth_cmpl.bd_addr);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        show_bonded_devices();

		esp_gap_conn_params_t conn_params = {0};
		if (esp_ble_get_current_conn_params(param->ble_security.auth_cmpl.bd_addr, &conn_params) == ESP_OK) {
			ESP_LOGI(GATTS_TABLE_TAG, "current_conn_params: interval = %d , latency = %d , timeout = %d", conn_params.interval, conn_params.latency, conn_params.timeout);
		} else {
			ESP_LOGE(GATTS_TABLE_TAG, "get_current_conn_params() fails");
		}

		break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status = %d", param->remove_bond_dev_cmpl.status);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV");
        ESP_LOGI(GATTS_TABLE_TAG, "-----ESP_GAP_BLE_REMOVE_BOND_DEV----");
        esp_log_buffer_hex(GATTS_TABLE_TAG, (void *)param->remove_bond_dev_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "------------------------------------");
        break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            break;
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&heart_rate_adv_config);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&heart_rate_scan_rsp_config);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }else{
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }

        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	xdev_buffer rx_buffer;
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_event = %d", event);

    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name((const char*)dev_name);
            //generate a resolvable random address
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if,
                                      HRS_IDX_NB, HEART_RATE_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
            if(profile_handle_table[IDX_VALUE_BLE_STATUS] == param->read.handle)
            {
				esp_gatt_rsp_t rsp = {0};
            	rsp.attr_value.handle = param->read.handle;
            	rsp.attr_value.len = 1;
            	rsp.attr_value.value[0] = config_server_get_ble_config();
				const esp_err_t result = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, param->reg.status, &rsp);
				if (result != ESP_OK) {
					ESP_LOGE(GATTS_TABLE_TAG, "esp_ble_gatts_send_response(status) fails: %d", result);
				}
            }
            break;
        case ESP_GATTS_WRITE_EVT:
#ifndef NDEBUG		
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT, write value:");
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
#endif

            if(profile_handle_table[IDX_VALUE_COMM_TX] == param->write.handle)
            {
				rx_buffer.dev_channel = DEV_BLE;

				int offset = 0;
				while (offset < param->write.len) {
					rx_buffer.usLen = MIN(param->write.len - offset, sizeof(rx_buffer.ucElement));
					memcpy(rx_buffer.ucElement, param->write.value + offset, rx_buffer.usLen);
					if (xQueueSend(*xBle_RX_Queue, &rx_buffer, pdMS_TO_TICKS(50)) != pdTRUE) {
						ESP_LOGW(GATTS_TABLE_TAG, "BLE rx_queue overflow, tx_queue = %d", uxQueueMessagesWaiting(*xBle_TX_Queue));
						break;
					}
					offset += rx_buffer.usLen;
				}

				if (param->write.need_rsp) {
					esp_gatt_rsp_t rsp = {0};
					rsp.attr_value.handle = param->write.handle;
					const esp_err_t result = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
					if (result != ESP_OK) {
						ESP_LOGE(GATTS_TABLE_TAG, "esp_ble_gatts_send_response(tx) fails: %d", result);
					} else {
						ESP_LOGW(GATTS_TABLE_TAG, "respose sent");
					}
				}

				vTaskDelay(pdMS_TO_TICKS(1));
            }
            else if(profile_handle_table[IDX_VALUE_BLE_STATUS] == param->write.handle)
            {
            	if(param->write.len == 1 && (param->write.value[0] == 0 || param->write.value[0] == 1))
            	{
            		config_server_set_ble_config(param->write.value[0]);
					ble_send(param->write.value, 1, IDX_VALUE_BLE_STATUS); // simple reply
            	}
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            // NOTE: The ESP32 documentation doesn't explain how MTU negotiation works.
            // From the ESP SPP server demo, the characteristic is declared with a size of 512
            // and then this event determines the actual MTU size.
			// From experimentation the esp_ble_gatt_set_local_mtu function is related to this.
			// It seems the value set by esp_ble_gatt_set_local_mtu is sent to the client during
			// the connection. Then if the client supports it, it will send this ESP_GATTS_MTU_EVT
			// with the MTU value the client can support.
            //
            // As noted in above the call to esp_ble_gatt_set_local_mtu is not sent by
            // Car Scanner ELM OBD2 on Android. So it uses the default MTU of 23.
            const int spp_mtu_size = param->mtu.mtu;
            // Each BLE packet has 3 header bytes, so the actual amount of data that
            // can be sent is (MTU - 3).
            //
            // set the max data size to the minimum of (spp_mtu_size - 3) and BLE_ATT_DATA_MAX_SIZE
            ble_max_data_size = BLE_ATT_DATA_MAX_SIZE <= (spp_mtu_size - 3) ? BLE_ATT_DATA_MAX_SIZE : (spp_mtu_size -3);
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, mtu_size = %d, ble_buf_size = %d", spp_mtu_size, ble_max_data_size);
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
        	config_server_stop();
        	wifi_network_deinit();

			logBtDeviceAddress("remote BD_ADDR", param->connect.remote_bda);

			memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
			conn_params.latency = 0;
			conn_params.min_int = ESP_BLE_CONN_INT_MIN;
			conn_params.max_int = ESP_BLE_CONN_INT_MIN;
			conn_params.timeout = ESP_BLE_CONN_SUP_TOUT_MAX / 8; // 4s

			if (esp_ble_gap_update_conn_params(&conn_params) == ESP_OK) {
				ESP_LOGI(GATTS_TABLE_TAG, "update_conn_params: interval_min = %d , interval_max = %d , latency = %d , timeout = %d",
					conn_params.min_int, conn_params.max_int, conn_params.latency, conn_params.timeout);
			} else {
				ESP_LOGE(GATTS_TABLE_TAG, "update(conn_params) fails");
			}

    	    spp_conn_id = param->connect.conn_id;
    	    spp_gatts_if = gatts_if;
    	    //is_connected = true;
    	    xEventGroupSetBits(s_ble_event_group, BLE_CONNECTED_BIT);
    	    gpio_set_level(conn_led, 0);
            /* start security connect with peer device when receive the connect event sent by the master */
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
			// Error 61 (0x3d) means "CONNECTION TERMINATED DUE TO MIC FAILURE". 
			// The Message Integrity Check (MIC) is a 4-bytes extra field added to the BLE packet when encryption is enabled. 
			//
            ESP_LOGW(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);

			clear_perm_commands(true);
            //is_connected = false;
			xEventGroupClearBits(s_ble_event_group, BLE_CONNECTED_BIT);
            gpio_set_level(conn_led, 1);

			wifi_network_restart();
        	config_server_restart();
            /* start advertising again when missing the connect */
            if (esp_ble_gap_start_advertising(&heart_rate_adv_params) != ESP_OK) {
				ESP_LOGE(GATTS_TABLE_TAG, "esp_ble_gap_start_advertising() fails");
			}
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            if (param->congest.congested)
            {
//                can_send_notify = false;
            	xEventGroupSetBits(s_ble_event_group, BLE_CONGEST_BIT);
            }
            else
            {
            	xEventGroupClearBits(s_ble_event_group, BLE_CONGEST_BIT);
//                can_send_notify = true;
//                xSemaphoreGive(gatts_semaphore);
            }
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x",param->add_attr_tab.num_handle);
            if (param->create.status == ESP_GATT_OK){
                if(param->add_attr_tab.num_handle == HRS_IDX_NB) {
                    memcpy(profile_handle_table, param->add_attr_tab.handles,
                    sizeof(profile_handle_table));
                    esp_ble_gatts_start_service(profile_handle_table[IDX_SERVICE_COMM]);
                }else{
                    ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
                }
            }else{
                ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
            }
        break;
    }

        default:
           break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[HEART_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < HEART_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static void ble_task(void *pvParameters)
{
	xdev_buffer tx_buffer;
	uint8_t ble_send_buf[BLE_ATT_DATA_MAX_SIZE];
	uint32_t ble_send_buf_len = 0;

	while(1)
	{
wait_ble:
				xEventGroupWaitBits(s_ble_event_group,
									BLE_CONNECTED_BIT,
									pdFALSE,
									pdFALSE,
									portMAX_DELAY);

				xQueuePeek(*xBle_TX_Queue, &tx_buffer, portMAX_DELAY);

				if (!ble_tx_ready()) {
					vTaskDelay(pdMS_TO_TICKS(1));
					goto wait_ble;
				}

				int free_packet = esp_ble_get_cur_sendable_packets_num(spp_conn_id);

				if(free_packet && !(BLE_CONGEST_BIT & xEventGroupGetBits(s_ble_event_group)))
				{
					while(xQueuePeek(*xBle_TX_Queue, &tx_buffer, pdMS_TO_TICKS(2)))
					{
						// figure out how many packets are needed to send this tx_buffer
						int num_req_packets = ((ble_send_buf_len + tx_buffer.usLen) / ble_max_data_size);
						// Round up. Only part of a packet might be needed and integer math rounds down.
						if((ble_send_buf_len + tx_buffer.usLen) % ble_max_data_size) {
							num_req_packets++;
						}

						if(free_packet < num_req_packets) {
							// We don't have enough free_packets to send this item
							break;
						}

						xQueueReceive(*xBle_TX_Queue, &tx_buffer, 0);

						int tx_buffer_copied = 0;
						while(tx_buffer_copied < tx_buffer.usLen)
						{
							int ble_send_buf_remaining = ble_max_data_size - ble_send_buf_len;
							int tx_buffer_remaining = tx_buffer.usLen - tx_buffer_copied;
							// only copy bytes that will fit in the ble_send_buf
							int copy_len = tx_buffer_remaining >= ble_send_buf_remaining ? ble_send_buf_remaining : tx_buffer_remaining;
							memcpy(ble_send_buf+ble_send_buf_len, tx_buffer.ucElement+tx_buffer_copied, copy_len);
							ble_send_buf_len += copy_len;
							tx_buffer_copied += copy_len;

							// If we have a full ble_send_buf, send it. Otherwise wait to fill it with more bytes,
							// If there are no more bytes to fill it with, then it will be sent at the end.
							if(ble_send_buf_len == ble_max_data_size)
							{
			//					xEventGroupWaitBits(s_ble_event_group,
			//										BLE_CONNECTED_BIT,
			//										pdFALSE,
			//										pdFALSE,
			//										portMAX_DELAY);
			//					ESP_LOG_BUFFER_HEXDUMP(GATTS_TABLE_TAG, ble_send_buf, ble_send_buf_len, ESP_LOG_INFO);
			
								ble_send(ble_send_buf, ble_send_buf_len, IDX_VALUE_COMM_RX);
								ble_send_buf_len = 0;
								free_packet -= 1;

								if(free_packet == 0 && (tx_buffer.usLen - tx_buffer_copied) > 0)
								{
									// We did a computation above to make sure we had a enough
									// free_packets. If we are down to zero and there are still
									// bytes to send, then something went wrong
									ESP_LOGE(GATTS_TABLE_TAG, "Ran out of free_packets too soon");
									break;
								}
							}
						}
					}
					if(free_packet != 0 && ble_send_buf_len != 0)
					{
			//			ESP_LOG_BUFFER_HEXDUMP(GATTS_TABLE_TAG, ble_send_buf, ble_send_buf_len, ESP_LOG_INFO);
						ble_send(ble_send_buf, ble_send_buf_len, IDX_VALUE_COMM_RX);
						ble_send_buf_len = 0;
					}
				}

		//		ESP_LOGI(GATTS_TABLE_TAG, "esp_ble_get_cur_sendable_packets_num 1: %d", esp_ble_get_cur_sendable_packets_num(spp_conn_id));
		//		while(!ble_tx_ready())
		//		{
		//			vTaskDelay(pdMS_TO_TICKS(1));
		//		}
		//		xEventGroupWaitBits(s_ble_event_group,
		//							BLE_CONNECTED_BIT,
		//							pdFALSE,
		//							pdFALSE,
		//							portMAX_DELAY);
		//		ble_send(ble_send_buf, ble_send_buf_len, IDX_VALUE_COMM_RX);
		//		ESP_LOGI(GATTS_TABLE_TAG, "esp_ble_get_cur_sendable_packets_num 2: %d", esp_ble_get_cur_sendable_packets_num(spp_conn_id));
	}
}

bool ble_connected(void)
{
	EventBits_t uxBits;
	if(s_ble_event_group != NULL)
	{
		uxBits = xEventGroupGetBits(s_ble_event_group);

		return (uxBits & BLE_CONNECTED_BIT)?1:0;
	}
	else return 0;
}

static uint32_t ble_pass_key = 0;
void ble_init(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, uint8_t connected_led, int passkey, uint8_t* uid)
{
	esp_err_t ret;

	if(conn_led == 0 && dev_name[0] == 0)
	{
		strcpy((char*)dev_name, (char*)uid);
		conn_led = connected_led;
		ble_pass_key = passkey;
		ESP_LOGW(GATTS_TABLE_TAG, "ble passkey: %lu", ble_pass_key);
	}

	if(xBle_TX_Queue == NULL)
	{
		xBle_TX_Queue = xTXp_Queue;
	}
	if(xBle_RX_Queue == NULL)
	{
		xBle_RX_Queue = xRXp_Queue;
	}

	if(s_ble_event_group == NULL)
	{
		s_ble_event_group = xEventGroupCreate();
	}

	xEventGroupClearBits(s_ble_event_group, BLE_CONNECTED_BIT);
	xEventGroupClearBits(s_ble_event_group, BLE_CONGEST_BIT);
//	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
	esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
	ret = esp_bluedroid_init_with_cfg(&cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gatts_app_register(ESP_HEART_RATE_APP_ID);
	if (ret){
		ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

    // The documentation on how this works is not clear. It is used in the ESP
	// SPP server demo. From experimentation, the value set here is sometimes
	// sent back to us via the ESP_GATTS_MTU_EVT. Only when it is sent back,
	// does it mean the MTU has been increased from the default of 23. From
	// looking at the ESP code its max value is 517.
	//
	// Tested configurations:
	// - Realdash on Android: this value is sent back
	// - ble-serial on MacOS: this value is sent back
	// - Carscanner on Android: this value is not sent back. And based on
	//   experimentation the message length is capped at 20 bytes which is
	//   consistent with the default MTU setting of 23.
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(517);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
	/* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
	esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
	esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;           //set the IO capability to No output No input
	uint8_t key_size = 16;      //the key size should be 7~16 bytes
	uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
	//set static passkey
//	uint32_t passkey = 123456;
	uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
	uint8_t oob_support = ESP_BLE_OOB_DISABLE;

	esp_ble_gap_set_security_param(ESP_BLE_SM_CLEAR_STATIC_PASSKEY, &ble_pass_key, sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &ble_pass_key, sizeof(uint32_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
	/* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
	and the response key means which key you can distribute to the master;
	If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
	and the init key means which key you can distribute to the slave. */
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
	esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

	if(xble_handle == NULL)
	{
		xTaskCreate(ble_task, "ble_task", 1024*4, (void*)AF_INET, 5, &xble_handle);
	}

//    esp_log_level_set(GATTS_TABLE_TAG, ESP_LOG_NONE);
}

void ble_disable(void)
{
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
}
void ble_enable(void)
{
	ble_init(0,0,0,0,0);
//	esp_bluedroid_enable();
}

void ble_disconnect()
{
	if (ble_connected()) {
		esp_ble_gap_disconnect(conn_params.bda);
	}
}
