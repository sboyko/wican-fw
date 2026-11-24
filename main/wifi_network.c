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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <lwip/sockets.h>

#include "esp_log_wican.h"
#include "config_server.h"
#include "wican_settings.h"
#include "ble.h"
#include "types.h"

#define TAG  __func__

static esp_netif_t* ap_netif;
static esp_netif_t* sta_netif;


static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT     			BIT1
#define WIFI_DISCONNECTED_BIT      	BIT2
#define WIFI_INIT_BIT     		 	BIT3
#define WIFI_CONNECT_IDLE_BIT     	BIT4

static TaskHandle_t xwifi_handle = NULL;
static const TickType_t connect_delay[] = {1000, 1000, 1000, 1000, 1000,1000};

static void wifi_network_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
//	static int64_t last_try = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
    	ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        esp_wifi_connect(); // Impacts STA and APSTA modes only
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
    	ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
    	config_server_wifi_connected(0);

    	xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    	xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
    	xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
    	ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        char sta_ip[20] = {0};
        sprintf(sta_ip, "%d.%d.%d.%d", IP2STR(&event->ip_info.ip));

        config_server_set_sta_ip(sta_ip);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
        config_server_wifi_connected(1);
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
    	ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(((wifi_event_ap_staconnected_t*) event_data)->mac), ((wifi_event_ap_staconnected_t*) event_data)->aid);
        if((settings_get_int(CFG_WIC_HSI) & WIC_HSI_BLE) == WIC_HSI_BLE)
        {
			ble_disable();
			ESP_LOGW(TAG, "disable ble");
        }
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
    	ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(((wifi_event_ap_stadisconnected_t*) event_data)->mac), ((wifi_event_ap_stadisconnected_t*) event_data)->aid);
        if((settings_get_int(CFG_WIC_HSI) & WIC_HSI_BLE) == WIC_HSI_BLE)
        {
			ble_enable();
			ESP_LOGW(TAG, "enable ble");
        }
    }
    else if(event_id == WIFI_EVENT_AP_START)
    {
    	ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
    }
}

void wifi_network_deinit()
{
    if ((settings_get_int(CFG_WIC_HSI) & (WIC_HSI_WIFI | WIC_HSI_WEB)) == 0) {
        return;
    }

	xEventGroupWaitBits(s_wifi_event_group,
						WIFI_CONNECT_IDLE_BIT,
						pdFALSE,
						pdFALSE,
						portMAX_DELAY);

	xEventGroupClearBits(s_wifi_event_group, WIFI_INIT_BIT);

	ESP_LOGW(TAG, "wifi deinit");

	esp_wifi_disconnect(); // Disconnect WiFi station from the AP.
    esp_wifi_stop(); // Stop WiFi, impact all modes (AP/STA/APSTA).

    esp_event_handler_unregister(IP_EVENT,
    								IP_EVENT_STA_GOT_IP,
									&wifi_network_event_handler);
    esp_event_handler_unregister(WIFI_EVENT,
									ESP_EVENT_ANY_ID,
									&wifi_network_event_handler);
}

void wifi_network_restart()
{
    const uint32_t wic_hsi = settings_get_int(CFG_WIC_HSI);
    if ((wic_hsi & (WIC_HSI_WIFI | WIC_HSI_WEB)) == 0) {
        return;
    }

    xEventGroupSetBits(s_wifi_event_group, WIFI_INIT_BIT);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_network_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_network_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    esp_err_t err = esp_wifi_start(); // Start WiFi, impact all modes (AP/STA/APSTA).
    ESP_RETURN_VOID_ON_ERROR(err, TAG, "failed: %x", err);

    if ((wic_hsi & WIC_HSI_WEB) == WIC_HSI_WEB) {
        err = esp_wifi_connect(); // Impacts STA and APSTA modes only
        ESP_RETURN_VOID_ON_ERROR(err, TAG, "failed: %x", err);
    }

    ESP_LOGW(TAG, "WiFi restarted");
}

bool wifi_network_is_connected()
{
    if ((settings_get_int(CFG_WIC_HSI) & (WIC_HSI_WIFI | WIC_HSI_WEB)) == 0) {
        return 0;
    }

	EventBits_t ux_bits;
	if(s_wifi_event_group != NULL)
	{
		ux_bits = xEventGroupGetBits(s_wifi_event_group);

		return (ux_bits & WIFI_CONNECTED_BIT);
	}
	else return 0;
}

static void wifi_conn_task(void *pvParameters)
{
	while(1)
	{
		xEventGroupWaitBits(s_wifi_event_group,
					WIFI_INIT_BIT | WIFI_DISCONNECTED_BIT,
		            pdFALSE,
		            pdTRUE,
		            portMAX_DELAY);
		ESP_LOGI(TAG, "Trying to connect...");
		xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);

        esp_wifi_connect(); // Impacts STA and APSTA modes only

        xEventGroupWaitBits(s_wifi_event_group,
					WIFI_CONNECT_IDLE_BIT,
		            pdFALSE,
		            pdTRUE,
		            portMAX_DELAY);

		vTaskDelay(pdTICKS_TO_MS(connect_delay[s_retry_num++]));
		s_retry_num %= (sizeof(connect_delay)/sizeof(TickType_t));
	}
}

void wifi_network_init()
{
    const uint32_t wic_hsi = settings_get_int(CFG_WIC_HSI);
    if ((wic_hsi & (WIC_HSI_WIFI | WIC_HSI_WEB)) == 0) {
        return;
    }

	if(s_wifi_event_group == NULL)
	{
		s_wifi_event_group = xEventGroupCreate();
		ap_netif = esp_netif_create_default_wifi_ap();

		sta_netif = esp_netif_create_default_wifi_sta();
	}

	if(xEventGroupGetBits(s_wifi_event_group) & WIFI_INIT_BIT)
	{
		return;
	}
//	xEventGroupClearBits(s_wifi_event_group, WIFI_INIT_BIT);


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if((wic_hsi & WIC_HSI_BLE) == WIC_HSI_BLE)
    {
    	ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MIN_MODEM) );
    }
    else
	{
		ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_NONE) );
	}

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_network_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_network_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    if((wic_hsi & WIC_HSI_WEB) == WIC_HSI_WEB)
    {
        wifi_config_t wifi_config_sta = {
            .sta = {
                .ssid = "",
                .password = "",
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                * However these modes are deprecated and not advisable to be used. Incase your Access point
                * doesn't support WPA2, these mode can be enabled by commenting below line */
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .rm_enabled = 1,
                .btm_enabled = 1,
                .scan_method = WIFI_ALL_CHANNEL_SCAN,
                .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                .bssid_set = false,

                .pmf_cfg = {
                    .capable = true,
                    .required = false
                },
            },
        };

		settings_get_string(CFG_WIFI_STA_SSID, (char*)wifi_config_sta.sta.ssid, sizeof(wifi_config_sta.sta.ssid));
		settings_get_string(CFG_WIFI_STA_PASS, (char*)wifi_config_sta.sta.password, sizeof(wifi_config_sta.sta.password));

		if (wifi_config_sta.sta.ssid[0] == 0 || wifi_config_sta.sta.password[0] == 0) {
			// "WiCANabitabit"
			static const uint8_t rand1[13] = { 0xA3, 0x7F, 0x21, 0xDE, 0x9C, 0x44, 0xB8, 0x06, 0xE5, 0x37, 0xF1, 0x5A, 0xC2 };
			memcpy(wifi_config_sta.sta.ssid, rand1, sizeof(rand1));
			const uint8_t key1[13]  = { 0xF4, 0x16, 0x62, 0x9F, 0xD2, 0x25, 0xDA, 0x6F, 0x91, 0x56, 0x93, 0x33, 0xB6 };
			for (size_t i = 0; i < sizeof(key1); i++) {
				wifi_config_sta.sta.ssid[i] ^= key1[i];
			}

			// "airabit123"
			static const uint8_t rand2[10] = { 0x4D, 0x8E, 0x1B, 0xF3, 0x67, 0xA2, 0x09, 0xC4, 0x55, 0xE8 };
			memcpy(wifi_config_sta.sta.password, rand2, sizeof(rand2));
			const uint8_t key2[10] = { 0x2C, 0xE7, 0x69, 0x92, 0x05, 0xCB, 0x7D, 0xF5, 0x67, 0xDB };
			for (size_t i = 0; i < sizeof(key2); i++) {
				wifi_config_sta.sta.password[i] ^= key2[i];
			}
		}

        ESP_ERROR_CHECK(esp_wifi_set_mode((wic_hsi & WIC_HSI_WIFI) == WIC_HSI_WIFI ? WIFI_MODE_APSTA : WIFI_MODE_STA));
    	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta) );
    	if(xwifi_handle == NULL)
    	{
    		xTaskCreate(wifi_conn_task, "wifi_conn_task", 1024*3, NULL, 5, &xwifi_handle);
    	}
    }

    if ((wic_hsi & WIC_HSI_WIFI) == WIC_HSI_WIFI)
    {
        const int8_t channel = settings_get_int(CFG_WIFI_AP_CH);
        ESP_LOGI(TAG, "AP Channel: %d", channel);

        wifi_config_t wifi_config_ap = {
            .ap = {
                .max_connection = 4,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK
            },
        };
        wifi_config_ap.ap.channel = channel;

        fill_adapter_name((char *)wifi_config_ap.ap.ssid);
		ESP_ERROR_CHECK(settings_get_string(CFG_WIFI_AP_PASS, (char*)wifi_config_ap.ap.password, sizeof(wifi_config_ap.ap.password)));

    	ESP_ERROR_CHECK(esp_wifi_set_mode((wic_hsi & WIC_HSI_WEB) == WIC_HSI_WEB ? WIFI_MODE_APSTA : WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
        ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));

        esp_netif_ip_info_t ipInfo = {
			.netmask = {
				.addr = ipaddr_addr("255.255.255.0"),
			},
		};
		if (settings_get_string(CFG_WIFI_AP_IP, (char*)wifi_config_ap.ap.password, sizeof(wifi_config_ap.ap.password)) == ESP_OK) {
			ipInfo.ip.addr = ipaddr_addr((const char*)wifi_config_ap.ap.password);
			ipInfo.gw.addr = ipaddr_addr((const char*)wifi_config_ap.ap.password);
		}
        esp_netif_dhcps_stop(ap_netif);
        esp_netif_set_ip_info(ap_netif, &ipInfo);
        esp_netif_dhcps_start(ap_netif);
    }

    ESP_ERROR_CHECK(esp_wifi_start()); // Start WiFi, impact all modes (AP/STA/APSTA).
    xEventGroupSetBits(s_wifi_event_group, WIFI_INIT_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
//    esp_wifi_connect();
    ESP_LOGI(TAG, "wifi_init finished.");

}
