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
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <lwip/sockets.h>

#include "esp_log_wican.h"
#include "config_server.h"
#include "ble.h"
#include "types.h"

#define WIFI_TAG  __func__

static esp_netif_t* ap_netif;
static esp_netif_t* sta_netif;


static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT     			BIT1
#define WIFI_DISCONNECTED_BIT      	BIT2
#define WIFI_INIT_BIT     		 	BIT3
#define WIFI_CONNECT_IDLE_BIT     	BIT4

char sta_ip[20] = {0};

static const TickType_t connect_delay[] = {1000, 1000, 1000, 1000, 1000,1000};

static void wifi_network_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
//	static int64_t last_try = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
    	ESP_LOGI(WIFI_TAG, "WIFI_EVENT_STA_START");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
    	ESP_LOGI(WIFI_TAG, "WIFI_EVENT_STA_DISCONNECTED");
    	config_server_wifi_connected(0);

    	xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    	xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
    	xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
    	ESP_LOGI(WIFI_TAG, "IP_EVENT_STA_GOT_IP");
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

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
    	ESP_LOGI(WIFI_TAG, "WIFI_EVENT_AP_STACONNECTED");
        ESP_LOGI(WIFI_TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(((wifi_event_ap_staconnected_t*) event_data)->mac), ((wifi_event_ap_staconnected_t*) event_data)->aid);
        if(config_server_get_ble_config())
        {
			ble_disable();
			ESP_LOGW(WIFI_TAG, "disable ble");
        }
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
    	ESP_LOGI(WIFI_TAG, "WIFI_EVENT_AP_STADISCONNECTED");
        ESP_LOGI(WIFI_TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(((wifi_event_ap_stadisconnected_t*) event_data)->mac), ((wifi_event_ap_stadisconnected_t*) event_data)->aid);
        if(config_server_get_ble_config())
        {
			ble_enable();
			ESP_LOGW(WIFI_TAG, "enable ble");
        }
    }
    else if(event_id == WIFI_EVENT_AP_START)
    {
    	ESP_LOGI(WIFI_TAG, "WIFI_EVENT_AP_START");
    }
}

void wifi_network_deinit(void)
{
	xEventGroupWaitBits(s_wifi_event_group,
						WIFI_CONNECT_IDLE_BIT,
						pdFALSE,
						pdFALSE,
						portMAX_DELAY);

	xEventGroupClearBits(s_wifi_event_group, WIFI_INIT_BIT);

	ESP_LOGW(WIFI_TAG, "wifi deinit");

	esp_wifi_disconnect();
    esp_err_t err = esp_wifi_stop();

    esp_event_handler_unregister(IP_EVENT,
    								IP_EVENT_STA_GOT_IP,
									&wifi_network_event_handler);
    esp_event_handler_unregister(WIFI_EVENT,
									ESP_EVENT_ANY_ID,
									&wifi_network_event_handler);
    if (err == ESP_ERR_WIFI_NOT_INIT)
    {
        return;
    }
}
void wifi_network_restart(void)
{
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

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK)
    {
        return;
    }
    esp_wifi_connect();

    ESP_LOGW(WIFI_TAG, "WiFi restarted");
}
bool wifi_network_is_connected(void)
{
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
		ESP_LOGI(WIFI_TAG, "Trying to connect...");
		xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
		esp_wifi_connect();
		xEventGroupWaitBits(s_wifi_event_group,
					WIFI_CONNECT_IDLE_BIT,
		            pdFALSE,
		            pdTRUE,
		            portMAX_DELAY);
		vTaskDelay(pdTICKS_TO_MS(connect_delay[s_retry_num++]));
		s_retry_num %= (sizeof(connect_delay)/sizeof(TickType_t));
	}
}
static TaskHandle_t xwifi_handle = NULL;
void wifi_network_init(char* sta_ssid, char* sta_pass)
{
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


    int8_t channel = config_server_get_ap_ch();
	if(channel == -1)
	{
		channel = 6;
	}
	ESP_LOGI(WIFI_TAG, "AP Channel: %d", channel);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if(config_server_get_ble_config())
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
    static wifi_config_t wifi_config_sta = {
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
    static wifi_config_t wifi_config_ap =
    {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    wifi_config_ap.ap.channel = channel;

    if(config_server_get_wifi_mode() == APSTA_MODE)// || (sta_ssid != 0 && sta_pass != 0))
    {
    	if(sta_ssid == 0 && sta_pass == 0)
    	{
    		strcpy( (char*)wifi_config_sta.sta.ssid, (char*)config_server_get_sta_ssid());
    		strcpy( (char*)wifi_config_sta.sta.password, (char*)config_server_get_sta_pass());
    	}
    	else
    	{
        	strcpy( (char*)wifi_config_sta.sta.ssid, (char*)sta_ssid);
        	strcpy( (char*)wifi_config_sta.sta.password, (char*)sta_pass);
    	}
    	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta) );
    	if(xwifi_handle == NULL)
    	{
    		xTaskCreate(wifi_conn_task, "wifi_conn_task", 1024*3, NULL, 5, &xwifi_handle);
    	}
    }
    else
    {
    	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    }


    fill_adapter_name((char *)wifi_config_ap.ap.ssid);
    strcpy( (char*)wifi_config_ap.ap.password, "239239239"/*(char*)config_server_get_ap_pass()*/);

    esp_netif_ip_info_t ipInfo;
    IP4_ADDR(&ipInfo.ip, 192,168,80,1);
	IP4_ADDR(&ipInfo.gw, 192,168,80,1);
	IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
	esp_netif_dhcps_stop(ap_netif);
	esp_netif_set_ip_info(ap_netif, &ipInfo);
	esp_netif_dhcps_start(ap_netif);

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap));
	ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());
    xEventGroupSetBits(s_wifi_event_group, WIFI_INIT_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECT_IDLE_BIT);
//    esp_wifi_connect();
    ESP_LOGI(WIFI_TAG, "wifi_init finished.");

}
