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
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include  "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"
#include "comm_server.h"
#include "lwip/sockets.h"
#include "driver/twai.h"
#include "ver.h"
#include "types.h"
#include "config_server.h"
#include "realdash.h"
#include "slcan.h"
#include "can.h"
#include "ble.h"
#include "wifi_network.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "gvret.h"
#include "sleep_mode.h"
#include "wc_uart.h"
#include "elm327.h"
#include "mqtt.h"
#include "esp_mac.h"
#include "ftp.h"

#define TAG 		__func__
#define TX_GPIO_NUM             	0
#define RX_GPIO_NUM             	3
#define CONNECTED_LED_GPIO_NUM		8
#define ACTIVE_LED_GPIO_NUM			9
#define BLE_EN_PIN_NUM				5
#define PWR_LED_GPIO_NUM			7
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<CONNECTED_LED_GPIO_NUM) | (1ULL<<ACTIVE_LED_GPIO_NUM) | (1ULL<<PWR_LED_GPIO_NUM) | (1ULL<<CAN_STDBY_GPIO_NUM))
#define BLE_EN_PIN_SEL		(1ULL<<BLE_EN_PIN_NUM)
#define BLE_Enabled()		(!gpio_get_level(BLE_EN_PIN_NUM))

static QueueHandle_t xMsg_Tx_Queue, xMsg_Rx_Queue, xmsg_ws_tx_queue, xmsg_ble_tx_queue, xmsg_uart_tx_queue, xmsg_mqtt_rx_queue;
static QueueHandle_t* host_txQueue = NULL;
static int host_tx_failed_waits = 0;

static uint8_t protocol = SLCAN;
static const TickType_t RESPONSE_TICKS = pdMS_TO_TICKS(10);

uint8_t project_hardware_rev;
int FTP_TASK_FINISH_BIT = BIT2;
EventGroupHandle_t xEventTask;
static uint8_t mqtt_elm327_log_en = 0;

static void log_can_to_mqtt(twai_message_t *frame, uint8_t type)
{
	static mqtt_can_message_t mqtt_msg;

	mqtt_msg.frame.extd = frame->extd;
	mqtt_msg.frame.rtr = frame->rtr;
	mqtt_msg.frame.ss = frame->ss;
	mqtt_msg.frame.self = frame->self;
	mqtt_msg.frame.dlc_non_comp = frame->dlc_non_comp;
	mqtt_msg.frame.identifier = frame->identifier;
	mqtt_msg.frame.data_length_code = frame->data_length_code;

	mqtt_msg.frame.data[0] = frame->data[0];
	mqtt_msg.frame.data[1] = frame->data[1];
	mqtt_msg.frame.data[2] = frame->data[2];
	mqtt_msg.frame.data[3] = frame->data[3];
	mqtt_msg.frame.data[4] = frame->data[4];
	mqtt_msg.frame.data[5] = frame->data[5];
	mqtt_msg.frame.data[6] = frame->data[6];
	mqtt_msg.frame.data[7] = frame->data[7];

	mqtt_msg.type = type;
	xQueueSend( xmsg_mqtt_rx_queue, &mqtt_msg, RESPONSE_TICKS );
}

static void elm327_log_can(twai_message_t *frame, uint8_t type)
{
#ifndef NDEBUG
	char buff[40];
	int offset = elm327_print_canid(buff, frame);

	for (int i = 0; i < TWAI_FRAME_MAX_DLC && i < frame->data_length_code; i++) {
		offset += sprintf(buff + offset, " %02X", frame->data[i]);
	}

	const char* tag = (type == ELM327_CAN_TX ? "can_tx_task" : "can_rx_task");
	ESP_LOGI(tag, "%s", buff);
#endif
}

static void process_led(bool state)
{
	static bool current_state;
	static int64_t last_change;

	if(!can_is_enabled())
	{
		gpio_set_level(ACTIVE_LED_GPIO_NUM, 1);
		current_state = 0;
		last_change = esp_timer_get_time();
	}

	if(esp_timer_get_time() - last_change < 20*1000)
	{
		return;
	}
	if(current_state != state)
	{
		last_change = esp_timer_get_time();
		current_state = state;
	}
	else
	{
		return;
	}
	if(state == 1)
	{
		gpio_set_level(ACTIVE_LED_GPIO_NUM, 0);
	}
	else
	{
		gpio_set_level(ACTIVE_LED_GPIO_NUM, 1);
	}
}

//TODO: make this pretty?
void host_tx_task(char* str, uint32_t len, QueueHandle_t *q)
{
	static xdev_buffer xsend_buffer;

	const int totalLength = (len == 0 ? strlen(str) : len);
	int offset = 0;

	while (offset < totalLength) {
		xsend_buffer.usLen = MIN(totalLength - offset, sizeof(xsend_buffer.ucElement));
		memcpy(xsend_buffer.ucElement, str + offset, xsend_buffer.usLen);
		if (xQueueSend( *q, &xsend_buffer, RESPONSE_TICKS ) != pdTRUE) {
			assert(false);
			break;
		}
		offset += xsend_buffer.usLen;
	
#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG, xsend_buffer.ucElement, xsend_buffer.usLen, ESP_LOG_INFO);
#endif
	}
}

bool fnHasNewData()
{
	return uxQueueMessagesWaiting(xMsg_Rx_Queue) > 0;
}

static void host_rx_task(void *pvParameters)
{
	xdev_buffer ucTCP_RX_Buffer;

	while(1)
	{
		const uint8_t perm_delay = (protocol == OBD_ELM327 ? elm327_perm_delay() : 0);

		if (xQueueReceive(xMsg_Rx_Queue, &ucTCP_RX_Buffer, pdMS_TO_TICKS(perm_delay > 0 ? perm_delay : 15)) != pdTRUE) {
			if(perm_delay > 0 && host_txQueue) {
				elm327_process_perm_cmd(host_txQueue, fnHasNewData);
				host_tx_failed_waits = 0;
			} else {
				if (++host_tx_failed_waits > 400) {
					host_txQueue = NULL;
				}
			}
			continue;
		}
		host_tx_failed_waits = 0;

#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG, ucTCP_RX_Buffer.ucElement, ucTCP_RX_Buffer.usLen, ESP_LOG_INFO);
#endif

		uint8_t* msg_ptr = ucTCP_RX_Buffer.ucElement;
		int temp_len = ucTCP_RX_Buffer.usLen;

		if(ucTCP_RX_Buffer.dev_channel == DEV_WIFI) {
			host_txQueue = &xMsg_Tx_Queue;
		} else if(ucTCP_RX_Buffer.dev_channel == DEV_BLE) {
			host_txQueue = &xmsg_ble_tx_queue;
		} else if(ucTCP_RX_Buffer.dev_channel == DEV_UART) {
			host_txQueue = &xmsg_uart_tx_queue;
		} else { // if(ucTCP_RX_Buffer.dev_channel == DEV_WIFI_WS) {
			host_txQueue = &xmsg_ws_tx_queue;
		}

		if(protocol == OBD_ELM327)
		{
			if (ucTCP_RX_Buffer.dev_channel == DEV_WIFI_WS && config_server_ws_connected()) {
				twai_message_t tx_msg;
				slcan_parse_str(msg_ptr, temp_len, &tx_msg, host_txQueue);
			} else {
				elm327_process_cmd(msg_ptr, temp_len, host_txQueue, fnHasNewData);
			}
		}
		else if(protocol == SLCAN)
		{
			twai_message_t tx_msg;
			slcan_parse_str(msg_ptr, temp_len, &tx_msg, host_txQueue);
		}
		else if(protocol == REALDASH)
		{
			twai_message_t tx_msg;
			if(real_dash_parse_66(&tx_msg, msg_ptr) == 0) {
				real_dash_parse_44(&tx_msg, msg_ptr, temp_len);
			}

			tx_msg.self = 0;
			can_send(&tx_msg, portMAX_DELAY);
		}
		else if(protocol == SAVVYCAN)
		{
			twai_message_t tx_msg;
			gvret_parse(msg_ptr, temp_len, &tx_msg, &xMsg_Tx_Queue);
		}
	}
}

static void can_rx_task(void *pvParameters)
{
	xdev_buffer ucTCP_TX_Buffer;
    twai_message_t rx_msg;
	mqtt_can_message_t mqtt_rx_msg;

	while(1)
	{
        if(can_receive(&rx_msg, pdMS_TO_TICKS(2)) == ESP_OK)
        {
			// ESP_LOGI(TAG, "%08X%c  %02X %02X %02X %02X %02X %02X %02X %02X",
			// 	(unsigned int)(rx_msg.identifier&TWAI_EXTD_ID_MASK), (rx_msg.extd ? 'x' : ' '),
			// 	(unsigned int)rx_msg.data[0], (unsigned int)rx_msg.data[1], (unsigned int)rx_msg.data[2],
			// 	(unsigned int)rx_msg.data[3], (unsigned int)rx_msg.data[4], (unsigned int)rx_msg.data[5],
			// 	(unsigned int)rx_msg.data[6], (unsigned int)rx_msg.data[7]);

        	process_led(1);

			QueueHandle_t* const txQueue = host_txQueue;
			if(txQueue)
			{
				ucTCP_TX_Buffer.ucElement[0] = 0;
				ucTCP_TX_Buffer.usLen = 0;

				if(protocol == OBD_ELM327)
				{
					if (txQueue == &xmsg_ws_tx_queue && config_server_ws_connected()) {
						ucTCP_TX_Buffer.usLen = slcan_parse_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
					} else {
						ucTCP_TX_Buffer.usLen = elm327_process_can_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
					}
				}
				else if(protocol == SLCAN)
				{
					ucTCP_TX_Buffer.usLen = slcan_parse_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
				}
				else if(protocol == REALDASH)
				{
					ucTCP_TX_Buffer.usLen = real_dash_set_66(&rx_msg, ucTCP_TX_Buffer.ucElement);
				}
				else if(protocol == SAVVYCAN)
				{
					ucTCP_TX_Buffer.usLen = gvret_parse_can_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
				}


				if(ucTCP_TX_Buffer.usLen > 0)
				{
					host_tx_failed_waits = 0;
					if (xQueueSend( *txQueue, &ucTCP_TX_Buffer, RESPONSE_TICKS ) != pdTRUE) {
						assert(false);
					}
				}
			}

			if(mqtt_connected() && mqtt_elm327_log_en == 0)
			{
				mqtt_rx_msg.frame.extd = rx_msg.extd;
				mqtt_rx_msg.frame.rtr = rx_msg.rtr;
				mqtt_rx_msg.frame.ss = rx_msg.ss;
				mqtt_rx_msg.frame.self = rx_msg.self;
				mqtt_rx_msg.frame.dlc_non_comp = rx_msg.dlc_non_comp;
				mqtt_rx_msg.frame.identifier = rx_msg.identifier;
				mqtt_rx_msg.frame.data_length_code = rx_msg.data_length_code;

				mqtt_rx_msg.frame.data[0] = rx_msg.data[0];
				mqtt_rx_msg.frame.data[1] = rx_msg.data[1];
				mqtt_rx_msg.frame.data[2] = rx_msg.data[2];
				mqtt_rx_msg.frame.data[3] = rx_msg.data[3];
				mqtt_rx_msg.frame.data[4] = rx_msg.data[4];
				mqtt_rx_msg.frame.data[5] = rx_msg.data[5];
				mqtt_rx_msg.frame.data[6] = rx_msg.data[6];
				mqtt_rx_msg.frame.data[7] = rx_msg.data[7];

				mqtt_rx_msg.type = MQTT_CAN;
				xQueueSend( xmsg_mqtt_rx_queue, &mqtt_rx_msg, RESPONSE_TICKS );
			}
        }
		else
		{
	        process_led(0);
		}
	}
}

static uint8_t derived_mac_addr[6] = {0};
static uint8_t uid[33];
static uint8_t ble_uid[33];
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

	gpio_set_level(CONNECTED_LED_GPIO_NUM, 1);
	gpio_set_level(ACTIVE_LED_GPIO_NUM, 1);

    xMsg_Rx_Queue = xQueueCreate(32, sizeof( xdev_buffer) );
    xMsg_Tx_Queue = xQueueCreate(32, sizeof( xdev_buffer) );
    xmsg_ws_tx_queue = xQueueCreate(32, sizeof( xdev_buffer) );

	esp_ota_mark_app_valid_cancel_rollback();

    ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_SOFTAP));
    sprintf((char *)ble_uid,"WiC_%02x%02x%02x%02x%02x%02x",
            derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
            derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
    sprintf((char *)uid,"%02x%02x%02x%02x%02x%02x",
            derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
            derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
	
	config_server_start(&xmsg_ws_tx_queue, &xMsg_Rx_Queue, CONNECTED_LED_GPIO_NUM, (char*)&uid[0]);
	slcan_init(&host_tx_task);

	int8_t can_datarate = config_server_get_can_rate();
	(can_datarate != -1) ? can_init(can_datarate):can_init(CAN_500K);

	if(can_datarate != -1)
	{
		can_set_bitrate(can_datarate);
	}
	else
	{
		ESP_LOGE(TAG, "error going to default CAN_500K");
		can_set_bitrate(CAN_500K);
	}

	if(config_server_get_can_mode() == CAN_NORMAL)
	{
		can_set_silent(0);
	}
	else
	{
		can_set_silent(1);
	}

	static twai_filter_config_t allPassFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
	can_set_filter(allPassFilter.acceptance_code);
	can_set_mask(allPassFilter.acceptance_mask);

	protocol = config_server_protocol();
//	protocol = OBD_ELM327;

	if(protocol == REALDASH)
	{
//		int can_datarate = config_server_get_can_rate();
		if(can_datarate != -1)
		{
			can_set_bitrate(can_datarate);
		}
		else
		{
			ESP_LOGE(TAG, "error going to default CAN_500K");
			can_set_bitrate(CAN_500K);
		}

		can_enable();
	}
	else if(protocol == SAVVYCAN)
	{
		gvret_init(&host_tx_task);
		can_enable();
	}
	else if(protocol == OBD_ELM327)
	{
//		can_init(CAN_500K);
		can_set_bitrate(can_datarate);
		can_enable();
		if(config_server_mqtt_en_config() && config_server_mqtt_elm327_log())
		{
			mqtt_elm327_log_en = config_server_mqtt_elm327_log();
			elm327_init(&host_tx_task, log_can_to_mqtt);
		}
		else
		{
			elm327_init(&host_tx_task, elm327_log_can);
		}
	}

	if(config_server_mqtt_en_config())
	{
		can_set_bitrate(can_datarate);
		xmsg_mqtt_rx_queue = xQueueCreate(32, sizeof(mqtt_can_message_t) );
		can_enable();
		mqtt_init((char*)&uid[0], CONNECTED_LED_GPIO_NUM, &xmsg_mqtt_rx_queue);
	}
//	else if(protocol == MQTT)
//	{
//		xmsg_mqtt_rx_queue = xQueueCreate(32, sizeof( twai_message_t) );
//		can_init(CAN_500K);
//		can_enable();
//
//		mqtt_init((char*)&uid[0], CONNECTED_LED_GPIO_NUM, &xmsg_mqtt_rx_queue);
//	}


	wifi_network_init(NULL, NULL);
	int32_t port = config_server_get_port();

	if(port == -1)
	{
		port = 3333;
	}
	if(config_server_get_port_type() == UDP_PORT)
	{
		tcp_server_init(port, &xMsg_Tx_Queue, &xMsg_Rx_Queue, CONNECTED_LED_GPIO_NUM, 1);
	}
	else
	{
		tcp_server_init(port, &xMsg_Tx_Queue, &xMsg_Rx_Queue, CONNECTED_LED_GPIO_NUM, 0);
	}

    if(config_server_get_ble_config())
    {
    	int pass = config_server_ble_pass();
    	xmsg_ble_tx_queue = xQueueCreate(64, sizeof( xdev_buffer) );
    	ble_init(&xmsg_ble_tx_queue, &xMsg_Rx_Queue, CONNECTED_LED_GPIO_NUM, pass, &ble_uid[0]);
    }



    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
    {
        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
        ESP_LOGI(TAG, "Project Name: %s", running_app_info.project_name);

        if(strstr(running_app_info.project_name, "usb") != 0)
        {
        	project_hardware_rev = WICAN_USB_V100;
        	ESP_LOGI(TAG, "project_hardware_rev: USB");

			xmsg_uart_tx_queue = xQueueCreate(64, sizeof( xdev_buffer) );
       		wc_uart_init(&xmsg_uart_tx_queue, &xMsg_Rx_Queue, CONNECTED_LED_GPIO_NUM);
        }
        else
        {
        	ESP_LOGI(TAG, "project_hardware_rev: OBD");
            if(strstr(running_app_info.project_name, "hv210") != 0)
            {
            	project_hardware_rev = WICAN_V210;
            }
            else
            {
            	project_hardware_rev = WICAN_V300;
            }
        }
    }

    xTaskCreate(can_rx_task, "can_rx_task", 1024*3, (void*)AF_INET, 5, NULL);
    xTaskCreate(host_rx_task, "host_rx_task", 1024*3, (void*)AF_INET, 5, NULL);

    if(project_hardware_rev != WICAN_V210)
    {
		if(config_server_get_sleep_config())
		{
			float sleep_voltage = 0;

			if(config_server_get_sleep_volt(&sleep_voltage) != -1)
			{
				sleep_mode_init(1, sleep_voltage);
			}
			else
			{
				sleep_mode_init(0, 13.1f);
			}
		}
		else
		{
			sleep_mode_init(0, 13.1f);
		}
    }
    else
    {
    	sleep_mode_init(0, 13.1f);
    }

    gpio_set_level(PWR_LED_GPIO_NUM, 1);
    

	// xEventTask = xEventGroupCreate();
	// xTaskCreate(ftp_task, "FTP", 1024*6, NULL, 2, NULL);
	// xEventGroupWaitBits( xEventTask,
	// FTP_TASK_FINISH_BIT, /* The bits within the event group to wait for. */
	// pdTRUE, /* BIT_0 should be cleared before returning. */
	// pdFALSE, /* Don't wait for both bits, either bit will do. */
	// portMAX_DELAY);/* Wait forever. */  
	esp_log_level_set("*", ESP_LOG_NONE);
}
