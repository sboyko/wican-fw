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

#include "gps_common.h"
#include "gps_nmea.h"

#include <stdatomic.h>


#define TAG  __func__

#define PWR_LED_GPIO_NUM            7  // blue (HL1 (USB/W) - 0: off / 1: on)
#define CAN_TR_LED_GPIO_NUM         8  // green (HL5 (CAN TR) - 0: on / 1: off)
#define CAN_RX_LED_GPIO_NUM         9  // yellow (HL6 (CAN RX) - 0: on / 1: off)
#define KLINE_GPS_LED_GPIO_NUM      10 // (HL2 (K-Line) / HL3 (GPS) - 0: KLine / 1: GPS)

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<CAN_TR_LED_GPIO_NUM) | (1ULL<<CAN_RX_LED_GPIO_NUM) | (1ULL<<PWR_LED_GPIO_NUM) | (1ULL<<CAN_STDBY_GPIO_NUM) | (1ULL<<KLINE_GPS_LED_GPIO_NUM))

static QueueHandle_t xMsg_Tx_Queue, xmsg_ws_tx_queue, xmsg_ble_tx_queue, xmsg_uart_tx_queue;
static QueueHandle_t xMsg_Rx_Queue, xmsg_mqtt_rx_queue, xmsg_uart_rx_queue;

static QueueHandle_t* host_txQueue = NULL;
static int64_t host_tx_last_time = 0; // last time we successfully scheduled send message to host
static int64_t host_rx_last_time = 0; // last time we successfully received message from host (or fake-received when bulk-mode)
static const TickType_t RESPONSE_TICKS = pdMS_TO_TICKS(20);

static uint8_t protocol = SLCAN;

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

static void set_can_rx_led(bool state)
{
	static bool current_state = false; // CAN RX 'off' on start
	static int64_t next_check = 0;

	if (!can_is_enabled()) {
		if (!current_state)  {
			return;
		}
	} else if (next_check > esp_timer_get_time()) {
		return;
	}

	const bool new_state = (can_is_enabled() ? state : false);
	if (current_state == new_state) {
		return;
	}

	current_state = new_state;
	gpio_set_level(CAN_RX_LED_GPIO_NUM, current_state ? 0 : 1);
	next_check = esp_timer_get_time() +  20*1000;
}

static void set_power_led(bool state)
{
	static bool current_state = true; // WiCAN connected 'on' on start
	static int64_t next_check = 0;

	if (next_check > esp_timer_get_time()) {
		return;
	}
	if (current_state == state) {
		return;
	}

	current_state = state;
	gpio_set_level(PWR_LED_GPIO_NUM, current_state ? 1 : 0);
	next_check = esp_timer_get_time() +  20*1000;
}

static bool hasHostTxConnection()
{
	return esp_timer_get_time() - host_rx_last_time < 6*1000*1000; // doubled PING time
}

//TODO: make this pretty?
bool host_tx_task(const char* str, uint32_t len, QueueHandle_t *q)
{
	xdev_buffer xsend_buffer;

	const int totalLength = (len == 0 ? strlen(str) : len);
	int offset = 0;

	if (!hasHostTxConnection()) { // addition guard
		return false;
	}

	while (offset < totalLength) {
		xsend_buffer.usLen = MIN(totalLength - offset, sizeof(xsend_buffer.ucElement));
		memcpy(xsend_buffer.ucElement, str + offset, xsend_buffer.usLen);
		if (xQueueSend( *q, &xsend_buffer, RESPONSE_TICKS ) != pdTRUE) {
			ESP_LOGE(TAG, "xQueueSend() fails");
			//assert(false);
			return false;
		}
		offset += xsend_buffer.usLen;
	
#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG, xsend_buffer.ucElement, xsend_buffer.usLen, ESP_LOG_INFO);
#endif
	}

	host_tx_last_time = esp_timer_get_time();

	return true;
}

int hostHasNewData()
{
	return uxQueueMessagesWaiting(xMsg_Rx_Queue);
}

static QueueHandle_t* getHostTxQueue()
{
	return atomic_load(&host_txQueue);
}

static void setHostTxQueue(QueueHandle_t* const queue)
{
	QueueHandle_t* const old_txQueue = atomic_exchange(&host_txQueue, queue);

	if (queue == NULL && old_txQueue != NULL) {
		ESP_LOGW(TAG, "Set to NULL");

		vTaskDelay(pdMS_TO_TICKS(10));
		xQueueReset(*old_txQueue);
	}

	if (config_server_get_ble_config()) {
		if (old_txQueue != queue) {
			if (old_txQueue == &xmsg_uart_tx_queue && queue == NULL) {
				ble_enable();
				ESP_LOGW(TAG, "enable ble");
			} else if (queue == &xmsg_uart_tx_queue) {
				ble_disable();
				ESP_LOGW(TAG, "disable ble");
			}
		}
	}
}

static void host_rx_task(void *pvParameters)
{
	xdev_buffer rx_buffer;
	host_rx_last_time = esp_timer_get_time();

	while(1)
	{
		const uint8_t perm_delay = (protocol == OBD_ELM327 ? elm327_perm_delay() : 0);

		/**
		 * modes:
		 * - normal (recv-send)
		 * - perm_commands (send), ping_3s (for recv)
		 * - monitor (send), ping_3s (for recv)
		 * - bulk (recv), idle_cmd (for missed recv)
		 */
		if (xQueueReceive(xMsg_Rx_Queue, &rx_buffer, pdMS_TO_TICKS(perm_delay > 0 ? perm_delay : 15)) != pdTRUE) {
			if (!hasHostTxConnection()) { // no 'ping' for over 3s*2 (looks like AKM was terminated)
				if (getHostTxQueue()) {
					setHostTxQueue(NULL);
					// esp_restart();
				} else {
					if (config_server_get_ble_config()) {
						ble_restart_advertising();
					}
				}

				host_rx_last_time = esp_timer_get_time(); // prevents spamming
			}

			bool hasCommand = false;
			if (getHostTxQueue()) {
				if (perm_delay > 0) {
					hasCommand = elm327_process_perm_cmd(&rx_buffer);
				} else if (esp_timer_get_time() - host_rx_last_time > 2*1000*1000) {
					if (elm327_process_idle_cmd(&rx_buffer)) { // send idle_cmd in case 'bulk' mode in on
						hasCommand = true;
						host_rx_last_time = esp_timer_get_time();
					}
				}
			}
			if (!hasCommand) {
				continue;
			}
		} else {
			host_rx_last_time = esp_timer_get_time();
		}

#ifndef NDEBUG
		ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buffer.ucElement, rx_buffer.usLen, ESP_LOG_INFO);
#endif

		uint8_t* msg_ptr = rx_buffer.ucElement;
		int temp_len = rx_buffer.usLen;

		if(rx_buffer.dev_channel == DEV_WIFI) {
			setHostTxQueue(&xMsg_Tx_Queue);
		} else if(rx_buffer.dev_channel == DEV_BLE) {
			setHostTxQueue(&xmsg_ble_tx_queue);
		} else if(rx_buffer.dev_channel == DEV_UART) {
			setHostTxQueue(&xmsg_uart_tx_queue);
		} else { // if(rx_buffer.dev_channel == DEV_WIFI_WS) {
			setHostTxQueue(&xmsg_ws_tx_queue);
		}

		if(protocol == OBD_ELM327)
		{
			if (rx_buffer.dev_channel == DEV_WIFI_WS && config_server_ws_connected()) {
				twai_message_t tx_msg;
				slcan_parse_str(msg_ptr, temp_len, &tx_msg, host_txQueue);
			} else {
				elm327_process_cmd(msg_ptr, temp_len, host_txQueue, hostHasNewData);
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

	while(true)
	{
		if (!can_is_enabled()) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		if(can_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK)
		{
			// ESP_LOGI(TAG, "%08X%c  %02X %02X %02X %02X %02X %02X %02X %02X",
			// 	(unsigned int)(rx_msg.identifier&TWAI_EXTD_ID_MASK), (rx_msg.extd ? 'x' : ' '),
			// 	(unsigned int)rx_msg.data[0], (unsigned int)rx_msg.data[1], (unsigned int)rx_msg.data[2],
			// 	(unsigned int)rx_msg.data[3], (unsigned int)rx_msg.data[4], (unsigned int)rx_msg.data[5],
			// 	(unsigned int)rx_msg.data[6], (unsigned int)rx_msg.data[7]);

			set_can_rx_led(true);

			ucTCP_TX_Buffer.ucElement[0] = 0;
			ucTCP_TX_Buffer.usLen = 0;

			if(protocol == OBD_ELM327)
			{
				// if (txQueue == &xmsg_ws_tx_queue && config_server_ws_connected()) {
				// 	ucTCP_TX_Buffer.usLen = slcan_parse_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
				// } else {
					ucTCP_TX_Buffer.usLen = elm327_process_can_frame(ucTCP_TX_Buffer.ucElement, &rx_msg);
				// }
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

			QueueHandle_t* const txQueue = getHostTxQueue();
			if(txQueue)
			{
				if(ucTCP_TX_Buffer.usLen > 0)
				{
					if (!host_tx_task((const char*)ucTCP_TX_Buffer.ucElement, ucTCP_TX_Buffer.usLen, txQueue)) {
						set_can_rx_led(false);
					}
				}
			}
			else // no activity on WiCAN
			{
				can_disable();
				gpio_set_level(CAN_TR_LED_GPIO_NUM, 1); // CAN TR 'off'
				set_can_rx_led(false);
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
			set_can_rx_led(false);
		}
	}
}

static void nmea_rx_task(void *pvParameters)
{
	// commands (Trema GPS модуль ATGM336H):
	// $PCAS02,1000  // gps_select_updaterate(1) - УСТАНОВКА ЧАСТОТЫ ОБНОВЛЕНИЯ ДАННЫХ (1 раз в секунду)
	// $PCAS03,0,0,0,0,1,0,0,0,0,0,0,,0,0  // gps_select_composition(NMEA_RMC) - УСТАНОВКА СОСТАВА ПАКЕТА NMEA
	// $PCAS10,0  // gps_reset(0) (0 - HOT_START, 1 - WARM_START, 2 - COLD_START, 3 - FACTORY_SET)
	char gps_init_commands[] = "\
$PCAS02,1000\r\
$PCAS03,0,0,0,0,1,0,0,0,0,0,0,,0,0\r\
";
	const int gps_uart_baudRate = 9600;
	const int gps_uart_parity = 0;
	const int gps_uart_dataBits = 8;
	const int gps_uart_stopBits = 0;

	xdev_buffer waste_buffer;

	while(true)
	{
		vTaskDelay(pdMS_TO_TICKS(50));
		gps_wait_enabled(portMAX_DELAY);

		QueueHandle_t* const txQueue = getHostTxQueue();
		if (txQueue == NULL) { // no activity on WiCAN
			gps_set_enabled(false); // pause 'nmea_rx_task'
			wc_gps_update(false); // switch to USB (if not K-Line of course)
			continue;
		}

		if (txQueue == &xmsg_uart_tx_queue // protects against simulteneous use of USB and GPS
				|| wc_kline_active() // don't mess with K-Line
				|| elm327_process_idle_cmd(&waste_buffer) // detects that 'bulk' mode in on
				) {
			continue;
		}

		if (!wc_gps_active()) {
			if (!wc_gps_set_baudrate(gps_uart_baudRate, gps_uart_parity, gps_uart_dataBits, gps_uart_stopBits)) {
				continue;
			}
			gps_nmea_send_commands(gps_init_commands, txQueue);
		}

		wc_gps_update(true);
		gps_nmea_read_sentence(2000, /*gps_nmea_debug_process_sentence*/gps_nmea_process_sentence, getHostTxQueue);
	}
}

static void ping_pong_task(void *pvParameters)
{
	while(true)
	{
		vTaskDelay(pdMS_TO_TICKS(20));

		set_power_led(true);

		/**
		 * Acts so that WiCAN won't be silent for more than 500 ms.
		 * As the result host would respond with PING in case of no-send for over then 3s and also not in bulk-mode.
		 */
		if (esp_timer_get_time() - host_tx_last_time > 500*1000) { // no send to host for 500ms
			QueueHandle_t* const txQueue = getHostTxQueue();
			if (txQueue) {
				host_tx_task("\r", 0, txQueue);
			}
		}
	}
}

void notify_send_status(bool sent)
{
	// reverse logic - each successful send results in temporary 'off'
	if (sent) {
		set_power_led(false);
	}
}

static char* ensurePrintable(char* str, const int maxLength)
{
	for (int i = 0; i < maxLength; ++i) {
		if (!isprint((int) str[i])) {
			str[i] = 0;
			break;
		}
	}

	str[maxLength - 1] = 0;
	return str;
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

	gpio_set_level(PWR_LED_GPIO_NUM, 1); // WiCAN connected 'on'
	gpio_set_level(CAN_TR_LED_GPIO_NUM, 1); // CAN TR 'off'
	gpio_set_level(CAN_RX_LED_GPIO_NUM, 1); // CAN RX 'off'
	gpio_set_level(KLINE_GPS_LED_GPIO_NUM, 1); // GPS led 'on'

	xMsg_Rx_Queue = xQueueCreate(WICAN_RX_QUEUE_SIZE, sizeof( xdev_buffer) ); // common RX queue
	xMsg_Tx_Queue = xQueueCreate(32, sizeof( xdev_buffer) ); // TCP TX queue
	xmsg_ws_tx_queue = xQueueCreate(64, sizeof( xdev_buffer) ); // WS TX queue

	esp_ota_mark_app_valid_cancel_rollback();

	ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_SOFTAP));
	sprintf((char *)ble_uid,"WiC_%02x%02x%02x%02x%02x%02x",
			derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
			derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
	sprintf((char *)uid,"%02x%02x%02x%02x%02x%02x",
			derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
			derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
	
	config_server_start(&xmsg_ws_tx_queue, &xMsg_Rx_Queue, GPIO_NUM_NC, (char*)&uid[0]);
	slcan_init(&host_tx_task);

	/*
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
	*/
	can_init(CAN_100K); // one-time initialization, actual CAN bitrate will be set further

	//protocol = config_server_protocol();
	protocol = OBD_ELM327;

	if(protocol == REALDASH)
	{
		int can_datarate = config_server_get_can_rate();
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
		/*
		can_set_bitrate(can_datarate);
		can_enable();
		*/
		if(config_server_mqtt_en_config() && config_server_mqtt_elm327_log())
		{
			mqtt_elm327_log_en = config_server_mqtt_elm327_log();
			elm327_init(&host_tx_task, log_can_to_mqtt, CAN_TR_LED_GPIO_NUM);
		}
		else
		{
			elm327_init(&host_tx_task, elm327_log_can, CAN_TR_LED_GPIO_NUM);
		}

		gps_nmea_init(&host_tx_task);
	}

	if(config_server_mqtt_en_config())
	{
		int can_datarate = config_server_get_can_rate();
		can_set_bitrate(can_datarate);
		xmsg_mqtt_rx_queue = xQueueCreate(32, sizeof(mqtt_can_message_t) );
		can_enable();
		mqtt_init((char*)&uid[0], GPIO_NUM_NC, &xmsg_mqtt_rx_queue);
	}
//	else if(protocol == MQTT)
//	{
//		xmsg_mqtt_rx_queue = xQueueCreate(32, sizeof( twai_message_t) );
//		can_init(CAN_500K);
//		can_enable();
//
//		mqtt_init((char*)&uid[0], GPIO_NUM_NC, &xmsg_mqtt_rx_queue);
//	}


	const bool use_modem = true;
	wifi_network_init(use_modem ? "WiCANabitabit" : NULL, use_modem ? "airabit123" : NULL);
	int32_t port = config_server_get_port();

	if(port == -1)
	{
		port = 3333;
	}
	if(config_server_get_port_type() == UDP_PORT)
	{
		tcp_server_init(port, &xMsg_Tx_Queue, &xMsg_Rx_Queue, GPIO_NUM_NC, 1);
	}
	else
	{
		tcp_server_init(port, &xMsg_Tx_Queue, &xMsg_Rx_Queue, GPIO_NUM_NC, 0);
	}

	if(config_server_get_ble_config())
	{
		int pass = config_server_ble_pass();
		xmsg_ble_tx_queue = xQueueCreate(64, sizeof( xdev_buffer) ); // BLE TX queue
		ble_init(&xmsg_ble_tx_queue, &xMsg_Rx_Queue, GPIO_NUM_NC, pass, &ble_uid[0]);
	}



	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_app_desc_t running_app_info;
	if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
	{
		ESP_LOGI(TAG, "Running firmware version: %s", ensurePrintable(running_app_info.version, sizeof(running_app_info.version)));
		ESP_LOGI(TAG, "Project Name: %s", ensurePrintable(running_app_info.project_name, sizeof(running_app_info.project_name)));

		if(strstr(running_app_info.project_name, "usb") != 0)
		{
			project_hardware_rev = WICAN_USB_V100;
			ESP_LOGI(TAG, "project_hardware_rev: USB");

			xmsg_uart_tx_queue = xQueueCreate(32, sizeof( xdev_buffer) ); // USB / K-Line TX queue
			xmsg_uart_rx_queue = xQueueCreate(16, sizeof( xdev_buffer) ); // K-Line RX queue
	   		wc_uart_init(&xmsg_uart_tx_queue, &xMsg_Rx_Queue, &xmsg_uart_rx_queue, KLINE_GPS_LED_GPIO_NUM);
			
			elm327_uart_init(&xmsg_uart_tx_queue, &xmsg_uart_rx_queue);
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

	xTaskCreate(host_rx_task, "host_rx_task", 1024*3, (void*)AF_INET, 5, NULL);
	xTaskCreate(can_rx_task, "can_rx_task", 1024*3, (void*)AF_INET, 5, NULL);
	xTaskCreate(nmea_rx_task, "nmea_rx_task", 1024*3, (void*)AF_INET, 5, NULL);
	xTaskCreate(ping_pong_task, "ping_pong_task", 1024*2, (void*)AF_INET, 5, NULL);

	// temporary disable due to conflict with setup_uart_usb(..) in 'wc_uart.c'
	/*
	uint8_t enable_sleep = 0;
	float sleep_voltage = 13.1f;
	if (project_hardware_rev != WICAN_V210) {
		if (config_server_get_sleep_config()) {
			if (config_server_get_sleep_volt(&sleep_voltage)) {
				enable_sleep = 1;
			}
		}
	}
	sleep_mode_init(enable_sleep, sleep_voltage);
	*/

	// xEventTask = xEventGroupCreate();
	// xTaskCreate(ftp_task, "FTP", 1024*6, NULL, 2, NULL);
	// xEventGroupWaitBits( xEventTask,
	// FTP_TASK_FINISH_BIT, /* The bits within the event group to wait for. */
	// pdTRUE, /* BIT_0 should be cleared before returning. */
	// pdFALSE, /* Don't wait for both bits, either bit will do. */
	// portMAX_DELAY);/* Wait forever. */  
	esp_log_level_set("*", ESP_LOG_NONE);
}
