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
#include "esp_timer.h"
#include "esp_log_wican.h"
#include "can.h"
#include "wc_uart.h"
#include "sleep_mode.h"
#include "elm327.h"
#include "types.h"
#include "gps_common.h"

#include <ctype.h>

#define TAG  __func__

static QueueHandle_t can_rx_queue;
static QueueHandle_t *xuart_tx_queue = NULL, *xuart_rx_queue = NULL;
static int terminalR_led = GPIO_NUM_NC; // not connected

const char *ok_str = "OK";
const char *question_mark_str = "?";
const char *device_description = "ELM327 v1.3a meatPi";
const char *identify = "OBDLink MX";


bool (*elm327_response)(const char*, uint32_t, QueueHandle_t *q);
void (*elm327_can_log)(twai_message_t* frame, uint8_t type);
// The fields are ordered this way so the data can be tightly packed.
// See elm327_set_default_config for a more readable ordering.
typedef struct __xelm327_config
{
	uint32_t header;
	uint32_t rx_address;
	uint32_t req_timeout;
	uint32_t fc_header;
	uint8_t fc_data[5];
	//uint8_t protocol;
	uint8_t priority_bits;
	uint8_t fc_data_length:3;
	uint8_t fc_mode:2;
	uint8_t fc_enabled:1; // 'AT CFC0/CFC1' command
	uint8_t linefeed:1;
	uint8_t echo:1;
	uint8_t space_print:1;
	uint8_t show_header:1;
	uint8_t header_is_set:1;
	uint8_t fc_header_is_set:1;
	uint8_t rx_address_is_set:1;
	uint8_t display_dlc:1;
	uint8_t allow_long_messages:1; // 'AT AL' command
	uint8_t auto_formatting:1; // 'AT CAF0/CAF1' command
	uint8_t monitor_all:1; // 'AT MA' command

	uint8_t bitrate_index; // index like 'CAN_250K' in can.h
	uint8_t extd: 1; // Extended Frame Format (29bit ID)
	uint8_t terminal_resistor: 1; // on/off terminal resistor

	uint8_t perm_cmd_count;
	uint8_t perm_cmd_index;
	char perm_cmd_list[85]; // each permanent command is of CMD_LENGTH
	uint8_t perm_cmd_delay; // delay between two consecutive permanent commands in ms

	uint8_t uds_rps_skip_prefix[8]; // can't be longer then CAN frame
	int uds_rps_skip_size; // actual size of 'skip prefix', that is [1..8], 0 means not used
	int uds_rps_last_rx_size; // value of 'rx queue size' which last sent to user when 'skip prefix' is active
	int uds_rps_last_count;

	int kline_baud; // baud rate
	int kline_parity; // E_PARITY_N = 0, E_PARITY_O = 1, E_PARITY_E = 2, E_PARITY_M = 3, E_PARITY_S = 4,
	int kline_data_bits; // E_DATABITS_7 = 7, E_DATABITS_8 = 8,
	int kline_stop_bits; // E_STOPBITS_1 = 0, E_STOPBITS_2 = 2,
	int kline_monoline; // '1' means K-Line, i.e. echo-bytes should be checked

}_xelm327_config_t;


static const uint8_t CMD_LENGTH = 17; // Single Frame + 1 byte for 'req_expected_rsp'
#define KWP_COMMAND_LENGHT 260 // maximum KWP message length in bytes (including header and CS)

static _xelm327_config_t elm327_config;
static bool elm327_waiting_answer = false;

static void elm327_set_default_config(bool reset_protocol)
{
	// Header or ID settings
	elm327_config.priority_bits = 0x18;
	elm327_config.header_is_set = 0;
	elm327_config.header = 0;

	// Response address filter settings
	elm327_config.rx_address_is_set = 0;
	elm327_config.rx_address = 0;

	// See reset_all for why this is optional
	if (reset_protocol)
	{
		//elm327_config.protocol = '6';
		elm327_config.bitrate_index = CAN_250K;
		elm327_config.extd = 0;
		elm327_config.terminal_resistor = 1;
	}

	elm327_config.req_timeout = 0x32; //50 ms

	// Flow Control Settings
	elm327_config.fc_mode = 0;
	elm327_config.fc_enabled = 1; // The default setting is CFC1 - Flow Controls on
	elm327_config.fc_header_is_set = 0;
	elm327_config.fc_header = 0;
	elm327_config.fc_data_length = 0;
	memset(elm327_config.fc_data, 0, sizeof(elm327_config.fc_data));

	// Display settings
	elm327_config.show_header = 0;
	elm327_config.linefeed = 0;
	elm327_config.echo = 1;
	elm327_config.space_print = 1;
	elm327_config.display_dlc = 0;

	// The standard OBDII protocols restrict the number of data bytes in a message to seven, 
	// which the ELM327 normally does as well (for both send and receive).
	// If AL is selected, the ELM327 will allow long sends (eight data bytes) and long receives (unlimited in number).
	// The default is AL off (and NL selected).
	elm327_config.allow_long_messages = 0;

	// Determine whether the ELM327 assists you with the formatting of the CAN data that is sent and received. 
	// With CAN Automatic Formatting enabled (CAF1), the formatting (PCI) bytes will be automatically generated for you when sending, 
	// and will be removed when receiving.
	// Auto Formatting on (CAF1) is the default setting.
	elm327_config.auto_formatting = 1;

	elm327_config.monitor_all = 0; // off till explicitly turned on

	elm327_config.perm_cmd_count = 0;
	elm327_config.perm_cmd_index = 0;
	elm327_config.perm_cmd_list[0] = 0;
	elm327_config.perm_cmd_delay = 0;

	elm327_config.uds_rps_skip_size = 0;

	elm327_config.kline_baud = 10400; // baud rate
	elm327_config.kline_parity = 0; // E_PARITY_N = 0, E_PARITY_O = 1, E_PARITY_E = 2, E_PARITY_M = 3, E_PARITY_S = 4,
	elm327_config.kline_data_bits = 8; // E_DATABITS_7 = 7, E_DATABITS_8 = 8,
	elm327_config.kline_stop_bits = 0; // E_STOPBITS_1 = 0, E_STOPBITS_2 = 2,
	elm327_config.kline_monoline = 1; // '1' means K-Line, i.e. echo-bytes should be checked

}

typedef char* (*elm327_command_callback)(const char* command_str);
typedef struct _xelm327_cmd
{
	const char * const command;
	const elm327_command_callback command_interpreter;
}xelm327_cmd_t;

static esp_err_t can_tx_task(twai_message_t *message)
{
	if( elm327_can_log != NULL) {
		elm327_can_log(message, ELM327_CAN_TX);
	}

	// maximum send wait will be 30*3 + 2 = 92 ms
	//
	const TickType_t one_ms = pdMS_TO_TICKS(1);
	const TickType_t send_wait_ms = one_ms * 3;
	int retry_count = 3;

	esp_err_t result = can_send(message, send_wait_ms);
	while (result != ESP_OK) {
		if ((result == ESP_FAIL || result == ESP_ERR_TIMEOUT) && --retry_count > 0) {
			//ESP_LOGW(TAG, "can_send() fails (repeat) , reason = 0x%04X", result);
			vTaskDelay(one_ms);
		} else {
			ESP_LOGE(TAG, "can_send() fails (skip) , reason = 0x%04X , data [%02X %02X %02X %02X]",
				result, message->data[0], message->data[1], message->data[2], message->data[3]);
			break;
		}
		
		result = can_send(message, send_wait_ms);
	}

	notify_send_status(result == ESP_OK);
	return result;
}


static unsigned int elm327_parse_hex_char(char chr)
{
    unsigned int result = 0;
    if (chr >= '0' && chr <= '9') result = chr - '0';
    else if (chr >= 'A' && chr <= 'F') result = 10 + chr - 'A';
    else if (chr >= 'a' && chr <= 'f') result = 10 + chr - 'a';

    return result;
}

static unsigned int elm327_parse_hex_str(const char *str, int len)
{
	assert(strlen(str) >= len);

    unsigned int result = 0;
    for (int i = 0, in = strlen(str); i < len && i < in; i++) {
		result += elm327_parse_hex_char(str[i]) << (4 * (len - i - 1));
	}
    return result;
}

static void elm327_fill_data_from_hex_str(const char *str, uint8_t *data, int data_length)
{
	assert(strlen(str) >= data_length * 2);

	for (int i = 0, in = strlen(str); i < data_length && i * 2 + 1 < in; i++) {
		data[i] = (elm327_parse_hex_char(str[i*2]) << 4) + (elm327_parse_hex_char(str[i*2 + 1]));
	}
}

static char* elm327_return_ok(const char* command_str)
{
	return (char*)ok_str;
}

static char* elm327_set_linefeed(const char* command_str)
{
	if(command_str[1] == '1')
	{
		elm327_config.linefeed = 1;
	}
	else if(command_str[1] == '0')
	{
		elm327_config.linefeed = 0;
	}
	else
	{
		return 0;
	}
	return (char*)ok_str;
}

static char* elm327_allow_long_messages(const char* command_str)
{
	elm327_config.allow_long_messages = 1;
	return (char*)ok_str;
}

static char* elm327_auto_formatting(const char* command_str)
{
	if(command_str[3] == '1')
	{
		elm327_config.auto_formatting = 1;
	}
	else if(command_str[3] == '0')
	{
		elm327_config.auto_formatting = 0;
	}
	else
	{
		return 0;
	}
	return (char*)ok_str;
}

static char* elm327_set_echo(const char* command_str)
{
	if(command_str[1] == '1')
	{
		elm327_config.echo = 1;
	}
	else if(command_str[1] == '0')
	{
		elm327_config.echo = 0;
	}
	else
	{
		return 0;
	}
	return (char*)ok_str;
}

static char* elm327_header_on_off(const char* command_str)
{
	if(command_str[1] == '1')
	{
		elm327_config.show_header = 1;
	}
	else if(command_str[1] == '0')
	{
		elm327_config.show_header = 0;
	}
	else
	{
		return 0;
	}
	return (char*)ok_str;
}


static char* elm327_device_description(const char* command_str)
{
	return (char*)device_description;
}

static char* elm327_identify(const char* command_str)
{
	return (char*)identify;
}

static char* elm327_monitor_all(const char* command_str)
{
	if (!elm327_config.monitor_all) {
		elm327_config.monitor_all = 1;
		ESP_LOGW(TAG, "Monitor All is on");
	}
	return ""; // skip reply
}

static esp_err_t elm327_send_can_cmd(const char* command)
{
	twai_message_t txframe;
	
	// Initialize the data
	memset(txframe.data, 0xAA, 8);
	txframe.data_length_code = 8; // CAN frames always have a data length code of 8, this is different than the PCI byte (txframe.data[0])
	txframe.self = 0;
	txframe.rtr = 0;

	const size_t arg_size = strlen(command);
	int offset = 0;

	if (arg_size == 21) { // normal Can Id
		txframe.extd = 0;
		txframe.identifier = elm327_parse_hex_str(command, 3);
		offset += 3;
	} else { // extended Can Id
		txframe.extd = 1;
		txframe.identifier = elm327_parse_hex_str(command, 8);
		offset += 8;
	}

	elm327_fill_data_from_hex_str(command + offset, &txframe.data[0], 8);

	return can_tx_task(&txframe);
}

static char* elm327_special_send(const char* command_str)
{
	elm327_send_can_cmd(command_str + 3); // skips length of 'spc'

	return ""; // skip reply
}

static char* elm327_restore_defaults_or_display_dlc(const char* command_str)
{
	size_t arg_size = strlen(command_str+1);
	if(arg_size == 0)
	{
		// ATD: restore defaults
		elm327_set_default_config(false);
	}
	else
	{
		// TODO: currently we don't use this display_dlc property
		if(command_str[1] == '1')
		{
			// ATD1: display DLC
			elm327_config.display_dlc = 1;
		}
		else if(command_str[1] == '0')
		{
			// ATD0: don't display DLC
			elm327_config.display_dlc = 0;
		}
		else
		{
			return 0;
		}
	}

	return (char*)ok_str;
}

/*
static void elm327_set_filter(uint32_t filter)
{
	can_disable();

	if (filter == 0xFFFFFFFF) {
		ESP_LOGI(TAG, "accept all");

		static const twai_filter_config_t allPassFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
		can_set_filter(allPassFilter.acceptance_code);
		can_set_mask(allPassFilter.acceptance_mask);
	} else {
		ESP_LOGI(TAG, "single 0x%08X", (unsigned int)filter);

		can_set_filter(filter << 3);
		can_set_mask(~TWAI_EXTD_ID_MASK);
	}

	can_enable();

	if (filter != 0xFFFFFFFF) {
		twai_message_t rx_frame;
		while( xQueueReceive(can_rx_queue, &rx_frame, 0) ) {
			// cleanup before new send request
		}
	}
}
*/

static char* elm327_reset_all(const char* command_str)
{
	// TODO: this should set the active protocol to be
	// the stored protocol. The SP command should store the
	// protocol, and TP could be used to try a new protocol without
	// storing it. Also if a SP 0 is sent then protocol scanning will
	// happen and and only if a valid protocol is found will it be stored.
	// Currently we don't have a explict stored protocol, so for now
	// when ATZ or ATD is called we don't change the protocol.
	// This approach is required because at least some clients do
	// not set the protocol again after calling ATZ.
	elm327_set_default_config(false);

	//elm327_set_filter(0xFFFFFFFF); // accept all CAN messages

	ESP_LOGW(TAG, "Reset protocol");

	return (char*)device_description;
}

/*
 * ELM327 supports 3 sizes of parameters to the set header (SH)
 * command.
 * - 3 hex digits (xyz) means to set the header field to 00 0x yz
 * - 6 hex digits (xx yy zz)
 * - 8 hex digits (ww xx yy zz)
 *
 * There is also the CP command for setting the priority bits of
 * a 29bit header. Basically that is an alternative way to set the
 * the ww in the 8 hex digit param.
 *
 * Because the CP command could be sent after set header
 * And because the header could be used for either a 11bit or 29bit
 * message. And because the TWAI spec doesn't indicate if the extra
 * bytes of the 11bit identifier would be ignored or not.
 * It seems easier and safer to store the header and priority bytes
 * separately.
 */
static char* elm327_set_header(const char* command_str)
{
	size_t id_size = strlen(command_str+2);

	if(id_size == 3 || id_size == 6)
	{
		// SH xyz or SH xx yy zz
		elm327_config.header = elm327_parse_hex_str(command_str+2, id_size);
		elm327_config.header_is_set = 1;
	}
	else if(id_size == 8)
	{
		// SH ww xx yy zz
		// The documentation isn't clear if priority bits should be stored separately.
		// The key question is whether setting the header again with the SH xx yy zz
		// form would keep the priority bits set here or not.
		// Since it is ambiguous, storing them separately is the easiest to implement
		elm327_config.priority_bits = elm327_parse_hex_str(command_str+2, 2) & 0x1F;
		elm327_config.header = elm327_parse_hex_str(command_str+4, 6);
		elm327_config.header_is_set = 1;
	}
	else
	{
		return 0;
	}

	return (char*)ok_str;
}

static char* elm327_set_priority_bits(const char* command_str)
{
	size_t arg_size = strlen(command_str+2);

	if(arg_size != 2) return 0;

	// Only save 5 bits
	elm327_config.priority_bits = elm327_parse_hex_str(command_str+2, arg_size) & 0x1F;

	return (char*)ok_str;
}

/*
 * Could be
 * - ATCRA
 * - ATCRA xyz
 * - ATCRA wwxxyyzz
 *
 * TODO: this should handle "X" characters. That would be best handled with a
 * mask property in the config. This mask property would be the the same one
 * that is set by the (not yet implemented) CM command.
 */
static char* elm327_set_receive_address(const char* command_str)
{
	const size_t arg_size = strlen(command_str+3);

	if(arg_size == 0 || strncmp(command_str+3, "xxx", 3) == 0)
	{
		// elm327_config.rx_address_is_set = 0;

		// elm327_set_filter(0xFFFFFFFF); // accept all CAN messages
	}
	else if(arg_size == 3 || arg_size == 8)
	{
		elm327_config.rx_address_is_set = 1;
		elm327_config.rx_address = elm327_parse_hex_str(command_str+3, arg_size);

		// elm327_set_filter(elm327_config.rx_address); // accept only given message
	}
	// else
	// {
	// 	return 0;
	// }

	return (char*)ok_str;
}

static char* elm327_set_fc_mode(const char* command_str)
{
	size_t arg_size = strlen(command_str+4);

	if (arg_size != 1)
	{
		return 0;
	}

	int mode = elm327_parse_hex_str(command_str+4, 1);
	if (mode < 0 || mode > 2)
	{
		return 0;
	}

	if ((mode == 1 || mode == 2) && elm327_config.fc_data_length == 0)
	{
		// The flow control data needs to be set before using modes 1 or 2
		return 0;
	}

	if (mode == 1 && elm327_config.fc_header_is_set == 0)
	{
		// The flow control header has to be set before using mode 1
		return 0;
	}

	elm327_config.fc_mode = mode;
	return (char*)ok_str;
}

static char* elm327_set_fc_enabled(const char* command_str)
{
	if(command_str[3] == '1')
	{
		elm327_config.fc_enabled = 1;
	}
	else if(command_str[3] == '0')
	{
		elm327_config.fc_enabled = 0;
	}
	else
	{
		return 0;
	}
	return (char*)ok_str;
}

static char* elm327_set_fc_header(const char* command_str)
{
	size_t id_size = strlen(command_str+4);

	if(!(id_size == 3 || id_size == 8))
	{
		// Only "FC SH xyz" or "FC SH ww xx yy zz" is allowed
		return 0;
	}

	elm327_config.fc_header = elm327_parse_hex_str(command_str+4, id_size);
	elm327_config.fc_header_is_set = 1;
	return (char*)ok_str;
}

static char* elm327_set_fc_data(const char* command_str)
{
	size_t data_size = strlen(command_str+4)/2;

	if(data_size < 1 || data_size > 5)
	{
		return 0;

	}

	elm327_config.fc_data_length = data_size;
	elm327_fill_data_from_hex_str(command_str+4, elm327_config.fc_data, data_size);
	return (char*)ok_str;
}

static int bitrate_to_index(const int bitrate)
{
	switch (bitrate) {
	case 1000000: return CAN_1000K;
	case 800000: return CAN_800K;
	case 500000: return CAN_500K;
	case 250000: return CAN_250K;
	case 125000: return CAN_125K;
	case 100000: return CAN_100K;
	case 50000: return CAN_50K;
	case 25000: return CAN_25K;
	case 20000: return CAN_20K;
	case 10000: return CAN_10K;
	case 5000: return CAN_5K;
	default:
		ESP_LOGE(TAG, "unexpected CAN bitrate: %d, 250K will be used", bitrate);
		return CAN_250K;
	}
}

static char* elm327_set_protocol(const char* command_str)
{
	/*
	//Handle SPAx, and set it as x. 
	//TODO: add support for auto sp
	if(command_str[2] == 'a' || command_str[2] == 'A')
	{
		if(command_str[3] == '6' || command_str[3] == '7' || 
			command_str[3] == '8' || command_str[3] == '9')
		{
			elm327_config.protocol = command_str[3];
		}
		else
		{
			elm327_config.protocol = '4';
		}
	}
	else
	{
		elm327_config.protocol = command_str[2];
	}
	
	ESP_LOGI(TAG, "proto = %c", elm327_config.protocol);

	// The header should not be reset when the protocol is changed
	// unless protocol autoscanning is being used. And even in that case
	// the header bytes should revert to their set value after the
	// autoscanning is complete. See the 2nd and 3rd paragraphs of the
	// "SH xx yy zz" section of the ELM docs.
	//
	// In some cases Carscanner sends the header first and then changes
	// the protocol.
	if(elm327_config.protocol == '6' || elm327_config.protocol == '7')
	{
		can_disable();
		vTaskDelay(pdMS_TO_TICKS(15));
		can_set_bitrate(CAN_500K);
		can_enable();
		vTaskDelay(pdMS_TO_TICKS(15));
	}
	else if(elm327_config.protocol == '8' || elm327_config.protocol == '9')
	{
		can_disable();
		vTaskDelay(pdMS_TO_TICKS(15));
		can_set_bitrate(CAN_250K);
		can_enable();
		vTaskDelay(pdMS_TO_TICKS(15));
	}
	*/

	// restore spaces for sscanf
	char *p = (char*)&command_str[2];
	while ((p = strchr(p, '_')) != NULL) {
		*p = ' ';
	}

	int bitrate = CAN_250K, extd = 0, terminalR = 1;
	const int fieldCount = sscanf(&command_str[2], "%d %d %d", &bitrate, &extd, &terminalR); // considers length of 'sp'
	if (fieldCount != 3) {
		ESP_LOGE(TAG, "malformed SP command: %s", command_str);
		return (char*)question_mark_str;
	} else {
		ESP_LOGI(TAG, "baud: %d, extd: %d, terminalR: %d", bitrate, extd, terminalR);
	}

	elm327_config.bitrate_index = bitrate_to_index(bitrate);
	elm327_config.extd = extd;
	elm327_config.terminal_resistor = terminalR;

	can_disable();
	vTaskDelay(pdMS_TO_TICKS(15));

	can_set_bitrate(elm327_config.bitrate_index);
	gpio_set_level(terminalR_led, elm327_config.terminal_resistor ? 0 : 1);

	static twai_filter_config_t allPassFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
	can_set_filter(allPassFilter.acceptance_code);
	can_set_mask(allPassFilter.acceptance_mask);

	can_set_silent(0);

	can_enable();
	vTaskDelay(pdMS_TO_TICKS(15));

	return (char*)ok_str;
}

static char* elm327_set_timeout(const char* command_str)
{
	elm327_config.req_timeout = (strtol((char *) &command_str[2], NULL, 16) & 0xFFFF); // allows to use 2-byte timeout

	if(elm327_config.req_timeout == 0)
	{
		elm327_config.req_timeout = 0x32;
	}

	ESP_LOGI(TAG, "%lu ms", (TickType_t)((elm327_config.req_timeout*4.096) / portTICK_PERIOD_MS));

	return (char*)ok_str;
}

static char* elm327_input_voltage(const char* command_str)
{
	static char volt[10] = "";
	float voltage = 0;

	if(sleep_mode_get_voltage(&voltage) != -1)
	{
		sprintf(volt, "%.1fV",voltage);
	}
	else
	{
		sprintf(volt, "%.1f",0.0f);
	}
	return (char*)volt;
}

static uint32_t elm327_get_identifier()
{
	/*
	switch(elm327_config.protocol) {
		case '6':
		case '8':
			// The TWAI api isn't clear if it handles masking the header
			// So to be safe we mask it ourselves
			return elm327_config.header & TWAI_STD_ID_MASK;
		case '7':
		case '9':
			return (elm327_config.priority_bits << 24) | elm327_config.header;
		default:
			// In theory this line shouldn't be hit,
			// but just in case return something reasonable
			return elm327_config.header;
	}
	*/

	if (elm327_config.extd) {
		return ((elm327_config.priority_bits << 24) | elm327_config.header) & TWAI_EXT_ID_MASK;
	} else {
		return elm327_config.header & TWAI_STD_ID_MASK;
	}
}

/*
 * TODO: this should support the CM and CF commands.
 *
 * It isn't clear if setting the priority bits with the CP should change the
 * default filter. Based on the commands sent by Carscanner it seems like the CP
 * priority bits do not change the default filter.
 */
static uint8_t elm327_should_receive(twai_message_t *rx_frame)
{
	uint32_t identifier = rx_frame->identifier;
	if(elm327_config.rx_address_is_set)
	{
		if (identifier == elm327_config.rx_address) {
			return true;
		} else {
			//ESP_LOGW(TAG, "skip by rx_address %08X", (unsigned int)rx_frame->identifier&TWAI_EXTD_ID_MASK);
			return false;
		}
	}
	return false;
	// else
	// {
	// 	if(rx_frame->extd)
	// 	{
	// 		return identifier >= 0x18DAF100 && identifier <= 0x18DAF1FF;
	// 	}
	// 	else
	// 	{
	// 		return identifier >= 0x7E8 && identifier <= 0x7EF;
	// 	}
	// }
}

static void elm327_send_flow_control_frame(twai_message_t *first_frame)
{
	twai_message_t txframe;

	// Use the same size id as the first frame
	txframe.extd = first_frame->extd;

	if (first_frame->extd)
	{
		if (elm327_config.fc_mode == 0 || elm327_config.fc_mode == 2)
		{
			// Automatically compute the identifier from the first frame
			//
			// We find the source ECU and then construct an identifier with
			// - the default priority bits (18)
			// - a physical type (DA) as opposed to a functional type
			// - the source_ecu as the destination
			// - ourselves (F1) as the source
			uint8_t source_ecu = 0xFF & first_frame->identifier;
			txframe.identifier = 0x18DA00F1 | (source_ecu << 8);
		}
		else
		{
			// fc_mode 1: use the configured header
			txframe.identifier = elm327_config.fc_header;
		}
	}
	else
	{
		if (elm327_config.fc_mode == 0 || elm327_config.fc_mode == 2)
		{
			// Automatically compute the identifier from the first frame
			//
			// We set the 4th bit to 0. Apparently when the first two nibbles are
			// 7E, this 4th bit indicates wether a message is being sent to or
			// received from an ECU identified by the last 3 bits.
			// For example 0x7E8 becomes 0x7E0 and 0x7EF becomes 0x7E7
			txframe.identifier = first_frame->identifier & 0xFF7;
		}
		else
		{
			// fc_mode 1: use the configured header
			txframe.identifier = elm327_config.fc_header & TWAI_STD_ID_MASK;
		}
	}

	// Initialize the data
	memset(txframe.data, 0xAA, 8);
	txframe.data_length_code = 8; // CAN frames always have a data length code of 8
	txframe.self = 0;
	txframe.rtr = 0;

	if (elm327_config.fc_mode == 0)
	{
		// data[0] & 0xF0 == 0x30 identifies it as a flow control frame,
		// data[0] & 0X0F is the flow status:
		// - 0 tells the ECU we are ready to receive more frames
		// - 1 tells the ECU to wait
		// - 2 tells the ECU we are overloaded and it should abort
		txframe.data[0] = 0x30;
		// Block Size:
		// - 0 means send all of the frames
		// - greater than 0 means send this number of frames then wait for a flow control ACK
		txframe.data[1] = 0x00;
		// Separation time in ms. There are also special values for large separation times.
		// The spec indicates this is supposed to be the time from when the last bit of the
		// last frame was sent to when the first bit of the next frame. However the spec
		// notes that some ECUs will implement this as the time from first bit to first bit
		//
		// Note: this value of 10ms is just a guess. Some docs have 20ms in their examples.
		// When carscanner sets the flow control data it uses a value 0ms.
		txframe.data[2] = 10;
	}
	else
	{
		// mode 1 or 2: use the data set by the client
		memcpy(txframe.data, elm327_config.fc_data, elm327_config.fc_data_length);
	}

	can_tx_task(&txframe);
}

static TickType_t elapsedTimeMs(int64_t txtime)
{
	return (TickType_t)(((esp_timer_get_time() - txtime)/1000)/portTICK_PERIOD_MS);
}

// API
int elm327_rx_queue_size(QueueHandle_t *rx_queue)
{
	if (elm327_config.uds_rps_skip_size == 0) {
		return -1;
	}

	const int rx_size = uxQueueMessagesWaiting(*rx_queue);
	const int rx_size_round = (rx_size / 10) * 10;

	if (elm327_config.uds_rps_last_rx_size == -1) {
		if (rx_size_round < 10) {
			return -1; // wait till queue will be initially filled
		}
	}

	bool sendWait = false;
	if (rx_size_round == 0 || rx_size_round == WICAN_RX_QUEUE_SIZE) {
		sendWait = ((elm327_config.uds_rps_last_count % 10) == 0);
		elm327_config.uds_rps_last_count += 1;
	} else if (elm327_config.uds_rps_last_rx_size != rx_size_round) {
		sendWait = true;
		elm327_config.uds_rps_last_count = 0;
	}
	
	if (!sendWait) {
		return -1;
	}

	elm327_config.uds_rps_last_rx_size = rx_size_round;
	return rx_size_round;
}

static void close_skip_mode()
{
	if (elm327_config.uds_rps_skip_size > 0) {
		elm327_config.uds_rps_skip_size = 0;
		ESP_LOGW(TAG, "UDS skip mode is off");
	}
}

static void elm327_request_wait_answer(uint8_t req_expected_rsp, twai_message_t *txframe, bool fc_less_mode, char *rsp, QueueHandle_t *queue, int (*fnHasNewData)());

/*__attribute__((optimize("O0")))*/ static void elm327_request(const char *cmd, const size_t cmd_len, bool fc_less_mode, char *rsp, QueueHandle_t *queue, int (*fnHasNewData)())
{
	twai_message_t txframe;
	txframe.identifier = elm327_get_identifier();
	txframe.extd = elm327_config.extd;//elm327_config.protocol == '7' || elm327_config.protocol == '9';

	// Initialize the data
	memset(txframe.data, 0xAA, 8);
	txframe.data_length_code = 8;  // CAN frames always have a data length code of 8
	txframe.rtr = 0;
	txframe.self = 0;

	uint8_t req_expected_rsp = 0xFF;

	// If the command length is odd then the last digit is the number of frames
	// to expect in response. This is an optimization supported by the ELM327
	// protocol. It is so the OBD2 device doesn't have to wait to see if there
	// are more frames. Once it gets the expected number it can stop waiting and
	// return the result.
	if(cmd_len % 2 == 1)
	{
		req_expected_rsp = elm327_parse_hex_char(cmd[cmd_len - 1]);
		if(req_expected_rsp > 0x0F)
		{
			req_expected_rsp = 0xFF;
		}
	}
	ESP_LOGI(TAG, "req_expected_rsp = %u", req_expected_rsp);

	const uint8_t cmd_data_length = cmd_len / 2;
	if(elm327_config.allow_long_messages && cmd_data_length == 8)
	{
		elm327_fill_data_from_hex_str(cmd, &txframe.data[0], cmd_data_length);
	}
	else if(cmd_data_length > 7)
	{
		// commands can't be longer than 7 bytes unless flow control is used
		// FIXME: this should use the linefeed setting and match the number of
		// `\r`s that are normally sent.
		elm327_response("?\r>", 0, queue);
		return;
	}
	else
	{
		txframe.data[0] = cmd_data_length;
		elm327_fill_data_from_hex_str(cmd, &txframe.data[1], cmd_data_length);
	}

	twai_message_t rx_frame;
	if (req_expected_rsp != 0) {
		while( xQueueReceive(can_rx_queue, &rx_frame, 0) ) {
			// cleanup before new send request
			ESP_LOGW(TAG, "skip before send %08X", (unsigned int)rx_frame.identifier&TWAI_EXTD_ID_MASK);
		}
	}

	elm327_waiting_answer = true;
	if (can_tx_task(&txframe) != ESP_OK) {
		if (req_expected_rsp != 0 && req_expected_rsp != 0xFF) {
			elm327_waiting_answer = false;
			elm327_response("CAN ERROR\r>", 0, queue);
			close_skip_mode();
			return;
		}
	}

	if (req_expected_rsp != 0) {
		elm327_request_wait_answer(req_expected_rsp, &txframe, fc_less_mode, rsp, queue, fnHasNewData);
	}
	elm327_waiting_answer = false;
}

static void elm327_request_wait_answer(uint8_t req_expected_rsp, twai_message_t *txframe, bool fc_less_mode, char *rsp, QueueHandle_t *queue, int (*fnHasNewData)())
{
	TickType_t totalMs = (elm327_config.req_timeout*4.096) / portTICK_PERIOD_MS;
	const int64_t txtime = esp_timer_get_time();
	bool rsp_found = false;
	uint8_t number_of_rsp = 0;

	uint8_t sendControlFrame = false;
	twai_message_t rx_frame;

	while (true)
	{
		const int64_t txtime_local = esp_timer_get_time();

		if (xQueueReceive(can_rx_queue, &rx_frame, 5)) {
			totalMs += elapsedTimeMs(txtime_local);

			//reset timeout after response is received
			rsp_found = true;
			number_of_rsp++;

			// Identify what kind of frame this is.
			int rx_frame_data_length = 0;
			const uint8_t frame_type = rx_frame.data[0] & 0xF0;
			if (frame_type == 0x10 || frame_type == 0x50) // it can be Abit Ecu which supports fc-less mode
			{
				// This is a first frame
				// Send a flow control response so we can get the remaining frames
				if(elm327_config.fc_enabled)
				{
					elm327_send_flow_control_frame(&rx_frame);
				}
				// Length of the full data is:
				//   ((0x0F & data[0]) << 8 | data[1])
				//
				// For now, we say the data length is 7 so the second part
				// of the data length (data[1]) is sent plus the 6 bytes of
				// actual data.
				//
				// TODO: if elm327_config.show_header is disabled then we
				// should send the length of the full data on its own line
				// and then send `0: [6 bytes of data]` on the next line for
				// the first frame
				rx_frame_data_length = 7;

				rx_frame.data[0] |= 0x40;
				sendControlFrame = (frame_type == 0x10);
				const uint16_t expectedLength = ((0x0F & rx_frame.data[0]) << 8 | rx_frame.data[1]);
				req_expected_rsp = (uint8_t)(expectedLength / 7) + 1;
			}
			else if (frame_type == 0x20 || frame_type == 0x60) // it can be Abit Ecu which supports fc-less mode
			{
				// This is a consecutive frame
				// Sequence index of the frame is 0x0F & data[0]
				// From the examples in the ELM327 docs the final consecutive frame includes
				// any padding bytes. In theory we could be smarter and figure out how many
				// of the total bytes we've received and then not print the padding bytes.
				// However this is complex since the frames might come in out of order and
				// there might be more than 15 of them.
				// TODO: if elm327_config.show_header is disabled then we should add a prefix to the
				// printed line: `[sequence index]: [7 bytes of data]`.
				rx_frame_data_length = 7;

				rx_frame.data[0] |= 0x40;
			}
			else if (frame_type == 0x30)
			{
				if (fc_less_mode) {
					//ESP_LOGW(TAG, "skip CanTP CF (%s)", (req_expected_rsp != 0xFF && req_expected_rsp == number_of_rsp) ? "ok" : "fail");
					assert(req_expected_rsp != 0xFF && req_expected_rsp == number_of_rsp);
					break;
				}
				// This is a flow control frame from an ECU
				//
				// TODO: if we start supporting sending more than 7 bytes of
				// data. We'll have to send first frames ourselves, and
				// we'll need to handle receiving flow control frames from
				// ECUs. In the meantime if we get one just send all the
				// bytes to the client
				rx_frame_data_length = 7;

				rx_frame.data[3] = 0x01; // don't send nor wait for FC frames (Abit specific)
				rx_frame.data[4] = ~rx_frame.data[3];
			}
			else
			{
				// This is a single frame
				rx_frame_data_length = rx_frame.data[0];
			}

			const bool rsp_complete = (req_expected_rsp != 0xFF && req_expected_rsp == number_of_rsp);

			if (elm327_config.uds_rps_skip_size > 0
					&& memcmp(elm327_config.uds_rps_skip_prefix, rx_frame.data, elm327_config.uds_rps_skip_size) == 0
					) {
				//ESP_LOGW(TAG, "skip response: %s", rsp);

				if (rsp_complete) {
					break;
				} else {
					continue;
				}
			}

			uint8_t data_offset = 0;

			// Based on the "CAF0 AND CAF1" section of the ELM doc, if headers are shown
			// the PCI byte(s) (usually just data[0]) should be printed.
			if(elm327_config.show_header)
			{
				elm327_print_canid(rsp, &rx_frame);
				rx_frame_data_length = 8;
			} else if(!elm327_config.auto_formatting) {
				rx_frame_data_length = 8;
			} else {
				// If this is a first frame, consecutive frame, or flow control frame the PCI (rx_frame.data[0]) will
				// not be a valid length without some processing, so just print all 7 bytes
				data_offset = 1;
				if(rx_frame_data_length > 7) rx_frame_data_length = 7;
			}

			int rsp_offset = strlen(rsp);
			for (int i = 0; i < rx_frame_data_length; i++) {
				rsp_offset += sprintf(rsp + rsp_offset, "%02X", rx_frame.data[data_offset + i]);
			}

			strcat(rsp, "\r");
//			ESP_LOGW(TAG, "ELM327 send: %s", rsp);
//			ESP_LOG_BUFFER_HEX(TAG, rsp, strlen(rsp));

			//if (rsp_complete) {
			//	strcat(rsp, ">");
			//}

			elm327_response(rsp, 0, queue);
			rsp[0] = 0;

			if (rsp_complete) {
				ESP_LOGI(TAG, "response time = %lu ms", elapsedTimeMs(txtime));
				break;
			}

			static const uint8_t BS_MAX = 0xFF; // RX_QUEUE_LENGTH + 1; // esp32-can fails to receive more then (.rx_queue_len + 1) simultaneous CAN messages
			if (sendControlFrame && ((number_of_rsp - 1) % BS_MAX) == 0) {
				memset(txframe->data, 0xAA, 8);

				txframe->data[0] = 0x30;
				txframe->data[1] = BS_MAX;
				txframe->data[2] = 0x00; // zero duration between two consecutive frames

				rx_frame.data[3] = 0x01; // don't send nor wait for FC frames (Abit specific)
				rx_frame.data[4] = ~rx_frame.data[3];

				if (can_tx_task(txframe) != ESP_OK) {
					elm327_response("CAN ERROR\r>", 0, queue);
					close_skip_mode();
					break;
				}
			}
		}
		else
		{
			const TickType_t elapsedMs = elapsedTimeMs(txtime);
			
			if (elapsedMs >= totalMs) {
				ESP_LOGW(TAG, "response timeout = %lu ms", elapsedMs);

				if (!rsp_found) {
					strcat(rsp, "NO DATA");
					close_skip_mode();
				}
				strcat(rsp, "\r>");
			
				elm327_response(rsp, 0, queue);

				break;
			} else if (elm327_config.uds_rps_skip_size == 0 && elapsedMs >= totalMs / 10 && fnHasNewData()) {
				ESP_LOGW(TAG, "response reset by incoming data = %lu ms", elapsedMs);
				break;
			}
		}
	}
}

static void elm327_kline_send(const char *cmd, const size_t cmd_len, QueueHandle_t *q, int (*fnHasNewData)())
{
	// prepare KWP message
	const int kwp_bytes_count = MIN(cmd_len / 2, KWP_COMMAND_LENGHT);
	if (kwp_bytes_count == 0) {
		elm327_response("kwp_empty_message_ CAN ERROR\r>", 0, q);
		return;
	}

	char rsp[KWP_COMMAND_LENGHT];
	elm327_fill_data_from_hex_str(cmd, (uint8_t *) rsp, kwp_bytes_count);

	// clear incoming queue
	xQueueReset(*xuart_rx_queue);

	// send KWP message
	if (!elm327_response(rsp, kwp_bytes_count, xuart_tx_queue)) {
		elm327_response("kwp_send_fails_ CAN ERROR\r>", 0, q);
		close_skip_mode();
		notify_send_status(false);
		return;
	} else {
		notify_send_status(true);
	}

	TickType_t totalMs = (elm327_config.req_timeout*4.096) / portTICK_PERIOD_MS;
	const int64_t txtime = esp_timer_get_time();

	xdev_buffer xsend_buffer;
	int echo_length = (elm327_config.kline_monoline == 1 ? kwp_bytes_count : 0);
	int data_length = 0;
	int headerSize = 1;

	static const int START_OFFSET = 3; // kline-specific 'kwp' prefix
	int offset = START_OFFSET;
	int offset_off = 0;

	uint8_t rps_prefix[8];
	uint8_t rps_prefix_size = 0;

	while (true) {
		const int64_t txtime_local = esp_timer_get_time();

		if (xQueueReceive(*xuart_rx_queue, &xsend_buffer, 5)) {
			totalMs += elapsedTimeMs(txtime_local);

			const int data_offset = echo_length;
			if (echo_length > 0) {
				// check echo bytes
				const int echo_offset = (kwp_bytes_count - echo_length);
				for (int i = 0, in = MIN(echo_length, xsend_buffer.usLen); i < in; ++i) {
					if (rsp[echo_offset + i] != xsend_buffer.ucElement[i]) {
						elm327_response("kwp_echo_mismatch_ CAN ERROR\r>", 0, q);
						close_skip_mode();
						return;
					}
				}

				if (echo_length > xsend_buffer.usLen) {
					echo_length -= xsend_buffer.usLen;
					xsend_buffer.usLen = 0;
				} else {
					xsend_buffer.usLen -= echo_length;
					echo_length = 0;

					rsp[0] = 0;
					strcat(rsp, "kwf"); // first packet
					offset = START_OFFSET;
				}
			}

			for (int i = 0; i < xsend_buffer.usLen; ++i) {
				offset += sprintf(rsp + offset, "%02X", xsend_buffer.ucElement[i + data_offset]);
				
				if (elm327_config.uds_rps_skip_size > 0) {
					if (rps_prefix_size < sizeof(rps_prefix)) {
						rps_prefix[rps_prefix_size] = xsend_buffer.ucElement[i + data_offset];
						rps_prefix_size += 1;
					}
				}

				if (data_length == 0) {
					// tries to get the length of data bytes of KWP message
					if (offset == START_OFFSET + headerSize * 2) {
						if (headerSize == 1) {
							const uint8_t byteFmt = xsend_buffer.ucElement[i + data_offset];
							const bool isShort = ((byteFmt & 0x3f) != 0x00);
							headerSize = (isShort ? 3 : 4);
							data_length = (isShort ? (byteFmt & 0x3f) : 0);
						} else {
							data_length = xsend_buffer.ucElement[i + data_offset];
						}
					}
				} else if (offset + offset_off >= START_OFFSET + (data_length + headerSize + 1) * 2) {
					// got the whole KWP message
					bool shouldSkip = false;
					if (elm327_config.uds_rps_skip_size > 0
							&& rps_prefix_size >= elm327_config.uds_rps_skip_size + headerSize) {
						shouldSkip = (memcmp(elm327_config.uds_rps_skip_prefix, rps_prefix + headerSize, elm327_config.uds_rps_skip_size) == 0);
					}

					if (!shouldSkip) {
						strcat(rsp, "\r");
						elm327_response(rsp, 0, q);
					}
					return;
				} else if (offset + 3 > MIN(sizeof(xsend_buffer.ucElement), sizeof(rsp))) {
					// got a part of KWP message (using the size of underling buffer)
					strcat(rsp, "\r");
					elm327_response(rsp, 0, q);

					offset_off += (offset - START_OFFSET);
					
					rsp[0] = 0;
					strcat(rsp, "kwp"); // rest packet(s)
					offset = START_OFFSET;
				}
			}
		}
		else
		{
			const TickType_t elapsedMs = elapsedTimeMs(txtime);
			
			if (elapsedMs >= totalMs) {
				ESP_LOGW(TAG, "response timeout = %lu ms", elapsedMs);

				if (offset == START_OFFSET) {
					if (echo_length > 0) {
						strcpy(rsp, (echo_length == kwp_bytes_count ? "kwp_no_echo_" : "kwp_incomplete_echo_"));
					}
					strcat(rsp, "NO DATA");
					close_skip_mode();
				}
				strcat(rsp, "\r>");
			
				elm327_response(rsp, 0, q);

				break;
			} else if (elm327_config.uds_rps_skip_size == 0 && elapsedMs >= totalMs / 10 && fnHasNewData()) {
				ESP_LOGW(TAG, "response reset by incoming data = %lu ms", elapsedMs);
				break;
			}
		}
	}
}

static void elm327_kline_baud(const char* command_str, QueueHandle_t *q)
{
	// restore spaces for sscanf from "BAUD 57600" / "BAUD 10400_0_8_0_1"
	char *p = (char*)&command_str[4];
	while ((p = strchr(p, '_')) != NULL) {
		*p = ' ';
	}

	int parity = 0, dataBits = 8, stopBits = 0, monoline = 1;
	const int fieldCount = sscanf(&command_str[4], "%d %d %d %d %d", &elm327_config.kline_baud, &parity, &dataBits, &stopBits, &monoline); // considers length of 'baud'
	if (fieldCount == 5) {
		elm327_config.kline_parity = parity;
		elm327_config.kline_data_bits = dataBits;
		elm327_config.kline_stop_bits = stopBits;
		elm327_config.kline_monoline = monoline;
	}

	if (wc_kline_set_baudrate(elm327_config.kline_baud, elm327_config.kline_parity, elm327_config.kline_data_bits, elm327_config.kline_stop_bits)) {
		elm327_response("OK\r>", 0, q);
	} else {
		elm327_response("kwp_baud_fails_ CAN ERROR\r>", 0, q);
	}
}

static void elm327_kline_request(const char *cmd, const size_t cmd_len, QueueHandle_t *q, int (*fnHasNewData)())
{
	if (!strncmp(cmd, "close", 5)) {
		wc_kline_update(false);
		return;
	}

	if (xuart_tx_queue == q) { // protection against simulteneous use of USB and KLine
		elm327_response("kwp_simulteneous_use_of_USB_and_KLine_ CAN ERROR\r>", 0, q);
		return;
	}

	wc_kline_update(true);

	if (!strncmp(cmd, "baud", 4)) {
		elm327_kline_baud(cmd, q);
 	} else {
		elm327_kline_send(cmd, cmd_len, q, fnHasNewData);
	}
}

static void elm327_comm_send(const char *cmd, const size_t cmd_len, QueueHandle_t *q, int (*fnHasNewData)())
{
	// restore spaces for sscanf from "data_expectedReplySize_timeoutMs"
	char *p = (char*)cmd;
	while ((p = strchr(p, '_')) != NULL) {
		*p = ' ';
	}

	size_t dataSize = 0;
	int expectedReplySize = 0;
	int timeoutMs = 0;

	p = (char*)cmd;
	if ((p = strchr(p, ' ')) != NULL) {
		*p = '\0';
		dataSize = (p - cmd) / 2;
		
		sscanf(p + 1, "%d %d", &expectedReplySize, &timeoutMs);
	} else {
		dataSize = cmd_len / 2;
	}

	// prepare COMM data
	const int kwp_bytes_count = MIN(dataSize, KWP_COMMAND_LENGHT);
	if (kwp_bytes_count == 0) {
		elm327_response("com_empty_message_ CAN ERROR\r>", 0, q);
		return;
	}

	char rsp[KWP_COMMAND_LENGHT];
	elm327_fill_data_from_hex_str(cmd, (uint8_t *) rsp, kwp_bytes_count);

	// clear incoming queue
	xQueueReset(*xuart_rx_queue);

	// send KWP message
	if (!elm327_response(rsp, kwp_bytes_count, xuart_tx_queue)) {
		elm327_response("com_send_fails_ CAN ERROR\r>", 0, q);
		notify_send_status(false);
		return;
	} else {
		notify_send_status(true);
	}

	if (expectedReplySize == 0) {
		return;
	}

	const int64_t endTime = esp_timer_get_time() + timeoutMs * 1000;

	xdev_buffer xsend_buffer;
	int echo_length = (elm327_config.kline_monoline == 1 ? kwp_bytes_count : 0);
	int offset = 0;

	int recvCount = 0;
	while (recvCount < abs(expectedReplySize)) {
		if (endTime < esp_timer_get_time()) {
			return; // timeout
		}
		if (fnHasNewData()) {
			return; // reset by incoming data
		}

		if (xQueueReceive(*xuart_rx_queue, &xsend_buffer, pdMS_TO_TICKS(timeoutMs))) {
			const int data_offset = echo_length;
			if (echo_length > 0) { // check echo bytes
				const int echo_offset = (kwp_bytes_count - echo_length);
				for (int i = 0, in = MIN(echo_length, xsend_buffer.usLen); i < in; ++i) {
					if (rsp[echo_offset + i] != xsend_buffer.ucElement[i]) {
						elm327_response("com_echo_mismatch_ CAN ERROR\r>", 0, q);
						return;
					}
				}

				if (echo_length > xsend_buffer.usLen) {
					echo_length -= xsend_buffer.usLen;
					continue;
				} else {
					xsend_buffer.usLen -= echo_length;
					echo_length = 0;

					rsp[0] = 0;
					strcat(rsp, "com");
					offset = 3;
				}
			}

			recvCount += xsend_buffer.usLen;

			for (int i = 0; i < xsend_buffer.usLen; ++i) {
				offset += sprintf(rsp + offset, "%02X", xsend_buffer.ucElement[i + data_offset]);
			}
		}
	}

	if (expectedReplySize < 0) { // read all remaining bytes
		while (xQueueReceive(*xuart_rx_queue, &xsend_buffer, 5)) {
			for (int i = 0; i < xsend_buffer.usLen; ++i) {
				offset += sprintf(rsp + offset, "%02X", xsend_buffer.ucElement[i]);
			}
		}
	}

	strcat(rsp, "\r");
	elm327_response(rsp, 0, q);
}

static void elm327_comm_request(const char *cmd, const size_t cmd_len, QueueHandle_t *q, int (*fnHasNewData)())
{
	if (xuart_tx_queue == q) { // protection against simulteneous use of USB and KLine
		elm327_response("com_simulteneous_use_of_USB_and_KLine_ CAN ERROR\r>", 0, q);
		return;
	}

	wc_kline_update(true);

	elm327_comm_send(cmd, cmd_len, q, fnHasNewData);
}

static void elm327_gps_request(const char *cmd, const size_t cmd_len, QueueHandle_t *q, int (*fnHasNewData)())
{
	if (strncmp(cmd, "off", 3) == 0) {
		gps_set_enabled(false, false); // pause 'nmea_rx_task'
		vTaskDelay(pdMS_TO_TICKS(20));
		
		wc_gps_update(false); // switch to USB (if not K-Line of course)
		return;
	}

	if (xuart_tx_queue == q) { // protection against simulteneous use of USB and GPS
		return;
	}

	const bool isDebugUnknowSentences = (strncmp(cmd, "dbg", 3) == 0);
	gps_set_enabled(true, isDebugUnknowSentences); // resume 'nmea_rx_task'
}

// API
bool elm327_process_perm_cmd(xdev_buffer *rx_buffer)
{
	if (elm327_config.perm_cmd_count == 0) {
		return false;
	}

	if (elm327_config.perm_cmd_index >= elm327_config.perm_cmd_count) {
		elm327_config.perm_cmd_index = 0;
	}

	const int offset = elm327_config.perm_cmd_index * CMD_LENGTH;
	const char *cmd_buffer = elm327_config.perm_cmd_list + offset;

	if (!strncmp(cmd_buffer, "kwp", 3)) {
		static const int DDLI_CML_LENGTH = 15; // 'kpw' + '8210F121C064'

		memcpy(rx_buffer->ucElement, cmd_buffer, DDLI_CML_LENGTH);
		memcpy(rx_buffer->ucElement + DDLI_CML_LENGTH, "\r", 1);
		rx_buffer->usLen = DDLI_CML_LENGTH + 1;
	} else {
		memcpy(rx_buffer->ucElement, cmd_buffer, CMD_LENGTH);
		memcpy(rx_buffer->ucElement + CMD_LENGTH, "\r", 1);
		rx_buffer->usLen = CMD_LENGTH + 1;
	}

	elm327_config.perm_cmd_index += 1;
	return true;
}

// API
bool elm327_process_idle_cmd(xdev_buffer *rx_buffer)
{
	if (elm327_config.uds_rps_skip_size == 1) { // K-Line
		memcpy(rx_buffer->ucElement, "kwp8210F13E01C2\r", 16);
		rx_buffer->usLen = 16;
		return true;
	} else if (elm327_config.uds_rps_skip_size == 2) { // CAN
		memcpy(rx_buffer->ucElement, "023E01AAAAAAAAAA1\r", CMD_LENGTH + 1);
		rx_buffer->usLen = CMD_LENGTH + 1;
		return true;
	} else {
		return false;
	}
}

// API
uint8_t elm327_perm_delay()
{
	if (elm327_config.perm_cmd_count == 0) {
		return 0; // means no permanent command to send
	}
	return MAX(elm327_config.perm_cmd_delay, 1);
}

// API
void clear_perm_commands(bool close_monitor_all)
{
	if (elm327_config.perm_cmd_count != 0) {
		elm327_config.perm_cmd_count = 0;
		elm327_config.perm_cmd_index = 0;
		elm327_config.perm_cmd_list[0] = 0;

		ESP_LOGI(TAG, "clear permanent commands");
	}

	if (close_monitor_all) {
		if (elm327_config.monitor_all) {
			elm327_config.monitor_all = 0;
			ESP_LOGW(TAG, "Monitor All is off");
		}
		close_skip_mode();
	}
}

static char* elm327_perm_send(const char* command_str)
{
	if (elm327_config.perm_cmd_count >= sizeof(elm327_config.perm_cmd_list) / CMD_LENGTH) {
		ESP_LOGE(TAG, "exeed maximum count of permanent commands = %d", (sizeof(elm327_config.perm_cmd_list) / CMD_LENGTH));
		return (char*)question_mark_str;
	}
	ESP_LOGI(TAG, "add permanent command at index = %d , delay = %d ms", elm327_config.perm_cmd_count, elm327_config.perm_cmd_delay);

	const int offset = elm327_config.perm_cmd_count * CMD_LENGTH;
	strncpy(elm327_config.perm_cmd_list + offset, command_str + 3, MIN(CMD_LENGTH, strlen(command_str + 3))); // considers length of 'prs'
	elm327_config.perm_cmd_count += 1;

	return ""; // skip reply
}

static char* elm327_perm_reset(const char* command_str)
{
	clear_perm_commands(false);

	elm327_config.perm_cmd_delay = elm327_parse_hex_str(command_str + 3, strlen(command_str + 3)); // considers length of 'prr'

	ESP_LOGI(TAG, "set permanent commands delay = %d ms", elm327_config.perm_cmd_delay);

	return ""; // skip reply
}

static char* elm327_uds_response_skip(const char* command_str)
{
	size_t data_size = MIN(strlen(command_str + 4) / 2, 8); // considers length of 'rsps'
	elm327_fill_data_from_hex_str(command_str + 4, elm327_config.uds_rps_skip_prefix, data_size);
	
	elm327_config.uds_rps_skip_size = data_size;
	elm327_config.uds_rps_last_rx_size = -1;
	elm327_config.uds_rps_last_count = 0;

	return (char*)ok_str;
}

static char* elm327_uds_response_all(const char* command_str)
{
	close_skip_mode();

	return (char*)ok_str;
}

static char* elm327_ping(const char* command_str)
{
	const int mode = elm327_parse_hex_str(command_str + 4, strlen(command_str + 4)); // considers length of 'ping'
	if (mode == 0) {
		return ""; // skip reply
	}
	return (char*)ok_str;
}

static void printRamUsage(char *buf)
{
	multi_heap_info_t info = {0};
	heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
	// info.total_free_bytes;   // total currently free in all non-continues blocks
	// info.minimum_free_bytes;  // minimum free ever
	// info.largest_free_block;   // largest continues block to allocate big array

	// ESP_LOGW("TAG", "total_free_bytes = %d , minimum_free_bytes = %d , largest_free_block = %d",
	//     info.total_free_bytes, info.minimum_free_bytes, info.largest_free_block);

	sprintf(buf, "(free: %d , min_free: %d)", info.total_free_bytes, info.minimum_free_bytes);
}


const xelm327_cmd_t elm327_commands[] = {
											{"spc", elm327_special_send},//special send (Abit specific - should be the first in the list)
											{"prs", elm327_perm_send},//send permanent command (Abit specific - should be the first in the list)
											{"prr", elm327_perm_reset},//reset permanent commands (Abit specific - should be the first in the list)
											{"rsps", elm327_uds_response_skip},//setup UDS response filtering (Abit specific - should be the first in the list)
											{"rspa", elm327_uds_response_all},//reset UDS response filtering (Abit specific - should be the first in the list)
											{"ping", elm327_ping},//client sends ping to tell that it's still on the line (Abit specific - should be the first in the list)

											{"fcsd", elm327_set_fc_data},// set the flow control data
											{"fcsh", elm327_set_fc_header},// set the flow control header
											{"fcsm", elm327_set_fc_mode}, // determine if the fc_data and/or fc_header is uses
											{"cfc", elm327_set_fc_enabled}, // CAN Flow Control off or on
											{"cra", elm327_set_receive_address},
											{"cp", elm327_set_priority_bits},// set five most significant bits of 29bit header
											{"sh", elm327_set_header},// set header to xyz, xx yy zz, or ww xx yy zz
											{"at", elm327_return_ok},//adaptive timing control
											{"sp", elm327_set_protocol},//set protocol to h and save as new default, 6, 7, 8, 9 or ah set protocol to auto, h
											{"rv", elm327_input_voltage},//read input voltage
											{"pc", elm327_return_ok},//close protocol
											{"st", elm327_set_timeout},//set timeout
											{"d", elm327_restore_defaults_or_display_dlc},//set all to defaults or change display DLC
											{"z", elm327_reset_all},// reset all/software reset
											{"ws", elm327_reset_all},// Warm Start (This command causes the ELM327 to perform a complete reset. It is very similar to the AT Z command, but does not include the power on LED test.)
											{"s", elm327_return_ok},// printing of spaces off or on
											{"e", elm327_set_echo},// echo off or on
											{"h", elm327_header_on_off},//headers off or on
											{"l", elm327_set_linefeed},//linefeeds off or on
											{"al", elm327_allow_long_messages},// Allow Long messages 
											{"caf", elm327_auto_formatting},// CAN Auto Formatting off or on
											{"csm", elm327_return_ok},// CAN Silent Monitoring off or on (TODO:)
											{"@", elm327_device_description},//display device description
											{"i", elm327_identify},//identify yourself
											{"ma", elm327_monitor_all},//monitor all on

											{NULL, NULL},
									};


void elm327_process_cmd(const uint8_t *buf, const uint8_t len, QueueHandle_t *q, int (*fnHasNewData)())
{
	// Because the cmd_buffer and cmd_len are static they keep their value
	// across multiple calls. So if a buf is an incomplete command the next
	// call will keep add to the cmd_buffer until the ending CR is found.
	static char cmd_buffer[KWP_COMMAND_LENGHT * 2 + 5]; // maximum KWP message (2 hex digits for each byte) + 'kwp' prefix + '\r'
	static uint16_t cmd_buffer_len = 0;
	char cmd_response[64];

	if (len > 4 && memcmp(buf, "AT WS", 5) == 0) {
		cmd_buffer_len = 0;
		gps_set_enabled(false, false); // pause 'nmea_rx_task'
	}

	for(int i = 0; i < len; i++)
	{
		assert(cmd_buffer_len + 1 < sizeof(cmd_buffer));

		if((buf[i] == '\r' && cmd_buffer_len > 0) || cmd_buffer_len + 1 >= sizeof(cmd_buffer))
		{
	//		ESP_LOGI(TAG, "end of command i: %d, cmd_buffer_len: %u", i, cmd_buffer_len);
			cmd_buffer[cmd_buffer_len] = 0;
			cmd_response[0] = 0;
			uint8_t cmd_found_flag = 0;

			if(!strncmp(cmd_buffer, "kwp", 3))
			{
				elm327_kline_request(cmd_buffer + 3, cmd_buffer_len - 3, q, fnHasNewData);
			}
			else if(!strncmp(cmd_buffer, "com", 3))
			{
				elm327_comm_request(cmd_buffer + 3, cmd_buffer_len - 3, q, fnHasNewData);
			}
			else if(!strncmp(cmd_buffer, "gps", 3))
			{
				elm327_gps_request(cmd_buffer + 3, cmd_buffer_len - 3, q, fnHasNewData);
			}
			else if(!strncmp(cmd_buffer, "at", 2))
			{
				for(int j = 0; elm327_commands[j].command != NULL; j++)
				{
					if(!strncmp(&cmd_buffer[2], elm327_commands[j].command, strlen(elm327_commands[j].command)))
					{
						char *ret_ptr = elm327_commands[j].command_interpreter(&cmd_buffer[2]);
						if(ret_ptr != 0)
						{
							strcat(cmd_response, ret_ptr);
							if (!strncmp(cmd_buffer+2, "ws", 2)) {
								printRamUsage(cmd_response + strlen(cmd_response));
							}
							cmd_found_flag = 1;
							ESP_LOGI(TAG, "cmd: %s, rsp: %s", elm327_commands[j].command, cmd_response);
							break;
						}

					}
				}

				if(!cmd_found_flag)
				{
					strcat(cmd_response, (char*)question_mark_str);
				}

				if (strlen(cmd_response) > 0) { // so that empty command will not reply
					strcat(cmd_response, "\r");
					if (elm327_config.linefeed) {
						strcat(cmd_response, "\n");
					}
					strcat(cmd_response, ">");

					elm327_response(cmd_response, 0, q);
				}

				if(cmd_buffer[2] == 'z')
				{
					// When the command is a reset ignore any other commands in the buffer.
					// This approach fixes an issue seen in the Carscanner Android app.
					// It might actually match a real ELM327 chip since the documentation
					// for the chip say an ATZ is like a full reset of the chip. So it
					// would make sense that a full reset would imply anything in the
					// incoming serial buffer would be cleared.
					//
					// In the Carscanner app a reset (ATZ) and an echo off (ATE0) are sent
					// in a single BLE message. Without the code below, elm327_process_cmd
					// would respond to the ATZ and the ATE0. When it does this,
					// Carscanner gets out of sync: the Carscanner log shows the next
					// command with a response from the previous command.
					cmd_buffer_len = 0;

					esp_restart();
					break;
				}
			}
			else // this is a request
			{
				if ((cmd_buffer_len % CMD_LENGTH) != 0) {
					ESP_LOGE(TAG, "cmd_len fail, len = %d, data = '%s'", cmd_buffer_len, cmd_buffer);
					//assert(false);
				}

				for (uint16_t j = 0; j + CMD_LENGTH <= cmd_buffer_len; j += CMD_LENGTH) {
					bool fc_less_mode = false;
					if (cmd_buffer[j] == '5' || cmd_buffer[j] == '6') {
						cmd_buffer[j] -= 4; // normalize Abit FC-less frames
						fc_less_mode = true;
					}

					cmd_response[0] = 0;
					elm327_request(cmd_buffer + j, CMD_LENGTH, fc_less_mode, cmd_response, q, fnHasNewData);
				}
			}

			cmd_buffer_len = 0;
		}
		else
		{
			if (isspace(buf[i])) {
				// To stop monitoring, simply send space character to the ELM327, then wait for it to respond with a prompt character ('>')
				if (cmd_buffer_len == 0) {
					if (elm327_config.monitor_all) {
						elm327_config.monitor_all = 0;
						ESP_LOGW(TAG, "Monitor All is off");

						elm327_response("\r>", 0, q);
					}
					close_skip_mode();
				}
			} else {
				cmd_buffer[cmd_buffer_len] = (char)tolower(buf[i]);
				cmd_buffer_len += 1;
			}
		}
	}
}

int8_t elm327_process_can_frame(const uint8_t *buf, twai_message_t *frame)
{
	const bool isElmFrame = elm327_should_receive(frame);
	bool printFame = false;

	// Let elm327 decide which messages to process
	if (isElmFrame) {
		if( elm327_can_log != NULL) {
			elm327_can_log(frame, ELM327_CAN_RX);
		}

		if (elm327_waiting_answer) {
			xQueueSend(can_rx_queue, frame, portMAX_DELAY);
		} else {
			printFame = true;
		}
	}
	else if (elm327_config.monitor_all) {
		printFame = true;
	}

	if (printFame) {
		char* rsp = (char*)buf;
		int offset = elm327_print_canid(rsp, frame);

		for (int i = 0; i < TWAI_FRAME_MAX_DLC; i++) {
			offset += sprintf(rsp + offset, "%02X", frame->data[i]);
		}
		strcat(rsp, "\r");
		offset += 1;

		if (isElmFrame) {
			ESP_LOGW(TAG, "CanTp: %s", rsp);
		}

		return offset;
	}
	return 0;
}

int elm327_print_canid(char *buff, twai_message_t *frame)
{
	if (frame->extd == 0) {
		return sprintf(buff, "%03X", (unsigned int)frame->identifier&TWAI_STD_ID_MASK);
	} else {
		return sprintf(buff, "%08X", (unsigned int)frame->identifier&TWAI_EXTD_ID_MASK);
	}
}

void elm327_init(bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q), void (*can_log)(twai_message_t* frame, uint8_t type), int terminal_resistor_led)
{
	elm327_set_default_config(true);
	elm327_response = send_to_host;
	elm327_can_log = can_log;
	can_rx_queue = xQueueCreate(RX_QUEUE_LENGTH * 4, sizeof(twai_message_t));
	terminalR_led = terminal_resistor_led;
}

void elm327_uart_init(QueueHandle_t *tx_queue, QueueHandle_t *rx_queue)
{
    xuart_tx_queue = tx_queue;
	xuart_rx_queue = rx_queue;
}
