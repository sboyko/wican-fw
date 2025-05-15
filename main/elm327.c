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
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "can.h"
#include "sleep_mode.h"
#include "elm327.h"

#include <ctype.h>
#include <string.h>

#define TAG 		__func__

static QueueHandle_t *can_rx_queue = NULL;

const char *ok_str = "OK";
const char *question_mark_str = "?";
const char *device_description = "ELM327 v1.3a meatPi";
const char *identify = "OBDLink MX";


void (*elm327_response)(char*, uint32_t, QueueHandle_t *q);
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
	uint8_t protocol;
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

	uint8_t perm_cmd_count;
	uint8_t perm_cmd_index;
	char perm_cmd_list[85]; // each permanent command is of CMD_LENGTH
	uint8_t perm_cmd_delay; // delay between two consecutive permanent commands in ms

}_xelm327_config_t;


static const uint8_t CMD_LENGTH = 17; // Single Frame + 1 byte for 'req_expected_rsp'
static char cmd_response[128];

static _xelm327_config_t elm327_config;

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
		elm327_config.protocol = '6';
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
}

typedef char* (*elm327_command_callback)(const char* command_str);
typedef struct _xelm327_cmd
{
	const char * const command;
	const elm327_command_callback command_interpreter;
}xelm327_cmd_t;


static esp_err_t can_tx_task(twai_message_t *message, TickType_t ticks_to_wait)
{
	ESP_LOGI(TAG, "%08X%c  %02X %02X %02X %02X %02X %02X %02X %02X",
		(unsigned int)(message->identifier&TWAI_EXTD_ID_MASK), (message->extd ? 'x' : ' '),
		(unsigned int)message->data[0], (unsigned int)message->data[1], (unsigned int)message->data[2],
		(unsigned int)message->data[3], (unsigned int)message->data[4], (unsigned int)message->data[5],
		(unsigned int)message->data[6], (unsigned int)message->data[7]);

	const esp_err_t result = can_send(message, ticks_to_wait);
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "can_send() fails: %d", result);
	}
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
		ESP_LOGI(TAG, "Monitor All is on");
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

	return can_tx_task(&txframe, 1);
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
		while( xQueueReceive(*can_rx_queue, ( void * ) &rx_frame, 0) ) {
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

static char* elm327_set_protocol(const char* command_str)
{
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

	return (char*)ok_str;
}


static char* elm327_set_timeout(const char* command_str)
{
	elm327_config.req_timeout = (strtol((char *) &command_str[2], NULL, 16) & 0xFFFF); // allows to use 2-byte timeout

	if(elm327_config.req_timeout == 0)
	{
		elm327_config.req_timeout = 0x32;
	}

	ESP_LOGI(TAG, "req_timeout = %lu ms", (TickType_t)((elm327_config.req_timeout*4.096) / portTICK_PERIOD_MS));

	return (char*)ok_str;
}

static char hex_to_num(char a)
{
	char x = a;
	if(x >= 'a')
		x = x - 'a' + 10;
	// Uppercase letters
	else if(x >= 'A')
		x = x - 'A' + 10;
	// Numbers
	else
		x = x - '0';

	return x;
}
static char* elm327_describe_protocol(const char* command_str)
{
	static char protocol[10][50] = {
								"AUTO",
								"SAE J1850 PWM",	//1
								"SAE J1850 VPW",	//2
								"ISO 9141-2",		//3
								"ISO 14230-4 KWP/5",	//4
								"ISO 14230-4 KWP",		//5
								"ISO 15765-4 CAN (11 bit ID, 500 kbaud)",	//6
								"ISO 15765-4 CAN (29 bit ID, 500 kbaud)",	//7
								"ISO 15765-4 CAN (11 bit ID, 250 kbaud)",	//8
								"ISO 15765-4 CAN (29 bit ID, 250 kbaud)"};	//9

	uint8_t protocool_number = (uint8_t)hex_to_num(elm327_config.protocol);
	if(protocool_number >= 10)
	{
		return 0;
	}
	return (char*)&protocol[protocool_number][0];
}

static char* elm327_describe_protocol_num(const char* command_str)
{
	static char protocol_number_str[3];

	sprintf( protocol_number_str, "%c", elm327_config.protocol);

	return protocol_number_str;
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
	if(!elm327_config.header_is_set) {
		switch(elm327_config.protocol) {
			case '6':
			case '8':
				// return 0x7E0
				return 0x7DF;
			case '7':
			case '9':
				// return 0x18DAF10A;
				return 0x18DB33F1;
			default:
				// In theory this line shouldn't be hit,
				// but just in case return something reasonable
				return 0x7DF;
		}
	} else {
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
			//ESP_LOGW(TAG, "skip by rx_address %08X%c", (unsigned int)(rx_frame->identifier&TWAI_EXTD_ID_MASK), (rx_frame->extd ? 'x' : ' '));
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

	if( elm327_can_log != NULL)
	{
		elm327_can_log(&txframe, ELM327_CAN_TX);
	}
	
	can_tx_task(&txframe, 1);
}

static TickType_t elapsedTimeMs(int64_t txtime)
{
	return (TickType_t)(((esp_timer_get_time() - txtime)/1000)/portTICK_PERIOD_MS);
}

 __attribute__((optimize("O0"))) static int8_t elm327_request(char *cmd, size_t cmd_len, char *rsp, QueueHandle_t *queue, bool (*fnHasNewData)())
{
	static int rsp_nowait_count = 0;

	//ESP_LOG_BUFFER_HEX(TAG, cmd, cmd_len);

	if((elm327_config.protocol != '6') && (elm327_config.protocol != '8') && (elm327_config.protocol != '7') && (elm327_config.protocol != '9'))
	{
		if(elm327_config.protocol == '1' || elm327_config.protocol == '2') {
			strcat(rsp, "NO DATA\r\r>");
		} else {
			strcat(rsp, "BUS INIT: ...ERROR\r\r>");
		}

		elm327_response(rsp, 0, queue);
		return 0;
	}

	twai_message_t txframe;
	txframe.identifier = elm327_get_identifier();
	txframe.extd = elm327_config.protocol == '7' || elm327_config.protocol == '9';

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

	uint8_t cmd_data_length = cmd_len / 2;
	if(elm327_config.allow_long_messages && cmd_data_length == 8)
	{
		elm327_fill_data_from_hex_str(cmd, &txframe.data[0], cmd_data_length);
	}
	else if(cmd_data_length > 7)
	{
		// commands can't be longer than 7 bytes unless flow control is used
		// FIXME: this should use the linefeed setting and match the number of
		// `\r`s that are normally sent.
		strcat(rsp, "?\r>");
		elm327_response(rsp, 0, queue);
		return 0;
	}
	else
	{
		txframe.data[0] = cmd_data_length;
		elm327_fill_data_from_hex_str(cmd, &txframe.data[1], cmd_data_length);
	}

	if( elm327_can_log != NULL)
	{
		elm327_can_log(&txframe, ELM327_CAN_TX);
	}

	twai_message_t rx_frame;
	if (req_expected_rsp != 0) {
		while( xQueueReceive(*can_rx_queue, ( void * ) &rx_frame, 0) ) {
			// cleanup before new send request
			ESP_LOGW(TAG, "skip before send %08X%c", (unsigned int)(rx_frame.identifier&TWAI_EXTD_ID_MASK), (rx_frame.extd ? 'x' : ' '));
		}
	}

	can_tx_task(&txframe, 1);

	if (req_expected_rsp == 0) {
		//strcat((char*)rsp, "\r>");
		//elm327_response(rsp, 0, queue);
		rsp_nowait_count += 1;
		if (rsp_nowait_count >= TX_QUEUE_LENGTH) {
			rsp_nowait_count = 0;
			vTaskDelay(pdMS_TO_TICKS(1));
		}

		return 0;
	}
	rsp_nowait_count = 0;

	TickType_t totalMs = (elm327_config.req_timeout*4.096) / portTICK_PERIOD_MS;
	const int64_t txtime = esp_timer_get_time();
	uint8_t rsp_found = 0;
	uint8_t number_of_rsp = 0;

	uint8_t sendControlFrame = false;

	while (true)
	{
		const int64_t txtime_local = esp_timer_get_time();

		if (xQueueReceive(*can_rx_queue, ( void * ) &rx_frame, 5)) {
			totalMs += elapsedTimeMs(txtime_local);

			if( elm327_can_log != NULL)
			{
				elm327_can_log(&rx_frame, ELM327_CAN_RX);
			}
			//reset timeout after response is received
			rsp_found = 1;
			number_of_rsp++;

			// Identify what kind of frame this is.
			int rx_frame_data_length = 0;
			uint8_t frame_type = rx_frame.data[0] & 0xF0;
			if (frame_type == 0x10)
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
				sendControlFrame = true;
				const uint16_t expectedLength = ((0x0F & rx_frame.data[0]) << 8 | rx_frame.data[1]);
				req_expected_rsp = (uint8_t)(expectedLength / 7) + 1;
			}
			else if (frame_type == 0x20)
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

			uint8_t data_offset = 0;

			// Based on the "CAF0 AND CAF1" section of the ELM doc, if headers are shown
			// the PCI byte(s) (usually just data[0]) should be printed.
			if(elm327_config.show_header)
			{
				if(rx_frame.extd == 0) {
					sprintf(rsp, "%03lX", rx_frame.identifier&TWAI_STD_ID_MASK);
				} else {
					sprintf(rsp, "%08lX", rx_frame.identifier&TWAI_EXTD_ID_MASK);
				}
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

			bool rsp_complete = (req_expected_rsp != 0xFF && req_expected_rsp == number_of_rsp);
			if (rsp_complete) {
				strcat(rsp, ">");
			}

			elm327_response(rsp, 0, queue);
			rsp[0] = 0;

			if (rsp_complete) {
				ESP_LOGI(TAG, "response time = %lu ms", elapsedTimeMs(txtime));
				break;
			}

			static const uint8_t BS_MAX = RX_QUEUE_LENGTH + 1; // esp32-can fails to receive more then (.rx_queue_len + 1) simultaneous CAN messages
			if (sendControlFrame && ((number_of_rsp - 1) % BS_MAX) == 0) {
				memset(txframe.data, 0xAA, 8);

				txframe.data[0] = 0x30;
				txframe.data[1] = BS_MAX;
				txframe.data[2] = 0x00; // zero duration between two consecutive frames

				can_tx_task(&txframe, 1);
			}
		}
		else
		{
			const TickType_t elapsedMs = elapsedTimeMs(txtime);
			
			if (elapsedMs >= totalMs) {
				ESP_LOGW(TAG, "response timeout = %lu ms", elapsedMs);

				if (rsp_found == 0) {
					strcat(rsp, "NO DATA\r\r>");
				} else {
					strcat(rsp, "\r>");
				}
			
				elm327_response(rsp, 0, queue);

				break;
			} else if (fnHasNewData()) {
				ESP_LOGW(TAG, "response reset by incoming data = %lu ms", elapsedMs);
				break;
			}
		}
	}

	return 0;
}

static char* elm327_perm_send(const char* command_str)
{
	if (elm327_config.perm_cmd_count >= sizeof(elm327_config.perm_cmd_list) / CMD_LENGTH) {
		ESP_LOGE(TAG, "exeed maximum count of permanent commands = %d", (sizeof(elm327_config.perm_cmd_list) / CMD_LENGTH));
		return (char*)question_mark_str;
	}
	ESP_LOGI(TAG, "add permanent command at index = %d , delay = %d ms", elm327_config.perm_cmd_count, elm327_config.perm_cmd_delay);

	const int offset = elm327_config.perm_cmd_count * CMD_LENGTH;
	strcat(elm327_config.perm_cmd_list + offset, command_str + 3); // considers length of 'prs'
	elm327_config.perm_cmd_count += 1;

	return ""; // skip reply
}

static char* elm327_perm_reset(const char* command_str)
{
	elm327_config.perm_cmd_count = 0;
	elm327_config.perm_cmd_index = 0;
	elm327_config.perm_cmd_list[0] = 0;
	elm327_config.perm_cmd_delay = elm327_parse_hex_str(command_str + 3, strlen(command_str + 3)); // considers length of 'prr'

	ESP_LOGI(TAG, "clear permanent commands, delay = %d ms", elm327_config.perm_cmd_delay);

	return ""; // skip reply
}

uint8_t elm327_perm_delay()
{
	if (elm327_config.perm_cmd_count == 0) {
		return 0; // means no permanent command to send
	}
	return MAX(elm327_config.perm_cmd_delay, 1);
}

void elm327_process_perm_cmd(QueueHandle_t *q, bool (*fnHasNewData)())
{
	if (elm327_config.perm_cmd_count == 0) {
		return;
	}

	if (elm327_config.perm_cmd_index >= elm327_config.perm_cmd_count) {
		elm327_config.perm_cmd_index = 0;
	}

	const int offset = elm327_config.perm_cmd_index * CMD_LENGTH;
	cmd_response[0] = 0;
	elm327_request(elm327_config.perm_cmd_list + offset, CMD_LENGTH, cmd_response, q, fnHasNewData);

	elm327_config.perm_cmd_index += 1;
}


const xelm327_cmd_t elm327_commands[] = {
											{"spc", elm327_special_send},//special send (Abit specific - should be the first in the list)
											{"prs", elm327_perm_send},//send permanent command (Abit specific - should be the first in the list)
											{"prr", elm327_perm_reset},//reset  permanent commands (Abit specific - should be the first in the list)

											{"fcsd", elm327_set_fc_data},// set the flow control data
											{"fcsh", elm327_set_fc_header},// set the flow control header
											{"fcsm", elm327_set_fc_mode}, // determine if the fc_data and/or fc_header is uses
											{"cfc", elm327_set_fc_enabled}, // CAN Flow Control off or on
											{"dpn", elm327_describe_protocol_num},//describe protocol by number
											{"cra", elm327_set_receive_address},
											{"cp", elm327_set_priority_bits},// set five most significant bits of 29bit header
											{"dp", elm327_describe_protocol},//describe current protocol
											{"sh", elm327_set_header},// set header to xyz, xx yy zz, or ww xx yy zz
											{"at", elm327_return_ok},//adaptive timing control
											{"sp", elm327_set_protocol},//set protocol to h and save as new default, 6, 7, 8, 9
																	 // or ah	set protocol to auto, h
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


void elm327_process_cmd(uint8_t *buf, uint8_t len, QueueHandle_t *q, bool (*fnHasNewData)())
{
	// Because the cmd_buffer and cmd_len are static they keep their value
	// across multiple calls. So if a buf is an incomplete command the next
	// call will keep add to the cmd_buffer until the ending CR is found.
	static char cmd_buffer[100];
	static uint16_t cmd_len = 0;
	uint8_t cmd_found_flag = 0;

	for(int i = 0; i < len; i++)
	{
		if((buf[i] == '\r' && cmd_len > 0) || cmd_len + 2 > sizeof(cmd_buffer))
		{
	//		ESP_LOGI(TAG, "end of command i: %d, cmd_len: %u", i, cmd_len);
			cmd_buffer[cmd_len] = 0;
			cmd_response[0] = 0;
			cmd_found_flag = 0;

			if(!strncmp(cmd_buffer, "at", 2))
			{
				for(int j = 0; elm327_commands[j].command != NULL; j++)
				{
					if(!strncmp(&cmd_buffer[2], elm327_commands[j].command, strlen(elm327_commands[j].command)))
					{
						char *ret_ptr = elm327_commands[j].command_interpreter(&cmd_buffer[2]);
						if(ret_ptr != 0)
						{
							strcat(cmd_response, ret_ptr);
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

				if (strlen(cmd_response) > 0) { // so that 'AT MA' command will not reply
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
					cmd_len = 0;
					break;
				}
			}
			else // this is a request
			{
				for (uint16_t j = 0; j < cmd_len; j += CMD_LENGTH) {
					if (cmd_buffer[j] == '5' || cmd_buffer[j] == '6') {
						cmd_buffer[j] -= 4; // normalize Abit FC-less frames
					}

					cmd_response[0] = 0;
					elm327_request(cmd_buffer + j, CMD_LENGTH, cmd_response, q, fnHasNewData);
				}
			}

			cmd_len = 0;
		}
		else
		{
			//clear queue before sending command
			if (isspace(buf[i])) {
				if (elm327_config.monitor_all) {
					// To stop monitoring, simply send any single character to the ELM327, then wait for it to respond with a prompt character ('>')
					elm327_config.monitor_all = 0;
			
					elm327_response("\r>", 0, q);

					ESP_LOGW(TAG, "Monitor All is off");
				}
			} else {
				cmd_buffer[cmd_len++] = (char)tolower(buf[i]);
			}
		}
	}
}

int8_t elm327_process_can_frame(uint8_t *buf, twai_message_t *frame)
{
	// Let elm327.c decide which messages to process
	if (elm327_should_receive(frame)) {
		if (xQueueSend(*can_rx_queue, frame, pdMS_TO_TICKS(0)) != pdTRUE) {
			ESP_LOGE(TAG, "fails to queue %08X%c", (unsigned int)(frame->identifier&TWAI_EXTD_ID_MASK), (frame->extd ? 'x' : ' '));
		}
	} else if (elm327_config.monitor_all) {
		char* rsp = (char*)buf;
		int offset = 0;

		if (frame->extd == 0) {
			offset += sprintf(rsp, "%03lX", frame->identifier&TWAI_STD_ID_MASK);
		} else {
			offset += sprintf(rsp, "%08lX", frame->identifier&TWAI_EXTD_ID_MASK);
		}

		for (int i = 0; i < TWAI_FRAME_MAX_DLC; i++) {
			offset += sprintf(rsp + offset, "%02X", frame->data[i]);
		}
		strcat(rsp, "\r");
		offset += 1;

		return offset;
	}
	
	return 0;
}

void elm327_init(void (*send_to_host)(char*, uint32_t, QueueHandle_t *q), QueueHandle_t *rx_queue, void (*can_log)(twai_message_t* frame, uint8_t type))
{
	elm327_set_default_config(true);
	elm327_response = send_to_host;
	can_rx_queue = rx_queue;
	elm327_can_log = can_log;
}
