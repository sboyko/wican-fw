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


#ifndef __TYPES_H__
#define __TYPES_H__

#define WICAN_RX_QUEUE_SIZE	100 // size of the input messages queue (used for all protocols)

#define DEV_BUFFER_LENGTH	64

typedef enum
{
	DEV_WIFI = 0,
	DEV_WIFI_WS,
	DEV_BLE,
	DEV_UART
} dev_channel_t;


typedef struct __xdev_buffer
{
	int usLen;
	uint8_t ucElement[DEV_BUFFER_LENGTH];
	dev_channel_t dev_channel;
} xdev_buffer;

// Updates status of recent send() task (both CAN and K-Line).
// Called from elm327 commands processor.
void notify_send_status(const bool sent);

// Updates connection status with 'channel'.
// Note that some channels cannot detect closing event (USB for example).
void notify_connection_closed(const dev_channel_t channel);

// Fills name (unique, based on MAC address) of this adapter
void fill_adapter_name(char* name);

// Prints (via ESP_LOGI) overall memory usage and the list of all tasksk with uxTaskGetStackHighWaterMark
void debug_mem_usage();

#endif
