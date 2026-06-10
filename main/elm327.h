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


#ifndef __ELM327__
#define __ELM327__

#include <hal/twai_types.h>

#define ELM327_CAN_RX   0x01
#define ELM327_CAN_TX   0x02

typedef struct __xdev_buffer xdev_buffer;

void elm327_init(bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q), void (*can_log)(twai_message_t* frame, uint8_t type), int terminal_resistor_led);
void elm327_process_cmd(const uint8_t *buf, const uint8_t len, QueueHandle_t *q, int (*fnHasNewData)());
bool elm327_process_perm_cmd(xdev_buffer *rx_buffer);
bool elm327_process_idle_cmd(xdev_buffer *rx_buffer);
uint8_t elm327_perm_delay();
void clear_perm_commands(bool close_monitor_all);
int8_t elm327_process_can_frame(const uint8_t *buf, twai_message_t *frame);
int elm327_print_canid(char *buff, twai_message_t *frame);

int elm327_rx_queue_size(QueueHandle_t *rx_queue);

void elm327_uart_init(QueueHandle_t *tx_queue, QueueHandle_t *rx_queue);

#endif
