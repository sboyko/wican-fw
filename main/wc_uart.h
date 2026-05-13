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

#ifndef __WC_UART_H__
#define __WC_UART_H__

void wc_uart_init(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, QueueHandle_t *kLineRX_Queue, uint8_t connected_led, uint8_t kline_led);

// Set baud rate for KLine
bool wc_kline_baudrate(const int baudRate, const int parity, const int dataBits, const int stopBits);

/**
 * Set (periodically) KLine using mode, results in switching LED on.
 * KLine using mode will be automatically turned off when not called for more then 10 seconds.
 */
void wc_kline_enable(bool isOnNotOff);

#endif
