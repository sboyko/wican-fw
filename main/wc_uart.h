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

void wc_uart_init(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, QueueHandle_t *kLineRX_Queue, uint8_t kline_gps_led);


// Set baud rate for KLine
bool wc_kline_set_baudrate(const int baudRate, const int parity, const int dataBits, const int stopBits);

/**
 * Set (periodically) KLine using mode, results in switching 'kline_gps_led' to K-Line mode.
 * KLine using mode will be automatically turned off (to USB mode) when not called for more then 10 seconds.
 */
void wc_kline_update(bool isOnNotOff);

// checks that K-Line is active right now
bool wc_kline_active();


// Set baud rate for GPS
bool wc_gps_set_baudrate(const int baudRate, const int parity, const int dataBits, const int stopBits);

/**
 * Set (periodically) GPS using mode, results in switching 'kline_gps_led' to GPS mode.
 * GPS using mode will be automatically turned off (to USB mode) when not called for more then 10 seconds.
 */
void wc_gps_update(bool isOnNotOff);

// checks that GPS is active right now
bool wc_gps_active();

#endif
