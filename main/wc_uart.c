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
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "elm327.h"
#include "wc_uart.h"
#include "gps_common.h"


typedef enum {
    UART_USB = 1,
    UART_KLINE = 2,
    UART_GPS = 3,
} UartMode;

static const int UART_USB_BAUDRATE = 2400000; //460800
static const int UART_RX_BUF_SIZE = 1024;
static const int RX_QUEUE_WAIT_TIME_MS = 10;
#define KWP_COMMAND_LENGHT 260 // maximum KWP message length in bytes (including header and CS)

static const uart_port_t uart_num = UART_NUM_0;

static QueueHandle_t *xuart_tx_queue = NULL, *xuart_rx_queue = NULL, *kline_rx_queue = NULL;
static QueueHandle_t gps_rx_queue;
static QueueHandle_t uart0_queue;
static int kline_gps_led = GPIO_NUM_NC; // not connected

static UartMode request_uart_mode = UART_USB;
static int request_uart_baud = 0;
static int64_t uart_mode_time = 0;

static const uart_config_t usb_uart_config = {
    .baud_rate = UART_USB_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static uart_config_t kline_uart_config = {
    .baud_rate = 0,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static uart_config_t gps_uart_config = {
    .baud_rate = 0,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static void setup_uart_usb(const uart_config_t* const uart_config)
{
    // We won't use a buffer for sending data.
    //uart_driver_install(uart_num, UART_RX_BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_LEVEL1);
    int ret = uart_driver_install(uart_num, UART_RX_BUF_SIZE * 2, 0, 10, &uart0_queue, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM); // note that 'CONFIG_UART_ISR_IN_IRAM=y' in config
	if (ret != ESP_OK) {
        ESP_LOGE(__func__, "uart_driver_install() fails (%d)", ret);
    }

    uart_param_config(uart_num, uart_config);

    // // Enable UART RX FIFO full threshold interrupts
    // uart_enable_intr_mask(uart_num, UART_INTR_RXFIFO_FULL|UART_INTR_RXFIFO_OVF);

//																					output,			input
//    esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);
//    uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 2, 10);
    uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void close_uart_usb()
{
    uart_driver_delete(uart_num);
}

static void reset_uart(int baudRate)
{
    /*
    close_uart_usb();
    vTaskDelay(pdMS_TO_TICKS(10));
    setup_uart_usb(baudRate);
    vTaskDelay(pdMS_TO_TICKS(10));
    */
}

static void uart_rx_task(void *arg)
{
	xdev_buffer io_buffer;
    uart_event_t event;
    char ws_data[KWP_COMMAND_LENGHT];

    UartMode uart_mode = UART_KLINE;
    int uart_baud = UART_USB_BAUDRATE;

    int failed_waits = 0;
    int wait_ms = 2;
    int received_bytes = 0;
    bool uart_wait_mode = false;

    while (true) {
        if (uart_mode != UART_USB
                && (uart_mode_time == 0 || esp_timer_get_time() - uart_mode_time > 10*1000*1000)) {
            request_uart_mode = UART_USB;
            request_uart_baud = UART_USB_BAUDRATE;
        }
        if (uart_mode != request_uart_mode) {
            uart_mode = request_uart_mode;
            
            gpio_set_level(kline_gps_led, uart_mode == UART_KLINE ? 0 : 1);
        }
        if (uart_baud != request_uart_baud) {
            uart_baud = request_uart_baud;

            close_uart_usb();

            switch (uart_mode) {
                case UART_KLINE: setup_uart_usb(&kline_uart_config); break;
                case UART_GPS: setup_uart_usb(&gps_uart_config); break;
                default: setup_uart_usb(&usb_uart_config);
            }
        }

     	if (xQueueReceive(*xuart_tx_queue, &io_buffer, pdMS_TO_TICKS(uart_mode == UART_USB ? 0 : 1))) {
            failed_waits = 0;

            int offset = 0;
            while (true) {
                memcpy(ws_data + offset, io_buffer.ucElement, io_buffer.usLen);
                offset += io_buffer.usLen;

                if (xQueuePeek(*xuart_tx_queue, &io_buffer, 0)
                        && offset + io_buffer.usLen < sizeof(ws_data)) {
                    if (xQueueReceive(*xuart_tx_queue, &io_buffer, 0) != pdTRUE) {
                        break;
                    }
                } else {
                    break;
                }
            }

			int to_write = offset;
			while (to_write > 0)
			{
				int written = uart_write_bytes(uart_num, ws_data + (offset - to_write), to_write);
				if (written < 0) {
                    reset_uart(uart_baud);
                    break;
				}
				to_write -= written;
			}
        }

        while (xQueueReceive(uart0_queue, &event, pdMS_TO_TICKS(wait_ms))) {
            if (event.type == UART_DATA && event.size > 0) { // got new data
                failed_waits = 0;

                io_buffer.dev_channel = DEV_UART;
                received_bytes += event.size;

                int offset = 0;
                while (offset < event.size) {
                    io_buffer.usLen = MIN(event.size - offset, sizeof(io_buffer.ucElement));
                    io_buffer.usLen = uart_read_bytes(uart_num, io_buffer.ucElement, io_buffer.usLen, 0);
                    if (io_buffer.usLen <= 0) {
                        if (io_buffer.usLen < 0) {
                            reset_uart(uart_baud);
                        }
                        break;
                    }

                    if (uart_mode == UART_KLINE) {
                        xQueueSend(*kline_rx_queue, &io_buffer, pdMS_TO_TICKS(10));
                    } else if (uart_mode == UART_GPS) {
                        xQueueSend(gps_rx_queue, &io_buffer, pdMS_TO_TICKS(10));
                    } else {
                        if (xQueueSend(*xuart_rx_queue, &io_buffer, pdMS_TO_TICKS(1)) != pdTRUE) {
#ifndef NDEBUG
                            sprintf(ws_data, "overflow!!\r");
                            uart_write_bytes(uart_num, ws_data, strlen(ws_data));
#endif                            
                            break;
                        }
                    }
                    offset += io_buffer.usLen;
                }

                if (uart_mode == UART_USB && received_bytes > UART_RX_BUF_SIZE / 2) {
                    received_bytes = 0;

                    const int size = elm327_rx_queue_size(xuart_rx_queue);
                    if (size >= 0) {
                        sprintf(ws_data, "WAIT_%d\r", size);
                        uart_write_bytes(uart_num, ws_data, strlen(ws_data));
                        uart_wait_mode = (size > WICAN_RX_QUEUE_SIZE / 2);
                    }
                }

                //vTaskDelay(pdMS_TO_TICKS(1)); // give a chance for CAN to process incoming commands
                break;
            }
            // else if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
            //     reset_uart(uart_baud);
            //     break;
            // }
        }

        if (uart_wait_mode) {
            const int size = elm327_rx_queue_size(xuart_rx_queue);
            if (size >= 0) {
                sprintf(ws_data, "WAIT_%d\r", size);
                uart_write_bytes(uart_num, ws_data, strlen(ws_data));
                uart_wait_mode = (size > WICAN_RX_QUEUE_SIZE / 2);

                vTaskDelay(pdMS_TO_TICKS(1)); // prevents spamming
            }
        }
        
        wait_ms = (++failed_waits > 1500) ? RX_QUEUE_WAIT_TIME_MS : (uart_mode == UART_USB ? 2 : 1);
    }
}

/*
static void uart_tx_task(void *arg)
{
    xdev_buffer tx_buffer;

    while (1)
    {
    	if (xQueueReceive(*xuart_tx_queue, &tx_buffer, portMAX_DELAY)) {
        	if (uart_write_bytes(uart_num, tx_buffer.ucElement, tx_buffer.usLen) != tx_buffer.usLen) {
                assert(false);
            }
        }

//    	rx_buffer.usLen = uart_read_bytes(uart_num, rx_buffer.ucElement, UART_RX_BUF_SIZE, 1 / portTICK_PERIOD_MS);
//    	rx_buffer.dev_channel = DEV_UART;
//    	if(rx_buffer.usLen > 0)
//    	{
//    		xQueueSend(*xuart_rx_queue, ( void * ) &rx_buffer, portMAX_DELAY );
//    	}
    }
}
*/

// API
void wc_uart_init(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, QueueHandle_t *kLineRX_Queue, uint8_t kline_led)
{
    xuart_tx_queue = xTXp_Queue;
	xuart_rx_queue = xRXp_Queue;
    kline_rx_queue = kLineRX_Queue;
    gps_rx_queue = xQueueCreate(10, sizeof( xdev_buffer) ); // GPS RX queue
    kline_gps_led = kline_led;

    gps_serial_init(xuart_tx_queue, &gps_rx_queue);

    setup_uart_usb(&usb_uart_config);

    // Note: looks like one task is faster then two separate tasks
    //
    //xTaskCreate(uart_tx_task, "uart_tx_task", 1024*2, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*4, NULL, 5, NULL);
}

// API
bool wc_kline_set_baudrate(const int baudRate, const int parity, const int dataBits, const int stopBits)
{
    kline_uart_config.baud_rate = baudRate;
    kline_uart_config.parity = (parity == 0 ? UART_PARITY_DISABLE : (parity == 1 ? UART_PARITY_ODD : UART_PARITY_EVEN));
    kline_uart_config.data_bits = (dataBits == 7 ? UART_DATA_7_BITS : UART_DATA_8_BITS);
    kline_uart_config.stop_bits = (stopBits == 0 ? UART_STOP_BITS_1 : UART_STOP_BITS_2);

    wc_kline_update(true);
    request_uart_baud = baudRate;

    // 'rx_task' should have a chance to process
    vTaskDelay(pdMS_TO_TICKS(RX_QUEUE_WAIT_TIME_MS));

    return true;

    // if (uart_set_baudrate(uart_num, baudRate) == ESP_OK) {
    //     vTaskDelay(pdMS_TO_TICKS(5));

    //     uint32_t actualBaudRate = 0;
    //     if (uart_get_baudrate(uart_num, &actualBaudRate) == ESP_OK && actualBaudRate == baudRate) {
    //         return true;
    //     }
    // }
    // return false;
}

// API
void wc_kline_update(bool isOnNotOff)
{
    if (isOnNotOff) {
        uart_mode_time = esp_timer_get_time();
        request_uart_mode = UART_KLINE;
    } else {
        uart_mode_time = 0;
    }
}

// API
bool wc_kline_active()
{
    return request_uart_mode == UART_KLINE;
}

// API
bool wc_gps_set_baudrate(const int baudRate, const int parity, const int dataBits, const int stopBits)
{
    if (wc_kline_active()) {
        return false;
    }

    gps_uart_config.baud_rate = baudRate;
    gps_uart_config.parity = (parity == 0 ? UART_PARITY_DISABLE : (parity == 1 ? UART_PARITY_ODD : UART_PARITY_EVEN));
    gps_uart_config.data_bits = (dataBits == 7 ? UART_DATA_7_BITS : UART_DATA_8_BITS);
    gps_uart_config.stop_bits = (stopBits == 0 ? UART_STOP_BITS_1 : UART_STOP_BITS_2);

    wc_gps_update(true);
    request_uart_baud = baudRate;

    // 'rx_task' should have a chance to process
    vTaskDelay(pdMS_TO_TICKS(RX_QUEUE_WAIT_TIME_MS));

    return true;
}

// API
void wc_gps_update(bool isOnNotOff)
{
    if (wc_kline_active()) {
        return;
    }

    if (isOnNotOff) {
        uart_mode_time = esp_timer_get_time();
        request_uart_mode = UART_GPS;
    } else {
        uart_mode_time = 0;
    }
}

bool wc_gps_active()
{
    return request_uart_mode == UART_GPS;
}
