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
#include "esp_system.h"
//#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "types.h"
#include "lwip/sockets.h"

static const int RX_BUF_SIZE = 1024;

static QueueHandle_t *xuart_tx_queue, *xuart_rx_queue;
static QueueHandle_t uart0_queue;

static void uart_rx_task(void *arg)
{
	xdev_buffer io_buffer;
    uart_event_t event;
    char ws_data[DEV_BUFFER_LENGTH * 2];

    int failed_waits = 0;
    int wait_ms = 1;

    while (true) {
     	if (xQueueReceive(*xuart_tx_queue, &io_buffer, 0)) {
            failed_waits = 0;

            int offset = 0;
            while (true) {
                memcpy(ws_data + offset, io_buffer.ucElement, io_buffer.usLen);
                offset += io_buffer.usLen;

                if (xQueuePeek(*xuart_tx_queue, &io_buffer, pdMS_TO_TICKS(0))
                        && offset + io_buffer.usLen < sizeof(ws_data)) {
                    if (xQueueReceive(*xuart_tx_queue, &io_buffer, 0) != pdTRUE) {
                        //ESP_LOGE(TAG, "xQueueReceive() fails");
                        assert(false);
                    }
                } else {
                    break;
                }
            }

			int to_write = offset;
			while (to_write > 0)
			{
				int written = uart_write_bytes(UART_NUM_0, ws_data + (offset - to_write), to_write);
				if (written < 0) {
                    //ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    assert(false);
                    break;
				}
				to_write -= written;
			}
        }

        while(xQueueReceive(uart0_queue, &event, pdMS_TO_TICKS(wait_ms))) {
            if (event.type == UART_DATA && event.size > 0) { // got new data
                failed_waits = 0;

                io_buffer.dev_channel = DEV_UART;
                int offset = 0;

                while (offset < event.size) {
                    io_buffer.usLen = MIN(event.size - offset, sizeof(io_buffer.ucElement));
                    io_buffer.usLen = uart_read_bytes(UART_NUM_0, io_buffer.ucElement, io_buffer.usLen, 0);
                    if (io_buffer.usLen < 0) {
                        assert(false);
                        break;
                    }
                    xQueueSend(*xuart_rx_queue, &io_buffer, portMAX_DELAY);
                    offset += io_buffer.usLen;
                }

                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        
        wait_ms = (++failed_waits > 2000) ? 20 : 1;
    }
}

/*
static void uart_tx_task(void *arg)
{
    xdev_buffer tx_buffer;

    while (1)
    {
    	if (xQueueReceive(*xuart_tx_queue, &tx_buffer, portMAX_DELAY)) {
        	if (uart_write_bytes(UART_NUM_0, tx_buffer.ucElement, tx_buffer.usLen) != tx_buffer.usLen) {
                assert(false);
            }
        }

//    	rx_buffer.usLen = uart_read_bytes(UART_NUM_0, rx_buffer.ucElement, RX_BUF_SIZE, 1 / portTICK_PERIOD_MS);
//    	rx_buffer.dev_channel = DEV_UART;
//    	if(rx_buffer.usLen > 0)
//    	{
//    		xQueueSend(*xuart_rx_queue, ( void * ) &rx_buffer, portMAX_DELAY );
//    	}
    }
}
*/

void wc_uart_init(QueueHandle_t *xTXp_Queue, QueueHandle_t *xRXp_Queue, uint8_t connected_led)
{
    const uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    xuart_tx_queue = xTXp_Queue;
	xuart_rx_queue = xRXp_Queue;
    // We won't use a buffer for sending data.
    //uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, ESP_INTR_FLAG_LEVEL1);
	uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 4, 0, 10, &uart0_queue, ESP_INTR_FLAG_LEVEL1);
    uart_param_config(UART_NUM_0, &uart_config);
//																					output,			input
//    esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num);
//    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 2, 10);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Note: looks like one task is faster then two separate tasks
    //
    //xTaskCreate(uart_tx_task, "uart_tx_task", 1024*2, (void*)AF_INET, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 1024*4, (void*)AF_INET, 5, NULL);
}


