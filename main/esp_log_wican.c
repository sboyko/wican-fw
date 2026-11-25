#include "esp_log_wican.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mbedtls/error.h>


/**
 * Uses appropriate logging for:
 *  esp_err_t
 *  mbedTLS (int codes)
 */
void wican_log_error(char* const buffer, int err_code, const wican_log_error_t err_type)
{
	static const int ERR_DESCRIPTION_MAX_SIZE = 30;

	if (err_type == ERR_ESP_TYPE) {
		const char* err_text = esp_err_to_name(err_code);
		if (err_text != NULL) {
			strlcpy(buffer + strlen(buffer), err_text, ERR_DESCRIPTION_MAX_SIZE);
		}
		sprintf(buffer + strlen(buffer), "_esp_%x", err_code);
	}
	else if (err_type == ERR_MBEDTLS_TYPE) {
		mbedtls_strerror(err_code, buffer, ERR_DESCRIPTION_MAX_SIZE);
		sprintf(buffer + strlen(buffer), "_mbed_%04x", err_code); // note that actually 'err_code' is negative
	}
}

/**
 * Output:
 *  list of all tasksk with uxTaskGetStackHighWaterMark
 *  overall memory usage
 *  heap integrity check
 */
void wican_debug_mem_usage(
	bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q),
	QueueHandle_t *q
) {

	char buff[140];

#if ( CONFIG_FREERTOS_USE_TRACE_FACILITY == 1 )
	// Get the number of tasks
	const uint32_t task_count = uxTaskGetNumberOfTasks();
	// Dynamically allocate an array to hold the states
	TaskStatus_t *task_status_array = malloc(task_count * sizeof(TaskStatus_t));
	if (task_status_array != NULL) {
		uxTaskGetSystemState(task_status_array, task_count, NULL);

		for (int i = 0; i < task_count; ++i) {
			snprintf(buff, sizeof(buff), "Dbg:task=%s|state=%d|prio=%2d|mark=%d\r",
				task_status_array[i].pcTaskName,
				task_status_array[i].eCurrentState,
				task_status_array[i].uxCurrentPriority,
				(int)task_status_array[i].usStackHighWaterMark
			);
			send_to_host(buff, 0, q);
		}
		
		free(task_status_array);
	}
#endif

	multi_heap_info_t info = {0};
	heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task

	snprintf(buff, sizeof(buff), "Dbg:m_free=%d|m_alloc=%d|largest_free=%2d|min_free=%d|alloc_bl=%d|free_bl=%d\r",
		info.total_free_bytes, info.total_allocated_bytes, info.largest_free_block, info.minimum_free_bytes,
		info.allocated_blocks, info.free_blocks
	);
	send_to_host(buff, 0, q);


	snprintf(buff, sizeof(buff), "Dbg:heap_integrity=%s\r",
		(heap_caps_check_integrity_all(false) ? "Good" : "Broken")
	);
	send_to_host(buff, 0, q);

}
