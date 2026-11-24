#pragma once

/*
Disable Application Logs (Compile-Time)
 menuconfig -> Component config -> Log -> Log Level -> Default log verbosity = No output (CONFIG_LOG_DEFAULT_LEVEL_NONE=y)
 menuconfig -> Bootloader config -> Log -> Bootloader log verbosity = No output (CONFIG_BOOTLOADER_LOG_LEVEL_NONE=y)

Disable the Hardware Console Output
 menuconfig -> Component config -> ESP System Settings -> Channel for console output = None (CONFIG_ESP_CONSOLE_NONE=y)
 menuconfig -> Boot ROM behavior = Permanently silent (The rom log behavior can only be changed for once, specific eFuse bit(s) will be burned at app boot stage.)
*/

// Note: don't forget to turn off 'configUSE_TRACE_FACILITY' (Enables additional structure members and functions to assist with execution visualization and tracing)
//  menuconfig -> Component config -> FreeRTOS -> Kernel -> configUSE_TRACE_FACILITY (checkbox)

/**
 * Для Release-версии нужно:
 * 
 * - включить (CONFIG_COMPILER_OPTIMIZATION_CHECKS_SILENT=y)
 *   удаляет текстовые строки сообщений об ошибках из внутренних макросов проверки ошибок ESP-IDF (таких как ESP_RETURN_ON_ERROR, 
 *   ESP_GOTO_ON_ERROR, ESP_RETURN_ON_FALSE и аналогичных), чтобы уменьшить размер итоговой прошивки во Flash-памяти
 * 
 * - отключить (CONFIG_MBEDTLS_ERROR_STRINGS=n)
 *   отключает текстовые строки ошибок mbedTLS для экономии места
 * 
 * - отключить (CONFIG_ESP_ERR_TO_NAME_LOOKUP=n)
 *   Таблица текстовых имен ошибок занимает заметный объем Flash-памяти (несколько килобайт)
 * 
 */

// typedef enum {
//     ESP_LOG_NONE    = 0,    /*!< No log output */
//     ESP_LOG_ERROR   = 1,    /*!< Critical errors, software module can not recover on its own */
//     ESP_LOG_WARN    = 2,    /*!< Error conditions from which recovery measures have been taken */
//     ESP_LOG_INFO    = 3,    /*!< Information messages which describe normal flow of events */
//     ESP_LOG_DEBUG   = 4,    /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
//     ESP_LOG_VERBOSE = 5,    /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
//     ESP_LOG_MAX     = 6,    /*!< Number of levels supported */
// } esp_log_level_t;
#define ESP_LOG_MAIN 0

#include <esp_check.h> // implies <esp_log.h>

#if ESP_LOG_MAIN == 0
    #undef ESP_LOGV
    #define ESP_LOGV(...) do { } while(0)

    #undef ESP_LOGD
    #define ESP_LOGD(...) do { } while(0)

    #undef ESP_LOGI
    #define ESP_LOGI(...) do { } while(0)

    #undef ESP_LOGW
    #define ESP_LOGW(...) do { } while(0)

    #undef ESP_LOGE
    #define ESP_LOGE(...) do { } while(0)

    #undef ESP_LOG_BUFFER_HEXDUMP
    #define ESP_LOG_BUFFER_HEXDUMP(...) do { } while(0)

    #undef ESP_LOG_BUFFER_HEX
    #define ESP_LOG_BUFFER_HEX(...) do { } while(0)
#else
    #ifdef NDEBUG
        #undef ESP_LOG_BUFFER_HEXDUMP
        #define ESP_LOG_BUFFER_HEXDUMP(...) do { } while(0)

        #undef ESP_LOG_BUFFER_HEX
        #define ESP_LOG_BUFFER_HEX(...) do { } while(0)
    #endif // NDEBUG
#endif // ESP_LOG_MAIN

typedef enum {
    ERR_ESP_TYPE     = 1, // esp_err_t
    ERR_MBEDTLS_TYPE = 2, // коды ошибок из mbedTLS (int)
} wican_log_error_t;

void wican_log_error(char* const buffer, int err_code, const wican_log_error_t err_type);
