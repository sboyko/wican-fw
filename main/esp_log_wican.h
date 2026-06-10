#ifndef esp_log_wican_h
#define esp_log_wican_h

#include "esp_log.h"

// Note: don't forget to turn off 'configUSE_TRACE_FACILITY' option

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

#endif // esp_log_wican_h