# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.
set(srcs  "main.c" "comm_server.c" "config_server.c" "realdash.c" "slcan.c" "can.c" "ble.c" "wifi_network.c" "gvret.c" "wc_uart.c" "elm327.c" "mqtt.c" "sleep_mode.c" 
          "esp_websocket_client.c" "ecc256.c" "aes_gcm.c" "gps_common.c" "gps_nmea.c" "ftp.c" "wican_settings.c" "wican_commands.c" "esp_log_wican.c"
)
set(requires  esp_timer esp_wifi nvs_flash fatfs vfs driver esp-tls esp_adc esp_eth log app_update esp_http_server bt spiffs freertos mqtt json spi_flash)
idf_component_register(
    SRCS "${srcs}"         # list the source files of this component
    INCLUDE_DIRS           # optional, add here public include directories
    PRIV_INCLUDE_DIRS      # optional, add here private include directories
    REQUIRES "${requires}" # optional, list the public requirements (component names)
    PRIV_REQUIRES          # optional, list the private requirements
)
