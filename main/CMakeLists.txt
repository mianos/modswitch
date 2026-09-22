set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "main.cpp" "MosWebServer.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                         espressif__esp-modbus
                         nvsstoragemanager
                         wifimanager
                         settingsbase
                         webserver
                         jsonwrapper
                         esp_driver_gpio
                         esp_driver_uart
                         esp_wifi
                         esp_netif
                         esp_event
                         nvs_flash
                         app_update
                         esp_partition
                         esp_timer)
