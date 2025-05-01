idf_component_register(SRCS "main.c" "handlers.c" "logic_analyzer.cpp" "logic_analyzer_i2s.cpp"
                    INCLUDE_DIRS "." "../include"
                    EMBED_FILES "../spiffs/uPlot.iife.min.js"
                    REQUIRES esp_http_server esp_wifi esp_netif nvs_flash spiffs esp_adc driver freertos esp_system esp_driver_gpio)
