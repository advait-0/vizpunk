idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "../spiffs/uPlot.iife.min.js"
                    REQUIRES esp_http_server esp_wifi esp_netif nvs_flash spiffs)
