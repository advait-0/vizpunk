#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_random.h"
#include "dirent.h"

#define TAG "ESP32_GRAPH"

// TO DO: Replace with real readings
int get_sensor_value() {
    return esp_random() % 4096; // Random value between 0-4096
}

// Serve the main HTML page
esp_err_t page_handler(httpd_req_t *req) {
    const char *html = "<!DOCTYPE html>"
        "<html lang='en'><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>ESP32 Voltage Graph</title>"
        "<script src='/uPlot.iife.min.js'></script>"
        "<style>"
        "body { font-family: Arial, sans-serif; text-align: center; padding: 20px; }"
        ".container { max-width: 600px; margin: auto; padding: 20px; border: 2px solid #ddd; border-radius: 10px; box-shadow: 2px 2px 12px #aaa; }"
        "#chart { width: 100%; height: 300px; margin-top: 20px; }"
        "</style></head>"
        "<body>"
        "<h1>ESP32 Voltage Monitoring</h1>"
        "<div class='container'>"
        "<button onclick='location.reload()'>Refresh Graph</button>"
        "</div>"
        "<h2>Live Voltage Graph</h2><div id='chart'></div>"
        "<script>"
        "let data = [[], []];"
        "let maxDataPoints = 50;"
        "const startTime = Date.now();"

        "const opts = {"
        "    title: 'Voltage vs. Time',"
        "    width: 600,"
        "    height: 300,"
        "    scales: { x: { time: false }, y: { min: 0, max: 5000 } },"
        "    axes: ["
        "        { label: 'Time (s)' },"
        "        { label: 'Voltage (mV)' }"
        "    ],"
        "    series: [ {}, { label: 'Voltage', stroke: 'red', width: 2 } ]"
        "};"

        "document.addEventListener('DOMContentLoaded', () => {"
        "    if (typeof uPlot !== 'undefined') {"
        "        window.chart = new uPlot(opts, data, document.getElementById('chart'));"
        "        setInterval(fetchData, 1000);"
        "    } else { console.error('uPlot failed to load.'); }"
        "});"

        "function fetchData() {"
        "    fetch('/graph')"
        "    .then(res => res.text())"
        "    .then(value => {"
        "        let voltage = parseFloat(value);"
        "        let elapsedTime = ((Date.now() - startTime) / 1000).toFixed(2);"
        "        if (!isNaN(voltage)) {"
        "            data[0].push(parseFloat(elapsedTime));"
        "            data[1].push(voltage);"
        "            if (data[0].length > maxDataPoints) {"
        "                data[0].shift();"
        "                data[1].shift();"
        "            }"
        "            window.chart.setData(data);"
        "        }"
        "    })"
        "    .catch(err => console.error('Fetch error:', err));"
        "}"
        "</script></body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// Serve JavaScript file from SPIFFS
esp_err_t js_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/uPlot.iife.min.js", "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open uplot.js");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "application/javascript");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// Serve real-time sensor data
esp_err_t graph_handler(httpd_req_t *req) {
    char data[16];
    snprintf(data, sizeof(data), "%d", get_sensor_value());
    httpd_resp_send(req, data, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Start HTTP Server
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t page_uri = { .uri = "/", .method = HTTP_GET, .handler = page_handler, .user_ctx = NULL };
        httpd_uri_t js_uri = { .uri = "/uPlot.iife.min.js", .method = HTTP_GET, .handler = js_handler, .user_ctx = NULL };
        httpd_uri_t graph_uri = { .uri = "/graph", .method = HTTP_GET, .handler = graph_handler, .user_ctx = NULL };

        httpd_register_uri_handler(server, &page_uri);
        httpd_register_uri_handler(server, &js_uri);
        httpd_register_uri_handler(server, &graph_uri);
    }
    return server;
}

// Initialize SPIFFS
void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS Mount Failed");
    } else {
        ESP_LOGI(TAG, "SPIFFS Mounted Successfully");
    }
}

void list_spiffs_files() {
    DIR* dir = opendir("/spiffs");
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            ESP_LOGI("SPIFFS", "Found file: %s", entry->d_name);
        }
        closedir(dir);
    } else {
        ESP_LOGE("SPIFFS", "Failed to open directory");
    }
}

// Initialize Wi-Fi
void init_wifi() {
    ESP_LOGI(TAG, "Setting up Wi-Fi...");
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = {
        .ap = { .ssid = "ESP32_Graph", .ssid_len = strlen("ESP32_Graph"), .password = "", .max_connection = 4, .authmode = WIFI_AUTH_OPEN }
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "Wi-Fi Setup Complete!");
}

// Main App Entry
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_spiffs();
    list_spiffs_files();
    init_wifi();
    start_webserver();
}
