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

#define TAG "ESP32_WEB"

// Function to get random sensor value
int get_sensor_value() {
    return esp_random() % 4096;
}

// Serve landing page
esp_err_t page_handler(httpd_req_t *req) {
    const char *html = "<!DOCTYPE html>"
        "<html lang='en'>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>ESP32 Control Panel</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; background-color: #f4f4f4; margin: 0; }"
        ".container { text-align: center; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); width: 90%; max-width: 400px; }"
        "h1 { margin-bottom: 20px; }"
        ".btn { display: block; width: 100%; padding: 12px; margin: 10px 0; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }"
        ".btn.adc { background: #007BFF; color: white; }"
        ".btn.function { background: #28A745; color: white; }"
        ".btn.psu { background: #FFC107; color: white; }"
        ".btn:hover { opacity: 0.9; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class='container'>"
        "<h1>ESP32 Control Panel</h1>"
        "<button class='btn adc' onclick=\"location.href='/adc'\">ADC</button>"
        "<button class='btn function' onclick=\"location.href='/function_generator'\">Function Generator</button>"
        "<button class='btn psu' onclick=\"location.href='/psu'\">PSU</button>"
        "</div>"
        "</body>"
        "</html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// Serve ADC page
esp_err_t adc_handler(httpd_req_t *req) {
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

// Serve Function Generator page
esp_err_t function_generator_handler(httpd_req_t *req) {
    const char *html = "<html><body><h1>Function Generator Settings</h1>"
        "<p>Configure waveform parameters here.</p>"
        "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve PSU page
esp_err_t psu_handler(httpd_req_t *req) {
    const char *html = "<html><body><h1>PSU Settings</h1>"
        "<p>Set output voltage and current.</p>"
        "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve real-time sensor data
esp_err_t graph_handler(httpd_req_t *req) {
    char data[16];
    snprintf(data, sizeof(data), "%d", get_sensor_value());
    httpd_resp_send(req, data, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve JavaScript file from SPIFFS
esp_err_t js_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/uPlot.iife.min.js", "r");  
    if (!f) {
        ESP_LOGE(TAG, "Failed to open uPlot.iife.min.js in SPIFFS");
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

// Start HTTP Server
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uris[] = {
            {"/", HTTP_GET, page_handler, NULL},
            {"/adc", HTTP_GET, adc_handler, NULL},
            {"/function_generator", HTTP_GET, function_generator_handler, NULL},
            {"/psu", HTTP_GET, psu_handler, NULL},
            {"/graph", HTTP_GET, graph_handler, NULL},
            {"/uPlot.iife.min.js", HTTP_GET, js_handler, NULL}
        };
        for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(server, &uris[i]);
        }
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
    esp_vfs_spiffs_register(&conf);
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
        .ap = { .ssid = "ESP32_Device", .ssid_len = strlen("ESP32_Device"), .password = "", .max_connection = 4, .authmode = WIFI_AUTH_OPEN }
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
}

// Main function
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    init_spiffs();
    list_spiffs_files();
    init_wifi();
    start_webserver();
}
