#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#define WIFI_SSID "ESP32_Network"
#define WIFI_PASS "12345678"
#define MAX_STA_CONN 4

static const char *TAG = "wifi_ap";

// HTML Webpage with Dummy Buttons and Graph
const char html_page[] = "<!DOCTYPE html>"
"<html><head><title>ESP32 Interactive UI</title>"
"<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
"<style>"
"body {font-family: Arial, sans-serif; text-align: center; padding: 20px;}"
".button {margin: 10px; padding: 15px; font-size: 18px; background: #008CBA; color: white; border: none; cursor: pointer;}"
".button:hover {background: #005f73;}"
".container {max-width: 600px; margin: auto; padding: 20px; border: 2px solid #ddd; border-radius: 10px; box-shadow: 2px 2px 12px #aaa;}"
"</style></head>"
"<body><h1>ESP32 Web Server</h1>"
"<p>Lorem ipsum dolor sit amet, consectetur adipiscing elit. Suspendisse varius enim in eros elementum tristique.</p>"
"<div class='container'>"
"<button class='button' onclick=\"fetch('/action1').then(res => res.text()).then(alert)\">Click Me 1</button>"
"<button class='button' onclick=\"fetch('/action2').then(res => res.text()).then(alert)\">Click Me 2</button>"
"<button class='button' onclick=\"fetch('/action3').then(res => res.text()).then(alert)\">Click Me 3</button>"
"</div>"
"<h2>Live Data Graph</h2><canvas id='dataChart' width='400' height='200'></canvas>"
"<script>"
"var ctx = document.getElementById('dataChart').getContext('2d');"
"var chart = new Chart(ctx, {type: 'line', data: {labels: [], datasets: [{label: 'Random Data', data: [], borderColor: 'red', fill: false}]}}, options: {scales: {y: {beginAtZero: true}}}});"
"function updateGraph() {fetch('/graph').then(res => res.text()).then(value => {"
"chart.data.labels.push(new Date().toLocaleTimeString());"
"chart.data.datasets[0].data.push(parseInt(value));"
"if(chart.data.labels.length > 10) {chart.data.labels.shift(); chart.data.datasets[0].data.shift();}"
"chart.update();});}"
"setInterval(updateGraph, 1000);"
"</script></body></html>";

// Root page handler
esp_err_t webpage_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// Dummy button handlers
esp_err_t action1_handler(httpd_req_t *req) {
    httpd_resp_send(req, "You clicked Button 1!", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
esp_err_t action2_handler(httpd_req_t *req) {
    httpd_resp_send(req, "You clicked Button 2!", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
esp_err_t action3_handler(httpd_req_t *req) {
    httpd_resp_send(req, "You clicked Button 3!", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Graph data handler
esp_err_t graph_handler(httpd_req_t *req) {
    char response[16];
    snprintf(response, sizeof(response), "%d", rand() % 100);  // Random values
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Start Web Server
httpd_handle_t start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = webpage_handler };
        httpd_uri_t uri_action1 = { .uri = "/action1", .method = HTTP_GET, .handler = action1_handler };
        httpd_uri_t uri_action2 = { .uri = "/action2", .method = HTTP_GET, .handler = action2_handler };
        httpd_uri_t uri_action3 = { .uri = "/action3", .method = HTTP_GET, .handler = action3_handler };
        httpd_uri_t uri_graph = { .uri = "/graph", .method = HTTP_GET, .handler = graph_handler };

        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_action1);
        httpd_register_uri_handler(server, &uri_action2);
        httpd_register_uri_handler(server, &uri_action3);
        httpd_register_uri_handler(server, &uri_graph);
    }
    return server;
}

// Wi-Fi Event Handler
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Client connected");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Client disconnected");
    }
}

// Initialize Wi-Fi Access Point
void wifi_init_softap() {
    ESP_LOGI(TAG, "Setting up WiFi Access Point...");
    
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. Connect to '%s' and open http://192.168.4.1", WIFI_SSID);
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_softap();
    start_webserver();
}
