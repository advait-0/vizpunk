#include <esp_http_server.h>
#include "esp_log.h"

static const char *TAG = "FUNC_GEN";
#define LOG_TAG "ESP32_WEB"

esp_err_t page_handler(httpd_req_t *req);

esp_err_t adc_handler(httpd_req_t *req);

esp_err_t dummy_handler(httpd_req_t *req);

esp_err_t function_generator_handler(httpd_req_t *req);

esp_err_t set_function_generator_handler(httpd_req_t *req);

esp_err_t psu_handler(httpd_req_t *req);

esp_err_t graph_handler(httpd_req_t *req);

esp_err_t js_handler(httpd_req_t *req);

void send_ad9833(uint16_t freq_lsb, uint16_t freq_msb, const char* wave);

void calculate_frequency_registers(float freq_hz, uint16_t *freq_lsb, uint16_t *freq_msb);

int get_sensor_value();
