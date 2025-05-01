#include <stdio.h>
#include <string.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <sys/param.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_random.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "dirent.h"
#include "math.h"
#include "handlers.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0

EventGroupHandle_t wifi_event_group;
// wifi_event_group = xEventGroupCreate();

#ifdef __cplusplus
extern "C" {
#endif

void setup(void);  // C-style declaration of the C++ function

#ifdef __cplusplus
}
#endif

#define DNS_PORT 53
#define FSYNC_PIN 5
#define MOSI_PIN 23
#define SCLK_PIN 18
#define SINE_WAVE     0x0000
#define TRIANGLE_WAVE 0x0002
#define SQUARE_WAVE   0x0020  // MSB output
#define SQUARE_WAVE_DIV2 0x0008  // MSB/2 output
#define EXIT 0x2002

#define ADC_UNIT             ADC_UNIT_1
#define ADC_CHANNEL          ADC_CHANNEL_6      // Example: GPIO34
#define SAMPLE_FREQUENCY_HZ  50000              // 10 kHz sampling frequency
#define BUFFER_SIZE          SOC_ADC_DIGI_DATA_BYTES_PER_CONV              // DMA buffer size in bytes
#define TIMEOUT_MS           1000               // Timeout for ADC read

volatile uint16_t adc_data = 0;  // Shared variable
SemaphoreHandle_t mutex;
void command_loop_task(void *param);

adc_continuous_handle_t adc_handle;
uint8_t adc_dma_buffer[BUFFER_SIZE] = {0};

// // Wi-Fi event handler for connection events
// static void wifi_event_handler(void* arg, esp_event_base_t event_base,
//     int32_t event_id, void* event_data) {
// if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
// // Set the connected bit when the AP is started
// xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
// ESP_LOGI(LOG_TAG, "Wi-Fi AP Started");
// }
// // Handle other events if needed (e.g., Wi-Fi stop, disconnect, etc.)
// }

void init_adc_continuous(void)
{
    // Create the ADC continuous handle.
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUFFER_SIZE,
        .conv_frame_size = SOC_ADC_DIGI_DATA_BYTES_PER_CONV,   // Process in chunks
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // Configure continuous ADC conversion.
    // In ESP-IDF 5.4, the configuration structure contains an array named 'adc_pattern'
    adc_continuous_config_t adc_cont_config = {
        .sample_freq_hz = SAMPLE_FREQUENCY_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // Using ADC1 only
        .pattern_num = 1,                    // Only one channel in our pattern
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
    };

    adc_digi_pattern_config_t adc_pattern = {
            .atten = ADC_ATTEN_DB_12,    // Use 12 dB attenuation (recommended replacement)
            .channel = ADC_CHANNEL,
            .bit_width = ADC_BITWIDTH_12,
            .unit = ADC_UNIT_1
    };

    adc_cont_config.adc_pattern = &adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_cont_config));

    // Start ADC continuous mode.
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void adc_read_task(void *pvParameters)
{
    // ret_size is declared as uint32_t per API requirements.
    uint32_t ret_size = 0;
    while (true) {
        // Read ADC data from the DMA buffer.
        esp_err_t ret = adc_continuous_read(adc_handle, adc_dma_buffer, BUFFER_SIZE, &ret_size, TIMEOUT_MS);
        if (ret == ESP_OK && ret_size > 0) {
            // Assume each ADC sample occupies 2 bytes.
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_dma_buffer;
            adc_data = p->type1.data;
            // ESP_LOGI(TAG, "ADC Value: %d", adc_data);
            // xSemaphoreGive(mutex);
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)); //TEST THIS
    }
}

gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << FSYNC_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
};

spi_device_handle_t spi;

uint16_t swap_bytes(uint16_t value) {
    return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

// Function to calculate AD9833 frequency registers
void calculate_frequency_registers(float freq_hz, uint16_t *freq_lsb, uint16_t *freq_msb) {
    uint32_t freq_word = (uint32_t)((freq_hz * pow(2, 28)) / 25000000.0);
    *freq_lsb = (freq_word & 0x3FFF) | 0x4000;
    *freq_msb = ((freq_word >> 14) & 0x3FFF) | 0x4000;

    ESP_LOGI(TAG, "Calculated Freq: %.2f Hz, LSB: 0x%04X, MSB: 0x%04X", freq_hz, *freq_lsb, *freq_msb);
}

// Send data to AD9833
void send_ad9833(uint16_t freq_lsb, uint16_t freq_msb, const char* wave) {
    freq_lsb = swap_bytes(freq_lsb);
    freq_msb = swap_bytes(freq_msb);
    gpio_set_level(FSYNC_PIN, 0);
    uint16_t reset_cmd = swap_bytes(0x2100);  // 0x0021 after swap
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = &reset_cmd,
    };
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);

    // Send frequency LSB
    gpio_set_level(FSYNC_PIN, 0);
    t.tx_buffer = &freq_lsb;
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);

    // Send frequency MSB
    gpio_set_level(FSYNC_PIN, 0);
    t.tx_buffer = &freq_msb;
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);

    // Select wave output
    ESP_LOGI(TAG, "Wave type: %s", wave);
    gpio_set_level(FSYNC_PIN, 0);
    uint16_t wave_cmd;
    if (strcmp(wave, "sine") == 0) {
        wave_cmd = SINE_WAVE | EXIT;
    } else if (strcmp(wave, "triangle") == 0) {
        wave_cmd = TRIANGLE_WAVE | EXIT;
    } else if (strcmp(wave, "square") == 0) {
        wave_cmd = SQUARE_WAVE | EXIT;
    } else {
        ESP_LOGW(TAG, "Invalid wave type: %s, defaulting to SINE", wave);
        wave_cmd = SINE_WAVE | EXIT;
    }
    wave_cmd = swap_bytes(wave_cmd);
    t.tx_buffer = &wave_cmd;
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);

    ESP_LOGI(TAG, "Sending to AD9833 -> LSB: 0x%04X, MSB: 0x%04X, EXIT: 0x%04X", freq_lsb, freq_msb, wave_cmd);
}

// Function to get random sensor value
int get_sensor_value() {
    return esp_random() % 4096;
}

void init_spi() {
    // Declare the spi handle here

   spi_bus_config_t buscfg = {
       .mosi_io_num = MOSI_PIN,
       .miso_io_num = -1,
       .sclk_io_num = SCLK_PIN,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 16,
   };
   esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus Init Failed: %s", esp_err_to_name(ret));
        return;
    }

   spi_device_interface_config_t devcfg = {
       .clock_speed_hz = 1000000, // 1 MHz
       .mode = 2,
       .spics_io_num = -1,
       .queue_size = 1,
   };

   ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Device Add Failed: %s", esp_err_to_name(ret));
        return;
    }
}

void dns_server_task(void *pvParameters) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in server_addr, client_addr;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(DNS_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

    char buffer[512];
    socklen_t client_len = sizeof(client_addr);

    while (1) {
        int recv_len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &client_len);
        if (recv_len > 0) {
            // Respond with our ESP32's IP address (192.168.4.1)
            buffer[2] |= 0x80; // Set response flag
            buffer[3] |= 0x00; // Set recursion available
            buffer[7] = 1;     // Set answer count to 1

            // Set answer section
            size_t offset = recv_len;
            buffer[offset++] = 0xC0; buffer[offset++] = 0x0C; // Pointer to question
            buffer[offset++] = 0x00; buffer[offset++] = 0x01; // Type A
            buffer[offset++] = 0x00; buffer[offset++] = 0x01; // Class IN
            buffer[offset++] = 0x00; buffer[offset++] = 0x00; buffer[offset++] = 0x00; buffer[offset++] = 0x3C; // TTL
            buffer[offset++] = 0x00; buffer[offset++] = 0x04; // Data length
            buffer[offset++] = 192; buffer[offset++] = 168; buffer[offset++] = 4; buffer[offset++] = 1; // 192.168.4.1

            sendto(sock, buffer, offset, 0, (struct sockaddr*)&client_addr, client_len);
        }
    }
}

void start_dns_server() {
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
}

esp_err_t redirect_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
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

// Initialize SPIFFS
void init_spiffs() {
    ESP_LOGI(LOG_TAG, "Initializing SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_vfs_spiffs_register(&conf);
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 14;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uris[] = {
            {"/", HTTP_GET, page_handler, NULL},
            {"/set_function_generator", HTTP_GET, set_function_generator_handler, NULL},
            {"/logic_analyzer", HTTP_GET, logic_analyzer_handler, NULL},
            {"/start_logic_capture", HTTP_GET, start_logic_capture_handler, NULL},
            {"/adc", HTTP_GET, adc_handler, NULL},
            {"/function_generator", HTTP_GET, function_generator_handler, NULL},
            {"/psu", HTTP_GET, psu_handler, NULL},
            {"/graph", HTTP_GET, graph_handler, NULL},
            {"/uPlot.iife.min.js", HTTP_GET, js_handler, NULL},
            {"/hotspot-detect.html", HTTP_GET, page_handler, NULL}, // Apple Handle captive portal check
            {"/generate_204", HTTP_GET, page_handler, NULL},
            {"/generate204", HTTP_GET, page_handler, NULL}, // Android captive portal check
            {"/favicon.ico", HTTP_GET, dummy_handler, NULL},
            {"/chat", HTTP_GET, dummy_handler, NULL},
            {"*", HTTP_GET, redirect_handler, NULL}
        };

        for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(server, &uris[i]);
        }
    }
    return server;
}

// Initialize Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data) {
if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
ESP_LOGI(LOG_TAG, "Wi-Fi AP Started");
xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}
}

void init_wifi() {
ESP_LOGI(LOG_TAG, "Setting up Wi-Fi...");

wifi_event_group = xEventGroupCreate();
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());
esp_netif_create_default_wifi_ap();

wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&cfg));

// Set country (fix channel issues)
wifi_country_t country = {
.cc = "IN",  // Change to your country if needed
.schan = 1,
.nchan = 13,
.policy = WIFI_COUNTRY_POLICY_AUTO
};
ESP_ERROR_CHECK(esp_wifi_set_country(&country));

wifi_config_t wifi_config = {
.ap = {
.ssid = "Vizpunk",
.ssid_len = strlen("Vizpunk"),
.password = "",  // WPA2 requires at least 8 chars
.max_connection = 4,
.authmode = WIFI_AUTH_OPEN
}
};

if (strlen((char *)wifi_config.ap.password) == 0) {
wifi_config.ap.authmode = WIFI_AUTH_OPEN;
}

ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                             ESP_EVENT_ANY_ID,
                             &wifi_event_handler,
                             NULL,
                             NULL));
ESP_ERROR_CHECK(esp_wifi_start());

ESP_LOGI(LOG_TAG, "Waiting for Wi-Fi to start AP...");
xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
ESP_LOGI(LOG_TAG, "Wi-Fi AP is ready.");
}

// Main function
void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO); // Optional: sets all logs to INFO
    esp_log_level_set("intr_alloc", ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(nvs_flash_init());
    gpio_config(&io_conf);
    init_spi();
    init_spiffs();
    list_spiffs_files();
    init_wifi();
    vTaskDelay(pdMS_TO_TICKS(10000));
    // xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    start_dns_server();
    start_webserver();
    // init_adc_continuous();
    // setup();
    // xTaskCreatePinnedToCore(command_loop_task, "CommandLoopTask", 4096, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(adc_read_task, "ADC_Read_Task", 4096, NULL, 7, NULL, 1);
}
