#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define ADC_UNIT             ADC_UNIT_1
#define ADC_CHANNEL          ADC_CHANNEL_6      // Example: GPIO34
#define SAMPLE_FREQUENCY_HZ  500000              // 10 kHz sampling frequency
#define BUFFER_SIZE          SOC_ADC_DIGI_DATA_BYTES_PER_CONV              // DMA buffer size in bytes
#define TIMEOUT_MS           1000               // Timeout for ADC read

static const char *TAG = "ADC_PRINT";

adc_continuous_handle_t adc_handle;
uint8_t adc_dma_buffer[BUFFER_SIZE] = {0};

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

void adc_print_task(void *pvParameters)
{
    // ret_size is declared as uint32_t per API requirements.
    uint32_t ret_size = 0;
    while (true) {
        // Read ADC data from the DMA buffer.
        esp_err_t ret = adc_continuous_read(adc_handle, adc_dma_buffer, BUFFER_SIZE, &ret_size, TIMEOUT_MS);
        if (ret == ESP_OK && ret_size > 0) {
            // Assume each ADC sample occupies 2 bytes.
            for (uint32_t i = 0; i < ret_size; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_dma_buffer[i];
                uint16_t data = p->type1.data;

                ESP_LOGI(TAG, "ADC Value: %d", data);
            }
        } else {
            ESP_LOGW(TAG, "No ADC data available or read timed out");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    // Initialize continuous ADC sampling.
    init_adc_continuous();

    // Create the ADC printing task pinned to core 1.
    xTaskCreatePinnedToCore(adc_print_task, "ADC_Print_Task", 4096, NULL, 3, NULL, 1);
}
