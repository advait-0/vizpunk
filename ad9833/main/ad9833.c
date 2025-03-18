#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define FSYNC_PIN 5    
#define MOSI_PIN 23    
#define SCLK_PIN 18    
#define SPI_HOST VSPI_HOST      

// Byte swap function
uint16_t swap_bytes(uint16_t value) {
    return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

// Calculate frequency registers with integrated byte swapping
void calculate_frequency_registers(float freq_hz, uint16_t *freq_lsb, uint16_t *freq_msb) {
    uint32_t freq_word = (uint32_t)((freq_hz * pow(2, 28)) / 25000000.0);
    
    // Extract and format LSB with '01' prefix
    uint16_t raw_lsb = (freq_word & 0x3FFF) | 0x4000;
    *freq_lsb = swap_bytes(raw_lsb);
    
    // Extract and format MSB with '10' prefix
    uint16_t raw_msb = ((freq_word >> 14) & 0x3FFF) | 0x4000;
    *freq_msb = swap_bytes(raw_msb);
}

void app_main() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FSYNC_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(FSYNC_PIN, 1); // initially high
   
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = -1,   
        .sclk_io_num = SCLK_PIN,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1, 
        .max_transfer_sz = 16,
    };
    spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED);
   
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz
        .mode = 2,                
        .spics_io_num = -1,     
        .queue_size = 1,
    };
    spi_device_handle_t spi;
    spi_bus_add_device(SPI_HOST, &devcfg, &spi);
   
    // Send reset command
    gpio_set_level(FSYNC_PIN, 0);
    uint16_t reset_cmd = swap_bytes(0x2100);  // 0x0221 after swap
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = &reset_cmd,
    };
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);

    // Calculate frequency registers for 1 kHz
    uint16_t freq_lsb, freq_msb;
    calculate_frequency_registers(40000.0, &freq_lsb, &freq_msb);
    
    // Send frequency LSB
    gpio_set_level(FSYNC_PIN, 0);
    t.tx_buffer = &freq_lsb;  // Will be 0x0A57 after swap
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);
    
    // Send frequency MSB
    gpio_set_level(FSYNC_PIN, 0);
    t.tx_buffer = &freq_msb;  // Will be 0x8F42 after swap
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);
    
    // Select sine wave output
    gpio_set_level(FSYNC_PIN, 0);
    uint16_t sine_cmd = swap_bytes(0xC000);  // 0x00C0 after swap
    t.tx_buffer = &sine_cmd;
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);
    
    // Exit reset
    gpio_set_level(FSYNC_PIN, 0);
    uint16_t exit_reset = swap_bytes(0x2002);  // 0x0220 after swap
    t.tx_buffer = &exit_reset;
    spi_device_transmit(spi, &t);
    gpio_set_level(FSYNC_PIN, 1);
    
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}