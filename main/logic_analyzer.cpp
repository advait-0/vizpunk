
  /*************************************************************************
 *
 *  ESP32 Logic Analyzer
 *  Copyright (C) 2020 Erdem U. Altinyurt
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *************************************************************************
 *
 *  This project steals some code from
 *  https://github.com/igrr/esp32-cam-demo for I2S DMA
 *  and
 *  https://github.com/gillham/logic_analyzer/issues for SUMP protocol as template.
 *
 */
#include "logic_analyzer.hpp"
extern i2s_parallel_config_t cfg;

i2s_parallel_buffer_desc_t bufdesc;

#define DEBUG_UART_NUM UART_NUM_0
#define DEBUG_BAUD     115200

#define OLS_UART_NUM   UART_NUM_1
#define OLS_TX_PIN     13
#define OLS_RX_PIN     12
#define OLS_BAUD       2000000  // or whatever baud you used

#define FILE_PATH "/spiffs/capture.bin"


uint32_t time_debug_indice_dma[10] = {0};
uint16_t time_debug_indice_dma_p = 0;

uint32_t time_debug_indice_rle[10] = {0};
uint16_t time_debug_indice_rle_p = 0;

uint16_t time_debug_indice_lenght = 10;

int stop_at_desc = -1;
unsigned int logicIndex = 0;
unsigned int triggerIndex = 0;
uint32_t readCount = CAPTURE_SIZE;
unsigned int delayCount = 0;
uint16_t trigger = 0;
uint16_t trigger_values = 0;
unsigned int useMicro = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;
bool rleEnabled = 0;
uint32_t clock_per_read = 0;

int8_t    rle_process = -1;
uint8_t   rle_buff[rle_size];
uint8_t*  rle_buff_p = nullptr;
uint8_t*  rle_buff_end = nullptr;
uint8_t   rle_sample_counter = 0;
uint32_t  rle_total_sample_counter = 0;
uint8_t   rle_value_holder = 0;

uint8_t channels_to_read = 3;
camera_state_t* s_state = nullptr;

// i2s_parallel_state_t* i2s_state[2] = {nullptr, nullptr};  // actual definition


void rle_init();

i2s_parallel_state_t* i2s_state[2] = {NULL, NULL};

uart_config_t uart_config = {
  .baud_rate = OLS_BAUD,
  .data_bits = UART_DATA_8_BITS,
  .parity    = UART_PARITY_DISABLE,
  .stop_bits = UART_STOP_BITS_1,
  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  .rx_flow_ctrl_thresh = 0,
  .source_clk = UART_SCLK_APB, // good default for most ESP32 chips
  .flags = {0},
};

void setup_led_pin() {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << ledPin),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
}

void setup(void) {
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.begin(Serial_Debug_Port_Baud);
  //Using for development
  //OLS_Port.begin(OLS_Port_Baud, SERIAL_8N1, 13, 12);
  #endif

  // OLS_Port.begin(OLS_Port_Baud);

  //WiFi.mode(WIFI_OFF);
  //btStop();

  // pinMode(ledPin, OUTPUT);

  dma_desc_init(CAPTURE_SIZE);

  cfg.gpio_bus[0]  = 0;
  cfg.gpio_bus[1]  = 32;//GPIO01 used for UART 0 RX, able to use it if you select different UART port (1,2) as OLS_Port
  cfg.gpio_bus[2]  = 2;
  cfg.gpio_bus[3]  = 33;//GPIO03 used for UART 0 TX
  cfg.gpio_bus[4]  = 4;
  cfg.gpio_bus[5]  = 5;
  cfg.gpio_bus[6]  = 26; //GPIO06 used for SCK, bootloop, //GPIO16 is UART2 RX
  cfg.gpio_bus[7]  = 27; //GPIO07 used for SDO, bootloop  //GPIO17 is UART2 TX

  cfg.gpio_bus[8]  = 18;//GPIO8 used for SDI, bootloop
  cfg.gpio_bus[9]  = 19;//GPIO9 lead SW_CPU_RESET on WROVER module
  cfg.gpio_bus[10] = 20;//GPI10 lead SW_CPU_RESET on WROVER module
  cfg.gpio_bus[11] = 21;//GPIO11 used for CMD, bootloop
  cfg.gpio_bus[12] = 12;
  cfg.gpio_bus[13] = 13;
  cfg.gpio_bus[14] = 14;
  cfg.gpio_bus[15] = 15;

  cfg.gpio_clk_out= 22; // Pin22 used for LedC output
  cfg.gpio_clk_in = 23; // Pin23 used for XCK input from LedC

  //GPIO 24,28,29,30,31 results bootloop

  //cfg.bits = I2S_PARALLEL_BITS_8; //not implemented yet...
  cfg.bits = I2S_PARALLEL_BITS_16;
  cfg.clkspeed_hz = 2 * 1000 * 1000; //resulting pixel clock = 1MHz
  cfg.buf = &bufdesc;

  //enable_out_clock(I2S_HZ);
  //fill_dma_desc( bufdesc );
  esp_task_wdt_deinit();

  // Call your setup
  vTaskDelay(pdMS_TO_TICKS(500));
  // xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
  i2s_parallel_setup(&cfg);

  // Re-initialize WDT (enable with default config)
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 5000,                        // 5-second timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
    .trigger_panic = false
  };
  esp_task_wdt_init(&twdt_config);

}

void captureMilli(void);
void getCmd(void);
void blinkled(void);
void get_metadata(void);

int cmdByte = 0;
uint8_t cmdBytes[5];

void command_loop_task(void *param) {
  while (1) {
      vTaskDelay(1); // Avoid WDT

      int bytes_available = 1;
      // uart_get_buffered_data_len(OLS_Port, (size_t*)&bytes_available);
      if (bytes_available > 0) {
          // uart_read_bytes(OLS_Port, &cmdByte, 1, pdMS_TO_TICKS(100));

          #ifdef _DEBUG_MODE_
          Serial_Debug_Port.printf("CMD: 0x%02X\r\n", cmdByte);
          #endif
          captureMilli();

          // switch (cmdByte) {
          //     case SUMP_RESET:
          //         break;

          //     case SUMP_QUERY:
          //         uart_write_bytes(OLS_Port, "1ALS", strlen("1ALS"));
          //         break;

          //     case SUMP_ARM:
          //         captureMilli();
          //         break;

          //     case SUMP_TRIGGER_MASK_CH_A:
          //         getCmd();
          //         trigger = ((uint16_t)cmdBytes[1] << 8 ) | cmdBytes[0];
          //         #ifdef _DEBUG_MODE_
          //         if (trigger) {
          //             Serial_Debug_Port.printf("Trigger Set for inputs : ");
          //             for (int i = 0; i < 16; i++)
          //                 if (( trigger >> i) & 0x1 )
          //                     Serial_Debug_Port.printf("%d,  ", i);
          //             Serial_Debug_Port.println();
          //         }
          //         #endif
          //         break;

          //     case SUMP_TRIGGER_VALUES_CH_A:
          //         getCmd();
          //         trigger_values = ((uint16_t)cmdBytes[1] << 8 ) | cmdBytes[0];
          //         #ifdef _DEBUG_MODE_
          //         if (trigger) {
          //             Serial_Debug_Port.printf("Trigger Val for inputs : ");
          //             for (int i = 0; i < 16; i++)
          //                 if (( trigger >> i) & 0x1 )
          //                     Serial_Debug_Port.printf("%c,  ", (( trigger_values >> i ) & 0x1 ? 'H' : 'L') );
          //             Serial_Debug_Port.println();
          //         }
          //         #endif
          //         break;

          //     case SUMP_TRIGGER_MASK_CH_B:
          //     case SUMP_TRIGGER_MASK_CH_C:
          //     case SUMP_TRIGGER_MASK_CH_D:
          //     case SUMP_TRIGGER_VALUES_CH_B:
          //     case SUMP_TRIGGER_VALUES_CH_C:
          //     case SUMP_TRIGGER_VALUES_CH_D:
          //     case SUMP_TRIGGER_CONFIG_CH_A:
          //     case SUMP_TRIGGER_CONFIG_CH_B:
          //     case SUMP_TRIGGER_CONFIG_CH_C:
          //     case SUMP_TRIGGER_CONFIG_CH_D:
          //         getCmd();
          //         break;

          //     case SUMP_SET_DIVIDER:
          //         getCmd();
          //         divider = cmdBytes[2];
          //         divider = (divider << 8) + cmdBytes[1];
          //         divider = (divider << 8) + cmdBytes[0];
          //         setupDelay();
          //         break;

          //     case SUMP_SET_READ_DELAY_COUNT:
          //         getCmd();
          //         readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
          //         if (readCount > MAX_CAPTURE_SIZE) readCount = MAX_CAPTURE_SIZE;
          //         delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
          //         if (delayCount > MAX_CAPTURE_SIZE) delayCount = MAX_CAPTURE_SIZE;
          //         break;

          //     case SUMP_SET_FLAGS:
          //         getCmd();
          //         rleEnabled = cmdBytes[1] & 0x1;
          //         channels_to_read = (~(cmdBytes[0] >> 2) & 0x0F);
          //         #ifdef _DEBUG_MODE_
          //         Serial_Debug_Port.printf("RLE: %c\n", rleEnabled ? 'Y' : 'N');
          //         Serial_Debug_Port.printf("Demux: %c\n", cmdBytes[0] & 0x01 ? 'Y' : 'N');
          //         Serial_Debug_Port.printf("Filter: %c\n", cmdBytes[0] & 0x02 ? 'Y' : 'N');
          //         Serial_Debug_Port.printf("Channels: 0x%X\n", channels_to_read);
          //         Serial_Debug_Port.printf("Ext Clock: %c\n", cmdBytes[0] & 0x40 ? 'Y' : 'N');
          //         Serial_Debug_Port.printf("Inv Clock: %c\n", cmdBytes[0] & 0x80 ? 'Y' : 'N');
          //         #endif
          //         break;

          //     case SUMP_GET_METADATA:
          //         get_metadata();
          //         break;

          //     case SUMP_SELF_TEST:
          //         break;

          //     default:
          //         #ifdef _DEBUG_MODE_
          //         Serial_Debug_Port.printf("Unrecognized cmd 0x%02X\r\n", cmdByte);
          //         #endif
          //         getCmd();
          //         break;
          }
      }
}

void getCmd() {
  vTaskDelay(pdMS_TO_TICKS(10));  // 10 milliseconds delay
  uint8_t cmdBytes[4];
  int len = 4; // Number of bytes to read
  uart_read_bytes(OLS_Port, cmdBytes, len, pdMS_TO_TICKS(100)); // Timeout after 100 ms
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("CMDs ");
  for (int q = 0; q < 4; q++) {
    Serial_Debug_Port.printf(" 0x%02X", cmdBytes[q]);
  }
  Serial_Debug_Port.println();
  #endif
}

void get_metadata() {
  /* device name */
  uint8_t device_name[] = "ESP32 Logic Analyzer v0.31";
  uart_write_bytes(OLS_Port, (const char*) &device_name[0], sizeof(device_name) - 1);
  uart_write_bytes(OLS_Port, (const char*) "\x00", 1);  // End of string

  /* firmware version */
  uart_write_bytes(OLS_Port, (const char*) "\x02", 1);
  uart_write_bytes(OLS_Port, (const char*) "0.10", 4); // Firmware version
  uart_write_bytes(OLS_Port, (const char*) "\x00", 1);  // End of string

  /* sample memory */
  uart_write_bytes(OLS_Port, (const char*) "\x21", 1);
  uint32_t capture_size = CAPTURE_SIZE;
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_size)[3], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_size)[2], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_size)[1], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_size)[0], 1);

  /* sample rate (20MHz) */
  uint32_t capture_speed = 20e6;
  uart_write_bytes(OLS_Port, (const char*) "\x23", 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_speed)[3], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_speed)[2], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_speed)[1], 1);
  uart_write_bytes(OLS_Port, (const char*) &((uint8_t*)&capture_speed)[0], 1);

  /* number of probes */
  uart_write_bytes(OLS_Port, (const char*) "\x40", 1);
  uart_write_bytes(OLS_Port, (const char*) &cfg.bits, 1);  // The number of probes

  /* protocol version (2) */
  uart_write_bytes(OLS_Port, (const char*) "\x41", 1);
  uart_write_bytes(OLS_Port, (const char*) "\x02", 1);

  /* end of data */
  uart_write_bytes(OLS_Port, (const char*) "\x00", 1);
}

void setupDelay() {
  double rate = 100000000.0 / (divider + 1.0);
  enable_out_clock((int)rate);
  #ifdef _DEBUG_MODE_
  Serial_Debug_Port.printf("Capture Speed : %.2f Mhz\r\n", rate/1000000.0);
  #endif
}

void captureMilli() {
  // Setup LED indicator
  esp_rom_gpio_pad_select_gpio(ledPin);
  gpio_set_direction(ledPin, GPIO_MODE_OUTPUT);
  gpio_set_level(ledPin, 1);

  ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
  rle_init();
  vTaskDelay(1); // yield to other tasks (1 tick)

  start_dma_capture();
  vTaskDelay(1);
  I2S0.conf.rx_start = 1;
  vTaskDelay(100); // Required delay for error-free capture

  // Record start time
  uint32_t start_time = xTaskGetTickCount();

  // Capture loop, capped to 5 seconds
  int capture_time_ms = 5000;  // 5 seconds
  int elapsed_ms = 0;

  while (!s_state->dma_done && elapsed_ms < capture_time_ms) {
      vTaskDelay(100);
      elapsed_ms += 100;
  }

  // Force stop if timeout reached but DMA not done
  if (!s_state->dma_done) {
      ESP_LOGW(TAG, "Capture timeout reached. Stopping DMA.");
      I2S0.conf.rx_start = 0;
      s_state->dma_done = true;  // Mark as done to proceed
  }

  // Stop I2S RX after timeout or done
  I2S0.conf.rx_start = 0;

  vTaskDelay(1);
  gpio_set_level(ledPin, 0);
  vTaskDelay(1);
  ESP_LOGD(TAG, "Copying buffer.");
  vTaskDelay(1);
  int filled_desc = ceil((readCount / 2.0) / s_state->dma_sample_per_desc);
  int filled_sample_offset = ((readCount / 2) % s_state->dma_sample_per_desc);
  int filled_full_sample_offset = s_state->dma_sample_per_desc;
  int tx_count = 0;
  filled_desc--;
  filled_full_sample_offset--;
  if (filled_sample_offset-- == 0)
    filled_sample_offset = filled_full_sample_offset;

  dma_elem_t cur;
  vTaskDelay(1);
  if (s_state->dma_desc_triggered < 0) {
    s_state->dma_desc_triggered = 0;
    ESP_LOGD(TAG, "Normal TX");
  } else {
    ESP_LOGD(TAG, "Triggered TX");
  }
  vTaskDelay(1);
  FILE* fp = fopen(FILE_PATH, "wb");
  if (!fp) {
    ESP_LOGE(TAG, "Failed to open file for writing.");
    return;
  }

  if (rleEnabled) {
    int rle_fill = (rle_buff_p - rle_buff);

    if (channels_to_read == 3) {
#if ALLOW_ZERO_RLE
      for (int i = rle_fill - 4; i >= 0; i -= 4) {
        uint8_t data[4] = {
          static_cast<uint8_t>(rle_buff[i + 2] | 0x00),
          static_cast<uint8_t>(rle_buff[i + 3] | 0x80),
          static_cast<uint8_t>(rle_buff[i + 0] & 0xFF),
          static_cast<uint8_t>(rle_buff[i + 1] & 0x7F)
        };
        fwrite(data, 1, 4, fp);
      }
#else
      for (int i = rle_fill - 2; i >= 0; i -= 2) {
        fwrite(&rle_buff[i], 1, 1, fp);
        fwrite(&rle_buff[i + 1], 1, 1, fp);
      }
#endif
    } else {
#if ALLOW_ZERO_RLE
      for (int i = rle_fill - 2; i >= 0; i -= 2) {
        if (rle_buff[i + 1] != 0) {
          uint8_t data[2] = {
            static_cast<uint8_t>(rle_buff[i + 1] | 0x80),
            static_cast<uint8_t>(rle_buff[i] & 0x7F)
          };
          fwrite(data, 1, 2, fp);
        }
      }
#else
      for (int i = rle_fill - 1; i >= 0; i--) {
        fwrite(&rle_buff[i], 1, 1, fp);
      }
#endif
    }

    fclose(fp);
    ESP_LOGI(TAG, "Binary capture written to %s", FILE_PATH);
  }
  else {
    FILE* f = fopen("/spiffs/capture.bin", "wb");
    if (!f) {
      ESP_LOGE(TAG, "Failed to open file for writing");
      return;
    }

    for (int j = filled_desc; j >= 0; j--) {
      ESP_LOGD(TAG, "filled_buff trgx = %d",
               (j + s_state->dma_desc_triggered + s_state->dma_desc_count) % s_state->dma_desc_count);

      gpio_set_level(ledPin, !gpio_get_level(ledPin));

      for (int i = (j == filled_desc ? filled_sample_offset : filled_full_sample_offset); i >= 0; i--) {
        dma_elem_t cur = s_state->dma_buf[
          (j + s_state->dma_desc_triggered + s_state->dma_desc_count) % s_state->dma_desc_count
        ][i];

        if (channels_to_read == 1) {
          fwrite(&cur.sample2, sizeof(cur.sample2), 1, f);
          fwrite(&cur.sample1, sizeof(cur.sample1), 1, f);
        }
        else if (channels_to_read == 2) {
          fwrite(&cur.unused2, sizeof(cur.unused2), 1, f);
          fwrite(&cur.unused1, sizeof(cur.unused1), 1, f);
        }
        else if (channels_to_read == 3) {
          fwrite(&cur.sample2, sizeof(cur.sample2), 1, f);
          fwrite(&cur.unused2, sizeof(cur.unused2), 1, f);
          fwrite(&cur.sample1, sizeof(cur.sample1), 1, f);
          fwrite(&cur.unused1, sizeof(cur.unused1), 1, f);
        }

        tx_count += 2;

        if (tx_count >= readCount) {
          #ifdef _DEBUG_MODE_
            Serial_Debug_Port.printf("BRexit triggered.");
          #endif
          fclose(f);
          goto brexit;
        }
      }
      vTaskDelay(1);
    }

    fclose(f);
  }

brexit:
  ESP_LOGD(TAG, "End. TX: %d", tx_count);
  gpio_set_level(ledPin, 0);
}
