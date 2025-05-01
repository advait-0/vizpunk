static const char* TAG = "esp32la";

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "soc/i2s_struct.h"
#include "rom/lldesc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "esp_private/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "esp_attr.h"
#include "hal/gpio_hal.h"
#include "esp32/rom/gpio.h"
#include "esp_task_wdt.h"

// extern i2s_parallel_config_t cfg;

#ifdef __cplusplus
extern "C" {
#endif
void setup(void);
void command_loop_task(void *param);
void fast_rle_block_encode_asm_8bit_ch1(uint8_t* buf, uint32_t len);
void fast_rle_block_encode_asm_8bit_ch2(uint8_t* buf, uint32_t len);
void fast_rle_block_encode_asm_16bit(uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif


esp_err_t dma_desc_init(int raw_byte_size);
void setupDelay();
void enable_out_clock( uint32_t freq_in_hz );
void start_dma_capture(void);
void i2s_conf_reset();
void command_loop_task(void *param);

#define _DEBUG_MODE_x

#ifndef _DEBUG_MODE_

#define OLS_Port UART_NUM_1      //Serial //Serial1 //Serial2
#define OLS_Port_Baud 921600 //115200 // 3e6

#else
#define OLS_Port Serial2     //Serial //Serial1 //Serial2
#define OLS_Port_Baud 3e6    //115200 // 3e6

#define Serial_Debug_Port Serial //Serial1 //Serial2
#define Serial_Debug_Port_Baud 921600 //115200
#endif

#define ALLOW_ZERO_RLE 0

 /// ALLOW_ZERO_RLE 1 is Fast mode.
 //Add RLE Count 0 to RLE stack for non repeated values and postpone the RLE processing so faster.
 // 8Bit Mode : ~28.4k clock per 4k block, captures 3000us while inspecting ~10Mhz clock at 20Mhz mode
 //16Bit Mode : ~22.3k clock per 4k block, captures 1500us while inspecting ~10Mhz clock at 20Mhz mode

 /// ALLOW_ZERO_RLE 0 is Slow mode.
 //just RAW RLE buffer. It doesn't add 0 count values for non-repeated RLE values and process flags on the fly, so little slow but efficient.
 // 8Bit Mode : ~34.7k clock per 4k block, captures 4700us while inspecting ~10Mhz clock at 20Mhz mode
 //16Bit Mode : ~30.3k clock per 4k block, captures 2400us while inspecting ~10Mhz clock at 20Mhz mode

#define CAPTURE_SIZE 128000
//#define CAPTURE_SIZE 12000
#define rle_size 96000

#define ledPin GPIO_NUM_21 //Led on while running and Blinks while transfering data.

extern uint32_t time_debug_indice_dma[10];
extern uint16_t time_debug_indice_dma_p;

extern uint32_t time_debug_indice_rle[10];
extern uint16_t time_debug_indice_rle_p;

extern uint16_t time_debug_indice_lenght;

extern int stop_at_desc;
extern unsigned int logicIndex;
extern unsigned int triggerIndex;
extern uint32_t readCount;
extern unsigned int delayCount;
extern uint16_t trigger;
extern uint16_t trigger_values;
extern unsigned int useMicro;
extern unsigned int delayTime;
extern unsigned long divider;
extern bool rleEnabled;
extern uint32_t clock_per_read;


typedef enum {
  I2S_PARALLEL_BITS_8   = 8,
  I2S_PARALLEL_BITS_16  = 16,
  I2S_PARALLEL_BITS_32  = 32,
} i2s_parallel_cfg_bits_t;

typedef struct {
  void* memory;
  size_t size;
} i2s_parallel_buffer_desc_t;

typedef struct {
  int8_t gpio_bus[16];
  int8_t gpio_clk_in;
  int8_t gpio_clk_out;

  int clkspeed_hz;
  i2s_parallel_cfg_bits_t bits;
  i2s_parallel_buffer_desc_t* buf;
} i2s_parallel_config_t;

typedef struct {
  volatile lldesc_t* dmadesc;
  int desccount;
} i2s_parallel_state_t;

extern i2s_parallel_state_t* i2s_state[2];

#define DMA_MAX (4096-4)

//Calculate the amount of dma descs needed for a buffer desc
static int calc_needed_dma_descs_for(i2s_parallel_buffer_desc_t *desc) {
  int ret = (desc->size + DMA_MAX - 1) / DMA_MAX;
  return ret;
}

typedef union {
    struct {
        uint8_t sample2;
        uint8_t unused2;
        uint8_t sample1;
        uint8_t unused1;
      };
    struct{
      uint16_t val2;
      uint16_t val1;
      };
    uint32_t val;
} dma_elem_t;

typedef enum {
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */

    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, .
     *
     * but appears as 00 s2 00 s1, 00 s4 00 s3 at DMA buffer somehow...
     *
     */

    SM_0A00_0B00 = 3,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
} i2s_sampling_mode_t;

typedef struct {
    lldesc_t *dma_desc;
    dma_elem_t **dma_buf;
    bool dma_done;
    size_t dma_desc_count;
    size_t dma_desc_cur;
    int dma_desc_triggered;
    size_t dma_received_count;
    size_t dma_filtered_count;
    size_t dma_buf_width;
    size_t dma_sample_count;
    size_t dma_val_per_desc;
    size_t dma_sample_per_desc;
    i2s_sampling_mode_t sampling_mode;
//    dma_filter_t dma_filter;
    intr_handle_t i2s_intr_handle;
//    QueueHandle_t data_ready;
//    SemaphoreHandle_t frame_ready;
//    TaskHandle_t dma_filter_task;
} camera_state_t;

extern camera_state_t *s_state;

void i2s_parallel_setup( const i2s_parallel_config_t *cfg);

extern uint8_t channels_to_read;
/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK_CH_A 0xC0
#define SUMP_TRIGGER_MASK_CH_B 0xC4
#define SUMP_TRIGGER_MASK_CH_C 0xC8
#define SUMP_TRIGGER_MASK_CH_D 0xCC

#define SUMP_TRIGGER_VALUES_CH_A 0xC1
#define SUMP_TRIGGER_VALUES_CH_B 0xC5
#define SUMP_TRIGGER_VALUES_CH_C 0xC9
#define SUMP_TRIGGER_VALUES_CH_D 0xCD

#define SUMP_TRIGGER_CONFIG_CH_A 0xC2
#define SUMP_TRIGGER_CONFIG_CH_B 0xC6
#define SUMP_TRIGGER_CONFIG_CH_C 0xCA
#define SUMP_TRIGGER_CONFIG_CH_D 0xCE

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#define MAX_CAPTURE_SIZE CAPTURE_SIZE

// logic_analyzer.hpp

extern int8_t    rle_process;
extern uint8_t   rle_buff[rle_size];
extern uint8_t*  rle_buff_p;
extern uint8_t*  rle_buff_end;
extern uint8_t   rle_sample_counter;
extern uint32_t  rle_total_sample_counter;
extern uint8_t   rle_value_holder;


