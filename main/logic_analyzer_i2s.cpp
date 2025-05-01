#include "logic_analyzer.hpp"

i2s_parallel_config_t cfg;

  void start_dma_capture(void) {
    s_state->dma_done = false;
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    s_state->dma_filtered_count = 0;
    s_state->dma_desc_triggered = 0;

  time_debug_indice_dma_p=0;
  time_debug_indice_rle_p=0;
  for(int i=0; i < time_debug_indice_lenght; i++ )
  {
    time_debug_indice_dma[i]=time_debug_indice_rle[i]=0;
  }


    ESP_ERROR_CHECK(esp_intr_disable(s_state->i2s_intr_handle));
    i2s_conf_reset();

    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;

    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;

    ESP_LOGD(TAG, "DMA Tigger : 0x%X", trigger );
    if (trigger || rleEnabled) {
      stop_at_desc = -1;
      //Due waiting trigger, we continuously capturing, so we can't set rx_eof_num
      I2S0.rx_eof_num = s_state->dma_buf_width;
      I2S0.int_ena.in_suc_eof = 0;
      I2S0.int_ena.rx_take_data = 0;
      lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
      pd->eof = 0;
      pd->qe.stqe_next = &s_state->dma_desc[0];
    }
    else {
      s_state->dma_desc_triggered=-1;
      I2S0.rx_eof_num = readCount/2; // why /2 ???
      I2S0.int_ena.in_suc_eof = 1;
      I2S0.int_ena.rx_take_data = 1;
      //
      lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
      pd->eof = 1;
      pd->qe.stqe_next = 0x0;
    }

    //clear dma buffers
    for (int i = 0; i < s_state->dma_desc_count; ++i)
    {
      memset( s_state->dma_buf[i], 0, s_state->dma_desc[i].length);
    }

    //Enable the Interrupt
    ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));

    //attachInterrupt(0, i2s_trigger_isr, RISING);

    //taskENTER_CRITICAL(&myMutex);
    //I2S0.conf.rx_start = 1;
  }

  uint16_t buff_process_trigger_1(uint16_t *buff, int size, bool printit=true){
    uint16_t a = 0;
    for(int i=0 ; i < size/2 ; i+=2)
    {
      a |= buff[i];
      if(printit)
        ESP_LOGD(TAG, "Process trigger 1: 0x%X", a);
    }
    return a;
    }

  uint16_t buff_process_trigger_0(uint16_t *buff, int size, bool printit=true){
    uint16_t a = 0xFF;
    for(int i=0 ; i < size/2 ; i+=2)
    a &= buff[i];
    if(printit)
      ESP_LOGD(TAG, "Process trigger 0: 0x%X", a);
    return a;
    }

  static void IRAM_ATTR i2s_isr(void* arg) {
    if(trigger==0){
      //ESP_LOGD(TAG, "DMA INT Number %d Status 0x%x", s_state->dma_desc_cur, I2S0.int_raw.val );

    /*
    ESP_LOGD(TAG, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
    ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
    ESP_LOGD(TAG, "DMA INT in_done? %d", I2S0.int_raw.in_done );
    ESP_LOGD(TAG, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
    ESP_LOGD(TAG, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );
    ESP_LOGD( "\r\n" );
    */

    //ESP_LOGD(TAG, "Executing DMA ISR on core %d", xPortGetCoreID() );
    }

    //gpio_set_level(, 1); //Should show a pulse on the logic analyzer when an interrupt occurs
    //gpio_set_level(, 0);
    if(I2S0.int_raw.in_done){ //filled desc
      if(time_debug_indice_dma_p < time_debug_indice_lenght) time_debug_indice_dma[time_debug_indice_dma_p++]=xthal_get_ccount();

      //if(channels_to_read==1)
      //  dmabuff_compresser_ch1( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count] );
      //else if(channels_to_read==2)
      //  dmabuff_compresser_ch2( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count] );

      if(time_debug_indice_dma_p < time_debug_indice_lenght) time_debug_indice_rle[time_debug_indice_rle_p++]=xthal_get_ccount();


      //Serial_Debug_Port.printf("DMA INT Number %d Status 0x%xX\r\n", s_state->dma_desc_cur, I2S0.int_raw.val);

      if(trigger && (stop_at_desc==-1)){
        static uint16_t trigger_helper_0;
        static uint16_t trigger_helper_1;
        trigger_helper_0 = buff_process_trigger_0((uint16_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count ], s_state->dma_buf_width, false);
        trigger_helper_1 = buff_process_trigger_1((uint16_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count ], s_state->dma_buf_width, false);
        ESP_LOGD(TAG, "DMA INT Number %d Status 0x%x TL0: x%X TH1: 0x%X",
          s_state->dma_desc_cur,
          (unsigned int)I2S0.int_raw.val,
          (unsigned int)trigger_helper_0,
          (unsigned int)trigger_helper_1);

        //ESP_LOGD(TAG, "trigger ^ trigger_values 0x%X trigger ^ trigger_values ^ trigger_helper_0 0x%x ", trigger ^ trigger_values,  trigger ^ trigger_values ^ trigger_helper_0 );
        if(( trigger_values & trigger_helper_1 ) || ( (trigger ^ trigger_values) & ~trigger_helper_0 ) ){
          ESP_LOGD(TAG, "DMA Triggered at desc %d (%d)",  s_state->dma_desc_triggered, s_state->dma_desc_triggered%s_state->dma_desc_count );
          if(rleEnabled){
            if( channels_to_read == 1 )
                fast_rle_block_encode_asm_8bit_ch1( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
            else if ( channels_to_read == 2 )
                fast_rle_block_encode_asm_8bit_ch2( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
            else
                fast_rle_block_encode_asm_16bit( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
            }
          else{
            stop_at_desc= (s_state->dma_desc_cur + (readCount / (s_state->dma_buf_width/2))-1) % s_state->dma_desc_count;
            ESP_LOGD(TAG, "DMA BREAK! Stop at desc %d", stop_at_desc);

            s_state->dma_desc_triggered = s_state->dma_desc_cur;
            I2S0.rx_eof_num = readCount/2 ;//- s_state->dma_buf_width/4000; // Total capacity - desc capacity
            I2S0.int_ena.in_suc_eof = 1;
            }
          trigger=0;
          I2S0.int_clr.val = I2S0.int_raw.val;
          }
        }
      else if(rleEnabled){
        ESP_LOGD(TAG,"Processing DMA Desc: %d (%d)\r\n", s_state->dma_desc_cur,  s_state->dma_desc_cur % s_state->dma_desc_count);
        //Serial_Debug_Port.printf("Processing DMA Desc: %d (%d)\r\n", s_state->dma_desc_cur,  s_state->dma_desc_cur % s_state->dma_desc_count);
        //Serial_Debug_Port.printf(".");
          if(rleEnabled){
            if( channels_to_read == 1 )
                fast_rle_block_encode_asm_8bit_ch1( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
            else if ( channels_to_read == 2 )
                fast_rle_block_encode_asm_8bit_ch2( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
            else
                fast_rle_block_encode_asm_16bit( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);

            if(time_debug_indice_rle_p < time_debug_indice_lenght)
              time_debug_indice_rle[time_debug_indice_rle_p++]=xthal_get_ccount();
            }

        if( (rle_size - (rle_buff_p - rle_buff )) < 4000) {
          //break;
          ESP_LOGD(TAG,"Break due rle_buff fill: %d\r\n", (rle_size - (rle_buff_p - rle_buff )));
          I2S0.int_ena.in_suc_eof = 1;
          esp_intr_disable(s_state->i2s_intr_handle);
          i2s_conf_reset();
          I2S0.conf.rx_start = 0;
          s_state->dma_done = true;
          }

        //rle doesn't need to stop at rx eof
        I2S0.int_clr.val = I2S0.int_raw.val;
        s_state->dma_desc_cur++;
        return;
        }
      s_state->dma_desc_cur++;
      }

    if(trigger==0)  //Not stop while not triggered.
    if(
      //I2S0.int_raw.in_dscr_empty ||
        I2S0.int_raw.in_suc_eof   ||
        I2S0.int_raw.rx_take_data || //>20 Mhz
      //!I2S0.int_raw.rx_rempty || //<=20khz
      s_state->dma_desc_cur == s_state->dma_desc_count || //triggger
      0
      )
      {
        ESP_LOGD(TAG, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
        ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
        ESP_LOGD(TAG, "DMA INT in_done? %d", I2S0.int_raw.in_done );
        ESP_LOGD(TAG, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
        ESP_LOGD(TAG, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );

        //s_state->dma_desc_cur=0;
        esp_intr_disable(s_state->i2s_intr_handle);
        i2s_conf_reset();
        I2S0.conf.rx_start = 0;
        s_state->dma_done = true;
     }
    //vTaskDelay(1);
    I2S0.int_clr.val = I2S0.int_raw.val;
    }

  static void dma_desc_deinit(){
      ESP_ERROR_CHECK(esp_intr_disable(s_state->i2s_intr_handle));
      if (s_state->dma_buf) {
          for (int i = 0; i < s_state->dma_desc_count; ++i) {
              free(s_state->dma_buf[i]);
          }
      }
      free(s_state->dma_buf);
      free(s_state->dma_desc);
  }

  esp_err_t dma_desc_init(int raw_byte_size){
      s_state = (camera_state_t*) malloc (sizeof(camera_state_t));
      assert(raw_byte_size % 4 == 0);

      ESP_LOGD(TAG, "Buffer Total (for DMA): %d bytes", raw_byte_size);
      size_t dma_desc_count = 1;
      size_t buf_size = raw_byte_size;
      /*
      while (buf_size >= 4096)
      {
          buf_size /= 2;
          dma_desc_count *= 2;
      }
      s_state->dma_buf_width = buf_size;
      s_state->dma_val_per_desc = buf_size/2;
      s_state->dma_sample_per_desc = buf_size/4;
      s_state->dma_desc_count = dma_desc_count;
  */
      s_state->dma_buf_width = buf_size = 4000;
      s_state->dma_val_per_desc = 2000;
      s_state->dma_sample_per_desc = 1000; //4bytes has only 2bytes sample on 16bit mode.
      s_state->dma_desc_count = dma_desc_count = raw_byte_size/4000;

      ESP_LOGD(TAG, "DMA buffer size: %d", buf_size);
      ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);
      ESP_LOGD(TAG, "DMA buffer total: %d bytes", buf_size * dma_desc_count);

      s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
      if (s_state->dma_buf == NULL) {
          return ESP_ERR_NO_MEM;
      }
      s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
      if (s_state->dma_desc == NULL) {
          return ESP_ERR_NO_MEM;
      }
      size_t dma_sample_count = 0;
      for (int i = 0; i < dma_desc_count; ++i) {
          ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
          dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
          if (buf == NULL) {
              ESP_LOGD(TAG, "NO_MEM!");
              return ESP_ERR_NO_MEM;
          }
          s_state->dma_buf[i] = buf;
          ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);

          lldesc_t* pd = &s_state->dma_desc[i];
          pd->length = buf_size/2;
          dma_sample_count += buf_size / 2; // indeed /4 because each sample is 4 bytes
          pd->size = buf_size;
          pd->owner = 1;
          pd->sosf = 1;
          pd->buf = (uint8_t*) buf;
          pd->offset = 0;
          pd->empty = 0;
          pd->eof = 0;
          pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
          if( i+1 == dma_desc_count ){
             pd->eof = 1;
             pd->qe.stqe_next = 0x0;
             //pd->eof = 0;
             //pd->qe.stqe_next = &s_state->dma_desc[0];
            }
      }
      s_state->dma_done = false;
      s_state->dma_sample_count = dma_sample_count;
      ESP_LOGD(TAG, "DMA dma_sample_count: %d", dma_sample_count);
      return ESP_OK;
  }

static void gpio_setup_in(int gpio, int sig, int inv) {
    if (gpio == -1) return;

    // Configure as input
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Route the GPIO input signal to the peripheral signal using gpio_matrix_in
    gpio_matrix_in(gpio, sig, inv);
}

  void i2s_conf_reset(){
    // Toggle some reset bits in LC_CONF register
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;

    // Toggle some reset bits in CONF register
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.conf.tx_fifo_reset = 1;
    I2S0.conf.tx_fifo_reset = 0;
    while (I2S0.state.rx_fifo_reset_back) {}
    }

  void i2s_parallel_setup(const i2s_parallel_config_t *cfg) {

    //Figure out which signal numbers to use for routing
    ESP_LOGI(TAG, "Setting up parallel I2S bus at I2S%d\n", 0);
    int sig_data_base, sig_clk;

    sig_data_base = I2S0I_DATA_IN0_IDX;
    sig_clk       = I2S0I_WS_IN_IDX;
    //sig_clk       = I2S0I_BCK_IN_IDX;

    //Route the signals
    gpio_setup_in(cfg->gpio_bus[0], sig_data_base + 0, false); // D0
    gpio_setup_in(cfg->gpio_bus[1], sig_data_base + 1, false); // D1
    gpio_setup_in(cfg->gpio_bus[2], sig_data_base + 2, false); // D2
    gpio_setup_in(cfg->gpio_bus[3], sig_data_base + 3, false); // D3
    gpio_setup_in(cfg->gpio_bus[4], sig_data_base + 4, false); // HS
    gpio_setup_in(cfg->gpio_bus[5], sig_data_base + 5, false); // VS
    gpio_setup_in(cfg->gpio_bus[6], sig_data_base + 6, false); // VS
    gpio_setup_in(cfg->gpio_bus[7], sig_data_base + 7, false); // VS

    gpio_setup_in(cfg->gpio_bus[8], sig_data_base + 8, false); // VS
    gpio_setup_in(cfg->gpio_bus[9], sig_data_base + 9, false); // VS
    gpio_setup_in(cfg->gpio_bus[10], sig_data_base + 10, false); // VS
    gpio_setup_in(cfg->gpio_bus[11], sig_data_base + 11, false); // VS
    gpio_setup_in(cfg->gpio_bus[12], sig_data_base + 12, false); // VS
    gpio_setup_in(cfg->gpio_bus[13], sig_data_base + 13, false); // VS
    gpio_setup_in(cfg->gpio_bus[14], sig_data_base + 14, false); // VS
    gpio_setup_in(cfg->gpio_bus[15], sig_data_base + 15, false); // VS

    gpio_setup_in(cfg->gpio_clk_in, sig_clk, false);
  //gpio_matrix_in(0x38,    I2S0I_WS_IN_IDX, false);

    gpio_matrix_in(0x38,    I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38,    I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(0x38,    I2S0I_H_ENABLE_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);



    //Initialize I2S dev
    // Toggle some reset bits in LC_CONF register
    // Toggle some reset bits in CONF register
    i2s_conf_reset();

    I2S0.conf.rx_slave_mod = 1;  // Enable slave mode (sampling clock is external)
    I2S0.conf2.val = 0;          // Enable LCD mode
    I2S0.conf2.lcd_en = 1;       // Enable parallel mode
    I2S0.conf2.camera_en = 1;    // Use HSYNC/VSYNC/HREF to control sampling

    // f i2s = fpll / (Num + b/a )) where fpll=80Mhz
    I2S0.clkm_conf.val = 0;       // Configure clock divider

    I2S0.clkm_conf.clka_en = 0;   // select PLL_D2_CLK. Digital Multiplexer that select between APLL_CLK or PLL_D2_CLK.
    //I2S0.clkm_conf.clk_en = 1;

    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 4;

    /*
    I2S0.clkm_conf.clkm_div_a = 3;//  24Mhz
    I2S0.clkm_conf.clkm_div_b = 1;
    I2S0.clkm_conf.clkm_div_num = 3;
    */

    I2S0.fifo_conf.dscr_en = 1;   // FIFO will sink data to DMA

    // FIFO configuration
    //I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod = 1; // SM_0A0B_0C0D = 1,
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    //dev->conf_chan.val = 0;
    I2S0.conf_chan.rx_chan_mod = 1;

    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 1;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 1;
    I2S0.conf.rx_short_sync = 1;
  //  I2S0.conf.rx_mono = 0;
  //  I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;
  //  I2S0.timing.rx_dsync_sw = 1;

    I2S0.sample_rate_conf.val = 0;
    // Clear flags which are used in I2S serial mode
    //I2S0.sample_rate_conf.rx_bits_mod = 8;
    //I2S0.sample_rate_conf.rx_bits_mod = 16;
    I2S0.sample_rate_conf.rx_bits_mod = cfg->bits;
    //dev->sample_rate_conf.rx_bck_div_num = 16; //ToDo: Unsure about what this does...
    I2S0.sample_rate_conf.rx_bck_div_num = 1;  // datasheet says this must be 2 or greater (but 1 seems to work)

    // this combination is 20MHz
    //dev->sample_rate_conf.tx_bck_div_num=1;
    //dev->clkm_conf.clkm_div_num=3; // datasheet says this must be 2 or greater (but lower values seem to work)

    //Allocate DMA descriptors
    i2s_state[0] = (i2s_parallel_state_t*)malloc(sizeof(i2s_parallel_state_t));
    i2s_parallel_state_t *st = i2s_state[0];

    s_state->dma_done = false;
    s_state->dma_desc_cur = 0;
    s_state->dma_received_count = 0;
    s_state->dma_filtered_count = 0;
    //esp_intr_disable(s_state->i2s_intr_handle);
    // i2s_conf_reset();


    ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
    I2S0.rx_eof_num = s_state->dma_sample_count;
    I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;

    //Setup I2S DMA Interrupt
    esp_err_t err = esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
               ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED,
               &i2s_isr, NULL, &s_state->i2s_intr_handle);


    //Enable the Interrupt
    //ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));

  //  I2S0.conf.rx_start = 1;
  }

  #include "driver/ledc.h"
  #include "driver/periph_ctrl.h"

  void enable_out_clock(uint32_t freq_in_hz) {
    ledc_timer_config_t ledc_timer{};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.duty_resolution  = LEDC_TIMER_1_BIT;
    ledc_timer.freq_hz          = freq_in_hz;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        // handle error
    }

    ledc_channel_config_t ledc_channel{};
    ledc_channel.gpio_num       = cfg.gpio_clk_out;
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_0;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.duty           = 1;
    ledc_channel.hpoint         = 0;

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        // handle error
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}

bool rle_init(void){
  rle_buff_p=0;
  rle_sample_counter=0;
  rle_total_sample_counter=0;
  rle_value_holder=0;
  rle_process=-1;

  rle_buff_p=rle_buff;
  rle_buff_end = rle_buff+rle_size-4;

  memset( rle_buff, 0x00, rle_size);
  return true;
}

void dma_serializer( dma_elem_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4
i2s_parallel_state_t* i2s_state[2] = {NULL, NULL};
  for ( int i = 0 ; i < s_state->dma_buf_width/4 ; i++ ){
     uint8_t y =  dma_buffer[i].sample2;
     dma_buffer[i].sample2 = dma_buffer[i].sample1;
     dma_buffer[i].sample1 = y;
   }
}


void dmabuff_compresser_ch1( uint8_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4.... - > s1,s2,s3,s4,00,00,00,00...
  for ( int i=0, p=0 ; i < s_state->dma_buf_width-4 ; i+=4 ){
   dma_buffer[p++]=dma_buffer[i+1];
   dma_buffer[p++]=dma_buffer[i+3];
   }
}
void dmabuff_compresser_ch2( uint8_t *dma_buffer ){
  //00,s1,00,s2,00,s3,00,s4.... - > s1,s2,s3,s4,00,00,00,00...
  for ( int i=0, p=0 ; i < s_state->dma_buf_width-4 ; i+=4 ){
   dma_buffer[p++]=dma_buffer[i+0];
   dma_buffer[p++]=dma_buffer[i+2];
   }
}

extern "C" {
void fast_rle_block_encode_asm_8bit_ch1(uint8_t *dma_buffer, uint32_t sample_size){ //size, not count
//    uint8_t *desc_buff_end=dma_buffer;
//    unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */

   int dword_count=(sample_size/4) -1;

//    clocka = xthal_get_ccount();

   /* No, Assembly is not that hard. You are just too lazzy. */

   // "a4"=&dma_buffer, "a5"=dword_count, "a6"=&rle_buff_p:

    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 2        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 0        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0          \n" // rle_val == new_val skip

#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_0:                \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end    \n" // Prepare zero-overhead loop
      "loopStart:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start:          \n"
      "l8ui a9, a4, 2        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add    \n" // if count >= 128 branch
      "rle_1_127:            \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127    \n" // if count >= 128 branch

      "rle_1_add:            \n"
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction.

      "rle_1_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start:          \n"
      "l8ui a9, a4, 0        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add    \n" // if count >= 128 branch
      "rle_2_127:            \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127    \n" // if count >= 128 branch

      "rle_2_add:            \n"
#if ALLOW_ZERO_RLE
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end:              \n"

"bltui a5, 128, rle_end_add  \n" // if count >= 128 branch
      "rle_end_127:          \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127  \n" // if count >= 128 branch

      "rle_end_add:          \n"
#if ALLOW_ZERO_RLE
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
      "and  a8, a8, a11      \n" //
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip       \n"
      "or   a5, a5, a10      \n" //
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_end_skip:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");


//     clockb = xthal_get_ccount();
    //if(time_debug_indice_rle_p < time_debug_indice_lenght)
    //  time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;

 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

}

void fast_rle_block_encode_asm_8bit_ch2(uint8_t *dma_buffer, uint32_t sample_size){ //size, not count
//    uint8_t *desc_buff_end=dma_buffer;
//    unsigned clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
    * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
    */

   int dword_count=(sample_size/4) -1;

//    clocka = xthal_get_ccount();

   /* No, Assembly is not that hard. You are just too lazy. */

    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 3        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 1        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0_ch2      \n" // rle_val == new_val skip

#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_0_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end_ch2    \n" // Prepare zero-overhead loop
      "loopStart_ch2:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_ch2:          \n"
      "l8ui a9, a4, 3        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end_ch2      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add_ch2    \n" // if count >= 128 branch
      "rle_1_127_ch2:            \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127_ch2    \n" // if count >= 128 branch

      "rle_1_add_ch2:            \n"
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip_ch2         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip_ch2:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction.

      "rle_1_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_ch2:          \n"
      "l8ui a9, a4, 1        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end_ch2      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add_ch2   \n" // if count >= 128 branch
      "rle_2_127_ch2:            \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127_ch2    \n" // if count >= 128 branch

      "rle_2_add_ch2:            \n"
#if ALLOW_ZERO_RLE
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip_ch2         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip_ch2:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_ch2:              \n"

"bltui a5, 128, rle_end_add_ch2  \n" // if count >= 128 branch
      "rle_end_127_ch2:          \n"
#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127_ch2  \n" // if count >= 128 branch

      "rle_end_add_ch2:          \n"
#if ALLOW_ZERO_RLE
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2
#else
      "and  a8, a8, a11      \n" //
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip_ch2       \n"
      "or   a5, a5, a10      \n" //
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_end_skip_ch2:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit_ch2:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");


//     clockb = xthal_get_ccount();

 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

}

void fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, uint32_t sample_size){ //size, not count
   //uint8_t *desc_buff_end=dma_buffer;
   //uint32_t clocka=0,clockb=0;

   /* We have to encode RLE samples quick.
    * Each sample need to be encoded under 12 clocks @240Mhz CPU
    * for capture 20Mhz sampling speed.
    */

   /* expected structure of DMA memory    : S1s1,S2s2,S3s3,S4s4
    * actual data structure of DMA memory : S2s2,S1s1,S4s4,S3s3
    */

   int dword_count=(sample_size/4) -1;

   //clocka = xthal_get_ccount();

   /* No, Assembly is not that hard. You are just too lazzy. */

    __asm__ __volatile__(
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l16ui a8, a4, 2       \n" // a8 as rle_val #2 is first   a8=&(dma_buffer+2)
      "l16ui a9, a4, 0       \n" // a9 as new_val               a9=&dma_buffer
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x8000      \n" // init rle_masks for count
      "movi a11, 0x7FFF      \n" // init rle_masks for data

"beq  a9, a8, rle_0_16       \n" // rle_val == new_val skip

#if !ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7FFF
#endif
      "s16i  a8, a6, 0       \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s16i  a5, a6, 2       \n" // rle_buff_p+2 = rle_counter //not needed
      "addi a6, a6, 2        \n" // rle_buff_p +=2             //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 2        \n" // rle_buff_p +=2
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_0_16:             \n"
      "addi a5, a5, 1        \n" // rle_counter++
"loopnez %1, rle_loop_end_16 \n" // Prepare zero-overhead loop
      "loopStart_16:         \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_16:       \n"
      "l16ui a9, a4, 2       \n" // a9 as rle_val #2 is first
"beq  a9, a8, rle_1_end_16   \n" // rle_val == new_val skip

      "rle_1_127_16:         \n" //not needed

      "rle_1_add_16:         \n"
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 22.2k clocks
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // * 0x7F
      "s16i  a8, a6, 0       \n"// *rle_buff_p=rle_val;
"beqz a5, rle_1_skip_16      \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n"// *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p --
      "rle_1_skip_16:        \n"
      "addi a6, a6, 2        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1

      "rle_1_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_16:       \n"
      "l16ui a9, a4, 0       \n" // a9 as rle_val #0 is second
"beq  a9, a8, rle_2_end_16   \n" // rle_val == new_val continue

      "rle_2_add_16:         \n"
#if ALLOW_ZERO_RLE
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_2_skip_16      \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p --
      "rle_2_skip_16:        \n"
      "addi a6, a6, 2        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_16:      \n"

      "rle_end_add_16:       \n"
#if ALLOW_ZERO_RLE
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_end_skip_16    \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p --
      "rle_end_skip_16:      \n"
      "addi a6, a6, 2        \n" // rle_buff_p --
#endif

      "exit_16:              \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

    //clockb = xthal_get_ccount();

    //if(time_debug_indice_rle_p < time_debug_indice_lenght)
    //time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb);
      //time_debug_indice_rle[time_debug_indice_rle_p++]=(clockb-clocka);

    //Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    //Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    //Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );


/*
    Serial_Debug_Port.printf("RLE Block Output:\r\n");
    for(int i=0; i < sample_size/40 ; i++ )
      Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
    Serial_Debug_Port.println();
    */
}
}