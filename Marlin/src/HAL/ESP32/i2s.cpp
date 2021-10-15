/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfigPre.h"

#include "i2s.h"

#include "../shared/Marduino.h"
#include <driver/periph_ctrl.h>
#include <soc/lldesc.h>
#include <soc/i2s_struct.h>
#include <freertos/queue.h>
#include "../../module/stepper.h"

#define DMA_BUF_COUNT 8                                // number of DMA buffers to store data//用于存储数据的DMA缓冲区数
#define DMA_BUF_LEN   4092                             // maximum size in bytes//最大大小（字节）
#define I2S_SAMPLE_SIZE 4                              // 4 bytes, 32 bits per sample//4字节，每个样本32位
#define DMA_SAMPLE_COUNT DMA_BUF_LEN / I2S_SAMPLE_SIZE // number of samples per buffer//每个缓冲区的样本数

typedef enum {
  I2S_NUM_0 = 0x0,  /*!< I2S 0*/
  I2S_NUM_1 = 0x1,  /*!< I2S 1*/
  I2S_NUM_MAX,
} i2s_port_t;

typedef struct {
  uint32_t     **buffers;
  uint32_t     *current;
  uint32_t     rw_pos;
  lldesc_t     **desc;
  xQueueHandle queue;
} i2s_dma_t;

static portMUX_TYPE i2s_spinlock[I2S_NUM_MAX] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED};

// TODO simplify this don't need ref to i2s1//要简化这一点，不需要参考i2s1
#if CONFIG_IDF_TARGET_ESP32
  static i2s_dev_t* I2S[I2S_NUM_MAX] = {&I2S0, &I2S1};
#elif CONFIG_IDF_TARGET_ESP32S2
  static i2s_dev_t* I2S[I2S_NUM_MAX] = {&I2S0};
#endif
static i2s_dma_t dma;

// output value//产值
uint32_t i2s_port_data = 0;

#define I2S_ENTER_CRITICAL()  portENTER_CRITICAL(&i2s_spinlock[i2s_num])
#define I2S_EXIT_CRITICAL()   portEXIT_CRITICAL(&i2s_spinlock[i2s_num])

static inline void gpio_matrix_out_check(uint32_t gpio, uint32_t signal_idx, bool out_inv, bool oen_inv) {
  //if pin = -1, do not need to configure//如果引脚=-1，则不需要配置
  if (gpio != -1) {
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction((gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, signal_idx, out_inv, oen_inv);
  }
}

static esp_err_t i2s_reset_fifo(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  I2S[i2s_num]->conf.rx_fifo_reset = 1;
  I2S[i2s_num]->conf.rx_fifo_reset = 0;
  I2S[i2s_num]->conf.tx_fifo_reset = 1;
  I2S[i2s_num]->conf.tx_fifo_reset = 0;
  I2S_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t i2s_start(i2s_port_t i2s_num) {
  //start DMA link//启动DMA链路
  I2S_ENTER_CRITICAL();
  i2s_reset_fifo(i2s_num);

  //reset dma//重置dma
  I2S[i2s_num]->lc_conf.in_rst = 1;
  I2S[i2s_num]->lc_conf.in_rst = 0;
  I2S[i2s_num]->lc_conf.out_rst = 1;
  I2S[i2s_num]->lc_conf.out_rst = 0;

  I2S[i2s_num]->conf.tx_reset = 1;
  I2S[i2s_num]->conf.tx_reset = 0;
  I2S[i2s_num]->conf.rx_reset = 1;
  I2S[i2s_num]->conf.rx_reset = 0;

  I2S[i2s_num]->int_clr.val = 0xFFFFFFFF;
  I2S[i2s_num]->out_link.start = 1;
  I2S[i2s_num]->conf.tx_start = 1;
  I2S_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t i2s_stop(i2s_port_t i2s_num) {
  I2S_ENTER_CRITICAL();
  I2S[i2s_num]->out_link.stop = 1;
  I2S[i2s_num]->conf.tx_start = 0;

  I2S[i2s_num]->int_clr.val = I2S[i2s_num]->int_st.val; //clear pending interrupt//清除挂起中断
  I2S_EXIT_CRITICAL();

  return ESP_OK;
}

static void IRAM_ATTR i2s_intr_handler_default(void *arg) {
  int dummy;
  lldesc_t *finish_desc;
  portBASE_TYPE high_priority_task_awoken = pdFALSE;

  if (I2S0.int_st.out_eof) {
    // Get the descriptor of the last item in the linkedlist//获取linkedlist中最后一项的描述符
    finish_desc = (lldesc_t*) I2S0.out_eof_des_addr;

    // If the queue is full it's because we have an underflow,//如果队列已满，那是因为我们有下溢，
    // more than buf_count isr without new data, remove the front buffer//超过buf_计数的isr没有新数据，请删除前缓冲区
    if (xQueueIsQueueFullFromISR(dma.queue))
      xQueueReceiveFromISR(dma.queue, &dummy, &high_priority_task_awoken);

    xQueueSendFromISR(dma.queue, (void *)(&finish_desc->buf), &high_priority_task_awoken);
  }

  if (high_priority_task_awoken == pdTRUE) portYIELD_FROM_ISR();

  // clear interrupt//清除中断
  I2S0.int_clr.val = I2S0.int_st.val; //clear pending interrupt//清除挂起中断
}

void stepperTask(void *parameter) {
  SERIAL_ECHO_MSG("stepperTask");
  uint32_t remaining = 0;

  while (1) {
    xQueueReceive(dma.queue, &dma.current, portMAX_DELAY);
    dma.rw_pos = 0;

    while (dma.rw_pos < DMA_SAMPLE_COUNT) {
      // Fill with the port data post pulse_phase until the next step//填充端口数据后脉冲_相位，直到下一步
      if (remaining) {
        i2s_push_sample();
        remaining--;
      }
      else {
        Stepper::pulse_phase_isr();
        remaining = Stepper::block_phase_isr();
      }
    }
  }
}

int i2s_init() {

         SERIAL_ECHO_MSG("i2s_init");

  periph_module_enable(PERIPH_I2S0_MODULE);

  /**
   * Each i2s transfer will take
   *   fpll = PLL_D2_CLK      -- clka_en = 0
   *
   *   fi2s = fpll / N + b/a  -- N = clkm_div_num
   *   fi2s = 160MHz / 2
   *   fi2s = 80MHz
   *
   *   fbclk = fi2s / M   -- M = tx_bck_div_num
   *   fbclk = 80MHz / 2
   *   fbclk = 40MHz
   *
   *   fwclk = fbclk / 32
   *
   *   for fwclk = 250kHz (4µS pulse time)
   *      N = 10
   *      M = 20
   */

  // Allocate the array of pointers to the buffers//将指针数组分配给缓冲区
  dma.buffers = (uint32_t **)malloc(sizeof(uint32_t*) * DMA_BUF_COUNT);
  if (!dma.buffers) return -1;

  // Allocate each buffer that can be used by the DMA controller//分配DMA控制器可以使用的每个缓冲区
  for (int buf_idx = 0; buf_idx < DMA_BUF_COUNT; buf_idx++) {
    dma.buffers[buf_idx] = (uint32_t*) heap_caps_calloc(1, DMA_BUF_LEN, MALLOC_CAP_DMA);
    if (dma.buffers[buf_idx] == nullptr) return -1;
  }

  // Allocate the array of DMA descriptors//分配DMA描述符数组
  dma.desc = (lldesc_t**) malloc(sizeof(lldesc_t*) * DMA_BUF_COUNT);
  if (!dma.desc) return -1;

  // Allocate each DMA descriptor that will be used by the DMA controller//分配DMA控制器将使用的每个DMA描述符
  for (int buf_idx = 0; buf_idx < DMA_BUF_COUNT; buf_idx++) {
    dma.desc[buf_idx] = (lldesc_t*) heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma.desc[buf_idx] == nullptr) return -1;
  }

  // Initialize//初始化
  for (int buf_idx = 0; buf_idx < DMA_BUF_COUNT; buf_idx++) {
    dma.desc[buf_idx]->owner = 1;
    dma.desc[buf_idx]->eof = 1; // set to 1 will trigger the interrupt//设置为1将触发中断
    dma.desc[buf_idx]->sosf = 0;
    dma.desc[buf_idx]->length = DMA_BUF_LEN;
    dma.desc[buf_idx]->size = DMA_BUF_LEN;
    dma.desc[buf_idx]->buf = (uint8_t *) dma.buffers[buf_idx];
    dma.desc[buf_idx]->offset = 0;
    dma.desc[buf_idx]->empty = (uint32_t)((buf_idx < (DMA_BUF_COUNT - 1)) ? (dma.desc[buf_idx + 1]) : dma.desc[0]);
  }

  dma.queue = xQueueCreate(DMA_BUF_COUNT, sizeof(uint32_t *));

  // Set the first DMA descriptor//设置第一个DMA描述符
  I2S0.out_link.addr = (uint32_t)dma.desc[0];

  // stop i2s//停止i2s
  i2s_stop(I2S_NUM_0);

  // configure I2S data port interface.//配置I2S数据端口接口。
  i2s_reset_fifo(I2S_NUM_0);

  //reset i2s//重置i2s
  I2S0.conf.tx_reset = 1;
  I2S0.conf.tx_reset = 0;
  I2S0.conf.rx_reset = 1;
  I2S0.conf.rx_reset = 0;

  //reset dma//重置dma
  I2S0.lc_conf.in_rst = 1;
  I2S0.lc_conf.in_rst = 0;
  I2S0.lc_conf.out_rst = 1;
  I2S0.lc_conf.out_rst = 0;

  //Enable and configure DMA//启用和配置DMA
  I2S0.lc_conf.check_owner = 0;
  I2S0.lc_conf.out_loop_test = 0;
  I2S0.lc_conf.out_auto_wrback = 0;
  I2S0.lc_conf.out_data_burst_en = 0;
  I2S0.lc_conf.outdscr_burst_en = 0;
  I2S0.lc_conf.out_no_restart_clr = 0;
  I2S0.lc_conf.indscr_burst_en = 0;
  I2S0.lc_conf.out_eof_mode = 1;

  I2S0.conf2.lcd_en = 0;
  I2S0.conf2.camera_en = 0;

  #if CONFIG_IDF_TARGET_ESP32
    I2S0.pdm_conf.pcm2pdm_conv_en = 0;
    I2S0.pdm_conf.pdm2pcm_conv_en = 0;

    I2S0.pdm_conf.rx_pdm_en = 0;
    I2S0.pdm_conf.tx_pdm_en = 0;
  #endif

  I2S0.fifo_conf.dscr_en = 0;

  I2S0.conf_chan.tx_chan_mod = (
    #if ENABLED(I2S_STEPPER_SPLIT_STREAM)
      4
    #else
      0
    #endif
  );
  I2S0.fifo_conf.tx_fifo_mod = 0;
  I2S0.conf.tx_mono = 0;

  I2S0.conf_chan.rx_chan_mod = 0;
  I2S0.fifo_conf.rx_fifo_mod = 0;
  I2S0.conf.rx_mono = 0;

  I2S0.fifo_conf.dscr_en = 1; //connect dma to fifo//将dma连接到fifo

  I2S0.conf.tx_start = 0;
  I2S0.conf.rx_start = 0;

  I2S0.conf.tx_msb_right = 1;
  I2S0.conf.tx_right_first = 1;

  I2S0.conf.tx_slave_mod = 0; // Master//主人
  I2S0.fifo_conf.tx_fifo_mod_force_en = 1;

  I2S0.conf.tx_short_sync = 0;
  I2S0.conf.rx_short_sync = 0;
  I2S0.conf.tx_msb_shift = 0;
  I2S0.conf.rx_msb_shift = 0;

  // set clock//设置时钟
  #if CONFIG_IDF_TARGET_ESP32
    I2S0.clkm_conf.clka_en = 0;       // Use PLL/2 as reference//使用PLL/2作为参考
  #elif CONFIG_IDF_TARGET_ESP32S2
    I2S0.clkm_conf.clk_sel = 2;
  #endif
  I2S0.clkm_conf.clkm_div_num = 10; // minimum value of 2, reset value of 4, max 256//最小值为2，重置值为4，最大值为256
  I2S0.clkm_conf.clkm_div_a = 0;    // 0 at reset, what about divide by 0? (not an issue)//重置时为0，除以0怎么样？（不是问题）
  I2S0.clkm_conf.clkm_div_b = 0;    // 0 at reset//重置时为0

  // fbck = fi2s / tx_bck_div_num//fbck=fi2s/tx\u bck\u div\u num
  I2S0.sample_rate_conf.tx_bck_div_num = 2; // minimum value of 2 defaults to 6//最小值2默认为6

  // Enable TX interrupts//启用发送中断
  I2S0.int_ena.out_eof = 1;
  I2S0.int_ena.out_dscr_err = 0;
  I2S0.int_ena.out_total_eof = 0;
  I2S0.int_ena.out_done = 0;

  // Allocate and Enable the I2S interrupt//分配并启用I2S中断
  intr_handle_t i2s_isr_handle;
  esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, i2s_intr_handler_default, nullptr, &i2s_isr_handle);
  esp_intr_enable(i2s_isr_handle);

  // Create the task that will feed the buffer//创建将提供缓冲区的任务
  xTaskCreatePinnedToCore(stepperTask, "StepperTask", 10000, nullptr, 1, nullptr, CONFIG_ARDUINO_RUNNING_CORE); // run I2S stepper task on same core as rest of Marlin//在与Marlin的其余部分相同的核心上运行I2S步进器任务

  // Route the i2s pins to the appropriate GPIO//将i2s引脚布线至相应的GPIO
  gpio_matrix_out_check(I2S_DATA, I2S0O_DATA_OUT23_IDX, 0, 0);
  gpio_matrix_out_check(I2S_BCK, I2S0O_BCK_OUT_IDX, 0, 0);
  gpio_matrix_out_check(I2S_WS, I2S0O_WS_OUT_IDX, 0, 0);

  // Start the I2S peripheral//启动I2S外围设备
  return i2s_start(I2S_NUM_0);
}

void i2s_write(uint8_t pin, uint8_t val) {
  #if ENABLED(I2S_STEPPER_SPLIT_STREAM)
    if (pin >= 16) {
      SET_BIT_TO(I2S0.conf_single_data, pin, val);
      return;
    }
  #endif
  SET_BIT_TO(i2s_port_data, pin, val);
}

uint8_t i2s_state(uint8_t pin) {
  #if ENABLED(I2S_STEPPER_SPLIT_STREAM)
    if (pin >= 16) return TEST(I2S0.conf_single_data, pin);
  #endif
  return TEST(i2s_port_data, pin);
}

void i2s_push_sample() {
  dma.current[dma.rw_pos++] = i2s_port_data;
}

#endif // ARDUINO_ARCH_ESP32//ARDUINO_ARCH_ESP32
