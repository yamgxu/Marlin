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
#pragma once

#include <stdint.h>
#include <driver/timer.h>

// ------------------------// ------------------------
// Defines//定义
// ------------------------// ------------------------
#define FORCE_INLINE __attribute__((always_inline)) inline

typedef uint64_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFFULL

#ifndef STEP_TIMER_NUM
  #define STEP_TIMER_NUM        0  // Timer Index for Stepper//步进电机的定时器索引
#endif
#ifndef PULSE_TIMER_NUM
  #define PULSE_TIMER_NUM       STEP_TIMER_NUM
#endif
#ifndef TEMP_TIMER_NUM
  #define TEMP_TIMER_NUM        1  // Timer Index for Temperature//温度计时器索引
#endif
#ifndef PWM_TIMER_NUM
  #define PWM_TIMER_NUM         2  // index of timer to use for PWM outputs//用于PWM输出的计时器索引
#endif
#ifndef TONE_TIMER_NUM
  #define TONE_TIMER_NUM        3  // index of timer for beeper tones//蜂鸣器铃声计时器索引
#endif

#define HAL_TIMER_RATE APB_CLK_FREQ // frequency of timer peripherals//定时器外围设备的频率

#if ENABLED(I2S_STEPPER_STREAM)
  #define STEPPER_TIMER_PRESCALE     1
  #define STEPPER_TIMER_RATE         250000                           // 250khz, 4µs pulses of i2s word clock//250khz，4µs i2s字时钟脉冲
  #define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs // wrong would be 0.25//步进定时器每微秒的滴答声//错误为0.25
#else
  #define STEPPER_TIMER_PRESCALE     40
  #define STEPPER_TIMER_RATE         ((HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE)) // frequency of stepper timer, 2MHz//步进定时器的频率，2MHz
  #define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000)              // stepper timer ticks per µs//步进定时器滴答声每微秒
#endif

#define STEP_TIMER_MIN_INTERVAL   8 // minimum time in µs between stepper interrupts//步进机中断之间的最短时间（µs）

#define TONE_TIMER_PRESCALE    1000 // Arbitrary value, no idea what i'm doing here//任意值，不知道我在这里做什么

#define TEMP_TIMER_PRESCALE    1000 // prescaler for setting Temp timer, 72Khz//用于设置温度定时器的预分频器，72Khz
#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency//温度中断频率

#define PWM_TIMER_PRESCALE       10
#if ENABLED(FAST_PWM_FAN)
  #define PWM_TIMER_FREQUENCY  FAST_PWM_FAN_FREQUENCY
#else
  #define PWM_TIMER_FREQUENCY  (50*128) // 50Hz and 7bit resolution//50Hz和7bit分辨率
#endif
#define MAX_PWM_PINS             32 // Number of PWM pin-slots//PWM引脚插槽的数量

#define PULSE_TIMER_RATE         STEPPER_TIMER_RATE   // frequency of pulse timer//脉冲定时器频率
#define PULSE_TIMER_PRESCALE     STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() extern "C" void tempTC_Handler()
#endif
#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() extern "C" void stepTC_Handler()
#endif
#ifndef HAL_PWM_TIMER_ISR
  #define HAL_PWM_TIMER_ISR() extern "C" void pwmTC_Handler()
#endif
#ifndef HAL_TONE_TIMER_ISR
  #define HAL_TONE_TIMER_ISR() extern "C" void toneTC_Handler()
#endif

extern "C" {
  void tempTC_Handler();
  void stepTC_Handler();
  void pwmTC_Handler();
  void toneTC_Handler();
}

// ------------------------// ------------------------
// Types//类型
// ------------------------// ------------------------

typedef struct {
  timer_group_t  group;
  timer_idx_t    idx;
  uint32_t       divider;
  void           (*fn)();
} tTimerConfig;

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

extern const tTimerConfig TimerConfig[];

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

void HAL_timer_start (const uint8_t timer_num, uint32_t frequency);
void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t count);
hal_timer_t HAL_timer_get_compare(const uint8_t timer_num);
hal_timer_t HAL_timer_get_count(const uint8_t timer_num);

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

#define HAL_timer_isr_prologue(TIMER_NUM)
#define HAL_timer_isr_epilogue(TIMER_NUM)
