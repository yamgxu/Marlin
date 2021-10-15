/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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

/**
 * HAL Timers for Arduino Due and compatible (SAM3X8E)
 */

#include <stdint.h>

// ------------------------// ------------------------
// Defines//定义
// ------------------------// ------------------------

#define FORCE_INLINE __attribute__((always_inline)) inline

typedef uint32_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF

#define HAL_TIMER_RATE         ((F_CPU) / 2)    // frequency of timers peripherals//定时器和外围设备的频率

#ifndef STEP_TIMER_NUM
  #define STEP_TIMER_NUM        2  // Timer Index for Stepper//步进电机的定时器索引
#endif
#ifndef PULSE_TIMER_NUM
  #define PULSE_TIMER_NUM       STEP_TIMER_NUM
#endif
#ifndef TEMP_TIMER_NUM
  #define TEMP_TIMER_NUM        4  // Timer Index for Temperature//温度计时器索引
#endif
#ifndef TONE_TIMER_NUM
  #define TONE_TIMER_NUM        6  // index of timer to use for beeper tones//用于蜂鸣器铃声的计时器索引
#endif

#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency//温度中断频率

#define STEPPER_TIMER_RATE     HAL_TIMER_RATE   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)//步进定时器的频率（HAL\u定时器\u速率/步进定时器\u预刻度）
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs//步进定时器滴答声每微秒
#define STEPPER_TIMER_PRESCALE (CYCLES_PER_MICROSECOND / STEPPER_TIMER_TICKS_PER_US)

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer//脉冲定时器频率
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() void TC2_Handler()
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() void TC4_Handler()
#endif
#ifndef HAL_TONE_TIMER_ISR
  #define HAL_TONE_TIMER_ISR() void TC6_Handler()
#endif

// ------------------------// ------------------------
// Types//类型
// ------------------------// ------------------------

typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
  uint8_t     priority;
} tTimerConfig;

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

extern const tTimerConfig TimerConfig[];

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC = compare;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_CV;
}

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  const tTimerConfig * const pConfig = &TimerConfig[timer_num];
  // Reading the status register clears the interrupt flag//读取状态寄存器将清除中断标志
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_SR;
}

#define HAL_timer_isr_epilogue(TIMER_NUM)
