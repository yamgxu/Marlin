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
 * HAL For LPC1768
 */

#include <stdint.h>

#include "../../core/macros.h"

#define SBIT_TIMER0 1
#define SBIT_TIMER1 2

#define SBIT_CNTEN 0

#define SBIT_MR0I  0 // Timer 0 Interrupt when TC matches MR0//TC与MR0匹配时定时器0中断
#define SBIT_MR0R  1 // Timer 0 Reset TC on Match//计时器0在匹配时重置TC
#define SBIT_MR0S  2 // Timer 0 Stop TC and PC on Match//计时器0停止TC和PC的匹配
#define SBIT_MR1I  3
#define SBIT_MR1R  4
#define SBIT_MR1S  5
#define SBIT_MR2I  6
#define SBIT_MR2R  7
#define SBIT_MR2S  8
#define SBIT_MR3I  9
#define SBIT_MR3R 10
#define SBIT_MR3S 11

// ------------------------// ------------------------
// Defines//定义
// ------------------------// ------------------------

#define _HAL_TIMER(T) _CAT(LPC_TIM, T)
#define _HAL_TIMER_IRQ(T) TIMER##T##_IRQn
#define __HAL_TIMER_ISR(T) extern "C" void TIMER##T##_IRQHandler()
#define _HAL_TIMER_ISR(T)  __HAL_TIMER_ISR(T)

typedef uint32_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF

#define HAL_TIMER_RATE         ((F_CPU) / 4)  // frequency of timers peripherals//定时器和外围设备的频率

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
  #define PWM_TIMER_NUM         3  // Timer Index for PWM//PWM定时器指标
#endif

#define TEMP_TIMER_RATE        1000000
#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency//温度中断频率

#define STEPPER_TIMER_RATE     HAL_TIMER_RATE   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)//步进定时器的频率（HAL\u定时器\u速率/步进定时器\u预刻度）
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs//步进定时器滴答声每微秒
#define STEPPER_TIMER_PRESCALE (CYCLES_PER_MICROSECOND / STEPPER_TIMER_TICKS_PER_US)

#define PULSE_TIMER_RATE       STEPPER_TIMER_RATE   // frequency of pulse timer//脉冲定时器频率
#define PULSE_TIMER_PRESCALE   STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() _HAL_TIMER_ISR(STEP_TIMER_NUM)
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() _HAL_TIMER_ISR(TEMP_TIMER_NUM)
#endif

// Timer references by index//按索引的计时器引用
#define STEP_TIMER_PTR _HAL_TIMER(STEP_TIMER_NUM)
#define TEMP_TIMER_PTR _HAL_TIMER(TEMP_TIMER_NUM)

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------
void HAL_timer_init();
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  switch (timer_num) {
    case 0: STEP_TIMER_PTR->MR0 = compare; break; // Stepper Timer Match Register 0//步进定时器匹配寄存器0
    case 1: TEMP_TIMER_PTR->MR0 = compare; break; //    Temp Timer Match Register 0//临时计时器匹配寄存器0
  }
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return STEP_TIMER_PTR->MR0; // Stepper Timer Match Register 0//步进定时器匹配寄存器0
    case 1: return TEMP_TIMER_PTR->MR0; //    Temp Timer Match Register 0//临时计时器匹配寄存器0
  }
  return 0;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return STEP_TIMER_PTR->TC; // Stepper Timer Count//步进定时器计数
    case 1: return TEMP_TIMER_PTR->TC; //    Temp Timer Count//临时计时器计数
  }
  return 0;
}

FORCE_INLINE static void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: NVIC_EnableIRQ(TIMER0_IRQn); break; // Enable interrupt handler//启用中断处理程序
    case 1: NVIC_EnableIRQ(TIMER1_IRQn); break; // Enable interrupt handler//启用中断处理程序
  }
}

FORCE_INLINE static void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: NVIC_DisableIRQ(TIMER0_IRQn); break; // Disable interrupt handler//禁用中断处理程序
    case 1: NVIC_DisableIRQ(TIMER1_IRQn); break; // Disable interrupt handler//禁用中断处理程序
  }

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

// This function is missing from CMSIS//CMSIS中缺少此功能
FORCE_INLINE static bool NVIC_GetEnableIRQ(IRQn_Type IRQn) {
  return TEST(NVIC->ISER[uint32_t(IRQn) >> 5], uint32_t(IRQn) & 0x1F);
}

FORCE_INLINE static bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return NVIC_GetEnableIRQ(TIMER0_IRQn); // Check if interrupt is enabled or not//检查中断是否启用
    case 1: return NVIC_GetEnableIRQ(TIMER1_IRQn); // Check if interrupt is enabled or not//检查中断是否启用
  }
  return false;
}

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: SBI(STEP_TIMER_PTR->IR, SBIT_CNTEN); break;
    case 1: SBI(TEMP_TIMER_PTR->IR, SBIT_CNTEN); break;
  }
}

#define HAL_timer_isr_epilogue(TIMER_NUM)
