/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
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

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Defines//定义
// --------------------------------------------------------------------------// --------------------------------------------------------------------------
#define RTC_TIMER_NUM       8   // This is not a TC but a RTC//这不是TC，而是RTC

typedef uint32_t hal_timer_t;
#define HAL_TIMER_TYPE_MAX 0xFFFFFFFF

#define HAL_TIMER_RATE      F_CPU   // frequency of timers peripherals//定时器和外围设备的频率

#ifndef STEP_TIMER_NUM
  #define STEP_TIMER_NUM        0  // Timer Index for Stepper//步进电机的定时器索引
#endif
#ifndef PULSE_TIMER_NUM
  #define PULSE_TIMER_NUM       STEP_TIMER_NUM
#endif
#ifndef TEMP_TIMER_NUM
  #define TEMP_TIMER_NUM        RTC_TIMER_NUM // Timer Index for Temperature//温度计时器索引
#endif

#define TEMP_TIMER_FREQUENCY   1000 // temperature interrupt frequency//温度中断频率

#define STEPPER_TIMER_RATE          HAL_TIMER_RATE   // frequency of stepper timer (HAL_TIMER_RATE / STEPPER_TIMER_PRESCALE)//步进定时器的频率（HAL\u定时器\u速率/步进定时器\u预刻度）
#define STEPPER_TIMER_TICKS_PER_US  (STEPPER_TIMER_RATE / 1000000) // stepper timer ticks per µs//步进定时器滴答声每微秒
#define STEPPER_TIMER_PRESCALE      (CYCLES_PER_MICROSECOND / STEPPER_TIMER_TICKS_PER_US)

#define PULSE_TIMER_RATE          STEPPER_TIMER_RATE
#define PULSE_TIMER_PRESCALE      STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US  STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT()   HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED()               HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT()  HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

#define TC_PRIORITY(t)        t == SERVO_TC ? 1                                     \
                               : (t == STEP_TIMER_NUM || t == PULSE_TIMER_NUM) ? 2  \
                               : (t == TEMP_TIMER_NUM) ? 6                          \
                               : 7

#define _TC_HANDLER(t)          void TC##t##_Handler()
#define TC_HANDLER(t)           _TC_HANDLER(t)
#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR()  TC_HANDLER(STEP_TIMER_NUM)
#endif
#if STEP_TIMER_NUM != PULSE_TIMER_NUM
  #define HAL_PULSE_TIMER_ISR() TC_HANDLER(PULSE_TIMER_NUM)
#endif
#if TEMP_TIMER_NUM == RTC_TIMER_NUM
  #define HAL_TEMP_TIMER_ISR()  void RTC_Handler()
#else
  #define HAL_TEMP_TIMER_ISR()  TC_HANDLER(TEMP_TIMER_NUM)
#endif

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Types//类型
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

typedef struct {
  union {
    Tc  *pTc;
    Rtc *pRtc;
  };
  IRQn_Type   IRQ_Id;
  uint8_t     priority;
} tTimerConfig;

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Public Variables//公共变量
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

extern const tTimerConfig TimerConfig[];

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Public functions//公共职能
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);

FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t compare) {
  // Should never be called with timer RTC_TIMER_NUM//不应使用计时器RTC\U timer\U NUM调用
  Tc * const tc = TimerConfig[timer_num].pTc;
  tc->COUNT32.CC[0].reg = compare;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_compare(const uint8_t timer_num) {
  // Should never be called with timer RTC_TIMER_NUM//不应使用计时器RTC\U timer\U NUM调用
  Tc * const tc = TimerConfig[timer_num].pTc;
  return (hal_timer_t)tc->COUNT32.CC[0].reg;
}

FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  // Should never be called with timer RTC_TIMER_NUM//不应使用计时器RTC\U timer\U NUM调用
  Tc * const tc = TimerConfig[timer_num].pTc;
  tc->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_READSYNC;
  SYNC(tc->COUNT32.SYNCBUSY.bit.CTRLB || tc->COUNT32.SYNCBUSY.bit.COUNT);
  return tc->COUNT32.COUNT.reg;
}

void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

FORCE_INLINE static void HAL_timer_isr_prologue(const uint8_t timer_num) {
  if (timer_num == RTC_TIMER_NUM) {
    Rtc * const rtc = TimerConfig[timer_num].pRtc;
    // Clear interrupt flag//清除中断标志
    rtc->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;
  }
  else {
    Tc * const tc = TimerConfig[timer_num].pTc;
    // Clear interrupt flag//清除中断标志
    tc->COUNT32.INTFLAG.reg = TC_INTFLAG_OVF;
  }
}

#define HAL_timer_isr_epilogue(timer_num)
