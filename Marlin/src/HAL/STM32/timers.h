/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2017 Victor Perez
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

#include "../../inc/MarlinConfig.h"

// ------------------------// ------------------------
// Defines//定义
// ------------------------// ------------------------

// STM32 timers may be 16 or 32 bit. Limiting HAL_TIMER_TYPE_MAX to 16 bits//STM32定时器可以是16位或32位。限制HAL_定时器类型_最大为16位
// avoids issues with STM32F0 MCUs, which seem to pause timers if UINT32_MAX//避免STM32F0 MCU出现问题，如果UINT32_最大，则可能会暂停计时器
// is written to the register. STM32F4 timers do not manifest this issue,//已写入登记簿。STM32F4计时器未显示此问题，
// even when writing to 16 bit timers.//即使在写入16位计时器时也是如此。
////
// The range of the timer can be queried at runtime using IS_TIM_32B_COUNTER_INSTANCE.//可以在运行时使用IS_TIM_32B_COUNTER_实例查询计时器的范围。
// This is a more expensive check than a simple compile-time constant, so its//这是一个比简单编译时常量更昂贵的检查，因此
// implementation is deferred until the desire for a 32-bit range outweighs the cost//直到对32位范围的需求超过了成本，实现才会推迟
// of adding a run-time check and HAL_TIMER_TYPE_MAX is refactored to allow unique//对添加运行时检查和HAL_TIMER_TYPE_MAX的方法进行了重构，以允许
// values for each timer.//每个计时器的值。
#define hal_timer_t uint32_t
#define HAL_TIMER_TYPE_MAX UINT16_MAX

#define NUM_HARDWARE_TIMERS 2

#ifndef STEP_TIMER_NUM
  #define STEP_TIMER_NUM        0  // Timer Index for Stepper//步进电机的定时器索引
#endif
#ifndef PULSE_TIMER_NUM
  #define PULSE_TIMER_NUM       STEP_TIMER_NUM
#endif
#ifndef TEMP_TIMER_NUM
  #define TEMP_TIMER_NUM        1  // Timer Index for Temperature//温度计时器索引
#endif

#define TEMP_TIMER_FREQUENCY 1000   // Temperature::isr() is expected to be called at around 1kHz//温度：：isr（）预计在1kHz左右被调用

// TODO: get rid of manual rate/prescale/ticks/cycles taken for procedures in stepper.cpp//TODO:在stepper.cpp中去掉手动速率/预刻度/刻度/周期
#define STEPPER_TIMER_RATE 2000000 // 2 Mhz//2兆赫
extern uint32_t GetStepperTimerClkFreq();
#define STEPPER_TIMER_PRESCALE (GetStepperTimerClkFreq() / (STEPPER_TIMER_RATE))
#define STEPPER_TIMER_TICKS_PER_US ((STEPPER_TIMER_RATE) / 1000000) // stepper timer ticks per µs//步进定时器滴答声每微秒

#define PULSE_TIMER_RATE STEPPER_TIMER_RATE
#define PULSE_TIMER_PRESCALE STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US STEPPER_TIMER_TICKS_PER_US

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt(STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_disable_interrupt(STEP_TIMER_NUM)
#define STEPPER_ISR_ENABLED() HAL_timer_interrupt_enabled(STEP_TIMER_NUM)

#define ENABLE_TEMPERATURE_INTERRUPT() HAL_timer_enable_interrupt(TEMP_TIMER_NUM)
#define DISABLE_TEMPERATURE_INTERRUPT() HAL_timer_disable_interrupt(TEMP_TIMER_NUM)

extern void Step_Handler();
extern void Temp_Handler();

#ifndef HAL_STEP_TIMER_ISR
  #define HAL_STEP_TIMER_ISR() void Step_Handler()
#endif
#ifndef HAL_TEMP_TIMER_ISR
  #define HAL_TEMP_TIMER_ISR() void Temp_Handler()
#endif

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

extern HardwareTimer *timer_instance[];

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency);
void HAL_timer_enable_interrupt(const uint8_t timer_num);
void HAL_timer_disable_interrupt(const uint8_t timer_num);
bool HAL_timer_interrupt_enabled(const uint8_t timer_num);

// Configure timer priorities for peripherals such as Software Serial or Servos.//为软件串行或伺服等外围设备配置计时器优先级。
// Exposed here to allow all timer priority information to reside in timers.cpp//在此处公开以允许所有计时器优先级信息驻留在timers.cpp中
void SetTimerInterruptPriorities();

// FORCE_INLINE because these are used in performance-critical situations//强制_内联，因为它们用于性能关键的情况
FORCE_INLINE bool HAL_timer_initialized(const uint8_t timer_num) {
  return timer_instance[timer_num] != nullptr;
}
FORCE_INLINE static hal_timer_t HAL_timer_get_count(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) ? timer_instance[timer_num]->getCount() : 0;
}

// NOTE: Method name may be misleading.//注意：方法名称可能有误导性。
// STM32 has an Auto-Reload Register (ARR) as opposed to a "compare" register//STM32具有自动重新加载寄存器（ARR），而不是“比较”寄存器
FORCE_INLINE static void HAL_timer_set_compare(const uint8_t timer_num, const hal_timer_t overflow) {
  if (HAL_timer_initialized(timer_num)) {
    timer_instance[timer_num]->setOverflow(overflow + 1, TICK_FORMAT); // Value decremented by setOverflow()//由setOverflow（）递减的值
    // wiki: "force all registers (Autoreload, prescaler, compare) to be taken into account"//wiki：“强制考虑所有寄存器（自动加载、预分频器、比较）”
    // So, if the new overflow value is less than the count it will trigger a rollover interrupt.//因此，如果新的溢出值小于计数，它将触发翻转中断。
    if (overflow < timer_instance[timer_num]->getCount())  // Added 'if' here because reports say it won't boot without it//在这里添加了“if”，因为有报道说没有它它就无法启动
      timer_instance[timer_num]->refresh();
  }
}

#define HAL_timer_isr_prologue(TIMER_NUM)
#define HAL_timer_isr_epilogue(TIMER_NUM)
