/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

/**
 * HAL Timers for Arduino Due and compatible (SAM3X8E)
 */

#ifdef ARDUINO_ARCH_SAM

// ------------------------// ------------------------
// Includes//包括
// ------------------------// ------------------------
#include "../../inc/MarlinConfig.h"
#include "HAL.h"

// ------------------------// ------------------------
// Local defines//局部定义
// ------------------------// ------------------------

#define NUM_HARDWARE_TIMERS 9

// ------------------------// ------------------------
// Private Variables//私有变量
// ------------------------// ------------------------

const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] = {
  { TC0, 0, TC0_IRQn,  3}, // 0 - [servo timer5]//0-[伺服时间R5]
  { TC0, 1, TC1_IRQn,  0}, // 1// 1
  { TC0, 2, TC2_IRQn,  2}, // 2 - stepper//2步进电机
  { TC1, 0, TC3_IRQn,  0}, // 3 - stepper for BOARD_ARCHIM1//3-电路板ARCHIM1的步进器
  { TC1, 1, TC4_IRQn, 15}, // 4 - temperature//4-温度
  { TC1, 2, TC5_IRQn,  3}, // 5 - [servo timer3]//5-[伺服定时器3]
  { TC2, 0, TC6_IRQn, 14}, // 6 - tone//六音
  { TC2, 1, TC7_IRQn,  0}, // 7// 7
  { TC2, 2, TC8_IRQn,  0}, // 8// 8
};

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  Tc *tc = TimerConfig[timer_num].pTimerRegs;
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  uint32_t channel = TimerConfig[timer_num].channel;

  // Disable interrupt, just in case it was already enabled//禁用中断，以防它已被启用
  NVIC_DisableIRQ(irq);

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();

  // Disable timer interrupt//禁用定时器中断
  tc->TC_CHANNEL[channel].TC_IDR = TC_IDR_CPCS;

  // Stop timer, just in case, to be able to reconfigure it//停止计时器，以防万一，以便能够重新配置它
  TC_Stop(tc, channel);

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority(irq, TimerConfig [timer_num].priority);

  // wave mode, reset counter on match with RC,//波形模式，与RC匹配时重置计数器，
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);

  // Set compare value//设置比较值
  TC_SetRC(tc, channel, VARIANT_MCK / 2 / frequency);

  // And start timer//和启动计时器
  TC_Start(tc, channel);

  // enable interrupt on RC compare//启用RC比较上的中断
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;

  // Finally, enable IRQ//最后，启用IRQ
  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  NVIC_EnableIRQ(irq);
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  NVIC_DisableIRQ(irq);

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

// missing from CMSIS: Check if interrupt is enabled or not//CMSIS缺失：检查中断是否启用
static bool NVIC_GetEnabledIRQ(IRQn_Type IRQn) {
  return TEST(NVIC->ISER[uint32_t(IRQn) >> 5], uint32_t(IRQn) & 0x1F);
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  return NVIC_GetEnabledIRQ(irq);
}

#endif // ARDUINO_ARCH_SAM//阿杜伊诺·阿丘·萨姆
