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

/**
 * HAL Timers for Teensy 4.0 (IMXRT1062DVL6A) / 4.1 (IMXRT1062DVJ6A)
 */

#ifdef __IMXRT1062__

#include "../../inc/MarlinConfig.h"

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  switch (timer_num) {
    case 0:
      CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL; // turn off 24mhz mode//关闭24mhz模式
      CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);

      GPT1_CR = 0;                   // disable timer//禁用计时器
      GPT1_SR = 0x3F;                // clear all prior status//清除所有先前状态
      GPT1_PR = GPT1_TIMER_PRESCALE - 1;
      GPT1_CR |= GPT_CR_CLKSRC(1);   //clock selection #1 (peripheral clock = 150 MHz)//时钟选择#1（外围时钟=150 MHz）
      GPT1_CR |= GPT_CR_ENMOD;       //reset count to zero before enabling//启用前将计数重置为零
      GPT1_CR |= GPT_CR_OM1(1);      // toggle mode//切换模式
      GPT1_OCR1 = (GPT1_TIMER_RATE / frequency) -1; // Initial compare value//初始比较值
      GPT1_IR = GPT_IR_OF1IE;        // Compare3 value//比较3值
      GPT1_CR |= GPT_CR_EN;          //enable GPT2 counting at 150 MHz//在150 MHz下启用GPT2计数

      OUT_WRITE(15, HIGH);
      attachInterruptVector(IRQ_GPT1, &stepTC_Handler);
      NVIC_SET_PRIORITY(IRQ_GPT1, 16);
      break;
    case 1:
      CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL; // turn off 24mhz mode//关闭24mhz模式
      CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

      GPT2_CR = 0;                   // disable timer//禁用计时器
      GPT2_SR = 0x3F;                // clear all prior status//清除所有先前状态
      GPT2_PR = GPT2_TIMER_PRESCALE - 1;
      GPT2_CR |= GPT_CR_CLKSRC(1);   //clock selection #1 (peripheral clock = 150 MHz)//时钟选择#1（外围时钟=150 MHz）
      GPT2_CR |= GPT_CR_ENMOD;       //reset count to zero before enabling//启用前将计数重置为零
      GPT2_CR |= GPT_CR_OM1(1);      // toggle mode//切换模式
      GPT2_OCR1 = (GPT2_TIMER_RATE / frequency) -1; // Initial compare value//初始比较值
      GPT2_IR = GPT_IR_OF1IE;        // Compare3 value//比较3值
      GPT2_CR |= GPT_CR_EN;          //enable GPT2 counting at 150 MHz//在150 MHz下启用GPT2计数

      OUT_WRITE(14, HIGH);
      attachInterruptVector(IRQ_GPT2, &tempTC_Handler);
      NVIC_SET_PRIORITY(IRQ_GPT2, 32);
      break;
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0:
      NVIC_ENABLE_IRQ(IRQ_GPT1);
      break;
    case 1:
      NVIC_ENABLE_IRQ(IRQ_GPT2);
      break;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: NVIC_DISABLE_IRQ(IRQ_GPT1); break;
    case 1: NVIC_DISABLE_IRQ(IRQ_GPT2); break;
  }

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  asm volatile("dsb");
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  switch (timer_num) {
    case 0: return (NVIC_IS_ENABLED(IRQ_GPT1));
    case 1: return (NVIC_IS_ENABLED(IRQ_GPT2));
  }
  return false;
}

void HAL_timer_isr_prologue(const uint8_t timer_num) {
  switch (timer_num) {
    case 0:
      GPT1_SR = GPT_IR_OF1IE;  // clear OF3 bit//清除3位
      break;
    case 1:
      GPT2_SR = GPT_IR_OF1IE;  // clear OF3 bit//清除3位
      break;
  }
  asm volatile("dsb");
}

#endif // __IMXRT1062__//_uuimxrt1062__
