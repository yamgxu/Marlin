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
#ifdef __SAMD51__

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Includes//包括
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

#include "../../inc/MarlinConfig.h"
#include "ServoTimers.h" // for SERVO_TC//用于伺服系统

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Local defines//局部定义
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 8

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Private Variables//私有变量
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

const tTimerConfig TimerConfig[NUM_HARDWARE_TIMERS+1] = {
  { {.pTc=TC0},  TC0_IRQn, TC_PRIORITY(0) },  // 0 - stepper (assigned priority 2)//0-步进器（分配的优先级为2）
  { {.pTc=TC1},  TC1_IRQn, TC_PRIORITY(1) },  // 1 - stepper (needed by 32 bit timers)//1-步进器（32位定时器需要）
  { {.pTc=TC2},  TC2_IRQn, 5              },  // 2 - tone (reserved by framework and fixed assigned priority 5)//2-音调（由框架保留，并固定分配优先级5）
  { {.pTc=TC3},  TC3_IRQn, TC_PRIORITY(3) },  // 3 - servo (assigned priority 1)//3-伺服（分配优先级1）
  { {.pTc=TC4},  TC4_IRQn, TC_PRIORITY(4) },  // 4 - software serial (no interrupts used)//4-软件串行（未使用中断）
  { {.pTc=TC5},  TC5_IRQn, TC_PRIORITY(5) },
  { {.pTc=TC6},  TC6_IRQn, TC_PRIORITY(6) },
  { {.pTc=TC7},  TC7_IRQn, TC_PRIORITY(7) },
  { {.pRtc=RTC}, RTC_IRQn, TC_PRIORITY(8) }   // 8 - temperature (assigned priority 6)//8-温度（分配的优先级为6）
};

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Private functions//私人职能
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

FORCE_INLINE void Disable_Irq(IRQn_Type irq) {
  NVIC_DisableIRQ(irq);

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();
}

// --------------------------------------------------------------------------// --------------------------------------------------------------------------
// Public functions//公共职能
// --------------------------------------------------------------------------// --------------------------------------------------------------------------

void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;

  // Disable interrupt, just in case it was already enabled//禁用中断，以防它已被启用
  Disable_Irq(irq);

  if (timer_num == RTC_TIMER_NUM) {
    Rtc * const rtc = TimerConfig[timer_num].pRtc;

    // Disable timer interrupt//禁用定时器中断
    rtc->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_CMP0;

    // RTC clock setup//时钟设置
    OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_XOSC32K;  // External 32.768KHz oscillator//外部32.768KHz振荡器

    // Stop timer, just in case, to be able to reconfigure it//停止计时器，以防万一，以便能够重新配置它
    rtc->MODE0.CTRLA.bit.ENABLE = false;
    SYNC(rtc->MODE0.SYNCBUSY.bit.ENABLE);

    // Mode, reset counter on match//模式，匹配时重置计数器
    rtc->MODE0.CTRLA.reg = RTC_MODE0_CTRLA_MODE_COUNT32 | RTC_MODE0_CTRLA_MATCHCLR;

    // Set compare value//设置比较值
    rtc->MODE0.COMP[0].reg = (32768 + frequency / 2) / frequency;
    SYNC(rtc->MODE0.SYNCBUSY.bit.COMP0);

    // Enable interrupt on compare//在比较时启用中断
    rtc->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;    // reset pending interrupt//重置挂起中断
    rtc->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;  // enable compare 0 interrupt//启用比较0中断

    // And start timer//和启动计时器
    rtc->MODE0.CTRLA.bit.ENABLE = true;
    SYNC(rtc->MODE0.SYNCBUSY.bit.ENABLE);
  }
  else {
    Tc * const tc = TimerConfig[timer_num].pTc;

    // Disable timer interrupt//禁用定时器中断
    tc->COUNT32.INTENCLR.reg = TC_INTENCLR_OVF; // disable overflow interrupt//禁用溢出中断

    // TCn clock setup//时钟设置
    const uint8_t clockID = GCLK_CLKCTRL_IDs[TCC_INST_NUM + timer_num];   // TC clock are preceeded by TCC ones//TC时钟由TCC时钟超前
    GCLK->PCHCTRL[clockID].bit.CHEN = false;
    SYNC(GCLK->PCHCTRL[clockID].bit.CHEN);
    GCLK->PCHCTRL[clockID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;   // 120MHz startup code programmed//120MHz启动代码已编程
    SYNC(!GCLK->PCHCTRL[clockID].bit.CHEN);

    // Stop timer, just in case, to be able to reconfigure it//停止计时器，以防万一，以便能够重新配置它
    tc->COUNT32.CTRLA.bit.ENABLE = false;
    SYNC(tc->COUNT32.SYNCBUSY.bit.ENABLE);

    // Reset timer//重置计时器
    tc->COUNT32.CTRLA.bit.SWRST = true;
    SYNC(tc->COUNT32.SYNCBUSY.bit.SWRST);

    // Wave mode, reset counter on compare match//波形模式，比较匹配时重置计数器
    tc->COUNT32.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
    tc->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_PRESCALER_DIV1;
    tc->COUNT32.CTRLBCLR.reg = TC_CTRLBCLR_DIR;
    SYNC(tc->COUNT32.SYNCBUSY.bit.CTRLB);

    // Set compare value//设置比较值
    tc->COUNT32.CC[0].reg = (HAL_TIMER_RATE) / frequency;
    tc->COUNT32.COUNT.reg = 0;

    // Enable interrupt on compare//在比较时启用中断
    tc->COUNT32.INTFLAG.reg = TC_INTFLAG_OVF;   // reset pending interrupt//重置挂起中断
    tc->COUNT32.INTENSET.reg = TC_INTENSET_OVF; // enable overflow interrupt//启用溢出中断

    // And start timer//和启动计时器
    tc->COUNT32.CTRLA.bit.ENABLE = true;
    SYNC(tc->COUNT32.SYNCBUSY.bit.ENABLE);
  }

  // Finally, enable IRQ//最后，启用IRQ
  NVIC_SetPriority(irq, TimerConfig[timer_num].priority);
  NVIC_EnableIRQ(irq);
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  NVIC_EnableIRQ(irq);
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  Disable_Irq(irq);
}

// missing from CMSIS: Check if interrupt is enabled or not//CMSIS缺失：检查中断是否启用
static bool NVIC_GetEnabledIRQ(IRQn_Type IRQn) {
  return TEST(NVIC->ISER[uint32_t(IRQn) >> 5], uint32_t(IRQn) & 0x1F);
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  const IRQn_Type irq = TimerConfig[timer_num].IRQ_Id;
  return NVIC_GetEnabledIRQ(irq);
}

#endif // __SAMD51__//_uusamd51__
