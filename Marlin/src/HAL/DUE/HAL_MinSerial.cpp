/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(POSTMORTEM_DEBUGGING)

#include "../shared/HAL_MinSerial.h"

#include <stdarg.h>

static void TXBegin() {
  // Disable UART interrupt in NVIC//在NVIC中禁用UART中断
  NVIC_DisableIRQ( UART_IRQn );

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();

  // Disable clock//禁用时钟
  pmc_disable_periph_clk( ID_UART );

  // Configure PMC//配置PMC
  pmc_enable_periph_clk( ID_UART );

  // Disable PDC channel//禁用PDC通道
  UART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  // Reset and disable receiver and transmitter//重置和禁用接收器和发射器
  UART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

  // Configure mode: 8bit, No parity, 1 bit stop//配置模式：8位，无奇偶校验，1位停止
  UART->UART_MR = UART_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO;

  // Configure baudrate (asynchronous, no oversampling) to BAUDRATE bauds//将波特率（异步，无过采样）配置为波特率
  UART->UART_BRGR = (SystemCoreClock / (BAUDRATE << 4));

  // Enable receiver and transmitter//启用接收器和发射器
  UART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}

// A SW memory barrier, to ensure GCC does not overoptimize loops//软件内存屏障，确保GCC不会过度优化循环
#define sw_barrier() __asm__ volatile("": : :"memory");
static void TX(char c) {
  while (!(UART->UART_SR & UART_SR_TXRDY)) { WDT_Restart(WDT); sw_barrier(); };
  UART->UART_THR = c;
}

void install_min_serial() {
  HAL_min_serial_init = &TXBegin;
  HAL_min_serial_out = &TX;
}

#if DISABLED(DYNAMIC_VECTORTABLE)
extern "C" {
  __attribute__((naked)) void JumpHandler_ASM() {
    __asm__ __volatile__ (
      "b CommonHandler_ASM\n"
    );
  }
  void __attribute__((naked, alias("JumpHandler_ASM"))) HardFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) BusFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) UsageFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) MemManage_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"))) NMI_Handler();
}
#endif

#endif // POSTMORTEM_DEBUGGING//死后调试
#endif // ARDUINO_ARCH_SAM//阿杜伊诺·阿丘·萨姆
