/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
#ifdef __STM32F1__

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(POSTMORTEM_DEBUGGING)

#include "../shared/HAL_MinSerial.h"
#include "watchdog.h"

#include <libmaple/usart.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>

/* Instruction Synchronization Barrier */
#define isb() __asm__ __volatile__ ("isb" : : : "memory")

/* Data Synchronization Barrier */
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")

static void TXBegin() {
  #if !WITHIN(SERIAL_PORT, 1, 6)
    #warning "Using POSTMORTEM_DEBUGGING requires a physical U(S)ART hardware in case of severe error."
    #warning "Disabling the severe error reporting feature currently because the used serial port is not a HW port."
  #else
    // We use MYSERIAL1 here, so we need to figure out how to get the linked register//我们在这里使用MYSERIAL1，所以我们需要弄清楚如何获取链接寄存器
    struct usart_dev* dev = MYSERIAL1.c_dev();

    // Or use this if removing libmaple//或者在删除libmaple时使用此选项
    // int irq = dev->irq_num;//intirq=dev->irq\u num；
    // int nvicUART[] = { NVIC_USART1 /* = 37 */, NVIC_USART2 /* = 38 */, NVIC_USART3 /* = 39 */, NVIC_UART4 /* = 52 */, NVIC_UART5 /* = 53 */ };//int nvicUART[]={NVIC_USART1/*=37*/，NVIC_USART2/*=38*/，NVIC_USART3/*=39*/，NVIC_UART4/*=52*/，NVIC_UART5/*=53*/}；
    // Disabling irq means setting the bit in the NVIC ICER register located at//禁用irq意味着设置位于以下位置的NVIC ICER寄存器中的位：
    // Disable UART interrupt in NVIC//在NVIC中禁用UART中断
    nvic_irq_disable(dev->irq_num);

    // Use this if removing libmaple//如果要删除libmaple，请使用此选项
    //SBI(NVIC_BASE->ICER[1], irq - 32);//SBI（NVIC_基地->ICER[1]，irq-32）；

    // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    dsb();
    isb();

    rcc_clk_disable(dev->clk_id);
    rcc_clk_enable(dev->clk_id);

    usart_reg_map *regs = dev->regs;
    regs->CR1 = 0; // Reset the USART//重置USART
    regs->CR2 = 0; // 1 stop bit//1停止位

    // If we don't touch the BRR (baudrate register), we don't need to recompute. Else we would need to call//如果我们不接触BRR（波特率寄存器），我们就不需要重新计算。否则我们需要打电话
    usart_set_baud_rate(dev, 0, BAUDRATE);

    regs->CR1 = (USART_CR1_TE | USART_CR1_UE); // 8 bits, no parity, 1 stop bit//8位，无奇偶校验，1个停止位
  #endif
}

// A SW memory barrier, to ensure GCC does not overoptimize loops//软件内存屏障，确保GCC不会过度优化循环
#define sw_barrier() __asm__ volatile("": : :"memory");
static void TX(char c) {
  #if WITHIN(SERIAL_PORT, 1, 6)
    struct usart_dev* dev = MYSERIAL1.c_dev();
    while (!(dev->regs->SR & USART_SR_TXE)) {
      TERN_(USE_WATCHDOG, HAL_watchdog_refresh());
      sw_barrier();
    }
    dev->regs->DR = c;
  #endif
}

void install_min_serial() {
  HAL_min_serial_init = &TXBegin;
  HAL_min_serial_out = &TX;
}

#if DISABLED(DYNAMIC_VECTORTABLE) && DISABLED(STM32F0xx) // Cortex M0 can't branch to a symbol that's too far, so we have a specific hack for them//Cortex M0无法分支到太远的符号，因此我们对它们有一个特定的攻击
extern "C" {
  __attribute__((naked)) void JumpHandler_ASM() {
    __asm__ __volatile__ (
      "b CommonHandler_ASM\n"
    );
  }
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __exc_hardfault();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __exc_busfault();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __exc_usagefault();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __exc_memmanage();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __exc_nmi();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __stm32reservedexception7();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __stm32reservedexception8();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __stm32reservedexception9();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __stm32reservedexception10();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) __stm32reservedexception13();
}
#endif

#endif // POSTMORTEM_DEBUGGING//死后调试
#endif // __STM32F1__//_uustm32f1__
