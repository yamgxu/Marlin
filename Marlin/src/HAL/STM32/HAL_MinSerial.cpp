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
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(POSTMORTEM_DEBUGGING)

#include "../shared/HAL_MinSerial.h"
#include "watchdog.h"

/* Instruction Synchronization Barrier */
#define isb() __asm__ __volatile__ ("isb" : : : "memory")

/* Data Synchronization Barrier */
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")

// Dumb mapping over the registers of a USART device on STM32//STM32上USART设备寄存器上的哑映射
struct USARTMin {
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
};

#if WITHIN(SERIAL_PORT, 1, 6)
  // Depending on the CPU, the serial port is different for USART1//根据CPU的不同，USART1的串行端口不同
  static const uintptr_t regsAddr[] = {
    TERN(STM32F1xx, 0x40013800, 0x40011000), // USART1//USART1
    0x40004400, // USART2//USART2
    0x40004800, // USART3//USART3
    0x40004C00, // UART4_BASE//UART4_基
    0x40005000, // UART5_BASE//UART5_基地
    0x40011400  // USART6//USART6
  };
  static USARTMin * regs = (USARTMin*)regsAddr[SERIAL_PORT - 1];
#endif

static void TXBegin() {
  #if !WITHIN(SERIAL_PORT, 1, 6)
    #warning "Using POSTMORTEM_DEBUGGING requires a physical U(S)ART hardware in case of severe error."
    #warning "Disabling the severe error reporting feature currently because the used serial port is not a HW port."
  #else
    // This is common between STM32F1/STM32F2 and STM32F4//这在STM32F1/STM32F2和STM32F4之间很常见
    const int nvicUART[] = { /* NVIC_USART1 */ 37, /* NVIC_USART2 */ 38, /* NVIC_USART3 */ 39, /* NVIC_UART4 */ 52, /* NVIC_UART5 */ 53, /* NVIC_USART6 */ 71 };
    int nvicIndex = nvicUART[SERIAL_PORT - 1];

    struct NVICMin {
      volatile uint32_t ISER[32];
      volatile uint32_t ICER[32];
    };

    NVICMin *nvicBase = (NVICMin*)0xE000E100;
    SBI32(nvicBase->ICER[nvicIndex >> 5], nvicIndex & 0x1F);

    // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    dsb();
    isb();

    // Example for USART1 disable:  (RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN))//USART1禁用示例：（RCC->APB2ENR&=~（RCC\u APB2ENR\u USART1EN））
    // Too difficult to reimplement here, let's query the STM32duino macro here//很难在这里重新实现，让我们在这里查询STM32duino宏
    #if SERIAL_PORT == 1
      __HAL_RCC_USART1_CLK_DISABLE();
      __HAL_RCC_USART1_CLK_ENABLE();
    #elif SERIAL_PORT == 2
      __HAL_RCC_USART2_CLK_DISABLE();
      __HAL_RCC_USART2_CLK_ENABLE();
    #elif SERIAL_PORT == 3
      __HAL_RCC_USART3_CLK_DISABLE();
      __HAL_RCC_USART3_CLK_ENABLE();
    #elif SERIAL_PORT == 4
      __HAL_RCC_UART4_CLK_DISABLE(); // BEWARE: UART4 and not USART4 here//注意：这里是UART4而不是USART4
      __HAL_RCC_UART4_CLK_ENABLE();
    #elif SERIAL_PORT == 5
      __HAL_RCC_UART5_CLK_DISABLE(); // BEWARE: UART5 and not USART5 here//注意：这里是UART5而不是USART5
      __HAL_RCC_UART5_CLK_ENABLE();
    #elif SERIAL_PORT == 6
      __HAL_RCC_USART6_CLK_DISABLE();
      __HAL_RCC_USART6_CLK_ENABLE();
    #endif

    uint32_t brr = regs->BRR;
    regs->CR1 = 0; // Reset the USART//重置USART
    regs->CR2 = 0; // 1 stop bit//1停止位

    // If we don't touch the BRR (baudrate register), we don't need to recompute.//如果我们不接触BRR（波特率寄存器），我们就不需要重新计算。
    regs->BRR = brr;

    regs->CR1 = _BV(3) | _BV(13); // 8 bits, no parity, 1 stop bit (TE | UE)//8位，无奇偶校验，1个停止位（TE | UE）
  #endif
}

// A SW memory barrier, to ensure GCC does not overoptimize loops//软件内存屏障，确保GCC不会过度优化循环
#define sw_barrier() __asm__ volatile("": : :"memory");
static void TX(char c) {
  #if WITHIN(SERIAL_PORT, 1, 6)
    constexpr uint32_t usart_sr_txe = _BV(7);
    while (!(regs->SR & usart_sr_txe)) {
      TERN_(USE_WATCHDOG, HAL_watchdog_refresh());
      sw_barrier();
    }
    regs->DR = c;
  #else
    // Let's hope a mystical guru will fix this, one day by writting interrupt-free USB CDC ACM code (or, at least, by polling the registers since interrupt will be queued but will never trigger)//希望有一天，一位神秘的大师能够解决这个问题，他可以编写无中断的USB CDC ACM代码（或者，至少可以轮询寄存器，因为中断将排队，但永远不会触发）
    // For now, it's completely lost to oblivion.//现在，它完全被遗忘了。
  #endif
}

void install_min_serial() {
  HAL_min_serial_init = &TXBegin;
  HAL_min_serial_out = &TX;
}

#if DISABLED(DYNAMIC_VECTORTABLE) && DISABLED(STM32F0xx) // Cortex M0 can't jump to a symbol that's too far from the current function, so we work around this in exception_arm.cpp//Cortex M0无法跳转到距离当前函数太远的符号，因此我们在exception_arm.cpp中解决了这个问题
extern "C" {
  __attribute__((naked)) void JumpHandler_ASM() {
    __asm__ __volatile__ (
      "b CommonHandler_ASM\n"
    );
  }
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) HardFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) BusFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) UsageFault_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) MemManage_Handler();
  void __attribute__((naked, alias("JumpHandler_ASM"), nothrow)) NMI_Handler();
}
#endif

#endif // POSTMORTEM_DEBUGGING//死后调试
#endif // ARDUINO_ARCH_STM32//ARDUINO_ARCH_STM32
