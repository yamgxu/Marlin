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
 * InterruptVectors_Due.cpp - This module relocates the Interrupt vector table to SRAM,
 *  allowing to register new interrupt handlers at runtime. Specially valuable and needed
 * because Arduino runtime allocates some interrupt handlers that we NEED to override to
 * properly support extended functionality, as for example, USB host or USB device (MSD, MTP)
 * and custom serial port handlers, and we don't actually want to modify and/or recompile the
 * Arduino runtime. We just want to run as much as possible on Stock Arduino
 *
 * Copyright (c) 2017 Eduardo José Tagle. All right reserved
 */
#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "HAL.h"
#include "InterruptVectors.h"

/* The relocated Exception/Interrupt Table - According to the ARM
   reference manual, alignment to 128 bytes should suffice, but in
   practice, we need alignment to 256 bytes to make this work in all
   cases */
__attribute__ ((aligned(256)))
static DeviceVectors ram_tab = { nullptr };

/**
 * This function checks if the exception/interrupt table is already in SRAM or not.
 * If it is not, then it copies the ROM table to the SRAM and relocates the table
 * by reprogramming the NVIC registers
 */
static pfnISR_Handler* get_relocated_table_addr() {
  // Get the address of the interrupt/exception table//获取中断/异常表的地址
  uint32_t isrtab = SCB->VTOR;

  // If already relocated, we are done!//如果已经搬迁，我们就完了！
  if (isrtab >= IRAM0_ADDR)
    return (pfnISR_Handler*)isrtab;

  // Get the address of the table stored in FLASH//获取存储在闪存中的表的地址
  const pfnISR_Handler* romtab = (const pfnISR_Handler*)isrtab;

  // Copy it to SRAM//将其复制到SRAM
  memcpy(&ram_tab, romtab, sizeof(ram_tab));

  // Disable global interrupts//禁用全局中断
  CRITICAL_SECTION_START();

  // Set the vector table base address to the SRAM copy//将向量表基址设置为SRAM副本
  SCB->VTOR = (uint32_t)(&ram_tab);

  // Reenable interrupts//可重入中断
  CRITICAL_SECTION_END();

  // Return the address of the table//返回表的地址
  return (pfnISR_Handler*)(&ram_tab);
}

pfnISR_Handler install_isr(IRQn_Type irq, pfnISR_Handler newHandler) {
  // Get the address of the relocated table//获取重新定位的表的地址
  pfnISR_Handler *isrtab = get_relocated_table_addr();

  // Disable global interrupts//禁用全局中断
  CRITICAL_SECTION_START();

  // Get the original handler//获取原始处理程序
  pfnISR_Handler oldHandler = isrtab[irq + 16];

  // Install the new one//安装新的
  isrtab[irq + 16] = newHandler;

  // Reenable interrupts//可重入中断
  CRITICAL_SECTION_END();

  // Return the original one//退回原版
  return oldHandler;
}

#endif
