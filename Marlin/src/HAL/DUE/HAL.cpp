/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
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

/**
 * HAL for Arduino Due and compatible (SAM3X8E)
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

#include <Wire.h>
#include "usb/usb_task.h"

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

uint16_t HAL_adc_result;

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

// HAL initialization task//HAL初始化任务
void HAL_init() {
  // Initialize the USB stack//初始化USB堆栈
  #if ENABLED(SDSUPPORT)
    OUT_WRITE(SDSS, HIGH);  // Try to set SDSS inactive before any other SPI users start up//在任何其他SPI用户启动之前，尝试将SDS设置为非活动状态
  #endif
  usb_task_init();
  TERN_(POSTMORTEM_DEBUGGING, install_min_serial()); // Install the min serial handler//安装最小串行处理程序
}

// HAL idle task//HAL空闲任务
void HAL_idletask() {
  // Perform USB stack housekeeping//执行USB堆栈管理
  usb_task_idle();
}

// Disable interrupts//禁用中断
void cli() { noInterrupts(); }

// Enable interrupts//启用中断
void sei() { interrupts(); }

void HAL_clear_reset_source() { }

uint8_t HAL_get_reset_source() {
  switch ((RSTC->RSTC_SR >> 8) & 0x07) {
    case 0: return RST_POWER_ON;
    case 1: return RST_BACKUP;
    case 2: return RST_WATCHDOG;
    case 3: return RST_SOFTWARE;
    case 4: return RST_EXTERNAL;
    default: return 0;
  }
}

void HAL_reboot() { rstc_start_software_reset(RSTC); }

void _delay_ms(const int delay_ms) {
  // Todo: port for Due?//Todo:到期港？
  delay(delay_ms);
}

extern "C" {
  extern unsigned int _ebss; // end of bss section//bss部分结束
}

// Return free memory between end of heap (or end bss) and whatever is current//返回堆端（或bss端）和任何当前值之间的可用内存
int freeMemory() {
  int free_memory, heap_end = (int)_sbrk(0);
  return (int)&free_memory - (heap_end ?: (int)&_ebss);
}

// ------------------------// ------------------------
// ADC//模数转换器
// ------------------------// ------------------------

void HAL_adc_start_conversion(const uint8_t ch) {
  HAL_adc_result = analogRead(ch);
}

uint16_t HAL_adc_get_result() {
  // nop//不
  return HAL_adc_result;
}

// Forward the default serial ports//转发默认串行端口
#if USING_HW_SERIAL0
  DefaultSerial1 MSerial0(false, Serial);
#endif
#if USING_HW_SERIAL1
  DefaultSerial2 MSerial1(false, Serial1);
#endif
#if USING_HW_SERIAL2
  DefaultSerial3 MSerial2(false, Serial2);
#endif
#if USING_HW_SERIAL3
  DefaultSerial4 MSerial3(false, Serial3);
#endif

#endif // ARDUINO_ARCH_SAM//阿杜伊诺·阿丘·萨姆
