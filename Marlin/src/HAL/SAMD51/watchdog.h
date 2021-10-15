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

// Initialize watchdog with a 4 second interrupt time//用4秒的中断时间初始化看门狗
void watchdog_init();

// Reset watchdog. MUST be called at least every 4 seconds after the//重置看门狗。必须在测试后至少每4秒调用一次
// first watchdog_init or SAMD will go into emergency procedures.//第一看门狗_init或SAMD将进入紧急程序。
inline void HAL_watchdog_refresh() {
  SYNC(WDT->SYNCBUSY.bit.CLEAR);        // Test first if previous is 'ongoing' to save time waiting for command execution//如果“上一步”正在进行，则首先测试，以节省等待命令执行的时间
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}
