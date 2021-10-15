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
#pragma once

/**
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#include <libmaple/iwdg.h>

// Initialize watchdog with a 4 or 8 second countdown time//用4秒或8秒倒计时初始化看门狗
void watchdog_init();

// Reset watchdog. MUST be called every 4 or 8 seconds after the//重置看门狗。必须在事件发生后每4或8秒调用一次
// first watchdog_init or the STM32F1 will reset.//第一个看门狗_init或STM32F1将复位。
void HAL_watchdog_refresh();
