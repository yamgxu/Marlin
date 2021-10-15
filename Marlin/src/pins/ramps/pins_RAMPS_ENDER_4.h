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

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "Ender-4 only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "Ender-4"

#include "pins_RAMPS.h"

// The board only has one PWM fan connector. The others are 12V always-on.//该板只有一个PWM风扇接头。其他的都是12V，一直开着。
// The default config uses this pin to control the brightness of the LED//默认配置使用此引脚控制LED的亮度
// band (case light). Thus the hotend and controller fans are always-on.//带（箱灯）。因此，热端和控制器风扇始终打开。

#if ENABLED(CASE_LIGHT_ENABLE)
  #undef FAN_PIN
  #ifndef CASE_LIGHT_PIN
    #define CASE_LIGHT_PIN RAMPS_D9_PIN
  #endif
#endif
