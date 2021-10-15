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
 * VORON Design v2 pin assignments
 * See https://github.com/mzbotreprap/VORON/blob/master/Firmware/Marlin/pins_RAMPS_VORON.h
 */

#define BOARD_INFO_NAME "VORON Design v2"

#define RAMPS_D8_PIN                          11

#include "pins_RAMPS.h"

////
// Heaters / Fans//加热器/风扇
////
#undef FAN_PIN
#define FAN_PIN                                5  // Using the pin for the controller fan since controller fan is always on.//使用控制器风扇的引脚，因为控制器风扇始终打开。
#define CONTROLLER_FAN_PIN                     8

////
// Auto fans//汽车风扇
////
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                      6  // Servo pin 6 for E3D Fan//E3D风扇的伺服销6
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN                      6  // Servo pin 6 for E3D Fan (same pin for both extruders since it's the same fan)//E3D风扇的伺服引脚6（两台挤出机的引脚相同，因为它是同一个风扇）
#endif

////
// LCDs and Controllers//液晶显示器和控制器
////
#undef BEEPER_PIN
