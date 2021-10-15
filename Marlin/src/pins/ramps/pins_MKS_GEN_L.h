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
 * MKS GEN L – Arduino Mega2560 with RAMPS v1.4 pin assignments
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS GEN L supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "MKS GEN L"

////
// Heaters / Fans//加热器/风扇
////
// Power outputs EFBF or EFBE//功率输出EFBF或EFBE
#define MOSFET_D_PIN 7

////
// CS Pins wired to avoid conflict with the LCD//连接CS引脚以避免与LCD冲突
// See https://www.thingiverse.com/asset:66604//看https://www.thingiverse.com/asset:66604
////

#ifndef X_CS_PIN
  #define X_CS_PIN 59
#endif

#ifndef Y_CS_PIN
  #define Y_CS_PIN 63
#endif

#include "pins_RAMPS.h"
