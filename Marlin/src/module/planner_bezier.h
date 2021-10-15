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
 * planner_bezier.h
 *
 * Compute and buffer movement commands for Bézier curves
 */

#include "../core/types.h"

void cubic_b_spline(
  const xyze_pos_t &position,       // current position//当前位置
  const xyze_pos_t &target,         // target position//目标位置
  const xy_pos_t (&offsets)[2],     // a pair of offsets//一对偏移量
  const_feedRate_t scaled_fr_mm_s,  // mm/s scaled by feedrate %//毫米/秒，按进给速度%缩放
  const uint8_t extruder
);
