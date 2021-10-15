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

// R25 = 100 kOhm, beta25 = 4267 K, 4.7 kOhm pull-up//R25=100千欧，beta25=4267千欧，上拉4.7千欧
// 100k ParCan thermistor (104GT-2)//100k帕坎热敏电阻（104GT-2）
// ATC Semitec 104GT-2/104NT-4-R025H42G (Used in ParCan)//ATC Semitec 104GT-2/104NT-4-R025H42G（用于ParCan）
// Verified by linagee. Source: https://www.mouser.com/datasheet/2/362/semitec%20usa%20corporation_gtthermistor-1202937.pdf//经利纳吉核实。资料来源：https://www.mouser.com/datasheet/2/362/semitec%20usa%20corporation_gtthermistor-1202937.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance//使用4.7kohm上拉、分压器数学和制造商提供的温度/电阻进行计算
constexpr temp_entry_t temptable_5[] PROGMEM = {
  { OV(   1), 713 },
  { OV(  17), 300 }, // top rating 300C//最高额定值300C
  { OV(  20), 290 },
  { OV(  23), 280 },
  { OV(  27), 270 },
  { OV(  31), 260 },
  { OV(  37), 250 },
  { OV(  43), 240 },
  { OV(  51), 230 },
  { OV(  61), 220 },
  { OV(  73), 210 },
  { OV(  87), 200 },
  { OV( 106), 190 },
  { OV( 128), 180 },
  { OV( 155), 170 },
  { OV( 189), 160 },
  { OV( 230), 150 },
  { OV( 278), 140 },
  { OV( 336), 130 },
  { OV( 402), 120 },
  { OV( 476), 110 },
  { OV( 554), 100 },
  { OV( 635),  90 },
  { OV( 713),  80 },
  { OV( 784),  70 },
  { OV( 846),  60 },
  { OV( 897),  50 },
  { OV( 937),  40 },
  { OV( 966),  30 },
  { OV( 986),  20 },
  { OV(1000),  10 },
  { OV(1010),   0 }
};
