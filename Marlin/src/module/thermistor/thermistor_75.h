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
 * R25 = 100 kOhm, beta25 = 4100 K, 4.7 kOhm pull-up,
 * Generic Silicon Heat Pad with NTC 100K thermistor
 *
 * Many generic silicone heat pads use the MGB18-104F39050L32 thermistor, applicable to various
 * wattages and voltages. This table is correct if this part is used. It's been optimized
 * to provide good granularity in the 60-110C range, good for PLA and ABS. For higher temperature
 * filament (e.g., nylon) uncomment HIGH_TEMP_RANGE_75 for increased accuracy. If higher
 * temperatures aren't used it can improve performance slightly to leave it commented out.
 */

//#define HIGH_TEMP_RANGE_75//#定义高温范围75

constexpr temp_entry_t temptable_75[] PROGMEM = { // Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor//带NTC 100K MGB18-104F39050L32热敏电阻的通用硅热垫
  { OV(111.06), 200 }, // v=0.542 r=571.747 res=0.501 degC/count//v=0.542 r=571.747 res=0.501摄氏度/计数

  #ifdef HIGH_TEMP_RANGE_75
    { OV(174.87), 175 }, // v=0.854 r=967.950 res=0.311 degC/count  These values are valid.  But they serve no//v=0.854 r=967.950分辨率=0.311摄氏度/计数这些值有效。但它们不起作用
    { OV(191.64), 170 }, // v=0.936 r=1082.139 res=0.284 degC/count  purpose.  It is better to delete them so//v=0.936 r=1082.139分辨率=0.284摄氏度/计数目的。因此，最好删除它们
    { OV(209.99), 165 }, // v=1.025 r=1212.472 res=0.260 degC/count  the search is quicker and get to the meaningful//v=1.025 r=1212.472 res=0.260 degC/count搜索速度更快，到达有意义的位置
    { OV(230.02), 160 }, // v=1.123 r=1361.590 res=0.239 degC/count  part of the table sooner.//v=1.123 r=1361.590 res=0.239 degC/计算表的一部分。
    { OV(251.80), 155 }, // v=1.230 r=1532.621 res=0.220 degC/count//v=1.230 r=1532.621 res=0.220摄氏度/计数
  #endif

  { OV(275.43), 150 }, // v=1.345 r=1729.283 res=0.203 degC/count//v=1.345 r=1729.283 res=0.203摄氏度/计数

  #ifdef HIGH_TEMP_RANGE_75
    { OV(300.92), 145 }, // v=1.469 r=1956.004 res=0.189 degC/coun//v=1.469 r=1956.004 res=0.189摄氏度/国家
  #endif

  { OV( 328.32), 140 }, // v=1.603 r=2218.081 res=0.176 degC/count//v=1.603 r=2218.081 res=0.176摄氏度/计数
  { OV( 388.65), 130 }, // v=1.898 r=2874.980 res=0.156 degC/count//v=1.898 r=2874.980 res=0.156摄氏度/计数
  { OV( 421.39), 125 }, // v=2.058 r=3286.644 res=0.149 degC/count//v=2.058 r=3286.644 res=0.149摄氏度/计数
  { OV( 455.65), 120 }, // v=2.225 r=3768.002 res=0.143 degC/count//v=2.225 r=3768.002分辨率=0.143摄氏度/计数
  { OV( 491.17), 115 }, // v=2.398 r=4332.590 res=0.139 degC/count//v=2.398 r=4332.590 res=0.139摄氏度/计数
  { OV( 527.68), 110 }, // v=2.577 r=4996.905 res=0.136 degC/count//v=2.577 r=4996.905分辨率=0.136摄氏度/计数
  { OV( 564.81), 105 }, // v=2.758 r=5781.120 res=0.134 degC/count//v=2.758 r=5781.120分辨率=0.134摄氏度/计数
  { OV( 602.19), 100 }, // v=2.940 r=6710.000 res=0.134 degC/count//v=2.940 r=6710.000 res=0.134摄氏度/计数
  { OV( 676.03),  90 }, // v=3.301 r=9131.018 res=0.138 degC/count//v=3.301 r=9131.018分辨率=0.138摄氏度/计数
  { OV( 745.85),  80 }, // v=3.642 r=12602.693 res=0.150 degC/count//v=3.642 r=12602.693 res=0.150摄氏度/计数
  { OV( 778.31),  75 }, // v=3.800 r=14889.001 res=0.159 degC/count//v=3.800 r=14889.001 res=0.159摄氏度/计数
  { OV( 808.75),  70 }, // v=3.949 r=17658.700 res=0.171 degC/count//v=3.949 r=17658.700分辨率=0.171摄氏度/计数
  { OV( 836.94),  65 }, // v=4.087 r=21028.040 res=0.185 degC/count//v=4.087 r=21028.040 res=0.185摄氏度/计数
  { OV( 862.74),  60 }, // v=4.213 r=25144.568 res=0.204 degC/count//v=4.213 r=25144.568分辨率=0.204摄氏度/计数
  { OV( 886.08),  55 }, // v=4.327 r=30196.449 res=0.227 degC/count//v=4.327 r=30196.449分辨率=0.227摄氏度/计数
  { OV( 906.97),  50 }, // v=4.429 r=36424.838 res=0.255 degC/count//v=4.429 r=36424.838分辨率=0.255摄氏度/计数
  { OV( 941.65),  40 }, // v=4.598 r=53745.337 res=0.333 degC/count//v=4.598 r=53745.337 res=0.333摄氏度/计数
  { OV( 967.76),  30 }, // v=4.725 r=80880.630 res=0.452 degC/count//v=4.725 r=80880.630分辨率=0.452摄氏度/计数
  { OV( 978.03),  25 }, // v=4.776 r=100000.000 res=0.535 degC/count//v=4.776 r=100000.000分辨率=0.535摄氏度/计数
  { OV( 981.68),  23 }, // v=4.793 r=109024.395 res=0.573 degC/count//v=4.793 r=109024.395 res=0.573摄氏度/计数
  { OV( 983.41),  22 }, // v=4.802 r=113875.430 res=0.594 degC/count//v=4.802 r=113875.430分辨率=0.594摄氏度/计数
  { OV( 985.08),  21 }, // v=4.810 r=118968.955 res=0.616 degC/count//v=4.810 r=118968.955分辨率=0.616摄氏度/计数
  { OV( 986.70),  20 }, // v=4.818 r=124318.354 res=0.638 degC/count//v=4.818 r=124318.354分辨率=0.638摄氏度/计数
  { OV( 993.94),  15 }, // v=4.853 r=155431.302 res=0.768 degC/count//v=4.853 r=155431.302分辨率=0.768摄氏度/计数
  { OV( 999.96),  10 }, // v=4.883 r=195480.023 res=0.934 degC/count//v=4.883 r=195480.023分辨率=0.934摄氏度/计数
  { OV(1008.95),   0 }  // v=4.926 r=314997.575 res=1.418 degC/count//v=4.926 r=314997.575 res=1.418 degC/计数
};
