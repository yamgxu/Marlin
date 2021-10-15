/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Malyan M180 pin assignments
 * Contributed by Timo Birnschein (timo.birnschein@microforge.de)
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Malyan M180 v.2"
////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            48
#define Y_STOP_PIN                            46
#define Z_STOP_PIN                            42

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     -1
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            55
#define X_DIR_PIN                             54
#define X_ENABLE_PIN                          56

#define Y_STEP_PIN                            59
#define Y_DIR_PIN                             58
#define Y_ENABLE_PIN                          60

#define Z_STEP_PIN                            63
#define Z_DIR_PIN                             62
#define Z_ENABLE_PIN                          64

#define E0_STEP_PIN                           25
#define E0_DIR_PIN                            24
#define E0_ENABLE_PIN                         26

#define E1_STEP_PIN                           29
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         39

////
// Temperature Sensors//温度传感器
////
#define TEMP_BED_PIN                          15  // Analog Input//模拟输入

// Extruder thermocouples 0 and 1 are read out by two separate ICs using//挤出机热电偶0和1由两个单独的IC使用
// SPI for Max6675 Thermocouple//Max6675热电偶的SPI
// Uses a separate SPI bus//使用单独的SPI总线
#define THERMO_SCK_PIN                        78  // E2 - SCK//E2-SCK
#define THERMO_DO_PIN                          3  // E5 - DO//E5-DO
#define THERMO_CS1_PIN                         5  // E3 - CS0//E3-CS0
#define THERMO_CS2_PIN                         2  // E4 - CS1//E4-CS1

#define MAX6675_SS_PIN            THERMO_CS1_PIN
#define MAX6675_SS2_PIN           THERMO_CS2_PIN
#define MAX6675_SCK_PIN           THERMO_SCK_PIN
#define MAX6675_DO_PIN             THERMO_DO_PIN

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           6
#define HEATER_1_PIN                          11
#define HEATER_BED_PIN                        45

#ifndef FAN_PIN
  #define FAN_PIN                              7  // M106 Sxxx command supported and tested. M107 as well.//支持并测试了M106 Sxxx命令。M107也是。
#endif

#ifndef FAN_PIN1
  #define FAN_PIN1                            12  // Currently Unsupported by Marlin//目前不受Marlin支持
#endif
