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
 * bq ZUM Mega 3D board definition
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#define BOARD_INFO_NAME "ZUM Mega 3D"

////
// Heaters / Fans//加热器/风扇
////
#define RAMPS_D8_PIN                          10
#define RAMPS_D9_PIN                          12
#define RAMPS_D10_PIN                          9
#define MOSFET_D_PIN                           7

////
// Auto fans//汽车风扇
////
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                     11
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN                      6
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN                      6
#endif
#ifndef E3_AUTO_FAN_PIN
  #define E3_AUTO_FAN_PIN                      6
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#define SPINDLE_LASER_ENA_PIN                 40  // Pullup or pulldown!//拉起还是拉下！
#define SPINDLE_LASER_PWM_PIN                 44  // Hardware PWM//硬件脉宽调制
#define SPINDLE_DIR_PIN                       42

////
// Limit Switches//限位开关
////
#define X_MAX_PIN                             79  // 2// 2

////
// Import RAMPS 1.3 pins//导入斜坡1.3引脚
////
#include "pins_RAMPS_13.h"

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#undef Z_MIN_PROBE_PIN
#define Z_MIN_PROBE_PIN                       19  // IND_S_5V//IND_S_5V

#undef Z_ENABLE_PIN
#define Z_ENABLE_PIN                          77  // 62// 62

////
// Steppers//踏步机
////
#define DIGIPOTSS_PIN                         22
#define DIGIPOT_CHANNELS { 4, 5, 3, 0, 1 }

////
// Temperature Sensors//温度传感器
////
#undef TEMP_1_PIN
#define TEMP_1_PIN                            14  // Analog Input (15)//模拟输入（15）

#undef TEMP_BED_PIN
#define TEMP_BED_PIN                          15  // Analog Input (14)//模拟输入（14）

////
// Misc. Functions//杂项。功能
////
#undef PS_ON_PIN                                  // 12// 12
#define PS_ON_PIN                             81  // External Power Supply//外部电源

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                      44  // Hardware PWM//硬件脉宽调制
#endif

// This board has headers for Z-min, Z-max and IND_S_5V *but* as the bq team//该板具有Z-min、Z-max和IND_S_5V*的标题，但*作为bq团队
// decided to ship the printer only with the probe and no additional Z-min//决定仅将打印机与探头一起装运，不增加Z-min
// endstop and the instruction manual advises the user to connect the probe to//endstop和使用手册建议用户将探头连接到
// IND_S_5V the option Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN will not work.//IND_S_5V Z_MIN_PROBE_使用的选项Z_MIN_ENDSTOP_引脚将不起作用。
#ifdef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #undef Z_MIN_PIN
  #undef Z_MAX_PIN
  #define Z_MIN_PIN                           19  // IND_S_5V//IND_S_5V
  #define Z_MAX_PIN                           18  // Z-MIN Label//Z-MIN标签
#endif

////
// Used by the Hephestos 2 heated bed upgrade kit//Hephestos 2加热床升级套件使用
////
#if ENABLED(HEPHESTOS2_HEATED_BED_KIT)
  #undef HEATER_BED_PIN
  #define HEATER_BED_PIN                       8
#endif
