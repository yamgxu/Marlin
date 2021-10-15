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
 * CartesioV11 pin assignments
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME "CN Controls V11"

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            43
#define Y_STOP_PIN                            45
#define Z_STOP_PIN                            42

////
// Steppers//踏步机
////
#define X_STEP_PIN                            34
#define X_DIR_PIN                             36
#define X_ENABLE_PIN                          35

#define Y_STEP_PIN                            37
#define Y_DIR_PIN                             39
#define Y_ENABLE_PIN                          38

#define Z_STEP_PIN                            40
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          41

#define E0_STEP_PIN                           29
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                          3

#define E1_STEP_PIN                           61
#define E1_DIR_PIN                            62
#define E1_ENABLE_PIN                         60

#define E2_STEP_PIN                           15
#define E2_DIR_PIN                            14
#define E2_ENABLE_PIN                         16

#define E3_STEP_PIN                           44
#define E3_DIR_PIN                            49
#define E3_ENABLE_PIN                         47

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             0  // Analog Input//模拟输入
#define TEMP_1_PIN                             3  // Analog Input.  3 for tool2 -> 2 for chambertemp//模拟输入。工具2为3->chambertemp为2
#define TEMP_2_PIN                             2  // Analog Input.  9 for tool3 -> 2 for chambertemp//模拟输入。9用于工具3->2用于chambertemp
#define TEMP_3_PIN                            11  // Analog Input. 11 for tool4 -> 2 for chambertemp//模拟输入。11用于工具4->2用于chambertemp
#define TEMP_BED_PIN                           1  // Analog Input//模拟输入

#ifndef TEMP_CHAMBER_PIN
  //#define TEMP_CHAMBER_PIN                   2  // Analog Input//#定义温度室引脚2//模拟输入
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           5
#define HEATER_1_PIN                          58
#define HEATER_2_PIN                          64
#define HEATER_3_PIN                          46
#define HEATER_BED_PIN                         2

#ifndef FAN_PIN
  //#define FAN_PIN                            7  // common PWM pin for all tools//#定义所有工具的风扇引脚7//公共PWM引脚
#endif

////
// Auto fans//汽车风扇
////
#define AUTO_FAN_PIN                           7
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E3_AUTO_FAN_PIN
  #define E3_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  53
#define SD_DETECT_PIN                         13

// Tools//工具

//#define TOOL_0_PIN                           4//#定义工具0引脚4
//#define TOOL_1_PIN                          59//#定义工具1的针脚59
//#define TOOL_2_PIN                           8//#定义刀具2引脚8
//#define TOOL_3_PIN                          30//#定义刀具3引脚30
//#define TOOL_PWM_PIN                         7  // common PWM pin for all tools//#定义所有刀具的刀具脉冲宽度调制引脚7//通用脉冲宽度调制引脚

// Common I/O//通用I/O

//#define FIL_RUNOUT_PIN                      -1//#定义FIL_跳动_引脚-1
//#define PWM_1_PIN                           11//#定义PWM_1_引脚11
//#define PWM_2_PIN                           10//#定义PWM_2_引脚10
//#define SPARE_IO                            12//#定义备用IO 12

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                             6

// Pins for DOGM SPI LCD Support//用于DOGM SPI LCD支持的引脚
#define DOGLCD_A0                             26
#define DOGLCD_CS                             24
#define DOGLCD_MOSI                           -1  // Prevent auto-define by Conditionals_post.h//防止通过条件自动定义\u post.h
#define DOGLCD_SCK                            -1

#define BTN_EN1                               23
#define BTN_EN2                               25
#define BTN_ENC                               27

// Hardware buttons for manual movement of XYZ//手动移动XYZ的硬件按钮
#define SHIFT_OUT_PIN                         19
#define SHIFT_LD_PIN                          18
#define SHIFT_CLK_PIN                         17

//#define UI1                                 31//#定义UI1 31
//#define UI2                                 22//#定义UI2 22

#define STAT_LED_BLUE_PIN                     -1
#define STAT_LED_RED_PIN                      31
