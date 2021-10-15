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
 * CartesioV12 pin assignments
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME "CN Controls V12"

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            19
#define Y_STOP_PIN                            22
#define Z_STOP_PIN                            23

////
// Steppers//踏步机
////
#define X_STEP_PIN                            25
#define X_DIR_PIN                             27
#define X_ENABLE_PIN                          26

#define Y_STEP_PIN                            28
#define Y_DIR_PIN                             30
#define Y_ENABLE_PIN                          29

#define Z_STEP_PIN                            31
#define Z_DIR_PIN                             33
#define Z_ENABLE_PIN                          32

#define E0_STEP_PIN                           57
#define E0_DIR_PIN                            55
#define E0_ENABLE_PIN                         58

#define E1_STEP_PIN                           61
#define E1_DIR_PIN                            62
#define E1_ENABLE_PIN                         60

#define E2_STEP_PIN                           46
#define E2_DIR_PIN                            66
#define E2_ENABLE_PIN                         44

#define E3_STEP_PIN                           45
#define E3_DIR_PIN                            69
#define E3_ENABLE_PIN                         47

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             0  // Analog Input//模拟输入
#define TEMP_1_PIN                             9  // Analog Input.  9 for tool2 -> 13 for chambertemp//模拟输入。工具2为9->chambertemp为13
#define TEMP_2_PIN                            13  // Analog Input. 10 for tool3 -> 13 for chambertemp//模拟输入。10用于工具3->13用于chambertemp
#define TEMP_3_PIN                            11  // Analog Input. 11 for tool4 -> 13 for chambertemp//模拟输入。11用于4号工具->13用于chambertemp
#define TEMP_BED_PIN                          14  // Analog Input//模拟输入

#ifndef TEMP_CHAMBER_PIN
  //#define TEMP_CHAMBER_PIN                  13  // Analog Input//#定义温度室针脚13//模拟输入
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          11
#define HEATER_1_PIN                           9
#define HEATER_2_PIN                           6
#define HEATER_3_PIN                           3
#define HEATER_BED_PIN                        24

#ifndef FAN_PIN
  #define FAN_PIN                              5  // 5 is PWMtool3 -> 7 is common PWM pin for all tools//5是PWMtool3->7是所有工具的公共PWM引脚
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
#define SD_DETECT_PIN                         15

// Tools//工具

//#define TOOL_0_PIN                          56//#定义工具_0_销56
//#define TOOL_0_PWM_PIN                      10  // red warning led at dual extruder//#定义工具\u 0\u PWM\u引脚10//双挤出机上的红色警告led
//#define TOOL_1_PIN                          59//#定义工具1的针脚59
//#define TOOL_1_PWM_PIN                       8  // lights at dual extruder//#定义双挤出机上的刀具\u 1 \u PWM \u引脚8//灯
//#define TOOL_2_PIN                           4//#定义刀具2引脚4
//#define TOOL_2_PWM_PIN                       5//#定义刀具_2_PWM_引脚5
//#define TOOL_3_PIN                          14//#定义工具3引脚14
//#define TOOL_3_PWM_PIN                       2//#定义刀具_3_PWM_引脚2

// Common I/O//通用I/O

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      18
#endif
//#define PWM_1_PIN                           12//#定义PWM_1_引脚12
//#define PWM_2_PIN                           13//#定义PWM_2_引脚13
//#define SPARE_IO                            17//#定义备用IO 17

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                            16

// Pins for DOGM SPI LCD Support//用于DOGM SPI LCD支持的引脚
#define DOGLCD_A0                             39
#define DOGLCD_CS                             35
#define DOGLCD_MOSI                           48
#define DOGLCD_SCK                            49
#define LCD_SCREEN_ROT_180

// The encoder and click button//打开编码器并单击按钮
#define BTN_EN1                               36
#define BTN_EN2                               34
#define BTN_ENC                               38

// Hardware buttons for manual movement of XYZ//手动移动XYZ的硬件按钮
#define SHIFT_OUT_PIN                         42
#define SHIFT_LD_PIN                          41
#define SHIFT_CLK_PIN                         40

//#define UI1                                 43//#定义UI143
//#define UI2                                 37//#定义UI2 37

#define STAT_LED_BLUE_PIN                     -1
#define STAT_LED_RED_PIN                      10  // TOOL_0_PWM_PIN//刀具\u 0\u脉宽调制\u引脚
