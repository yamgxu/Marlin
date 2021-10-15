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
 * Elefu RA Board Pin Assignments
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Elefu Ra v3"

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             35
#define X_MAX_PIN                             34
#define Y_MIN_PIN                             33
#define Y_MAX_PIN                             32
#define Z_MIN_PIN                             31
#define Z_MAX_PIN                             30

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     30
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            49
#define X_DIR_PIN                             13
#define X_ENABLE_PIN                          48

#define Y_STEP_PIN                            11
#define Y_DIR_PIN                              9
#define Y_ENABLE_PIN                          12

#define Z_STEP_PIN                             7
#define Z_DIR_PIN                              6
#define Z_ENABLE_PIN                           8

#define E0_STEP_PIN                           40
#define E0_DIR_PIN                            41
#define E0_ENABLE_PIN                         37

#define E1_STEP_PIN                           18
#define E1_DIR_PIN                            19
#define E1_ENABLE_PIN                         38

#define E2_STEP_PIN                           43
#define E2_DIR_PIN                            47
#define E2_ENABLE_PIN                         42

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             3  // Analog Input//模拟输入
#define TEMP_1_PIN                             2  // Analog Input//模拟输入
#define TEMP_2_PIN                             1  // Analog Input//模拟输入
#define TEMP_BED_PIN                           0  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          45  // 12V PWM1//12V PWM1
#define HEATER_1_PIN                          46  // 12V PWM2//12V PWM2
#define HEATER_2_PIN                          17  // 12V PWM3//12V PWM3
#define HEATER_BED_PIN                        44  // DOUBLE 12V PWM//双12V PWM

#ifndef FAN_PIN
  #define FAN_PIN                             16  // 5V PWM//5V脉宽调制
#endif

////
// Misc. Functions//杂项。功能
////
#define PS_ON_PIN                             10  // Set to -1 if using a manual switch on the PWRSW Connector//如果在PWRSW接头上使用手动开关，则设置为-1
#define SLEEP_WAKE_PIN                        26  // This feature still needs work//这个功能还需要改进
#define PHOTOGRAPH_PIN                        29

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                            36

#if ENABLED(RA_CONTROL_PANEL)

  #define SDSS                                53
  #define SD_DETECT_PIN                       28

  #define BTN_EN1                             14
  #define BTN_EN2                             39
  #define BTN_ENC                             15

#endif // RA_CONTROL_PANEL//RA_控制面板

#if ENABLED(RA_DISCO)
  // variables for which pins the TLC5947 is using//TLC5947使用引脚的变量
  #define TLC_CLOCK_PIN                       25
  #define TLC_BLANK_PIN                       23
  #define TLC_XLAT_PIN                        22
  #define TLC_DATA_PIN                        24

  // We also need to define pin to port number mapping for the 2560 to match the pins listed above.//我们还需要为2560定义管脚到端口号映射，以匹配上面列出的管脚。
  // If you change the TLC pins, update this as well per the 2560 datasheet! This currently only works with the RA Board.//如果更改TLC引脚，请根据2560数据表进行更新！这目前仅适用于RA板。
  #define TLC_CLOCK_BIT                        3
  #define TLC_CLOCK_PORT    &PORTA

  #define TLC_BLANK_BIT                        1
  #define TLC_BLANK_PORT    &PORTA

  #define TLC_DATA_BIT                         2
  #define TLC_DATA_PORT     &PORTA

  #define TLC_XLAT_BIT                         0
  #define TLC_XLAT_PORT     &PORTA

  // Change this to match your situation. Lots of TLCs takes up the arduino SRAM very quickly, so be careful//更改此选项以符合您的情况。很多TLC占用arduino SRAM的速度非常快，因此请小心
  // Leave it at at least 1 if you have enabled RA_LIGHTING//如果已启用RA_照明，则至少保留1
  // The number of TLC5947 boards chained together for use with the animation, additional ones will repeat the animation on them, but are not individually addressable and mimic those before them. You can leave the default at 2 even if you only have 1 TLC5947 module.//TLC5947板的数量链接在一起用于动画，其他板将在其上重复动画，但不能单独寻址并模仿之前的板。即使只有1个TLC5947模块，也可以将默认值保留为2。
  #define NUM_TLCS                             2

  // These TRANS_ARRAY values let you change the order the LEDs on the lighting modules will animate for chase functions.//这些TRANS_阵列值允许您更改照明模块上的LED为chase功能设置动画的顺序。
  // Modify them according to your specific situation.//根据您的具体情况进行修改。
  // NOTE: the array should be 8 long for every TLC you have. These defaults assume (2) TLCs.//注意：每个TLC的数组长度应为8。这些默认值假定为（2）个TLC。
  #define TRANS_ARRAY { 0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8 }    // forward//前进
  //#define TRANS_ARRAY { 7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15 }  // backward//#向后定义TRANS_数组{7,6,5,4,3,2,1,0,8,9,10,11,12,13,14,15}//
#endif // RA_DISCO//拉乌迪斯科舞厅
