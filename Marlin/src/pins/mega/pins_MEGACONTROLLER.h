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
 * Mega controller pin assignments
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Mega Controller supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#include "env_validate.h"

#define BOARD_INFO_NAME "Mega Controller"

////
// Servos//伺服
////
#define SERVO0_PIN                            30
#define SERVO1_PIN                            31
#define SERVO2_PIN                            32
#define SERVO3_PIN                            33

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             43
#define X_MAX_PIN                             42
#define Y_MIN_PIN                             38
#define Y_MAX_PIN                             41
#define Z_MIN_PIN                             40
#define Z_MAX_PIN                             37

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     37
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            62  // A8//A8
#define X_DIR_PIN                             63  // A9//A9
#define X_ENABLE_PIN                          61  // A7//A7

#define Y_STEP_PIN                            65  // A11//A11
#define Y_DIR_PIN                             66  // A12//A12
#define Y_ENABLE_PIN                          64  // A10//A10

#define Z_STEP_PIN                            68  // A14//A14
#define Z_DIR_PIN                             69  // A15//A15
#define Z_ENABLE_PIN                          67  // A13//A13

#define E0_STEP_PIN                           23
#define E0_DIR_PIN                            24
#define E0_ENABLE_PIN                         22

#define E1_STEP_PIN                           26
#define E1_DIR_PIN                            27
#define E1_ENABLE_PIN                         25

////
// Temperature Sensors//温度传感器
////
#if TEMP_SENSOR_0 == -1
  #define TEMP_0_PIN                           4  // Analog Input//模拟输入
#else
  #define TEMP_0_PIN                           0  // Analog Input//模拟输入
#endif

#if TEMP_SENSOR_1 == -1
  #define TEMP_1_PIN                           5  // Analog Input//模拟输入
#else
  #define TEMP_1_PIN                           2  // Analog Input//模拟输入
#endif

#define TEMP_2_PIN                             3  // Analog Input//模拟输入

#if TEMP_SENSOR_BED == -1
  #define TEMP_BED_PIN                         6  // Analog Input//模拟输入
#else
  #define TEMP_BED_PIN                         1  // Analog Input//模拟输入
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          29
#define HEATER_1_PIN                          34
#define HEATER_BED_PIN                        28

#ifndef FAN_PIN
  #define FAN_PIN                             39
#endif
#define FAN1_PIN                              35
#define FAN2_PIN                              36

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN            FAN2_PIN
#endif

#define FAN_SOFT_PWM

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  53
#define LED_PIN                               13

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                       2
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#if ENABLED(MINIPANEL)

  #define BEEPER_PIN                          46
  // Pins for DOGM SPI LCD Support//用于DOGM SPI LCD支持的引脚
  #define DOGLCD_A0                           47
  #define DOGLCD_CS                           45
  #define LCD_BACKLIGHT_PIN                   44  // backlight LED on PA3//PA3上的背光LED

  #define KILL_PIN                            12
  // GLCD features//GLCD功能
  // Uncomment screen orientation//取消注释屏幕方向
  //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
  //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
  //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270

  #define BTN_EN1                             48
  #define BTN_EN2                             11
  #define BTN_ENC                             10

  #define SD_DETECT_PIN                       49

#endif // MINIPANEL//微型面板

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#define SPINDLE_LASER_PWM_PIN                  6  // Hardware PWM//硬件脉宽调制
#define SPINDLE_LASER_ENA_PIN                  7  // Pullup!//拉起！
#define SPINDLE_DIR_PIN                        8
