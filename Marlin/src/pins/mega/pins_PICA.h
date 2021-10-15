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
 * Arduino Mega with PICA pin assignments
 *
 * PICA is Power, Interface, and Control Adapter and is open source hardware.
 * See https://github.com/mjrice/PICA for schematics etc.
 *
 * Applies to PICA, PICA_REVB
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "PICA"
#endif

/*
// Note that these are the "pins" that correspond to the analog inputs on the arduino mega.//请注意，这些是与arduino mega上的模拟输入相对应的“引脚”。
// These are not the same as the physical pin numbers//这些与物理pin码不同
  AD0 = 54;   AD1 = 55;   AD2 = 56;   AD3 = 57;
  AD4 = 58;   AD5 = 59;   AD6 = 60;   AD7 = 61;
  AD8 = 62;   AD9 = 63;   AD10 = 64;  AD11 = 65;
  AD12 = 66;  AD13 = 67;  AD14 = 68;  AD15 = 69;
*/

////
// Servos//伺服
////
#define SERVO0_PIN                             3
#define SERVO1_PIN                             4
#define SERVO2_PIN                             5
////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             14
#define X_MAX_PIN                             15
#define Y_MIN_PIN                             16
#define Y_MAX_PIN                             17
#define Z_MIN_PIN                             23
#define Z_MAX_PIN                             22

////
// Steppers//踏步机
////
#define X_STEP_PIN                            55
#define X_DIR_PIN                             54
#define X_ENABLE_PIN                          60

#define Y_STEP_PIN                            57
#define Y_DIR_PIN                             56
#define Y_ENABLE_PIN                          61

#define Z_STEP_PIN                            59
#define Z_DIR_PIN                             58
#define Z_ENABLE_PIN                          62

#define E0_STEP_PIN                           67
#define E0_DIR_PIN                            24
#define E0_ENABLE_PIN                         26

#define E1_STEP_PIN                           68
#define E1_DIR_PIN                            28
#define E1_ENABLE_PIN                         27

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             9  // Analog Input//模拟输入
#define TEMP_1_PIN                            10
#define TEMP_BED_PIN                          10
#define TEMP_2_PIN                            11
#define TEMP_3_PIN                            12

////
// Heaters / Fans//加热器/风扇
////
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                        10  // E0//E0
#endif
#ifndef HEATER_1_PIN
  #define HEATER_1_PIN                         2  // E1//E1
#endif
#define HEATER_BED_PIN                         8  // HEAT-BED//热床

#ifndef FAN_PIN
  #define FAN_PIN                              9
#endif
#ifndef FAN_2_PIN
  #define FAN_2_PIN                            7
#endif

#define SDPOWER_PIN                           -1
#define LED_PIN                               -1
#define PS_ON_PIN                             -1
#define KILL_PIN                              -1

#define SSR_PIN                                6

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS_PIN                      66  // Don't use 53 if using Display/SD card//如果使用显示卡/SD卡，请不要使用53
#else
  #define MAX6675_SS_PIN                      66  // Don't use 49 (SD_DETECT_PIN)//不要使用49（SD_DETECT_引脚）
#endif

////
// SD Support//SD支持
////
#define SD_DETECT_PIN                         49
#define SDSS                                  53

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                            29

#if HAS_WIRED_LCD
  #define LCD_PINS_RS                         33
  #define LCD_PINS_ENABLE                     30
  #define LCD_PINS_D4                         35
  #define LCD_PINS_D5                         32
  #define LCD_PINS_D6                         37
  #define LCD_PINS_D7                         36

  #define BTN_EN1                             47
  #define BTN_EN2                             48
  #define BTN_ENC                             31

  #define LCD_SDSS                            53
#endif
