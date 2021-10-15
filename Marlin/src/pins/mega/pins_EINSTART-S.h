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
 * Einstart-S pin assignments
 * PCB Silkscreen: 3DPrinterCon_v3.5
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME "Einstart-S"

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            44
#define Y_STOP_PIN                            43
#define Z_STOP_PIN                            42

////
// Steppers//踏步机
////
#define X_STEP_PIN                            76
#define X_DIR_PIN                             75
#define X_ENABLE_PIN                          73

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             32
#define Y_ENABLE_PIN                          72

#define Z_STEP_PIN                            34
#define Z_DIR_PIN                             35
#define Z_ENABLE_PIN                          33

#define E0_STEP_PIN                           36
#define E0_DIR_PIN                            37
#define E0_ENABLE_PIN                         30

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             0  // Analog Input//模拟输入
#define TEMP_BED_PIN                           1  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          83
#define HEATER_BED_PIN                        38

#define FAN_PIN                               82

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  53
#define LED_PIN                                4

////////////////////////////////////////////////////
// LCDs and Controllers ////液晶显示器和控制器//
////////////////////////////////////////////////////

////
// LCD Display output pins//液晶显示器输出引脚
////

// Requires #define U8GLIB_SH1106_EINSTART in Configuration.h//需要#在Configuration.h中定义U8GLIB_SH1106_einnstart
// u8glib constructor//u8glib构造函数
// U8GLIB_SH1106_128X64 u8g(DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, LCD_PINS_DC, LCD_PINS_RS);//U8GLIB_SH1106_128X64 u8g（DOGLCD_SCK、DOGLCD_MOSI、DOGLCD_CS、LCD_引脚_DC、LCD_引脚_RS）；

#define LCD_PINS_DC                           78
#define LCD_PINS_RS                           79
// DOGM SPI LCD Support//DOGM SPI LCD支持
#define DOGLCD_CS                              3
#define DOGLCD_MOSI                            2
#define DOGLCD_SCK                             5
#define DOGLCD_A0                              2

////
// LCD Display input pins//液晶显示器输入引脚
////
#define BTN_UP                                25
#define BTN_DWN                               26
#define BTN_LFT                               27
#define BTN_RT                                28

// 'OK' button//“确定”按钮
#define BTN_ENC                               29

// Set Kill to right arrow, same as RIGID_PANEL//将Kill设置为右箭头，与刚性面板相同
#define KILL_PIN                              28
