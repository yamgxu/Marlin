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
 *  Rev B  2 JUN 2017
 *
 *  Converted to Arduino pin numbering
 */

/**
 *  There are two Arduino IDE extensions that are compatible with this board
 *  and with the mainstream Marlin software.
 *
 *  Teensyduino - https://www.pjrc.com/teensy/teensyduino.html
 *    Select Teensy++ 2.0 in Arduino IDE from the 'Tools > Board' menu
 *
 *    Installation instructions are at the above URL.  Don't bother loading the
 *    libraries - they are not used with the Marlin software.
 *
 *  Printrboard - https://github.com/scwimbush/Printrboard-HID-Arduino-IDE-Support
 *
 *    Installation:
 *
 *       1. Go to the above URL, click on the "Clone or Download" button and then
 *          click on "Download ZIP" button.
 *       2. Unzip the file, find the "printrboard" directory and then copy it to the
 *          hardware directory in Arduino.  The Arduino hardware directory will probably
 *          be located in a path similar to this: C:\Program Files (x86)\Arduino\hardware.
 *       3. Restart Arduino.
 *       4. Select "Printrboard" from the 'Tools > Board' menu.
 *
 *  Teensyduino is the most popular option. Printrboard is used if your board doesn't have
 *  the Teensyduino bootloader on it.
 */

/**
 *  To burn the bootloader that comes with Printrboard:
 *
 *   1. Connect your programmer to the board.
 *   2. In the Arduino IDE select "Printrboard" and then select the programmer.
 *   3. In the Arduino IDE click on "burn bootloader". Don't worry about the "verify failed at 1F000" error message.
 *   4. The programmer is no longer needed. Remove it.
 */

#include "env_validate.h"

#define BOARD_INFO_NAME         "SAV MkI"
#define DEFAULT_MACHINE_NAME    BOARD_INFO_NAME
#define DEFAULT_SOURCE_CODE_URL "tinyurl.com/onru38b"

////
// Servos//伺服
////
#define SERVO0_PIN                            39  // F1  In teensy's pin definition for pinMode (in servo.cpp)//Tiensy针模式的针定义中的F1（在servo.cpp中）

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            25  // B5//B5
#define Y_STOP_PIN                            26  // B6//B6
//#define Z_STOP_PIN                          27  // B7//#定义Z_停止针27//B7
#define Z_STOP_PIN                            36  // E4 For inductive sensor.//E4用于感应式传感器。
//#define E_STOP_PIN                          36  // E4//#定义E_停止针36//E4

////
// Steppers//踏步机
////
#define X_STEP_PIN                            28  // A0//A0
#define X_DIR_PIN                             29  // A1//A1
#define X_ENABLE_PIN                          19  // E7//E7

#define Y_STEP_PIN                            30  // A2//A2
#define Y_DIR_PIN                             31  // A3//A3
#define Y_ENABLE_PIN                          18  // E6//E6

#define Z_STEP_PIN                            32  // A4//A4
#define Z_DIR_PIN                             33  // A5//A5
#define Z_ENABLE_PIN                          17  // C7//C7

#define E0_STEP_PIN                           34  // A6//A6
#define E0_DIR_PIN                            35  // A7//A7
#define E0_ENABLE_PIN                         13  // C3//C3

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             7  // F7  Analog Input (Extruder)//F7模拟输入（挤出机）
#define TEMP_BED_PIN                           6  // F6  Analog Input (Bed)//F6模拟输入（床）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          15  // C5 PWM3B - Extruder//C5 PWM3B-挤出机
#define HEATER_BED_PIN                        14  // C4 PWM3C - Bed//C4 PWM3C-床层

#ifndef FAN_PIN
  #define FAN_PIN                             16  // C6 PWM3A//C6 PWM3A
#endif

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  20  // B0//B0

// Extension header pin mapping//扩展头引脚映射
// ----------------------------// ----------------------------
//  SCL (I2C)-D0    A0 (An), IO//SCL（I2C）-D0 A0（An），IO
//  SDA (I2C)-D1    A1 (An), IO//SDA（I2C）-D1 A1（An），IO
//  RX1-D2          A2 (An), IO//RX1-D2 A2（安），IO
//  TX1-D3          A3 (An), IO//TX1-D3 A3（安），IO
//  PWM-D24         A4 (An), IO//PWM-D24 A4（安），IO
//  5V              GND//5V接地
//  12V             GND//12V接地
#define EXT_AUX_SCL_D0                         0  // D0  PWM0B//D0 PWM0B
#define EXT_AUX_SDA_D1                         1  // D1//D1
#define EXT_AUX_RX1_D2                         2  // D2//D2
#define EXT_AUX_TX1_D3                         3  // D3//D3
#define EXT_AUX_PWM_D24                       24  // B4  PWM2A//B4 PWM2A
#define EXT_AUX_A0                             0  // F0  Analog Input//模拟输入
#define EXT_AUX_A0_IO                         38  // F0  Digital IO//F0数字IO
#define EXT_AUX_A1                             1  // F1  Analog Input//F1模拟输入
#define EXT_AUX_A1_IO                         39  // F1  Digital IO//F1数字IO
#define EXT_AUX_A2                             2  // F2  Analog Input//模拟输入
#define EXT_AUX_A2_IO                         40  // F2  Digital IO//F2数字IO
#define EXT_AUX_A3                             3  // F3  Analog Input//F3模拟输入
#define EXT_AUX_A3_IO                         41  // F3  Digital IO//F3数字IO
#define EXT_AUX_A4                             4  // F4  Analog Input//F4模拟输入
#define EXT_AUX_A4_IO                         42  // F4  Digital IO//F4数字IO

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                            -1
#define LCD_PINS_RS                           -1
#define LCD_PINS_ENABLE                       -1

#if ENABLED(SAV_3DLCD)
  // For LCD SHIFT register LCD//用于LCD移位寄存器LCD
  #define SR_DATA_PIN             EXT_AUX_SDA_D1
  #define SR_CLK_PIN              EXT_AUX_SCL_D0
#endif

#if EITHER(SAV_3DLCD, SAV_3DGLCD)

  #define BTN_EN1                  EXT_AUX_A1_IO
  #define BTN_EN2                  EXT_AUX_A0_IO
  #define BTN_ENC                EXT_AUX_PWM_D24

  #define KILL_PIN                 EXT_AUX_A2_IO
  #define HOME_PIN                 EXT_AUX_A4_IO

#else                                             // Use the expansion header for spindle control//使用扩展收割台进行主轴控制

  ////
  // M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
  ////
  #define SPINDLE_LASER_PWM_PIN               24  // B4  PWM2A//B4 PWM2A
  #define SPINDLE_LASER_ENA_PIN               39  // F1  Pin should have a pullup!//F1引脚应该有一个拉起！
  #define SPINDLE_DIR_PIN                     40  // F2//F2

  #define CASE_LIGHT_PIN                       0  // D0  PWM0B//D0 PWM0B

#endif
