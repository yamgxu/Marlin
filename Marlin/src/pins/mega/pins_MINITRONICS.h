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
 * Minitronics v1.0/1.1 pin assignments
 */

/**
 * Rev B   2 JAN 2017
 *
 *  Added pin definitions for M3, M4 & M5 spindle control commands
 */

#if NOT_TARGET(__AVR_ATmega1281__)
  #error "Oops! Select 'Minitronics' in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "Minitronics supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "Minitronics v1.0/1.1"
////
// Limit Switches//限位开关
////
#define X_MIN_PIN                              5
#define X_MAX_PIN                              2
#define Y_MIN_PIN                              2
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                              6
#define Z_MAX_PIN                             -1

////
// Steppers//踏步机
////
#define X_STEP_PIN                            48
#define X_DIR_PIN                             47
#define X_ENABLE_PIN                          49

#define Y_STEP_PIN                            39  // A6//A6
#define Y_DIR_PIN                             40  // A0//A0
#define Y_ENABLE_PIN                          38

#define Z_STEP_PIN                            42  // A2//A2
#define Z_DIR_PIN                             43  // A6//A6
#define Z_ENABLE_PIN                          41  // A1//A1

#define E0_STEP_PIN                           45
#define E0_DIR_PIN                            44
#define E0_ENABLE_PIN                         27

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            35
#define E1_ENABLE_PIN                         37

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             7  // Analog Input//模拟输入
#define TEMP_1_PIN                             6  // Analog Input//模拟输入
#define TEMP_BED_PIN                           6  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           7  // EXTRUDER 1//挤出机1
#define HEATER_1_PIN                           8  // EXTRUDER 2//挤出机2
#define HEATER_BED_PIN                         3  // BED//床

#ifndef FAN_PIN
  #define FAN_PIN                              9
#endif

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  16
#define LED_PIN                               46

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                            -1

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define LCD_PINS_RS                         15  // CS chip select /SS chip slave select//CS芯片选择/SS芯片从属选择
  #define LCD_PINS_ENABLE                     11  // SID (MOSI)//SID（MOSI）
  #define LCD_PINS_D4                         10  // SCK (CLK) clock//时钟

  #define BTN_EN1                             18
  #define BTN_EN2                             17
  #define BTN_ENC                             25

  #define SD_DETECT_PIN                       30

#else

  #define LCD_PINS_RS                         -1
  #define LCD_PINS_ENABLE                     -1

  // Buttons are directly attached using keypad//按钮使用键盘直接连接
  #define BTN_EN1                             -1
  #define BTN_EN2                             -1
  #define BTN_ENC                             -1

  #define SD_DETECT_PIN                       -1  // Minitronics doesn't use this//Minitronics不使用这个
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#if HAS_CUTTER                                    // assumes we're only doing CNC work (no 3D printing)//假设我们只做CNC工作（无3D打印）
  #undef HEATER_BED_PIN
  #undef TEMP_BED_PIN                             // need to free up some pins but also need to//需要释放一些引脚，但也需要
  #undef TEMP_0_PIN                               // re-assign them (to unused pins) because Marlin//重新分配它们（到未使用的管脚），因为Marlin
  #undef TEMP_1_PIN                               // requires the presence of certain pins or else it//需要存在某些引脚，否则
  #define HEATER_BED_PIN                       4  // won't compile//不会编译
  #define TEMP_BED_PIN                        50
  #define TEMP_0_PIN                          51
  #define SPINDLE_LASER_ENA_PIN               52  // using A6 because it already has a pullup//使用A6，因为它已经具有上拉功能
  #define SPINDLE_LASER_PWM_PIN                3  // WARNING - LED & resistor pull up to +12/+24V stepper voltage//警告-LED和电阻器拉高至+12/+24V步进电压
  #define SPINDLE_DIR_PIN                     53
#endif
