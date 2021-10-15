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
  * 2017 Victor Perez Marlin for stm32f1 test
  * 2018 Modified by Pablo Crespo for Morpheus Board (https://github.com/pscrespo/Morpheus-STM32)
  */

/**
 * MORPHEUS Board pin assignments
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Bluepill based board"

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PB14
#define Y_STOP_PIN                          PB13
#define Z_STOP_PIN                          PB12

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB9
#endif

////
// Steppers//踏步机
////
// X & Y enable are the same//X&Y启用是相同的
#define X_STEP_PIN                          PB7
#define X_DIR_PIN                           PB6
#define X_ENABLE_PIN                        PB8

#define Y_STEP_PIN                          PB5
#define Y_DIR_PIN                           PB4
#define Y_ENABLE_PIN                        PB8

#define Z_STEP_PIN                          PA15
#define Z_DIR_PIN                           PA10
#define Z_ENABLE_PIN                        PB3

#define E0_STEP_PIN                         PA8
#define E0_DIR_PIN                          PB15
#define E0_ENABLE_PIN                       PA9

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PB1   // Analog Input (HOTEND thermistor)//模拟输入（热端热敏电阻）
#define TEMP_BED_PIN                        PB0   // Analog Input (BED thermistor)//模拟输入（床用热敏电阻）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PA2   // HOTEND MOSFET//热端MOSFET
#define HEATER_BED_PIN                      PA0   // BED MOSFET//床层MOSFET

#define FAN_PIN                             PA1   // FAN1 header on board - PRINT FAN//FAN1板上页眉-打印风扇

////
// Misc.//杂项。
////
#define LED_PIN                             PC13
#define SDSS                                PA3
#define TFTGLCD_CS                          PA4
#define SD_DETECT_PIN                       PC14
