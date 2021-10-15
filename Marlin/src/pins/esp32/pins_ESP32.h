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
 * Espressif ESP32 (Tensilica Xtensa LX6) pin assignments
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Espressif ESP32"

////
// I2S (steppers & other output-only pins)//I2S（步进器和其他仅输出引脚）
////
#define I2S_STEPPER_STREAM
#define I2S_WS                                25
#define I2S_BCK                               26
#define I2S_DATA                              27

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             34
#define Y_MIN_PIN                             35
#define Z_MIN_PIN                             15

////
// Steppers//踏步机
////
#define X_STEP_PIN                           128
#define X_DIR_PIN                            129
#define X_ENABLE_PIN                         130
//#define X_CS_PIN                             0//#定义X_CS_引脚0

#define Y_STEP_PIN                           131
#define Y_DIR_PIN                            132
#define Y_ENABLE_PIN                         133
//#define Y_CS_PIN                            13//#定义Y_CS_引脚13

#define Z_STEP_PIN                           134
#define Z_DIR_PIN                            135
#define Z_ENABLE_PIN                         136
//#define Z_CS_PIN                             5  // SS_PIN//#定义Z_CS_引脚5//SS_引脚

#define E0_STEP_PIN                          137
#define E0_DIR_PIN                           138
#define E0_ENABLE_PIN                        139
//#define E0_CS_PIN                           21//#定义E0_CS_引脚21

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            36  // Analog Input//模拟输入
#define TEMP_BED_PIN                          39  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           2
#define FAN_PIN                               13
#define HEATER_BED_PIN                         4

// SPI//SPI
#define SDSS                                   5
