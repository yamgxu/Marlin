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
#undef I2S_STEPPER_STREAM
#define I2S_WS                                -1
#define I2S_BCK                               -1
#define I2S_DATA                              -1
////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             18
#define Y_MIN_PIN                             17
#define Z_MIN_PIN                             16

////
// Steppers//踏步机
////
#define X_STEP_PIN                           14
#define X_DIR_PIN                            13
#define X_ENABLE_PIN                         11
//#define X_CS_PIN                             0//#定义X_CS_引脚0

#define Y_STEP_PIN                           9
#define Y_DIR_PIN                            8
#define Y_ENABLE_PIN                         X_ENABLE_PIN
//#define Y_CS_PIN                            13//#定义Y_CS_引脚13

#define Z_STEP_PIN                           4
#define Z_DIR_PIN                            1
#define Z_ENABLE_PIN                         X_ENABLE_PIN
//#define Z_CS_PIN                             5  // SS_PIN//#定义Z_CS_引脚5//SS_引脚

#define E0_STEP_PIN                          7
#define E0_DIR_PIN                           5
#define E0_ENABLE_PIN                        X_ENABLE_PIN
//#define E0_CS_PIN                           21//#定义E0_CS_引脚21

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            3  // Analog Input//模拟输入
#define TEMP_BED_PIN                          10  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           12
#define FAN_PIN                                0
#define HEATER_BED_PIN                         2

// SPI//SPI
#define SDSS                                   34
// TODO copied from stm32/inc/conditionals_adv.h see where that should be included//从stm32/inc/conditionals\u adv.h复制的TODO请参见应包含的位置
// The Sensitive Pins array is not optimizable//敏感管脚阵列不可优化
#define RUNTIME_ONLY_ANALOG_TO_DIGITAL
#define HARDWARE_SERIAL1_RX                   44
#define HARDWARE_SERIAL1_TX                   43
//#define TMC_BAUD_RATE 115200//#定义TMC_波特率115200
