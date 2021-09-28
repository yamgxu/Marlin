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

//
// I2S (steppers & other output-only pins)
//
#undef I2S_STEPPER_STREAM
#define I2S_WS                                -1
#define I2S_BCK                               -1
#define I2S_DATA                              -1
//
// Limit Switches
//
#define X_MIN_PIN                             3
#define Y_MIN_PIN                             4
#define Z_MIN_PIN                             5

//
// Steppers
//
#define X_STEP_PIN                           6
#define X_DIR_PIN                            7
#define X_ENABLE_PIN                         2
//#define X_CS_PIN                             0

#define Y_STEP_PIN                           8
#define Y_DIR_PIN                            9
#define Y_ENABLE_PIN                         X_ENABLE_PIN
//#define Y_CS_PIN                            13

#define Z_STEP_PIN                           10
#define Z_DIR_PIN                            11
#define Z_ENABLE_PIN                         X_ENABLE_PIN
//#define Z_CS_PIN                             5  // SS_PIN

#define E0_STEP_PIN                          12
#define E0_DIR_PIN                           13
#define E0_ENABLE_PIN                        X_ENABLE_PIN
//#define E0_CS_PIN                           21

//
// Temperature Sensors
//
#define TEMP_0_PIN                            1  // Analog Input
#define TEMP_BED_PIN                          0  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           0
#define FAN_PIN                                0
#define HEATER_BED_PIN                         0

// SPI
#define SDSS                                   34
// TODO copied from stm32/inc/conditionals_adv.h see where that should be included
// The Sensitive Pins array is not optimizable
#define RUNTIME_ONLY_ANALOG_TO_DIGITAL
#define HARDWARE_SERIAL1_RX                   44
#define HARDWARE_SERIAL1_TX                   43
#define TMC_BAUD_RATE 115200