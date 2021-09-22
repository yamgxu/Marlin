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
 * MRR ESPA pin assignments
 * MRR ESPA is a 3D printer control board based on the ESP32 microcontroller.
 * Supports 4 stepper drivers, heated bed, single hotend.
 */

#include "env_validate.h"

#if EXTRUDERS > 1 || E_STEPPERS > 1
#error "MRR ESPA only supports one E Stepper. Comment out this line to continue."
#elif HOTENDS > 1
#error "MRR ESPA only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME       "MRR ESPA"
#define BOARD_WEBSITE_URL     "github.com/maplerainresearch/MRR_ESPA"
#define DEFAULT_MACHINE_NAME  BOARD_INFO_NAME

//
// Disable I2S stepper stream
//
#undef I2S_STEPPER_STREAM
#undef I2S_WS
#undef I2S_BCK
#undef I2S_DATA

//
// Limit Switches
//
#define X_STOP_PIN                            14
#define Y_STOP_PIN                            13
#define Z_STOP_PIN                            3

//
// Steppers
//
#define X_STEP_PIN                            4
#define X_DIR_PIN                             5
#define X_ENABLE_PIN                          6
//#define X_CS_PIN                            21

#define Y_STEP_PIN                            7
#define Y_DIR_PIN                             8
#define Y_ENABLE_PIN                X_ENABLE_PIN
//#define Y_CS_PIN                            22

#define Z_STEP_PIN                            9
#define Z_DIR_PIN                             10
#define Z_ENABLE_PIN                X_ENABLE_PIN
//#define Z_CS_PIN                             5  // SS_PIN

#define E0_STEP_PIN                           11
#define E0_DIR_PIN                            12
#define E0_ENABLE_PIN               X_ENABLE_PIN
//#define E0_CS_PIN                           21

//
// Temperature Sensors
//
#define TEMP_0_PIN                            1  // Analog Input
//#define TEMP_BED_PIN                          2  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           15
//#define FAN_PIN                                16
//#define HEATER_BED_PIN                         17

//
// MicroSD card
//
#define SD_MOSI_PIN                           35
#define SD_MISO_PIN                           37
#define SD_SCK_PIN                            36
#define SD_SS_PIN                             34
//#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers

// Hardware serial pins
// Add the following to Configuration.h or Configuration_adv.h to assign
// specific pins to hardware Serial1.
// Note: Serial2 can be defined using HARDWARE_SERIAL2_RX and HARDWARE_SERIAL2_TX but
// MRR ESPA does not have enough spare pins for such reassignment.
//#define HARDWARE_SERIAL1_RX                 21
//#define HARDWARE_SERIAL1_TX                 22
