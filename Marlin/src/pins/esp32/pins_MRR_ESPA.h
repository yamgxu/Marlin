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

////
// Disable I2S stepper stream//禁用I2S步进器流
////
#undef I2S_STEPPER_STREAM
#undef I2S_WS
#undef I2S_BCK
#undef I2S_DATA

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            34
#define Y_STOP_PIN                            35
#define Z_STOP_PIN                            15

////
// Steppers//踏步机
////
#define X_STEP_PIN                            27
#define X_DIR_PIN                             26
#define X_ENABLE_PIN                          25
//#define X_CS_PIN                            21//#定义X_CS_引脚21

#define Y_STEP_PIN                            33
#define Y_DIR_PIN                             32
#define Y_ENABLE_PIN                X_ENABLE_PIN
//#define Y_CS_PIN                            22//#定义Y_CS_引脚22

#define Z_STEP_PIN                            14
#define Z_DIR_PIN                             12
#define Z_ENABLE_PIN                X_ENABLE_PIN
//#define Z_CS_PIN                             5  // SS_PIN//#定义Z_CS_引脚5//SS_引脚

#define E0_STEP_PIN                           16
#define E0_DIR_PIN                            17
#define E0_ENABLE_PIN               X_ENABLE_PIN
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

////
// MicroSD card//MicroSD卡
////
#define SD_MOSI_PIN                           23
#define SD_MISO_PIN                           19
#define SD_SCK_PIN                            18
#define SDSS                                   5
#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers//SD卡与TMC SPI驱动程序共享SPI

// Hardware serial pins//硬件串行引脚
// Add the following to Configuration.h or Configuration_adv.h to assign//将以下内容添加到Configuration.h或Configuration\u adv.h以分配
// specific pins to hardware Serial1.//硬件串行1的特定引脚。
// Note: Serial2 can be defined using HARDWARE_SERIAL2_RX and HARDWARE_SERIAL2_TX but//注：可以使用硬件\u Serial2\u RX和硬件\u Serial2\u TX定义Serial2，但
// MRR ESPA does not have enough spare pins for such reassignment.//MRR ESPA没有足够的备用引脚用于此类重新分配。
//#define HARDWARE_SERIAL1_RX                 21//#定义硬件\u串行1\u RX 21
//#define HARDWARE_SERIAL1_TX                 22//#定义硬件\u SERIAL1\u TX 22
