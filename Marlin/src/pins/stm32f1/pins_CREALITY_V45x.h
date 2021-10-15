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
 * Creality v4.5.2 and v4.5.3 (STM32F103RET6) board pin assignments
 */

#include "env_validate.h"

#define DEFAULT_MACHINE_NAME "Creality3D"

////
// Release PB4 (Z_STEP_PIN) from JTAG NRST role//从JTAG NRST角色中释放PB4（Z_步骤_引脚）
////
#define DISABLE_DEBUG

#define BOARD_NO_NATIVE_USB

////
// EEPROM//电可擦可编程只读存储器
////
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0//I2C-0上的EEPROM
  //#define SDCARD_EEPROM_EMULATION//#定义SD卡\u EEPROM\u仿真
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PA11
  #define IIC_EEPROM_SCL                    PA12
  #define MARLIN_EEPROM_SIZE               0x800  // 2Kb (24C16)//2Kb（24C16）
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE               0x800  // 2Kb//2Kb
#endif

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PC4
#define Y_STOP_PIN                          PC5
#define Z_STOP_PIN                          PA4

#define FIL_RUNOUT_PIN                      PA7

////
// Probe//探测
////
#define PROBE_TARE_PIN                      PA5

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PC3
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB7

#define Y_ENABLE_PIN                        PC3
#define Y_STEP_PIN                          PB6
#define Y_DIR_PIN                           PB5

#define Z_ENABLE_PIN                        PC3
#define Z_STEP_PIN                          PB4
#define Z_DIR_PIN                           PB3

#define E0_ENABLE_PIN                       PC3
#define E0_STEP_PIN                         PC2
#define E0_DIR_PIN                          PB9

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PB1   // TH1//TH1
#define TEMP_BED_PIN                        PB0   // TB1//TB1

////
// Heaters / Fans//加热器/风扇
////

#define FAN_SOFT_PWM

////
// SD Card//SD卡
////
#define SD_DETECT_PIN                       PC7
#define NO_SD_HOST_DRIVE                          // SD is only seen by the printer//只有打印机才能看到SD

#define SDIO_SUPPORT                              // Extra added by Creality//额外增加的费用
#define SDIO_CLOCK                       6000000  // In original source code overridden by Creality in sdio.h//在sdio.h中被Creality覆盖的原始源代码中

////
// Misc. Functions//杂项。功能
////
#define CASE_LIGHT_PIN                      PA6
