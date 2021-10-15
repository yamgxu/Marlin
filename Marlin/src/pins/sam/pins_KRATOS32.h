/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * KRATOS32
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "K.3D KRATOS32"

////
// EEPROM//电可擦可编程只读存储器
////
#if EITHER(NO_EEPROM_SELECTED, I2C_EEPROM)
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE             0x1F400  //   16KB//16KB
#endif

////
// Servos//伺服
////
#define SERVO0_PIN                             6
#define SERVO1_PIN                             5
#define SERVO2_PIN                            39
#define SERVO3_PIN                            40  // CAMERA_PIN (extended to the top of the LCD module)//摄像头插脚（延伸至LCD模块顶部）
#define SERVO4_PIN                            45  // FIL_RUNOUT_PIN//薄膜跳动销

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             28
#define X_MAX_PIN                             34
#define Y_MIN_PIN                             30
#define Y_MAX_PIN                             36
#define Z_MIN_PIN                             32
#define Z_MAX_PIN                             38

////
// Steppers//踏步机
////
#define X_STEP_PIN                            24
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          26
#ifndef X_CS_PIN
  #define X_CS_PIN                            25
#endif

#define Y_STEP_PIN                            17
#define Y_DIR_PIN                             16
#define Y_ENABLE_PIN                          22
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            27
#endif

#define Z_STEP_PIN                             2
#define Z_DIR_PIN                              3
#define Z_ENABLE_PIN                          15
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            29
#endif

#define E0_STEP_PIN                           61
#define E0_DIR_PIN                            60
#define E0_ENABLE_PIN                         62
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           31
#endif

#define E1_STEP_PIN                           64
#define E1_DIR_PIN                            63
#define E1_ENABLE_PIN                         65
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           37
#endif

#define E2_STEP_PIN                           68
#define E2_DIR_PIN                            67
#define E2_ENABLE_PIN                         69
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           35
#endif

#define E3_STEP_PIN                           51
#define E3_DIR_PIN                            53
#define E3_ENABLE_PIN                         49
#ifndef E3_CS_PIN
  #define E3_CS_PIN                           33
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             0  // Analog Input//模拟输入
#define TEMP_1_PIN                             1  // Analog Input//模拟输入
#define TEMP_2_PIN                             2  // Analog Input//模拟输入
#define TEMP_3_PIN                             3  // Analog Input//模拟输入
#define TEMP_BED_PIN                           4  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          13
#define HEATER_1_PIN                          12
#define HEATER_2_PIN                          11
#define HEATER_3_PIN                          10
#define HEATER_BED_PIN                         7  // BED//床

#ifndef FAN_PIN
  #define FAN_PIN                              9
#endif
#define FAN1_PIN                               8

////
// Misc. Functions//杂项。功能
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      45  // SERVO4_PIN//伺服4_销
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           59
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD

  #define BTN_EN1                             48
  #define BTN_EN2                             50
  #define BTN_ENC                             46

  #define SDSS                                 4
  #define SD_DETECT_PIN                       14

  #define BEEPER_PIN                          41
  #define KILL_PIN                            66

  #if IS_RRD_FG_SC

    #define LCD_PINS_RS                       42
    #define LCD_PINS_ENABLE                   43
    #define LCD_PINS_D4                       44

    #define BTN_BACK                          52

  #elif ENABLED(K3D_242_OLED_CONTROLLER)

    #define LCD_PINS_DC                       44
    #define LCD_PINS_RS                       42
    #define DOGLCD_CS                         52
    #define DOGLCD_MOSI                       43
    #define DOGLCD_SCK                        47
    #define DOGLCD_A0                LCD_PINS_DC

  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗
