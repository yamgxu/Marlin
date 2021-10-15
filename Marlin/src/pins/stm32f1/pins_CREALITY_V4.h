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
 * Creality 4.2.x (STM32F103RET6) board pin assignments
 */

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "Creality V4 only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "Creality V4"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Ender 3 V2"
#endif

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
// Servos//伺服
////
#ifndef SERVO0_PIN
  #ifndef HAS_PIN_27_BOARD
    #define SERVO0_PIN                      PB0   // BLTouch OUT//BLTouch OUT
  #else
    #define SERVO0_PIN                      PC6
  #endif
#endif

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PA5
#define Y_STOP_PIN                          PA6
#define Z_STOP_PIN                          PA7

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB1   // BLTouch IN//接触
#endif

////
// Filament Runout Sensor//灯丝偏移传感器
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // "Pulled-high"//“拉高”
#endif

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PC3
#ifndef X_STEP_PIN
  #define X_STEP_PIN                        PC2
#endif
#ifndef X_DIR_PIN
  #define X_DIR_PIN                         PB9
#endif

#define Y_ENABLE_PIN                        PC3
#ifndef Y_STEP_PIN
  #define Y_STEP_PIN                        PB8
#endif
#ifndef Y_DIR_PIN
  #define Y_DIR_PIN                         PB7
#endif

#define Z_ENABLE_PIN                        PC3
#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                        PB6
#endif
#ifndef Z_DIR_PIN
  #define Z_DIR_PIN                         PB5
#endif

#define E0_ENABLE_PIN                       PC3
#ifndef E0_STEP_PIN
  #define E0_STEP_PIN                       PB4
#endif
#ifndef E0_DIR_PIN
  #define E0_DIR_PIN                        PB3
#endif

////
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role//从JTAG NRST角色中释放PB4（Y_启用_引脚）
////
#define DISABLE_DEBUG

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PC5   // TH1//TH1
#define TEMP_BED_PIN                        PC4   // TB1//TB1

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PA1   // HEATER1//加热器1
#define HEATER_BED_PIN                      PA2   // HOT BED//热床

#ifndef FAN_PIN
  #define FAN_PIN                           PA0   // FAN//扇子
#endif
#if PIN_EXISTS(FAN)
  #define FAN_SOFT_PWM
#endif

////
// SD Card//SD卡
////
#define SD_DETECT_PIN                       PC7
#define SDCARD_CONNECTION                ONBOARD
#define ONBOARD_SPI_DEVICE                     1
#define ONBOARD_SD_CS_PIN                   PA4   // SDSS//SDS
#define SDIO_SUPPORT
#define NO_SD_HOST_DRIVE                          // This board's SD is only seen by the printer//此电路板的SD仅由打印机看到

#if ENABLED(CR10_STOCKDISPLAY) && NONE(RET6_12864_LCD, VET6_12864_LCD)
  #error "Define RET6_12864_LCD or VET6_12864_LCD to select pins for CR10_STOCKDISPLAY with the Creality V4 controller."
#endif

#if ENABLED(RET6_12864_LCD)

  // RET6 12864 LCD//RET612864液晶显示器
  #define LCD_PINS_RS                       PB12
  #define LCD_PINS_ENABLE                   PB15
  #define LCD_PINS_D4                       PB13

  #define BTN_ENC                           PB2
  #define BTN_EN1                           PB10
  #define BTN_EN2                           PB14

  #ifndef HAS_PIN_27_BOARD
    #define BEEPER_PIN                      PC6
  #endif

#elif ENABLED(VET6_12864_LCD)

  // VET6 12864 LCD//VET612864液晶显示器
  #define LCD_PINS_RS                       PA4
  #define LCD_PINS_ENABLE                   PA7
  #define LCD_PINS_D4                       PA5

  #define BTN_ENC                           PC5
  #define BTN_EN1                           PB10
  #define BTN_EN2                           PA6

#elif ENABLED(DWIN_CREALITY_LCD)

  // RET6 DWIN ENCODER LCD//RET6 DWN编码器LCD
  #define BTN_ENC                           PB14
  #define BTN_EN1                           PB15
  #define BTN_EN2                           PB12

  //#define LCD_LED_PIN                     PB2//#定义LCD_LED_引脚PB2
  #ifndef BEEPER_PIN
    #define BEEPER_PIN                      PB13
    #undef SPEAKER
  #endif

#elif ENABLED(DWIN_VET6_CREALITY_LCD)

  // VET6 DWIN ENCODER LCD//VET6 DWN编码器液晶显示器
  #define BTN_ENC                           PA6
  #define BTN_EN1                           PA7
  #define BTN_EN2                           PA4

  #define BEEPER_PIN                        PA5

#endif
