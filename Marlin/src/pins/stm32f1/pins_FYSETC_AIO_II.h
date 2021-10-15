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

#include "env_validate.h"

#define BOARD_INFO_NAME   "FYSETC AIO II"
#define BOARD_WEBSITE_URL "fysetc.com"

#define BOARD_NO_NATIVE_USB

#define DISABLE_JTAG

#define pins_v2_20190128                          // new pins define//新引脚定义

// Ignore temp readings during development.//在开发过程中忽略温度读数。
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000//#定义2000年的假温度宽限期

////
// Flash EEPROM Emulation//flasheeprom仿真
////
#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB//2KB
#endif

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PA1
#define Y_STOP_PIN                          PA0
#define Z_STOP_PIN                          PB14

////
// Filament runout//灯丝跳动
////
#ifdef pins_v2_20190128
  #define FIL_RUNOUT_PIN                    PB15
#else
  #define FIL_RUNOUT_PIN                    PB5
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PB8
#define X_DIR_PIN                           PB9
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PB2
#ifdef pins_v2_20190128
  #define Y_DIR_PIN                         PB3
#else
  #define Y_DIR_PIN                         PB0
#endif
#define Y_ENABLE_PIN                        PB1

#define Z_STEP_PIN                          PC0
#define Z_DIR_PIN                           PC1
#define Z_ENABLE_PIN                        PC2

#define E0_STEP_PIN                         PC15
#define E0_DIR_PIN                          PC14
#define E0_ENABLE_PIN                       PC13

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   */

  // Hardware serial with switch//硬件串行开关
  #define X_HARDWARE_SERIAL  MSerial2
  #define Y_HARDWARE_SERIAL  MSerial2
  #define Z_HARDWARE_SERIAL  MSerial2
  #define E0_HARDWARE_SERIAL MSerial2

  // Default TMC slave addresses//默认TMC从机地址
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS  0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS  1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS  2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS 3
  #endif

  // The 4xTMC2209 module doesn't have a serial multiplexer and//4xTMC2209模块没有串行多路复用器和
  // needs to set *_SLAVE_ADDRESS in Configuration_adv.h for X,Y,Z,E0//需要在X、Y、Z、E0的配置adv.h中设置*从机地址
  #if HAS_DRIVER(TMC2208)
    #define TMC_SERIAL_MULTIPLEXER
    #define SERIAL_MUL_PIN1                 PB13
    #define SERIAL_MUL_PIN2                 PB12
  #endif

  // Reduce baud rate to improve software serial reliability//降低波特率以提高软件串行可靠性
  #define TMC_BAUD_RATE                    19200
#endif

////
// Stepper current PWM//步进电流脉宽调制
////
#ifndef MOTOR_CURRENT_PWM_RANGE
  #define MOTOR_CURRENT_PWM_RANGE           1500  // origin:2000//来源：2000年
#endif
#define DEFAULT_PWM_MOTOR_CURRENT { 500, 500, 400 } // origin: {1300,1300,1250}//来源：{13001250}

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PC7
#define HEATER_BED_PIN                      PC6
#ifndef FAN_PIN
  #define FAN_PIN                           PC8
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_BED_PIN                        PC5   // Analog Input//模拟输入
#define TEMP_0_PIN                          PC4   // Analog Input//模拟输入

////
// Misc. Functions//杂项。功能
////
#define SDSS                                PA4

////
// LCD Pins//LCD引脚
////
#if HAS_WIRED_LCD

  #define BEEPER_PIN                        PC9

  #if HAS_MARLINUI_U8GLIB

    #define DOGLCD_A0                       PA15
    #ifdef pins_v2_20190128
      #define DOGLCD_CS                     PB5
    #else
      #define DOGLCD_CS                     PB7
    #endif

    //#define LCD_CONTRAST_INIT              190//#定义LCD对比度初始值190
    //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
    //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
    //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270

  #endif

  // not connected to a pin//未连接到pin
  #define SD_DETECT_PIN                     PC3

  #if IS_NEWPANEL
    // The encoder and click button//打开编码器并单击按钮
    #define BTN_EN1                         PC10
    #define BTN_EN2                         PC11
    #define BTN_ENC                         PC12
  #endif

  #ifdef pins_v2_20190128
    #define LCD_RESET_PIN                   PB4
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                 PB0
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                 PB6
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                 PB7
    #endif
  #else
    #define LCD_RESET_PIN                   PB6
    #ifndef RGB_LED_R_PIN
      #define RGB_LED_R_PIN                 PB3
    #endif
    #ifndef RGB_LED_G_PIN
      #define RGB_LED_G_PIN                 PB4
    #endif
    #ifndef RGB_LED_B_PIN
      #define RGB_LED_B_PIN                 PB5
    #endif
  #endif

#endif
