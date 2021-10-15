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
 * RAMPS-FD
 *
 * No EEPROM
 * Use 4k7 thermistor tables
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "RAMPS-FD v1"
#endif

#define INVERTED_HEATER_PINS
#define INVERTED_BED_PINS
#define INVERTED_FAN_PINS

////
// Servos//伺服
////
#define SERVO0_PIN                             7
#define SERVO1_PIN                             6
#define SERVO2_PIN                             5
#define SERVO3_PIN                             3

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             22
#define X_MAX_PIN                             30
#define Y_MIN_PIN                             24
#define Y_MAX_PIN                             38
#define Z_MIN_PIN                             26
#define Z_MAX_PIN                             34

////
// Steppers//踏步机
////
#define X_STEP_PIN                            63
#define X_DIR_PIN                             62
#define X_ENABLE_PIN                          48
#ifndef X_CS_PIN
  #define X_CS_PIN                            68
#endif

#define Y_STEP_PIN                            65
#define Y_DIR_PIN                             64
#define Y_ENABLE_PIN                          46
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            60
#endif

#define Z_STEP_PIN                            67
#define Z_DIR_PIN                             66
#define Z_ENABLE_PIN                          44
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            58
#endif

#define E0_STEP_PIN                           36
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         42
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           67
#endif

#define E1_STEP_PIN                           43
#define E1_DIR_PIN                            41
#define E1_ENABLE_PIN                         39
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           61
#endif

#define E2_STEP_PIN                           32
#define E2_DIR_PIN                            47
#define E2_ENABLE_PIN                         45
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           59
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             1  // Analog Input//模拟输入
#define TEMP_1_PIN                             2  // Analog Input//模拟输入
#define TEMP_2_PIN                             3  // Analog Input//模拟输入
#define TEMP_BED_PIN                           0  // Analog Input//模拟输入

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS_PIN                      53
#else
  #define MAX6675_SS_PIN                      49
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           9
#define HEATER_1_PIN                          10
#define HEATER_2_PIN                          11
#define HEATER_BED_PIN                         8

#ifndef FAN_PIN
  #define FAN_PIN                             12
#endif

////
// Misc. Functions//杂项。功能
////
#define SDSS                                   4
#define LED_PIN                               13

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD
  // ramps-fd lcd adaptor//斜坡fd lcd适配器

  #define BEEPER_PIN                          37
  #define BTN_EN1                             33
  #define BTN_EN2                             31
  #define BTN_ENC                             35
  #define SD_DETECT_PIN                       49

  #if IS_NEWPANEL
    #define LCD_PINS_RS                       16
    #define LCD_PINS_ENABLE                   17
  #endif

  #if ENABLED(FYSETC_MINI_12864)
    #define DOGLCD_CS            LCD_PINS_ENABLE
    #define DOGLCD_A0                LCD_PINS_RS
    #define DOGLCD_SCK                        76
    #define DOGLCD_MOSI                       75

    //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems//#定义FORCE\u SOFT\u SPI//如果硬件SPI的默认值导致显示问题，请使用此选项
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0//结果为LCD软SPI模式3，SD软SPI模式0

    #define LCD_RESET_PIN                     23  // Must be high or open for LCD to operate normally.//必须为高电平或开路，LCD才能正常工作。

    #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN                 25
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN                 27
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN                 29
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN                    25
    #endif

  #elif IS_NEWPANEL

    #define LCD_PINS_D4                       23
    #define LCD_PINS_D5                       25
    #define LCD_PINS_D6                       27
    #define LCD_PINS_D7                       29

    #if ENABLED(MINIPANEL)
      #define DOGLCD_CS                       25
      #define DOGLCD_A0                       27
    #endif

  #endif

  #if ANY(VIKI2, miniVIKI)
    #define DOGLCD_A0                         16
    #define KILL_PIN                          51
    #define STAT_LED_BLUE_PIN                 29
    #define STAT_LED_RED_PIN                  23
    #define DOGLCD_CS                         17
    #define DOGLCD_SCK                        76  // SCK_PIN   - Required for DUE Hardware SPI//SCK_引脚-需要适当的硬件SPI
    #define DOGLCD_MOSI                       75  // MOSI_PIN//莫西奥平
    #define DOGLCD_MISO                       74  // MISO_PIN//味噌皮
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1//#定义X_硬件_串行1
  //#define X2_HARDWARE_SERIAL Serial1//#定义X2_硬件_串行1
  //#define Y_HARDWARE_SERIAL  Serial1//#定义Y_硬件_串行1
  //#define Y2_HARDWARE_SERIAL Serial1//#定义Y2\u硬件\u串行1
  //#define Z_HARDWARE_SERIAL  Serial1//#定义Z_硬件_串行1
  //#define Z2_HARDWARE_SERIAL Serial1//#定义Z2_硬件_串行1
  //#define E0_HARDWARE_SERIAL Serial1//#定义E0_硬件_串行1
  //#define E1_HARDWARE_SERIAL Serial1//#定义E1_硬件_串行1
  //#define E2_HARDWARE_SERIAL Serial1//#定义E2_硬件_串行1
  //#define E3_HARDWARE_SERIAL Serial1//#定义E3\u硬件\u串行1
  //#define E4_HARDWARE_SERIAL Serial1//#定义E4\u硬件\u串行1
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#if HOTENDS < 3 && HAS_CUTTER && !PIN_EXISTS(SPINDLE_LASER_ENA)
  #define SPINDLE_LASER_ENA_PIN               45  // Use E2 ENA//使用E2-ENA
  #define SPINDLE_LASER_PWM_PIN               12  // Hardware PWM//硬件脉宽调制
  #define SPINDLE_DIR_PIN                     47  // Use E2 DIR//使用E2目录
#endif
