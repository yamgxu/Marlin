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
 * Ported sys0724 & Vynt
 */
#pragma once

/**
 * Arduino Mega? or Due with RuRAMPS4DUE pin assignments
 *
 * Applies to the following boards:
 *  RURAMPS4DUE      (Hotend0, Hotend1, Hotend2, Fan0, Fan1, Bed)
 *
 *  Differences between
 *     RADDS | RuRAMPS4DUE
 *           |
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "RuRAMPS4Due v1.1"

////
// Servos//伺服
////
#define SERVO0_PIN                             5
#define SERVO1_PIN                             3

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             45
#define X_MAX_PIN                             39
#define Y_MIN_PIN                             46
#define Y_MAX_PIN                             41
#define Z_MIN_PIN                             47
#define Z_MAX_PIN                             43

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     43
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            37  // Support Extension Board//支撑扩展板
#define X_DIR_PIN                             36
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            -1
#endif

#define Y_STEP_PIN                            32  // Support Extension Board//支撑扩展板
#define Y_DIR_PIN                             35
#define Y_ENABLE_PIN                          34
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            -1
#endif

#define Z_STEP_PIN                            30  // Support Extension Board//支撑扩展板
#define Z_DIR_PIN                              2
#define Z_ENABLE_PIN                          33
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            -1
#endif

#define E0_STEP_PIN                           29
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         31
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           -1
#endif

#define E1_STEP_PIN                           22
#define E1_DIR_PIN                            24
#define E1_ENABLE_PIN                         26
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           -1
#endif

#define E2_STEP_PIN                           25
#define E2_DIR_PIN                            23
#define E2_ENABLE_PIN                         27
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           -1
#endif

#define E3_STEP_PIN                           15  // Only For Extension Board//仅适用于扩展板
#define E3_DIR_PIN                            14
#define E3_ENABLE_PIN                         61
#ifndef E3_CS_PIN
  #define E3_CS_PIN                           -1
#endif

// For Future: Microstepping pins - Mapping not from fastio.h (?)//未来：微步进引脚-映射不是从fastio.h（？）
//#define E3_MS1_PIN         ?//#定义E3_MS1_引脚？
//#define E3_MS2_PIN         ?//#定义E3\U MS2\U引脚？
//#define E3_MS3_PIN         ?//#定义E3\U MS3\U引脚？

#if HAS_CUSTOM_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     49
#endif

#if HAS_FILAMENT_SENSOR
  #ifndef FIL_RUNOUT_PIN
    #define FIL_RUNOUT_PIN             Y_MIN_PIN
  #endif
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          13
#define HEATER_1_PIN                          12
#define HEATER_2_PIN                          11
#define HEATER_BED_PIN                         7  // BED H1//床H1

#ifndef FAN_PIN
  #define FAN_PIN                              9
#endif
#define FAN1_PIN                               8
#define CONTROLLER_FAN_PIN                    -1

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             0  // ANALOG A0//模拟A0
#define TEMP_1_PIN                             1  // ANALOG A1//模拟A1
#define TEMP_2_PIN                             2  // ANALOG A2//模拟A2
#define TEMP_3_PIN                             3  // ANALOG A3//模拟A3
#define TEMP_BED_PIN                           4  // ANALOG A4//模拟A4

// The thermocouple uses Analog pins//热电偶使用模拟引脚
#if ENABLED(VER_WITH_THERMOCOUPLE)                // Defined in Configuration.h//在Configuration.h中定义
  #define TEMP_4_PIN                           5  // A5//A5
  #define TEMP_5_PIN                           6  // A6 (Marlin 2.0 not support)//A6（马林2.0不支持）
#endif

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
/*
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS_PIN                      53
#else
  #define MAX6675_SS_PIN                      49
#endif
*/

////
// Misc. Functions//杂项。功能
////
#define SDSS                                   4  // 4,10,52 if using HW SPI.//4,10,52如果使用HW SPI。
#define LED_PIN                               -1  // 13 - HEATER_0_PIN//13-加热器0针
#define PS_ON_PIN                             -1  // 65// 65

// MKS TFT / Nextion Use internal USART-1//MKS TFT/Nextion使用内部USART-1
#define TFT_LCD_MODULE_COM                     1
#define TFT_LCD_MODULE_BAUDRATE              115600

// ESP WiFi Use internal USART-2//ESP WiFi使用内部USART-2
#define ESP_WIFI_MODULE_COM                    2
#define ESP_WIFI_MODULE_BAUDRATE             115600
#define ESP_WIFI_MODULE_RESET_PIN             -1
#define PIGGY_GPIO_PIN                        -1

////
// EEPROM//电可擦可编程只读存储器
////
#define MARLIN_EEPROM_SIZE                0x8000  // 32Kb (24lc256)//32Kb（24lc256）
#define I2C_EEPROM                                // EEPROM on I2C-0//I2C-0上的EEPROM
//#define EEPROM_SD                               // EEPROM on SDCARD//#定义EEPROM\u SD//SD卡上的EEPROM
//#define SPI_EEPROM                              // EEPROM on SPI-0//#定义SPI_EEPROM//SPI-0上的EEPROM
//#define SPI_CHAN_EEPROM1        ?//#定义SPI_CHAN_EEPROM1？
//#define SPI_EEPROM1_CS          ?//#定义SPI_EEPROM1_CS？
// 2K EEPROM//2K EEPROM
//#define SPI_EEPROM2_CS          ?//#定义SPI_EEPROM2_CS？
// 32Mb FLASH//32Mb闪存
//#define SPI_FLASH_CS            ?//#定义SPI\U闪存\U CS？

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD

  #if ANY(RADDS_DISPLAY, IS_RRD_SC, IS_RRD_FG_SC)
    #define BEEPER_PIN                        62
    #define LCD_PINS_D4                       48
    #define LCD_PINS_D5                       50
    #define LCD_PINS_D6                       52
    #define LCD_PINS_D7                       53
    #define SD_DETECT_PIN                     51
  #endif

  #if EITHER(RADDS_DISPLAY, IS_RRD_SC)

    #define LCD_PINS_RS                       63
    #define LCD_PINS_ENABLE                   64

  #elif IS_RRD_FG_SC

    #define LCD_PINS_RS                       52
    #define LCD_PINS_ENABLE                   53

  #elif HAS_U8GLIB_I2C_OLED

    #define BEEPER_PIN                        62
    #define LCD_SDSS                          10
    #define SD_DETECT_PIN                     51

  #elif ENABLED(FYSETC_MINI_12864)

    #define BEEPER_PIN                        62
    #define DOGLCD_CS                         64
    #define DOGLCD_A0                         63

    //#define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems//#定义FORCE\u SOFT\u SPI//如果硬件SPI的默认值导致显示问题，请使用此选项
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0//结果为LCD软SPI模式3，SD软SPI模式0

    #define LCD_RESET_PIN                     48  // Must be high or open for LCD to operate normally.//必须为高电平或开路，LCD才能正常工作。

    #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN                 50  // D5//D5
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN                 52  // D6//D6
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN                 53  // D7//D7
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN                    50  // D5//D5
    #endif

  #elif ENABLED(SPARK_FULL_GRAPHICS)

    //http://doku.radds.org/dokumentation/other-electronics/sparklcd///http://doku.radds.org/dokumentation/other-electronics/sparklcd/
    #error "Oops! SPARK_FULL_GRAPHICS not supported with RURAMPS4D."
    //#define LCD_PINS_D4                     29  //?//#定义LCD_引脚_D4 29/？
    //#define LCD_PINS_ENABLE                 27  //?//#定义LCD_引脚_启用27/？
    //#define LCD_PINS_RS                     25  //?//#定义LCD_引脚_RS 25/？
    //#define BTN_EN1                         35  //?//#定义BTN_EN1 35/？
    //#define BTN_EN2                         33  //?//#定义BTN_EN2 33/？
    //#define BTN_ENC                         37  //?//#定义BTN_ENC 37/？

  #endif // SPARK_FULL_GRAPHICS//SPARK_FULL_图形

  #if IS_NEWPANEL
    #define BTN_EN1                           44
    #define BTN_EN2                           42
    #define BTN_ENC                           40
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗
