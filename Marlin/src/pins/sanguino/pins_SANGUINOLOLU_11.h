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
 * Sanguinololu board pin assignments
 */

/**
 * Rev B    26 DEC 2016
 *
 * 1) added pointer to a current Arduino IDE extension
 * 2) added support for M3, M4 & M5 spindle control commands
 * 3) added case light pin definition
 */

/**
 * A useable Arduino IDE extension (board manager) can be found at
 * https://github.com/Lauszus/Sanguino
 *
 * This extension has been tested on Arduino 1.6.12 & 1.8.0
 *
 * Here's the JSON path:
 * https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
 *
 * When installing select 1.0.2
 *
 * Installation instructions can be found at https://learn.sparkfun.com/pages/CustomBoardsArduino
 * Just use the above JSON URL instead of Sparkfun's JSON.
 *
 * Once installed select the Sanguino board and then select the CPU.
 */

#define ALLOW_MEGA644P
#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Sanguinololu <1.2"
#endif

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            18
#define Y_STOP_PIN                            19
#define Z_STOP_PIN                            20

////
// Steppers//踏步机
////
#define X_STEP_PIN                            15
#define X_DIR_PIN                             21

#define Y_STEP_PIN                            22
#define Y_DIR_PIN                             23

#define Z_STEP_PIN                             3
#define Z_DIR_PIN                              2

#define E0_STEP_PIN                            1
#define E0_DIR_PIN                             0

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             7  // Analog Input (pin 33 extruder)//模拟输入（引脚33挤出机）
#define TEMP_BED_PIN                           6  // Analog Input (pin 34 bed)//模拟输入（引脚34）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          13  // (extruder)//（挤压机）

#if ENABLED(SANGUINOLOLU_V_1_2)

  #define HEATER_BED_PIN                      12  // (bed)//（床）
  #define X_ENABLE_PIN                        14
  #define Y_ENABLE_PIN                        14
  #define Z_ENABLE_PIN                        26
  #define E0_ENABLE_PIN                       14

  #if !defined(FAN_PIN) && ENABLED(LCD_I2C_PANELOLU2)
    #define FAN_PIN                            4  // Uses Transistor1 (PWM) on Panelolu2's Sanguino Adapter Board to drive the fan//使用Panelolu2的Sanguino适配器板上的晶体管1（PWM）驱动风扇
  #endif

#else

  #define HEATER_BED_PIN                      14  // (bed)//（床）
  #define X_ENABLE_PIN                         4
  #define Y_ENABLE_PIN                         4
  #define Z_ENABLE_PIN                         4
  #define E0_ENABLE_PIN                        4

#endif

#if !defined(FAN_PIN) && (MB(AZTEEG_X1, STB_11) || IS_MELZI)
  #define FAN_PIN                              4  // Works for Panelolu2 too//也适用于Panelolu2
#endif

////
// Misc. Functions//杂项。功能
////

/**
 * In some versions of the Sanguino libraries the pin
 * definitions are wrong, with SDSS = 24 and LED_PIN = 28 (Melzi).
 * If you encounter issues with these pins, upgrade your
 * Sanguino libraries! See #368.
 */
//#define SDSS                                24//#定义SDSS 24
#define SDSS                                  31

#if IS_MELZI
  #define LED_PIN                             27
#elif MB(STB_11)
  #define LCD_BACKLIGHT_PIN                   17  // LCD backlight LED//LCD背光LED
#endif

#if NONE(SPINDLE_FEATURE, LASER_FEATURE) && ENABLED(SANGUINOLOLU_V_1_2) && !BOTH(IS_ULTRA_LCD, IS_NEWPANEL) // try to use IO Header//尝试使用IO头
  #define CASE_LIGHT_PIN                       4  // Hardware PWM  - see if IO Header is available//硬件PWM-查看IO标头是否可用
#endif

/**
 * Sanguinololu 1.4 AUX pins:
 *
 *           PWM  TX1  RX1  SDA  SCL
 *  12V  5V  D12  D11  D10  D17  D16
 *  GND GND  D31  D30  D29  D28  D27
 *            A4   A3   A2   A1   A0
 */

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD

  #define SD_DETECT_PIN                       -1

  #if HAS_MARLINUI_U8GLIB

    #if ENABLED(LCD_FOR_MELZI)

      #define LCD_PINS_RS                     17
      #define LCD_PINS_ENABLE                 16
      #define LCD_PINS_D4                     11
      #define KILL_PIN                        10
      #define BEEPER_PIN                      27

      #ifndef BOARD_ST7920_DELAY_1
        #define BOARD_ST7920_DELAY_1 DELAY_NS(0)
      #endif
      #ifndef BOARD_ST7920_DELAY_2
        #define BOARD_ST7920_DELAY_2 DELAY_NS(188)
      #endif
      #ifndef BOARD_ST7920_DELAY_3
        #define BOARD_ST7920_DELAY_3 DELAY_NS(0)
      #endif

    #elif ENABLED(U8GLIB_ST7920)                  // SPI GLCD 12864 ST7920 ( like [www.digole.com] ) For Melzi V2.0//适用于Melzi V2.0的SPI GLCD 12864 ST7920（如[www.digole.com]）

      #if IS_MELZI
        #define LCD_PINS_RS                   30  // CS chip select /SS chip slave select//CS芯片选择/SS芯片从属选择
        #define LCD_PINS_ENABLE               29  // SID (MOSI)//SID（MOSI）
        #define LCD_PINS_D4                   17  // SCK (CLK) clock//时钟
        // Pin 27 is taken by LED_PIN, but Melzi LED does nothing with//引脚27由LED_引脚占据，但Melzi LED对其不起任何作用
        // Marlin so this can be used for BEEPER_PIN. You can use this pin//Marlin，所以这可以用于寻呼机PIN。你可以用这个别针
        // with M42 instead of BEEPER_PIN.//使用M42而不是蜂鸣器_引脚。
        #define BEEPER_PIN                    27

        #if IS_RRD_FG_SC
          #ifndef BOARD_ST7920_DELAY_1
            #define BOARD_ST7920_DELAY_1 DELAY_NS(0)
          #endif
          #ifndef BOARD_ST7920_DELAY_2
            #define BOARD_ST7920_DELAY_2 DELAY_NS(188)
          #endif
          #ifndef BOARD_ST7920_DELAY_3
            #define BOARD_ST7920_DELAY_3 DELAY_NS(0)
          #endif
        #endif

      #else                                       // Sanguinololu >=1.3//Sanguinololu>=1.3
        #define LCD_PINS_RS                    4
        #define LCD_PINS_ENABLE               17
        #define LCD_PINS_D4                   30
        #define LCD_PINS_D5                   29
        #define LCD_PINS_D6                   28
        #define LCD_PINS_D7                   27
      #endif

    #else

      #define DOGLCD_A0                       30

      #if ENABLED(MAKRPANEL)

        #define BEEPER_PIN                    29
        #define DOGLCD_CS                     17
        #define LCD_BACKLIGHT_PIN             28  // PA3//PA3

      #elif IS_MELZI

        #define BEEPER_PIN                    27
        #define DOGLCD_CS                     28

      #else                                       // !MAKRPANEL// !马克帕内尔

        #define DOGLCD_CS                     29

      #endif

    #endif

    // Uncomment screen orientation//取消注释屏幕方向
    //#define LCD_SCREEN_ROT_0//#定义LCD\u屏幕\u旋转\u 0
    //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
    //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
    //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270

  #elif ENABLED(ZONESTAR_LCD)                     // For the Tronxy Melzi boards//对于Tronxy Melzi板

    #define LCD_PINS_RS                       28
    #define LCD_PINS_ENABLE                   29
    #define LCD_PINS_D4                       10
    #define LCD_PINS_D5                       11
    #define LCD_PINS_D6                       16
    #define LCD_PINS_D7                       17

  #else

    #define LCD_PINS_RS                        4
    #define LCD_PINS_ENABLE                   17
    #define LCD_PINS_D4                       30
    #define LCD_PINS_D5                       29
    #define LCD_PINS_D6                       28
    #define LCD_PINS_D7                       27

  #endif

  #if ENABLED(LCD_FOR_MELZI)

    #define BTN_ENC                           28
    #define BTN_EN1                           29
    #define BTN_EN2                           30

  #elif ENABLED(ZONESTAR_LCD)                     // For the Tronxy Melzi boards//对于Tronxy Melzi板

    #define ADC_KEYPAD_PIN                     1
    #define BTN_EN1                           -1
    #define BTN_EN2                           -1

  #elif ENABLED(LCD_I2C_PANELOLU2)

    #if IS_MELZI
      #define BTN_ENC                         29
      #define LCD_SDSS                        30  // Panelolu2 SD card reader rather than the Melzi//Panelolu2 SD卡读卡器而非Melzi
    #else
      #define BTN_ENC                         30
    #endif

  #else                                           // !LCD_FOR_MELZI && !ZONESTAR_LCD && !LCD_I2C_PANELOLU2// !LCD___MELZI&&！ZONESTAR_LCD&amp！LCD_I2C_面板2

    #define BTN_ENC                           16
    #define LCD_SDSS                          28  // Smart Controller SD card reader rather than the Melzi//智能控制器SD卡读卡器而非Melzi

  #endif

  #if IS_NEWPANEL && !defined(BTN_EN1)
    #define BTN_EN1                           11
    #define BTN_EN2                           10
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#if HAS_CUTTER
  #if !MB(AZTEEG_X1) && ENABLED(SANGUINOLOLU_V_1_2) && !BOTH(IS_ULTRA_LCD, IS_NEWPANEL) // try to use IO Header//尝试使用IO头

    #define SPINDLE_LASER_ENA_PIN             10  // Pullup or pulldown!//拉起还是拉下！
    #define SPINDLE_LASER_PWM_PIN              4  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_DIR_PIN                   11

  #elif !MB(MELZI)                                // use X stepper motor socket//使用X步进电机插座

    /**
     *  To control the spindle speed and have an LCD you must sacrifice
     *  the Extruder and pull some signals off the X stepper driver socket.
     *
     *  The following assumes:
     *   - The X stepper driver socket is empty
     *   - The extruder driver socket has a driver board plugged into it
     *   - The X stepper wires are attached the the extruder connector
     */

    /**
     *  Where to get the spindle signals
     *
     *      spindle signal          socket name       socket name
     *                                         -------
     *                               /ENABLE  O|     |O  VMOT
     *                                   MS1  O|     |O  GND
     *                                   MS2  O|     |O  2B
     *                                   MS3  O|     |O  2A
     *                                /RESET  O|     |O  1A
     *                                /SLEEP  O|     |O  1B
     *  SPINDLE_LASER_PWM_PIN           STEP  O|     |O  VDD
     *  SPINDLE_LASER_ENA_PIN         DIR  O|     |O  GND
     *                                         -------
     *
     *  Note: Socket names vary from vendor to vendor.
     */
    #undef X_DIR_PIN
    #undef X_ENABLE_PIN
    #undef X_STEP_PIN
    #define X_DIR_PIN                          0
    #define X_ENABLE_PIN                      14
    #define X_STEP_PIN                         1
    #define SPINDLE_LASER_PWM_PIN             15  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_LASER_ENA_PIN             21  // Pullup!//拉起！
    #define SPINDLE_DIR_PIN                   -1  // No pin available on the socket for the direction pin//插座上没有用于方向插针的插针
  #endif
#endif
