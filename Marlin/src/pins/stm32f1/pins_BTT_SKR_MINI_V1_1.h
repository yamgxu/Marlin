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

#define BOARD_INFO_NAME "BTT SKR Mini V1.1"

//#define DISABLE_DEBUG//#定义禁用调试
#define DISABLE_JTAG

// Ignore temp readings during development.//在开发过程中忽略温度读数。
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000//#定义2000年的假温度宽限期

#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB//2KB
#endif

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                           PC2
#define X_MAX_PIN                           PA2
#define Y_MIN_PIN                           PC1
#define Y_MAX_PIN                           PA1
#define Z_MIN_PIN                           PC0
#define Z_MAX_PIN                           PC3

////
// Steppers//踏步机
////

#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PC7
#define X_ENABLE_PIN                        PB15

#define Y_STEP_PIN                          PB13
#define Y_DIR_PIN                           PB14
#define Y_ENABLE_PIN                        PB12

#define Z_STEP_PIN                          PB10
#define Z_DIR_PIN                           PB11
#define Z_ENABLE_PIN                        PB2

#define E0_STEP_PIN                         PC5
#define E0_DIR_PIN                          PB0
#define E0_ENABLE_PIN                       PC4

#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PB3
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PB4
  #endif
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PB5
  #endif
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PA8
#define FAN_PIN                             PC8
#define HEATER_BED_PIN                      PC9

////
// Temperature Sensors//温度传感器
////
#define TEMP_BED_PIN                        PB1   // Analog Input//模拟输入
#define TEMP_0_PIN                          PA0   // Analog Input//模拟输入

////
// LCD Pins//LCD引脚
////

/**
 *                _____                                             _____
 *            NC | · · | GND                                    5V | · · | GND
 *         RESET | · · | PB9 (SD_DETECT)             (LCD_D7) PC14 | · · | PC15 (LCD_D6)
 *  (MOSI)   PB5 | · · | PB8 (BTN_EN2)               (LCD_D5)  PB7 | · · | PC13 (LCD_D4)
 * (SD_SS)  PA15 | · · | PD2 (BTN_EN1)               (LCD_RS) PC12 | · · | PB6  (LCD_EN)
 *   (SCK)   PB3 | · · | PB4 (MISO)                 (BTN_ENC) PC11 | · · | PC10 (BEEPER)
 *                -----                                             -----
 *                EXP2                                              EXP1
 */

#if HAS_WIRED_LCD
  #define BEEPER_PIN                        PC10
  #define BTN_ENC                           PC11

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS                     PC15

    #define BTN_EN1                         PB6
    #define BTN_EN2                         PC13

    #define LCD_PINS_ENABLE                 PC14
    #define LCD_PINS_D4                     PB7

  #elif IS_TFTGLCD_PANEL

    #undef BEEPER_PIN
    #undef BTN_ENC

    #if ENABLED(TFTGLCD_PANEL_SPI)
      #define TFTGLCD_CS                    PD2
    #endif

    #define SD_DETECT_PIN                   PB9

  #else

    #define LCD_PINS_RS                     PC12

    #define BTN_EN1                         PD2
    #define BTN_EN2                         PB8

    #define LCD_PINS_ENABLE                 PB6

    #if ENABLED(FYSETC_MINI_12864)

      #define LCD_BACKLIGHT_PIN             -1
      #define LCD_RESET_PIN                 PC13
      #define DOGLCD_A0                     PC12
      #define DOGLCD_CS                     PB6
      #define DOGLCD_SCK                    PB3
      #define DOGLCD_MOSI                   PB5

      #define FORCE_SOFT_SPI                      // SPI MODE3//SPI模式3

      #define LED_PIN                       PB7   // red pwm//红色脉宽调制
      //#define LED_PIN                     PC15  // green//#定义LED_引脚PC15//绿色
      //#define LED_PIN                     PC14  // blue//#定义LED_引脚PC14//蓝色

      //#if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)//#如果有（FYSETC_MINI_12864_1_2，FYSETC_MINI_12864_2_0）
      //  #ifndef RGB_LED_R_PIN//#ifndef RGB#U LED#U R#U引脚
      //    #define RGB_LED_R_PIN PB7//#定义RGB_LED_R_引脚PB7
      //  #endif//#endif
      //  #ifndef RGB_LED_G_PIN//#ifndef RGB#U LED#U G#U引脚
      //    #define RGB_LED_G_PIN PC15//#定义RGB_LED_G_引脚PC15
      //  #endif//#endif
      //  #ifndef RGB_LED_B_PIN//#ifndef RGB_LED#u引脚
      //    #define RGB_LED_B_PIN PC14//#定义RGB_LED_引脚PC14
      //  #endif//#endif
      //#elif ENABLED(FYSETC_MINI_12864_2_1)//#已启用elif（FYSETC_MINI_12864_2_1）
      //  #define NEOPIXEL_PIN    PB7//#定义Neopix_引脚PB7
      //#endif//#恩迪夫

    #else                                         // !FYSETC_MINI_12864// !FYSETC_MINI_12864

      #define LCD_PINS_D4                   PC13
      #if IS_ULTIPANEL
        #define LCD_PINS_D5                 PB7
        #define LCD_PINS_D6                 PC15
        #define LCD_PINS_D7                 PC14

        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
        #endif

      #endif

    #endif // !FYSETC_MINI_12864// !FYSETC_MINI_12864

    #if HAS_MARLINUI_U8GLIB
      #ifndef BOARD_ST7920_DELAY_1
        #define BOARD_ST7920_DELAY_1 DELAY_NS(125)
      #endif
      #ifndef BOARD_ST7920_DELAY_2
        #define BOARD_ST7920_DELAY_2 DELAY_NS(125)
      #endif
      #ifndef BOARD_ST7920_DELAY_3
        #define BOARD_ST7920_DELAY_3 DELAY_NS(125)
      #endif
    #endif

  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

////
// SD Card//SD卡
////

// By default the onboard SD is enabled.//默认情况下，车载SD已启用。
// Change SDCARD_CONNECTION from 'ONBOARD' to 'LCD' for an external (LCD module) SD//将外部（LCD模块）SD的SD卡_连接从“板载”更改为“LCD”
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD)
  #define SPI_DEVICE                           3
  #define SD_DETECT_PIN                     PB9
  #define SD_SCK_PIN                        PB3
  #define SD_MISO_PIN                       PB4
  #define SD_MOSI_PIN                       PB5
  #define SD_SS_PIN                         PA15
#elif SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PA3
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_SS_PIN                         PA4
#endif
#define ONBOARD_SPI_DEVICE                     1  // SPI1//SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card//“系统”SD卡的芯片选择
