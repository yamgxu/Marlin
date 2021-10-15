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

/**
 * 2017 Victor Perez Marlin for stm32f1 test
 */

#define BOARD_INFO_NAME      "Chitu3D"
#define DEFAULT_MACHINE_NAME "STM32F103RET6"

#define BOARD_NO_NATIVE_USB

// Enable I2C_EEPROM for testing//启用I2C_EEPROM进行测试
//#define I2C_EEPROM//#定义I2C_EEPROM

// Ignore temp readings during development.//在开发过程中忽略温度读数。
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000//#定义2000年的假温度宽限期

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PG10
#define Y_STOP_PIN                          PA12
#define Z_STOP_PIN                          PA14

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PE6
#define X_ENABLE_PIN                        PC13

#define Y_STEP_PIN                          PE2
#define Y_DIR_PIN                           PE3
#define Y_ENABLE_PIN                        PE4

#define Z_STEP_PIN                          PB9
#define Z_DIR_PIN                           PE0
#define Z_ENABLE_PIN                        PE1

#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB5
#define E0_ENABLE_PIN                       PB8

////
// Misc. Functions//杂项。功能
////
#define SDSS                                -1
#define LED_PIN                             -1

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                    PA8   // 8// 8
#endif

#define PS_ON_PIN                           -1
#define KILL_PIN                            PD6   // LED strip 24v//LED带24v

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PD12  // HOT-END//热端
#define HEATER_1_PIN                        -1
#define HEATER_2_PIN                        -1

#define HEATER_BED_PIN                      PG11  // HOT-BED//热床
#define HEATER_BED2_PIN                     -1    // BED2//床2
#define HEATER_BED3_PIN                     -1    // BED3//床3

#ifndef FAN_PIN
  #define FAN_PIN                           PG14  // MAIN BOARD FAN//主板风扇
#endif

#define FAN_SOFT_PWM

////
// Temperature Sensors//温度传感器
////
#define TEMP_BED_PIN                        PA0   // Analog Input//模拟输入
#define TEMP_0_PIN                          PA1   // Analog Input//模拟输入

////
// LCD Pins//LCD引脚
////
#if HAS_WIRED_LCD

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define LCD_PINS_RS                     PD1   // 49  // CS chip select /SS chip slave select//49//CS芯片选择/SS芯片从属选择
    #define LCD_PINS_ENABLE                 PD3   // 51  // SID (MOSI)//51//SID（MOSI）
    #define LCD_PINS_D4                     PD4   // 52  // SCK (CLK) clock//52//SCK（时钟）时钟
  #elif BOTH(IS_NEWPANEL, PANEL_ONE)
    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_ENABLE                 PD2
    #define LCD_PINS_D4                     PB12
    #define LCD_PINS_D5                     PB13
    #define LCD_PINS_D6                     PB14
    #define LCD_PINS_D7                     PB15
  #else
    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_ENABLE                 PD2
    #define LCD_PINS_D4                     PB12
    #define LCD_PINS_D5                     PB13
    #define LCD_PINS_D6                     PB14
    #define LCD_PINS_D7                     PB15
    #if !IS_NEWPANEL
      #define BEEPER_PIN                    PC1   // 33// 33
      // Buttons attached to a shift register//与移位寄存器相连的按钮
      // Not wired yet//还没连线
      //#define SHIFT_CLK_PIN               PC6   // 38//#定义移位时钟引脚PC6//38
      //#define SHIFT_LD_PIN                PC10  // 42//#定义移位针PC10//42
      //#define SHIFT_OUT_PIN               PC8   // 40//#定义移位针PC8//40
      //#define SHIFT_EN_PIN                PA1   // 17//#定义移位针PA1//17
    #endif
  #endif

  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN                    PC5

      #define BTN_EN1                       PB15  // 31// 31
      #define BTN_EN2                       PC1   // 33// 33
      #define BTN_ENC                       PC3   // 35// 35

      #define SD_DETECT_PIN                 PD1   // 49// 49
      #define KILL_PIN                      PC9   // 41// 41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN           PC7   // 39// 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                       PE0   // 64// 64
      #define BTN_EN2                       PD11  // 59// 59
      #define BTN_ENC                       PD15  // 63// 63
      #define SD_DETECT_PIN                 PC10  // 42// 42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                       PC15  // 47// 47
      #define BTN_EN2                       PC11  // 43// 43
      #define BTN_ENC                       PC0   // 32// 32
      #define LCD_SDSS                      PD5   // 53// 53
      #define SD_DETECT_PIN                 -1
      #define KILL_PIN                      PC9   // 41// 41

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                       PB6   // 22   // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.// 22   // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf 解释40/42。
      #define BTN_EN2                       PA7   //  7   // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.//7//22/7在坡道上未使用。22未使用，7斜坡13上的伺服0_销。

      #define BTN_ENC                       -1
      #define LCD_SDSS                      PD5   // 53// 53
      #define SD_DETECT_PIN                 PD1   // 49// 49

    #elif ANY(VIKI2, miniVIKI)

      #define BEEPER_PIN                    PC1   // 33// 33

      // Pins for DOGM SPI LCD Support//用于DOGM SPI LCD支持的引脚
      #define DOGLCD_A0                     PC12  // 44// 44
      #define DOGLCD_CS                     PC13  // 45// 45
      #define LCD_SCREEN_ROT_180

      #define BTN_EN1                       PB6   // 22// 22
      #define BTN_EN2                       PA7   //  7//  7
      #define BTN_ENC                       PC7   // 39// 39

      #define SDSS                          PD5   // 53// 53
      #define SD_DETECT_PIN                 -1    // Pin 49 for display sd interface, 72 for easy adapter board//引脚49用于显示sd接口，72用于简易适配器板

      #define KILL_PIN                      PB15  // 31// 31

      #define STAT_LED_RED_PIN              PC0   // 32// 32
      #define STAT_LED_BLUE_PIN             PC3   // 35// 35

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1                       PC3   // 35// 35
      #define BTN_EN2                       PC5   // 37// 37
      #define BTN_ENC                       PB15  // 31// 31
      #define SD_DETECT_PIN                 PD1   // 49// 49
      #define LCD_SDSS                      PD5   // 53// 53
      #define KILL_PIN                      PC9   // 41// 41
      #define BEEPER_PIN                    PB7   // 23// 23
      #define DOGLCD_CS                     PB13  // 29// 29
      #define DOGLCD_A0                     PB11  // 27// 27
      #define LCD_BACKLIGHT_PIN             PC1   // 33// 33

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN                    PC10  // 42// 42
      // Pins for DOGM SPI LCD Support//用于DOGM SPI LCD支持的引脚
      #define DOGLCD_A0                     PC12  // 44// 44
      #define DOGLCD_CS                     PE2   // 66// 66
      #define LCD_BACKLIGHT_PIN             PE1   // 65   // backlight LED on A11/D65//65//A11/D65上的背光LED
      #define SDSS                          PD5   // 53// 53

      #define KILL_PIN                      PE0   // 64// 64
      // GLCD features//GLCD功能
      // Uncomment screen orientation//取消注释屏幕方向
      //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
      //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
      //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270
      // The encoder and click button//打开编码器并单击按钮
      #define BTN_EN1                       PC8   // 40// 40
      #define BTN_EN2                       PD15  // 63// 63
      #define BTN_ENC                       PD11  // 59// 59
      // not connected to a pin//未连接到pin
      #define SD_DETECT_PIN                 PD1   // 49// 49

    #else

      // Beeper on AUX-4//AUX-4上的蜂鸣器
      #define BEEPER_PIN                    PC1   // 33// 33

      // Buttons directly attached to AUX-2//直接连接到AUX-2的按钮
      #if IS_RRW_KEYPAD
        #define BTN_EN1                     PE0   // 64// 64
        #define BTN_EN2                     PD11  // 59// 59
        #define BTN_ENC                     PD15  // 63// 63
        #define SHIFT_OUT_PIN               PC8   // 40// 40
        #define SHIFT_CLK_PIN               PC12  // 44// 44
        #define SHIFT_LD_PIN                PC10  // 42// 42
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1                     PD11  // 59   // AUX2 PIN 3//59//AUX2引脚3
        #define BTN_EN2                     PD15  // 63   // AUX2 PIN 4//63//AUX2引脚4
        #define BTN_ENC                     PD1   // 49   // AUX3 PIN 7//49//AUX3引脚7
      #else
        #define BTN_EN1                     PC5   // 37// 37
        #define BTN_EN2                     PC3   // 35// 35
        #define BTN_ENC                     PB15  // 31// 31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN               PD1   // 49// 49
        #define KILL_PIN                    PC9   // 41// 41
      #else
        //#define SD_DETECT_PIN             -1    // Ramps doesn't use this//#定义SD_DETECT_引脚-1//Ramps不使用此选项
      #endif

    #endif
  #endif // IS_NEWPANEL//这是新小组吗

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗
