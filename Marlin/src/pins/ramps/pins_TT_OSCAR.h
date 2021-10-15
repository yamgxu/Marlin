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

#if HOTENDS > 5 || E_STEPPERS > 5
  #error "TTOSCAR supports up to 5 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "TT OSCAR"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

////
// Servos//伺服
////
#define SERVO0_PIN                            11
#define SERVO1_PIN                            12
#define SERVO2_PIN                             5
#define SERVO3_PIN                             4

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             14
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN             SERVO3_PIN
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#define X_CS_PIN                              57

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#define Y_CS_PIN                              58

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62
#define Z_CS_PIN                              53

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#define E0_CS_PIN                             49

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#define E1_CS_PIN                      E0_CS_PIN

#define E2_STEP_PIN                           63
#define E2_DIR_PIN                            22
#define E2_ENABLE_PIN                         59
#define E2_CS_PIN                      E0_CS_PIN

#define E3_STEP_PIN                           32
#define E3_DIR_PIN                            40
#define E3_ENABLE_PIN                         39
#define E3_CS_PIN                      E0_CS_PIN

#define E4_STEP_PIN                           43
#define E4_DIR_PIN                            42
#define E4_ENABLE_PIN                         47
#define E4_CS_PIN                      E0_CS_PIN

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
  //#define E3_HARDWARE_SERIAL Serial1//#定义E3\u硬件\u串行1

  ////
  // Software serial//软件系列
  ////

  #define X_SERIAL_TX_PIN                     -1  // 59// 59
  #define X_SERIAL_RX_PIN                     -1  // 63// 63
  #define X2_SERIAL_TX_PIN                    -1
  #define X2_SERIAL_RX_PIN                    -1

  #define Y_SERIAL_TX_PIN                     -1  // 64// 64
  #define Y_SERIAL_RX_PIN                     -1  // 40// 40
  #define Y2_SERIAL_TX_PIN                    -1
  #define Y2_SERIAL_RX_PIN                    -1

  #define Z_SERIAL_TX_PIN                     -1  // 44// 44
  #define Z_SERIAL_RX_PIN                     -1  // 42// 42
  #define Z2_SERIAL_TX_PIN                    -1
  #define Z2_SERIAL_RX_PIN                    -1

  #define E0_SERIAL_TX_PIN                    -1  // 66// 66
  #define E0_SERIAL_RX_PIN                    -1  // 65// 65
  #define E1_SERIAL_TX_PIN                    -1
  #define E1_SERIAL_RX_PIN                    -1
  #define E2_SERIAL_TX_PIN                    -1
  #define E2_SERIAL_RX_PIN                    -1
  #define E3_SERIAL_TX_PIN                    -1
  #define E3_SERIAL_RX_PIN                    -1
  #define E4_SERIAL_TX_PIN                    -1
  #define E4_SERIAL_RX_PIN                    -1
  #define E5_SERIAL_RX_PIN                    -1
  #define E6_SERIAL_RX_PIN                    -1
  #define E7_SERIAL_RX_PIN                    -1
#endif

////
// Default pins for TMC software SPI//TMC软件SPI的默认引脚
////
//#if ENABLED(TMC_USE_SW_SPI)//#如果启用（TMC\U使用\U SW\U SPI）
//  #ifndef TMC_SW_MOSI//#如果没有TMC#U SW#U MOSI
//    #define TMC_SW_MOSI    66//#定义TMC_SW_MOSI 66
//  #endif//#endif
//  #ifndef TMC_SW_MISO//#如果没有TMC#u SW#u MISO
//    #define TMC_SW_MISO    44//#定义TMC_SW_味噌44
//  #endif//#endif
//  #ifndef TMC_SW_SCK//#如果没有TMC#U SW#U SCK
//    #define TMC_SW_SCK     64//#定义TMC_SW_SCK 64
//  #endif//#endif
//#endif//#恩迪夫

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            13
#define TEMP_1_PIN                            15
#define TEMP_2_PIN                            10
#define TEMP_3_PIN                            11
#define TEMP_BED_PIN                          14

#if TEMP_SENSOR_CHAMBER > 0
  #define TEMP_CHAMBER_PIN                    12
#else
  #define TEMP_4_PIN                          12
#endif

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
//#if DISABLED(SDSUPPORT)//#如果禁用（SDSUPPORT）
//  #define MAX6675_SS_PIN   66   // Don't use 53 if using Display/SD card//#定义MAX6675_SS_引脚66//如果使用显示卡/SD卡，则不要使用53
//#else//#否则
//  #define MAX6675_SS_PIN   66   // Don't use 49 (SD_DETECT_PIN)//#定义MAX6675_SS_引脚66//不使用49（SD_检测_引脚）
//#endif//#恩迪夫

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           7
#define HEATER_2_PIN                          44
#define HEATER_BED_PIN                         8

#define FAN_PIN                                9

#if EXTRUDERS >= 5
  #define HEATER_4_PIN                         6
#else
  #define FAN1_PIN                             6
#endif

#if EXTRUDERS >= 4
  #define HEATER_3_PIN                        45
#else
  #define FAN2_PIN                            45
#endif

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  53
#define LED_PIN                               13

//#ifndef FILWIDTH_PIN//#ifndef FILWIDTH\U引脚
//  #define FILWIDTH_PIN      5   // Analog Input//#定义FILWIDTH_引脚5//模拟输入
//#endif//#恩迪夫

// DIO 4 (Servos plug) for the runout sensor.//跳动传感器的DIO 4（伺服插头）。
//#define FIL_RUNOUT_PIN              SERVO3_PIN//#定义FIL_跳动_销伺服3_销

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12
#endif

////
// Case Light//箱灯
////
#if ENABLED(CASE_LIGHT_ENABLE) && !PIN_EXISTS(CASE_LIGHT) && !defined(SPINDLE_LASER_ENABLE_PIN)
  #if !NUM_SERVOS                                 // Prefer the servo connector//首选伺服连接器
    #define CASE_LIGHT_PIN                     6  // Hardware PWM//硬件脉宽调制
  #elif HAS_FREE_AUX2_PINS                        // Try to use AUX 2//尝试使用辅助2
    #define CASE_LIGHT_PIN                    44  // Hardware PWM//硬件脉宽调制
  #endif
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#if ENABLED(SPINDLE_LASER_ENABLE) && !PIN_EXISTS(SPINDLE_LASER_ENABLE)
  #if !NUM_SERVOS                                 // Prefer the servo connector//首选伺服连接器
    #define SPINDLE_LASER_ENABLE_PIN           4  // Pullup or pulldown!//拉起还是拉下！
    #define SPINDLE_LASER_PWM_PIN              6  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_DIR_PIN                    5
  #elif HAS_FREE_AUX2_PINS                        // Try to use AUX 2//尝试使用辅助2
    #define SPINDLE_LASER_ENABLE_PIN          40  // Pullup or pulldown!//拉起还是拉下！
    #define SPINDLE_LASER_PWM_PIN             44  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_DIR_PIN                   65
  #endif
#endif

////
// Průša i3 MK2 Multiplexer Support//Průša i3 MK2多路复用器支持
////
//#ifndef E_MUX0_PIN//#ifndef E_MUX0_引脚
//  #define E_MUX0_PIN       58   // Y_CS_PIN//#定义E_MUX0_引脚58//Y_CS_引脚
//#endif//#恩迪夫
//#ifndef E_MUX1_PIN//#ifndef E_MUX1_引脚
//  #define E_MUX1_PIN       53   // Z_CS_PIN//#定义E_MUX1_引脚53//Z_CS_引脚
//#endif//#恩迪夫
//#ifndef E_MUX2_PIN//#ifndef E_MUX2_引脚
//  #define E_MUX2_PIN       49   // En_CS_PIN//#定义E_MUX2_引脚49//En_CS_引脚
//#endif//#恩迪夫

////////////////////////////////////////////////////
// LCDs and Controllers ////液晶显示器和控制器//
////////////////////////////////////////////////////

#if HAS_WIRED_LCD

  ////
  // LCD Display output pins//液晶显示器输出引脚
  ////
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS                       49  // CS chip select /SS chip slave select//CS芯片选择/SS芯片从属选择
    #define LCD_PINS_ENABLE                   51  // SID (MOSI)//SID（MOSI）
    #define LCD_PINS_D4                       52  // SCK (CLK) clock//时钟

  #elif BOTH(IS_NEWPANEL, PANEL_ONE)

    #define LCD_PINS_RS                       40
    #define LCD_PINS_ENABLE                   42
    #define LCD_PINS_D4                       65
    #define LCD_PINS_D5                       66
    #define LCD_PINS_D6                       44
    #define LCD_PINS_D7                       64

  #elif ENABLED(ZONESTAR_LCD)

    #define LCD_PINS_RS                       64
    #define LCD_PINS_ENABLE                   44
    #define LCD_PINS_D4                       63
    #define LCD_PINS_D5                       40
    #define LCD_PINS_D6                       42
    #define LCD_PINS_D7                       65
    #define ADC_KEYPAD_PIN                    12

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS                     27
      #define LCD_PINS_ENABLE                 29
      #define LCD_PINS_D4                     25

      #if !IS_NEWPANEL
        #define BEEPER_PIN                    37
      #endif

    #else

      #if EITHER(MKS_12864OLED, MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC                   25  // Set as output on init//在init上设置为输出
        #define LCD_PINS_RS                   27  // Pull low for 1s to init//拉低1s至初始
        // DOGM SPI LCD Support//DOGM SPI LCD支持
        #define DOGLCD_CS                     16
        #define DOGLCD_MOSI                   17
        #define DOGLCD_SCK                    23
        #define DOGLCD_A0            LCD_PINS_DC
      #else
        #define LCD_PINS_RS                   16
        #define LCD_PINS_ENABLE               17
        #define LCD_PINS_D4                   23
        #define LCD_PINS_D5                   25
        #define LCD_PINS_D6                   27
      #endif

      #define LCD_PINS_D7                     29

      #if !IS_NEWPANEL
        #define BEEPER_PIN                    33
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register//与移位寄存器相连的按钮
      // Not wired yet//还没连线
      //#define SHIFT_CLK_PIN                 38//#定义换档锁定销38
      //#define SHIFT_LD_PIN                  42//#定义SHIFT_LD_引脚42
      //#define SHIFT_OUT_PIN                 40//#定义SHIFT_OUT_引脚40
      //#define SHIFT_EN_PIN                  17//#定义SHIFT_EN_针脚17
    #endif

  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
  #endif

  ////
  // LCD Display input pins//液晶显示器输入引脚
  ////
  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN                      37

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1                       17
        #define BTN_EN2                       23
      #else
        #define BTN_EN1                       31
        #define BTN_EN2                       33
      #endif

      #define BTN_ENC                         35
      #define SD_DETECT_PIN                   49
      //#define KILL_PIN                      41//#定义压井U引脚41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN             39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                         64
      #define BTN_EN2                         59
      #define BTN_ENC                         63
      #define SD_DETECT_PIN                   42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                         47
      #define BTN_EN2                         43
      #define BTN_ENC                         32
      #define LCD_SDSS                        53
      //#define KILL_PIN                      41//#定义压井U引脚41

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                         22  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.// https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf 解释40/42。
      #define BTN_EN2                          7  // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.//22/7在坡道上未使用。22未使用，7斜坡13上的伺服0_销。
      #define BTN_ENC                         -1

      #define LCD_SDSS                        53
      #define SD_DETECT_PIN                   49

    #elif EITHER(VIKI2, miniVIKI)

      #define DOGLCD_CS                       45
      #define DOGLCD_A0                       44
      #define LCD_SCREEN_ROT_180

      #define BEEPER_PIN                      33
      #define STAT_LED_RED_PIN                32
      #define STAT_LED_BLUE_PIN               35

      #define BTN_EN1                         22
      #define BTN_EN2                          7
      #define BTN_ENC                         39

      #define SDSS                            53
      #define SD_DETECT_PIN                   -1  // Pin 49 for display SD interface, 72 for easy adapter board//引脚49用于显示SD接口，72用于简易适配器板
      //#define KILL_PIN                      31//#定义KILL_引脚31

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS                       29
      #define DOGLCD_A0                       27

      #define BEEPER_PIN                      23
      #define LCD_BACKLIGHT_PIN               33

      #define BTN_EN1                         35
      #define BTN_EN2                         37
      #define BTN_ENC                         31

      #define LCD_SDSS                        53
      #define SD_DETECT_PIN                   49
      //#define KILL_PIN                      41//#定义压井U引脚41

    #elif ENABLED(MKS_MINI_12864)

      #define DOGLCD_A0                       27
      #define DOGLCD_CS                       25

      // GLCD features//GLCD功能
      //#define LCD_CONTRAST_INIT            190//#定义LCD对比度初始值190
      // Uncomment screen orientation//取消注释屏幕方向
      //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
      //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
      //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270

      #define BEEPER_PIN                      37

      #define LCD_BACKLIGHT_PIN               65  // backlight LED on A11/D65//A11/D65上的背光LED

      #define BTN_EN1                         31
      #define BTN_EN2                         33
      #define BTN_ENC                         35
      //#define SDSS                          53//#定义SDSS 53
      #define SD_DETECT_PIN                   49
      //#define KILL_PIN                      64//#定义KILL_引脚64

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN                      42
      // not connected to a pin//未连接到pin
      #define LCD_BACKLIGHT_PIN               65  // backlight LED on A11/D65//A11/D65上的背光LED

      #define DOGLCD_A0                       44
      #define DOGLCD_CS                       66

      // GLCD features//GLCD功能
      //#define LCD_CONTRAST_INIT            190//#定义LCD对比度初始值190
      // Uncomment screen orientation//取消注释屏幕方向
      //#define LCD_SCREEN_ROT_90//#定义LCD屏幕旋转90
      //#define LCD_SCREEN_ROT_180//#定义LCD屏幕旋转180
      //#define LCD_SCREEN_ROT_270//#定义LCD屏幕旋转270

      #define BTN_EN1                         40
      #define BTN_EN2                         63
      #define BTN_ENC                         59

      #define SDSS                            53
      #define SD_DETECT_PIN                   49
      //#define KILL_PIN                      64//#定义KILL_引脚64

    #else

      // Beeper on AUX-4//AUX-4上的蜂鸣器
      #define BEEPER_PIN                      33

      // Buttons are directly attached to AUX-2//按钮直接连接到AUX-2
      #if IS_RRW_KEYPAD
        #define SHIFT_OUT_PIN                 40
        #define SHIFT_CLK_PIN                 44
        #define SHIFT_LD_PIN                  42
        #define BTN_EN1                       64
        #define BTN_EN2                       59
        #define BTN_ENC                       63
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1                       59  // AUX2 PIN 3//AUX2引脚3
        #define BTN_EN2                       63  // AUX2 PIN 4//AUX2引脚4
        #define BTN_ENC                       49  // AUX3 PIN 7//AUX3引脚7
      #else
        #define BTN_EN1                       37
        #define BTN_EN2                       35
        #define BTN_ENC                       31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN                 49
        //#define KILL_PIN                    41//#定义压井U引脚41
      #endif

    #endif

  #endif // IS_NEWPANEL//这是新小组吗

#endif
