/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "LERDGE K supports up to 2 hotends / E-steppers."
#endif

#define BOARD_INFO_NAME      "Lerdge K"
#define DEFAULT_MACHINE_NAME "LERDGE"

// EEPROM//电可擦可编程只读存储器
#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C//强制使用软件I2C
  #define I2C_SCL_PIN                       PG14
  #define I2C_SDA_PIN                       PG13
  #define MARLIN_EEPROM_SIZE             0x10000
#endif

// USB Flash Drive support//USB闪存驱动器支持
#define HAS_OTG_USB_HOST_SUPPORT

////
// Servos//伺服
////
#define SERVO0_PIN                          PB11

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PG3
#define Y_STOP_PIN                          PG4
#define Z_STOP_PIN                          PG5

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
//#ifndef Z_MIN_PROBE_PIN//#ifndef Z_MIN_探头_引脚
//  #define Z_MIN_PROBE_PIN                 PG6//#定义Z_最小_探针_引脚PG6
//#endif//#恩迪夫

////
// Filament runout//灯丝跳动
////
#define FIL_RUNOUT_PIN                      PE5
#define FIL_RUNOUT2_PIN                     PE6

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PG1
#define X_DIR_PIN                           PB10
#define X_ENABLE_PIN                        PG0
//#ifndef X_CS_PIN//#ifndef X_CS_引脚
//  #define X_CS_PIN                        PE0//#定义X_CS#u引脚PE0
//#endif//#恩迪夫

#define Y_STEP_PIN                          PF14
#define Y_DIR_PIN                           PF15
#define Y_ENABLE_PIN                        PF13
//#ifndef Y_CS_PIN//#ifndef Y_CS_引脚
//  #define Y_CS_PIN                        PE1//#定义Y#U CS#U引脚PE1
//#endif//#恩迪夫

#define Z_STEP_PIN                          PF11
#define Z_DIR_PIN                           PF12
#define Z_ENABLE_PIN                        PC5
//#ifndef Z_CS_PIN//#ifndef Z_CS_引脚
//  #define Z_CS_PIN                        PE2//#定义Z_CS#u引脚PE2
//#endif//#恩迪夫

#define E0_STEP_PIN                         PC14
#define E0_DIR_PIN                          PC13
#define E0_ENABLE_PIN                       PC15
//#ifndef E0_CS_PIN//#ifndef E0_CS_引脚
//  #define E0_CS_PIN                       PE3//#定义E0_CS_引脚PE3
//#endif//#恩迪夫

#define E1_STEP_PIN                         PF1
#define E1_DIR_PIN                          PF0
#define E1_ENABLE_PIN                       PF2
//#ifndef E1_CS_PIN//#ifndef E1_CS_引脚
//  #define E1_CS_PIN                       PE4//#定义E1_CS#u引脚PE4
//#endif//#恩迪夫

//#define E2_STEP_PIN                       PF4  // best guess//#定义E2_步骤_引脚PF4//最佳猜测
//#define E2_DIR_PIN                        PF3  // best guess//#定义E2_DIR_引脚PF3//最佳猜测
//#define E2_ENABLE_PIN                     PF5  // best guess//#定义E2_启用_引脚PF5//最佳猜测
//#ifndef E2_CS_PIN//#ifndef E2_CS_引脚
//  #define E2_CS_PIN                       PB2  // best guess//#定义E2_CS_引脚PB2//最佳猜测
//#endif//#恩迪夫

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   */
  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                 PB2
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                 PB2
  #endif
  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                 PE2
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                 PE2
  #endif
  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                 PE3
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                 PE3
  #endif
  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                PE4
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                PE4
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                PE1
  #endif
  #ifndef E1_SERIAL_RX_PIN
    #define E1_SERIAL_RX_PIN                PE1
  #endif
  #ifndef EX_SERIAL_TX_PIN
    #define E2_SERIAL_TX_PIN                PE0
  #endif
  #ifndef EX_SERIAL_RX_PIN
    #define E2_SERIAL_RX_PIN                PE0
  #endif
  // Reduce baud rate to improve software serial reliability//降低波特率以提高软件串行可靠性
  #define TMC_BAUD_RATE                    19200
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PC1   // Analog Input//模拟输入
#define TEMP_1_PIN                          PC2   // Analog Input//模拟输入
#define TEMP_BED_PIN                        PC0   // Analog Input//模拟输入

// Lergde-K can choose thermocouple/thermistor mode in software.//Lergde-K可在软件中选择热电偶/热敏电阻模式。
// For use with thermistors, these pins must be OUT/LOW.//与热敏电阻一起使用时，这些引脚必须为OUT/LOW。
// This is done automatically.//这是自动完成的。
#define TEMP_0_TR_ENABLE_PIN                PF10
#define TEMP_1_TR_ENABLE_PIN                PF9

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PA1
#define HEATER_1_PIN                        PA0
#define HEATER_BED_PIN                      PA2

#ifndef FAN_PIN
  #define FAN_PIN                           PF7
#endif

#define FAN1_PIN                            PF6

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PB1
#endif

#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN                   PB0
#endif

#define CONTROLLER_FAN_PIN                  PF8

////
// LED / Lighting//LED/照明
////
//#define CASE_LIGHT_PIN_CI                 -1//#定义案例\灯\插脚\ CI-1
//#define CASE_LIGHT_PIN_DO                 -1//#定义案例\u灯\u引脚\u DO-1
//#define NEOPIXEL_PIN                      -1//#定义Neopix_引脚-1
#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                     PB8   // swap R and G pin for compatibility with real wires//交换R和G引脚以与实际导线兼容
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                     PB7
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                     PB9
#endif

////
// SD support//SD支持
////
#define SDIO_SUPPORT
#define SDIO_CLOCK                       4800000

////
// Misc. Functions//杂项。功能
////
#define SDSS                                PC11
#define LED_PIN                             PA15  // Alive//活的
#define PS_ON_PIN                           PA4
#define KILL_PIN                            -1
#define POWER_LOSS_PIN                      PA4   // Power-loss / nAC_FAULT//电源丢失/nAC_故障

#define SD_SCK_PIN                          PC12
#define SD_MISO_PIN                         PC8
#define SD_MOSI_PIN                         PD2
#define SD_SS_PIN                           PC11

#define SD_DETECT_PIN                       PA8
#define BEEPER_PIN                          PC7

////
// LCD / Controller//液晶显示器/控制器
////

#if HAS_FSMC_TFT
  //#define TFT_DRIVER             LERDGE_ST7796//#定义TFT_驱动器LERDGE_ST7796

  #define TFT_RESET_PIN                     PD6
  #define TFT_BACKLIGHT_PIN                 PD3

  #define TFT_CS_PIN                        PD7
  #define TFT_RS_PIN                        PD11

  #define TOUCH_CS_PIN                      PG15
  #define TOUCH_SCK_PIN                     PB3
  #define TOUCH_MOSI_PIN                    PB5
  #define TOUCH_MISO_PIN                    PB4
#endif

#if IS_NEWPANEL
  #define BTN_EN1                           PG10
  #define BTN_EN2                           PG11
  #define BTN_ENC                           PG9
#endif
