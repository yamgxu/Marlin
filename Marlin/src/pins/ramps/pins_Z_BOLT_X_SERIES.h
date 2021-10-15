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
 *  Z-Bolt X Series board – based on Arduino Mega2560
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#if HOTENDS > 4 || E_STEPPERS > 4
  #error "Z-Bolt X Series board supports up to 4 hotends / E-steppers."
#endif

#define BOARD_INFO_NAME "Z-Bolt X Series"

////
// Servos//伺服
////
#ifndef SERVO0_PIN
  #define SERVO0_PIN                          11
#endif
#ifndef SERVO3_PIN
  #define SERVO3_PIN                           4
#endif

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                              3
#ifndef X_MAX_PIN
  #define X_MAX_PIN                            2
#endif
#define Y_MIN_PIN                             14
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            -1
#endif

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            -1
#endif

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            -1
#endif

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           -1
#endif

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           -1
#endif

// Red//红色的
#define E2_STEP_PIN                           42
#define E2_DIR_PIN                            40
#define E2_ENABLE_PIN                         65
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           -1
#endif

// Black//黑色的
#define E3_STEP_PIN                           44
#define E3_DIR_PIN                            64
#define E3_ENABLE_PIN                         66
#ifndef E3_CS_PIN
  #define E3_CS_PIN                           -1
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            13  // Analog Input//模拟输入
#define TEMP_1_PIN                            15  // Analog Input//模拟输入
#define TEMP_2_PIN                             5  // Analog Input (BLACK)//模拟输入（黑色）
#define TEMP_3_PIN                             9  // Analog Input (RED)//模拟输入（红色）
#define TEMP_BED_PIN                          14  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           7
#define HEATER_2_PIN                           6
#define HEATER_3_PIN                           5
#define HEATER_BED_PIN                         8

#define FAN_PIN                                9

////
// Misc. Functions//杂项。功能
////
#define SDSS                                  53
#define LED_PIN                               13

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         5  // Analog Input on AUX2//AUX2上的模拟输入
#endif

// Оn the servos connector//在伺服系统连接器上
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                       4
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !defined(CASE_LIGHT_PIN) && !defined(SPINDLE_LASER_ENA_PIN)
  #if NUM_SERVOS <= 1                             // Prefer the servo connector//首选伺服连接器
    #define CASE_LIGHT_PIN                     6  // Hardware PWM//硬件脉宽调制
  #elif HAS_FREE_AUX2_PINS
    #define CASE_LIGHT_PIN                    44  // Hardware PWM//硬件脉宽调制
  #endif
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#if HAS_CUTTER && !PIN_EXISTS(SPINDLE_LASER_ENA)
  #if !defined(NUM_SERVOS) || NUM_SERVOS == 0     // Prefer the servo connector//首选伺服连接器
    #define SPINDLE_LASER_ENA_PIN              4  // Pullup or pulldown!//拉起还是拉下！
    #define SPINDLE_LASER_PWM_PIN              6  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_DIR_PIN                    5
  #elif HAS_FREE_AUX2_PINS
    #define SPINDLE_LASER_ENA_PIN             40  // Pullup or pulldown!//拉起还是拉下！
    #define SPINDLE_LASER_PWM_PIN             44  // Hardware PWM//硬件脉宽调制
    #define SPINDLE_DIR_PIN                   65
  #endif
#endif

////
// TMC software SPI//TMC软件SPI
////
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                       66
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                       44
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                        64
  #endif
#endif

#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
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

  ////
  // Software serial//软件系列
  ////

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                   40
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                   63
  #endif
  #ifndef X2_SERIAL_TX_PIN
    #define X2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef X2_SERIAL_RX_PIN
    #define X2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                   59
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                   64
  #endif
  #ifndef Y2_SERIAL_TX_PIN
    #define Y2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Y2_SERIAL_RX_PIN
    #define Y2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                   42
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                   65
  #endif
  #ifndef Z2_SERIAL_TX_PIN
    #define Z2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Z2_SERIAL_RX_PIN
    #define Z2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  44
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                  66
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E1_SERIAL_RX_PIN
    #define E1_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_TX_PIN
    #define E2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_RX_PIN
    #define E2_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_TX_PIN
    #define E3_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_RX_PIN
    #define E3_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_TX_PIN
    #define E4_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_RX_PIN
    #define E4_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_TX_PIN
    #define E5_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_RX_PIN
    #define E5_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_TX_PIN
    #define E6_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_RX_PIN
    #define E6_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_TX_PIN
    #define E7_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_RX_PIN
    #define E7_SERIAL_RX_PIN                  -1
  #endif
#endif
