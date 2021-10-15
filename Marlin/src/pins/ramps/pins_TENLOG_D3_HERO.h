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
 * Tenlog pin assignments
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Tenlog supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "Tenlog D3 Hero"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

////
// Servos//伺服
////
#define SERVO0_PIN                            11
#define SERVO1_PIN                             6
#define SERVO2_PIN                            -1  // Original pin 5 used for hotend fans//用于热端风扇的原装针脚5
#define SERVO3_PIN                             4

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             14
//#define Y_MAX_PIN                           15  // Connected to "DJ" plug on extruder heads//#定义Y_MAX_引脚15//连接至挤出机头上的“DJ”插头
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     15  // Ramps is normally 32//坡道通常为32
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
//#ifndef X_CS_PIN//#ifndef X_CS_引脚
  //#define X_CS_PIN                          53//#定义X_CS_引脚53
//#endif//#恩迪夫

#define X2_STEP_PIN                           36
#define X2_DIR_PIN                            34
#define X2_ENABLE_PIN                         30
//#ifndef X2_CS_PIN//#ifndef X2_CS_引脚
  //#define X2_CS_PIN                         53//#定义X2_CS_引脚53
//#endif//#恩迪夫

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
//#ifndef Y_CS_PIN//#ifndef Y_CS_引脚
 //#define Y_CS_PIN                           49//#定义Y_CS_引脚49
//#endif//#恩迪夫

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62
//#ifndef Z_CS_PIN//#ifndef Z_CS_引脚
 //#define Z_CS_PIN                           40//#定义Z_CS_引脚40
//#endif//#恩迪夫

#define Z2_STEP_PIN                           65
#define Z2_DIR_PIN                            66
#define Z2_ENABLE_PIN                         64
//#ifndef Z2_CS_PIN//#ifndef Z2_CS_引脚
 //#define Z2_CS_PIN                          40//#定义Z2_CS_引脚40
//#endif//#恩迪夫

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
//#ifndef E0_CS_PIN//#ifndef E0_CS_引脚
  //define E0_CS_PIN                          42//定义E0_CS_引脚42
//#endif//#恩迪夫

#define E1_STEP_PIN                           57
#define E1_DIR_PIN                            58
#define E1_ENABLE_PIN                         59
//#ifndef E1_CS_PIN//#ifndef E1_CS_引脚
  //define E1_CS_PIN                          44//定义E1_CS_引脚44
//#endif//#恩迪夫

//#define E2_STEP_PIN                         42//#定义E2_步骤_针脚42
//#define E2_DIR_PIN                          43//#定义E2_DIR_引脚43
//#define E2_ENABLE_PIN                       44//#定义E2_启用_引脚44

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            13  // Analog Input//模拟输入
#define TEMP_1_PIN                            15  // Analog Input//模拟输入
#define TEMP_BED_PIN                          14  // Analog Input//模拟输入

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS_PIN                      -1  // Don't use 53 if using Display/SD card//如果使用显示卡/SD卡，请不要使用53
#else
  #define MAX6675_SS_PIN                      -1  // Don't use 49 (SD_DETECT_PIN)//不要使用49（SD_DETECT_引脚）
#endif

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                          11
#define HEATER_BED_PIN                         8

#define FAN_PIN                                9
#define FAN1_PIN                               5  // Normall this would be a servo pin//正常情况下，这将是一个伺服销

// XXX Runout support unknown?//XXX输出支持未知？
//#define NUM_RUNOUT_SENSORS                   0//#定义传感器数量\u跳动\u 0
//#define FIL_RUNOUT_PIN                      22//#定义FIL_跳动针22
//#define FIL_RUNOUT2_PIN                     21//#定义FIL_输出2_引脚21

////
// Misc. Functions//杂项。功能
////
//#define CASE_LIGHT_PIN                       5//#定义案例灯针脚5
#define SDSS                                  53
//#ifndef LED_PIN//#ifndef LED_引脚
  //#define LED_PIN                           13//#定义LED_引脚13
//#endif//#恩迪夫

//#define SPINDLE_LASER_PWM_PIN               -1  // Hardware PWM//#定义主轴\激光器\脉宽调制\引脚-1//硬件脉宽调制
//#define SPINDLE_LASER_ENA_PIN                4  // Pullup!//#定义主轴\u激光器\u ENA\u引脚4//上拉！

// Use the RAMPS 1.4 Analog input 5 on the AUX2 connector//使用AUX2接头上的斜坡1.4模拟输入5
//#define FILWIDTH_PIN                         5  // Analog Input//#定义FILU引脚5//模拟输入

////
// LCD / Controller//液晶显示器/控制器
////

//#if IS_RRD_SC//#如果它是_RRD(SC),

#define LCD_PINS_RS                           -1
#define LCD_PINS_ENABLE                       -1
#define LCD_PINS_D4                           -1
#define LCD_PINS_D5                           -1
#define LCD_PINS_D6                           -1
#define LCD_PINS_D7                           -1
//#define BTN_EN1                             31//#定义BTN_EN1 31
//#define BTN_EN2                             33//#定义BTN_EN2 33
//#define BTN_ENC                             35//#定义BTN_ENC 35
#define SD_DETECT_PIN                         49
//#ifndef KILL_PIN//#ifndef压井销
  //#define KILL_PIN                          41//#定义压井U引脚41
//#endif//#恩迪夫
//#ifndef BEEPER_PIN//#ifndef蜂鸣器\u引脚
#define BEEPER_PIN                            -1
//#endif//#恩迪夫

//#endif // IS_RRD_SC//#endif//IS\u RRD\u SC
