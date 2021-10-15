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
 * AZTEEG_X3_PRO (Arduino Mega) pin assignments
 */

#define REQUIRE_MEGA2560
#include "env_validate.h"

#if HOTENDS > 5 || E_STEPPERS > 5
  #error "Azteeg X3 Pro supports up to 5 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "Azteeg X3 Pro"

////
// RAMPS pins overrides//斜坡引脚覆盖
////

////
// Servos//伺服
////
// Tested this pin with bed leveling on a Delta with 1 servo.//在带有1个伺服的三角架上用床身调平对该销进行了测试。
// Physical wire attachment on EXT1: GND, 5V, D47.//EXT1上的物理导线连接：接地，5V，D47。
////
#define SERVO0_PIN                            47

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                             3
#define Y_STOP_PIN                            14
#define Z_STOP_PIN                            18

#ifndef FAN_PIN
  #define FAN_PIN                              6
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !PIN_EXISTS(CASE_LIGHT)
  #define CASE_LIGHT_PIN                      44
#endif

////
// Import RAMPS 1.4 pins//导入坡道1.4引脚
////
#include "pins_RAMPS.h"

// DIGIPOT slave addresses//DIGIPOT从机地址
#ifndef DIGIPOT_I2C_ADDRESS_A
  #define DIGIPOT_I2C_ADDRESS_A             0x2C  // unshifted slave address for first DIGIPOT 0x2C (0x58 <- 0x2C << 1)//第一个数字端口0x2C的未移位从机地址（0x58<-0x2C<<1）
#endif
#ifndef DIGIPOT_I2C_ADDRESS_B
  #define DIGIPOT_I2C_ADDRESS_B             0x2E  // unshifted slave address for second DIGIPOT 0x2E (0x5C <- 0x2E << 1)//第二个数字端口0x2E（0x5C<-0x2E<<1）的未移位从属地址
#endif

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     18
#endif

////
// Steppers//踏步机
////
#define E2_STEP_PIN                           23
#define E2_DIR_PIN                            25
#define E2_ENABLE_PIN                         40

#define E3_STEP_PIN                           27
#define E3_DIR_PIN                            29
#define E3_ENABLE_PIN                         41

#define E4_STEP_PIN                           43
#define E4_DIR_PIN                            37
#define E4_ENABLE_PIN                         42

////
// Temperature Sensors//温度传感器
////
#define TEMP_2_PIN                            12  // Analog Input//模拟输入
#define TEMP_3_PIN                            11  // Analog Input//模拟输入
#define TEMP_4_PIN                            10  // Analog Input//模拟输入
#define TC1                                    4  // Analog Input (Thermo couple on Azteeg X3Pro)//模拟输入（Azteeg X3Pro上的热电偶）
#define TC2                                    5  // Analog Input (Thermo couple on Azteeg X3Pro)//模拟输入（Azteeg X3Pro上的热电偶）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_2_PIN                          16
#define HEATER_3_PIN                          17
#define HEATER_4_PIN                           4
#define HEATER_5_PIN                           5
#define HEATER_6_PIN                           6
#define HEATER_7_PIN                          11

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN                   4  // Pin used for the fan to cool motherboard (-1 to disable)//用于风扇冷却主板的引脚（-1禁用）
#endif

////
// Auto fans//汽车风扇
////
#define AUTO_FAN_PIN                           5
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E3_AUTO_FAN_PIN
  #define E3_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#undef BEEPER_PIN
#define BEEPER_PIN                            33

#if ANY(VIKI2, miniVIKI)
  #undef SD_DETECT_PIN
  #define SD_DETECT_PIN                       49  // For easy adapter board//为方便适配器板
  #undef BEEPER_PIN
  #define BEEPER_PIN                          12  // 33 isn't physically available to the LCD display//33在物理上不可用于LCD显示器
#else
  #define STAT_LED_RED_PIN                    32
  #define STAT_LED_BLUE_PIN                   35
#endif

////
// Misc. Functions//杂项。功能
////
#if ENABLED(CASE_LIGHT_ENABLE) && PIN_EXISTS(CASE_LIGHT) && defined(DOGLCD_A0) && DOGLCD_A0 == CASE_LIGHT_PIN
  #undef DOGLCD_A0                                // Steal pin 44 for the case light; if you have a Viki2 and have connected it//偷取箱灯的针脚44；如果您有一个Viki2并连接了它
  #define DOGLCD_A0                           57  // following the Panucatt wiring diagram, you may need to tweak these pin assignments//按照Panucatt接线图，您可能需要调整这些引脚分配
                                // as the wiring diagram uses pin 44 for DOGLCD_A0//因为接线图使用引脚44作为DOGLCD_A0
#endif

////
// M3/M4/M5 - Spindle/Laser Control//M3/M4/M5-主轴/激光控制
////
#undef SPINDLE_LASER_PWM_PIN                      // Definitions in pins_RAMPS.h are no good with the AzteegX3pro board//pins_RAMPS.h中的定义对于AzteegX3pro板不好
#undef SPINDLE_LASER_ENA_PIN
#undef SPINDLE_DIR_PIN

#if HAS_CUTTER                                    // EXP2 header//EXP2头
  #if ANY(VIKI2, miniVIKI)
    #define BTN_EN2                           31  // Pin 7 needed for Spindle PWM//主轴PWM所需的针脚7
  #endif
  #define SPINDLE_LASER_PWM_PIN                7  // Hardware PWM//硬件脉宽调制
  #define SPINDLE_LASER_ENA_PIN               20  // Pullup!//拉起！
  #define SPINDLE_DIR_PIN                     21
#endif
