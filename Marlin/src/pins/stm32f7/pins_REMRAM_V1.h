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

#if NOT_TARGET(STM32F7xx)
  #error "Oops! Select an STM32F7 board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME      "RemRam v1"
#define DEFAULT_MACHINE_NAME "RemRam"

#if NO_EEPROM_SELECTED
  #define SRAM_EEPROM_EMULATION                   // Emulate the EEPROM using Backup SRAM//使用备份SRAM模拟EEPROM
#endif

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "RemRam only supports one hotend / E-stepper. Comment out this line to continue."
#endif

////
// Limit Switches//限位开关
////
#if DISABLED(SENSORLESS_HOMING)
  #define X_MIN_PIN                           58
  #define X_MAX_PIN                           59
  #define Y_MIN_PIN                           60
  #define Y_MAX_PIN                           61
  #define Z_MIN_PIN                           62
  #define Z_MAX_PIN                           63
#else
  #define X_STOP_PIN                          36
  #define Y_STOP_PIN                          39
  #define Z_MIN_PIN                           62
  #define Z_MAX_PIN                           42
#endif

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     26  // EXT_D1//分机D1
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            22
#define X_DIR_PIN                             35
#define X_ENABLE_PIN                          34
#define X_CS_PIN                              14

#define Y_STEP_PIN                            23
#define Y_DIR_PIN                             38
#define Y_ENABLE_PIN                          37
#define Y_CS_PIN                              15

#define Z_STEP_PIN                            24
#define Z_DIR_PIN                             41
#define Z_ENABLE_PIN                          40
#define Z_CS_PIN                              16

#define E0_STEP_PIN                           25
#define E0_DIR_PIN                            44
#define E0_ENABLE_PIN                         43
#define E0_CS_PIN                             10

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            64  // THERM_1//THERM_1
#define TEMP_1_PIN                            65  // THERM_2//THERM_2
#define TEMP_BED_PIN                          66  // THERM_3//热释光3

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          33
#define HEATER_BED_PIN                        31

#ifndef FAN_PIN
  #define FAN_PIN                             30  // "FAN1"//“FAN1”
#endif
#define FAN1_PIN                              32  // "FAN2"//“风扇2”

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                     32
#endif

////
// Servos//伺服
////
#define SERVO0_PIN                            26  // PWM_EXT1//PWM_EXT1
#define SERVO1_PIN                            27  // PWM_EXT2//PWM_EXT2

#define SDSS                                  57  // Onboard SD card reader//车载SD卡读卡器
//#define SDSS                                 9  // LCD SD card reader//#定义SDSS 9//LCD SD卡读卡器
#define LED_PIN                               21  // STATUS_LED//状态指示灯

////
// LCD / Controller//液晶显示器/控制器
////
#define SD_DETECT_PIN                         56  // SD_CARD_DET//SD卡数据
#define BEEPER_PIN                            46  // LCD_BEEPER//液晶寻呼机
#define LCD_PINS_RS                           49  // LCD_RS//液晶显示器
#define LCD_PINS_ENABLE                       48  // LCD_EN//液晶显示器
#define LCD_PINS_D4                           50  // LCD_D4//液晶显示器D4
#define LCD_PINS_D5                           51  // LCD_D5//液晶显示器D5
#define LCD_PINS_D6                           52  // LCD_D6//液晶显示器D6
#define LCD_PINS_D7                           53  // LCD_D7//液晶显示器D7
#define BTN_EN1                               54  // BTN_EN1//BTN_EN1
#define BTN_EN2                               55  // BTN_EN2//BTN_EN2
#define BTN_ENC                               47  // BTN_ENC//BTN_ENC

////
// Timers//计时器
////

#define STEP_TIMER                             2
