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
 * Dreammaker Overlord v1.1 pin assignments
 */

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Overlord Controller supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#include "env_validate.h"

#define BOARD_INFO_NAME         "OVERLORD"
#define DEFAULT_MACHINE_NAME    BOARD_INFO_NAME

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            24
#define Y_STOP_PIN                            28
#define Z_MIN_PIN                             46
#define Z_MAX_PIN                             32

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     46  // JP4, Tfeed1//JP4，Tfeed1
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_PIN                      44  // JP3, Tfeed2//JP3，Tfeed2
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            25
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          27

#define Y_STEP_PIN                            31
#define Y_DIR_PIN                             33
#define Y_ENABLE_PIN                          29

#define Z_STEP_PIN                            37
#define Z_DIR_PIN                             39
#define Z_ENABLE_PIN                          35

#define E0_STEP_PIN                           43
#define E0_DIR_PIN                            45
#define E0_ENABLE_PIN                         41

#define E1_STEP_PIN                           49
#define E1_DIR_PIN                            47
#define E1_ENABLE_PIN                         48

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             8  // Analog Input//模拟输入
#define TEMP_1_PIN                             9  // Analog Input - Redundant temp sensor//模拟输入-冗余温度传感器
#define TEMP_2_PIN                            12  // Analog Input//模拟输入
#define TEMP_3_PIN                            14  // Analog Input//模拟输入
#define TEMP_BED_PIN                          10  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           2
#define HEATER_1_PIN                           3
#define HEATER_BED_PIN                         4

#define FAN_PIN                                7  // material cooling fan//物料冷却风扇

////
// SD Card//SD卡
////
#define SDSS                                  53
#define SD_DETECT_PIN                         38

////
// Misc. Functions//杂项。功能
////
#define LED_PIN                               13  // On PCB status led//PCB上的状态指示灯
#define PS_ON_PIN                             12  // For stepper/heater/fan power. Active HIGH.//用于步进电机/加热器/风扇电源。活跃高。
#define POWER_LOSS_PIN                        34  // Power check - whether hotends/steppers/fans have power//电源检查-热端/步进电机/风扇是否通电

#if ENABLED(BATTERY_STATUS_AVAILABLE)
  #undef BATTERY_STATUS_PIN
  #define BATTERY_STATUS_PIN                  26  // Status of power loss battery, whether it is charged (low) or charging (high)//电池失电状态，无论是充电（低）还是充电（高）
#endif
#if ENABLED(INPUT_VOLTAGE_AVAILABLE)
  #undef VOLTAGE_DETECTION_PIN
  #define VOLTAGE_DETECTION_PIN               11  // Analog Input - ADC Voltage level of main input//模拟输入-主输入的ADC电压电平
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_MARLINUI_U8GLIB
  // OVERLORD OLED pins//霸王OLED引脚
  #define LCD_PINS_RS                         20
  #define LCD_PINS_D5                         21
  #define LCD_PINS_ENABLE                     15
  #define LCD_PINS_D4                         14
  #define LCD_PINS_D6                          5
  #define LCD_PINS_D7                          6
  #ifndef LCD_RESET_PIN
    #define LCD_RESET_PIN                      5  // LCD_PINS_D6//LCD_引脚_D6
  #endif
#endif

#if IS_NEWPANEL
  #define BTN_ENC                             16  // Enter Pin//输入Pin码
  #define BTN_UP                              19  // Button UP Pin//纽扣销
  #define BTN_DWN                             17  // Button DOWN Pin//按钮下销
#endif

// Additional connectors/pins on the Overlord V1.X board//霸王V1.X板上的其他连接器/引脚
#define PCB_VERSION_PIN                       22
#define APPROACH_PIN                          11  // JP7, Tpd//JP7，Tpd
#define GATE_PIN                              36  // Threshold, JP6, Tg//阈值，JP6，Tg
