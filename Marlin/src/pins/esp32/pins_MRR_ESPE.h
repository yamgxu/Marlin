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
 * MRR ESPE pin assignments
 * MRR ESPE is a 3D printer control board based on the ESP32 microcontroller.
 * Supports 5 stepper drivers (using I2S stepper stream), heated bed,
 * single hotend, and LCD controller.
 */

#include "env_validate.h"

#if EXTRUDERS > 2 || E_STEPPERS > 2
  #error "MRR ESPE only supports two E Steppers. Comment out this line to continue."
#elif HOTENDS > 1
  #error "MRR ESPE only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "MRR ESPE"
#define BOARD_WEBSITE_URL    "github.com/maplerainresearch/MRR_ESPE"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            35
#define Y_STOP_PIN                            32
#define Z_STOP_PIN                            33

////
// Enable I2S stepper stream//启用I2S步进器流
////
#undef I2S_STEPPER_STREAM
#define I2S_STEPPER_STREAM
#define I2S_WS                                26
#define I2S_BCK                               25
#define I2S_DATA                              27
#undef LIN_ADVANCE                                // Currently, I2S stream does not work with linear advance//目前，I2S流不适用于线性推进

////
// Steppers//踏步机
////
#define X_STEP_PIN                           129
#define X_DIR_PIN                            130
#define X_ENABLE_PIN                         128
//#define X_CS_PIN                            21//#定义X_CS_引脚21

#define Y_STEP_PIN                           132
#define Y_DIR_PIN                            133
#define Y_ENABLE_PIN                         131
//#define Y_CS_PIN                            22//#定义Y_CS_引脚22

#define Z_STEP_PIN                           135
#define Z_DIR_PIN                            136
#define Z_ENABLE_PIN                         134
//#define Z_CS_PIN                             5  // SS_PIN//#定义Z_CS_引脚5//SS_引脚

#define E0_STEP_PIN                          138
#define E0_DIR_PIN                           139
#define E0_ENABLE_PIN                        137
//#define E0_CS_PIN                           21//#定义E0_CS_引脚21

#define E1_STEP_PIN                          141
#define E1_DIR_PIN                           142
#define E1_ENABLE_PIN                        140
//#define E1_CS_PIN                           22//#定义E1_CS_引脚22

#define Z2_STEP_PIN                          141
#define Z2_DIR_PIN                           142
#define Z2_ENABLE_PIN                        140
//#define Z2_CS_PIN                            5//#定义Z2_CS_引脚5

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            36  // Analog Input//模拟输入
#define TEMP_1_PIN                            34  // Analog Input//模拟输入
#define TEMP_BED_PIN                          39  // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                         145  // 2// 2
#define FAN_PIN                              146  // 15// 15
#define HEATER_BED_PIN                       144  // 4// 4

#define CONTROLLER_FAN_PIN                   147
//#define E0_AUTO_FAN_PIN                    148  // need to update Configuration_adv.h @section extruder//#定义E0_AUTO_FAN_引脚148//需要更新截面挤出机的配置
//#define E1_AUTO_FAN_PIN                    149  // need to update Configuration_adv.h @section extruder//#定义E1_AUTO_FAN_引脚149//需要更新截面挤出机的配置
#define FAN1_PIN                             149

////
// MicroSD card//MicroSD卡
////
#define SD_MOSI_PIN                           23
#define SD_MISO_PIN                           19
#define SD_SCK_PIN                            18
#define SDSS                                   5
#define USES_SHARED_SPI                           // SPI is shared by SD card with TMC SPI drivers//SD卡与TMC SPI驱动程序共享SPI

////////////////////////////////////////////////////
// LCDs and Controllers ////液晶显示器和控制器//
////////////////////////////////////////////////////

#if HAS_MARLINUI_U8GLIB

  #define LCD_PINS_RS                         13
  #define LCD_PINS_ENABLE                     17
  #define LCD_PINS_D4                         16

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN                       151

  #elif IS_RRD_FG_SC

    #define BEEPER_PIN                       151

    //#define LCD_PINS_D5                    150//#定义LCD_引脚_D5 150
    //#define LCD_PINS_D6                    152//#定义LCD_引脚_D6 152
    //#define LCD_PINS_D7                    153//#定义LCD_引脚_D7 153

  #else

    #error "Only CR10_STOCKDISPLAY and REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER are currently supported. Comment out this line to continue."

  #endif

  #define BTN_EN1                              0
  #define BTN_EN2                             12
  #define BTN_ENC                             14

#endif // HAS_MARLINUI_U8GLIB//马林努伊能说会道吗

// Hardware serial pins//硬件串行引脚
// Add the following to Configuration.h or Configuration_adv.h to assign//将以下内容添加到Configuration.h或Configuration\u adv.h以分配
// specific pins to hardware Serial1 and Serial2.//硬件Serial1和Serial2的特定引脚。
// Note: Serial2 can be defined using HARDWARE_SERIAL2_RX and HARDWARE_SERIAL2_TX but//注：可以使用硬件\u Serial2\u RX和硬件\u Serial2\u TX定义Serial2，但
// MRR ESPA does not have enough spare pins for such reassignment.//MRR ESPA没有足够的备用引脚用于此类重新分配。
//#define HARDWARE_SERIAL1_RX                 21//#定义硬件\u串行1\u RX 21
//#define HARDWARE_SERIAL1_TX                 22//#定义硬件\u SERIAL1\u TX 22
//#define HARDWARE_SERIAL2_RX                  2//#定义硬件\u序列2\u接收2
//#define HARDWARE_SERIAL2_TX                  4//#定义硬件\u序列2\u发送4
