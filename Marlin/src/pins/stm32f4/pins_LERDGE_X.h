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

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "LERDGE X only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "Lerdge X"
#define DEFAULT_MACHINE_NAME "LERDGE"

#define STEP_TIMER                             4
#define TEMP_TIMER                             2

#define I2C_EEPROM
#define I2C_SCL_PIN                         PB8
#define I2C_SDA_PIN                         PB9
#define MARLIN_EEPROM_SIZE               0x10000  // FM24CL64 F-RAM 64K (8Kx8)//FM24CL64 F-RAM 64K（8Kx8）

// USB Flash Drive support//USB闪存驱动器支持
#define HAS_OTG_USB_HOST_SUPPORT

////
// Servos//伺服
////
//#define SERVO0_PIN                        PD13//#定义伺服0_引脚PD13

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PB12
#define Y_STOP_PIN                          PB13
#define Z_STOP_PIN                          PB14

////
// Filament runout//灯丝跳动
////
#define FIL_RUNOUT_PIN                      PE1

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
//#ifndef Z_MIN_PROBE_PIN//#ifndef Z_MIN_探头_引脚
//  #define Z_MIN_PROBE_PIN                 PB15//#定义Z_最小_探头_引脚PB15
//#endif//#恩迪夫

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PB10
#define X_DIR_PIN                           PB2
#define X_ENABLE_PIN                        PB11

#define Y_STEP_PIN                          PB0
#define Y_DIR_PIN                           PC5
#define Y_ENABLE_PIN                        PB1

#define Z_STEP_PIN                          PA7
#define Z_DIR_PIN                           PA6
#define Z_ENABLE_PIN                        PC4

#define E0_STEP_PIN                         PA4
#define E0_DIR_PIN                          PA3
#define E0_ENABLE_PIN                       PA5

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PC0   // Analog Input//模拟输入
#define TEMP_1_PIN                          -1    // Analog Input//模拟输入
#define TEMP_BED_PIN                        PC1   // Analog Input//模拟输入

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PA1
#define HEATER_1_PIN                        -1
#define HEATER_BED_PIN                      PA2

//#ifndef FAN_PIN//#ifndef风扇销
//  #define FAN_PIN                         PC15//#定义风扇插脚PC15
//#endif//#恩迪夫
#define FAN1_PIN                            PC15
#define FAN2_PIN                            PA0

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PC15  // FAN1_PIN//范努平
#endif

////
// LED / Lighting//LED/照明
////
//#define CASE_LIGHT_PIN_CI                 -1//#定义案例\灯\插脚\ CI-1
//#define CASE_LIGHT_PIN_DO                 -1//#定义案例\u灯\u引脚\u DO-1
//#define NEOPIXEL_PIN                      -1//#定义Neopix_引脚-1

////
// Misc. Functions//杂项。功能
////
#define SDSS                                PC11
#define LED_PIN                             PC7   // Alive//活的
#define PS_ON_PIN                           -1
#define KILL_PIN                            -1

// Lerdge supports auto-power off and power loss sense through a single pin.//Lerge通过单个引脚支持自动断电和断电检测。
#define POWER_LOSS_PIN                      PC14  // Power-loss / nAC_FAULT//电源丢失/nAC_故障

#define SD_SCK_PIN                          PC12
#define SD_MISO_PIN                         PC8
#define SD_MOSI_PIN                         PD2
#define SD_SS_PIN                           PC11

////
// SD support//SD支持
////
#define SDIO_SUPPORT
#define SD_DETECT_PIN                       PA8
#define SDIO_CLOCK                       4800000

////
// LCD / Controller//液晶显示器/控制器
////

// The LCD is initialized in FSMC mode//LCD在FSMC模式下初始化
#define BEEPER_PIN                          PD12

#define BTN_EN1                             PE3
#define BTN_EN2                             PE4
#define BTN_ENC                             PE2

#define TFT_RESET_PIN                       PD6
#define TFT_BACKLIGHT_PIN                   PD3

#define TFT_CS_PIN                          PD7
#define TFT_RS_PIN                          PD11

#define TOUCH_CS_PIN                        PB6
#define TOUCH_SCK_PIN                       PB3
#define TOUCH_MOSI_PIN                      PB5
#define TOUCH_MISO_PIN                      PB4
