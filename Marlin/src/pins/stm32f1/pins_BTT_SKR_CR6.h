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
 * BigTreeTech SKR CR-6 (STM32F103RET6) board pin assignments
 */

#define DEFAULT_MACHINE_NAME "Creality3D"
#define BOARD_INFO_NAME "BTT SKR CR-6"

#include "env_validate.h"

////
// Release PB4 (Z_STEP_PIN) from JTAG NRST role//从JTAG NRST角色中释放PB4（Z_步骤_引脚）
////
#define DISABLE_DEBUG

////
// USB connect control//USB连接控制
////
#define USB_CONNECT_PIN                     PA14
#define USB_CONNECT_INVERTING              false

////
// EEPROM//电可擦可编程只读存储器
////

#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
#endif

/* I2C */
#if ENABLED(I2C_EEPROM)
  #define IIC_EEPROM_SDA                    PB7
  #define IIC_EEPROM_SCL                    PB6

  #define MARLIN_EEPROM_SIZE              0x1000  // 4KB//4KB
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE              0x1000  // 4KB//4KB
#endif

#define E2END           (MARLIN_EEPROM_SIZE - 1)  // 2KB//2KB

////
// Limit Switches//限位开关
////

#define X_STOP_PIN                          PC0
#define Y_STOP_PIN                          PC1
#define Z_STOP_PIN                          PC14  // Endtop or Probe//端盖或探头

#define FIL_RUNOUT_PIN                      PC15

////
// Probe//探测
////
#define PROBE_TARE_PIN                      PA1
#define PROBE_ACTIVATION_SWITCH_PIN         PC2   // Optoswitch to Enable Z Probe//用于启用Z探头的光电开关

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PB14
#define X_STEP_PIN                          PB13
#define X_DIR_PIN                           PB12

#define Y_ENABLE_PIN                        PB11
#define Y_STEP_PIN                          PB10
#define Y_DIR_PIN                           PB2

#define Z_ENABLE_PIN                        PB1
#define Z_STEP_PIN                          PB0
#define Z_DIR_PIN                           PC5

#define E0_ENABLE_PIN                       PD2
#define E0_STEP_PIN                         PB3
#define E0_DIR_PIN                          PB4

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PA0   // TH1//TH1
#define TEMP_BED_PIN                        PC3   // TB1//TB1

////
// Heaters / Fans//加热器/风扇
////

#define HEATER_0_PIN                        PC8   // HEATER1//加热器1
#define HEATER_BED_PIN                      PC9   // HOT BED//热床

#define FAN_PIN                             PC6   // FAN//扇子
#define FAN_SOFT_PWM

#define CONTROLLER_FAN_PIN                  PC7

////
// LCD / Controller//液晶显示器/控制器
////
#if ENABLED(CR10_STOCKDISPLAY)
  #define BTN_ENC                           PA15
  #define BTN_EN1                           PA9
  #define BTN_EN2                           PA10

  #define LCD_PINS_RS                       PB8
  #define LCD_PINS_ENABLE                   PB15
  #define LCD_PINS_D4                       PB9

  #define BEEPER_PIN                        PB5
#endif

#if HAS_TMC_UART
  /**
   * TMC2209 stepper drivers
   * Hardware serial communication ports.
   */
  #define X_HARDWARE_SERIAL  MSerial4
  #define Y_HARDWARE_SERIAL  MSerial4
  #define Z_HARDWARE_SERIAL  MSerial4
  #define E0_HARDWARE_SERIAL MSerial4

  // Default TMC slave addresses//默认TMC从机地址
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS  0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS  1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS  2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS 3
  #endif
#endif

////
// SD Card//SD卡
////

#define HAS_ONBOARD_SD

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PC4
  #define ONBOARD_SD_CS_PIN                 PA4   // Chip select for "System" SD card//“系统”SD卡的芯片选择
#endif

////
// Misc. Functions//杂项。功能
////
#define LED_CONTROL_PIN                     PA13

#ifndef NEOPIXEL_PIN
  #define NEOPIXEL_PIN                      PA8
#endif
