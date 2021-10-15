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

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "Anet ET4 only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Anet ET4 1.x"
#endif

////
// EEPROM//电可擦可编程只读存储器
////

// Use one of these or SDCard-based Emulation will be used//使用其中一个或将使用基于SD卡的仿真
#if NO_EEPROM_SELECTED
  //#define SRAM_EEPROM_EMULATION                 // Use BackSRAM-based EEPROM emulation//#定义SRAM_EEPROM_仿真//使用基于后向RAM的EEPROM仿真
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation//使用基于Flash的EEPROM仿真
  //#define IIC_BL24CXX_EEPROM                    // Use I2C EEPROM onboard IC (AT24C04C, Size 4KB, PageSize 16B)//#定义IIC_BL24CXX_EEPROM//使用I2C EEPROM板载IC（AT24C04C，尺寸4KB，页面尺寸16B）
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the//通过将写操作分散到整个系统来减少延迟和闪存磨损
  // 128 kB sector allocated for EEPROM emulation.//分配给EEPROM仿真的128 kB扇区。
  #define FLASH_EEPROM_LEVELING
#elif ENABLED(IIC_BL24CXX_EEPROM)
  #define IIC_EEPROM_SDA                    PB11
  #define IIC_EEPROM_SCL                    PB10
  #define EEPROM_DEVICE_ADDRESS             0xA0
  #define MARLIN_EEPROM_SIZE              0x1000  // 4KB//4KB
#endif

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PC13
#define Y_STOP_PIN                          PE12
#define Z_STOP_PIN                          PE11

////
// Z Probe//Z探头
////
#if ENABLED(BLTOUCH)
  #error "You will need to use 24V to 5V converter and remove one resistor and capacitor from the motherboard. See https://github.com/davidtgbe/Marlin/blob/bugfix-2.0.x/docs/Tutorials/bltouch-en.md for more information. Comment out this line to proceed at your own risk."//github.com/davidtgbe/Marlin/blob/bugfix-2.0.x/docs/Tutorials/bltouch-en.md了解更多信息。注释掉这一行，继续操作的风险由您自己承担。”
  #define SERVO0_PIN                        PC3
#elif !defined(Z_MIN_PROBE_PIN)
  #define Z_MIN_PROBE_PIN                   PC3
#endif

////
// Filament Runout Sensor//灯丝偏移传感器
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA2
#endif

////
// Power Loss Detection//功率损耗检测
////
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PA8
#endif

////
// LED PIN//LED引脚
////
#define LED_PIN                             PD12

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PB6
#define X_DIR_PIN                           PB5
#define X_ENABLE_PIN                        PB7

#define Y_STEP_PIN                          PB3
#define Y_DIR_PIN                           PD6
#define Y_ENABLE_PIN                        PB4

#define Z_STEP_PIN                          PA12
#define Z_DIR_PIN                           PA11
#define Z_ENABLE_PIN                        PA15

#define E0_STEP_PIN                         PB9
#define E0_DIR_PIN                          PB8
#define E0_ENABLE_PIN                       PE0

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PA1
#define TEMP_BED_PIN                        PA4

////
// Heaters//加热器
////
#define HEATER_0_PIN                        PA0
#define HEATER_BED_PIN                      PE2

////
// Fans//扇子
////
#define FAN_PIN                             PE3   // Layer fan//层扇
#define FAN1_PIN                            PE1   // Hotend fan//热端风扇

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN               FAN1_PIN
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#define TFT_RESET_PIN                       PE6
#define TFT_CS_PIN                          PD7
#define TFT_RS_PIN                          PD13
#define TFT_INTERFACE_FSMC_8BIT

#define LCD_USE_DMA_FSMC                          // Use DMA transfers to send data to the TFT//使用DMA传输将数据发送到TFT
#define FSMC_CS_PIN                   TFT_CS_PIN
#define FSMC_RS_PIN                   TFT_RS_PIN

////
// Touch Screen//触摸屏
// https://ldm-systems.ru/f/doc/catalog/HY-TFT-2,8/XPT2046.pdf// https://ldm-systems.ru/f/doc/catalog/HY-TFT-2，8/XPT2046.pdf
////
#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PB2
  #define TOUCH_SCK_PIN                     PB0
  #define TOUCH_MOSI_PIN                    PE5
  #define TOUCH_MISO_PIN                    PE4
  #define TOUCH_INT_PIN                     PB1
#endif

#if ENABLED(ANET_ET5_TFT35)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X            17125
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y           -11307
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X                   -26
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y                   337
  #endif
  #ifndef TOUCH_ORIENTATION
    #define TOUCH_ORIENTATION     TOUCH_PORTRAIT
  #endif
#elif ENABLED(ANET_ET4_TFT28)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X           -11838
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y             8776
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X                   333
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y                   -17
  #endif
  #ifndef TOUCH_ORIENTATION
    #define TOUCH_ORIENTATION     TOUCH_PORTRAIT
  #endif
#endif

////
// SD Card//SD卡
////
//#define SDIO_SUPPORT//#定义SDIO_支持

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION         CUSTOM_CABLE
#endif

#if ENABLED(SDSUPPORT)

  #define SDIO_D0_PIN                       PC8
  #define SDIO_D1_PIN                       PC9
  #define SDIO_D2_PIN                       PC10
  #define SDIO_D3_PIN                       PC11
  #define SDIO_CK_PIN                       PC12
  #define SDIO_CMD_PIN                      PD2

  #if DISABLED(SDIO_SUPPORT)
    #define SOFTWARE_SPI
    #define SDSS                     SDIO_D3_PIN
    #define SD_SCK_PIN               SDIO_CK_PIN
    #define SD_MISO_PIN              SDIO_D0_PIN
    #define SD_MOSI_PIN             SDIO_CMD_PIN
  #endif

  #ifndef SD_DETECT_PIN
    #define SD_DETECT_PIN                   PD3
  #endif

#endif
