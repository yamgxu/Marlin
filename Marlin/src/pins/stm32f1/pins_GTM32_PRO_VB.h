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
 * Geeetech GTM32 Pro VB board pin assignments
 * http://www.geeetech.com/wiki/index.php/File:Hardware_GTM32_PRO_VB.pdf
 *
 * Also applies to GTM32 Pro VD
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME    "GTM32 Pro VB"
#endif
#define DEFAULT_MACHINE_NAME "STM32F103VET6"

#define BOARD_NO_NATIVE_USB

//#define DISABLE_DEBUG//#定义禁用调试

////
// It is required to disable JTAG function because its pins are//需要禁用JTAG功能，因为其引脚为
// used as GPIO to drive the Y axis stepper.//用作GPIO以驱动Y轴步进器。
// DO NOT ENABLE!//不要启用！
////
#define DISABLE_JTAG

////
// If you don't need the SWDIO functionality (any more), you may//如果您不再需要SWDIO功能，您可以
// disable SWD here to regain PA13/PA14 pins for other use.//在此处禁用SWD，以重新获得PA13/PA14管脚以供其他用途。
////
//#define DISABLE_JTAGSWD//#定义禁用JTAGSWD

// Ignore temp readings during development.//在开发过程中忽略温度读数。
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000//#定义2000年的假温度宽限期

// Enable EEPROM Emulation for this board as it doesn't have EEPROM//为该板启用EEPROM仿真，因为它没有EEPROM
#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define MARLIN_EEPROM_SIZE              0x1000  // 4KB//4KB
#endif

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                           PE5   // ENDSTOPS 15,17//止动块15,17
#define X_MAX_PIN                           PE4   // ENDSTOPS 16,18//止动块16,18
#define Y_MIN_PIN                           PE3   // ENDSTOPS 9,11//终点站9,11
#define Y_MAX_PIN                           PE2   // ENDSTOPS 10,12//终点站10,12
#define Z_MIN_PIN                           PE1   // ENDSTOPS 3,5//终点站3,5
#define Z_MAX_PIN                           PE0   // ENDSTOPS 4,6//止动块4,6

////
// Steppers//踏步机
////
#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PD13
#define X_ENABLE_PIN                        PA8

#define Y_STEP_PIN                          PA12
#define Y_DIR_PIN                           PA11
#define Y_ENABLE_PIN                        PA15

#define Z_STEP_PIN                          PD6
#define Z_DIR_PIN                           PD3
#define Z_ENABLE_PIN                        PB3

// Extruder stepper pins//挤出机步进销
// NOTE: Numbering here is made according to EXT connector numbers,//注：此处的编号是根据EXT连接器编号进行的，
//       the FANx_PWM line numbering in the schematics is reverse.//示意图中的FANx_PWM线编号与此相反。
//       That is, E0_*_PIN are the E2_* lines connected to E2_A1 step//也就是说，E0_*\u引脚是连接到E2_A1步骤的E2_*线
//       stick that drives the EXT0 output on the board.//驱动板上EXT0输出的棒。
////
#define E0_STEP_PIN                         PC14
#define E0_DIR_PIN                          PC13
#define E0_ENABLE_PIN                       PC15

#define E1_STEP_PIN                         PA0
#define E1_DIR_PIN                          PB6
#define E1_ENABLE_PIN                       PA1

#define E2_STEP_PIN                         PB2
#define E2_DIR_PIN                          PB11
#define E2_ENABLE_PIN                       PC4

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PB0   // EXT0 port//EXT0端口
#define HEATER_1_PIN                        PB5   // EXT1 port//EXT1端口
#define HEATER_2_PIN                        PB4   // EXT2 port//EXT2端口
#define HEATER_BED_PIN                      PB1   // CON2X3 hotbed port//CON2X3温床端口

////
// These are FAN PWM pins on EXT0..EXT2 connectors.//这些是EXT0..EXT2连接器上的风扇PWM引脚。
////
//#define FAN_PIN                           PB9   // EXT0 port//#定义风扇引脚PB9//EXT0端口
#define FAN1_PIN                            PB8   // EXT1 port//EXT1端口
#define FAN2_PIN                            PB7   // EXT2 port//EXT2端口

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                   PB9   // EXT0 port, used as main extruder fan//EXT0端口，用作挤出机主风扇
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PC2   // EXT0 port//EXT0端口
#define TEMP_1_PIN                          PC1   // EXT1 port//EXT1端口
#define TEMP_2_PIN                          PC0   // EXT2 port//EXT2端口
#define TEMP_BED_PIN                        PC3   // CON2X3 hotbed port//CON2X3温床端口

////
// Misc. Functions//杂项。功能
////
#define LED_PWM                             PD12  // External LED, pin 2 on LED labeled connector//外部LED，LED上的针脚2标有接头

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD

  #if IS_RRD_SC
    ////
    // LCD display on J2 FFC40//J2 FFC40上的LCD显示屏
    // Geeetech's LCD2004A Control Panel is very much like//Geeetech的LCD2004A控制面板非常类似
    // RepRapDiscount Smart Controller, but adds an FFC40 connector//RepraDiscount智能控制器，但添加了FFC40连接器
    ////
    #define LCD_PINS_RS                     PE6   // CS chip select /SS chip slave select//CS芯片选择/SS芯片从属选择
    #define LCD_PINS_ENABLE                 PE14  // SID (MOSI)//SID（MOSI）
    #define LCD_PINS_D4                     PD8   // SCK (CLK) clock//时钟
    #define LCD_PINS_D5                     PD9
    #define LCD_PINS_D6                     PD10
    #define LCD_PINS_D7                     PE15

  #else
    ////
    // Serial LCDs can be implemented in ExtUI//串行lcd可以在ExtUI中实现
    ////
    //#define LCD_UART_TX                   PD8//#定义LCD\U UART\U TX PD8
    //#define LCD_UART_RX                   PD9//#定义LCD\U UART\U RX PD9
  #endif

  #if HAS_MARLINUI_U8GLIB
    #ifndef BOARD_ST7920_DELAY_1
      #define BOARD_ST7920_DELAY_1 DELAY_NS(96)
    #endif
    #ifndef BOARD_ST7920_DELAY_2
      #define BOARD_ST7920_DELAY_2 DELAY_NS(48)
    #endif
    #ifndef BOARD_ST7920_DELAY_3
      #define BOARD_ST7920_DELAY_3 DELAY_NS(715)
    #endif
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

#if IS_RRD_SC
  ////
  // Geeetech's LCD2004A Control Panel is very much like//Geeetech的LCD2004A控制面板非常类似
  // RepRapDiscount Smart Controller, but adds an FFC40 connector//RepraDiscount智能控制器，但添加了FFC40连接器
  // connected with a flat wire to J2 connector on the board.//用扁线连接到板上的J2接头。
  ////
  #define BTN_EN1                           PE8
  #define BTN_EN2                           PE9
  #define BTN_ENC                           PE13

  #define GTM32_PRO_VB_USE_LCD_BEEPER
  #define GTM32_PRO_VB_USE_EXT_SDCARD
#endif

////
// Beeper//传呼机
////
#ifdef GTM32_PRO_VB_USE_LCD_BEEPER
  // This is pin 32 on J2 FFC40 and pin, goes to the beeper//这是J2 FFC40上的针脚32，针脚连接到蜂鸣器
  // on Geeetech's version of RepRapDiscount Smart Controller//关于Geeetech版本的Reprapd Discount智能控制器
  // (e.g. on Rostock 301)//（例如，在罗斯托克301上）
  #define BEEPER_PIN                        PE12
#else
  // This is the beeper on the board itself//这是电路板上的传呼机
  #define BEEPER_PIN                        PB10
#endif

/**
 * The on-board TF_CARD_SOCKET microSD card socket has no SD Detect pin wired.
 *
 * The FFC10 (SD_CARD) connector has the same pins as those routed to the FFC40 (J2)
 * connector, which usually go to the SD Card slot on the Geeetech version of the
 * RepRapDiscount Smart Controller. Both connectors have the card detect signal.
 *
 * The on-board SD card and the external card (on either SD_CARD or J2) are two
 * separate devices and can work simultaneously. Unfortunately, Marlin only supports
 * a single SPI Flash device (as of 2019-07-05) so only one is enabled here.
 */
#if ENABLED(GTM32_PRO_VB_USE_EXT_SDCARD)
  ////
  // SD Card on RepRapDiscount Smart Controller (J2) or on SD_CARD connector//Reprapd Discount智能控制器（J2）或SD卡连接器上的SD卡
  ////
  #define SD_SS_PIN                         PC11
  #define SD_SCK_PIN                        PC12
  #define SD_MOSI_PIN                       PD2
  #define SD_MISO_PIN                       PC8
  #define SD_DETECT_PIN                     PC7
#else
  ////
  // Use the on-board card socket labeled TF_CARD_SOCKET//使用标有TF_卡_插座的车载卡插座
  ////
  #define SD_SS_PIN                         PA4
  #define SD_SCK_PIN                        PA5
  #define SD_MOSI_PIN                       PA7
  #define SD_MISO_PIN                       PA6
  #define SD_DETECT_PIN                     -1    // Card detect is not connected//卡检测未连接
#endif

#define SDSS                           SD_SS_PIN

////
// ESP WiFi can be soldered to J9 connector which is wired to USART2.//ESP WiFi可以焊接到连接到USART2的J9连接器上。
// Must define WIFISUPPORT in Configuration.h for the printer.//必须在Configuration.h中为打印机定义WIFI支持。
////
#define ESP_WIFI_MODULE_COM                    2
#define ESP_WIFI_MODULE_BAUDRATE          115200
#define ESP_WIFI_MODULE_RESET_PIN           -1
