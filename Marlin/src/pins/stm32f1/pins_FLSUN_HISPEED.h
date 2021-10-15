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
 * FLSUN HiSpeed V1 (STM32F103VET6) board pin assignments
 * FLSun Hispeed (clone MKS_Robin_miniV2) board.
 *
 * MKS Robin Mini USB uses UART3 (PB10-TX, PB11-RX)
 * #define SERIAL_PORT_2 3
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 1 || E_STEPPERS > 1
  #error "FLSUN HiSpeedV1 only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "FLSun HiSpeedV1"
#define BOARD_WEBSITE_URL    "github.com/Foxies-CSTL"

#define BOARD_NO_NATIVE_USB

// Avoid conflict with TIMER_SERVO when using the STM32 HAL//使用STM32 HAL时，避免与定时器_伺服发生冲突
#define TEMP_TIMER 5

////
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role//从JTAG NRST角色中释放PB4（Y_启用_引脚）
////
#define DISABLE_DEBUG

////
// EEPROM//电可擦可编程只读存储器
////
#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB//2KB
#endif

////
// SPI//SPI
// Note: FLSun Hispeed (clone MKS_Robin_miniV2) board is using SPI2 interface.//注：FLSun Hispeed（克隆MKS_Robin_miniV2）板使用SPI2接口。
////
#define SD_SCK_PIN                          PB13  // SPI2//SPI2
#define SD_MISO_PIN                         PB14  // SPI2//SPI2
#define SD_MOSI_PIN                         PB15  // SPI2//SPI2
#define SPI_DEVICE 2

// SPI Flash//SPI闪光
#define HAS_SPI_FLASH                          1
#define SPI_FLASH_SIZE                 0x1000000  // 16MB//16MB

#if HAS_SPI_FLASH
  // SPI 2//SPI 2
  #define W25QXX_CS_PIN                     PB12  // SPI2_NSS / Flash chip-select//SPI2\u NSS/Flash芯片选择
  #define W25QXX_MOSI_PIN                   PB15
  #define W25QXX_MISO_PIN                   PB14
  #define W25QXX_SCK_PIN                    PB13
#endif

////
// Servos//伺服
////
//#define SERVO0_PIN                        PA8   // use IO0 to enable BLTOUCH support/remove Mks_Wifi//#定义SERVO0_引脚PA8//使用IO0启用BLTOUCH支持/删除Mks_Wifi

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PA15  // -X//-X
#define Y_STOP_PIN                          PA12  // -Y//-Y
#define Z_MIN_PIN                           PA11  // -Z//-Z
#define Z_MAX_PIN                           PC4   // +Z//+Z

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN            MT_DET_1_PIN
#endif

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PE4   // X_EN//休恩
#define X_STEP_PIN                          PE3   // X_STEP//X_步
#define X_DIR_PIN                           PE2   // X_DIR//X_DIR

#define Y_ENABLE_PIN                        PE1   // Y_EN//尤恩
#define Y_STEP_PIN                          PE0   // Y_STEP//Y_步
#define Y_DIR_PIN                           PB9   // Y_DIR//尤迪尔

#define Z_ENABLE_PIN                        PB8   // Z_EN//苏恩
#define Z_STEP_PIN                          PB5   // Z_STEP//Z_阶
#define Z_DIR_PIN                           PB4   // Z_DIR//Z_DIR

#define E0_ENABLE_PIN                       PB3   // E0_EN//E0_EN
#define E0_STEP_PIN                         PD6   // E0_STEP//E0_步
#define E0_DIR_PIN                          PD3   // E0_DIR//E0_目录

/**
 * FLSUN Hi-Speed has no hard-wired UART pins for TMC drivers.
 * Several wiring options are provided below, defaulting to
 * to the most compatible.
 */
#if HAS_TMC_UART
  // SoftwareSerial with one pin per driver//软件串行，每个驱动器一个引脚
  // Compatible with TMC2208 and TMC2209 drivers//与TMC2208和TMC2209驱动程序兼容
  #define X_SERIAL_TX_PIN                   PA10  // RXD1//RXD1
  #define X_SERIAL_RX_PIN                   PA10  // RXD1//RXD1
  #define Y_SERIAL_TX_PIN                   PA9   // TXD1//TXD1
  #define Y_SERIAL_RX_PIN                   PA9   // TXD1//TXD1
  #define Z_SERIAL_TX_PIN                   PC7   // IO1//IO1
  #define Z_SERIAL_RX_PIN                   PC7   // IO1//IO1
  #define TMC_BAUD_RATE                    19200
#else
  // Motor current PWM pins//电机电流PWM引脚
  #define MOTOR_CURRENT_PWM_XY_PIN          PA6   // VREF2/3 CONTROL XY//VREF2/3控制XY
  #define MOTOR_CURRENT_PWM_Z_PIN           PA7   // VREF4 CONTROL Z//VREF4控制Z
  #define MOTOR_CURRENT_PWM_RANGE           1500  // (255 * (1000mA / 65535)) * 257 = 1000 is equal 1.6v Vref in turn equal 1Amp//（255*（1000mA/65535））*257=1000等于1.6v Vref，反过来等于1MP
  #ifndef DEFAULT_PWM_MOTOR_CURRENT
    #define DEFAULT_PWM_MOTOR_CURRENT { 800, 800, 800 }
  #endif

  /**
   * MKS Robin_Wifi or another ESP8266 module
   *
   *      __ESP(M1)__       -J1-
   *  GND| 15 | | 08 |+3v3  (22)  RXD1      (PA10)
   *     | 16 | | 07 |MOSI  (21)  TXD1      (PA9)   Active LOW, probably OK to leave floating
   *  IO2| 17 | | 06 |MISO  (19)  IO1       (PC7)   Leave as unused (ESP3D software configures this with a pullup so OK to leave as floating)
   *  IO0| 18 | | 05 |CLK   (18)  IO0       (PA8)   Must be HIGH (ESP3D software configures this with a pullup so OK to leave as floating)
   *  IO1| 19 | | 03 |EN    (03)  WIFI_EN           Must be HIGH for module to run
   *     | nc | | nc |      (01)  WIFI_CTRL (PA5)
   *   RX| 21 | | nc |
   *   TX| 22 | | 01 |RST
   *       ￣￣ AE￣￣
   */
  // Module ESP-WIFI//模块ESP-WIFI
  #define ESP_WIFI_MODULE_COM                  2  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this//还必须将串行_端口或串行_端口_2设置为该值
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2//必须使用与串行端口和串行端口2相同的波特率
  #define ESP_WIFI_MODULE_RESET_PIN         PA5   // WIFI CTRL/RST//WIFI CTRL/RST
  #define ESP_WIFI_MODULE_ENABLE_PIN        -1
  #define ESP_WIFI_MODULE_TXD_PIN           PA9   // MKS or ESP WIFI RX PIN//MKS或ESP WIFI接收PIN码
  #define ESP_WIFI_MODULE_RXD_PIN           PA10  // MKS or ESP WIFI TX PIN//MKS或ESP WIFI发送PIN码
#endif

////
// EXTRUDER//挤出机
////
#if AXIS_DRIVER_TYPE_E0(TMC2208) || AXIS_DRIVER_TYPE_E0(TMC2209)
  #define E0_SERIAL_TX_PIN                  PA8   // IO0//IO0
  #define E0_SERIAL_RX_PIN                  PA8   // IO0//IO0
  #define TMC_BAUD_RATE                    19200
#else
  // Motor current PWM pins//电机电流PWM引脚
  #define MOTOR_CURRENT_PWM_E_PIN           PB0   // VREF1 CONTROL E//VREF1控制E
  #define MOTOR_CURRENT_PWM_RANGE           1500  // (255 * (1000mA / 65535)) * 257 = 1000 is equal 1.6v Vref in turn equal 1Amp//（255*（1000mA/65535））*257=1000等于1.6v Vref，反过来等于1MP
  #ifndef DEFAULT_PWM_MOTOR_CURRENT
   #define DEFAULT_PWM_MOTOR_CURRENT { 800, 800, 800 }
  #endif
#endif

////
// Temperature Sensors (THM)//温度传感器（THM）
////
#define TEMP_0_PIN                          PC1   // TEMP_E0//温度E0
#define TEMP_BED_PIN                        PC0   // TEMP_BED//临时床

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PC3   // HEATER_E0//加热器_E0
#define HEATER_BED_PIN                      PA0   // HEATER_BED-WKUP//加热器床上工作

#define FAN_PIN                             PB1   // E_FAN//欧凡

////
// Misc. Functions//杂项。功能
////
//#define POWER_LOSS_PIN                    PA1   // PW_SO//#定义电源损耗引脚PA1//PW\U SO
#if ENABLED(BACKUP_POWER_SUPPLY)
  #define POWER_LOSS_PIN                    PA2   // PW_DET (UPS) MKSPWC//PW_DET（UPS）MKSPWC
#endif

/**
 *    Connector J2
 *      -------
 * DIO O|1   2|O  3v3
 * CSK O|3   5|O  GND
 * RST O|5   6|O  GND
 *      -------
 */
//#define SW_DIO                            PA13//#定义SW_DIO PA13
//#define SW_CLK                            PA14//#定义SW_CLK PA14
//#define SW_RST                            NRST  // (14)//#定义SW_RST NRST/（14）

////
// Power Supply Control//电源控制
////
#if ENABLED(PSU_CONTROL)
  #define KILL_PIN                          PA2   // PW_DET//普华永道
  #define KILL_PIN_INVERTING                true
  //#define PS_ON_PIN                       PA3   // PW_CN /PW_OFF//#定义PS_ON_PIN PA3//PW_CN/PW_OFF
#endif

#define MT_DET_1_PIN                        PA4   // MT_DET//德特山
#define MT_DET_2_PIN                        PE6   // FALA_CRTL//法拉乌尔酒店
#define MT_DET_PIN_INVERTING               false

////
// LED / NEOPixel//LED/NEOPixel
////
//#define LED_PIN                           PB2   // BOOT1//#定义LED_引脚PB2//BOOT1

#if ENABLED(NEOPIXEL_LED)
  #define LED_PWM                           PC7   // IO1//IO1
  #ifndef NEOPIXEL_PIN
    #define NEOPIXEL_PIN                 LED_PWM  // USED WIFI IO0/IO1 PIN//已使用WIFI IO0/IO1 PIN
  #endif
#endif

////
// SD Card//SD卡
////
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

// Use the on-board card socket labeled SD_Extender//使用标有SD_Extender的车载卡插座
#if SD_CONNECTION_IS(CUSTOM_CABLE)
  #define SD_SCK_PIN                        PC12
  #define SD_MISO_PIN                       PC8
  #define SD_MOSI_PIN                       PD2
  #define SD_SS_PIN                         -1
  #define SD_DETECT_PIN                     PD12  // SD_CD (if -1 no detection)//SD_CD（如果-1未检测到）
#else
  #define SDIO_SUPPORT
  #define SDIO_CLOCK                     4500000  // 4.5 MHz//4.5兆赫
  #define SDIO_READ_RETRIES                   16
  #define ONBOARD_SPI_DEVICE                   1  // SPI1//SPI1
  #define ONBOARD_SD_CS_PIN                 PC11
  #define SD_DETECT_PIN                     -1    // SD_CD (-1 active refresh)//SD_CD（-1个活动刷新）
#endif

////
// LCD / Controller//液晶显示器/控制器
////
#ifndef BEEPER_PIN
  #define BEEPER_PIN                        PC5
#endif

#if ENABLED(SPEAKER) && BEEPER_PIN == PC5
  #error "FLSun HiSpeed default BEEPER_PIN is not a SPEAKER."
#endif

#if HAS_FSMC_TFT || HAS_GRAPHICAL_TFT
  #define TFT_CS_PIN                        PD7   // NE4//NE4
  #define TFT_RS_PIN                        PD11  // A0//A0
#endif

#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers
   * Supported screens are based on the ILI9341, ST7789V and ILI9328 (320x240)
   * ILI9488 is not supported
   * Define init sequences for other screens in u8g_dev_tft_320x240_upscale_from_128x64.cpp
   *
   * If the screen stays white, disable 'LCD_RESET_PIN'
   * to let the bootloader init the screen.
   *
   * Setting an 'LCD_RESET_PIN' may cause a flicker when entering the LCD menu
   * because Marlin uses the reset as a failsafe to revive a glitchy LCD.
   */
  //#define TFT_RESET_PIN                   PC6   // FSMC_RST//#定义TFT复位引脚PC6//FSMC\u RST
  #define TFT_BACKLIGHT_PIN                 PD13
  #define FSMC_CS_PIN                 TFT_CS_PIN  // NE4//NE4
  #define FSMC_RS_PIN                 TFT_RS_PIN  // A0//A0

  #define LCD_USE_DMA_FSMC                        // Use DMA transfers to send data to the TFT//使用DMA传输将数据发送到TFT
  #define FSMC_DMA_DEV                      DMA2
  #define FSMC_DMA_CHANNEL               DMA_CH5
  #ifdef TFT_CLASSIC_UI
    #define TFT_MARLINBG_COLOR            0x3186  // Grey//灰色的
    #define TFT_MARLINUI_COLOR            0xC7B6  // Green//绿色的
    #define TFT_BTARROWS_COLOR            0xDEE6  // Yellow//黄色的
    #define TFT_BTOKMENU_COLOR            0x145F  // Cyan//青色
  #endif
  #define TFT_BUFFER_SIZE                  14400
#elif HAS_GRAPHICAL_TFT
  #define TFT_RESET_PIN                     PC6
  #define TFT_BACKLIGHT_PIN                 PD13
#endif

#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PC2   // SPI2_NSS//SPI2\U NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK//SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO//味噌
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI//SPI2_MOSI
  #define TOUCH_INT_PIN                     -1
#endif
