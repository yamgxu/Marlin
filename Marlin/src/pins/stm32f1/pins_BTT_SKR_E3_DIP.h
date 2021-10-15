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

#define BOARD_INFO_NAME "BTT SKR E3 DIP V1.x"

// Release PB3/PB4 (TMC_SW Pins) from JTAG pins//从JTAG引脚上释放PB3/PB4（TMC_SW引脚）
#define DISABLE_JTAG

// Ignore temp readings during development.//在开发过程中忽略温度读数。
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000//#定义2000年的假温度宽限期

#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB//2KB
#endif

////
// Servos//伺服
////
#define SERVO0_PIN                          PA1   // SERVOS//伺服

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PC1   // X-STOP//X光圈
#define Y_STOP_PIN                          PC0   // Y-STOP//Y形挡块
#define Z_STOP_PIN                          PC15  // Z-STOP//Z-STOP

////
// Z Probe must be this pin//Z探头必须位于该引脚上
////
#define Z_MIN_PROBE_PIN                     PC14  // PROBE//探测

////
// Filament Runout Sensor//灯丝偏移传感器
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PC2   // E0-STOP//E0停止
#endif

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PC7
#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PB15
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC10
#endif

#define Y_ENABLE_PIN                        PB14
#define Y_STEP_PIN                          PB13
#define Y_DIR_PIN                           PB12
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PC11
#endif

#define Z_ENABLE_PIN                        PB11
#define Z_STEP_PIN                          PB10
#define Z_DIR_PIN                           PB2
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PC12
#endif

#define E0_ENABLE_PIN                       PB1
#define E0_STEP_PIN                         PB0
#define E0_DIR_PIN                          PC5
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD2
#endif

////
// Software SPI pins for TMC2130 stepper drivers//TMC2130步进驱动器的软件SPI引脚
////
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PB5
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PB4
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PB3
  #endif
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  MSerial1//#定义X_硬件_串行MSerial1
  //#define Y_HARDWARE_SERIAL  MSerial1//#定义Y_硬件_串行MSerial1
  //#define Z_HARDWARE_SERIAL  MSerial1//#定义Z_硬件_串行MSerial1
  //#define E0_HARDWARE_SERIAL MSerial1//#定义E0_硬件_串行MSerial1

  ////
  // Software serial//软件系列
  ////
  #define X_SERIAL_TX_PIN                   PC10
  #define X_SERIAL_RX_PIN                   PC10

  #define Y_SERIAL_TX_PIN                   PC11
  #define Y_SERIAL_RX_PIN                   PC11

  #define Z_SERIAL_TX_PIN                   PC12
  #define Z_SERIAL_RX_PIN                   PC12

  #define E0_SERIAL_TX_PIN                  PD2
  #define E0_SERIAL_RX_PIN                  PD2

  // Reduce baud rate to improve software serial reliability//降低波特率以提高软件串行可靠性
  #define TMC_BAUD_RATE                    19200
#endif

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PA0   // Analog Input "TH0"//模拟输入“TH0”
#define TEMP_BED_PIN                        PC3   // Analog Input "TB0"//模拟输入“TB0”

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PC8   // "HE"//“他”
#define HEATER_BED_PIN                      PC9   // "HB"//“血红蛋白”
#define FAN_PIN                             PA8   // "FAN0"//“FAN0”

////
// USB connect control//USB连接控制
////
#define USB_CONNECT_PIN                     PC13
#define USB_CONNECT_INVERTING              false

/**
 *                 _____
 *             5V | 1 2 | GND
 *  (LCD_EN) PB7  | 3 4 | PB8  (LCD_RS)
 *  (LCD_D4) PB9  | 5 6   PA10 (BTN_EN2)
 *          RESET | 7 8 | PA9  (BTN_EN1)
 * (BTN_ENC) PB6  | 9 10| PA15 (BEEPER)
 *                 -----
 *                 EXP1
 */

#if HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN                      PA15

    #define BTN_ENC                         PB6
    #define BTN_EN1                         PA9
    #define BTN_EN2                         PA10

    #define LCD_PINS_RS                     PB8
    #define LCD_PINS_ENABLE                 PB7
    #define LCD_PINS_D4                     PB9

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!//ANET A8 LCD控制器-必须转换为3.3V-连接到5V会损坏电路板！

    #error "CAUTION! ZONESTAR_LCD requires wiring modifications. See 'pins_BTT_SKR_MINI_E3_common.h' for details. Comment out this line to continue."

    #define LCD_PINS_RS                     PB9
    #define LCD_PINS_ENABLE                 PB6
    #define LCD_PINS_D4                     PB8
    #define LCD_PINS_D5                     PA10
    #define LCD_PINS_D6                     PA9
    #define LCD_PINS_D7                     PA15
    #define ADC_KEYPAD_PIN                  PA1   // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!//将伺服引脚重新用于ADC-连接到5V会损坏电路板！

  #elif EITHER(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

    /** Creality Ender-2 display pinout
     *                   _____
     *               5V | 1 2 | GND
     *      (MOSI) PB7  | 3 4 | PB8  (LCD_RS)
     *    (LCD_A0) PB9  | 5 6   PA10 (BTN_EN2)
     *            RESET | 7 8 | PA9  (BTN_EN1)
     *   (BTN_ENC) PB6  | 9 10| PA15 (SCK)
     *                   -----
     *                    EXP1
     */

    #define BTN_ENC                         PB6
    #define BTN_EN1                         PA9
    #define BTN_EN2                         PA10

    #define DOGLCD_CS                       PB8
    #define DOGLCD_A0                       PB9
    #define DOGLCD_SCK                      PA15
    #define DOGLCD_MOSI                     PB7
    #define FORCE_SOFT_SPI
    #define LCD_BACKLIGHT_PIN               -1

  #else
    #error "Only CR10_STOCKDISPLAY, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, and MKS_LCD12864A/B are currently supported on the BIGTREE_SKR_E3_DIP."
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

#if BOTH(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  #error "CAUTION! LCD_FYSETC_TFT81050 requires wiring modifications. See 'pins_BTT_SKR_E3_DIP.h' for details. Comment out this line to continue."

  /** FYSETC TFT TFT81050 display pinout
   *
   *               Board                                     Display
   *               _____                                      _____
   *           5V | 1 2 | GND               (SPI1-MISO) MISO | 1 2 | SCK   (SPI1-SCK)
   * (FREE)   PB7 | 3 4 | PB8  (LCD_CS)     (PA9)  MOD_RESET | 3 4 | SD_CS (PA10)
   * (FREE)   PB9 | 5 6   PA10 (SD_CS)      (PB8)     LCD_CS | 5 6   MOSI  (SPI1-MOSI)
   *        RESET | 7 8 | PA9  (MOD_RESET)  (PA15)    SD_DET | 7 8 | RESET
   * (BEEPER) PB6 | 9 10| PA15 (SD_DET)                  GND | 9 10| 5V
   *               -----                                      -----
   *                EXP1                                       EXP1
   *
   * Needs custom cable:
   *
   *    Board   Adapter   Display
   *           _________
   *   EXP1-1 ----------- EXP1-10
   *   EXP1-2 ----------- EXP1-9
   *   SPI1-4 ----------- EXP1-6
   *   EXP1-4 ----------- EXP1-5
   *   SP11-3 ----------- EXP1-2
   *   EXP1-6 ----------- EXP1-4
   *   EXP1-7 ----------- EXP1-8
   *   EXP1-8 ----------- EXP1-3
   *   SPI1-1 ----------- EXP1-1
   *  EXP1-10 ----------- EXP1-7
   */

  #define CLCD_SPI_BUS                         1  // SPI1 connector//SPI1连接器

  #define BEEPER_PIN                        PB6

  #define CLCD_MOD_RESET                    PA9
  #define CLCD_SPI_CS                       PB8

#endif // TOUCH_UI_FTDI_EVE && LCD_FYSETC_TFT81050//触摸屏FTDI EVE和LCD FYSETC TFT81050

////
// SD Support//SD支持
////

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PC4
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
#elif SD_CONNECTION_IS(LCD) && BOTH(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)
  #define SD_DETECT_PIN                     PA15
  #define SD_SS_PIN                         PA10
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with SKR E3 DIP."
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1//SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card//“系统”SD卡的芯片选择
#define SDSS                   ONBOARD_SD_CS_PIN
