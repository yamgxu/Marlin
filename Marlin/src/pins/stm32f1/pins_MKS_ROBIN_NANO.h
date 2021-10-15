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
 * MKS Robin nano (STM32F130VET6) board pin assignments
 * https://github.com/makerbase-mks/MKS-Robin-Nano-V1.X/tree/master/hardware
 */

#if NOT_TARGET(STM32F1, STM32F1xx)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin nano supports up to 2 hotends / E-steppers. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "MKS Robin Nano"

#define BOARD_NO_NATIVE_USB

// Avoid conflict with TIMER_SERVO when using the STM32 HAL//使用STM32 HAL时，避免与定时器_伺服发生冲突
#define TEMP_TIMER 5

////
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role//从JTAG NRST角色中释放PB4（Y_启用_引脚）
////
#define DISABLE_JTAG

////
// EEPROM//电可擦可编程只读存储器
////
#if EITHER(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U) // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2KB//2KB
#endif

#define SPI_DEVICE                             2

////
// Servos//伺服
////
#define SERVO0_PIN                          PA8   // Enable BLTOUCH//启用BLTOUCH

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                          PA15
#define Y_STOP_PIN                          PA12
#define Z_MIN_PIN                           PA11
#define Z_MAX_PIN                           PC4

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PE4
#define X_STEP_PIN                          PE3
#define X_DIR_PIN                           PE2

#define Y_ENABLE_PIN                        PE1
#define Y_STEP_PIN                          PE0
#define Y_DIR_PIN                           PB9

#define Z_ENABLE_PIN                        PB8
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB4

#define E0_ENABLE_PIN                       PB3
#define E0_STEP_PIN                         PD6
#define E0_DIR_PIN                          PD3

#define E1_ENABLE_PIN                       PA3
#define E1_STEP_PIN                         PA6
#define E1_DIR_PIN                          PA1

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PC1   // TH1//TH1
#define TEMP_1_PIN                          PC2   // TH2//TH2
#define TEMP_BED_PIN                        PC0   // TB1//TB1

////
// Heaters / Fans//加热器/风扇
////
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PC3
#endif
#if HOTENDS == 1
  #ifndef FAN1_PIN
    #define FAN1_PIN                        PB0
  #endif
#else
  #ifndef HEATER_1_PIN
    #define HEATER_1_PIN                    PB0
  #endif
#endif
#ifndef FAN_PIN
  #define FAN_PIN                           PB1   // FAN//扇子
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PA0
#endif

////
// Thermocouples//热电偶
////
//#define MAX6675_SS_PIN                    PE5   // TC1 - CS1//#定义MAX6675_不锈钢_引脚PE5//TC1-CS1
//#define MAX6675_SS_PIN                    PE6   // TC2 - CS2//#定义MAX6675_不锈钢_引脚PE6//TC2-CS2

////
// Misc. Functions//杂项。功能
////
#if HAS_TFT_LVGL_UI
  //#define MKSPWC//#定义MKSPWC
  #ifdef MKSPWC
    #define SUICIDE_PIN                     PB2   // Enable MKSPWC SUICIDE PIN//启用MKSPWC自杀针
    #define SUICIDE_PIN_INVERTING          false  // Enable MKSPWC PIN STATE//启用MKSPWC引脚状态
    #define KILL_PIN                        PA2   // Enable MKSPWC DET PIN//启用MKSPWC DET引脚
    #define KILL_PIN_STATE                  true  // Enable MKSPWC PIN STATE//启用MKSPWC引脚状态
  #endif

  #define MT_DET_1_PIN                      PA4   // LVGL UI FILAMENT RUNOUT1 PIN//LVGL UI灯丝跳动1引脚
  #define MT_DET_2_PIN                      PE6   // LVGL UI FILAMENT RUNOUT2 PIN//LVGL UI灯丝跳动2引脚
  #define MT_DET_PIN_INVERTING             false  // LVGL UI filament RUNOUT PIN STATE//LVGL UI灯丝偏转引脚状态

  #define WIFI_IO0_PIN                      PC13  // MKS ESP WIFI IO0 PIN//MKS ESP WIFI IO0引脚
  #define WIFI_IO1_PIN                      PC7   // MKS ESP WIFI IO1 PIN//MKS ESP WIFI IO1引脚
  #define WIFI_RESET_PIN                    PA5   // MKS ESP WIFI RESET PIN//MKS ESP无线重置PIN码
#else
  //#define POWER_LOSS_PIN                  PA2   // PW_DET//#定义电源损耗引脚PA2//PW\U DET
  //#define PS_ON_PIN                       PB2   // PW_OFF//#定义PS_ON_引脚PB2//PW_OFF
  #define FIL_RUNOUT_PIN                    PA4
  #define FIL_RUNOUT2_PIN                   PE6
#endif

//#define LED_PIN                           PB2//#定义LED_引脚PB2

////
// SD Card//SD卡
////
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#define SDIO_SUPPORT
#define SDIO_CLOCK                       4500000  // 4.5 MHz//4.5兆赫
#define SD_DETECT_PIN                       PD12
#define ONBOARD_SD_CS_PIN                   PC11

////
// LCD / Controller//液晶显示器/控制器
////
#define BEEPER_PIN                          PC5

/**
 * Note: MKS Robin TFT screens use various TFT controllers.
 * If the screen stays white, disable 'TFT_RESET_PIN'
 * to let the bootloader init the screen.
 */
// Shared FSMC Configs//共享FSMC配置
#if HAS_FSMC_TFT
  #define DOGLCD_MOSI                       -1    // Prevent auto-define by Conditionals_post.h//防止通过条件自动定义\u post.h
  #define DOGLCD_SCK                        -1

  #define TOUCH_CS_PIN                      PA7   // SPI2_NSS//SPI2\U NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK//SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO//味噌
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI//SPI2_MOSI

  #define TFT_RESET_PIN                     PC6   // FSMC_RST//FSMC_RST
  #define TFT_BACKLIGHT_PIN                 PD13

  #define LCD_USE_DMA_FSMC                        // Use DMA transfers to send data to the TFT//使用DMA传输将数据发送到TFT
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define FSMC_DMA_DEV                      DMA2
  #define FSMC_DMA_CHANNEL               DMA_CH5

  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          2

  #define TFT_BUFFER_SIZE                  14400
#endif

#define HAS_SPI_FLASH                          1
#if HAS_SPI_FLASH
  #define SPI_FLASH_SIZE               0x1000000  // 16MB//16MB
  #define W25QXX_CS_PIN                     PB12
  #define W25QXX_MOSI_PIN                   PB15
  #define W25QXX_MISO_PIN                   PB14
  #define W25QXX_SCK_PIN                    PB13
#endif
