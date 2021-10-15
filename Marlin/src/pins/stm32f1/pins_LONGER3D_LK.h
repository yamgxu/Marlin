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

/**
 * Longer3D LK1/LK2 & Alfawise U20/U30 (STM32F103VET6) board pin assignments
 */

#if NOT_TARGET(__STM32F1__, STM32F1xx)
  #error "Oops! Select a STM32F1 board in 'Tools > Board.'"
#elif HOTENDS > 1 || E_STEPPERS > 1
  #error "Longer3D only supports one hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME "Longer3D"

#define BOARD_NO_NATIVE_USB

//#define DISABLE_DEBUG                           //  We still want to debug with STLINK...//#定义禁用调试//我们仍然希望使用STLINK进行调试。。。
#define DISABLE_JTAG                              //  We free the jtag pins (PA15) but keep STLINK//我们释放jtag引脚（PA15），但保留STLINK
                                                  //  Release PB4 (STEP_X_PIN) from JTAG NRST role.//从JTAG NRST角色中释放PB4（步骤X引脚）。
////
// Limit Switches//限位开关
////
#define X_MIN_PIN                           PC1   // pin 16//引脚16
#define X_MAX_PIN                           PC0   // pin 15 (Filament sensor on Alfawise setup)//针脚15（Alfawise设置上的灯丝传感器）
#define Y_MIN_PIN                           PC15  // pin 9//引脚9
#define Y_MAX_PIN                           PC14  // pin 8 (Unused in stock Alfawise setup)//引脚8（库存中未使用的Alfawise设置）
#define Z_MIN_PIN                           PE6   // pin 5 Standard Endstop or Z_Probe endstop function//引脚5标准止动器或Z_探头止动器功能
#define Z_MAX_PIN                           PE5   // pin 4 (Unused in stock Alfawise setup)//引脚4（库存中未使用的Alfawise设置）
                                 // May be used for BLTouch Servo function on older variants (<= V08)//可用于旧型号（<=V08）上的BLTouch伺服功能
#define ONBOARD_ENDSTOPPULLUPS

////
// Filament Sensor//灯丝传感器
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PC0   // XMAX plug on PCB used as filament runout sensor on Alfawise boards (inverting true)//PCB上的XMAX插头用作Alfawise板上的灯丝偏移传感器（反转为真）
#endif

////
// Steppers//踏步机
////
#define X_ENABLE_PIN                        PB5   // pin 91//引脚91
#define X_STEP_PIN                          PB4   // pin 90//引脚90
#define X_DIR_PIN                           PB3   // pin 89//引脚89

#define Y_ENABLE_PIN                        PB8   // pin 95//引脚95
#define Y_STEP_PIN                          PB7   // pin 93//引脚93
#define Y_DIR_PIN                           PB6   // pin 92//引脚92

#define Z_ENABLE_PIN                        PE1   // pin 98//引脚98
#define Z_STEP_PIN                          PE0   // pin 97//引脚97
#define Z_DIR_PIN                           PB9   // pin 96//引脚96

#define E0_ENABLE_PIN                       PE4   // pin 3//引脚3
#define E0_STEP_PIN                         PE3   // pin 2//引脚2
#define E0_DIR_PIN                          PE2   // pin 1//引脚1

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                          PA0   // pin 23 (Nozzle 100K/3950 thermistor)//针脚23（喷嘴100K/3950热敏电阻）
#define TEMP_BED_PIN                        PA1   // pin 24 (Hot Bed 100K/3950 thermistor)//引脚24（热床100K/3950热敏电阻）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                        PD3   // pin 84 (Nozzle Heat Mosfet)//引脚84（喷嘴热Mosfet）
#define HEATER_BED_PIN                      PA8   // pin 67 (Hot Bed Mosfet)//引脚67（热床Mosfet）

#define FAN_PIN                             PA15  // pin 77 (4cm Fan)//引脚77（4厘米风扇）
#define FAN_SOFT_PWM                              // Required to avoid issues with heating or STLink//需要避免加热或STLink问题
#define FAN_MIN_PWM                           35  // Fan will not start in 1-30 range//风扇不会在1-30范围内启动
#define FAN_MAX_PWM                          255

//#define BEEPER_PIN                        PD13  // pin 60 (Servo PWM output 5V/GND on Board V0G+) made for BL-Touch sensor//#定义为BL触摸传感器制作的蜂鸣器_引脚PD13//引脚60（伺服PWM输出5V/GND板载V0G+）
                                                  // Can drive a PC Buzzer, if connected between PWM and 5V pins//如果连接在PWM和5V引脚之间，则可以驱动PC蜂鸣器

#define LED_PIN                             PC2   // pin 17//引脚17

// Longer3D board mosfets are passing by default//默认情况下，Longer3D板MOSFET通过
// Avoid nozzle heat and fan start before serial init//在串行初始化之前，避免喷嘴发热和风扇启动
#define BOARD_OPENDRAIN_MOSFETS

#define BOARD_PREINIT() { \
  OUT_WRITE_OD(HEATER_0_PIN, 0); \
  OUT_WRITE_OD(HEATER_BED_PIN, 0); \
  OUT_WRITE_OD(FAN_PIN, 0); \
}

////
// PWM for a servo probe//伺服探头的脉宽调制
// Other servo devices are not supported on this board!//此板上不支持其他伺服设备！
////
#if HAS_Z_SERVO_PROBE
  #define SERVO0_PIN                        PD13  // Open drain PWM pin on the V0G (GND or floating 5V)//V0G上的开漏PWM引脚（接地或浮动5V）
  #define SERVO0_PWM_OD                           // Comment this if using PE5//如果使用PE5，请对此进行注释

  //#define SERVO0_PIN                      PE5   // Pulled up PWM pin on the V08 (3.3V or 0)//#定义V08上的伺服0_引脚PE5//上拉PWM引脚（3.3V或0）
  //#undef Z_MAX_PIN                              // Uncomment if using ZMAX connector (PE5)//#未定义Z_MAX_引脚//如果使用ZMAX连接器（PE5），则取消注释
#endif

#define TFT_RESET_PIN                       PC4   // pin 33//引脚33
#define TFT_BACKLIGHT_PIN                   PD12  // pin 59//针脚59
#define FSMC_CS_PIN                         PD7   // pin 88 = FSMC_NE1//引脚88=FSMC_NE1
#define FSMC_RS_PIN                         PD11  // pin 58 A16 Register. Only one address needed//引脚58 A16寄存器。只需要一个地址

#define LCD_USE_DMA_FSMC                          // Use DMA transfers to send data to the TFT//使用DMA传输将数据发送到TFT
#define FSMC_DMA_DEV                        DMA2
#define FSMC_DMA_CHANNEL                 DMA_CH5

#define DOGLCD_MOSI                         -1    // Prevent auto-define by Conditionals_post.h//防止通过条件自动定义\u post.h
#define DOGLCD_SCK                          -1

// Buffer for Color UI//彩色用户界面缓冲区
#define TFT_BUFFER_SIZE                     3200

/**
 * Note: Alfawise U20/U30 boards DON'T use SPI2, as the hardware designer
 * mixed up MOSI and MISO pins. SPI is managed in SW, and needs pins
 * declared below.
 */
#if NEED_TOUCH_PINS
  #define TOUCH_CS_PIN                      PB12  // pin 51 SPI2_NSS//引脚51 SPI2\U NSS
  #define TOUCH_SCK_PIN                     PB13  // pin 52//针脚52
  #define TOUCH_MOSI_PIN                    PB14  // pin 53//插脚53
  #define TOUCH_MISO_PIN                    PB15  // pin 54//引脚54
  #define TOUCH_INT_PIN                     PC6   // pin 63 (PenIRQ coming from ADS7843)//引脚63（PenIRQ来自ADS7843）
#endif

////
// Persistent Storage//持久存储
// If no option is selected below the SD Card will be used//如果未选择下面的任何选项，则将使用SD卡
////
#if NO_EEPROM_SELECTED
  //#define SPI_EEPROM//#定义SPI_EEPROM
  #define FLASH_EEPROM_EMULATION
#endif

#if ENABLED(SPI_EEPROM)
  // SPI1 EEPROM Winbond W25Q64 (8MB/64Mbits)//SPI1 EEPROM Winbond W25Q64（8MB/64Mbits）
  #define SPI_CHAN_EEPROM1                     1
  #define SPI_EEPROM1_CS                    PC5   // pin 34//引脚34
  #define EEPROM_SCK          BOARD_SPI1_SCK_PIN  // PA5 pin 30//PA5引脚30
  #define EEPROM_MISO        BOARD_SPI1_MISO_PIN  // PA6 pin 31//PA6引脚31
  #define EEPROM_MOSI        BOARD_SPI1_MOSI_PIN  // PA7 pin 32//PA7引脚32
  #define EEPROM_PAGE_SIZE               0x1000U  // 4KB (from datasheet)//4KB（来自数据表）
  #define MARLIN_EEPROM_SIZE 16UL * (EEPROM_PAGE_SIZE)   // Limit to 64KB for now...//目前限制为64KB。。。
#elif ENABLED(FLASH_EEPROM_EMULATION)
  // SoC Flash (framework-arduinoststm32-maple/STM32F1/libraries/EEPROM/EEPROM.h)//SoC闪存（framework-TSTM32-maple/STM32F1/libraries/EEPROM/EEPROM.h）
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2KB//2KB
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE (EEPROM_PAGE_SIZE)
#else
  #define MARLIN_EEPROM_SIZE              0x800U  // On SD, Limit to 2KB, require this amount of RAM//在SD上，限制为2KB，需要此数量的RAM
#endif
