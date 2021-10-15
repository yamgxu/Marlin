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

// Source: https://github.com/stm32duino/Arduino_Core_STM32/blob/master/variants/ST3DP001_EVAL/variant.cpp//资料来源：https://github.com/stm32duino/Arduino_Core_STM32/blob/master/variants/ST3DP001_EVAL/variant.cpp

/**
 * HOW TO COMPILE
 *
 * PlatformIO - Use the STM32F401VE_STEVAL environment (or the "Auto Build Marlin" extension).
 *
 * Arduino - Tested with 1.8.10
 *      Install library per https://github.com/stm32duino/Arduino_Core_STM32
 *      Make the following selections under the TOOL menu in the Arduino IDE
 *          Board: "3D printer boards"
 *          Board part number: "STEVAL-3DP001V1"
 *          U(S)ART support: "Enabled (generic "Serial")"
 *          USB support (if available): "CDC (no generic "Serial")"
 *          Optimize: "Smallest (-Os default)"
 *          C Runtime Library: "newlib Nano (default)"
 */

#pragma once

#include "env_validate.h"

#ifndef MACHINE_NAME
  #define MACHINE_NAME "STEVAL-3DP001V1"
#endif

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             39  // PD8   X_STOP//PD8 X_站
#define Y_MIN_PIN                             40  // PD9   Y_STOP//PD9油站
#define Z_MIN_PIN                             41  // PD10  Z_STOP//PD10 Z_站

#define X_MAX_PIN                             44  // PD0   W_STOP//PD0 W_站
#define Y_MAX_PIN                             43  // PA8   V_STOP//PA8 V_站
#define Z_MAX_PIN                             42  // PD11  U_STOP//PD11 U_站

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
//#ifndef Z_MIN_PROBE_PIN//#ifndef Z_MIN_探头_引脚
//  #define Z_MIN_PROBE_PIN  16  // PA4//#定义Z_MIN_探针_引脚16//PA4
//#endif//#恩迪夫

////
// Filament runout//灯丝跳动
////
//#define FIL_RUNOUT_PIN                      53  // PA3    BED_THE//#定义管脚53//PA3的管脚跳动

////
// Steppers//踏步机
////

#define X_STEP_PIN                            61  // PE14    X_PWM//PE14 X_脉宽调制
#define X_DIR_PIN                             62  // PE15    X_DIR//PE15 X_DIR
#define X_ENABLE_PIN                          60  // PE13    X_RES//PE13 X_RES
#define X_CS_PIN                              16  // PA4     SPI_CS//PA4 SPI_CS

#define Y_STEP_PIN                            64  // PB10    Y_PWM//pb10y_脉宽调制
#define Y_DIR_PIN                             65  // PE9     Y_DIR//PE9 Y_DIR
#define Y_ENABLE_PIN                          63  // PE10    Y_RES//PE10 Y_RES
#define Y_CS_PIN                              16  // PA4     SPI_CS//PA4 SPI_CS

#define Z_STEP_PIN                            67  // PC6     Z_PWM//PC6 Z_脉宽调制
#define Z_DIR_PIN                             68  // PC0     Z_DIR//PC0 Z_DIR
#define Z_ENABLE_PIN                          66  // PC15    Z_RES//PC15 Z_RES
#define Z_CS_PIN                              16  // PA4     SPI_CS//PA4 SPI_CS

#define E0_STEP_PIN                           71  // PD12    E1_PW//PD12 E1_PW
#define E0_DIR_PIN                            70  // PC13    E1_DIR//PC13 E1_目录
#define E0_ENABLE_PIN                         69  // PC14    E1_RE//PC14 E1_RE
#define E0_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS

#define E1_STEP_PIN                           73  // PE5     E2_PWM//PE5 E2_脉宽调制
#define E1_DIR_PIN                            74  // PE6     E2_DIR//PE6 E2_目录
#define E1_ENABLE_PIN                         72  // PE4     E2_RESE//PE4 E2_研究所
#define E1_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS

#define E2_STEP_PIN                           77  // PB8     E3_PWM//PB8 E3_脉宽调制
#define E2_DIR_PIN                            76  // PE2     E3_DIR//PE2 E3_目录
#define E2_ENABLE_PIN                         75  // PE3     E3_RESE//PE3 E3_研究所
#define E2_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS

// needed to pass a sanity check//需要通过健康检查
#define X2_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define Y2_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define Z2_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define Z3_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define E3_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define E4_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS
#define E5_CS_PIN                             16  // PA4     SPI_CS//PA4 SPI_CS

#if HAS_L64XX
  #define L6470_CHAIN_SCK_PIN                 17  // PA5//PA5
  #define L6470_CHAIN_MISO_PIN                18  // PA6//尼龙6
  #define L6470_CHAIN_MOSI_PIN                19  // PA7//PA7
  #define L6470_CHAIN_SS_PIN                  16  // PA4//PA4

  //#define SD_SCK_PIN       L6470_CHAIN_SCK_PIN//#定义SD_SCK_引脚L6470_链_SCK_引脚
  //#define SD_MISO_PIN     L6470_CHAIN_MISO_PIN//#定义SD_MISO_引脚L6470_链_MISO_引脚
  //#define SD_MOSI_PIN     L6470_CHAIN_MOSI_PIN//#定义SD_MOSI_引脚L6470_链_MOSI_引脚
#else
  //#define SD_SCK_PIN                        13  // PB13    SPI_S//#定义SD_SCK_引脚13//PB13 SPI_S
  //#define SD_MISO_PIN                       12  // PB14    SPI_M//#定义SD_MISO_引脚12//PB14 SPI_M
  //#define SD_MOSI_PIN                       11  // PB15    SPI_M//#定义SD_MOSI_引脚11//PB15 SPI_M
#endif

/**
 * Macro to reset/enable L6474 stepper drivers
 *
 * IMPORTANT - To disable (bypass) L6474s, install the corresponding
 *             resistors (R11 - R17) and change the "V" to "0" for the
 *             corresponding pins here:
 */
#define ENABLE_RESET_L64XX_CHIPS(V)   do{ OUT_WRITE(X_ENABLE_PIN, V); \
                                          OUT_WRITE(Y_ENABLE_PIN, V); \
                                          OUT_WRITE(Z_ENABLE_PIN, V); \
                                          OUT_WRITE(E0_ENABLE_PIN,V); \
                                          OUT_WRITE(E1_ENABLE_PIN,V); \
                                          OUT_WRITE(E2_ENABLE_PIN,V); \
                                        }while(0)

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             3  // Analog input 3,  digital pin 54   PA0     E1_THERMISTOR//模拟输入3，数字引脚54 PA0 E1_热敏电阻
#define TEMP_1_PIN                             4  // Analog input 4,  digital pin 55   PA1     E2_THERMISTOR//模拟输入4，数字引脚55 PA1 E2_热敏电阻
#define TEMP_2_PIN                             5  // Analog input 5,  digital pin 56   PA2     E3_THERMISTOR//模拟输入5，数字引脚56 PA2 E3_热敏电阻
#define TEMP_BED_PIN                           0  // Analog input 0,  digital pin 51   PC2     BED_THERMISTOR_1//模拟输入0，数字针脚51 PC2床\u热敏电阻\u 1
#define TEMP_BED_1_PIN                         1  // Analog input 1,  digital pin 52   PC3     BED_THERMISTOR_2//模拟输入1，数字针脚52 PC3床\u热敏电阻\u 2
#define TEMP_BED_2_PIN                         2  // Analog input 2,  digital pin 53   PA3     BED_THERMISTOR_3//模拟输入2，数字针脚53 PA3床\u热敏电阻\u 3

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          48  // PC7   E1_HEAT_PWM//PC7 E1_加热_脉宽调制
#define HEATER_1_PIN                          49  // PB0   E2_HEAT_PWM//PB0 E2_热量_脉宽调制
#define HEATER_2_PIN                          50  // PB1   E3_HEAT_PWM//PB1 E3_加热_脉宽调制
#define HEATER_BED_PIN                        46  // PD14 (BED_HEAT_1 FET//PD14（床层加热1场效应晶体管
#define HEATER_BED_1_PIN                      45  // PD13 (BED_HEAT_2 FET//PD13（床层加热2场效应晶体管
#define HEATER_BED_2_PIN                      47  // PD15 (BED_HEAT_3 FET//PD15（床层加热3场效应晶体管

#define FAN_PIN                               57  // PC4   E1_FAN   PWM pin, Part cooling fan FET//PC4 E1_风扇PWM引脚，部分冷却风扇FET
#define FAN1_PIN                              58  // PC5   E2_FAN   PWM pin, Extruder fan FET//PC5 E2_风扇PWM引脚，挤出机风扇FET
#define FAN2_PIN                              59  // PE8   E3_FAN   PWM pin, Controller fan FET//PE8 E3_风扇PWM引脚，控制器风扇FET

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                     58  // FAN1_PIN//范努平
#endif

////
// Misc functions//杂项功能
////
#define LED_PIN                               -1  // 9 // PE1 green LED   Heart beat//9//PE1绿色LED心跳
#define PS_ON_PIN                             -1
#define KILL_PIN                              -1
#define POWER_LOSS_PIN                        -1  // PWR_LOSS / nAC_FAULT//PWR_损失/nAC_故障

////
// LCD / Controller//液晶显示器/控制器
////
//#define SD_DETECT_PIN                       66  // PA15    SD_CA//#定义SD\U检测引脚66//PA15 SD\U CA
//#define BEEPER_PIN                          24  // PC9     SDIO_D1//#定义蜂鸣器引脚24//PC9 SDIO\U D1
//#define LCD_PINS_RS                         65  // PE9     Y_DIR//#定义LCD引脚65//PE9 Y\U方向
//#define LCD_PINS_ENABLE                     59  // PE8     E3_FAN//#定义LCD引脚\u启用59//PE8 E3\u风扇
//#define LCD_PINS_D4                         10  // PB12    SPI_C//#定义LCD_引脚_D4 10//PB12 SPI_C
//#define LCD_PINS_D5                         13  // PB13    SPI_S//#定义LCD引脚5 13//PB13 SPI
//#define LCD_PINS_D6                         12  // PB14    SPI_M//#定义LCD引脚6 12//PB14 SPI
//#define LCD_PINS_D7                         11  // PB15    SPI_M//#定义LCD引脚11//PB15 SPI
//#define BTN_EN1                             57  // PC4     E1_FAN//#定义BTN_EN1 57//PC4 E1_风扇
//#define BTN_EN2                             58  // PC5     E2_FAN//#定义BTN_EN2 58//PC5 E2_风扇
//#define BTN_ENC                             52  // PC3     BED_THE//#定义BTN_ENC 52//PC3床

////
// Extension pins//延长销
////
//#define EXT0_PIN                            49  // PB0     E2_HEAT//#定义EXT0\u引脚49//PB0 E2\u热
//#define EXT1_PIN                            50  // PB1     E3_HEAT//#定义EXT1_引脚50//PB1 E3_热
//#define EXT2_PIN                                // PB2    not used (tied to ground//#定义EXT2_引脚//PB2未使用（连接至接地
//#define EXT3_PIN                            39  // PD8     X_STOP//#定义EXT3\u引脚39//PD8 X\u停止
//#define EXT4_PIN                            40  // PD9     Y_STOP//#定义EXT4_引脚40//PD9 Y_停止
//#define EXT5_PIN                            41  // PD10    Z_STOP//#定义EXT5_引脚41//PD10 Z_停止
//#define EXT6_PIN                            42  // PD11//#定义EXT6_引脚42//PD11
//#define EXT7_PIN                            71  // PD12    E1_PW//#定义EXT7_引脚71//PD12 E1_PW
//#define EXT8_PIN                            64  // PB10    Y_PWM//#定义EXT8_引脚64//PB10 Y_PWM

// WIFI//无线网络
//  2   // PD3   CTS//2//PD3电流互感器
//  3   // PD4   RTS//3//PD4 RTS
//  4   // PD5   TX//4//PD5 TX
//  5   // PD6   RX//5//PD6 RX
//  6   // PB5   WIFI_WAKEUP//6//PB5 WIFI_唤醒
//  7   // PE11  WIFI_RESET//7//PE11 WIFI_重置
//  8   // PE12  WIFI_BOOT//8//PE12 WIFI_引导

// I2C USER//I2C用户
// 14   // PB7   SDA//14//PB7 SDA
// 15   // PB6   SCL//15//PB6症状自评量表

// JTAG//JTAG
// 20   // PA13  JTAG_TMS/SWDIO//20//PA13 JTAG_TMS/SWDIO
// 21   // PA14  JTAG_TCK/SWCLK//21//PA14 JTAG_TCK/SWCLK
// 22   // PB3   JTAG_TDO/SWO//22//PB3 JTAG_TDO/SWO

////
// Onboard SD support//机载SD支持
////
#define SDIO_D0_PIN                           23  // PC8   SDIO_D0//PC8 SDIO_D0
#define SDIO_D1_PIN                           24  // PC9   SDIO_D1//PC9 SDIO_D1
//#define SD_CARD_DETECT_PIN                  25  // PA15  SD_CARD_DETECT//#定义SD卡检测引脚25//PA15 SD卡检测
#define SDIO_D2_PIN                           26  // PC10  SDIO_D2//PC10 SDIO_D2
#define SDIO_D3_PIN                           27  // PC11  SDIO_D3//PC11 SDIO_D3
#define SDIO_CK_PIN                           28  // PC12  SDIO_CK//PC12 SDIO_CK
#define SDIO_CMD_PIN                          29  // PD2   SDIO_CMD//PD2 SDIO_命令

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SDIO_SUPPORT                            // Use SDIO for onboard SD//将SDIO用于车载SD

  #ifndef SDIO_SUPPORT
    #define SOFTWARE_SPI                          // Use soft SPI for onboard SD//对车载SD使用软SPI
    #define SDSS                     SDIO_D3_PIN
    #define SD_SCK_PIN               SDIO_CK_PIN
    #define SD_MISO_PIN              SDIO_D0_PIN
    #define SD_MOSI_PIN             SDIO_CMD_PIN
  #endif
#endif

#ifndef SDSS
  #define SDSS                                16  // PA4    SPI_CS//PA4 SPI_CS
#endif

// OTG//OTG
// 30   // PA11  OTG_DM//30//PA11 OTG_DM
// 31   // PA12  OTG_DP//31//PA12 OTG\U DP

// USER_PINS//用户密码
// 34   // PD7   USER3//34//PD7用户3
// 35   // PB9   USER1//35//PB9用户1
// 36   // PE0   USER2//36//PE0用户2
// 37   // PB4   USER4//37//PB4用户4

// USERKET//用户市场
// 38   // PE7   USER_BUTTON//38//PE7用户按钮

//  0   // PA9   TX//0//PA9 TX
//  1   // PA10  RX//1//PA10 RX

// IR/PROBE//红外/探针
// 32   // PD1   IR_OUT//32//PD1红外输出
// 33   // PC1   IR_ON//33//PC1 IR_ON

/**
 * Logical pin vs. port/pin cross reference
 *
 * PA0  54   //  E1_THERMISTOR       PA9   0   //  TX
 * PA1  55   //  E2_THERMISTOR       PA10  1   //  RX
 * PA2  56   //  E3_THERMISTOR       PD3   2   //  CTS
 * PA3  53   //  BED_THERMISTOR_3    PD4   3   //  RTS
 * PA4  16   //  SPI_CS              PD5   4   //  TX
 * PA5  17   //  SPI_SCK             PD6   5   //  RX
 * PA6  18   //  SPI_MISO            PB5   6   //  WIFI_WAKEUP
 * PA7  19   //  SPI_MOSI            PE11  7   //  WIFI_RESET
 * PA8  43   //  V_STOP              PE12  8   //  WIFI_BOOT
 * PA9   0   //  TX                  PE1   9   //  STATUS_LED
 * PA10  1   //  RX                  PB12 10   //  SPI_CS
 * PA11 30   //  OTG_DM              PB15 11   //  SPI_MOSI
 * PA12 31   //  OTG_DP              PB14 12   //  SPI_MISO
 * PA13 20   //  JTAG_TMS/SWDIO      PB13 13   //  SPI_SCK
 * PA14 21   //  JTAG_TCK/SWCLK      PB7  14   //  SDA
 * PA15 25   //  SD_CARD_DETECT      PB6  15   //  SCL
 * PB0  49   //  E2_HEAT_PWM         PA4  16   //  SPI_CS
 * PB1  50   //  E3_HEAT_PWM         PA5  17   //  SPI_SCK
 * PB3  22   //  JTAG_TDO/SWO        PA6  18   //  SPI_MISO
 * PB4  37   //  USER4               PA7  19   //  SPI_MOSI
 * PB5   6   //  WIFI_WAKEUP         PA13 20   //  JTAG_TMS/SWDIO
 * PB6  15   //  SCL                 PA14 21   //  JTAG_TCK/SWCLK
 * PB7  14   //  SDA                 PB3  22   //  JTAG_TDO/SWO
 * PB8  77   //  E3_PWM              PC8  23   //  SDIO_D0
 * PB9  35   //  USER1               PC9  24   //  SDIO_D1
 * PB10 64   //  Y_PWM               PA15 25   //  SD_CARD_DETECT
 * PB12 10   //  SPI_CS              PC10 26   //  SDIO_D2
 * PB13 13   //  SPI_SCK             PC11 27   //  SDIO_D3
 * PB14 12   //  SPI_MISO            PC12 28   //  SDIO_CK
 * PB15 11   //  SPI_MOSI            PD2  29   //  SDIO_CMD
 * PC0  68   //  Z_DIR               PA11 30   //  OTG_DM
 * PC1  33   //  IR_ON               PA12 31   //  OTG_DP
 * PC2  51   //  BED_THERMISTOR_1    PD1  32   //  IR_OUT
 * PC3  52   //  BED_THERMISTOR_2    PC1  33   //  IR_ON
 * PC4  57   //  E1_FAN              PD7  34   //  USER3
 * PC5  58   //  E2_FAN              PB9  35   //  USER1
 * PC6  67   //  Z_PWM               PE0  36   //  USER2
 * PC7  48   //  E1_HEAT_PWM         PB4  37   //  USER4
 * PC8  23   //  SDIO_D0             PE7  38   //  USER_BUTTON
 * PC9  24   //  SDIO_D1             PD8  39   //  X_STOP
 * PC10 26   //  SDIO_D2             PD9  40   //  Y_STOP
 * PC11 27   //  SDIO_D3             PD10 41   //  Z_STOP
 * PC12 28   //  SDIO_CK             PD11 42   //  U_STOP
 * PC13 70   //  E1_DIR              PA8  43   //  V_STOP
 * PC14 69   //  E1_RESET            PD0  44   //  W_STOP
 * PC15 66   //  Z_RESET             PD13 45   //  BED_HEAT_2
 * PD0  44   //  W_STOP              PD14 46   //  BED_HEAT_1
 * PD1  32   //  IR_OUT              PD15 47   //  BED_HEAT_3
 * PD2  29   //  SDIO_CMD            PC7  48   //  E1_HEAT_PWM
 * PD3   2   //  CTS                 PB0  49   //  E2_HEAT_PWM
 * PD4   3   //  RTS                 PB1  50   //  E3_HEAT_PWM
 * PD5   4   //  TX                  PC2  51   //  BED_THERMISTOR_1
 * PD6   5   //  RX                  PC3  52   //  BED_THERMISTOR_2
 * PD7  34   //  USER3               PA3  53   //  BED_THERMISTOR_3
 * PD8  39   //  X_STOP              PA0  54   //  E1_THERMISTOR
 * PD9  40   //  Y_STOP              PA1  55   //  E2_THERMISTOR
 * PD10 41   //  Z_STOP              PA2  56   //  E3_THERMISTOR
 * PD11 42   //  U_STOP              PC4  57   //  E1_FAN
 * PD12 71   //  E1_PWM              PC5  58   //  E2_FAN
 * PD13 45   //  BED_HEAT_2          PE8  59   //  E3_FAN
 * PD14 46   //  BED_HEAT_1          PE13 60   //  X_RESET
 * PD15 47   //  BED_HEAT_3          PE14 61   //  X_PWM
 * PE0  36   //  USER2               PE15 62   //  X_DIR
 * PE1   9   //  STATUS_LED          PE10 63   //  Y_RESET
 * PE2  76   //  E3_DIR              PB10 64   //  Y_PWM
 * PE3  75   //  E3_RESET            PE9  65   //  Y_DIR
 * PE4  72   //  E2_RESET            PC15 66   //  Z_RESET
 * PE5  73   //  E2_PWM              PC6  67   //  Z_PWM
 * PE6  74   //  E2_DIR              PC0  68   //  Z_DIR
 * PE7  38   //  USER_BUTTON         PC14 69   //  E1_RESET
 * PE8  59   //  E3_FAN              PC13 70   //  E1_DIR
 * PE9  65   //  Y_DIR               PD12 71   //  E1_PWM
 * PE10 63   //  Y_RESET             PE4  72   //  E2_RESET
 * PE11  7   //  WIFI_RESET          PE5  73   //  E2_PWM
 * PE12  8   //  WIFI_BOOT           PE6  74   //  E2_DIR
 * PE13 60   //  X_RESET             PE3  75   //  E3_RESET
 * PE14 61   //  X_PWM               PE2  76   //  E3_DIR
 * PE15 62   //  X_DIR               PB8  77   //  E3_PWM
 */
