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
 * Mightyboard Rev.E pin assignments
 * also works for Rev D boards. It's all rev E despite what the silk screen says
 */

/**
 * Rev B  2 JAN 2017
 *
 *  Added pin definitions for:
 *    M3, M4 & M5 spindle control commands
 *    case light
 *
 *  Corrected pin assignment for EX2_HEAT_PIN pin. Changed it from 9 to 11.  The port
 *  number (B5) agrees with the schematic but B5 is assigned to logical pin 11.
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME      "Mightyboard"
#define DEFAULT_MACHINE_NAME "MB Replicator"

////
// Servos//伺服
////
#define SERVO0_PIN                            36  // C1 (1280-EX1)//C1（1280-EX1）
#define SERVO1_PIN                            37  // C0 (1280-EX2)//C0（1280-EX2）
#define SERVO2_PIN                            40  // G1 (1280-EX3)//G1（1280-EX3）
#define SERVO3_PIN                            41  // G0 (1280-EX4)//G0（1280-EX4）

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             49  // L0//L0
#define X_MAX_PIN                             48  // L1//L1
#define Y_MIN_PIN                             47  // L2//L2
#define Y_MAX_PIN                             46  // L3//L3
#define Z_MIN_PIN                             43  // L6//L6
#define Z_MAX_PIN                             42  // L7//L7

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     42
#endif

////
// Filament Runout Pins//灯丝跳动销
////
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      49
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                     47
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            55  // F1//F1
#define X_DIR_PIN                             54  // F0//F0
#define X_ENABLE_PIN                          56  // F2//F2

#define Y_STEP_PIN                            59  // F5//F5
#define Y_DIR_PIN                             58  // F4//F4
#define Y_ENABLE_PIN                          60  // F6//F6

#define Z_STEP_PIN                            63  // K1//K1
#define Z_DIR_PIN                             62  // K0//K0
#define Z_ENABLE_PIN                          64  // K2//K2

#define E0_STEP_PIN                           25  // A3//A3
#define E0_DIR_PIN                            24  // A2//A2
#define E0_ENABLE_PIN                         26  // A4//A4

#define E1_STEP_PIN                           29  // A7//A7
#define E1_DIR_PIN                            28  // A6//A6
#define E1_ENABLE_PIN                         39  // G2//G2

////
// I2C Digipots - MCP4018//I2C数字端口-MCP4018
// Address 5E (2F << 1)//地址5E（2F<<1）
// Set from 0 - 127 with stop bit.//使用停止位从0到127进行设置。
// (Ex. 3F << 1 | 1)//（例如3F<1 | 1）
////
#define DIGIPOTS_I2C_SCL                      76  // J5//J5
#define DIGIPOTS_I2C_SDA_X                    57  // F3//F3
#define DIGIPOTS_I2C_SDA_Y                    61  // F7//F7
#define DIGIPOTS_I2C_SDA_Z                    65  // K3//K3
#define DIGIPOTS_I2C_SDA_E0                   27  // A5//A5
#define DIGIPOTS_I2C_SDA_E1                   77  // J6//J6

#ifndef DIGIPOT_I2C_ADDRESS_A
  #define DIGIPOT_I2C_ADDRESS_A             0x2F  // unshifted slave address (5E <- 2F << 1)//非移位从属地址（5E<-2F<<1）
#endif
#define DIGIPOT_ENABLE_I2C_PULLUPS                // MightyBoard doesn't have hardware I2C pin pull-ups.//MightyBoard没有硬件I2C引脚上拉。

////
// Temperature Sensors//温度传感器
////
// K7 - 69 / ADC15 - 15//K7-69/ADC15-15
#define TEMP_BED_PIN                          15

// SPI for Max6675 or Max31855 Thermocouple//Max6675或Max31855热电偶的SPI
// Uses a separate SPI bus//使用单独的SPI总线
////
//  3 E5 DO (SO)//3 E5 DO（SO）
//  5 E3 CS1//5 E3 CS1
//  2 E4 CS2//2 E4 CS2
// 78 E2 SCK//78 E2 SCK
////
#define THERMO_SCK_PIN                        78  // E2//E2
#define THERMO_DO_PIN                          3  // E5//E5
#define THERMO_CS1_PIN                         5  // E3//E3
#define THERMO_CS2_PIN                         2  // E4//E4

#define MAX6675_SS_PIN            THERMO_CS1_PIN
#define MAX6675_SS2_PIN           THERMO_CS2_PIN
#define MAX6675_SCK_PIN           THERMO_SCK_PIN
#define MAX6675_DO_PIN             THERMO_DO_PIN

////
// Augmentation for auto-assigning plugs//自动分配插头的扩充
////
// Two thermocouple connectors allows for either//两个热电偶接头允许
// 2 extruders or 1 extruder and a heated bed.//2台挤出机或1台挤出机和一个加热床。
// With no heated bed, an additional 24V fan is possible.//在没有加热床的情况下，可以使用额外的24V风扇。
////

// Labels from the schematic://原理图中的标签：
#define EX1_HEAT_PIN                           6  // H3//H3
#define EX1_FAN_PIN                            7  // H4//H4
#define EX2_HEAT_PIN                          11  // B5//B5
#define EX2_FAN_PIN                           12  // B6//B6
#define HBP_PIN                               45  // L4//L4
#define EXTRA_FET_PIN                         44  // L5//L5

#if HAS_MULTI_HOTEND
  #if TEMP_SENSOR_BED
    #define IS_EEB
  #else
    #define IS_EEF
  #endif
#elif TEMP_SENSOR_BED
  #define IS_EFB
#else
  #define IS_EFF
#endif

////
// Heaters / Fans (24V)//加热器/风扇（24V）
////
#define HEATER_0_PIN                EX1_HEAT_PIN

#if ENABLED(IS_EFB)                               // Hotend, Fan, Bed//热端、风扇、床
  #define HEATER_BED_PIN                 HBP_PIN
#elif ENABLED(IS_EEF)                             // Hotend, Hotend, Fan//热端，热端，风扇
  #define HEATER_1_PIN              EX2_HEAT_PIN
#elif ENABLED(IS_EEB)                             // Hotend, Hotend, Bed//热端，热端，床
  #define HEATER_1_PIN              EX2_HEAT_PIN
  #define HEATER_BED_PIN                 HBP_PIN
#elif ENABLED(IS_EFF)                             // Hotend, Fan, Fan//热端，风扇，风扇
  #define FAN1_PIN                       HBP_PIN
#endif

#ifndef FAN_PIN
  #if EITHER(IS_EFB, IS_EFF)                      // Hotend, Fan, Bed or Hotend, Fan, Fan//热端，风扇，床或热端，风扇，风扇
    #define FAN_PIN                 EX2_HEAT_PIN
  #elif EITHER(IS_EEF, IS_SF)                     // Hotend, Hotend, Fan or Spindle, Fan//热端，热端，风扇或主轴，风扇
    #define FAN_PIN                      HBP_PIN
  #else
    #define FAN_PIN                EXTRA_FET_PIN
  #endif
#endif

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN         EX2_FAN_PIN
#endif

////
// Misc. Functions//杂项。功能
////
#define LED_PIN                               13  // B7//B7
#define CUTOFF_RESET_PIN                      16  // H1//H1
#define CUTOFF_TEST_PIN                       17  // H0//H0
#define CUTOFF_SR_CHECK_PIN                   70  // G4 (TOSC1)//G4（TOSC1）

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD

  #if IS_RRD_FG_SC

    #define LCD_PINS_RS                       33  // C4: LCD-STROBE//C4:LCD选通
    #define LCD_PINS_ENABLE                   72  // J2: LEFT//J2：左
    #define LCD_PINS_D4                       35  // C2: LCD-CLK//C2:LCD-CLK
    #define LCD_PINS_D5                       32  // C5: RLED//C5:RLED
    #define LCD_PINS_D6                       34  // C3: LCD-DATA//C3:LCD-DATA
    #define LCD_PINS_D7                       31  // C6: GLED//C6：高兴

    #define BTN_EN2                           75  // J4, UP//J4，起来
    #define BTN_EN1                           73  // J3, DOWN//J3，放下
    //STOP button connected as KILL_PIN//停止按钮连接为KILL_引脚
    #define KILL_PIN                          14  // J1, RIGHT//J1,对
    //KILL - not connected//KILL-未连接

    #define BEEPER_PIN                         8  // H5, SD_WP//H5，SD_WP

    //on board leds//车载LED
    #define STAT_LED_RED_LED          SERVO0_PIN  // C1 (1280-EX1, DEBUG2)//C1（1280-EX1，调试2）
    #define STAT_LED_BLUE_PIN         SERVO1_PIN  // C0 (1280-EX2, DEBUG3)//C0（1280-EX2，调试3）

  #else
    // Replicator uses a 3-wire SR controller with HD44780//Replicator使用带HD44780的3线SR控制器
    #define SR_DATA_PIN                       34  // C3//C3
    #define SR_CLK_PIN                        35  // C2//C2
    #define SR_STROBE_PIN                     33  // C4//C4

    #define BTN_UP                            75  // J4//J4
    #define BTN_DWN                           73  // J3//J3
    #define BTN_LFT                           72  // J2//J2
    #define BTN_RT                            14  // J1//J1

    // Disable encoder//禁用编码器
    #undef BTN_EN1
    #undef BTN_EN2

    #define BEEPER_PIN                         4  // G5//G5

    #define STAT_LED_RED_PIN                  32  // C5//碳五
    #define STAT_LED_BLUE_PIN                 31  // C6 (Actually green)//C6（实际为绿色）

  #endif

  #define BTN_CENTER                          15  // J0//J0
  #define BTN_ENC                     BTN_CENTER

#endif // HAS_WIRED_LCD//有有线液晶显示器吗

////
// SD Card//SD卡
////
#define SDSS                                  53  // B0//B0
#define SD_DETECT_PIN                          9  // H6//H6

////
// TMC 220x//TMC 220x
////
#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  #define X_HARDWARE_SERIAL Serial2
  #define Y_HARDWARE_SERIAL Serial1

  /**
   * Software serial
   */

  #define X_SERIAL_TX_PIN                     16
  #define X_SERIAL_RX_PIN                     17

  #define Y_SERIAL_TX_PIN                     18
  #define Y_SERIAL_RX_PIN                     19

  #define Z_SERIAL_TX_PIN                     41
  #define Z_SERIAL_RX_PIN                     66

  #define E0_SERIAL_TX_PIN                    40
  #define E0_SERIAL_RX_PIN                    67

  #define E1_SERIAL_TX_PIN                    37
  #define E1_SERIAL_RX_PIN                    68

#endif
