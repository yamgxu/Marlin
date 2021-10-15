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
 * ARCHIM1 pin assignment
 *
 * The Archim 1.0 board requires Arduino Archim addons installed.
 *
 * - Add the following URL to Arduino IDE's Additional Board Manager URLs:
 *   https://raw.githubusercontent.com/ultimachine/ArduinoAddons/master/package_ultimachine_index.json
 *
 * - In the Arduino IDE Board Manager search for Archim and install the package.
 *
 * - Change your target board to "Archim".
 *
 * Further information on the UltiMachine website...
 *   https://github.com/ultimachine/Archim/wiki
 */

#if NOT_TARGET(__SAM3X8E__)
  #error "Oops! Select 'Archim' in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Archim 1.0"

////
// Timers//计时器
////
// These are already defined in DUE, so must be undefined first//这些已在到期日中定义，因此必须先未定义
#define STEP_TIMER_NUM                         3
#define HAL_STEP_TIMER_ISR()  void TC3_Handler()

////
// Items marked * have been altered from Archim v1.0//标记为*的项目已从Archim v1.0更改
////

////
// Servos//伺服
////
#define SERVO0_PIN                            20  // D20 PB12 (Header J20 20)//D20 PB12（割台J20 20）
#define SERVO1_PIN                            21  // D21 PB13 (Header J20 19)//D21 PB13（割台J20 19）

////
// Limit Switches//限位开关
////
#define X_MIN_PIN                             14  // PD4 MIN ES1//PD4分钟ES1
#define X_MAX_PIN                             32  // PD10 MAX ES1//PD10最大ES1
#define Y_MIN_PIN                             29  // PD6 MIN ES2//PD6分钟ES2
#define Y_MAX_PIN                             15  // PD5 MAX ES2//PD5最大ES2
#define Z_MIN_PIN                             31  // PA7 MIN ES3//PA7分钟ES3
#define Z_MAX_PIN                             30  // PD9 MAX ES3//PD9最大ES3

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      66  // D66 PB15 (Header J20 15)//D66 PB15（割台J20 15）
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                     67  // D67 PB16 (Header J20 16)//D67 PB16（割台J20 16）
#endif

////
// Steppers//踏步机
////
#define X_STEP_PIN                            40  // PC8 STEP1 *//PC8步骤1*
#define X_DIR_PIN                             59  // PA4 DIR1 *//PA4-DIR1*
#define X_ENABLE_PIN                          41  // PC9 EN1//PC9 EN1

#define Y_STEP_PIN                            49  // PC14 STEP2 *//PC14步骤2*
#define Y_DIR_PIN                             47  // PC16 DIR2 *//PC16-DIR2*
#define Y_ENABLE_PIN                          48  // PC15 EN2 *//PC15 EN2*

#define Z_STEP_PIN                            36  // PC4 STEP Z *//PC4步骤Z*
#define Z_DIR_PIN                            107  // PB10 DIR Z *//PB10目录Z*
#define Z_ENABLE_PIN                          96  // PC10 EN Z -AddOns *//PC10 EN Z-插件*

#define E0_STEP_PIN                           78  // PB23 STEP3 *//PB23步骤3*
#define E0_DIR_PIN                            22  // PB26 DIR3 *//PB26-DIR3*
#define E0_ENABLE_PIN                         97  // PB24 EN3 -Addons *//PB24 EN3-插件*

#define E1_STEP_PIN                           26  // PD1 STEP4 *//PD1步骤4*
#define E1_DIR_PIN                            27  // PD2 DIR4 *//PD2-DIR4*
#define E1_ENABLE_PIN                         28  // PD3 EN4 *//PD3 EN4*

// Microstepping mode pins *//微步模式引脚*
#define X_MS1_PIN                             39  // PC7 MOD0E1 - As listed in schematic//PC7 MOD0E1-如示意图所示
#define X_MS2_PIN                             38  // PC6 MOD1E1//PC6模块1
#define X_MS3_PIN                             37  // PC5 MOD2E1//PC5模块2E1

#define Y_MS1_PIN                             50  // PC13 MODE0E2//PC13模式0E2
#define Y_MS2_PIN                             51  // PC12 MODE1E2//PC12模式1E2
#define Y_MS3_PIN                             92  // PC11 MODE2E2 - AddOns//PC11模式2E2-插件

#define Z_MS1_PIN                             44  // PC19 MOD0E Z//PC19模块Z
#define Z_MS2_PIN                             45  // PC18 MOD1E Z//PC18模块1e Z
#define Z_MS3_PIN                             46  // PC17 MOD2E Z//PC17 MOD2E Z

#define E0_MS1_PIN                           105  // PB22 MOD0E3 - AddOns//PB22 MOD0E3-插件
#define E0_MS2_PIN                           106  // PC27 MOD1E3 - AddOns//PC27 MOD1E3-插件
#define E0_MS3_PIN                           104  // PC20 MOD2E3 - AddOns//PC20 MOD2E3-插件

#define E1_MS1_PIN                            25  // PD0 MOD0E4//PD0 MOD0E4
#define E1_MS2_PIN                            18  // PA11 MOD1E4//PA11 MOD1E4
#define E1_MS3_PIN                            19  // PA10 MOD2E4//PA10 MOD2E4

// Motor current PWM pins *//电机电流PWM引脚*
#define MOTOR_CURRENT_PWM_X_PIN               58  // PA6   X-REF TIOB2//PA6 X-REF TiO2
#define MOTOR_CURRENT_PWM_Y_PIN               12  // PD8   Y-REF TIOB8//PD8 Y-REF TIOB8
#define MOTOR_CURRENT_PWM_Z_PIN               10  // PC29  Z-REF TIOB7//PC29 Z-REF TIOB7
#define MOTOR_CURRENT_PWM_E0_PIN               3  // PC28 E1-REF TIOA7//PC28 E1-REF TIOA7
#define MOTOR_CURRENT_PWM_E1_PIN              11  // PD7  E2-REF TIOA8//PD7 E2-REF TIOA8

#define MOTOR_CURRENT_PWM_RANGE              2750 // (3.3 Volts * 100000 Ohms) / (100000 Ohms + 20000 Ohms) = 2.75 Volts (max vref)//（3.3伏*100000欧姆）/（100000欧姆+20000欧姆）=2.75伏（最大vref）
#define DEFAULT_PWM_MOTOR_CURRENT  { 1000, 1000, 1000 } //, 1000, 1000}   // X Y Z E0 E1, 1000 = 1000mAh//，1000，1000}//X Y Z E0 E1，1000=1000mAh

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            10  // D10 PB19 THERM AN1 *//D10 PB19热分析仪AN1*
#define TEMP_1_PIN                             9  // D9 PB18 THERM AN2 *//D9 PB18热分析仪AN2*
#define TEMP_2_PIN                             8  // D8 PB17 THERM AN4 *//D8 PB17热分析仪AN4*
#define TEMP_BED_PIN                          11  // D11 PB20 THERM AN3 *//D11 PB20热交换器AN3*

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           6  // D6 PC24 FET_PWM3//D6 PC24 FET_PWM3
#define HEATER_1_PIN                           7  // D7 PC23 FET_PWM4//D7 PC23 FET_PWM4
#define HEATER_2_PIN                           8  // D8 PC22 FET_PWM5//D8 PC22 FET_PWM5
#define HEATER_BED_PIN                         9  // D9 PC21 BED_PWM//D9 PC21床用脉宽调制

#ifndef FAN_PIN
  #define FAN_PIN                              4  // D4 PC26 FET_PWM1//D4 PC26 FET_PWM1
#endif
#define FAN1_PIN                               5  // D5 PC25 FET_PWM2//D5 PC25 FET_PWM2

////
// Misc. Functions//杂项。功能
////

// Internal MicroSD card reader on the PCB//PCB上的内部MicroSD读卡器
#define INT_SCK_PIN                           42  // D42 PA19/MCCK//D42 PA19/MCCK
#define INT_MISO_PIN                          43  // D43 PA20/MCCDA//D43 PA20/MCCDA
#define INT_MOSI_PIN                          73  // D73 PA21/MCDA0//D73 PA21/MCDA0
#define INT_SDSS                              55  // D55 PA24/MCDA3//D55 PA24/MCDA3

// External SD card reader on SC2//SC2上的外部SD卡读卡器
#define SD_SCK_PIN                            76  // D76 PA27//D76第27页
#define SD_MISO_PIN                           74  // D74 PA25//D74 PA25
#define SD_MOSI_PIN                           75  // D75 PA26//D75 PA26
#define SDSS                                  87  // D87 PA29//D87第29页

// 2MB SPI Flash//2MB SPI闪存
#define SPI_FLASH_SS                          52  // D52 PB21//D52 PB21

////
// LCD / Controller//液晶显示器/控制器
////
#if HAS_WIRED_LCD
  #define BEEPER_PIN                          23  // D24 PA15_CTS1//D24 PA15_CTS1
  #define LCD_PINS_RS                         17  // D17 PA12_RXD1//D17 PA12RXD1
  #define LCD_PINS_ENABLE                     24  // D23 PA14_RTS1//D23 PA14_RTS1
  #define LCD_PINS_D4                         69  // D69 PA0_CANTX0//D69 PA0_CANTX0
  #define LCD_PINS_D5                         54  // D54 PA16_SCK1//D54 PA16_SCK1
  #define LCD_PINS_D6                         68  // D68 PA1_CANRX0//D68 PA1_CANRX0
  #define LCD_PINS_D7                         34  // D34 PC2_PWML0//D34 PC2_PWML0

  #define SD_DETECT_PIN                        2  // D2  PB25_TIOA0//D2 PB25_TIOA0

  #if IS_NEWPANEL
    // Buttons on AUX-2//AUX-2上的按钮
    #define BTN_EN1                           60  // D60 PA3_TIOB1//D60 PA3_TIOB1
    #define BTN_EN2                           13  // D13 PB27_TIOB0//D13 PB27_TIOB0
    #define BTN_ENC                           16  // D16 PA13_TXD1//D16 PA13_TXD1
  #endif // IS_NEWPANEL//这是新小组吗

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder//检测编码器的存在
  #endif

#endif // HAS_WIRED_LCD//有有线液晶显示器吗
