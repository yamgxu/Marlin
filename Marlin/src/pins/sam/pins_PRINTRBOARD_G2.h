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
 * PRINTRBOARD_G2
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Printrboard G2"
#endif

////
// Servos//伺服
////
//#define SERVO0_PIN                          -1//#定义伺服0_引脚-1
//#define SERVO1_PIN                          -1//#定义伺服1_引脚-1

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            22  // PB26//PB26
#define Y_STOP_PIN                            18  // PA11//PA11
#define Z_STOP_PIN                            19  // PA10//尼龙10

////
// Z Probe (when not Z_MIN_PIN)//Z探头（非Z_MIN_引脚时）
////
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     22
#endif

#ifndef FIL_RUNOUT_PIN
  //#define FIL_RUNOUT_PIN                    57  // PA22//#定义FIL_跳动_引脚57//PA22
#endif
#ifndef FIL_RUNOUT2_PIN
  //#define FIL_RUNOUT2_PIN                   21  // PB13//#定义FIL_输出2_引脚21//PB13
#endif

////
// LED defines//LED定义
////
//#define NEOPIXEL_TYPE                 NEO_GRBW  // NEO_GRBW / NEO_GRB - four/three channel driver type (defined in Adafruit_NeoPixel.h)//#定义NEOPIXEL_类型NEO_GRBW//NEO_GRBW/NEO_GRB-四/三通道驱动器类型（在Adafruit_NEOPIXEL.h中定义）
//#define NEOPIXEL_PIN                        20  // LED driving pin on motherboard//#定义主板上的NEOPIXEL_引脚20//LED驱动引脚
//#define NEOPIXEL_PIXELS                      3  // Number of LEDs in the strip//#定义NEOPIXEL_像素3//条带中的LED数量
//#define SDA0                                20  // PB12 NeoPixel pin I2C data//#定义SDA0 20//PB12 NeoPixel引脚I2C数据
//#define SCL0                                21  // PB13              I2C clock//#定义SCL0 21//PB13 I2C时钟

// D0_12 #REF! (INDICATOR_LED)//D0_12#参考！（指示灯（发光二极管）
// B28 JTAG-CLK//B28 JTAG-CLK
// B31 JTAG_TMS /SWD_DIO//B31 JTAG_TMS/SWD_DIO
//A18 INTERRUPT_OUT//A18中断输出
//A12 USART_RX not used//A12未使用USART\U RX
//A13 USART_TX not used//A13未使用USART\U TX
//A14 UART_RTS//A14通用异步收发器
//A15 UART_CTS//A15通用异步收发器
//PB2 Unassigned//PB2未分配
//PB4 to PB9 Unassigned//PB4至PB9未分配
//#define UART_RX_PIN                          0  // PA8    "RX0"//#定义UART_RX_引脚0//PA8“RX0”
//#define UART_TX_PIN                          1  // PA9    "TX0"//#定义UART\U TX\U引脚1//PA9“TX0”
//#define UART_RTS_PIN                        23  // PA14//#定义UART\U RTS\U引脚23//PA14
//#define UART_CTS_PIN                        24  // PA15//#定义UART\U CTS\U引脚24//PA15

////
// Steppers//踏步机
////
#define Z_STEP_PIN                            73  // PA21      MOTOR 1//PA21马达1
#define Z_DIR_PIN                             75  // PA26//第26页
#define Z_ENABLE_PIN                          74  // PA25//PA25

#define X_STEP_PIN                            66  // PB15      MOTOR 2//PB15电机2
#define X_DIR_PIN                             54  // PA16//第16页
#define X_ENABLE_PIN                          67  // PB16//PB16

#define Y_STEP_PIN                            34  // PA29      MOTOR 3//PA29电机3
#define Y_DIR_PIN                             35  // PB1//PB1
#define Y_ENABLE_PIN                          36  // PB0//PB0

#define E0_STEP_PIN                           53  // PB14      MOTOR 4//PB14电机4
#define E0_DIR_PIN                            78  // PB23//PB23
#define E0_ENABLE_PIN                         37  // PB22//PB22

// Microstepping mode pins//微步模式引脚
#define Z_MS1_PIN                             52  // PB21 MODE0     MOTOR 1//PB21模式0电机1
#define Z_MS2_PIN                             52  // PB21 MODE1//PB21模式1
#define Z_MS3_PIN                             65  // PB20 MODE2//PB20模式2

#define X_MS1_PIN                             43  // PA20 MODE0     MOTOR 2//PA20模式0电机2
#define X_MS2_PIN                             43  // PA20 MODE1//PA20模式1
#define X_MS3_PIN                             42  // PA19 MODE2//PA19模式2

#define Y_MS1_PIN                             77  // PA28 MODE0     MOTOR 3//PA28模式0电机3
#define Y_MS2_PIN                             77  // PA28 MODE1//PA28模式1
#define Y_MS3_PIN                             76  // PA27 MODE2//PA27模式2

#define E0_MS1_PIN                            38  // PB11 MODE0     MOTOR 4//PB11模式0电机4
#define E0_MS2_PIN                            38  // PB11 MODE1//PB11模式1
#define E0_MS3_PIN                            39  // PB10 MODE2//PB10模式2

// Motor current PWM pins//电机电流PWM引脚
#define MOTOR_CURRENT_PWM_X_PIN               62  // PB17        MOTOR 1//PB17电机1
#define MOTOR_CURRENT_PWM_Z_PIN               63  // PB18        MOTOR 2//PB18马达2
#define MOTOR_CURRENT_PWM_Y_PIN               64  // PB19        MOTOR 3//PB19电机3
#define MOTOR_CURRENT_PWM_E_PIN               61  // PA2         MOTOR 4//PA2马达4

#define DEFAULT_PWM_MOTOR_CURRENT { 300, 400, 1000}  // XY Z E0, 1000 = 1000mAh//XY Z E0，1000=1000mAh

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                             2  // digital 56 PA23//数字56 PA23
#define TEMP_BED_PIN                           5  // digital 59 PA4//数字59 PA4

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                          40  // PA5//PA5
#define HEATER_BED_PIN                        41  // PB24//PB24

#ifndef FAN_PIN
  #define FAN_PIN                             13  //  PB27 Fan1A//PB27 Fan1A
#endif
#define FAN1_PIN                              58  //  PA6  Fan1B//PA6 Fan1B

#define FET_SAFETY_PIN                        31  // PA7  must be pulsed low every 50 mS or FETs are turned off//PA7必须每50毫秒脉冲低或FET关闭
#define FET_SAFETY_DELAY                      50  // 50 mS delay between pulses//脉冲之间延迟50毫秒
#define FET_SAFETY_INVERTED                 true  // true - negative going pulse of 2 uS//真-负向脉冲2 uS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SD_MISO_PIN                           68  // set to unused pins for now//现在设置为未使用的管脚
#define SD_MOSI_PIN                           69  // set to unused pins for now//现在设置为未使用的管脚
#define SD_SCK_PIN                            70  // set to unused pins for now//现在设置为未使用的管脚
#define SDSS                                  71  // set to unused pins for now//现在设置为未使用的管脚

/**
 * G2 uses 8 pins that are not available in the DUE environment:
 *   34 PA29 - Y_STEP_PIN
 *   35 PB1  - Y_DIR_PIN
 *   36 PB0  - Y_ENABLE_PIN
 *   37 PB22 - E0_ENABLE_PIN
 *   38 PB11 - E0_MS1_PIN - normally used by the USB native port
 *   39 PB10 - E0_MS3_PIN - normally used by the USB native port
 *   40 PA5  - HEATER_0_PIN
 *   41 PB24 - HEATER_BED_PIN
 *
 * None of these are in the arduino_due_x variant so digitalWrite and digitalRead can't be used on them.
 *
 * They can be accessed via FASTIO functions WRITE, READ, OUT_WRITE, OUTPUT, ...
 */
