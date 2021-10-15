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
 * Leapfrog Xeed Driver board pin assignments
 *
 * This board is used by other Leapfrog printers in addition to the Xeed,
 * such as the Creatr HS and Bolt. The pin assignments vary wildly between
 * printer models. As such this file is currently specific to the Xeed.
 */

#include "env_validate.h"

#define BOARD_INFO_NAME "Leapfrog Xeed 2015"

////
// Limit Switches//限位开关
////
#define X_STOP_PIN                            47  // 'X Min'//“X分钟”
#define Y_STOP_PIN                            48  // 'Y Min'//‘Y Min’
#define Z_STOP_PIN                            49  // 'Z Min'//“Z Min”

////
// Steppers//踏步机
// The Xeed utilizes three Z-axis motors, which use the X, Y, and Z stepper connectors//Xeed使用三个Z轴电机，它们使用X、Y和Z步进连接器
// on the board. The X and Y steppers use external drivers, attached to signal-level//在董事会上。X和Y步进器使用连接到信号电平的外部驱动器
// Y-axis and X-axis connectors on the board, which map to distinct CPU pins from//板上的Y轴和X轴连接器，映射到不同的CPU引脚
// the on-board X/Y stepper drivers.//车载X/Y步进驱动程序。
////

// X-axis signal-level connector//X轴信号电平连接器
#define X_STEP_PIN                            65
#define X_DIR_PIN                             64
#define X_ENABLE_PIN                          66  // Not actually used on Xeed, could be repurposed//未在XED上实际使用，可以重新调整用途

// Y-axis signal-level connector//Y轴信号电平连接器
#define Y_STEP_PIN                            23
#define Y_DIR_PIN                             22
#define Y_ENABLE_PIN                          24  // Not actually used on Xeed, could be repurposed//未在XED上实际使用，可以重新调整用途

// ZMOT connector (Front Right Z Motor)//ZMOT接头（右前Z电机）
#define Z_STEP_PIN                            31
#define Z_DIR_PIN                             32
#define Z_ENABLE_PIN                          30

// XMOT connector (Rear Z Motor)//XMOT接头（后Z电机）
#define Z2_STEP_PIN                           28
#define Z2_DIR_PIN                            63
#define Z2_ENABLE_PIN                         29

// YMOT connector (Front Left Z Motor)//YMOT接头（左前Z电机）
#define Z3_STEP_PIN                           14
#define Z3_DIR_PIN                            15
#define Z3_ENABLE_PIN                         39

// EMOT2 connector//EMOT2连接器
#define E0_STEP_PIN                           37
#define E0_DIR_PIN                            40
#define E0_ENABLE_PIN                         36

// EMOT connector//EMOT连接器
#define E1_STEP_PIN                           34
#define E1_DIR_PIN                            35
#define E1_ENABLE_PIN                         33

////
// Filament runout//灯丝跳动
////
#define FIL_RUNOUT_PIN                        42  // ROT2 Connector//ROT2连接器
#define FIL_RUNOUT2_PIN                       44  // ROT1 Connector//ROT1连接器

////
// Temperature Sensors//温度传感器
////
#define TEMP_0_PIN                            15  // T3 Connector//T3连接器
#define TEMP_1_PIN                            13  // T1 Connector//T1连接器
#define TEMP_BED_PIN                          14  // BED Connector (Between T1 and T3)//床连接器（T1和T3之间）

////
// Heaters / Fans//加热器/风扇
////
#define HEATER_0_PIN                           8  // Misc Connector, pins 3 and 4 (Out2)//杂项接头针脚3和4（Out2）
#define HEATER_1_PIN                           9  // Misc Connector, pins 5 and 6 (Out3)//杂项接头针脚5和6（输出3）
#define HEATER_BED_PIN                         6  // Misc Connector, pins 9(-) and 10(+) (OutA)//杂项接头针脚9（-）和10（+）（OutA）

#define FAN_PIN                               10  // Misc Connector, pins 7(-) and 8 (+) (Out4)//杂项接头针脚7（-）和8（+）（输出4）

#define LED_PIN                               13

#define SOL1_PIN                               7  // Misc Connector, pins 1(-) and 2(+) (Out1)//其他接头针脚1（-）和2（+）（输出1）

// Door Closed Sensor//车门关闭传感器
//#define DOOR_PIN                            45  // HM1 Connector//#定义门U引脚45//HM1连接器
