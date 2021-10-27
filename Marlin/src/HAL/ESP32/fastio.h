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

#include "i2s.h"
#include "HAL.h"

/**
 * Utility functions
 */

// I2S expander pin mapping.//I2S扩展器引脚映射。
#define IS_I2S_EXPANDER_PIN(IO) TEST(IO, 7)
#define I2S_EXPANDER_PIN_INDEX(IO) (IO & 0x7F)

// Set pin as input//将引脚设置为输入
#define _SET_INPUT(IO)          pinMode(IO, INPUT)

// Set pin as output//将引脚设置为输出
#define _SET_OUTPUT(IO)         pinMode(IO, OUTPUT)

// Set pin as input with pullup mode//使用上拉模式将引脚设置为输入
#define _PULLUP(IO, v)          pinMode(IO, v ? INPUT_PULLUP : INPUT)

// Read a pin wrapper//读一个别针包装
#define READ(IO)                (IS_I2S_EXPANDER_PIN(IO) ? i2s_state(I2S_EXPANDER_PIN_INDEX(IO)) : digitalRead(IO))

// Write to a pin wrapper//写一个别针包装器
#define WRITE(IO, v)            (IS_I2S_EXPANDER_PIN(IO) ? i2s_write(I2S_EXPANDER_PIN_INDEX(IO), v) : digitalWrite1(IO, v))

// Set pin as input wrapper//将pin设置为输入包装器
#define SET_INPUT(IO)           _SET_INPUT(IO)

// Set pin as input with pullup wrapper//使用上拉包装器将引脚设置为输入
#define SET_INPUT_PULLUP(IO)    do{ _SET_INPUT(IO); _PULLUP(IO, HIGH); }while(0)

// Set pin as input with pulldown (substitution)//通过下拉设置引脚作为输入（替换）
#define SET_INPUT_PULLDOWN      SET_INPUT

// Set pin as output wrapper//将pin设置为输出包装器
#define SET_OUTPUT(IO)          do{ _SET_OUTPUT(IO); }while(0)

// Set pin as PWM//将引脚设置为PWM
#define SET_PWM                 SET_OUTPUT

// Set pin as output and init//将引脚设置为输出和初始
#define OUT_WRITE(IO,V)         do{ _SET_OUTPUT(IO); WRITE(IO,V); }while(0)

// digitalRead/Write wrappers//数字读/写包装器
#define extDigitalRead(IO)      digitalRead(IO)
#define extDigitalWrite(IO,V)   digitalWrite(IO,V)

// PWM outputs//PWM输出
#define PWM_PIN(P)              (P < 34 || P > 127) // NOTE Pins >= 34 are input only on ESP32, so they can't be used for output.//注：引脚>=34仅在ESP32上输入，因此不能用于输出。

// Toggle pin value//切换引脚值
#define TOGGLE(IO)              WRITE(IO, !READ(IO))

////
// Ports and functions//端口和功能
////

// UART//通用异步收发器
#define RXD        44
#define TXD        43

// TWI (I2C)//TWI（I2C）
#define SCL        5
#define SDA        4
