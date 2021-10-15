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
 * Fast I/O Routines for LPC1768/9
 * Use direct port manipulation to save scads of processor time.
 * Contributed by Triffid_Hunter and modified by Kliment, thinkyhead, Bob-the-Kuhn, et.al.
 */

/**
 * Description: Fast IO functions LPC1768
 *
 * For TARGET LPC1768
 */

#include "../shared/Marduino.h"

#define PWM_PIN(P)            true // all pins are PWM capable//所有引脚都支持PWM

#define LPC_PIN(pin)          LPC176x::gpio_pin(pin)
#define LPC_GPIO(port)        LPC176x::gpio_port(port)

#define SET_DIR_INPUT(IO)     LPC176x::gpio_set_input(IO)
#define SET_DIR_OUTPUT(IO)    LPC176x::gpio_set_output(IO)

#define SET_MODE(IO, mode)    pinMode(IO, mode)

#define WRITE_PIN_SET(IO)     LPC176x::gpio_set(IO)
#define WRITE_PIN_CLR(IO)     LPC176x::gpio_clear(IO)

#define READ_PIN(IO)          LPC176x::gpio_get(IO)
#define WRITE_PIN(IO,V)       LPC176x::gpio_set(IO, V)

/**
 * Magic I/O routines
 *
 * Now you can simply SET_OUTPUT(STEP); WRITE(STEP, HIGH); WRITE(STEP, LOW);
 *
 * Why double up on these macros? see https://gcc.gnu.org/onlinedocs/gcc-4.8.5/cpp/Stringification.html
 */

/// Read a pin///读别针
#define _READ(IO)             READ_PIN(IO)

/// Write to a pin///用别针写字
#define _WRITE(IO,V)          WRITE_PIN(IO,V)

/// toggle a pin///拨动别针
#define _TOGGLE(IO)           _WRITE(IO, !READ(IO))

/// set pin as input///将引脚设置为输入
#define _SET_INPUT(IO)        SET_DIR_INPUT(IO)

/// set pin as output///将引脚设置为输出
#define _SET_OUTPUT(IO)       SET_DIR_OUTPUT(IO)

/// set pin as input with pullup mode///使用上拉模式将引脚设置为输入
#define _PULLUP(IO,V)         pinMode(IO, (V) ? INPUT_PULLUP : INPUT)

/// set pin as input with pulldown mode///使用下拉模式将引脚设置为输入
#define _PULLDOWN(IO,V)       pinMode(IO, (V) ? INPUT_PULLDOWN : INPUT)

/// check if pin is an input///检查引脚是否为输入
#define _IS_INPUT(IO)         (!LPC176x::gpio_get_dir(IO))

/// check if pin is an output///检查引脚是否为输出
#define _IS_OUTPUT(IO)        (LPC176x::gpio_get_dir(IO))

/// Read a pin wrapper///读一个别针包装
#define READ(IO)              _READ(IO)

/// Write to a pin wrapper///写一个别针包装器
#define WRITE(IO,V)           _WRITE(IO,V)

/// toggle a pin wrapper///拨动别针包装器
#define TOGGLE(IO)            _TOGGLE(IO)

/// set pin as input wrapper///将pin设置为输入包装器
#define SET_INPUT(IO)         _SET_INPUT(IO)
/// set pin as input with pullup wrapper///使用上拉包装器将引脚设置为输入
#define SET_INPUT_PULLUP(IO)  do{ _SET_INPUT(IO); _PULLUP(IO, HIGH); }while(0)
/// set pin as input with pulldown wrapper///使用下拉包装器将pin设置为输入
#define SET_INPUT_PULLDOWN(IO) do{ _SET_INPUT(IO); _PULLDOWN(IO, HIGH); }while(0)
/// set pin as output wrapper  -  reads the pin and sets the output to that value///将pin设置为输出包装器-读取pin并将输出设置为该值
#define SET_OUTPUT(IO)        do{ _WRITE(IO, _READ(IO)); _SET_OUTPUT(IO); }while(0)
// set pin as PWM//将引脚设置为PWM
#define SET_PWM               SET_OUTPUT

/// check if pin is an input wrapper///检查pin是否为输入包装
#define IS_INPUT(IO)          _IS_INPUT(IO)
/// check if pin is an output wrapper///检查pin是否为输出包装器
#define IS_OUTPUT(IO)         _IS_OUTPUT(IO)

// Shorthand//速记
#define OUT_WRITE(IO,V)       do{ SET_OUTPUT(IO); WRITE(IO,V); }while(0)

// digitalRead/Write wrappers//数字读/写包装器
#define extDigitalRead(IO)    digitalRead(IO)
#define extDigitalWrite(IO,V) digitalWrite(IO,V)
