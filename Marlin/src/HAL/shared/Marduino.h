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
 * HAL/shared/Marduino.h
 */

#undef DISABLED       // Redefined by ESP32//由ESP32重新定义
#undef M_PI           // Redefined by all//重新定义
#undef _BV            // Redefined by some//被一些人重新定义
#undef SBI            // Redefined by arduino/const_functions.h//由arduino/const_函数重新定义。h
#undef CBI            // Redefined by arduino/const_functions.h//由arduino/const_函数重新定义。h
#undef sq             // Redefined by teensy3/wiring.h//由teensy3/wiring.h重新定义
#undef UNUSED         // Redefined by stm32f4xx_hal_def.h//由stm32f4xx_hal_def.h重新定义

#include <Arduino.h>  // NOTE: If included earlier then this line is a NOOP//注：如果包含在前面，则该行为NOOP

#undef DISABLED
#define DISABLED(V...) DO(DIS,&&,V)

#undef _BV
#define _BV(b) (1UL << (b))
#ifndef SBI
  #define SBI(A,B) (A |= _BV(B))
#endif
#ifndef CBI
  #define CBI(A,B) (A &= ~_BV(B))
#endif

#undef sq
#define sq(x) ((x)*(x))

#ifndef __AVR__
  #ifndef strchr_P // Some platforms define a macro (DUE, teensy35)//某些平台定义宏（到期，teensy35）
    inline const char* strchr_P(const char *s, int c) { return strchr(s,c); }
    //#define strchr_P(s,c) strchr(s,c)//#定义strchr\u P（s，c）strchr（s，c）
  #endif

  #ifndef snprintf_P
    #define snprintf_P snprintf
  #endif
  #ifndef vsnprintf_P
    #define vsnprintf_P vsnprintf
  #endif
#endif

// Restart causes//重新启动原因
#define RST_POWER_ON    1
#define RST_EXTERNAL    2
#define RST_BROWN_OUT   4
#define RST_WATCHDOG    8
#define RST_JTAG       16
#define RST_SOFTWARE   32
#define RST_BACKUP     64

#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif

// Remove compiler warning on an unused variable//删除未使用变量上的编译器警告
#ifndef UNUSED
  #define UNUSED(x) ((void)(x))
#endif

#ifndef FORCE_INLINE
  #define FORCE_INLINE inline __attribute__((always_inline))
#endif

#include "progmem.h"
