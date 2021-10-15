/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Copyright (c) 2021 X-Ryl669 [https://blog.cyril.by]
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

// We need SERIAL_ECHOPAIR and macros.h//我们需要串行接口和宏
#include "serial.h"

#if ENABLED(POSTMORTEM_DEBUGGING)
  // Useful macro for stopping the CPU on an unexpected condition//用于在意外情况下停止CPU的有用宏
  // This is used like SERIAL_ECHOPAIR, that is: a key-value call of the local variables you want//这与SERIAL_ECHOPAIR类似，也就是说：您想要的局部变量的键值调用
  // to dump to the serial port before stopping the CPU.//停止CPU前转储到串行端口。
                          // \/ Don't replace by SERIAL_ECHOPAIR since ONLY_FILENAME cannot be transformed to a PGM string on Arduino and it breaks building//\/不要替换为串行回音对，因为在Arduino上只有文件名不能转换为PGM字符串，并且会破坏建筑
  #define BUG_ON(V...) do { SERIAL_ECHO(ONLY_FILENAME); SERIAL_ECHO(__LINE__); SERIAL_ECHOLNPGM(": "); SERIAL_ECHOLNPAIR(V); SERIAL_FLUSHTX(); *(char*)0 = 42; } while(0)
#elif ENABLED(MARLIN_DEV_MODE)
  // Don't stop the CPU here, but at least dump the bug on the serial port//不要在这里停止CPU，但至少在串行端口上转储错误
                          // \/ Don't replace by SERIAL_ECHOPAIR since ONLY_FILENAME cannot be transformed to a PGM string on Arduino and it breaks building//\/不要替换为串行回音对，因为在Arduino上只有文件名不能转换为PGM字符串，并且会破坏建筑
  #define BUG_ON(V...) do { SERIAL_ECHO(ONLY_FILENAME); SERIAL_ECHO(__LINE__); SERIAL_ECHOLNPGM(": BUG!"); SERIAL_ECHOLNPAIR(V); SERIAL_FLUSHTX(); } while(0)
#else
  // Release mode, let's ignore the bug//释放模式，让我们忽略错误
  #define BUG_ON(V...) NOOP
#endif
