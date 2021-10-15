/** translatione by yx */
/*****************
 * bitmap_info.h *
 *****************/

/****************************************************************************
 *   Written By Marcio Teixeira 2019 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#pragma once

#ifndef FORCEDINLINE
  #define FORCEDINLINE __attribute__((always_inline)) inline
#endif

namespace FTDI {
   // The following functions *must* be inlined since we are relying on the compiler to do//以下函数*必须*内联，因为我们依赖于编译器来完成
   // substitution of the constants from the data structure rather than actually storing//从数据结构中替换常量，而不是实际存储
   // it in PROGMEM (which would fail, since we are not using pgm_read to read them).//它在PROGMEM中运行（这将失败，因为我们没有使用pgm_read来读取它们）。
   // Plus, by inlining, all the equations are evaluated at compile-time as everything//另外，通过内联，所有的方程在编译时都作为所有的函数进行计算
   // should be a constant.//应该是一个常数。

   typedef struct {
     const uint8_t  format;
     const uint16_t linestride;
     const uint8_t  filter;
     const uint8_t  wrapx;
     const uint8_t  wrapy;
     const uint32_t RAMG_offset;
     const uint16_t width;
     const uint16_t height;
   } bitmap_info_t;

   FORCEDINLINE uint32_t BITMAP_SOURCE (const bitmap_info_t& info) {return BITMAP_SOURCE (ftdi_memory_map::RAM_G + info.RAMG_offset);};
   FORCEDINLINE uint32_t BITMAP_LAYOUT (const bitmap_info_t& info) {return BITMAP_LAYOUT (info.format, info.linestride, info.height);};
   FORCEDINLINE uint32_t BITMAP_SIZE   (const bitmap_info_t& info) {return BITMAP_SIZE   (info.filter, info.wrapx, info.wrapy, info.width, info.height);}
}
