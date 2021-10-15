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

#include "../inc/MarlinConfigPre.h"
#include "../core/types.h"

// Format uint8_t (0-100) as rj string with 123% / _12% / __1% format//使用123%/\u 12%/\u 1%格式将uint8\u t（0-100）格式化为rj字符串
const char* pcttostrpctrj(const uint8_t i);

// Convert uint8_t (0-255) to a percentage, format as above//将uint8_t（0-255）转换为百分比，格式如上所述
const char* ui8tostr4pctrj(const uint8_t i);

// Convert uint8_t to string with 12 format//将uint8\u t转换为12格式的字符串
const char* ui8tostr2(const uint8_t x);

// Convert uint8_t to string with 123 format//将uint8\u t转换为123格式的字符串
const char* ui8tostr3rj(const uint8_t i);

// Convert int8_t to string with 123 format//将int8_t转换为123格式的字符串
const char* i8tostr3rj(const int8_t x);

#if HAS_PRINT_PROGRESS_PERMYRIAD
  // Convert 16-bit unsigned permyriad value to percent: 100 / 23 / 23.4 / 3.45//将16位无符号永久变量值转换为百分比：100/23/23.4/3.45
  const char* permyriadtostr4(const uint16_t xx);
#endif

// Convert uint16_t to string with 12345 format//将uint16\u t转换为12345格式的字符串
const char* ui16tostr5rj(const uint16_t x);

// Convert uint16_t to string with 1234 format//将uint16\u t转换为1234格式的字符串
const char* ui16tostr4rj(const uint16_t x);

// Convert uint16_t to string with 123 format//将uint16\u t转换为123格式的字符串
const char* ui16tostr3rj(const uint16_t x);

// Convert int16_t to string with 123 format//将int16_t转换为123格式的字符串
const char* i16tostr3rj(const int16_t x);

// Convert unsigned int to lj string with 123 format//将无符号int转换为123格式的lj字符串
const char* i16tostr3left(const int16_t xx);

// Convert signed int to rj string with _123, -123, _-12, or __-1 format//将带符号整数转换为带有_123、-123、-12或-1格式的rj字符串
const char* i16tostr4signrj(const int16_t x);

// Convert unsigned float to string with 1.2 format//将无符号浮点转换为1.2格式的字符串
const char* ftostr11ns(const_float_t x);

// Convert unsigned float to string with 1.23 format//将无符号浮点转换为1.23格式的字符串
const char* ftostr12ns(const_float_t x);

// Convert unsigned float to string with 12.3 format//将无符号浮点转换为12.3格式的字符串
const char* ftostr31ns(const_float_t x);

// Convert unsigned float to string with 123.4 format//将无符号浮点转换为123.4格式的字符串
const char* ftostr41ns(const_float_t x);

// Convert signed float to fixed-length string with 12.34 / _2.34 / -2.34 or -23.45 / 123.45 format//将有符号浮点转换为固定长度字符串，格式为12.34/_2.34/-2.34或-23.45/123.45
const char* ftostr42_52(const_float_t x);

// Convert signed float to fixed-length string with 023.45 / -23.45 format//将带符号浮点转换为023.45/-23.45格式的固定长度字符串
const char* ftostr52(const_float_t x);

// Convert signed float to fixed-length string with 12.345 / -2.345 or 023.456 / -23.456 format//将带符号浮点转换为12.345/-2.345或023.456/-23.456格式的固定长度字符串
const char* ftostr53_63(const_float_t x);

// Convert signed float to fixed-length string with 023.456 / -23.456 format//将带符号浮点转换为023.456/-23.456格式的固定长度字符串
const char* ftostr63(const_float_t x);

// Convert float to fixed-length string with +12.3 / -12.3 format//将浮点转换为固定长度字符串，格式为+12.3/-12.3
const char* ftostr31sign(const_float_t x);

// Convert float to fixed-length string with +123.4 / -123.4 format//将浮点转换为固定长度字符串，格式为+123.4/-123.4
const char* ftostr41sign(const_float_t x);

// Convert signed float to string (6 digit) with -1.234 / _0.000 / +1.234 format//将有符号浮点转换为-1.234/_0.000/+1.234格式的字符串（6位）
const char* ftostr43sign(const_float_t x, char plus=' ');

// Convert signed float to string (5 digit) with -1.2345 / _0.0000 / +1.2345 format//将带符号浮点转换为-1.2345/_0.0000/+1.2345格式的字符串（5位）
const char* ftostr54sign(const_float_t x, char plus=' ');

// Convert unsigned float to rj string with 12345 format//将无符号浮点转换为12345格式的rj字符串
const char* ftostr5rj(const_float_t x);

// Convert signed float to string with +1234.5 format//将带符号浮点转换为+1234.5格式的字符串
const char* ftostr51sign(const_float_t x);

// Convert signed float to space-padded string with -_23.4_ format//将带符号浮点转换为带空格的字符串，格式为-_23.4_
const char* ftostr52sp(const_float_t x);

// Convert signed float to string with +123.45 format//将带符号浮点转换为+123.45格式的字符串
const char* ftostr52sign(const_float_t x);

// Convert signed float to string with +12.345 format//将带符号浮点转换为+12.345格式的字符串
const char* ftostr53sign(const_float_t f);

// Convert unsigned float to string with 1234.5 format omitting trailing zeros//将无符号浮点转换为1234.5格式的字符串，省略尾随零
const char* ftostr51rj(const_float_t x);

// Convert float to rj string with 123 or -12 format//将float转换为123或-12格式的rj字符串
FORCE_INLINE const char* ftostr3(const_float_t x) { return i16tostr3rj(int16_t(x + (x < 0 ? -0.5f : 0.5f))); }

#if ENABLED(LCD_DECIMAL_SMALL_XY)
  // Convert float to rj string with 1234, _123, 12.3, _1.2, -123, _-12, or -1.2 format//使用1234、_123、12.3、_1.2、-123、12或-1.2格式将浮点转换为rj字符串
  const char* ftostr4sign(const_float_t fx);
#else
  // Convert float to rj string with 1234, _123, -123, __12, _-12, ___1, or __-1 format//将浮点转换为rj字符串，格式为1234、_123、-123、uu 12、u-12、uu-1或u-1
  FORCE_INLINE const char* ftostr4sign(const_float_t x) { return i16tostr4signrj(int16_t(x + (x < 0 ? -0.5f : 0.5f))); }
#endif
