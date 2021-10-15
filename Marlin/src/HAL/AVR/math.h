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
 * Optimized math functions for AVR
 */

// intRes = longIn1 * longIn2 >> 24//intRes=longIn1*longIn2>>24
// uses://用途：
// A[tmp] to store 0//要存储0的[tmp]
// B[tmp] to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.//B[tmp]存储48位结果的位16-23。顶部位用于对两个字节的结果进行四舍五入。
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.//请注意，未计算48位结果的下两个字节和上一个字节。
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.//这可能导致结果以1为单位输出，因为较低的字节可能导致进位进入较高的字节。
// B A are bits 24-39 and are the returned value//B A是位24-39，是返回值
// C B A is longIn1//C B A是最长的
// D C B A is longIn2//D C B A是长的2
////
static FORCE_INLINE uint16_t MultiU24X32toH16(uint32_t longIn1, uint32_t longIn2) {
  uint8_t tmp1;
  uint8_t tmp2;
  uint16_t intRes;
  __asm__ __volatile__(
    A("clr %[tmp1]")
    A("mul %A[longIn1], %B[longIn2]")
    A("mov %[tmp2], r1")
    A("mul %B[longIn1], %C[longIn2]")
    A("movw %A[intRes], r0")
    A("mul %C[longIn1], %C[longIn2]")
    A("add %B[intRes], r0")
    A("mul %C[longIn1], %B[longIn2]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %A[longIn1], %C[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %B[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %C[longIn1], %A[longIn2]")
    A("add %[tmp2], r0")
    A("adc %A[intRes], r1")
    A("adc %B[intRes], %[tmp1]")
    A("mul %B[longIn1], %A[longIn2]")
    A("add %[tmp2], r1")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("lsr %[tmp2]")
    A("adc %A[intRes], %[tmp1]")
    A("adc %B[intRes], %[tmp1]")
    A("mul %D[longIn2], %A[longIn1]")
    A("add %A[intRes], r0")
    A("adc %B[intRes], r1")
    A("mul %D[longIn2], %B[longIn1]")
    A("add %B[intRes], r0")
    A("clr r1")
      : [intRes] "=&r" (intRes),
        [tmp1] "=&r" (tmp1),
        [tmp2] "=&r" (tmp2)
      : [longIn1] "d" (longIn1),
        [longIn2] "d" (longIn2)
      : "cc"
  );
  return intRes;
}

// intRes = intIn1 * intIn2 >> 16//intRes=intIn1*intIn2>>16
// uses://用途：
// r26 to store 0//r26存储0
// r27 to store the byte 1 of the 24 bit result//r27存储24位结果的字节1
static FORCE_INLINE uint16_t MultiU16X8toH16(uint8_t charIn1, uint16_t intIn2) {
  uint8_t tmp;
  uint16_t intRes;
  __asm__ __volatile__ (
    A("clr %[tmp]")
    A("mul %[charIn1], %B[intIn2]")
    A("movw %A[intRes], r0")
    A("mul %[charIn1], %A[intIn2]")
    A("add %A[intRes], r1")
    A("adc %B[intRes], %[tmp]")
    A("lsr r0")
    A("adc %A[intRes], %[tmp]")
    A("adc %B[intRes], %[tmp]")
    A("clr r1")
      : [intRes] "=&r" (intRes),
        [tmp] "=&r" (tmp)
      : [charIn1] "d" (charIn1),
        [intIn2] "d" (intIn2)
      : "cc"
  );
  return intRes;
}
