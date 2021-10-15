/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../core/serial.h"
#include <stdint.h>

// Serial stuff here//这里有连载的东西
// Inside an exception handler, the CPU state is not safe, we can't expect the handler to resume//在异常处理程序中，CPU状态不安全，我们不能期望处理程序恢复
// and the software to continue. UART communication can't rely on later callback/interrupt as it might never happen.//和软件继续。UART通信不能依赖于以后的回调/中断，因为它可能永远不会发生。
// So, you need to provide some method to send one byte to the usual UART with the interrupts disabled//因此，您需要提供一些方法，在禁用中断的情况下将一个字节发送到常用的UART
// By default, the method uses SERIAL_CHAR but it's 100% guaranteed to break (couldn't be worse than nothing...)7//默认情况下，该方法使用SERIAL_CHAR，但它100%保证会中断（不会比没有更糟…）7
extern void (*HAL_min_serial_init)();
extern void (*HAL_min_serial_out)(char ch);

struct MinSerial {
  static bool force_using_default_output;
  // Serial output//串行输出
  static void TX(char ch) {
    if (force_using_default_output)
      SERIAL_CHAR(ch);
    else
      HAL_min_serial_out(ch);
  }
  // Send String through UART//通过UART发送字符串
  static void TX(const char *s) { while (*s) TX(*s++); }
  // Send a digit through UART//通过UART发送一个数字
  static void TXDigit(uint32_t d) {
    if (d < 10) TX((char)(d+'0'));
    else if (d < 16) TX((char)(d+'A'-10));
    else TX('?');
  }

  // Send Hex number through UART//通过UART发送十六进制数
  static void TXHex(uint32_t v) {
    TX("0x");
    for (uint8_t i = 0; i < 8; i++, v <<= 4)
      TXDigit((v >> 28) & 0xF);
  }

  // Send Decimal number through UART//通过UART发送十进制数
  static void TXDec(uint32_t v) {
    if (!v) {
      TX('0');
      return;
    }

    char nbrs[14];
    char *p = &nbrs[0];
    while (v != 0) {
      *p++ = '0' + (v % 10);
      v /= 10;
    }
    do {
      p--;
      TX(*p);
    } while (p != &nbrs[0]);
  }
  static void init() { if (!force_using_default_output) HAL_min_serial_init(); }
};
