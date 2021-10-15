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
#ifdef __AVR__

#include "../../inc/MarlinConfig.h"

#if ENABLED(USE_WATCHDOG)

#include "watchdog.h"

#include "../../MarlinCore.h"

// Initialize watchdog with 8s timeout, if possible. Otherwise, make it 4s.//如果可能，用8s超时初始化看门狗。否则，将其设置为4s。
void watchdog_init() {
  #if ENABLED(WATCHDOG_DURATION_8S) && defined(WDTO_8S)
    #define WDTO_NS WDTO_8S
  #else
    #define WDTO_NS WDTO_4S
  #endif
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    // Enable the watchdog timer, but only for the interrupt.//启用看门狗定时器，但仅用于中断。
    // Take care, as this requires the correct order of operation, with interrupts disabled.//当心，因为这需要正确的操作顺序，并禁用中断。
    // See the datasheet of any AVR chip for details.//有关详细信息，请参阅任何AVR芯片的数据表。
    wdt_reset();
    cli();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    _WD_CONTROL_REG = _BV(WDIE) | (WDTO_NS & 0x07) | ((WDTO_NS & 0x08) << 2); // WDTO_NS directly does not work. bit 0-2 are consecutive in the register but the highest value bit is at bit 5//直接使用WDTO\n不起作用。位0-2在寄存器中是连续的，但最高值位在位5
                                                                              // So worked for up to WDTO_2S//因此，工作时间长达2个月
    sei();
    wdt_reset();
  #else
    wdt_enable(WDTO_NS); // The function handles the upper bit correct.//该函数正确处理高位。
  #endif
  //delay(10000); // test it!//延迟（10000）；//试试看！
}

//===========================================================================//===========================================================================
//=================================== ISR ===================================//=============================================================ISR===================================
//===========================================================================//===========================================================================

// Watchdog timer interrupt, called if main program blocks >4sec and manual reset is enabled.//看门狗定时器中断，在主程序块>4秒且手动复位启用时调用。
#if ENABLED(WATCHDOG_RESET_MANUAL)
  ISR(WDT_vect) {
    sei();  // With the interrupt driven serial we need to allow interrupts.//对于中断驱动的串行，我们需要允许中断。
    SERIAL_ERROR_MSG(STR_WATCHDOG_FIRED);
    minkill();  // interrupt-safe final kill and infinite loop//中断安全最终杀死和无限循环
  }
#endif

#endif // USE_WATCHDOG//使用看门狗
#endif // __AVR__//_uuuavr__
