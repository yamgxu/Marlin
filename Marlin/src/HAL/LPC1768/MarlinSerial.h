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

#include <HardwareSerial.h>
#include <WString.h>

#include "../../inc/MarlinConfigPre.h"
#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif
#include "../../core/serial_hook.h"

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 32
#endif

class MarlinSerial : public HardwareSerial<RX_BUFFER_SIZE, TX_BUFFER_SIZE> {
public:
  MarlinSerial(LPC_UART_TypeDef *UARTx) : HardwareSerial<RX_BUFFER_SIZE, TX_BUFFER_SIZE>(UARTx) { }

  void end() {}

  #if ENABLED(EMERGENCY_PARSER)
    bool recv_callback(const char c) override;
  #endif
};

// On LPC176x framework, HardwareSerial does not implement the same interface as Arduino's Serial, so overloads//在LPC176x框架上，HardwareSerial没有实现与Arduino的串行接口相同的接口，因此重载
// of 'available' and 'read' method are not used in this multiple inheritance scenario.//在这个多重继承方案中，不使用'available'和'read'方法。
// Instead, use a ForwardSerial here that adapts the interface.//相反，在此处使用一个可调整接口的ForwardSerial。
typedef ForwardSerial1Class<MarlinSerial> MSerialT;
extern MSerialT MSerial0;
extern MSerialT MSerial1;
extern MSerialT MSerial2;
extern MSerialT MSerial3;

// Consequently, we can't use a RuntimeSerial either. The workaround would be to use//因此，我们也不能使用RuntimeSerial。解决办法是使用
// a RuntimeSerial<ForwardSerial<MarlinSerial>> type here. Ignore for now until it's actually required.//此处键入RuntimeSerial<ForwardSerial<MarlinSerial>>类型。暂时忽略，直到实际需要为止。
#if ENABLED(SERIAL_RUNTIME_HOOK)
  #error "SERIAL_RUNTIME_HOOK is not yet supported for LPC176x."
#endif
