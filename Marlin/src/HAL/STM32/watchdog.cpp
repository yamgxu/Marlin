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
#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(USE_WATCHDOG)

#define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout//4或8秒超时

#include "../../inc/MarlinConfig.h"

#include "watchdog.h"
#include <IWatchdog.h>

void watchdog_init() {
  #if DISABLED(DISABLE_WATCHDOG_INIT)
    IWatchdog.begin(WDT_TIMEOUT_US);
  #endif
}

void HAL_watchdog_refresh() {
  IWatchdog.reload();
  #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
    TOGGLE(LED_PIN);  // heartbeat indicator//心跳指示器
  #endif
}

#endif // USE_WATCHDOG//使用看门狗
#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC//ARDUINO_ARCH_STM32&&！STM32通用
