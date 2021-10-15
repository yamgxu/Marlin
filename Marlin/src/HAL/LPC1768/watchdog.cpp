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
#ifdef TARGET_LPC1768

#include "../../inc/MarlinConfig.h"

#if ENABLED(USE_WATCHDOG)

#include <lpc17xx_wdt.h>
#include "watchdog.h"

#define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout//4或8秒超时

void watchdog_init() {
  #if ENABLED(WATCHDOG_RESET_MANUAL)
    // We enable the watchdog timer, but only for the interrupt.//我们启用看门狗定时器，但仅用于中断。

    // Configure WDT to only trigger an interrupt//将WDT配置为仅触发中断
    // Disable WDT interrupt (just in case, to avoid triggering it!)//禁用WDT中断（以防万一，以避免触发它！）
    NVIC_DisableIRQ(WDT_IRQn);

    // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
    // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
    __DSB();
    __ISB();

    // Configure WDT to only trigger an interrupt//将WDT配置为仅触发中断
    // Initialize WDT with the given parameters//使用给定参数初始化WDT
    WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_INT_ONLY);

    // Configure and enable WDT interrupt.//配置并启用WDT中断。
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0); // Use highest priority, so we detect all kinds of lockups//使用最高优先级，因此我们可以检测所有类型的锁定
    NVIC_EnableIRQ(WDT_IRQn);
  #else
    WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
  #endif
  WDT_Start(WDT_TIMEOUT_US);
}

void HAL_watchdog_refresh() {
  WDT_Feed();
  #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
    TOGGLE(LED_PIN);  // heartbeat indicator//心跳指示器
  #endif
}

// Timeout state//超时状态
bool watchdog_timed_out() { return TEST(WDT_ReadTimeOutFlag(), 0); }
void watchdog_clear_timeout_flag() { WDT_ClrTimeOutFlag(); }

#endif // USE_WATCHDOG//使用看门狗
#endif // TARGET_LPC1768//目标为LPC1768
