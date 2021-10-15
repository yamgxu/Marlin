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
#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "../../MarlinCore.h"
#include "watchdog.h"

// Override Arduino runtime to either config or disable the watchdog//覆盖Arduino运行时以配置或禁用看门狗
////
// We need to configure the watchdog as soon as possible in the boot//我们需要在引导中尽快配置看门狗
// process, because watchdog initialization at hardware reset on SAM3X8E//进程，因为SAM3X8E上硬件重置时的看门狗初始化
// is unreliable, and there is risk of unintended resets if we delay//不可靠，如果我们延迟，则存在意外重置的风险
// that initialization to a later time.//该初始化将推迟到以后。
void watchdogSetup() {

  #if ENABLED(USE_WATCHDOG)

    // 4 seconds timeout//4秒超时
    uint32_t timeout = TERN(WATCHDOG_DURATION_8S, 8000, 4000);

    // Calculate timeout value in WDT counter ticks: This assumes//以WDT计数器刻度计算超时值：假设
    // the slow clock is running at 32.768 kHz watchdog//慢时钟以32.768 kHz的频率运行
    // frequency is therefore 32768 / 128 = 256 Hz//因此，频率为32768/128=256 Hz
    timeout = (timeout << 8) / 1000;
    if (timeout == 0)
      timeout = 1;
    else if (timeout > 0xFFF)
      timeout = 0xFFF;

    // We want to enable the watchdog with the specified timeout//我们希望启用指定超时的看门狗
    uint32_t value =
      WDT_MR_WDV(timeout) |               // With the specified timeout//使用指定的超时
      WDT_MR_WDD(timeout) |               // and no invalid write window//并且没有无效的写入窗口
    #if !(SAMV70 || SAMV71 || SAME70 || SAMS70)
      WDT_MR_WDRPROC   |                  // WDT fault resets processor only - We want//WDT故障仅重置处理器-我们需要
                                          // to keep PIO controller state//保持PIO控制器状态
    #endif
      WDT_MR_WDDBGHLT  |                  // WDT stops in debug state.//WDT在调试状态下停止。
      WDT_MR_WDIDLEHLT;                   // WDT stops in idle state.//WDT在空闲状态下停止。

    #if ENABLED(WATCHDOG_RESET_MANUAL)
      // We enable the watchdog timer, but only for the interrupt.//我们启用看门狗定时器，但仅用于中断。

      // Configure WDT to only trigger an interrupt//将WDT配置为仅触发中断
      value |= WDT_MR_WDFIEN;             // Enable WDT fault interrupt.//启用WDT故障中断。

      // Disable WDT interrupt (just in case, to avoid triggering it!)//禁用WDT中断（以防万一，以避免触发它！）
      NVIC_DisableIRQ(WDT_IRQn);

      // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
      // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
      __DSB();
      __ISB();

      // Initialize WDT with the given parameters//使用给定参数初始化WDT
      WDT_Enable(WDT, value);

      // Configure and enable WDT interrupt.//配置并启用WDT中断。
      NVIC_ClearPendingIRQ(WDT_IRQn);
      NVIC_SetPriority(WDT_IRQn, 0); // Use highest priority, so we detect all kinds of lockups//使用最高优先级，因此我们可以检测所有类型的锁定
      NVIC_EnableIRQ(WDT_IRQn);

    #else

      // a WDT fault triggers a reset//WDT故障会触发重置
      value |= WDT_MR_WDRSTEN;

      // Initialize WDT with the given parameters//使用给定参数初始化WDT
      WDT_Enable(WDT, value);

    #endif

    // Reset the watchdog//重置看门狗
    WDT_Restart(WDT);

  #else

    // Make sure to completely disable the Watchdog//确保完全禁用看门狗
    WDT_Disable(WDT);

  #endif
}

#if ENABLED(USE_WATCHDOG)
  // Initialize watchdog - On SAM3X, Watchdog was already configured//初始化看门狗-在SAM3X上，已配置看门狗
  //  and enabled or disabled at startup, so no need to reconfigure it//并在启动时启用或禁用，因此无需重新配置
  //  here.//在这里。
  void watchdog_init() {
    // Reset watchdog to start clean//重置看门狗以开始清理
    WDT_Restart(WDT);
  }
#endif // USE_WATCHDOG//使用看门狗

#endif
