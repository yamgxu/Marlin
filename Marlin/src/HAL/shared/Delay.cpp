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
#include "Delay.h"

#include "../../inc/MarlinConfig.h"

#if defined(__arm__) || defined(__thumb__)

  static uint32_t ASM_CYCLES_PER_ITERATION = 4;   // Initial bet which will be adjusted in calibrate_delay_loop//初始下注将在calibrate_delay_循环中调整

  // Simple assembler loop counting down//简单汇编程序循环倒计时
  void delay_asm(uint32_t cy) {
    cy = _MAX(cy / ASM_CYCLES_PER_ITERATION, 1U); // Zero is forbidden here//这里禁止零
    __asm__ __volatile__(
      A(".syntax unified") // is to prevent CM0,CM1 non-unified syntax//是为了防止CM0、CM1语法不统一
      L("1")
      A("subs %[cnt],#1")
      A("bne 1b")
      : [cnt]"+r"(cy)   // output: +r means input+output//输出：+r表示输入+输出
      :                 // input://输入：
      : "cc"            // clobbers://打击者：
    );
  }

  // We can't use CMSIS since it's not available on all platform, so fallback to hardcoded register values//我们不能使用CMSIS，因为它在所有平台上都不可用，所以要回退到硬编码寄存器值
  #define HW_REG(X)   *(volatile uint32_t *)(X)
  #define _DWT_CTRL   0xE0001000
  #define _DWT_CYCCNT 0xE0001004      // CYCCNT is 32bits, takes 37s or so to wrap.//CYCCNT是32位的，大约需要37秒来包装。
  #define _DEM_CR     0xE000EDFC
  #define _LAR        0xE0001FB0

  // Use hardware cycle counter instead, it's much safer//改用硬件循环计数器，更安全
  void delay_dwt(uint32_t count) {
    // Reuse the ASM_CYCLES_PER_ITERATION variable to avoid wasting another useless variable//重用ASM_CYCLES_PER_迭代变量，以避免浪费另一个无用变量
    uint32_t start = HW_REG(_DWT_CYCCNT) - ASM_CYCLES_PER_ITERATION, elapsed;
    do {
      elapsed = HW_REG(_DWT_CYCCNT) - start;
    } while (elapsed < count);
  }

  // Pointer to asm function, calling the functions has a ~20 cycles overhead//指向asm函数的指针，调用函数的开销约为20个周期
  DelayImpl DelayCycleFnc = delay_asm;

  void calibrate_delay_loop() {
    // Check if we have a working DWT implementation in the CPU (see https://developer.arm.com/documentation/ddi0439/b/Data-Watchpoint-and-Trace-Unit/DWT-Programmers-Model)//检查CPU中是否有工作的DWT实现（请参阅https://developer.arm.com/documentation/ddi0439/b/Data-Watchpoint-and-Trace-Unit/DWT-Programmers-Model)
    if (!HW_REG(_DWT_CTRL)) {
      // No DWT present, so fallback to plain old ASM nop counting//不存在DWT，因此退回到普通的旧ASM nop计数
      // Unfortunately, we don't exactly know how many iteration it'll take to decrement a counter in a loop//不幸的是，我们不知道在循环中减少一个计数器需要多少次迭代
      // It depends on the CPU architecture, the code current position (flash vs SRAM)//它取决于CPU架构、代码当前位置（闪存与SRAM）
      // So, instead of wild guessing and making mistake, instead//因此，与其胡乱猜测和犯错，不如
      // compute it once for all//一次计算一次
      ASM_CYCLES_PER_ITERATION = 1;
      // We need to fetch some reference clock before waiting//在等待之前，我们需要取一些参考时钟
      cli();
        uint32_t start = micros();
        delay_asm(1000); // On a typical CPU running in MHz, waiting 1000 "unknown cycles" means it'll take between 1ms to 6ms, that's perfectly acceptable//在以兆赫为单位运行的典型CPU上，等待1000个“未知周期”意味着需要1ms到6ms，这是完全可以接受的
        uint32_t end = micros();
      sei();
      uint32_t expectedCycles = (end - start) * ((F_CPU) / 1000000UL); // Convert microseconds to cycles//将微秒转换为周期
      // Finally compute the right scale//最后计算出正确的比例
      ASM_CYCLES_PER_ITERATION = (uint32_t)(expectedCycles / 1000);

      // No DWT present, likely a Cortex M0 so NOP counting is our best bet here//不存在DWT，可能是皮质M0，所以NOP计数是我们最好的选择
      DelayCycleFnc = delay_asm;
    }
    else {
      // Enable DWT counter//启用DWT计数器
      // From https://stackoverflow.com/a/41188674/1469714//从https://stackoverflow.com/a/41188674/1469714
      HW_REG(_DEM_CR) = HW_REG(_DEM_CR) | 0x01000000;   // Enable trace//启用跟踪
      #if __CORTEX_M == 7
        HW_REG(_LAR) = 0xC5ACCE55;                      // Unlock access to DWT registers, see https://developer.arm.com/documentation/ihi0029/e/ section B2.3.10//解锁对DWT寄存器的访问，请参阅https://developer.arm.com/documentation/ihi0029/e/ 第B2.3.10节
      #endif
      HW_REG(_DWT_CYCCNT) = 0;                          // Clear DWT cycle counter//清除DWT循环计数器
      HW_REG(_DWT_CTRL) = HW_REG(_DWT_CTRL) | 1;        // Enable DWT cycle counter//启用DWT循环计数器

      // Then calibrate the constant offset from the counter//然后校准与计数器的恒定偏移量
      ASM_CYCLES_PER_ITERATION = 0;
      uint32_t s = HW_REG(_DWT_CYCCNT);
      uint32_t e = HW_REG(_DWT_CYCCNT);  // (e - s) contains the number of cycle required to read the cycle counter//（e-s）包含读取循环计数器所需的循环数
      delay_dwt(0);
      uint32_t f = HW_REG(_DWT_CYCCNT);  // (f - e) contains the delay to call the delay function + the time to read the cycle counter//（f-e）包含调用延迟函数的延迟+读取周期计数器的时间
      ASM_CYCLES_PER_ITERATION = (f - e) - (e - s);

      // Use safer DWT function//使用更安全的DWT函数
      DelayCycleFnc = delay_dwt;
    }
  }

  #if ENABLED(MARLIN_DEV_MODE)
    void dump_delay_accuracy_check() {
      auto report_call_time = [](PGM_P const name, PGM_P const unit, const uint32_t cycles, const uint32_t total, const bool do_flush=true) {
        SERIAL_ECHOPGM("Calling ");
        SERIAL_ECHOPGM_P(name);
        SERIAL_ECHOLNPAIR(" for ", cycles);
        SERIAL_ECHOPGM_P(unit);
        SERIAL_ECHOLNPAIR(" took: ", total);
        SERIAL_ECHOPGM_P(unit);
        if (do_flush) SERIAL_FLUSHTX();
      };

      uint32_t s, e;

      SERIAL_ECHOLNPAIR("Computed delay calibration value: ", ASM_CYCLES_PER_ITERATION);
      SERIAL_FLUSH();
      // Display the results of the calibration above//显示上述校准结果
      constexpr uint32_t testValues[] = { 1, 5, 10, 20, 50, 100, 150, 200, 350, 500, 750, 1000 };
      for (auto i : testValues) {
        s = micros(); DELAY_US(i); e = micros();
        report_call_time(PSTR("delay"), PSTR("us"), i, e - s);
      }

      if (HW_REG(_DWT_CTRL)) {
        for (auto i : testValues) {
          s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(i); e = HW_REG(_DWT_CYCCNT);
          report_call_time(PSTR("runtime delay"), PSTR("cycles"), i, e - s);
        }

        // Measure the delay to call a real function compared to a function pointer//与函数指针相比，测量调用实函数的延迟
        s = HW_REG(_DWT_CYCCNT); delay_dwt(1); e = HW_REG(_DWT_CYCCNT);
        report_call_time(PSTR("delay_dwt"), PSTR("cycles"), 1, e - s);

        static PGMSTR(dcd, "DELAY_CYCLES directly ");

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES( 1); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"),  1, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES( 5); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"),  5, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(10); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"), 10, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(20); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"), 20, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(50); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"), 50, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(100); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"), 100, e - s, false);

        s = HW_REG(_DWT_CYCCNT); DELAY_CYCLES(200); e = HW_REG(_DWT_CYCCNT);
        report_call_time(dcd, PSTR("cycles"), 200, e - s, false);
      }
    }
  #endif // MARLIN_DEV_MODE//马林鱼开发模式


#else

  void calibrate_delay_loop() {}
  #if ENABLED(MARLIN_DEV_MODE)
    void dump_delay_accuracy_check() { SERIAL_ECHOPGM_P(PSTR("N/A on this platform")); }
  #endif

#endif
