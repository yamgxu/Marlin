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

#include "../../inc/MarlinConfigPre.h"

/**
 * Busy wait delay cycles routines:
 *
 *  DELAY_CYCLES(count): Delay execution in cycles
 *  DELAY_NS(count): Delay execution in nanoseconds
 *  DELAY_US(count): Delay execution in microseconds
 */

#include "../../core/macros.h"

void calibrate_delay_loop();

#if defined(__arm__) || defined(__thumb__)

  // We want to have delay_cycle function with the lowest possible overhead, so we adjust at the function at runtime based on the current CPU best feature//我们希望具有尽可能低的开销的delay_cycle函数，因此我们在运行时根据当前CPU的最佳特性对该函数进行调整
  typedef void (*DelayImpl)(uint32_t);
  extern DelayImpl DelayCycleFnc;

  // I've measured 36 cycles on my system to call the cycle waiting method, but it shouldn't change much to have a bit more margin, it only consume a bit more flash//我在我的系统上测量了36个周期来调用cycle waiting方法，但是它应该不会有太大的变化来获得更多的余量，它只会消耗更多的闪存
  #define TRIP_POINT_FOR_CALLING_FUNCTION   40

  // A simple recursive template class that output exactly one 'nop' of code per recursion//一个简单的递归模板类，每个递归只输出一个“nop”代码
  template <int N> struct NopWriter {
    FORCE_INLINE static void build() {
      __asm__ __volatile__("nop");
      NopWriter<N-1>::build();
    }
  };
  // End the loop//结束循环
  template <> struct NopWriter<0> { FORCE_INLINE static void build() {} };

  namespace Private {
    // Split recursing template in 2 different class so we don't reach the maximum template instantiation depth limit//将递归模板拆分为两个不同的类，这样我们就不会达到最大模板实例化深度限制
    template <bool belowTP, int N> struct Helper {
      FORCE_INLINE static void build() {
        DelayCycleFnc(N - 2); //  Approximative cost of calling the function (might be off by one or 2 cycles)//调用函数的近似成本（可能关闭一个或两个周期）
      }
    };

    template <int N> struct Helper<true, N> {
      FORCE_INLINE static void build() {
        NopWriter<N - 1>::build();
      }
    };

    template <> struct Helper<true, 0> {
      FORCE_INLINE static void build() {}
    };

  }
  // Select a behavior based on the constexpr'ness of the parameter//根据参数的常量选择行为
  // If called with a compile-time parameter, then write as many NOP as required to reach the asked cycle count//如果使用编译时参数调用，则根据需要写入尽可能多的NOP以达到请求的循环计数
  // (there is some tripping point here to start looping when it's more profitable than gruntly executing NOPs)//（当循环比粗暴执行NOP更有利可图时，这里有一些触发点可以开始循环）
  // If not called from a compile-time parameter, fallback to a runtime loop counting version instead//如果不是从编译时参数调用，则回退到运行时循环计数版本
  template <bool compileTime, int Cycles>
  struct SmartDelay {
    FORCE_INLINE SmartDelay(int) {
      if (Cycles == 0) return;
      Private::Helper<Cycles < TRIP_POINT_FOR_CALLING_FUNCTION, Cycles>::build();
    }
  };
  // Runtime version below. There is no way this would run under less than ~TRIP_POINT_FOR_CALLING_FUNCTION cycles//运行时版本如下。对于调用函数循环，这不可能在小于~TRIP\u POINT\u的情况下运行
  template <int T>
  struct SmartDelay<false, T> {
    FORCE_INLINE SmartDelay(int v) { DelayCycleFnc(v); }
  };

  #define DELAY_CYCLES(X) do { SmartDelay<IS_CONSTEXPR(X), IS_CONSTEXPR(X) ? X : 0> _smrtdly_X(X); } while(0)

  // For delay in microseconds, no smart delay selection is required, directly call the delay function//对于以微秒为单位的延迟，不需要智能延迟选择，直接调用延迟功能
  // Teensy compiler is too old and does not accept smart delay compile-time / run-time selection correctly//Teensy编译器太旧，无法正确接受智能延迟编译时/运行时选择
  #define DELAY_US(x) DelayCycleFnc((x) * ((F_CPU) / 1000000UL))

#elif defined(__AVR__)

  #define nop() __asm__ __volatile__("nop;\n\t":::)

  FORCE_INLINE static void __delay_4cycles(uint8_t cy) {
    __asm__ __volatile__(
      L("1")
      A("dec %[cnt]")
      A("nop")
      A("brne 1b")
      : [cnt] "+r"(cy)  // output: +r means input+output//输出：+r表示输入+输出
      :                 // input://输入：
      : "cc"            // clobbers://打击者：
    );
  }

  // Delay in cycles//周期延迟
  FORCE_INLINE static void DELAY_CYCLES(uint16_t x) {

    if (__builtin_constant_p(x)) {
      #define MAXNOPS 4

      if (x <= (MAXNOPS)) {
        switch (x) { case 4: nop(); case 3: nop(); case 2: nop(); case 1: nop(); }
      }
      else {
        const uint32_t rem = (x) % (MAXNOPS);
        switch (rem) { case 3: nop(); case 2: nop(); case 1: nop(); }
        if ((x = (x) / (MAXNOPS)))
          __delay_4cycles(x); // if need more then 4 nop loop is more optimal//如果需要更多，则4 nop循环更为优化
      }

      #undef MAXNOPS
    }
    else if ((x >>= 2))
      __delay_4cycles(x);
  }
  #undef nop

  // Delay in microseconds//以微秒为单位的延迟
  #define DELAY_US(x) DELAY_CYCLES((x) * ((F_CPU) / 1000000UL))

#elif defined(__PLAT_LINUX__) || defined(ESP32)

  // DELAY_CYCLES specified inside platform//平台内部规定的延迟周期

  // Delay in microseconds//以微秒为单位的延迟
  #define DELAY_US(x) DELAY_CYCLES((x) * ((F_CPU) / 1000000UL))
#else

  #error "Unsupported MCU architecture"

#endif

/**************************************************************
 *  Delay in nanoseconds. Requires the F_CPU macro.
 *  These macros follow avr-libc delay conventions.
 *
 * For AVR there are three possible operation modes, due to its
 * slower clock speeds and thus coarser delay resolution. For
 * example, when F_CPU = 16000000 the resolution is 62.5ns.
 *
 *  Round up (default)
 *    Round up the delay according to the CPU clock resolution.
 *    e.g., 100 will give a delay of 2 cycles (125ns).
 *
 *  Round down (DELAY_NS_ROUND_DOWN)
 *    Round down the delay according to the CPU clock resolution.
 *    e.g., 100 will be rounded down to 1 cycle (62.5ns).
 *
 *  Nearest (DELAY_NS_ROUND_CLOSEST)
 *    Round the delay to the nearest number of clock cycles.
 *    e.g., 165 will be rounded up to 3 cycles (187.5ns) because
 *          it's closer to the requested delay than 2 cycle (125ns).
 */

#ifndef __AVR__
  #undef DELAY_NS_ROUND_DOWN
  #undef DELAY_NS_ROUND_CLOSEST
#endif

#if ENABLED(DELAY_NS_ROUND_DOWN)
  #define DELAY_NS(x) DELAY_CYCLES((x) * ((F_CPU) / 1000000UL) / 1000UL)          // floor//地板
#elif ENABLED(DELAY_NS_ROUND_CLOSEST)
  #define DELAY_NS(x) DELAY_CYCLES(((x) * ((F_CPU) / 1000000UL) + 500) / 1000UL)  // round//圆的
#else
  #define DELAY_NS(x) DELAY_CYCLES(((x) * ((F_CPU) / 1000000UL) + 999) / 1000UL)  // "ceil"//“ceil”
#endif
