/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
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

#include "../../inc/MarlinConfig.h"

// ------------------------// ------------------------
// Local defines//局部定义
// ------------------------// ------------------------

// Default timer priorities. Override by specifying alternate priorities in the board pins file.//默认计时器优先级。通过在线路板引脚文件中指定备用优先级进行替代。
// The TONE timer is not present here, as it currently cannot be set programmatically. It is set//此处不存在音调计时器，因为当前无法以编程方式设置它。一切都安排好了
// by defining TIM_IRQ_PRIO in the variant.h or platformio.ini file, which adjusts the default//通过在variant.h或platformio.ini文件中定义TIM_IRQ_PRIO，可以调整默认值
// priority for STM32 HardwareTimer objects.//STM32 HardwareTimer对象的优先级。
#define SWSERIAL_TIMER_IRQ_PRIO_DEFAULT  1 // Requires tight bit timing to communicate reliably with TMC drivers//需要严格的位定时，以便与TMC驱动程序可靠通信
#define SERVO_TIMER_IRQ_PRIO_DEFAULT     1 // Requires tight PWM timing to control a BLTouch reliably//需要严格的PWM定时以可靠地控制BLTouch
#define STEP_TIMER_IRQ_PRIO_DEFAULT      2
#define TEMP_TIMER_IRQ_PRIO_DEFAULT     14 // Low priority avoids interference with other hardware and timers//低优先级可避免与其他硬件和计时器的干扰

#ifndef STEP_TIMER_IRQ_PRIO
  #define STEP_TIMER_IRQ_PRIO STEP_TIMER_IRQ_PRIO_DEFAULT
#endif
#ifndef TEMP_TIMER_IRQ_PRIO
  #define TEMP_TIMER_IRQ_PRIO TEMP_TIMER_IRQ_PRIO_DEFAULT
#endif
#if HAS_TMC_SW_SERIAL
  #include <SoftwareSerial.h>
  #ifndef SWSERIAL_TIMER_IRQ_PRIO
    #define SWSERIAL_TIMER_IRQ_PRIO SWSERIAL_TIMER_IRQ_PRIO_DEFAULT
  #endif
#endif
#if HAS_SERVOS
  #include "Servo.h"
  #ifndef SERVO_TIMER_IRQ_PRIO
    #define SERVO_TIMER_IRQ_PRIO SERVO_TIMER_IRQ_PRIO_DEFAULT
  #endif
#endif
#if ENABLED(SPEAKER)
  // Ensure the default timer priority is somewhere between the STEP and TEMP priorities.//确保默认计时器优先级介于步骤优先级和临时优先级之间。
  // The STM32 framework defaults to interrupt 14 for all timers. This should be increased so that//STM32框架默认为所有计时器的中断14。这应该增加，以便
  // timing-sensitive operations such as speaker output are not impacted by the long-running//对时间敏感的操作（如扬声器输出）不受长时间运行的扬声器的影响
  // temperature ISR. This must be defined in the platformio.ini file or the board's variant.h,//温度ISR。这必须在platformio.ini文件或电路板的variant.h中定义，
  // so that it will be consumed by framework code.//因此，它将被框架代码使用。
  #if !(TIM_IRQ_PRIO > STEP_TIMER_IRQ_PRIO && TIM_IRQ_PRIO < TEMP_TIMER_IRQ_PRIO)
    #error "Default timer interrupt priority is unspecified or set to a value which may degrade performance."
  #endif
#endif

#ifdef STM32F0xx
  #define MCU_STEP_TIMER 16
  #define MCU_TEMP_TIMER 17
#elif defined(STM32F1xx)
  #define MCU_STEP_TIMER  4
  #define MCU_TEMP_TIMER  2
#elif defined(STM32F401xC) || defined(STM32F401xE)
  #define MCU_STEP_TIMER  9
  #define MCU_TEMP_TIMER 10
#elif defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32H7xx)
  #define MCU_STEP_TIMER  6           // STM32F401 has no TIM6, TIM7, or TIM8//STM32F401没有TIM6、TIM7或TIM8
  #define MCU_TEMP_TIMER 14           // TIM7 is consumed by Software Serial if used.//TIM7由串行软件（如果使用）消耗。
#endif

#ifndef HAL_TIMER_RATE
  #define HAL_TIMER_RATE GetStepperTimerClkFreq()
#endif

#ifndef STEP_TIMER
  #define STEP_TIMER MCU_STEP_TIMER
#endif
#ifndef TEMP_TIMER
  #define TEMP_TIMER MCU_TEMP_TIMER
#endif

#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)
#define TEMP_TIMER_DEV _TIMER_DEV(TEMP_TIMER)

// ------------------------// ------------------------
// Private Variables//私有变量
// ------------------------// ------------------------

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { nullptr };

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

uint32_t GetStepperTimerClkFreq() {
  // Timer input clocks vary between devices, and in some cases between timers on the same device.//定时器输入时钟在不同设备之间变化，有时在同一设备上的定时器之间变化。
  // Retrieve at runtime to ensure device compatibility. Cache result to avoid repeated overhead.//在运行时检索以确保设备兼容性。缓存结果以避免重复开销。
  static uint32_t clkfreq = timer_instance[STEP_TIMER_NUM]->getTimerClkFreq();
  return clkfreq;
}

// frequency is in Hertz//频率单位为赫兹
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!HAL_timer_initialized(timer_num)) {
    switch (timer_num) {
      case STEP_TIMER_NUM: // STEPPER TIMER - use a 32bit timer if possible//步进定时器-如果可能，使用32位定时器
        timer_instance[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        /* Set the prescaler to the final desired value.
         * This will change the effective ISR callback frequency but when
         * HAL_timer_start(timer_num=0) is called in the core for the first time
         * the real frequency isn't important as long as, after boot, the ISR
         * gets called with the correct prescaler and count register. So here
         * we set the prescaler to the correct, final value and ignore the frequency
         * asked. We will call back the ISR in 1 second to start at full speed.
         *
         * The proper fix, however, would be a correct initialization OR a
         * HAL_timer_change(const uint8_t timer_num, const uint32_t frequency)
         * which changes the prescaler when an IRQ frequency change is needed
         * (for example when steppers are turned on)
         */

        timer_instance[timer_num]->setPrescaleFactor(STEPPER_TIMER_PRESCALE); //the -1 is done internally//-1是在内部完成的
        timer_instance[timer_num]->setOverflow(_MIN(hal_timer_t(HAL_TIMER_TYPE_MAX), (HAL_TIMER_RATE) / (STEPPER_TIMER_PRESCALE) /* /frequency */), TICK_FORMAT);
        break;
      case TEMP_TIMER_NUM: // TEMP TIMER - any available 16bit timer//临时计时器-任何可用的16位计时器
        timer_instance[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        // The prescale factor is computed automatically for HERTZ_FORMAT//对于HERTZ_格式，会自动计算预刻度因子
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        break;
    }

    // Disable preload. Leaving it default-enabled can cause the timer to stop if it happens//禁用预加载。如果启用默认设置，则会导致计时器停止
    // to exit the ISR after the start time for the next interrupt has already passed.//在下一个中断的开始时间已经过去之后退出ISR。
    timer_instance[timer_num]->setPreloadEnable(false);

    HAL_timer_enable_interrupt(timer_num);

    // Start the timer.//启动计时器。
    timer_instance[timer_num]->resume(); // First call to resume() MUST follow the attachInterrupt()//对resume（）的第一次调用必须在attachInterrupt（）之后

    // This is fixed in Arduino_Core_STM32 1.8.//这已在Arduino_Core_STM32 1.8中修复。
    // These calls can be removed and replaced with//这些呼叫可以删除并替换为
    // timer_instance[timer_num]->setInterruptPriority//timer\u实例[timer\u num]->setInterruptPriority
    switch (timer_num) {
      case STEP_TIMER_NUM:
        timer_instance[timer_num]->setInterruptPriority(STEP_TIMER_IRQ_PRIO, 0);
        break;
      case TEMP_TIMER_NUM:
        timer_instance[timer_num]->setInterruptPriority(TEMP_TIMER_IRQ_PRIO, 0);
        break;
    }
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !timer_instance[timer_num]->hasInterrupt()) {
    switch (timer_num) {
      case STEP_TIMER_NUM:
        timer_instance[timer_num]->attachInterrupt(Step_Handler);
        break;
      case TEMP_TIMER_NUM:
        timer_instance[timer_num]->attachInterrupt(Temp_Handler);
        break;
    }
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num)) timer_instance[timer_num]->detachInterrupt();
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) && timer_instance[timer_num]->hasInterrupt();
}

void SetTimerInterruptPriorities() {
  TERN_(HAS_TMC_SW_SERIAL, SoftwareSerial::setInterruptPriority(SWSERIAL_TIMER_IRQ_PRIO, 0));
  TERN_(HAS_SERVOS, libServo::setInterruptPriority(SERVO_TIMER_IRQ_PRIO, 0));
}

// ------------------------// ------------------------
// Detect timer conflicts//检测计时器冲突
// ------------------------// ------------------------

// This list serves two purposes. Firstly, it facilitates build-time mapping between//这份清单有两个目的。首先，它有助于在
// variant-defined timer names (such as TIM1) and timer numbers. It also replicates//变量定义的计时器名称（如TIM1）和计时器编号。它也可以复制
// the order of timers used in the framework's SoftwareSerial.cpp. The first timer in//框架的SoftwareSerial.cpp中使用的计时器顺序。世界上第一个计时器
// this list will be automatically used by SoftwareSerial if it is not already defined//如果尚未定义此列表，则SoftwareSerial将自动使用此列表
// in the board's variant or compiler options.//在板的变量或编译器选项中。
static constexpr struct {uintptr_t base_address; int timer_number;} stm32_timer_map[] = {
  #ifdef TIM18_BASE
    { uintptr_t(TIM18), 18 },
  #endif
  #ifdef TIM7_BASE
    { uintptr_t(TIM7),   7 },
  #endif
  #ifdef TIM6_BASE
    { uintptr_t(TIM6),   6 },
  #endif
  #ifdef TIM22_BASE
    { uintptr_t(TIM22), 22 },
  #endif
  #ifdef TIM21_BASE
    { uintptr_t(TIM21), 21 },
  #endif
  #ifdef TIM17_BASE
    { uintptr_t(TIM17), 17 },
  #endif
  #ifdef TIM16_BASE
    { uintptr_t(TIM16), 16 },
  #endif
  #ifdef TIM15_BASE
    { uintptr_t(TIM15), 15 },
  #endif
  #ifdef TIM14_BASE
    { uintptr_t(TIM14), 14 },
  #endif
  #ifdef TIM13_BASE
    { uintptr_t(TIM13), 13 },
  #endif
  #ifdef TIM11_BASE
    { uintptr_t(TIM11), 11 },
  #endif
  #ifdef TIM10_BASE
    { uintptr_t(TIM10), 10 },
  #endif
  #ifdef TIM12_BASE
    { uintptr_t(TIM12), 12 },
  #endif
  #ifdef TIM19_BASE
    { uintptr_t(TIM19), 19 },
  #endif
  #ifdef TIM9_BASE
    { uintptr_t(TIM9),   9 },
  #endif
  #ifdef TIM5_BASE
    { uintptr_t(TIM5),   5 },
  #endif
  #ifdef TIM4_BASE
    { uintptr_t(TIM4),   4 },
  #endif
  #ifdef TIM3_BASE
    { uintptr_t(TIM3),   3 },
  #endif
  #ifdef TIM2_BASE
    { uintptr_t(TIM2),   2 },
  #endif
  #ifdef TIM20_BASE
    { uintptr_t(TIM20), 20 },
  #endif
  #ifdef TIM8_BASE
    { uintptr_t(TIM8),   8 },
  #endif
  #ifdef TIM1_BASE
    { uintptr_t(TIM1),   1 }
  #endif
};

// Convert from a timer base address to its integer timer number.//将计时器基址转换为其整数计时器编号。
static constexpr int get_timer_num_from_base_address(uintptr_t base_address) {
  for (const auto &timer : stm32_timer_map)
    if (timer.base_address == base_address) return timer.timer_number;
  return 0;
}

// The platform's SoftwareSerial.cpp will use the first timer from stm32_timer_map.//平台的SoftwareSerial.cpp将使用stm32_timer_map中的第一个计时器。
#if HAS_TMC_SW_SERIAL && !defined(TIMER_SERIAL)
  #define  TIMER_SERIAL (stm32_timer_map[0].base_address)
#endif

// constexpr doesn't like using the base address pointers that timers evaluate to.//constexpr不喜欢使用计时器计算的基址指针。
// We can get away with casting them to uintptr_t, if we do so inside an array.//如果我们在一个数组中这样做的话，我们可以将它们施放到uintptru\t。
// GCC will not currently do it directly to a uintptr_t.//GCC目前不会直接向uintptr\t执行此操作。
IF_ENABLED(HAS_TMC_SW_SERIAL, static constexpr uintptr_t timer_serial[] = {uintptr_t(TIMER_SERIAL)});
IF_ENABLED(SPEAKER,           static constexpr uintptr_t timer_tone[]   = {uintptr_t(TIMER_TONE)});
IF_ENABLED(HAS_SERVOS,        static constexpr uintptr_t timer_servo[]  = {uintptr_t(TIMER_SERVO)});

enum TimerPurpose { TP_SERIAL, TP_TONE, TP_SERVO, TP_STEP, TP_TEMP };

// List of timers, to enable checking for conflicts.//计时器列表，用于检查冲突。
// Includes the purpose of each timer to ease debugging when evaluating at build-time.//包括每个计时器的用途，以便于在生成时进行评估时进行调试。
// This cannot yet account for timers used for PWM output, such as for fans.//这还不能解释用于PWM输出（如风扇）的定时器。
static constexpr struct { TimerPurpose p; int t; } timers_in_use[] = {
  #if HAS_TMC_SW_SERIAL
    {TP_SERIAL, get_timer_num_from_base_address(timer_serial[0])},  // Set in variant.h, or as a define in platformio.h if not present in variant.h//在variant.h中设置，如果variant.h中不存在，则在platformio.h中定义
  #endif
  #if ENABLED(SPEAKER)
    {TP_TONE, get_timer_num_from_base_address(timer_tone[0])},    // Set in variant.h, or as a define in platformio.h if not present in variant.h//在variant.h中设置，如果variant.h中不存在，则在platformio.h中定义
  #endif
  #if HAS_SERVOS
    {TP_SERVO, get_timer_num_from_base_address(timer_servo[0])},   // Set in variant.h, or as a define in platformio.h if not present in variant.h//在variant.h中设置，如果variant.h中不存在，则在platformio.h中定义
  #endif
  {TP_STEP, STEP_TIMER},
  {TP_TEMP, TEMP_TIMER},
};

static constexpr bool verify_no_timer_conflicts() {
  LOOP_L_N(i, COUNT(timers_in_use))
    LOOP_S_L_N(j, i + 1, COUNT(timers_in_use))
      if (timers_in_use[i].t == timers_in_use[j].t) return false;
  return true;
}

// If this assertion fails at compile time, review the timers_in_use array.//如果此断言在编译时失败，请查看\u use数组中的计时器\u。
// If default_envs is defined properly in platformio.ini, VS Code can evaluate the array//如果platformio.ini中正确定义了默认的_envs，VS代码可以计算数组
// when hovering over it, making it easy to identify the conflicting timers.//当鼠标悬停在它上面时，可以很容易地识别冲突的计时器。
static_assert(verify_no_timer_conflicts(), "One or more timer conflict detected. Examine \"timers_in_use\" to help identify conflict.");

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC//ARDUINO_ARCH_STM32&&！STM32通用
