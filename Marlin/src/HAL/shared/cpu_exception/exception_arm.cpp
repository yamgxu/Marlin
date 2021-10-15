/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Cyril Russo
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

/***************************************************************************
 * ARM CPU Exception handler
 ***************************************************************************/

#if defined(__arm__) || defined(__thumb__)


/*
  On ARM CPUs exception handling is quite powerful.

  By default, upon a crash, the CPU enters the handlers that have a higher priority than any other interrupts,
  so, in effect, no (real) interrupt can "interrupt" the handler (it's acting like if interrupts were disabled).

  If the handler is not called as re-entrant (that is, if the crash is not happening inside an interrupt or an handler),
  then it'll patch the return address to a dumping function (resume_from_fault) and save the crash state.
  The CPU will exit the handler and, as such, re-allow the other interrupts, and jump to the dumping function.
  In this function, the usual serial port (USB / HW) will be used to dump the crash (no special configuration required).

  The only case where it requires hardware UART is when it's crashing in an interrupt or a crash handler.
  In that case, instead of returning to the resume_from_fault function (and thus, re-enabling interrupts),
  it jumps to this function directly (so with interrupts disabled), after changing the behavior of the serial output
  wrapper to use the HW uart (and in effect, calling MinSerial::init which triggers a warning if you are using
  a USB serial port).

  In the case you have a USB serial port, this part will be disabled, and only that part (so that's the reason for
  the warning).
  This means that you can't have a crash report if the crash happens in an interrupt or an handler if you are using
  a USB serial port since it's physically impossible.
  You will get a crash report in all other cases.
*/

#include "exception_hook.h"
#include "../backtrace/backtrace.h"
#include "../HAL_MinSerial.h"

#define HW_REG(X)  (*((volatile unsigned long *)(X)))

// Default function use the CPU VTOR register to get the vector table.//默认函数使用CPU VTOR寄存器获取向量表。
// Accessing the CPU VTOR register is done in assembly since it's the only way that's common to all current tool//访问CPU VTOR寄存器是在汇编中完成的，因为这是所有当前工具通用的唯一方法
unsigned long get_vtor() { return HW_REG(0xE000ED08); } // Even if it looks like an error, it is not an error//即使它看起来像一个错误，它也不是一个错误
void * hook_get_hardfault_vector_address(unsigned vtor)  { return (void*)(vtor + 0x03); }
void * hook_get_memfault_vector_address(unsigned vtor)   { return (void*)(vtor + 0x04); }
void * hook_get_busfault_vector_address(unsigned vtor)   { return (void*)(vtor + 0x05); }
void * hook_get_usagefault_vector_address(unsigned vtor) { return (void*)(vtor + 0x06); }
void * hook_get_reserved_vector_address(unsigned vtor)   { return (void*)(vtor + 0x07); }

// Common exception frame for ARM, should work for all ARM CPU//ARM的通用异常框架，应适用于所有ARM CPU
// Described here (modified for convenience): https://interrupt.memfault.com/blog/cortex-m-fault-debug//此处描述（为方便起见进行了修改）：https://interrupt.memfault.com/blog/cortex-m-fault-debug
struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
};

struct __attribute__((packed)) ContextSavedFrame {
  uint32_t R0;
  uint32_t R1;
  uint32_t R2;
  uint32_t R3;
  uint32_t R12;
  uint32_t LR;
  uint32_t PC;
  uint32_t XPSR;

  uint32_t CFSR;
  uint32_t HFSR;
  uint32_t DFSR;
  uint32_t AFSR;
  uint32_t MMAR;
  uint32_t BFAR;

  uint32_t ESP;
  uint32_t ELR;
};

#if DISABLED(STM32F0xx)
  extern "C"
  __attribute__((naked)) void CommonHandler_ASM() {
    __asm__ __volatile__ (
      // Bit 2 of LR tells which stack pointer to use (either main or process, only main should be used anyway)//LR的第2位指示要使用哪个堆栈指针（无论是main还是process，都只应使用main）
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      // Save the LR in use when being interrupted//中断时保存正在使用的LR
      "mov r1, lr\n"
      // Get the exception number from the ICSR register//从ICSR寄存器中获取异常编号
      "ldr r2, =0xE000ED00\n"
      "ldr r2, [r2, #4]\n"
      "b CommonHandler_C\n"
    );
  }
#else // Cortex M0 does not support conditional mov and testing with a constant, so let's have a specific handler for it//Cortex M0不支持条件mov和常数测试，所以让我们为它指定一个特定的处理程序
  extern "C"
  __attribute__((naked)) void CommonHandler_ASM() {
    __asm__ __volatile__ (
      ".syntax unified\n"
      // Save the LR in use when being interrupted//中断时保存正在使用的LR
      "mov  r1, lr\n"
      // Get the exception number from the ICSR register//从ICSR寄存器中获取异常编号
      "ldr  r2, =0xE000ED00\n"
      "ldr  r2, [r2, #4]\n"
      "movs r0, #4\n"
      "tst  r1, r0\n"
      "beq _MSP\n"
      "mrs  r0, psp\n"
      "b CommonHandler_C\n"
      "_MSP:\n"
      "mrs  r0, msp\n"
      "b CommonHandler_C\n"
    );
  }

  #if DISABLED(DYNAMIC_VECTORTABLE) // Cortex M0 requires the handler's address to be within 32kB to the actual function to be able to branch to it//Cortex M0要求处理程序的地址与实际函数的距离在32kB以内，以便能够分支到它
    extern "C" {
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __exc_hardfault();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __exc_busfault();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __exc_usagefault();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __exc_memmanage();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __exc_nmi();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __stm32reservedexception7();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __stm32reservedexception8();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __stm32reservedexception9();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __stm32reservedexception10();
      void __attribute__((naked, alias("CommonHandler_ASM"), nothrow)) __stm32reservedexception13();
    }
    //TODO When going off from libmaple, you'll need to replace those by the one from STM32/HAL_MinSerial.cpp//当从libmaple出发时，您需要用STM32/HAL_MinSerial.cpp中的内容替换这些内容
  #endif
#endif

// Must be a macro to avoid creating a function frame//必须是宏才能避免创建功能框架
#define HALT_IF_DEBUGGING()             \
  do {                                  \
    if (HW_REG(0xE000EDF0) & _BV(0)) {  \
      __asm("bkpt 1");                  \
    }                                   \
} while (0)

// Resume from a fault (if possible) so we can still use interrupt based code for serial output//从故障恢复（如果可能的话），这样我们仍然可以使用基于中断的代码进行串行输出
// In that case, we will not jump back to the faulty code, but instead to a dumping code and then a//在这种情况下，我们不会跳回错误代码，而是跳回转储代码，然后跳回
// basic loop with watchdog calling or manual resetting//带看门狗呼叫或手动复位的基本回路
static ContextSavedFrame savedFrame;
static uint8_t           lastCause;
bool resume_from_fault() {
  static const char* causestr[] = { "Thread", "Rsvd", "NMI", "Hard", "Mem", "Bus", "Usage", "7", "8", "9", "10", "SVC", "Dbg", "13", "PendSV", "SysTk", "IRQ" };
  // Reinit the serial link (might only work if implemented in each of your boards)//重新连接串行链接（只有在每个电路板中实现时才可能工作）
  MinSerial::init();

  MinSerial::TX("\n\n## Software Fault detected ##\n");
  MinSerial::TX("Cause: "); MinSerial::TX(causestr[min(lastCause, (uint8_t)16)]); MinSerial::TX('\n');

  MinSerial::TX("R0   : "); MinSerial::TXHex(savedFrame.R0);   MinSerial::TX('\n');
  MinSerial::TX("R1   : "); MinSerial::TXHex(savedFrame.R1);   MinSerial::TX('\n');
  MinSerial::TX("R2   : "); MinSerial::TXHex(savedFrame.R2);   MinSerial::TX('\n');
  MinSerial::TX("R3   : "); MinSerial::TXHex(savedFrame.R3);   MinSerial::TX('\n');
  MinSerial::TX("R12  : "); MinSerial::TXHex(savedFrame.R12);  MinSerial::TX('\n');
  MinSerial::TX("LR   : "); MinSerial::TXHex(savedFrame.LR);   MinSerial::TX('\n');
  MinSerial::TX("PC   : "); MinSerial::TXHex(savedFrame.PC);   MinSerial::TX('\n');
  MinSerial::TX("PSR  : "); MinSerial::TXHex(savedFrame.XPSR); MinSerial::TX('\n');

  // Configurable Fault Status Register//可配置故障状态寄存器
  // Consists of MMSR, BFSR and UFSR//由MMSR、BFSR和UFSR组成
  MinSerial::TX("CFSR : "); MinSerial::TXHex(savedFrame.CFSR); MinSerial::TX('\n');

  // Hard Fault Status Register//硬故障状态寄存器
  MinSerial::TX("HFSR : "); MinSerial::TXHex(savedFrame.HFSR); MinSerial::TX('\n');

  // Debug Fault Status Register//调试故障状态寄存器
  MinSerial::TX("DFSR : "); MinSerial::TXHex(savedFrame.DFSR); MinSerial::TX('\n');

  // Auxiliary Fault Status Register//辅助故障状态寄存器
  MinSerial::TX("AFSR : "); MinSerial::TXHex(savedFrame.AFSR); MinSerial::TX('\n');

  // Read the Fault Address Registers. These may not contain valid values.//读取故障地址寄存器。这些可能不包含有效值。
  // Check BFARVALID/MMARVALID to see if they are valid values//检查BFARVALID/MMARVALID以查看它们是否为有效值
  // MemManage Fault Address Register//内存管理故障地址寄存器
  MinSerial::TX("MMAR : "); MinSerial::TXHex(savedFrame.MMAR); MinSerial::TX('\n');

  // Bus Fault Address Register//总线故障地址寄存器
  MinSerial::TX("BFAR : "); MinSerial::TXHex(savedFrame.BFAR); MinSerial::TX('\n');

  MinSerial::TX("ExcLR: "); MinSerial::TXHex(savedFrame.ELR); MinSerial::TX('\n');
  MinSerial::TX("ExcSP: "); MinSerial::TXHex(savedFrame.ESP); MinSerial::TX('\n');

  // The stack pointer is pushed by 8 words upon entering an exception, so we need to revert this//在输入异常时，堆栈指针会被推8个字，因此我们需要将其还原
  backtrace_ex(savedFrame.ESP + 8*4, savedFrame.LR, savedFrame.PC);

  // Call the last resort function here//在这里调用最后一个函数
  hook_last_resort_func();

  const uint32_t start = millis(), end = start + 100; // 100ms should be enough//100毫秒就足够了
  // We need to wait for the serial buffers to be output but we don't know for how long//我们需要等待串行缓冲区输出，但不知道要等待多长时间
  // So we'll just need to refresh the watchdog for a while and then stop for the system to reboot//因此，我们只需要刷新一段时间的看门狗，然后停止系统重新启动
  uint32_t last = start;
  while (PENDING(last, end)) {
    watchdog_refresh();
    while (millis() == last) { /* nada */ }
    last = millis();
    MinSerial::TX('.');
  }

  // Reset now by reinstantiating the bootloader's vector table//现在通过重新实例化引导加载程序的向量表来重置
  HW_REG(0xE000ED08) = 0;
  // Restart watchdog//重新启动看门狗
  #if DISABLED(USE_WATCHDOG)
    // No watchdog, let's perform ARMv7 reset instead by writing to AIRCR register with VECTKEY set to SYSRESETREQ//没有看门狗，让我们通过将VECTKEY设置为SYSRESETREQ写入空勤寄存器来执行ARMv7重置
    HW_REG(0xE000ED0C) = (HW_REG(0xE000ED0C) & 0x0000FFFF) | 0x05FA0004;
  #endif

  while(1) {} // Bad luck, nothing worked//运气不好，什么都没用
}

// Make sure the compiler does not optimize the frame argument away//确保编译器没有优化帧参数
extern "C"
__attribute__((optimize("O0")))
void CommonHandler_C(ContextStateFrame * frame, unsigned long lr, unsigned long cause) {

  // If you are using it'll stop here//如果你正在使用它，它将停止在这里
  HALT_IF_DEBUGGING();

  // Save the state to backtrace later on (don't call memcpy here since the stack can be corrupted)//将状态保存到稍后的回溯（不要在此处调用memcpy，因为堆栈可能已损坏）
  savedFrame.R0  = frame->r0;
  savedFrame.R1  = frame->r1;
  savedFrame.R2  = frame->r2;
  savedFrame.R3  = frame->r3;
  savedFrame.R12 = frame->r12;
  savedFrame.LR  = frame->lr;
  savedFrame.PC  = frame->pc;
  savedFrame.XPSR= frame->xpsr;
  lastCause = cause & 0x1FF;

  volatile uint32_t &CFSR = HW_REG(0xE000ED28);
  savedFrame.CFSR = CFSR;
  savedFrame.HFSR = HW_REG(0xE000ED2C);
  savedFrame.DFSR = HW_REG(0xE000ED30);
  savedFrame.AFSR = HW_REG(0xE000ED3C);
  savedFrame.MMAR = HW_REG(0xE000ED34);
  savedFrame.BFAR = HW_REG(0xE000ED38);
  savedFrame.ESP  = (unsigned long)frame; // Even on return, this should not be overwritten by the CPU//即使在返回时，这也不应该被CPU覆盖
  savedFrame.ELR  = lr;

  // First check if we can resume from this exception to our own handler safely//首先检查我们是否可以安全地从这个异常恢复到我们自己的处理程序
  // If we can, then we don't need to disable interrupts and the usual serial code//如果可以，那么我们就不需要禁用中断和通常的串行代码
  // can be used//可以使用

  //const uint32_t non_usage_fault_mask = 0x0000FFFF;//常数32非使用故障掩码=0x0000FFFF；
  //const bool non_usage_fault_occurred = (CFSR & non_usage_fault_mask) != 0;//const bool non_usage_fault_occurrent=（CFSR&non_usage_fault_mask）！=0;
  // the bottom 8 bits of the xpsr hold the exception number of the//xpsr的底部8位保存
  // executing exception or 0 if the processor is in Thread mode//如果处理器处于线程模式，则执行异常或0
  const bool faulted_from_exception = ((frame->xpsr & 0xFF) != 0);
  if (!faulted_from_exception) { // Not sure about the non_usage_fault, we want to try anyway, don't we ? && !non_usage_fault_occurred)//不确定非使用故障，我们还是想试试，不是吗？&&！非使用故障（发生故障）
    // Try to resume to our handler here//试着回到我们的处理程序这里
    CFSR |= CFSR; // The ARM programmer manual says you must write to 1 all fault bits to clear them so this instruction is correct//ARM编程器手册说，您必须将所有故障位写入1以清除它们，因此此指令是正确的
    // The frame will not be valid when returning anymore, let's clean it//帧在返回时将不再有效，让我们清理它
    savedFrame.CFSR = 0;

    frame->pc = (uint32_t)resume_from_fault; // Patch where to return to//修补返回的位置
    frame->lr = 0xDEADBEEF;  // If our handler returns (it shouldn't), let's make it trigger an exception immediately//如果我们的处理程序返回（它不应该返回），让我们让它立即触发异常
    frame->xpsr = _BV(24);   // Need to clean the PSR register to thumb II only//仅需清洁thumb II的PSR寄存器
    MinSerial::force_using_default_output = true;
    return; // The CPU will resume in our handler hopefully, and we'll try to use default serial output//CPU将在我们的处理程序中恢复，我们将尝试使用默认的串行输出
  }

  // Sorry, we need to emergency code here since the fault is too dangerous to recover from//抱歉，我们需要在此处输入紧急代码，因为故障太危险，无法恢复
  MinSerial::force_using_default_output = false;
  resume_from_fault();
}

void hook_cpu_exceptions() {
  #if ENABLED(DYNAMIC_VECTORTABLE)
    // On ARM 32bits CPU, the vector table is like this://在ARM 32位CPU上，向量表如下所示：
    // 0x0C => Hardfault//0x0C=>硬故障
    // 0x10 => MemFault//0x10=>MemFault
    // 0x14 => BusFault//0x14=>总线故障
    // 0x18 => UsageFault//0x18=>UsageFault

    // Unfortunately, it's usually run from flash, and we can't write to flash here directly to hook our instruction//不幸的是，它通常是从flash运行的，我们不能在这里直接写入flash来钩住我们的指令
    // We could set an hardware breakpoint, and hook on the fly when it's being called, but this//我们可以设置一个硬件断点，并在调用时动态挂接，但是
    // is hard to get right and would probably break debugger when attached//很难获得正确的结果，可能会在连接时破坏调试器

    // So instead, we'll allocate a new vector table filled with the previous value except//因此，我们将分配一个新的向量表，其中填充了以前的值，除了
    // for the fault we are interested in.//对于我们感兴趣的错误。
    // Now, comes the issue to figure out what is the current vector table size//现在，问题来了，弄清楚当前向量表的大小是多少
    // There is nothing telling us what is the vector table as it's per-cpu vendor specific.//没有任何东西可以告诉我们向量表是什么，因为它是针对每个cpu供应商的。
    // BUT: we are being called at the end of the setup, so we assume the setup is done//但是：我们在安装结束时被调用，所以我们假设安装已经完成
    // Thus, we can read the current vector table until we find an address that's not in flash, and it would mark the//因此，我们可以读取当前的向量表，直到我们找到一个不在flash中的地址，它会标记
    // end of the vector table (skipping the fist entry obviously)//向量表的末尾（明显跳过第一个条目）
    // The position of the program in flash is expected to be at 0x08xxx xxxx on all known platform for ARM and the//在所有已知的ARM平台上，闪存中程序的位置预计为0x08xxx xxxx
    // flash size is available via register 0x1FFFF7E0 on STM32 family, but it's not the case for all ARM boards//闪存大小可通过STM32系列上的寄存器0x1FFF7E0获得，但并非适用于所有ARM板
    // (accessing this register might trigger a fault if it's not implemented).//（如果未实现，访问此寄存器可能会触发故障）。

    // So we'll simply mask the top 8 bits of the first handler as an hint of being in the flash or not -that's poor and will//因此，我们将简单地屏蔽第一个处理程序的前8位，作为是否在闪存中的提示-这很糟糕，而且很有可能
    // probably break if the flash happens to be more than 128MB, but in this case, we are not magician, we need help from outside.//如果闪光灯刚好超过128MB，可能会中断，但在这种情况下，我们不是魔术师，我们需要外界的帮助。

    unsigned long *vecAddr = (unsigned long*)get_vtor();
    SERIAL_ECHOPGM("Vector table addr: ");
    SERIAL_PRINTLN(get_vtor(), HEX);

    #ifdef VECTOR_TABLE_SIZE
      uint32_t vec_size = VECTOR_TABLE_SIZE;
      alignas(128) static unsigned long vectable[VECTOR_TABLE_SIZE] ;
    #else
      #ifndef IS_IN_FLASH
        #define IS_IN_FLASH(X) (((unsigned long)X & 0xFF000000) == 0x08000000)
      #endif

      // When searching for the end of the vector table, this acts as a limit not to overcome//在搜索向量表的结尾时，这是一个无法克服的限制
      #ifndef VECTOR_TABLE_SENTINEL
        #define VECTOR_TABLE_SENTINEL 80
      #endif

      // Find the vector table size//查找向量表大小
      uint32_t vec_size = 1;
      while (IS_IN_FLASH(vecAddr[vec_size]) && vec_size < VECTOR_TABLE_SENTINEL)
        vec_size++;

      // We failed to find a valid vector table size, let's abort hooking up//我们找不到有效的向量表大小，让我们中止连接
      if (vec_size == VECTOR_TABLE_SENTINEL) return;
      // Poor method that's wasting RAM here, but allocating with malloc and alignment would be worst//糟糕的方法在这里浪费RAM，但使用malloc和对齐进行分配将是最糟糕的
      // 128 bytes alignement is required for writing the VTOR register//写入VTOR寄存器需要128字节对齐
      alignas(128) static unsigned long vectable[VECTOR_TABLE_SENTINEL];

      SERIAL_ECHOPGM("Detected vector table size: ");
      SERIAL_PRINTLN(vec_size, HEX);
    #endif

    uint32_t defaultFaultHandler = vecAddr[(unsigned)7];
    // Copy the current vector table into the new table//将当前向量表复制到新表中
    for (uint32_t i = 0; i < vec_size; i++) {
      vectable[i] = vecAddr[i];
      // Replace all default handler by our own handler//用我们自己的处理程序替换所有默认处理程序
      if (vectable[i] == defaultFaultHandler)
        vectable[i] = (unsigned long)&CommonHandler_ASM;
    }

    // Let's hook now with our functions//现在让我们用我们的函数挂钩
    vectable[(unsigned long)hook_get_hardfault_vector_address(0)]  = (unsigned long)&CommonHandler_ASM;
    vectable[(unsigned long)hook_get_memfault_vector_address(0)]   = (unsigned long)&CommonHandler_ASM;
    vectable[(unsigned long)hook_get_busfault_vector_address(0)]   = (unsigned long)&CommonHandler_ASM;
    vectable[(unsigned long)hook_get_usagefault_vector_address(0)] = (unsigned long)&CommonHandler_ASM;

    // Finally swap with our own vector table//最后用我们自己的向量表交换
    // This is supposed to be atomic, but let's do that with interrupt disabled//这应该是原子的，但让我们在禁用中断的情况下这样做

    HW_REG(0xE000ED08) = (unsigned long)vectable | _BV32(29); // 29th bit is for telling the CPU the table is now in SRAM (should be present already)//第29位用于告诉CPU表现在在SRAM中（应该已经存在）

    SERIAL_ECHOLNPGM("Installed fault handlers");
  #endif
}

#endif // __arm__ || __thumb__//手臂拇指__
