/** translatione by yx */
/***************************************************************************
 * ARM Stack Unwinder, Michael.McTernan.2001@cs.bris.ac.uk
 * Updated, adapted and several bug fixes on 2018 by Eduardo José Tagle
 *
 * This program is PUBLIC DOMAIN.
 * This means that there is no copyright and anyone is able to take a copy
 * for free and use it as they wish, with or without modifications, and in
 * any context, commercially or otherwise. The only limitation is that I
 * don't guarantee that the software is fit for any purpose or accept any
 * liability for its use or misuse - this software is without warranty.
 ***************************************************************************
 * File Description: Utility functions to access memory
 **************************************************************************/

#if defined(__arm__) || defined(__thumb__)

#include "unwmemaccess.h"
#include "../../../inc/MarlinConfig.h"

/* Validate address */

#ifdef ARDUINO_ARCH_SAM

  // For DUE, valid address ranges are//到期时，有效地址范围为
  //  SRAM  (0x20070000 - 0x20088000) (96kb)//SRAM（0x20070000-0x20088000）（96kb）
  //  FLASH (0x00080000 - 0x00100000) (512kb)//闪存（0x00080000-0x00100000）（512kb）
  ////
  #define START_SRAM_ADDR   0x20070000
  #define END_SRAM_ADDR     0x20088000
  #define START_FLASH_ADDR  0x00080000
  #define END_FLASH_ADDR    0x00100000

#elif defined(TARGET_LPC1768)

  // For LPC1769://对于LPC1769：
  //  SRAM  (0x10000000 - 0x10008000) (32kb)//SRAM（0x10000000-0x10008000）（32kb）
  //  FLASH (0x00000000 - 0x00080000) (512kb)//闪存（0x00000000-0x00080000）（512kb）
  ////
  #define START_SRAM_ADDR   0x10000000
  #define END_SRAM_ADDR     0x10008000
  #define START_FLASH_ADDR  0x00000000
  #define END_FLASH_ADDR    0x00080000

#elif defined(__STM32F1__) || defined(STM32F1xx) || defined(STM32F0xx)

  // For STM32F103ZET6/STM32F103VET6/STM32F0xx//对于STM32F103ZET6/STM32F103VET6/STM32F0xx
  //  SRAM  (0x20000000 - 0x20010000) (64kb)//SRAM（0x20000000-0x20010000）（64kb）
  //  FLASH (0x08000000 - 0x08080000) (512kb)//闪存（0x08000000-0x08080000）（512kb）
  ////
  #define START_SRAM_ADDR   0x20000000
  #define END_SRAM_ADDR     0x20010000
  #define START_FLASH_ADDR  0x08000000
  #define END_FLASH_ADDR    0x08080000

#elif defined(STM32F4) || defined(STM32F4xx)

  // For STM32F407VET//对于STM32F407VET
  //  SRAM  (0x20000000 - 0x20030000) (192kb)//SRAM（0x20000000-0x20030000）（192kb）
  //  FLASH (0x08000000 - 0x08080000) (512kb)//闪存（0x08000000-0x08080000）（512kb）
  ////
  #define START_SRAM_ADDR   0x20000000
  #define END_SRAM_ADDR     0x20030000
  #define START_FLASH_ADDR  0x08000000
  #define END_FLASH_ADDR    0x08080000

#elif MB(REMRAM_V1, NUCLEO_F767ZI)

  // For STM32F765VI in RemRam v1//对于RemRam v1中的STM32F765VI
  //  SRAM  (0x20000000 - 0x20080000) (512kb)//SRAM（0x20000000-0x20080000）（512kb）
  //  FLASH (0x08000000 - 0x08200000) (2048kb)//闪存（0x08000000-0x08200000）（2048kb）
  ////
  #define START_SRAM_ADDR   0x20000000
  #define END_SRAM_ADDR     0x20080000
  #define START_FLASH_ADDR  0x08000000
  #define END_FLASH_ADDR    0x08200000

#elif defined(__MK20DX256__)

  // For MK20DX256 in TEENSY 3.1 or TEENSY 3.2//适用于TEENSY 3.1或TEENSY 3.2中的MK20DX256
  //  SRAM  (0x1FFF8000 - 0x20008000) (64kb)//SRAM（0x1FF8000-0x20008000）（64kb）
  //  FLASH (0x00000000 - 0x00040000) (256kb)//闪存（0x00000000-0x00040000）（256kb）
  ////
  #define START_SRAM_ADDR   0x1FFF8000
  #define END_SRAM_ADDR     0x20008000
  #define START_FLASH_ADDR  0x00000000
  #define END_FLASH_ADDR    0x00040000

#elif defined(__MK64FX512__)

  // For MK64FX512 in TEENSY 3.5//适用于TEENSY 3.5中的MK64FX512
  //  SRAM  (0x1FFF0000 - 0x20020000) (192kb)//SRAM（0x1FF0000-0x20020000）（192kb）
  //  FLASH (0x00000000 - 0x00080000) (512kb)//闪存（0x00000000-0x00080000）（512kb）
  ////
  #define START_SRAM_ADDR   0x1FFF0000
  #define END_SRAM_ADDR     0x20020000
  #define START_FLASH_ADDR  0x00000000
  #define END_FLASH_ADDR    0x00080000

#elif defined(__MK66FX1M0__)

  // For MK66FX1M0 in TEENSY 3.6//适用于MK66FX1M0英寸TEENSY 3.6
  //  SRAM  (0x1FFF0000 - 0x20030000) (256kb)//SRAM（0x1FF0000-0x20030000）（256kb）
  //  FLASH (0x00000000 - 0x00140000) (1.25Mb)//闪存（0x00000000-0x00140000）（1.25Mb）
  ////
  #define START_SRAM_ADDR   0x1FFF0000
  #define END_SRAM_ADDR     0x20030000
  #define START_FLASH_ADDR  0x00000000
  #define END_FLASH_ADDR    0x00140000

#elif defined(__IMXRT1062__)

  // For IMXRT1062 in TEENSY 4.0/4/1//适用于TEENSY 4.0/4/1中的IMXRT1062
  //  ITCM (rwx):  ORIGIN = 0x00000000, LENGTH = 512K//ITCM（rwx）：原点=0x00000000，长度=512K
  //  DTCM (rwx):  ORIGIN = 0x20000000, LENGTH = 512K//DTCM（rwx）：原点=0x20000000，长度=512K
  //  RAM (rwx):   ORIGIN = 0x20200000, LENGTH = 512K//RAM（rwx）：原点=0x20200000，长度=512K
  //  FLASH (rwx): ORIGIN = 0x60000000, LENGTH = 1984K//闪光（rwx）：原点=0x60000000，长度=1984K
  ////
  #define START_SRAM_ADDR   0x00000000
  #define END_SRAM_ADDR     0x20280000
  #define START_FLASH_ADDR  0x60000000
  #define END_FLASH_ADDR    0x601F0000

#elif defined(__SAMD51P20A__)

  // For SAMD51x20, valid address ranges are//对于SAMD51x20，有效地址范围为
  //  SRAM  (0x20000000 - 0x20040000) (256kb)//SRAM（0x20000000-0x20040000）（256kb）
  //  FLASH (0x00000000 - 0x00100000) (1024kb)//闪存（0x00000000-0x00100000）（1024kb）
  ////
  #define START_SRAM_ADDR   0x20000000
  #define END_SRAM_ADDR     0x20040000
  #define START_FLASH_ADDR  0x00000000
  #define END_FLASH_ADDR    0x00100000

#else
  // Generic ARM code, that's testing if an access to the given address would cause a fault or not//通用ARM代码，即测试对给定地址的访问是否会导致故障
  // It can't guarantee an address is in RAM or Flash only, but we usually don't care//它不能保证地址只在RAM或闪存中，但我们通常不在乎

  #define NVIC_FAULT_STAT         0xE000ED28  // Configurable Fault Status Reg.//可配置故障状态寄存器。
  #define NVIC_CFG_CTRL           0xE000ED14  // Configuration Control Register//配置控制寄存器
  #define NVIC_FAULT_STAT_BFARV   0x00008000  // BFAR is valid//BFAR是有效的
  #define NVIC_CFG_CTRL_BFHFNMIGN 0x00000100  // Ignore bus fault in NMI/fault//忽略NMI中的总线故障/故障
  #define HW_REG(X)  (*((volatile unsigned long *)(X)))

  static bool validate_addr(uint32_t read_address) {
    bool     works = true;
    uint32_t intDisabled = 0;
    // Read current interrupt state//读取当前中断状态
    __asm__ __volatile__ ("MRS %[result], PRIMASK\n\t" : [result]"=r"(intDisabled) :: ); // 0 is int enabled, 1 for disabled//0已启用整型，1已禁用

    // Clear bus fault indicator first (write 1 to clear)//首先清除总线故障指示灯（写入1清除）
    HW_REG(NVIC_FAULT_STAT) |= NVIC_FAULT_STAT_BFARV;
    // Ignore bus fault interrupt//忽略总线故障中断
    HW_REG(NVIC_CFG_CTRL) |= NVIC_CFG_CTRL_BFHFNMIGN;
    // Disable interrupts if not disabled previously//如果以前未禁用，则禁用中断
    if (!intDisabled) __asm__ __volatile__ ("CPSID f");
    // Probe address//探测地址
    *(volatile uint32_t*)read_address;
    // Check if a fault happened//检查是否发生故障
    if ((HW_REG(NVIC_FAULT_STAT) & NVIC_FAULT_STAT_BFARV) != 0)
      works = false;
    // Enable interrupts again if previously disabled//如果以前禁用，请再次启用中断
    if (!intDisabled) __asm__ __volatile__ ("CPSIE f");
    // Enable fault interrupt flag//启用故障中断标志
    HW_REG(NVIC_CFG_CTRL) &= ~NVIC_CFG_CTRL_BFHFNMIGN;

    return works;
  }

#endif

#ifdef START_SRAM_ADDR
  static bool validate_addr(uint32_t addr) {

    // Address must be in SRAM range//地址必须在SRAM范围内
    if (addr >= START_SRAM_ADDR && addr < END_SRAM_ADDR)
      return true;

    // Or in FLASH range//或者在闪光范围内
    if (addr >= START_FLASH_ADDR && addr < END_FLASH_ADDR)
      return true;

    return false;
  }
#endif

bool UnwReadW(const uint32_t a, uint32_t *v) {
  if (!validate_addr(a))
    return false;

  *v = *(uint32_t *)a;
  return true;
}

bool UnwReadH(const uint32_t a, uint16_t *v) {
  if (!validate_addr(a))
    return false;

  *v = *(uint16_t *)a;
  return true;
}

bool UnwReadB(const uint32_t a, uint8_t *v) {
  if (!validate_addr(a))
    return false;

  *v = *(uint8_t *)a;
  return true;
}

#endif // __arm__ || __thumb__//手臂拇指__
