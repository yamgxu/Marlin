/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2016 Victor Perez victor_pv@hotmail.com
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

#if ENABLED(FLASH_EEPROM_EMULATION)

/* EEPROM emulation over flash with reduced wear
 *
 * We will use 2 contiguous groups of pages as main and alternate.
 * We want an structure that allows to read as fast as possible,
 * without the need of scanning the whole FLASH memory.
 *
 * FLASH bits default erased state is 1, and can be set to 0
 * on a per bit basis. To reset them to 1, a full page erase
 * is needed.
 *
 * Values are stored as differences that should be applied to a
 * completely erased EEPROM (filled with 0xFFs). We just encode
 * the starting address of the values to change, the length of
 * the block of new values, and the values themselves. All diffs
 * are accumulated into a RAM buffer, compacted into the least
 * amount of non overlapping diffs possible and sorted by starting
 * address before being saved into the next available page of FLASH
 * of the current group.
 * Once the current group is completely full, we compact it and save
 * it into the other group, then erase the current group and switch
 * to that new group and set it as current.
 *
 * The FLASH endurance is about 1/10 ... 1/100 of an EEPROM
 * endurance, but EEPROM endurance is specified per byte, not
 * per page. We can't emulate EE endurance with FLASH for all
 * bytes, but we can emulate endurance for a given percent of
 * bytes.
 */

//#define EE_EMU_DEBUG//#定义EE_EMU_调试

#define EEPROMSize     4096
#define PagesPerGroup   128
#define GroupCount        2
#define PageSize        256U

 /* Flash storage */
typedef struct FLASH_SECTOR {
  uint8_t page[PageSize];
} FLASH_SECTOR_T;

#define PAGE_FILL \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, \
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

#define FLASH_INIT_FILL \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL, \
  PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL,PAGE_FILL

/* This is the FLASH area used to emulate a 2Kbyte EEPROM  -- We need this buffer aligned
   to a 256 byte boundary. */
static const uint8_t flashStorage[PagesPerGroup * GroupCount * PageSize] __attribute__ ((aligned (PageSize))) = { FLASH_INIT_FILL };

/* Get the address of an specific page */
static const FLASH_SECTOR_T* getFlashStorage(int page) {
  return (const FLASH_SECTOR_T*)&flashStorage[page*PageSize];
}

static uint8_t buffer[256] = {0},   // The RAM buffer to accumulate writes//用于累积写入的RAM缓冲区
               curPage = 0,         // Current FLASH page inside the group//组内的当前闪存页
               curGroup = 0xFF;     // Current FLASH group//当前闪存组

#define DEBUG_OUT ENABLED(EE_EMU_DEBUG)
#include "../../core/debug_out.h"

static void ee_Dump(const int page, const void *data) {

  #ifdef EE_EMU_DEBUG

    const uint8_t *c = (const uint8_t*) data;
    char buffer[80];

    sprintf_P(buffer, PSTR("Page: %d (0x%04x)\n"), page, page);
    DEBUG_ECHO(buffer);

    char* p = &buffer[0];
    for (int i = 0; i< PageSize; ++i) {
      if ((i & 0xF) == 0) p += sprintf_P(p, PSTR("%04x] "), i);

      p += sprintf_P(p, PSTR(" %02x"), c[i]);
      if ((i & 0xF) == 0xF) {
        *p++ = '\n';
        *p = 0;
        DEBUG_ECHO(buffer);
        p = &buffer[0];
      }
    }

  #else
    UNUSED(page);
    UNUSED(data);
  #endif
}

/* Flash Writing Protection Key */
#define FWP_KEY    0x5Au

#if SAM4S_SERIES
  #define EEFC_FCR_FCMD(value) \
  ((EEFC_FCR_FCMD_Msk & ((value) << EEFC_FCR_FCMD_Pos)))
  #define EEFC_ERROR_FLAGS  (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE | EEFC_FSR_FLERR)
#else
  #define EEFC_ERROR_FLAGS  (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE)
#endif

/**
 * Writes the contents of the specified page (no previous erase)
 * @param page    (page #)
 * @param data    (pointer to the data buffer)
 */
__attribute__ ((long_call, section (".ramfunc")))
static bool ee_PageWrite(uint16_t page, const void *data) {

  uint16_t i;
  uint32_t addrflash = uint32_t(getFlashStorage(page));

  // Read the flash contents//阅读flash内容
  uint32_t pageContents[PageSize>>2];
  memcpy(pageContents, (void*)addrflash, PageSize);

  // We ONLY want to toggle bits that have changed, and that have changed to 0.//我们只想切换已更改的位，以及已更改为0的位。
  // SAM3X8E tends to destroy contiguous bits if reprogrammed without erasing, so//如果在不擦除的情况下重新编程，SAM3X8E往往会破坏连续位，因此
  // we try by all means to avoid this. That is why it says: "The Partial//我们尽一切努力避免这种情况。这就是为什么它说：“部分
  // Programming mode works only with 128-bit (or higher) boundaries. It cannot//编程模式仅适用于128位（或更高）边界。它不能
  // be used with boundaries lower than 128 bits (8, 16 or 32-bit for example)."//用于低于128位的边界（例如8、16或32位）。”
  // All bits that did not change, set them to 1.//所有未更改的位都设置为1。
  for (i = 0; i <PageSize >> 2; i++)
    pageContents[i] = (((uint32_t*)data)[i]) | (~(pageContents[i] ^ ((uint32_t*)data)[i]));

  DEBUG_ECHO_START();
  DEBUG_ECHOLNPAIR("EEPROM PageWrite   ", page);
  DEBUG_ECHOLNPAIR(" in FLASH address ", (uint32_t)addrflash);
  DEBUG_ECHOLNPAIR(" base address     ", (uint32_t)getFlashStorage(0));
  DEBUG_FLUSH();

  // Get the page relative to the start of the EFC controller, and the EFC controller to use//获取与EFC控制器的开头和要使用的EFC控制器相关的页面
  Efc *efc;
  uint16_t fpage;
  if (addrflash >= IFLASH1_ADDR) {
    efc = EFC1;
    fpage = (addrflash - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
  }
  else {
    efc = EFC0;
    fpage = (addrflash - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
  }

  // Get the page that must be unlocked, then locked//获取必须解锁然后锁定的页面
  uint16_t lpage = fpage & (~((IFLASH0_LOCK_REGION_SIZE / IFLASH0_PAGE_SIZE) - 1));

  // Disable all interrupts//禁用所有中断
  __disable_irq();

  // Get the FLASH wait states//获取闪存等待状态
  uint32_t orgWS = (efc->EEFC_FMR & EEFC_FMR_FWS_Msk) >> EEFC_FMR_FWS_Pos;

  // Set wait states to 6 (SAM errata)//将等待状态设置为6（SAM勘误表）
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(6);

  // Unlock the flash page//解锁flash页面
  uint32_t status;
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(lpage) | EEFC_FCR_FCMD(EFC_FCMD_CLB);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!//强制编译器不对此进行优化--NOPs不起作用！
    __asm__ __volatile__("");
  };

  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states//恢复原始等待状态
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts//可重入中断
    __enable_irq();

    DEBUG_ECHO_START();
    DEBUG_ECHOLNPAIR("EEPROM Unlock failure for page ", page);
    return false;
  }

  // Write page and lock:  Writing 8-bit and 16-bit data is not allowed and may lead to unpredictable data corruption.//写入页面和锁定：不允许写入8位和16位数据，这可能会导致无法预测的数据损坏。
  const uint32_t * aligned_src = (const uint32_t *) &pageContents[0]; /*data;*/
  uint32_t * p_aligned_dest = (uint32_t *) addrflash;
  for (i = 0; i < (IFLASH0_PAGE_SIZE / sizeof(uint32_t)); ++i) {
    *p_aligned_dest++ = *aligned_src++;
  }
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(fpage) | EEFC_FCR_FCMD(EFC_FCMD_WPL);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!//强制编译器不对此进行优化--NOPs不起作用！
    __asm__ __volatile__("");
  };

  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states//恢复原始等待状态
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts//可重入中断
    __enable_irq();

    DEBUG_ECHO_START();
    DEBUG_ECHOLNPAIR("EEPROM Write failure for page ", page);

    return false;
  }

  // Restore original wait states//恢复原始等待状态
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

  // Reenable interrupts//可重入中断
  __enable_irq();

  // Compare contents//比较内容
  if (memcmp(getFlashStorage(page),data,PageSize)) {

    #ifdef EE_EMU_DEBUG
      DEBUG_ECHO_START();
      DEBUG_ECHOLNPAIR("EEPROM Verify Write failure for page ", page);

      ee_Dump( page, (uint32_t *)addrflash);
      ee_Dump(-page, data);

      // Calculate count of changed bits//计算更改位的计数
      uint32_t *p1 = (uint32_t*)addrflash;
      uint32_t *p2 = (uint32_t*)data;
      int count = 0;
      for (i =0; i<PageSize >> 2; i++) {
        if (p1[i] != p2[i]) {
          uint32_t delta = p1[i] ^ p2[i];
          while (delta) {
            if ((delta&1) != 0)
              count++;
            delta >>= 1;
          }
        }
      }
      DEBUG_ECHOLNPAIR("--> Differing bits: ", count);
    #endif

    return false;
  }

  return true;
}

/**
 * Erases the contents of the specified page
 * @param page    (page #)
  */
__attribute__ ((long_call, section (".ramfunc")))
static bool ee_PageErase(uint16_t page) {

  uint16_t i;
  uint32_t addrflash = uint32_t(getFlashStorage(page));

  DEBUG_ECHO_START();
  DEBUG_ECHOLNPAIR("EEPROM PageErase  ", page);
  DEBUG_ECHOLNPAIR(" in FLASH address ", (uint32_t)addrflash);
  DEBUG_ECHOLNPAIR(" base address     ", (uint32_t)getFlashStorage(0));
  DEBUG_FLUSH();

  // Get the page relative to the start of the EFC controller, and the EFC controller to use//获取与EFC控制器的开头和要使用的EFC控制器相关的页面
  Efc *efc;
  uint16_t fpage;
  if (addrflash >= IFLASH1_ADDR) {
    efc = EFC1;
    fpage = (addrflash - IFLASH1_ADDR) / IFLASH1_PAGE_SIZE;
  }
  else {
    efc = EFC0;
    fpage = (addrflash - IFLASH0_ADDR) / IFLASH0_PAGE_SIZE;
  }

  // Get the page that must be unlocked, then locked//获取必须解锁然后锁定的页面
  uint16_t lpage = fpage & (~((IFLASH0_LOCK_REGION_SIZE / IFLASH0_PAGE_SIZE) - 1));

  // Disable all interrupts//禁用所有中断
  __disable_irq();

  // Get the FLASH wait states//获取闪存等待状态
  uint32_t orgWS = (efc->EEFC_FMR & EEFC_FMR_FWS_Msk) >> EEFC_FMR_FWS_Pos;

  // Set wait states to 6 (SAM errata)//将等待状态设置为6（SAM勘误表）
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(6);

  // Unlock the flash page//解锁flash页面
  uint32_t status;
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(lpage) | EEFC_FCR_FCMD(EFC_FCMD_CLB);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!//强制编译器不对此进行优化--NOPs不起作用！
    __asm__ __volatile__("");
  };
  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states//恢复原始等待状态
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts//可重入中断
    __enable_irq();

    DEBUG_ECHO_START();
    DEBUG_ECHOLNPAIR("EEPROM Unlock failure for page ",page);

    return false;
  }

  // Erase Write page and lock: Writing 8-bit and 16-bit data is not allowed and may lead to unpredictable data corruption.//擦除写入页并锁定：不允许写入8位和16位数据，并可能导致无法预测的数据损坏。
  uint32_t * p_aligned_dest = (uint32_t *) addrflash;
  for (i = 0; i < (IFLASH0_PAGE_SIZE / sizeof(uint32_t)); ++i) {
    *p_aligned_dest++ = 0xFFFFFFFF;
  }
  efc->EEFC_FCR = EEFC_FCR_FKEY(FWP_KEY) | EEFC_FCR_FARG(fpage) | EEFC_FCR_FCMD(EFC_FCMD_EWPL);
  while (((status = efc->EEFC_FSR) & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) {
    // force compiler to not optimize this -- NOPs don't work!//强制编译器不对此进行优化--NOPs不起作用！
    __asm__ __volatile__("");
  };
  if ((status & EEFC_ERROR_FLAGS) != 0) {

    // Restore original wait states//恢复原始等待状态
    efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

    // Reenable interrupts//可重入中断
    __enable_irq();

    DEBUG_ECHO_START();
    DEBUG_ECHOLNPAIR("EEPROM Erase failure for page ",page);

    return false;
  }

  // Restore original wait states//恢复原始等待状态
  efc->EEFC_FMR = (efc->EEFC_FMR & (~EEFC_FMR_FWS_Msk)) | EEFC_FMR_FWS(orgWS);

  // Reenable interrupts//可重入中断
  __enable_irq();

  // Check erase//检查擦除
  uint32_t * aligned_src = (uint32_t *) addrflash;
  for (i = 0; i < PageSize >> 2; i++) {
    if (*aligned_src++ != 0xFFFFFFFF) {
      DEBUG_ECHO_START();
      DEBUG_ECHOLNPAIR("EEPROM Verify Erase failure for page ",page);
      ee_Dump(page, (uint32_t *)addrflash);
      return false;
    }
  }

  return true;
}

static uint8_t ee_Read(uint32_t address, bool excludeRAMBuffer=false) {

  uint32_t baddr;
  uint32_t blen;

  // If we were requested an address outside of the emulated range, fail now//如果请求的地址超出模拟范围，请立即失败
  if (address >= EEPROMSize)
    return false;

  // Check that the value is not contained in the RAM buffer//检查RAM缓冲区中是否不包含该值
  if (!excludeRAMBuffer) {
    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block//获取块的地址
      baddr = buffer[i] | (buffer[i + 1] << 8);

      // Get the length of the block//获取块的长度
      blen = buffer[i + 2];

      // If we reach the end of the list, break loop//如果我们到达列表的末尾，请中断循环
      if (blen == 0xFF)
        break;

      // Check if data is contained in this block//检查此块中是否包含数据
      if (address >= baddr &&
        address < (baddr + blen)) {

        // Yes, it is contained. Return it!//是的，它是包含的。归还它！
        return buffer[i + 3 + address - baddr];
      }

      // As blocks are always sorted, if the starting address of this block is higher//因为块总是被排序的，如果这个块的起始地址更高
      // than the address we are looking for, break loop now - We wont find the value//比我们正在寻找的地址，现在打破循环-我们找不到值
      // associated to the address//与地址关联
      if (baddr > address)
        break;

      // Jump to the next block//跳到下一个街区
      i += 3 + blen;
    }
  }

  // It is NOT on the RAM buffer. It could be stored in FLASH. We are//它不在RAM缓冲区上。它可以存储在闪存中。我们是
  //  ensured on a given FLASH page, address contents are never repeated//确保在给定的FLASH页面上，地址内容不会重复
  //  but on different pages, there is no such warranty, so we must go//但是在不同的页面上，没有这样的保证，所以我们必须去
  //  backwards from the last written FLASH page to the first one.//从最后一个写的FLASH页面向后到第一个页面。
  for (int page = curPage - 1; page >= 0; --page) {

    // Get a pointer to the flash page//获取指向flash页面的指针
    uint8_t *pflash = (uint8_t*)getFlashStorage(page + curGroup * PagesPerGroup);

    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block//获取块的地址
      baddr = pflash[i] | (pflash[i + 1] << 8);

      // Get the length of the block//获取块的长度
      blen = pflash[i + 2];

      // If we reach the end of the list, break loop//如果我们到达列表的末尾，请中断循环
      if (blen == 0xFF)
        break;

      // Check if data is contained in this block//检查此块中是否包含数据
      if (address >= baddr && address < (baddr + blen))
        return pflash[i + 3 + address - baddr]; // Yes, it is contained. Return it!//是的，它是包含的。还它！

      // As blocks are always sorted, if the starting address of this block is higher//因为块总是被排序的，如果这个块的起始地址更高
      // than the address we are looking for, break loop now - We wont find the value//比我们正在寻找的地址，现在打破循环-我们找不到值
      // associated to the address//与地址关联
      if (baddr > address) break;

      // Jump to the next block//跳到下一个街区
      i += 3 + blen;
    }
  }

  // If reached here, value is not stored, so return its default value//如果在此处达到，则不会存储该值，因此返回其默认值
  return 0xFF;
}

static uint32_t ee_GetAddrRange(uint32_t address, bool excludeRAMBuffer=false) {
  uint32_t baddr,
           blen,
           nextAddr = 0xFFFF,
           nextRange = 0;

  // Check that the value is not contained in the RAM buffer//检查RAM缓冲区中是否不包含该值
  if (!excludeRAMBuffer) {
    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block//获取块的地址
      baddr = buffer[i] | (buffer[i + 1] << 8);

      // Get the length of the block//获取块的长度
      blen = buffer[i + 2];

      // If we reach the end of the list, break loop//如果我们到达列表的末尾，请中断循环
      if (blen == 0xFF) break;

      // Check if address and address + 1 is contained in this block//检查此块中是否包含地址和地址+1
      if (address >= baddr && address < (baddr + blen))
        return address | ((blen - address + baddr) << 16); // Yes, it is contained. Return it!//是的，它是包含的。还它！

      // Otherwise, check if we can use it as a limit//否则，请检查是否可以将其用作限制
      if (baddr > address && baddr < nextAddr) {
        nextAddr = baddr;
        nextRange = blen;
      }

      // As blocks are always sorted, if the starting address of this block is higher//因为块总是被排序的，如果这个块的起始地址更高
      // than the address we are looking for, break loop now - We wont find the value//比我们正在寻找的地址，现在打破循环-我们找不到值
      // associated to the address//与地址关联
      if (baddr > address) break;

      // Jump to the next block//跳到下一个街区
      i += 3 + blen;
    }
  }

  // It is NOT on the RAM buffer. It could be stored in FLASH. We are//它不在RAM缓冲区上。它可以存储在闪存中。我们是
  //  ensured on a given FLASH page, address contents are never repeated//确保在给定的FLASH页面上，地址内容不会重复
  //  but on different pages, there is no such warranty, so we must go//但是在不同的页面上，没有这样的保证，所以我们必须去
  //  backwards from the last written FLASH page to the first one.//从最后一个写的FLASH页面向后到第一个页面。
  for (int page = curPage - 1; page >= 0; --page) {

    // Get a pointer to the flash page//获取指向flash页面的指针
    uint8_t *pflash = (uint8_t*)getFlashStorage(page + curGroup * PagesPerGroup);

    uint16_t i = 0;
    while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Get the address of the block//获取块的地址
      baddr = pflash[i] | (pflash[i + 1] << 8);

      // Get the length of the block//获取块的长度
      blen = pflash[i + 2];

      // If we reach the end of the list, break loop//如果我们到达列表的末尾，请中断循环
      if (blen == 0xFF) break;

      // Check if data is contained in this block//检查此块中是否包含数据
      if (address >= baddr && address < (baddr + blen))
        return address | ((blen - address + baddr) << 16); // Yes, it is contained. Return it!//是的，它是包含的。还它！

      // Otherwise, check if we can use it as a limit//否则，请检查是否可以将其用作限制
      if (baddr > address && baddr < nextAddr) {
        nextAddr = baddr;
        nextRange = blen;
      }

      // As blocks are always sorted, if the starting address of this block is higher//因为块总是被排序的，如果这个块的起始地址更高
      // than the address we are looking for, break loop now - We wont find the value//比我们正在寻找的地址，现在打破循环-我们找不到值
      // associated to the address//与地址关联
      if (baddr > address) break;

      // Jump to the next block//跳到下一个街区
      i += 3 + blen;
    }
  }

  // If reached here, we will return the next valid address//如果到达此处，我们将返回下一个有效地址
  return nextAddr | (nextRange << 16);
}

static bool ee_IsPageClean(int page) {
  uint32_t *pflash = (uint32_t*) getFlashStorage(page);
  for (uint16_t i = 0; i < (PageSize >> 2); ++i)
    if (*pflash++ != 0xFFFFFFFF) return false;
  return true;
}

static bool ee_Flush(uint32_t overrideAddress = 0xFFFFFFFF, uint8_t overrideData=0xFF) {

  // Check if RAM buffer has something to be written//检查RAM缓冲区是否有要写入的内容
  bool isEmpty = true;
  uint32_t *p = (uint32_t*) &buffer[0];
  for (uint16_t j = 0; j < (PageSize >> 2); j++) {
    if (*p++ != 0xFFFFFFFF) {
      isEmpty = false;
      break;
    }
  }

  // If something has to be written, do so!//如果一定要写东西，就写吧！
  if (!isEmpty) {

    // Write the current ram buffer into FLASH//将当前ram缓冲区写入闪存
    ee_PageWrite(curPage + curGroup * PagesPerGroup, buffer);

    // Clear the RAM buffer//清除RAM缓冲区
    memset(buffer, 0xFF, sizeof(buffer));

    // Increment the page to use the next time//增加页面以便下次使用
    ++curPage;
  }

  // Did we reach the maximum count of available pages per group for storage ?//每个存储组的可用页面数是否达到最大值？
  if (curPage < PagesPerGroup) {

    // Do we have an override address ?//我们有替代地址吗？
    if (overrideAddress < EEPROMSize) {

      // Yes, just store the value into the RAM buffer//是的，只需将值存储到RAM缓冲区中
      buffer[0] = overrideAddress & 0xFF;
      buffer[0 + 1] = (overrideAddress >> 8) & 0xFF;
      buffer[0 + 2] = 1;
      buffer[0 + 3] = overrideData;
    }

    // Done!//完成了！
    return true;
  }

  // We have no space left on the current group - We must compact the values//我们在当前组上没有剩余空间-我们必须压缩值
  uint16_t i = 0;

  // Compute the next group to use//计算下一个要使用的组
  int curwPage = 0, curwGroup = curGroup + 1;
  if (curwGroup >= GroupCount) curwGroup = 0;

  uint32_t rdAddr = 0;
  do {

    // Get the next valid range//获取下一个有效范围
    uint32_t addrRange = ee_GetAddrRange(rdAddr, true);

    // Make sure not to skip the override address, if specified//如果指定，请确保不要跳过覆盖地址
    int rdRange;
    if (overrideAddress < EEPROMSize &&
      rdAddr <= overrideAddress &&
      (addrRange & 0xFFFF) > overrideAddress) {

      rdAddr = overrideAddress;
      rdRange = 1;
    }
    else {
      rdAddr = addrRange & 0xFFFF;
      rdRange = addrRange >> 16;
    }

    // If no range, break loop//如果没有范围，则断开循环
    if (rdRange == 0)
      break;

    do {

      // Get the value//获取值
      uint8_t rdValue = overrideAddress == rdAddr ? overrideData : ee_Read(rdAddr, true);

      // Do not bother storing default values//不要麻烦存储默认值
      if (rdValue != 0xFF) {

        // If we have room, add it to the buffer//如果我们有空间，把它加入缓冲区
        if (buffer[i + 2] == 0xFF) {

          // Uninitialized buffer, just add it!//未初始化的缓冲区，只需添加它！
          buffer[i] = rdAddr & 0xFF;
          buffer[i + 1] = (rdAddr >> 8) & 0xFF;
          buffer[i + 2] = 1;
          buffer[i + 3] = rdValue;

        }
        else {
          // Buffer already has contents. Check if we can extend it//缓冲区已包含内容。看看我们是否可以延长它

          // Get the address of the block//获取块的地址
          uint32_t baddr = buffer[i] | (buffer[i + 1] << 8);

          // Get the length of the block//获取块的长度
          uint32_t blen = buffer[i + 2];

          // Can we expand it ?//我们可以扩展它吗？
          if (rdAddr == (baddr + blen) &&
            i < (PageSize - 4) && /* This block has a chance to contain data AND */
            buffer[i + 2] < (PageSize - i - 3)) {/* There is room for this block to be expanded */

            // Yes, do it//是的，做吧
            ++buffer[i + 2];

            // And store the value//并存储值
            buffer[i + 3 + rdAddr - baddr] = rdValue;

          }
          else {

            // No, we can't expand it - Skip the existing block//不，我们无法展开它-跳过现有块
            i += 3 + blen;

            // Can we create a new slot ?//我们可以创建一个新插槽吗？
            if (i > (PageSize - 4)) {

              // Not enough space - Write the current buffer to FLASH//空间不足-将当前缓冲区写入闪存
              ee_PageWrite(curwPage + curwGroup * PagesPerGroup, buffer);

              // Advance write page (as we are compacting, should never overflow!)//高级写入页（当我们压缩时，永远不会溢出！）
              ++curwPage;

              // Clear RAM buffer//清除RAM缓冲区
              memset(buffer, 0xFF, sizeof(buffer));

              // Start fresh *///重新开始*/
              i = 0;
            }

            // Enough space, add the new block//如果空间足够，请添加新块
            buffer[i] = rdAddr & 0xFF;
            buffer[i + 1] = (rdAddr >> 8) & 0xFF;
            buffer[i + 2] = 1;
            buffer[i + 3] = rdValue;
          }
        }
      }

      // Go to the next address//转到下一个地址
      ++rdAddr;

      // Repeat for bytes of this range//对该范围的字节重复此操作
    } while (--rdRange);

    // Repeat until we run out of ranges//重复，直到超出范围
  } while (rdAddr < EEPROMSize);

  // We must erase the previous group, in preparation for the next swap//我们必须删除上一组，为下一次交换做准备
  for (int page = 0; page < curPage; page++) {
    ee_PageErase(page + curGroup * PagesPerGroup);
  }

  // Finally, Now the active group is the created new group//最后，现在活动组是创建的新组
  curGroup = curwGroup;
  curPage = curwPage;

  // Done!//完成了！
  return true;
}

static bool ee_Write(uint32_t address, uint8_t data) {

  // If we were requested an address outside of the emulated range, fail now//如果请求的地址超出模拟范围，请立即失败
  if (address >= EEPROMSize) return false;

  // Lets check if we have a block with that data previously defined. Block//让我们检查是否有一个块包含先前定义的数据。块
  //  start addresses are always sorted in ascending order//起始地址总是按升序排序
  uint16_t i = 0;
  while (i <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

    // Get the address of the block//获取块的地址
    uint32_t baddr = buffer[i] | (buffer[i + 1] << 8);

    // Get the length of the block//获取块的长度
    uint32_t blen = buffer[i + 2];

    // If we reach the end of the list, break loop//如果我们到达列表的末尾，请中断循环
    if (blen == 0xFF)
      break;

    // Check if data is contained in this block//检查此块中是否包含数据
    if (address >= baddr &&
      address < (baddr + blen)) {

      // Yes, it is contained. Just modify it//是的，它是包含的。修改一下就行了
      buffer[i + 3 + address - baddr] = data;

      // Done!//完成了！
      return true;
    }

    // Maybe we could add it to the front or to the back//也许我们可以把它加在前面或后面
    // of this block ?//这条街的尽头？
    if ((address + 1) == baddr || address == (baddr + blen)) {

      // Potentially, it could be done. But we must ensure there is room//也许，这是可以做到的。但我们必须确保有空间
      // so we can expand the block. Lets find how much free space remains//所以我们可以扩展块。让我们看看还有多少可用空间
      uint32_t iend = i;
      do {
        uint32_t ln = buffer[iend + 2];
        if (ln == 0xFF) break;
        iend += 3 + ln;
      } while (iend <= (PageSize - 4)); /* (PageSize - 4) because otherwise, there is not enough room for data and headers */

      // Here, inxt points to the first free address in the buffer. Do we have room ?//这里，inxt指向缓冲区中的第一个空闲地址。我们有房间吗？
      if (iend < PageSize) {
        // Yes, at least a byte is free - We can expand the block//是的，至少有一个字节是空闲的-我们可以扩展块

        // Do we have to insert at the beginning ?//我们必须在开头插入吗？
        if ((address + 1) == baddr) {

          // Insert at the beginning//在开头插入

          // Make room at the beginning for our byte//在开始时为我们的工作腾出空间
          memmove(&buffer[i + 3 + 1], &buffer[i + 3], iend - i - 3);

          // Adjust the header and store the data//调整标题并存储数据
          buffer[i] = address & 0xFF;
          buffer[i + 1] = (address >> 8) & 0xFF;
          buffer[i + 2]++;
          buffer[i + 3] = data;

        }
        else {

          // Insert at the end - There is a very interesting thing that could happen here://在结尾插入-这里可能会发生一件非常有趣的事情：
          //  Maybe we could coalesce the next block with this block. Let's try to do it!//也许我们可以把下一个街区和这个街区合并起来。让我们试着去做吧！
          uint16_t inext = i + 3 + blen;
          if (inext <= (PageSize - 4) &&
            (buffer[inext] | uint16_t(buffer[inext + 1] << 8)) == (baddr + blen + 1)) {
            // YES! ... we can coalesce blocks! . Do it!//是的。。。我们可以合并块。做吧！

            // Adjust this block header to include the next one//调整此块标题以包括下一块标题
            buffer[i + 2] += buffer[inext + 2] + 1;

            // Store data at the right place//将数据存储在正确的位置
            buffer[i + 3 + blen] = data;

            // Remove the next block header and append its data//删除下一个块标题并附加其数据
            memmove(&buffer[inext + 1], &buffer[inext + 3], iend - inext - 3);

            // Finally, as we have saved 2 bytes at the end, make sure to clean them//最后，由于我们在最后保存了2个字节，请确保清除它们
            buffer[iend - 2] = 0xFF;
            buffer[iend - 1] = 0xFF;

          }
          else {
            // NO ... No coalescing possible yet//不。。。还不可能合并

            // Make room at the end for our byte//在末尾给我们的字节腾出空间
            memmove(&buffer[i + 3 + blen + 1], &buffer[i + 3 + blen], iend - i - 3 - blen);

            // And add the data to the block//并将数据添加到块中
            buffer[i + 2]++;
            buffer[i + 3 + blen] = data;
          }
        }

        // Done!//完成了！
        return true;
      }
    }

    // As blocks are always sorted, if the starting address of this block is higher//因为块总是被排序的，如果这个块的起始地址更高
    // than the address we are looking for, break loop now - We wont find the value//比我们正在寻找的地址，现在打破循环-我们找不到值
    // associated to the address//与地址关联
    if (baddr > address) break;

    // Jump to the next block//跳到下一个街区
    i += 3 + blen;
  }

  // Value is not stored AND we can't expand previous block to contain it. We must create a new block//值未存储，无法展开上一个块以包含它。我们必须创建一个新块

  // First, lets find how much free space remains//首先，让我们看看还有多少可用空间
  uint32_t iend = i;
  while (iend <= (PageSize - 4)) { /* (PageSize - 4) because otherwise, there is not enough room for data and headers */
    uint32_t ln = buffer[iend + 2];
    if (ln == 0xFF) break;
    iend += 3 + ln;
  }

  // If there is room for a new block, insert it at the proper place//如果有空间放置新块，请将其插入适当的位置
  if (iend <= (PageSize - 4)) {

    // We have room to create a new block. Do so --- But add//我们有空间创建一个新的块。这样做，但要加上
    // the block at the proper position, sorted by starting//块位于正确的位置，按开始排序
    // address, so it will be possible to compact it with other blocks.//地址，因此可以将其与其他块压缩。

    // Make space//腾出空间
    memmove(&buffer[i + 4], &buffer[i], iend - i);

    // And add the block//然后添加块
    buffer[i] = address & 0xFF;
    buffer[i + 1] = (address >> 8) & 0xFF;
    buffer[i + 2] = 1;
    buffer[i + 3] = data;

    // Done!//完成了！
    return true;
  }

  // Not enough room to store this information on this FLASH page -  Perform a//没有足够的空间在此闪存页上存储此信息-执行
  // flush and override the address with the specified contents//用指定的内容刷新并重写地址
  return ee_Flush(address, data);
}

static void ee_Init() {

  // Just init once!//就一次！
  if (curGroup != 0xFF) return;

  // Clean up the SRAM buffer//清理SRAM缓冲区
  memset(buffer, 0xFF, sizeof(buffer));

  // Now, we must find out the group where settings are stored//现在，我们必须找出存储设置的组
  for (curGroup = 0; curGroup < GroupCount; curGroup++)
    if (!ee_IsPageClean(curGroup * PagesPerGroup)) break;

  // If all groups seem to be used, default to first group//如果似乎使用了所有组，则默认为第一组
  if (curGroup >= GroupCount) curGroup = 0;

  DEBUG_ECHO_START();
  DEBUG_ECHOLNPAIR("EEPROM Current Group: ",curGroup);
  DEBUG_FLUSH();

  // Now, validate that all the other group pages are empty//现在，验证所有其他组页面是否为空
  for (int grp = 0; grp < GroupCount; grp++) {
    if (grp == curGroup) continue;

    for (int page = 0; page < PagesPerGroup; page++) {
      if (!ee_IsPageClean(grp * PagesPerGroup + page)) {
        DEBUG_ECHO_START();
        DEBUG_ECHOLNPAIR("EEPROM Page ", page, " not clean on group ", grp);
        DEBUG_FLUSH();
        ee_PageErase(grp * PagesPerGroup + page);
      }
    }
  }

  // Finally, for the active group, determine the first unused page//最后，对于活动组，确定第一个未使用的页面
  // and also validate that all the other ones are clean//并验证所有其他的都是干净的
  for (curPage = 0; curPage < PagesPerGroup; curPage++) {
    if (ee_IsPageClean(curGroup * PagesPerGroup + curPage)) {
      ee_Dump(curGroup * PagesPerGroup + curPage, getFlashStorage(curGroup * PagesPerGroup + curPage));
      break;
    }
  }

  DEBUG_ECHO_START();
  DEBUG_ECHOLNPAIR("EEPROM Active page: ", curPage);
  DEBUG_FLUSH();

  // Make sure the pages following the first clean one are also clean//确保第一个干净页面后面的页面也干净
  for (int page = curPage + 1; page < PagesPerGroup; page++) {
    if (!ee_IsPageClean(curGroup * PagesPerGroup + page)) {
      DEBUG_ECHO_START();
      DEBUG_ECHOLNPAIR("EEPROM Page ", page, " not clean on active group ", curGroup);
      DEBUG_FLUSH();
      ee_Dump(curGroup * PagesPerGroup + page, getFlashStorage(curGroup * PagesPerGroup + page));
      ee_PageErase(curGroup * PagesPerGroup + page);
    }
  }
}

/* PersistentStore -----------------------------------------------------------*/

#include "../shared/eeprom_api.h"

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE 0x1000 // 4KB//4KB
#endif
size_t PersistentStore::capacity()    { return MARLIN_EEPROM_SIZE; }
bool PersistentStore::access_start()  { ee_Init();  return true; }
bool PersistentStore::access_finish() { ee_Flush(); return true; }

bool PersistentStore::write_data(int &pos, const uint8_t *value, size_t size, uint16_t *crc) {
  uint16_t written = 0;
  while (size--) {
    uint8_t * const p = (uint8_t * const)pos;
    uint8_t v = *value;
    if (v != ee_Read(uint32_t(p))) { // EEPROM has only ~100,000 write cycles, so only write bytes that have changed!//EEPROM只有约100000个写入周期，因此只有已更改的写入字节！
      ee_Write(uint32_t(p), v);
      if (++written & 0x7F) delay(2); else safe_delay(2); // Avoid triggering watchdog during long EEPROM writes//避免在长时间EEPROM写入期间触发看门狗
      if (ee_Read(uint32_t(p)) != v) {
        SERIAL_ECHO_MSG(STR_ERR_EEPROM_WRITE);
        return true;
      }
    }
    crc16(crc, &v, 1);
    pos++;
    value++;
  }
  return false;
}

bool PersistentStore::read_data(int &pos, uint8_t *value, size_t size, uint16_t *crc, const bool writing/*=true*/) {
  do {
    uint8_t c = ee_Read(uint32_t(pos));
    if (writing) *value = c;
    crc16(crc, &c, 1);
    pos++;
    value++;
  } while (--size);
  return false;
}

#endif // FLASH_EEPROM_EMULATION//闪存EEPROM模拟
#endif // ARDUINO_ARCH_SAM//阿杜伊诺·阿丘·萨姆
