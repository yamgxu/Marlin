/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 * Copyright (c) 2017 Victor Perez
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

#include "HAL.h"
#include "usb_serial.h"

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

#ifdef USBCON
  DefaultSerial1 MSerial0(false, SerialUSB);
#endif

#if ENABLED(SRAM_EEPROM_EMULATION)
  #if STM32F7xx
    #include <stm32f7xx_ll_pwr.h>
  #elif STM32F4xx
    #include <stm32f4xx_ll_pwr.h>
  #else
    #error "SRAM_EEPROM_EMULATION is currently only supported for STM32F4xx and STM32F7xx"
  #endif
#endif

#if HAS_SD_HOST_DRIVE
  #include "msc_sd.h"
  #include "usbd_cdc_if.h"
#endif

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

uint16_t HAL_adc_result;

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

// HAL initialization task//HAL初始化任务
void HAL_init() {
  FastIO_init();

  // Ensure F_CPU is a constant expression.//确保F_CPU是一个常量表达式。
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.//如果编译器在这里中断，这意味着应该在编译时计算的延迟代码将无法工作。
  // So better safe than sorry here.//所以这里安全总比抱歉好。
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if ENABLED(SDSUPPORT) && DISABLED(SDIO_SUPPORT) && (defined(SDSS) && SDSS != -1)
    OUT_WRITE(SDSS, HIGH); // Try to set SDSS inactive before any other SPI users start up//在任何其他SPI用户启动之前，尝试将SDS设置为非活动状态
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM//启用对备份SRAM的访问
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    LL_PWR_EnableBkUpRegulator();         // Enable backup regulator//启用备用调节器
    while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized//等待备用调节器初始化
  #endif

  SetTimerInterruptPriorities();

  #if ENABLED(EMERGENCY_PARSER) && USBD_USE_CDC
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial()); // Install the min serial handler//安装最小串行处理程序

  #if HAS_SD_HOST_DRIVE
    MSC_SD_init();                         // Enable USB SD card access//启用USB SD卡访问
  #endif

  #if PIN_EXISTS(USB_CONNECT)
    OUT_WRITE(USB_CONNECT_PIN, !USB_CONNECT_INVERTING);  // USB clear connection//USB清除连接
    delay(1000);                                         // Give OS time to notice//给操作系统时间通知
    WRITE(USB_CONNECT_PIN, USB_CONNECT_INVERTING);
  #endif
}

// HAL idle task//HAL空闲任务
void HAL_idletask() {
  #if HAS_SHARED_MEDIA
    // Stm32duino currently doesn't have a "loop/idle" method//Stm32duino当前没有“循环/空闲”方法
    CDC_resume_receive();
    CDC_continue_transmit();
  #endif
}

void HAL_clear_reset_source() { __HAL_RCC_CLEAR_RESET_FLAGS(); }

uint8_t HAL_get_reset_source() {
  return
    #ifdef RCC_FLAG_IWDGRST // Some sources may not exist...//有些来源可能不存在。。。
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)  ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG1RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG2RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG2RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_SFTRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)   ? RST_SOFTWARE :
    #endif
    #ifdef RCC_FLAG_PINRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)   ? RST_EXTERNAL :
    #endif
    #ifdef RCC_FLAG_PORRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)   ? RST_POWER_ON :
    #endif
    0
  ;
}

void HAL_reboot() { NVIC_SystemReset(); }

void _delay_ms(const int delay_ms) { delay(delay_ms); }

extern "C" {
  extern unsigned int _ebss; // end of bss section//bss部分结束
}

// ------------------------// ------------------------
// ADC//模数转换器
// ------------------------// ------------------------

// TODO: Make sure this doesn't cause any delay//TODO:确保这不会造成任何延迟
void HAL_adc_start_conversion(const uint8_t adc_pin) { HAL_adc_result = analogRead(adc_pin); }
uint16_t HAL_adc_get_result() { return HAL_adc_result; }

// Reset the system to initiate a firmware flash//重置系统以启动固件闪存
void flashFirmware(const int16_t) { HAL_reboot(); }

// Maple Compatibility//枫树兼容性
volatile uint32_t systick_uptime_millis = 0;
systickCallback_t systick_user_callback;
void systick_attach_callback(systickCallback_t cb) { systick_user_callback = cb; }
void HAL_SYSTICK_Callback() {
  systick_uptime_millis++;
  if (systick_user_callback) systick_user_callback();
}

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC//ARDUINO_ARCH_STM32&&！STM32通用
