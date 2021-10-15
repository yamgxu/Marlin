/** translatione by yx */
/**
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */

#pragma once

#include "../../../inc/MarlinConfig.h"

#include "macros.h"

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Added by Bill Greiman to speed up mass storage initialization with USB
   * flash drives and simple USB hard drives.
   * Disable this by defining DELAY(x) to be delay(x).
   */
  #define delay(x)  if ((x) < 200) safe_delay(x)
  /* Almost all USB flash drives and simple USB hard drives fail the write
   * protect test and add 20 - 30 seconds to USB init.  Set SKIP_WRITE_PROTECT
   * to nonzero to skip the test and assume the drive is writable.
   */
  #define SKIP_WRITE_PROTECT 1
  /* Since Marlin only cares about USB flash drives, we only need one LUN. */
  #define MASS_MAX_SUPPORTED_LUN 1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI Configuration//SPI配置
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef USB_SPI
  #define USB_SPI SPI
  //#define USB_SPI SPI1//#定义USB_SPI SPI1
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEBUGGING//调试
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 to activate serial debugging */
#define ENABLE_UHS_DEBUGGING 0

/* This can be used to select which serial port to use for debugging if
 * multiple serial ports are available.
 * For example Serial3.
 */
#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #define USB_HOST_SERIAL MYSERIAL1
#endif

#ifndef USB_HOST_SERIAL
  #define USB_HOST_SERIAL Serial
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Manual board activation//手动板激活
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 if you are using an Arduino Mega ADK board with MAX3421e built-in */
#define USE_UHS_MEGA_ADK 0 // If you are using Arduino 1.5.5 or newer there is no need to do this manually//如果您使用的是Arduino 1.5.5或更新版本，则无需手动执行此操作

/* Set this to 1 if you are using a Black Widdow */
#define USE_UHS_BLACK_WIDDOW 0

/* Set this to a one to use the xmem2 lock. This is needed for multitasking and threading */
#define USE_XMEM_SPI_LOCK 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Wii IR camera//Wii红外摄像机
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 to activate code for the Wii IR camera */
#define ENABLE_WII_IR_CAMERA 0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MASS STORAGE//大容量存储
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ******* IMPORTANT *******//*******重要*******
// Set this to 1 to support single LUN devices, and save RAM. -- I.E. thumb drives.//将此设置为1以支持单LUN设备，并保存RAM。-即拇指驱动器。
// Each LUN needs ~13 bytes to be able to track the state of each unit.//每个LUN需要约13个字节才能跟踪每个单元的状态。
#ifndef MASS_MAX_SUPPORTED_LUN
  #define MASS_MAX_SUPPORTED_LUN 8
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set to 1 to use the faster spi4teensy3 driver.//设置为1以使用更快的SPI4TENSY3驱动程序。
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef USE_SPI4TEENSY3
  #define USE_SPI4TEENSY3 1
#endif

// Disabled on the Teensy LC, as it is incompatible for now//在Teensy LC上禁用，因为它目前不兼容
#ifdef __MKL26Z64__
  #undef USE_SPI4TEENSY3
  #define USE_SPI4TEENSY3 0
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTOMATIC Settings//自动设置
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// No user serviceable parts below this line.//此线下无用户可维修零件。
// DO NOT change anything below here unless you are a developer!//除非您是开发人员，否则不要更改下面的任何内容！

//#include "version_helper.h"//#包括“version\u helper.h”

#if defined(__GNUC__) && defined(__AVR__)
  #ifndef GCC_VERSION
    #define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
  #endif
  #if GCC_VERSION < 40602 // Test for GCC < 4.6.2//GCC<4.6.2的试验
    #ifdef PROGMEM
      #undef PROGMEM
      #define PROGMEM __attribute__((section(".progmem.data"))) // Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734#c4//解决方法https://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734#c4
      #ifdef PSTR
        #undef PSTR
        #define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];})) // Copied from pgmspace.h in avr-libc source//从avr libc源中的pgmspace.h复制
      #endif
    #endif
  #endif
#endif

#if !defined(DEBUG_USB_HOST) && ENABLE_UHS_DEBUGGING
  #define DEBUG_USB_HOST
#endif

#if !defined(WIICAMERA) && ENABLE_WII_IR_CAMERA
  #define WIICAMERA
#endif

// To use some other locking (e.g. freertos),//要使用其他锁定（例如freertos），
// define XMEM_ACQUIRE_SPI and XMEM_RELEASE_SPI to point to your lock and unlock.//定义XMEM_ACQUIRE_SPI和XMEM_RELEASE_SPI以指向锁定和解锁。
// NOTE: NO argument is passed. You have to do this within your routine for//注意：不传递任何参数。你必须在日常生活中这样做
// whatever you are using to lock and unlock.//无论你用什么来锁定和解锁。
#ifndef XMEM_ACQUIRE_SPI
  #if USE_XMEM_SPI_LOCK || defined(USE_MULTIPLE_APP_API)
    #include <xmem.h>
  #else
    #define XMEM_ACQUIRE_SPI() (void(0))
    #define XMEM_RELEASE_SPI() (void(0))
  #endif
#endif

#if !defined(EXT_RAM) && defined(EXT_RAM_STACK) || defined(EXT_RAM_HEAP)
  #include <xmem.h>
#else
  #define EXT_RAM 0
#endif

#if defined(CORE_TEENSY) && defined(KINETISK)
  #define USING_SPI4TEENSY3 USE_SPI4TEENSY3
#else
  #define USING_SPI4TEENSY3 0
#endif
#if ((defined(ARDUINO_SAM_DUE) && defined(__SAM3X8E__)) || defined(__ARDUINO_X86__) || ARDUINO >= 10600) && !USING_SPI4TEENSY3
  #include <SPI.h> // Use the Arduino SPI library for the Arduino Due, Intel Galileo 1 & 2, Intel Edison or if the SPI library with transaction is available//对于Arduino Due、Intel Galileo 1&2、Intel Edison或带有事务的SPI库可用，请使用Arduino SPI库
#endif
#ifdef RBL_NRF51822
  #include <nrf_gpio.h>
  #include <SPI_Master.h>
  #define SPI SPI_Master
  #define MFK_CASTUINT8T (uint8_t) // RBLs return type for sizeof needs casting to uint8_t//sizeof的RBLs返回类型需要转换为uint8\t
#endif
#if defined(__PIC32MX__) || defined(__PIC32MZ__)
  #include <../../../../hardware/pic32/libraries/SPI/SPI.h> // Hack to use the SPI library//黑客使用SPI库
#endif

#if defined(ESP8266) || defined(ESP32)
  #define MFK_CASTUINT8T (uint8_t) // ESP return type for sizeof needs casting to uint8_t//sizeof的ESP返回类型需要转换为uint8\t
#endif

#ifdef STM32F4
  #include "stm32f4xx_hal.h"
  extern SPI_HandleTypeDef SPI_Handle; // Needed to be declared in your main.cpp//需要在main.cpp中声明
#endif

// Fix defines on Arduino Due//修复Arduino到期时的定义
#ifdef ARDUINO_SAM_DUE
  #ifdef tokSETUP
    #undef tokSETUP
  #endif
  #ifdef tokIN
    #undef tokIN
  #endif
  #ifdef tokOUT
    #undef tokOUT
  #endif
  #ifdef tokINHS
    #undef tokINHS
  #endif
  #ifdef tokOUTHS
    #undef tokOUTHS
  #endif
#endif

// Set defaults//设置默认值
#ifndef MFK_CASTUINT8T
  #define MFK_CASTUINT8T
#endif

// Workaround issue: https://github.com/esp8266/Arduino/issues/2078//解决问题：https://github.com/esp8266/Arduino/issues/2078
#ifdef ESP8266
  #undef PROGMEM
  #define PROGMEM
#undef PSTR
  #define PSTR(s) (s)
#undef pgm_read_byte
  #define pgm_read_byte(addr) (*reinterpret_cast<const uint8_t*>(addr))
  #undef pgm_read_word
  #define pgm_read_word(addr) (*reinterpret_cast<const uint16_t*>(addr))
#endif

#ifdef ARDUINO_ESP8266_WIFIO
  #error "This board is currently not supported"
#endif
