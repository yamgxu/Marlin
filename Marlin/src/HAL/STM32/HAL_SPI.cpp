/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

#include "../../inc/MarlinConfig.h"

#include <SPI.h>

// ------------------------// ------------------------
// Public Variables//公共变量
// ------------------------// ------------------------

static SPISettings spiConfig;

// ------------------------// ------------------------
// Public functions//公共职能
// ------------------------// ------------------------

#if ENABLED(SOFTWARE_SPI)

  // ------------------------// ------------------------
  // Software SPI//软件SPI
  // ------------------------// ------------------------

  #include "../shared/Delay.h"

  void spiBegin(void) {
    OUT_WRITE(SD_SS_PIN, HIGH);
    OUT_WRITE(SD_SCK_PIN, HIGH);
    SET_INPUT(SD_MISO_PIN);
    OUT_WRITE(SD_MOSI_PIN, HIGH);
  }

  // Use function with compile-time value so we can actually reach the desired frequency//使用具有编译时值的函数，以便实际达到所需的频率
  // Need to adjust this a little bit: on a 72MHz clock, we have 14ns/clock//需要稍微调整一下：在72MHz时钟上，我们有14ns/时钟
  // and we'll use ~3 cycles to jump to the method and going back, so it'll take ~40ns from the given clock here//我们将使用~3个周期跳转到方法并返回，因此从给定的时钟开始需要~40ns
  #define CALLING_COST_NS  (3U * 1000000000U) / (F_CPU)
  void (*delaySPIFunc)();
  void delaySPI_125()  { DELAY_NS(125 - CALLING_COST_NS); }
  void delaySPI_250()  { DELAY_NS(250 - CALLING_COST_NS); }
  void delaySPI_500()  { DELAY_NS(500 - CALLING_COST_NS); }
  void delaySPI_1000() { DELAY_NS(1000 - CALLING_COST_NS); }
  void delaySPI_2000() { DELAY_NS(2000 - CALLING_COST_NS); }
  void delaySPI_4000() { DELAY_NS(4000 - CALLING_COST_NS); }

  void spiInit(uint8_t spiRate) {
    // Use datarates Marlin uses//使用马林使用的数据速率
    switch (spiRate) {
      case SPI_FULL_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 8,000,000  actual: ~1.1M//期望值：8000000实际值：~110万
      case SPI_HALF_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 4,000,000  actual: ~1.1M//期望值：4000000实际值：~110万
      case SPI_QUARTER_SPEED:delaySPIFunc =  &delaySPI_250; break;  // desired: 2,000,000  actual: ~890K//期望值：2000000实际值：~890K
      case SPI_EIGHTH_SPEED: delaySPIFunc =  &delaySPI_500; break;  // desired: 1,000,000  actual: ~590K//期望值：1000000实际值：约59000
      case SPI_SPEED_5:      delaySPIFunc = &delaySPI_1000; break;  // desired:   500,000  actual: ~360K//期望值：500000实际值：~360K
      case SPI_SPEED_6:      delaySPIFunc = &delaySPI_2000; break;  // desired:   250,000  actual: ~210K//期望值：250000实际值：~210K
      default:               delaySPIFunc = &delaySPI_4000; break;  // desired:   125,000  actual: ~123K//期望值：125000实际值：~123K
    }
    SPI.begin();
  }

  // Begin SPI transaction, set clock, bit order, data mode//开始SPI事务、设置时钟、位顺序、数据模式
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) { /* do nothing */ }

  uint8_t HAL_SPI_STM32_SpiTransfer_Mode_3(uint8_t b) { // using Mode 3//使用模式3
    for (uint8_t bits = 8; bits--;) {
      WRITE(SD_SCK_PIN, LOW);
      WRITE(SD_MOSI_PIN, b & 0x80);

      delaySPIFunc();
      WRITE(SD_SCK_PIN, HIGH);
      delaySPIFunc();

      b <<= 1;        // little setup time//设置时间短
      b |= (READ(SD_MISO_PIN) != 0);
    }
    DELAY_NS(125);
    return b;
  }

  // Soft SPI receive byte//软SPI接收字节
  uint8_t spiRec() {
    DISABLE_ISRS();                                               // No interrupts during byte receive//字节接收期间无中断
    const uint8_t data = HAL_SPI_STM32_SpiTransfer_Mode_3(0xFF);
    ENABLE_ISRS();                                                // Enable interrupts//启用中断
    return data;
  }

  // Soft SPI read data//软SPI读取数据
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }

  // Soft SPI send byte//软SPI发送字节
  void spiSend(uint8_t data) {
    DISABLE_ISRS();                         // No interrupts during byte send//字节发送期间没有中断
    HAL_SPI_STM32_SpiTransfer_Mode_3(data); // Don't care what is received//不管收到什么
    ENABLE_ISRS();                          // Enable interrupts//启用中断
  }

  // Soft SPI send block//软SPI发送块
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++)
      spiSend(buf[i]);
  }

#else

  // ------------------------// ------------------------
  // Hardware SPI//硬件SPI
  // ------------------------// ------------------------

  /**
   * VGPV SPI speed start and PCLK2/2, by default 108/2 = 54Mhz
   */

  /**
   * @brief  Begin SPI port setup
   *
   * @return Nothing
   *
   * @details Only configures SS pin since stm32duino creates and initialize the SPI object
   */
  void spiBegin() {
    #if PIN_EXISTS(SD_SS)
      OUT_WRITE(SD_SS_PIN, HIGH);
    #endif
  }

  // Configure SPI for specified SPI speed//为指定的SPI速度配置SPI
  void spiInit(uint8_t spiRate) {
    // Use datarates Marlin uses//使用马林使用的数据速率
    uint32_t clock;
    switch (spiRate) {
      case SPI_FULL_SPEED:    clock = 20000000; break; // 13.9mhz=20000000  6.75mhz=10000000  3.38mhz=5000000  .833mhz=1000000//13.9mhz=20000000 6.75mhz=10000000 3.38mhz=5000000.833mhz=1000000
      case SPI_HALF_SPEED:    clock =  5000000; break;
      case SPI_QUARTER_SPEED: clock =  2500000; break;
      case SPI_EIGHTH_SPEED:  clock =  1250000; break;
      case SPI_SPEED_5:       clock =   625000; break;
      case SPI_SPEED_6:       clock =   300000; break;
      default:
        clock = 4000000; // Default from the SPI library//SPI库中的默认值
    }
    spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);

    SPI.setMISO(SD_MISO_PIN);
    SPI.setMOSI(SD_MOSI_PIN);
    SPI.setSCLK(SD_SCK_PIN);

    SPI.begin();
  }

  /**
   * @brief  Receives a single byte from the SPI port.
   *
   * @return Byte received
   *
   * @details
   */
  uint8_t spiRec() {
    uint8_t returnByte = SPI.transfer(0xFF);
    return returnByte;
  }

  /**
   * @brief  Receive a number of bytes from the SPI port to a buffer
   *
   * @param  buf   Pointer to starting address of buffer to write to.
   * @param  nbyte Number of bytes to receive.
   * @return Nothing
   *
   * @details Uses DMA
   */
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    memset(buf, 0xFF, nbyte);
    SPI.transfer(buf, nbyte);
  }

  /**
   * @brief  Send a single byte on SPI port
   *
   * @param  b Byte to send
   *
   * @details
   */
  void spiSend(uint8_t b) {
    SPI.transfer(b);
  }

  /**
   * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
   *
   * @param  buf   Pointer with buffer start address
   * @return Nothing
   *
   * @details Use DMA
   */
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    uint8_t rxBuf[512];
    SPI.transfer(token);
    SPI.transfer((uint8_t*)buf, &rxBuf, 512);
  }

#endif // SOFTWARE_SPI//软件SPI

#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC//ARDUINO_ARCH_STM32&&！STM32通用
