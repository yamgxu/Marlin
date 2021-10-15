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

/**
 * Adapted from Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 */

/**
 * HAL for AVR - SPI functions
 */

#ifdef __AVR__

#include "../../inc/MarlinConfig.h"

void spiBegin() {
  OUT_WRITE(SD_SS_PIN, HIGH);
  SET_OUTPUT(SD_SCK_PIN);
  SET_INPUT(SD_MISO_PIN);
  SET_OUTPUT(SD_MOSI_PIN);

  #if DISABLED(SOFTWARE_SPI)
    // SS must be in output mode even it is not chip select//SS必须处于输出模式，即使不是芯片选择模式
    //SET_OUTPUT(SD_SS_PIN);//设置输出（SD\U SS\U引脚）；
    // set SS high - may be chip select for another SPI device//设置SS高-可能是另一个SPI设备的芯片选择
    //#if SET_SPI_SS_HIGH//#如果将SPI设置为高
      //WRITE(SD_SS_PIN, HIGH);//写入（SD_-SS_引脚，高电平）；
    //#endif//#恩迪夫
    // set a default rate//设定默认利率
    spiInit(1);
  #endif
}

#if NONE(SOFTWARE_SPI, FORCE_SOFT_SPI)

  // ------------------------// ------------------------
  // Hardware SPI//硬件SPI
  // ------------------------// ------------------------

  // make sure SPCR rate is in expected bits//确保SPCR速率处于预期位
  #if (SPR0 != 0 || SPR1 != 1)
    #error "unexpected SPCR bits"
  #endif

  /**
   * Initialize hardware SPI
   * Set SCK rate to F_CPU/pow(2, 1 + spiRate) for spiRate [0,6]
   */
  void spiInit(uint8_t spiRate) {
    // See avr processor documentation//请参阅avr处理器文档
    CBI(
      #ifdef PRR
        PRR
      #elif defined(PRR0)
        PRR0
      #endif
        , PRSPI);

    SPCR = _BV(SPE) | _BV(MSTR) | (spiRate >> 1);
    SPSR = spiRate & 1 || spiRate == 6 ? 0 : _BV(SPI2X);
  }

  /** SPI receive a byte */
  uint8_t spiRec() {
    SPDR = 0xFF;
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
    return SPDR;
  }

  /** SPI read data  */
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    if (nbyte-- == 0) return;
    SPDR = 0xFF;
    for (uint16_t i = 0; i < nbyte; i++) {
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      buf[i] = SPDR;
      SPDR = 0xFF;
    }
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
    buf[nbyte] = SPDR;
  }

  /** SPI send a byte */
  void spiSend(uint8_t b) {
    SPDR = b;
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
  }

  /** SPI send block  */
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    SPDR = token;
    for (uint16_t i = 0; i < 512; i += 2) {
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      SPDR = buf[i];
      while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
      SPDR = buf[i + 1];
    }
    while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
  }


  /** begin spi transaction */
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
    // Based on Arduino SPI library//基于Arduino SPI库
    // Clock settings are defined as follows. Note that this shows SPI2X//时钟设置定义如下。注意，这显示了SPI2X
    // inverted, so the bits form increasing numbers. Also note that//反转，所以位形成递增的数字。还要注意
    // fosc/64 appears twice//fosc/64出现两次
    // SPR1 SPR0 ~SPI2X Freq//SPR1 SPR0~SPI2X频率
    //   0    0     0   fosc/2//0 fosc/2
    //   0    0     1   fosc/4//0 0 1 fosc/4
    //   0    1     0   fosc/8//01 0 fosc/8
    //   0    1     1   fosc/16//01 1 fosc/16
    //   1    0     0   fosc/32//100FOSC/32
    //   1    0     1   fosc/64//101FOSC/64
    //   1    1     0   fosc/64//101FOSC/64
    //   1    1     1   fosc/128//1 fosc/128

    // We find the fastest clock that is less than or equal to the//我们发现最快的时钟小于或等于
    // given clock rate. The clock divider that results in clock_setting//给定时钟频率。导致时钟设置的时钟分频器
    // is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the//是2^^（时钟div+1）。如果速度不够慢，我们将使用
    // slowest (128 == 2 ^^ 7, so clock_div = 6).//最慢（128==2^^7，所以时钟div=6）。
    uint8_t clockDiv;

    // When the clock is known at compiletime, use this if-then-else//当时钟在编译时已知时，使用此if-then-else
    // cascade, which the compiler knows how to completely optimize//级联，编译器知道如何完全优化
    // away. When clock is not known, use a loop instead, which generates//走开。当时钟未知时，使用循环代替，它会生成
    // shorter code.//较短的代码。
    if (__builtin_constant_p(spiClock)) {
      if (spiClock >= F_CPU / 2)       clockDiv = 0;
      else if (spiClock >= F_CPU / 4)  clockDiv = 1;
      else if (spiClock >= F_CPU / 8)  clockDiv = 2;
      else if (spiClock >= F_CPU / 16) clockDiv = 3;
      else if (spiClock >= F_CPU / 32) clockDiv = 4;
      else if (spiClock >= F_CPU / 64) clockDiv = 5;
      else                             clockDiv = 6;
    }
    else {
      uint32_t clockSetting = F_CPU / 2;
      clockDiv = 0;
      while (clockDiv < 6 && spiClock < clockSetting) {
        clockSetting /= 2;
        clockDiv++;
      }
    }

    // Compensate for the duplicate fosc/64//补偿重复的fosc/64
    if (clockDiv == 6) clockDiv = 7;

    // Invert the SPI2X bit//反转SPI2X位
    clockDiv ^= 0x1;

    SPCR = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
      (dataMode << CPHA) | ((clockDiv >> 1) << SPR0);
    SPSR = clockDiv | 0x01;
  }


#else // SOFTWARE_SPI || FORCE_SOFT_SPI//软件| SPI |强制|软| SPI

  // ------------------------// ------------------------
  // Software SPI//软件SPI
  // ------------------------// ------------------------

  // nop to tune soft SPI timing//nop用于调整软SPI定时
  #define nop asm volatile ("\tnop\n")

  void spiInit(uint8_t) { /* do nothing */ }

  // Begin SPI transaction, set clock, bit order, data mode//开始SPI事务、设置时钟、位顺序、数据模式
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) { /* do nothing */ }

  // Soft SPI receive byte//软SPI接收字节
  uint8_t spiRec() {
    uint8_t data = 0;
    // no interrupts during byte receive - about 8µs//字节接收期间无中断-约8µs
    cli();
    // output pin high - like sending 0xFF//输出引脚高-类似发送0xFF
    WRITE(SD_MOSI_PIN, HIGH);

    LOOP_L_N(i, 8) {
      WRITE(SD_SCK_PIN, HIGH);

      nop; // adjust so SCK is nice//调整以使SCK很好
      nop;

      data <<= 1;

      if (READ(SD_MISO_PIN)) data |= 1;

      WRITE(SD_SCK_PIN, LOW);
    }

    sei();
    return data;
  }

  // Soft SPI read data//软SPI读取数据
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }

  // Soft SPI send byte//软SPI发送字节
  void spiSend(uint8_t data) {
    // no interrupts during byte send - about 8µs//字节发送期间无中断-约8µs
    cli();
    LOOP_L_N(i, 8) {
      WRITE(SD_SCK_PIN, LOW);
      WRITE(SD_MOSI_PIN, data & 0x80);
      data <<= 1;
      WRITE(SD_SCK_PIN, HIGH);
    }

    nop; // hold SCK high for a few ns//将SCK保持在高位几秒钟
    nop;
    nop;
    nop;

    WRITE(SD_SCK_PIN, LOW);

    sei();
  }

  // Soft SPI send block//软SPI发送块
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++)
      spiSend(buf[i]);
  }

#endif // SOFTWARE_SPI || FORCE_SOFT_SPI//软件| SPI |强制|软| SPI

#endif // __AVR__//_uuuavr__
