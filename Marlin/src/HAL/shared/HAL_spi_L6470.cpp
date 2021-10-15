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
 * Software L6470 SPI functions originally from Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 */

#include "../../inc/MarlinConfig.h"

#if HAS_L64XX

#include "Delay.h"

#include "../../core/serial.h"
#include "../../libs/L64XX/L64XX_Marlin.h"

// Make sure GCC optimizes this file.//确保GCC优化了这个文件。
// Note that this line triggers a bug in GCC which is fixed by casting.//注意，这一行触发了GCC中的一个bug，该bug通过强制转换修复。
// See the note below.//见下面的注释。
#pragma GCC optimize (3)

// run at ~4Mhz//以~4Mhz的频率运行
inline uint8_t L6470_SpiTransfer_Mode_0(uint8_t b) { // using Mode 0//使用模式0
  for (uint8_t bits = 8; bits--;) {
    WRITE(L6470_CHAIN_MOSI_PIN, b & 0x80);
    b <<= 1;        // little setup time//设置时间短

    WRITE(L6470_CHAIN_SCK_PIN, HIGH);
    DELAY_NS(125);  // 10 cycles @ 84mhz//84mhz下的10个周期

    b |= (READ(L6470_CHAIN_MISO_PIN) != 0);

    WRITE(L6470_CHAIN_SCK_PIN, LOW);
    DELAY_NS(125);  // 10 cycles @ 84mhz//84mhz下的10个周期
  }
  return b;
}

inline uint8_t L6470_SpiTransfer_Mode_3(uint8_t b) { // using Mode 3//使用模式3
  for (uint8_t bits = 8; bits--;) {
    WRITE(L6470_CHAIN_SCK_PIN, LOW);
    WRITE(L6470_CHAIN_MOSI_PIN, b & 0x80);

    DELAY_NS(125);  // 10 cycles @ 84mhz//84mhz下的10个周期
    WRITE(L6470_CHAIN_SCK_PIN, HIGH);
    DELAY_NS(125);  // Need more delay for fast CPUs//快速CPU需要更多延迟

    b <<= 1;        // little setup time//设置时间短
    b |= (READ(L6470_CHAIN_MISO_PIN) != 0);
  }
  DELAY_NS(125);    // 10 cycles @ 84mhz//84mhz下的10个周期
  return b;
}

/**
 * L64XX methods for SPI init and transfer
 */
void L64XX_Marlin::spi_init() {
  OUT_WRITE(L6470_CHAIN_SS_PIN, HIGH);
  OUT_WRITE(L6470_CHAIN_SCK_PIN, HIGH);
  OUT_WRITE(L6470_CHAIN_MOSI_PIN, HIGH);
  SET_INPUT(L6470_CHAIN_MISO_PIN);

  #if PIN_EXISTS(L6470_BUSY)
    SET_INPUT(L6470_BUSY_PIN);
  #endif

  OUT_WRITE(L6470_CHAIN_MOSI_PIN, HIGH);
}

uint8_t L64XX_Marlin::transfer_single(uint8_t data, int16_t ss_pin) {
  // First device in chain has data sent last//链中的第一个设备最后发送了数据
  extDigitalWrite(ss_pin, LOW);

  DISABLE_ISRS(); // Disable interrupts during SPI transfer (can't allow partial command to chips)//禁用SPI传输期间的中断（不允许向芯片发送部分命令）
  const uint8_t data_out = L6470_SpiTransfer_Mode_3(data);
  ENABLE_ISRS();  // Enable interrupts//启用中断

  extDigitalWrite(ss_pin, HIGH);
  return data_out;
}

uint8_t L64XX_Marlin::transfer_chain(uint8_t data, int16_t ss_pin, uint8_t chain_position) {
  uint8_t data_out = 0;

  // first device in chain has data sent last//链中的第一个设备最后发送了数据
  extDigitalWrite(ss_pin, LOW);

  for (uint8_t i = L64XX::chain[0]; !L64xxManager.spi_abort && i >= 1; i--) {   // Send data unless aborted//除非中止，否则发送数据
    DISABLE_ISRS();   // Disable interrupts during SPI transfer (can't allow partial command to chips)//禁用SPI传输期间的中断（不允许向芯片发送部分命令）
    const uint8_t temp = L6470_SpiTransfer_Mode_3(uint8_t(i == chain_position ? data : dSPIN_NOP));
    ENABLE_ISRS();    // Enable interrupts//启用中断
    if (i == chain_position) data_out = temp;
  }

  extDigitalWrite(ss_pin, HIGH);
  return data_out;
}

/**
 * Platform-supplied L6470 buffer transfer method
 */
void L64XX_Marlin::transfer(uint8_t L6470_buf[], const uint8_t length) {
  // First device in chain has its data sent last//链中的第一个设备的数据最后发送

  if (spi_active) {                   // Interrupted SPI transfer so need to//SPI传输中断，因此需要
    WRITE(L6470_CHAIN_SS_PIN, HIGH);  //  guarantee min high of 650ns//保证最低高达650ns
    DELAY_US(1);
  }

  WRITE(L6470_CHAIN_SS_PIN, LOW);
  for (uint8_t i = length; i >= 1; i--)
    L6470_SpiTransfer_Mode_3(uint8_t(L6470_buf[i]));
  WRITE(L6470_CHAIN_SS_PIN, HIGH);
}

#pragma GCC reset_options

#endif // HAS_L64XX//有"L64XX"吗?
