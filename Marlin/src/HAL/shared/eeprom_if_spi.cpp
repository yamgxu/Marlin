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
 * Platform-independent Arduino functions for SPI EEPROM.
 * Enable USE_SHARED_EEPROM if not supplied by the framework.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SPI_EEPROM)

#include "eeprom_if.h"

void eeprom_init() {}

#if ENABLED(USE_SHARED_EEPROM)

#define CMD_WREN  6   // WREN//鹪鹩
#define CMD_READ  2   // WRITE//写
#define CMD_WRITE 2   // WRITE//写

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY    7
#endif

static void _eeprom_begin(uint8_t * const pos, const uint8_t cmd) {
  const uint8_t eeprom_temp[3] = {
    cmd,
    (unsigned(pos) >> 8) & 0xFF,  // Address High//地址高
     unsigned(pos)       & 0xFF   // Address Low//地址低
  };
  WRITE(SPI_EEPROM1_CS, HIGH);    // Usually free already//通常已经免费了
  WRITE(SPI_EEPROM1_CS, LOW);     // Activate the Bus//启动巴士
  spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);
                                  // Leave the Bus in-use//让公共汽车开着
}

uint8_t eeprom_read_byte(uint8_t *pos) {
  _eeprom_begin(pos, CMD_READ);   // Set read location and begin transmission//设置读取位置并开始传输

  const uint8_t v = spiRec(SPI_CHAN_EEPROM1); // After READ a value sits on the Bus//读取后，一个值位于总线上

  WRITE(SPI_EEPROM1_CS, HIGH);    // Done with device//使用设备完成

  return v;
}

void eeprom_write_byte(uint8_t *pos, uint8_t value) {
  const uint8_t eeprom_temp = CMD_WREN;
  WRITE(SPI_EEPROM1_CS, LOW);
  spiSend(SPI_CHAN_EEPROM1, &eeprom_temp, 1); // Write Enable//写启用

  WRITE(SPI_EEPROM1_CS, HIGH);      // Done with the Bus//车停了
  delay(1);                         // For a small amount of time//在一小段时间内

  _eeprom_begin(pos, CMD_WRITE);    // Set write address and begin transmission//设置写入地址并开始传输

  spiSend(SPI_CHAN_EEPROM1, value); // Send the value to be written//发送要写入的值
  WRITE(SPI_EEPROM1_CS, HIGH);      // Done with the Bus//车停了
  delay(EEPROM_WRITE_DELAY);        // Give page write time to complete//给页面写时间来完成
}

#endif // USE_SHARED_EEPROM//使用共享EEPROM
#endif // I2C_EEPROM//I2C_EEPROM
