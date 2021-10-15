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
#pragma once

/**
 * HAL/shared/HAL_SPI.h
 * Core Marlin definitions for SPI, implemented in the HALs
 */

#include "Marduino.h"
#include <stdint.h>

/**
 * SPI speed where 0 <= index <= 6
 *
 * Approximate rates :
 *
 *  0 :  8 - 10 MHz
 *  1 :  4 - 5 MHz
 *  2 :  2 - 2.5 MHz
 *  3 :  1 - 1.25 MHz
 *  4 :  500 - 625 kHz
 *  5 :  250 - 312 kHz
 *  6 :  125 - 156 kHz
 *
 *  On AVR, actual speed is F_CPU/2^(1 + index).
 *  On other platforms, speed should be in range given above where possible.
 */

#define SPI_FULL_SPEED      0   // Set SCK to max rate//将SCK设置为最大速率
#define SPI_HALF_SPEED      1   // Set SCK rate to half of max rate//将SCK速率设置为最大速率的一半
#define SPI_QUARTER_SPEED   2   // Set SCK rate to quarter of max rate//将SCK速率设置为最大速率的四分之一
#define SPI_EIGHTH_SPEED    3   // Set SCK rate to 1/8 of max rate//将SCK速率设置为最大速率的1/8
#define SPI_SIXTEENTH_SPEED 4   // Set SCK rate to 1/16 of max rate//将SCK速率设置为最大速率的1/16
#define SPI_SPEED_5         5   // Set SCK rate to 1/32 of max rate//将SCK速率设置为最大速率的1/32
#define SPI_SPEED_6         6   // Set SCK rate to 1/64 of max rate//将SCK速率设置为最大速率的1/64

////
// Standard SPI functions//标准SPI函数
////

// Initialize SPI bus//初始化SPI总线
void spiBegin();

// Configure SPI for specified SPI speed//为指定的SPI速度配置SPI
void spiInit(uint8_t spiRate);

// Write single byte to SPI//将单字节写入SPI
void spiSend(uint8_t b);

// Read single byte from SPI//从SPI读取单字节
uint8_t spiRec();

// Read from SPI into buffer//从SPI读入缓冲区
void spiRead(uint8_t *buf, uint16_t nbyte);

// Write token and then write from 512 byte buffer to SPI (for SD card)//写入令牌，然后从512字节缓冲区写入SPI（对于SD卡）
void spiSendBlock(uint8_t token, const uint8_t *buf);

// Begin SPI transaction, set clock, bit order, data mode//开始SPI事务、设置时钟、位顺序、数据模式
void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode);

////
// Extended SPI functions taking a channel number (Hardware SPI only)//采用通道号的扩展SPI函数（仅限硬件SPI）
////

// Write single byte to specified SPI channel//将单个字节写入指定的SPI通道
void spiSend(uint32_t chan, byte b);

// Write buffer to specified SPI channel//将缓冲区写入指定的SPI通道
void spiSend(uint32_t chan, const uint8_t *buf, size_t n);

// Read single byte from specified SPI channel//从指定的SPI通道读取单个字节
uint8_t spiRec(uint32_t chan);
