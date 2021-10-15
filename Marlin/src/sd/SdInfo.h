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
 * Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include <stdint.h>

// Based on the document://根据文件：
////
// SD Specifications//SD规格
// Part 1//第一部分
// Physical Layer//物理层
// Simplified Specification//简化规格
// Version 3.01//版本3.01
// May 18, 2010//2010年5月18日
////
// https://www.sdcard.org/downloads/pls/index.html// https://www.sdcard.org/downloads/pls/index.html

// SD card commands//SD卡命令
uint8_t const CMD0 = 0x00,    // GO_IDLE_STATE - init card in spi mode if CS low//进入空闲状态-如果CS低，则初始化卡处于spi模式
              CMD8 = 0x08,    // SEND_IF_COND - verify SD Memory Card interface operating condition//发送条件-验证SD存储卡接口操作条件
              CMD9 = 0x09,    // SEND_CSD - read the Card Specific Data (CSD register)//发送\u CSD-读取卡特定数据（CSD寄存器）
              CMD10 = 0x0A,   // SEND_CID - read the card identification information (CID register)//发送CID-读取卡标识信息（CID寄存器）
              CMD12 = 0x0C,   // STOP_TRANSMISSION - end multiple block read sequence//停止传输-结束多块读取序列
              CMD13 = 0x0D,   // SEND_STATUS - read the card status register//发送\ U状态-读取卡状态寄存器
              CMD17 = 0x11,   // READ_SINGLE_BLOCK - read a single data block from the card//读取单个数据块-从卡中读取单个数据块
              CMD18 = 0x12,   // READ_MULTIPLE_BLOCK - read a multiple data blocks from the card//读取多个数据块-从卡中读取多个数据块
              CMD24 = 0x18,   // WRITE_BLOCK - write a single data block to the card//写入块-将单个数据块写入卡
              CMD25 = 0x19,   // WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION//写入多块-写入数据块直到停止传输
              CMD32 = 0x20,   // ERASE_WR_BLK_START - sets the address of the first block to be erased//ERASE_WR_BLK_START-设置要擦除的第一个块的地址
              CMD33 = 0x21,   // ERASE_WR_BLK_END - sets the address of the last block of the continuous range to be erased//ERASE_WR_BLK_END-设置要擦除的连续范围的最后一个块的地址
              CMD38 = 0x26,   // ERASE - erase all previously selected blocks//擦除-擦除所有以前选定的块
              CMD55 = 0x37,   // APP_CMD - escape for application specific command//APP_CMD-应用程序特定命令的转义
              CMD58 = 0x3A,   // READ_OCR - read the OCR register of a card//读取OCR-读取卡的OCR寄存器
              CMD59 = 0x3B,   // CRC_ON_OFF - enable or disable CRC checking//CRC_ON_OFF-启用或禁用CRC检查
              ACMD23 = 0x17,  // SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be pre-erased before writing//SET_WR_BLK_ERASE_COUNT-设置写入前要预擦除的写入块数
              ACMD41 = 0x29;  // SD_SEND_OP_COMD - Sends host capacity support information and activates the card's initialization process//SD_SEND_OP_COMD-发送主机容量支持信息并激活卡的初始化过程

/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0x00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0x01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0x04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0xFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0xFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0xFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0x1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0x05;

/** Card IDentification (CID) register */
typedef struct CID {
  // byte 0//字节0
  /** Manufacturer ID */
  unsigned char mid;
  // byte 1-2//字节1-2
  /** OEM/Application ID */
  char oid[2];
  // byte 3-7//字节3-7
  /** Product name */
  char pnm[5];
  // byte 8//字节8
  /** Product revision least significant digit */
  unsigned char prv_m : 4;
  /** Product revision most significant digit */
  unsigned char prv_n : 4;
  // byte 9-12//字节9-12
  /** Product serial number */
  uint32_t psn;
  // byte 13//字节13
  /** Manufacturing date year low digit */
  unsigned char mdt_year_high : 4;
  /** not used */
  unsigned char reserved : 4;
  // byte 14//字节14
  /** Manufacturing date month */
  unsigned char mdt_month : 4;
  /** Manufacturing date year low digit */
  unsigned char mdt_year_low : 4;
  // byte 15//字节15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** CRC7 checksum */
  unsigned char crc : 7;
} cid_t;

/** CSD for version 1.00 cards */
typedef struct CSDV1 {
  // byte 0//字节0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1//字节1
  unsigned char taac;
  // byte 2//字节2
  unsigned char nsac;
  // byte 3//字节3
  unsigned char tran_speed;
  // byte 4//字节4
  unsigned char ccc_high;
  // byte 5//字节5
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6//字节6
  unsigned char c_size_high : 2;
  unsigned char reserved2 : 2;
  unsigned char dsr_imp : 1;
  unsigned char read_blk_misalign : 1;
  unsigned char write_blk_misalign : 1;
  unsigned char read_bl_partial : 1;
  // byte 7//字节7
  unsigned char c_size_mid;
  // byte 8//字节8
  unsigned char vdd_r_curr_max : 3;
  unsigned char vdd_r_curr_min : 3;
  unsigned char c_size_low : 2;
  // byte 9//字节9
  unsigned char c_size_mult_high : 2;
  unsigned char vdd_w_cur_max : 3;
  unsigned char vdd_w_curr_min : 3;
  // byte 10//字节10
  unsigned char sector_size_high : 6;
  unsigned char erase_blk_en : 1;
  unsigned char c_size_mult_low : 1;
  // byte 11//字节11
  unsigned char wp_grp_size : 7;
  unsigned char sector_size_low : 1;
  // byte 12//字节12
  unsigned char write_bl_len_high : 2;
  unsigned char r2w_factor : 3;
  unsigned char reserved3 : 2;
  unsigned char wp_grp_enable : 1;
  // byte 13//字节13
  unsigned char reserved4 : 5;
  unsigned char write_partial : 1;
  unsigned char write_bl_len_low : 2;
  // byte 14//字节14
  unsigned char reserved5: 2;
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Indicates the file format on the card */
  unsigned char file_format_grp : 1;
  // byte 15//字节15
  unsigned char always1 : 1;
  unsigned char crc : 7;
} csd1_t;

/** CSD for version 2.00 cards */
typedef struct CSDV2 {
  // byte 0//字节0
  unsigned char reserved1 : 6;
  unsigned char csd_ver : 2;
  // byte 1//字节1
  /** fixed to 0x0E */
  unsigned char taac;
  // byte 2//字节2
  /** fixed to 0 */
  unsigned char nsac;
  // byte 3//字节3
  unsigned char tran_speed;
  // byte 4//字节4
  unsigned char ccc_high;
  // byte 5//字节5
  /** This field is fixed to 9h, which indicates READ_BL_LEN=512 Byte */
  unsigned char read_bl_len : 4;
  unsigned char ccc_low : 4;
  // byte 6//字节6
  /** not used */
  unsigned char reserved2 : 4;
  unsigned char dsr_imp : 1;
  /** fixed to 0 */
  unsigned char read_blk_misalign : 1;
  /** fixed to 0 */
  unsigned char write_blk_misalign : 1;
  /** fixed to 0 - no partial read */
  unsigned char read_bl_partial : 1;
  // byte 7//字节7
  /** not used */
  unsigned char reserved3 : 2;
  /** high part of card size */
  unsigned char c_size_high : 6;
  // byte 8//字节8
  /** middle part of card size */
  unsigned char c_size_mid;
  // byte 9//字节9
  /** low part of card size */
  unsigned char c_size_low;
  // byte 10//字节10
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_high : 6;
  /** fixed to 1 - erase single is supported */
  unsigned char erase_blk_en : 1;
  /** not used */
  unsigned char reserved4 : 1;
  // byte 11//字节11
  unsigned char wp_grp_size : 7;
  /** sector size is fixed at 64 KB */
  unsigned char sector_size_low : 1;
  // byte 12//字节12
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_high : 2;
  /** fixed value of 2 */
  unsigned char r2w_factor : 3;
  /** not used */
  unsigned char reserved5 : 2;
  /** fixed value of 0 - no write protect groups */
  unsigned char wp_grp_enable : 1;
  // byte 13//字节13
  unsigned char reserved6 : 5;
  /** always zero - no partial block read*/
  unsigned char write_partial : 1;
  /** write_bl_len fixed for 512 byte blocks */
  unsigned char write_bl_len_low : 2;
  // byte 14//字节14
  unsigned char reserved7: 2;
  /** Do not use always 0 */
  unsigned char file_format : 2;
  unsigned char tmp_write_protect : 1;
  unsigned char perm_write_protect : 1;
  unsigned char copy : 1;
  /** Do not use always 0 */
  unsigned char file_format_grp : 1;
  // byte 15//字节15
  /** not used always 1 */
  unsigned char always1 : 1;
  /** checksum */
  unsigned char crc : 7;
} csd2_t;

/** union of old and new style CSD register */
union csd_t {
  csd1_t v1;
  csd2_t v2;
};
