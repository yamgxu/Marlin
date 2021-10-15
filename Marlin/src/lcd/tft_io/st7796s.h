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

#include "tft_io.h"

#include "../../inc/MarlinConfig.h"

#define ST7796S_MADCTL_MY  0x80 // Row Address Order//行地址顺序
#define ST7796S_MADCTL_MX  0x40 // Column Address Order//列地址顺序
#define ST7796S_MADCTL_MV  0x20 // Row/Column Exchange//行/列交换
#define ST7796S_MADCTL_ML  0x10 // Vertical Refresh Order//垂直刷新顺序
#define ST7796S_MADCTL_BGR 0x08 // RGB-BGR ORDER//RGB-BGR指令
#define ST7796S_MADCTL_RGB 0x00
#define ST7796S_MADCTL_MH  0x04 // Horizontal Refresh Order//水平刷新顺序

#define ST7796S_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, ST7796S_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    ST7796S_MADCTL_MX) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    ST7796S_MADCTL_MY)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_BGR
  #define ST7796S_COLOR ST7796S_MADCTL_BGR
#elif TFT_COLOR == TFT_COLOR_RGB
  #define ST7796S_COLOR ST7796S_MADCTL_RGB
#endif

#define ST7796S_MADCTL_DATA       (ST7796S_ORIENTATION) | (ST7796S_COLOR)

#define ST7796S_NOP        0x00 // No Operation//无操作
#define ST7796S_SWRESET    0x01 // Software reset//软件重置
#define ST7796S_RDDID      0x04 // Read Display ID//读取显示ID
#define ST7796S_RDNUMED    0x05 // Read Number of the Errors on DSI//读取DSI上的错误数
#define ST7796S_RDDST      0x09 // Read Display Status//读取显示状态
#define ST7796S_RDDPM      0x0A // Read Display Power Mode//读取显示电源模式
#define ST7796S_RDDMADCTL  0x0B // Read Display MADCTL//读取显示MADCTL
#define ST7796S_RDDCOLMOD  0x0C // Read Display Pixel Format//读取显示像素格式
#define ST7796S_RDDIM      0x0D // Read Display Image Mode//读取显示图像模式
#define ST7796S_RDDSM      0x0E // Read Display Signal Status//读取显示信号状态
#define ST7796S_RDDSDR     0x0F // Read Display Self-Diagnostic Result//读取并显示自诊断结果
#define ST7796S_SLPIN      0x10 // Sleep In//睡懒觉
#define ST7796S_SLPOUT     0x11 // Sleep Out//露宿
#define ST7796S_PTLON      0x12 // Partial Display Mode On//部分显示模式打开
#define ST7796S_NORON      0x13 // Normal Display Mode On//正常显示模式打开
#define ST7796S_INVOFF     0x20 // Display Inversion Off//显示反转关闭
#define ST7796S_INVON      0x21 // Display Inversion On//显示反转
#define ST7796S_DISPOFF    0x28 // Display Off//炫耀
#define ST7796S_DISPON     0x29 // Display On//展示
#define ST7796S_CASET      0x2A // Column Address Set//列地址集
#define ST7796S_RASET      0x2B // Row Address Set//行地址集
#define ST7796S_RAMWR      0x2C // Memory Write//内存写入
#define ST7796S_RAMRD      0x2E // Memory Read//内存读取
#define ST7796S_PTLAR      0x30 // Partial Area//局部区域
#define ST7796S_VSCRDEF    0x33 // Vertical Scrolling Definition//垂直滚动定义
#define ST7796S_TEOFF      0x34 // Tearing Effect Line OFF//撕裂效应线
#define ST7796S_TEON       0x35 // Tearing Effect Line On//线撕裂效应
#define ST7796S_MADCTL     0x36 // Memory Data Access Control//存储器数据访问控制
#define ST7796S_VSCSAD     0x37 // Vertical Scroll Start Address of RAM//RAM的垂直滚动起始地址
#define ST7796S_IDMOFF     0x38 // Idle Mode Off//怠速模式关闭
#define ST7796S_IDMON      0x39 // Idle Mode On//空闲模式打开
#define ST7796S_COLMOD     0x3A // Interface Pixel Format//接口像素格式
#define ST7796S_WRMEMC     0x3C // Write Memory Continue//写入内存继续
#define ST7796S_RDMEMC     0x3E // Read Memory Continue//读存储器继续
#define ST7796S_STE        0x44 // Set Tear ScanLine//设置撕裂扫描线
#define ST7796S_GSCAN      0x45 // Get ScanLine//获取扫描线
#define ST7796S_WRDISBV    0x51 // Write Display Brightness//写入显示亮度
#define ST7796S_RDDISBV    0x52 // Read Display Brightness Value//读取显示亮度值
#define ST7796S_WRCTRLD    0x53 // Write CTRL Display//写入CTRL显示
#define ST7796S_RDCTRLD    0x54 // Read CTRL value Display//读取CTRL值显示
#define ST7796S_WRCABC     0x55 // Write Adaptive Brightness Control//写自适应亮度控制
#define ST7796S_RDCABC     0x56 // Read Content Adaptive Brightness Control//读取内容自适应亮度控制
#define ST7796S_WRCABCMB   0x5E // Write CABC Minimum Brightness//写CABC最小亮度
#define ST7796S_RDCABCMB   0x5F // Read CABC Minimum Brightness//读取CABC最小亮度
#define ST7796S_RDFCS      0xAA // Read First Checksum//读取第一校验和
#define ST7796S_RDCFCS     0xAF // Read Continue Checksum//读继续校验和
#define ST7796S_RDID1      0xDA // Read ID1//读ID1
#define ST7796S_RDID2      0xDB // Read ID2//读ID2
#define ST7796S_RDID3      0xDC // Read ID3//读ID3

#define ST7796S_IFMODE     0xB0 // Interface Mode Control//接口模式控制
#define ST7796S_FRMCTR1    0xB1 // Frame Rate Control (In Normal Mode/Full Colors)//帧速率控制（在正常模式/全彩模式下）
#define ST7796S_FRMCTR2    0xB2 // Frame Rate Control 2 (In Idle Mode/8 colors)//帧速率控制2（空闲模式/8色）
#define ST7796S_FRMCTR3    0xB3 // Frame Rate Control 3(In Partial Mode/Full Colors)//帧速率控制3（部分模式/全色）
#define ST7796S_DIC        0xB4 // Display Inversion Control//显示反转控制
#define ST7796S_BPC        0xB5 // Blanking Porch Control//下料门廊控制
#define ST7796S_DFC        0xB6 // Display Function Control//显示功能控制
#define ST7796S_EM         0xB7 // Entry Mode Set//进入模式集
#define ST7796S_PWR1       0xC0 // Power Control 1//电源控制1
#define ST7796S_PWR2       0xC1 // Power Control 2//电源控制2
#define ST7796S_PWR3       0xC2 // Power Control 3//电源控制3
#define ST7796S_VCMPCTL    0xC5 // VCOM Control//VCOM控制
#define ST7796S_VCMOST     0xC6 // VCOM Offset Register//VCOM偏移寄存器
#define ST7796S_NVMADW     0xD0 // NVM Address/Data Write//NVM地址/数据写入
#define ST7796S_NVMBPROG   0xD1 // NVM Byte Program//字节程序
#define ST7796S_NVMSTRD    0xD2 // NVM Status Read//NVM状态读取
#define ST7796S_RDID4      0xD3 // Read ID4//读ID4
#define ST7796S_PGC        0xE0 // Positive Gamma Control//正γ对照
#define ST7796S_NGC        0xE1 // Negative Gamma Control//负伽马控制
#define ST7796S_DGC1       0xE2 // Digital Gamma Control 1//数字伽马控制1
#define ST7796S_DGC2       0xE3 // Digital Gamma Control 2//数字伽马控制2
#define ST7796S_DOCA       0xE8 // Display Output Ctrl Adjust//显示输出Ctrl-Adjust
#define ST7796S_CSCON      0xF0 // Command Set Control//命令集控制
#define ST7796S_SPIRC      0xFB // SPI Read Control//SPI读控制

static const uint16_t st7796s_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ST7796S_SWRESET), ESC_DELAY(100),
  ESC_REG(ST7796S_SLPOUT), ESC_DELAY(20),

  ESC_REG(ST7796S_CSCON), 0x00C3,  // enable command 2 part I//启用命令2第I部分
  ESC_REG(ST7796S_CSCON), 0x0096,  // enable command 2 part II//启用命令2第二部分

  ESC_REG(ST7796S_MADCTL), ST7796S_MADCTL_DATA,
  ESC_REG(ST7796S_COLMOD), 0x0055,

  ESC_REG(ST7796S_DIC), 0x0001,  // 1-dot inversion//单点反演
  ESC_REG(ST7796S_EM), 0x00C6,

  ESC_REG(ST7796S_PWR2), 0x0015,
  ESC_REG(ST7796S_PWR3), 0x00AF,
  ESC_REG(ST7796S_VCMPCTL), 0x0022,
  ESC_REG(ST7796S_VCMOST), 0x0000,
  ESC_REG(ST7796S_DOCA), 0x0040, 0x008A, 0x0000, 0x0000, 0x0029, 0x0019, 0x00A5, 0x0033,

  /* Gamma Correction. */
  ESC_REG(ST7796S_PGC), 0x00F0, 0x0004, 0x0008, 0x0009, 0x0008, 0x0015, 0x002F, 0x0042, 0x0046, 0x0028, 0x0015, 0x0016, 0x0029, 0x002D,
  ESC_REG(ST7796S_NGC), 0x00F0, 0x0004, 0x0009, 0x0009, 0x0008, 0x0015, 0x002E, 0x0046, 0x0046, 0x0028, 0x0015, 0x0015, 0x0029, 0x002D,

  ESC_REG(ST7796S_NORON),
  ESC_REG(ST7796S_WRCTRLD), 0x0024,
  ESC_REG(ST7796S_CSCON), 0x003C,  // disable command 2 part I//禁用命令2第一部分
  ESC_REG(ST7796S_CSCON), 0x0069,  // disable command 2 part II//禁用命令2第二部分
  ESC_REG(ST7796S_DISPON),
  ESC_END
};

static const uint16_t lerdge_st7796s_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ST7796S_SWRESET), ESC_DELAY(100),
  ESC_REG(ST7796S_SLPOUT), ESC_DELAY(20),

  ESC_REG(ST7796S_CSCON), 0x00C3,  // enable command 2 part I//启用命令2第I部分
  ESC_REG(ST7796S_CSCON), 0x0096,  // enable command 2 part II//启用命令2第二部分

  ESC_REG(ST7796S_MADCTL), ST7796S_MADCTL_DATA,
  ESC_REG(ST7796S_COLMOD), 0x0055,

  ESC_REG(ST7796S_DIC), 0x0001,  // 1-dot inversion//单点反演
  ESC_REG(ST7796S_EM), 0x00C6,

  ESC_REG(ST7796S_PWR2), 0x0015,
  ESC_REG(ST7796S_PWR3), 0x00AF,
  ESC_REG(ST7796S_VCMPCTL), 0x0022,
  ESC_REG(ST7796S_VCMOST), 0x0000,
  ESC_REG(ST7796S_DOCA), 0x0040, 0x008A, 0x0000, 0x0000, 0x0029, 0x0019, 0x00A5, 0x0033,

  /* Gamma Correction. */
  ESC_REG(ST7796S_PGC), 0x00F0, 0x0004, 0x0008, 0x0009, 0x0008, 0x0015, 0x002F, 0x0042, 0x0046, 0x0028, 0x0015, 0x0016, 0x0029, 0x002D,
  ESC_REG(ST7796S_NGC), 0x00F0, 0x0004, 0x0009, 0x0009, 0x0008, 0x0015, 0x002E, 0x0046, 0x0046, 0x0028, 0x0015, 0x0015, 0x0029, 0x002D,

  ESC_REG(ST7796S_INVON),      // Display inversion ON//显示反转
  ESC_REG(ST7796S_WRCTRLD), 0x0024,
  ESC_REG(ST7796S_CSCON), 0x003C,  // disable command 2 part I//禁用命令2第一部分
  ESC_REG(ST7796S_CSCON), 0x0069,  // disable command 2 part II//禁用命令2第二部分
  ESC_REG(ST7796S_DISPON),
  ESC_END
};
