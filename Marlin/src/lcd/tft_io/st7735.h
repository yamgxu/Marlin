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

#define ST7735_MADCTL_MY          0x80 // Row Address Order//行地址顺序
#define ST7735_MADCTL_MX          0x40 // Column Address Order//列地址顺序
#define ST7735_MADCTL_MV          0x20 // Row/Column Exchange//行/列交换
#define ST7735_MADCTL_ML          0x10 // Vertical Refresh Order//垂直刷新顺序
#define ST7735_MADCTL_BGR         0x08 // RGB-BGR ORDER//RGB-BGR指令
#define ST7735_MADCTL_RGB         0x00
#define ST7735_MADCTL_MH          0x04 // Horizontal Refresh Order//水平刷新顺序

#define ST7735_ORIENTATION_UP     0x00                                // 128x160 ; Cable on the upper side//128x160；电缆在上面
#define ST7735_ORIENTATION_RIGHT  ST7735_MADCTL_MV | ST7735_MADCTL_MY // 160x128 ; Cable on the right side//160x128；右侧的电缆
#define ST7735_ORIENTATION_LEFT   ST7735_MADCTL_MV | ST7735_MADCTL_MX // 160x128 ; Cable on the left side//160x128；左侧的电缆
#define ST7735_ORIENTATION_DOWN   ST7735_MADCTL_MX | ST7735_MADCTL_MY // 128x160 ; Cable on the lower side//128x160；下侧的电缆

#define ST7735_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, ST7735_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    ST7735_MADCTL_MX) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    ST7735_MADCTL_MY)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_RGB
  #define ST7735_COLOR ST7735_MADCTL_RGB
#elif TFT_COLOR == TFT_COLOR_BGR
  #define ST7735_COLOR ST7735_MADCTL_BGR
#endif

#define ST7735_MADCTL_DATA       (ST7735_ORIENTATION) | (ST7735_COLOR)

#define ST7735_NOP                0x00 // No Operation//无操作
#define ST7735_SWRESET            0x01 // Software reset//软件重置
#define ST7735_RDDID              0x04 // Read Display ID//读取显示ID
#define ST7735_RDDST              0x09 // Read Display Status//读取显示状态
#define ST7735_RDDPM              0x0A // Read Display Power Mode//读取显示电源模式
#define ST7735_RDDMADCTL          0x0B // Read Display MADCTL//读取显示MADCTL
#define ST7735_RDDCOLMOD          0x0C // Read Display Pixel Format//读取显示像素格式
#define ST7735_RDDIM              0x0D // Read Display Image Mode//读取显示图像模式
#define ST7735_RDDSM              0x0E // Read Display Signal Mode//读取显示信号模式
#define ST7735_SLPIN              0x10 // Sleep In//睡懒觉
#define ST7735_SLPOUT             0x11 // Sleep Out//露宿
#define ST7735_PTLON              0x12 // Partial Display Mode On//部分显示模式打开
#define ST7735_NORON              0x13 // Normal Display Mode On//正常显示模式打开
#define ST7735_INVOFF             0x20 // Display Inversion Off//显示反转关闭
#define ST7735_INVON              0x21 // Display Inversion On//显示反转
#define ST7735_GAMSET             0x26 // Gamma Set//伽马集
#define ST7735_DISPOFF            0x28 // Display Off // The delay time between DISPON and DISPOFF needs 120ms at least.//显示关闭//DISPON和DISPOFF之间的延迟时间至少需要120ms。
#define ST7735_DISPON             0x29 // Display On//展示
#define ST7735_CASET              0x2A // Column Address Set//列地址集
#define ST7735_RASET              0x2B // Row Address Set//行地址集
#define ST7735_RAMWR              0x2C // Memory Write//内存写入
#define ST7735_RAMRD              0x2E // Memory Read//内存读取
#define ST7735_PTLAR              0x30 // Partial Area//局部区域
#define ST7735_TEOFF              0x34 // Tearing Effect Line OFF//撕裂效应线
#define ST7735_TEON               0x35 // Tearing Effect Line ON//线撕裂效应
#define ST7735_MADCTL             0x36 // Memory Data Access Control//存储器数据访问控制
#define ST7735_IDMOFF             0x38 // Idle Mode Off//怠速模式关闭
#define ST7735_IDMON              0x39 // Idle Mode On//空闲模式打开
#define ST7735_COLMOD             0x3A // Interface Pixel Format//接口像素格式
#define ST7735_RDID1              0xDA // Read ID1 Value//读取ID1值
#define ST7735_RDID2              0xDB // Read ID2 Value//读取ID2值
#define ST7735_RDID3              0xDC // Read ID3 Value//读取ID3值

#define ST7735_FRMCTR1            0xB1 // Frame Rate Control (In normal mode/ Full colors)//帧速率控制（在正常模式/全彩模式下）
#define ST7735_FRMCTR2            0xB2 // Frame Rate Control (In Idle mode/ 8-colors)//帧速率控制（空闲模式/8色）
#define ST7735_FRMCTR3            0xB3 // Frame Rate Control (In Partial mode/ full colors)//帧速率控制（部分模式/全色）
#define ST7735_INVCTR             0xB4 // Display Inversion Control//显示反转控制
#define ST7735_DISSET5            0xB6 // Display Function set 5//显示功能集5
#define ST7735_PWCTR1             0xC0 // Power Control 1//电源控制1
#define ST7735_PWCTR2             0xC1 // Power Control 2//电源控制2
#define ST7735_PWCTR3             0xC2 // Power Control 3 (in Normal mode/ Full colors)//电源控制3（正常模式/全彩）
#define ST7735_PWCTR4             0xC3 // Power Control 4 (in Idle mode/ 8-colors)//电源控制4（空闲模式/8色）
#define ST7735_PWCTR5             0xC4 // Power Control 5 (in Partial mode/ full-colors)//电源控制5（部分模式/全色）
#define ST7735_VMCTR1             0xC5 // VCOM Control 1//VCOM控制1
#define ST7735_VMOFCTR            0xC7 // VCOM Offset Control//VCOM偏移控制
#define ST7735_WRID2              0xD1 // Write ID2 Value//写入ID2值
#define ST7735_WRID3              0xD2 // Write ID3 Value//写入ID3值
#define ST7735_PWCTR6             0xFC // Power Control 5 (in Partial mode + Idle mode)//功率控制5（部分模式+怠速模式）
#define ST7735_NVFCTR1            0xD9 // EEPROM Control Status//EEPROM控制状态
#define ST7735_NVFCTR2            0xDE // EEPROM Read Command//EEPROM读取命令
#define ST7735_NVFCTR3            0xDF // EEPROM Write Command//EEPROM写入命令
#define ST7735_GMCTRP1            0xE0 // Gamma (‘+’polarity) Correction Characteristics Setting//伽马（“+”极性）校正特性设置
#define ST7735_GMCTRN1            0xE1 // GMCTRN1 (E1h): Gamma ‘-’polarity Correction Characteristics Setting//GMCTRN1（E1h）：伽马极性校正特性设置
#define ST7735_EXTCTRL            0xF0 // Extension Command Control//扩展命令控制
#define ST7735_VCOM4L             0xFF // Vcom 4 Level Control//Vcom 4电平控制

static const uint16_t st7735_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ST7735_SWRESET), ESC_DELAY(100),
  ESC_REG(ST7735_SLPOUT),  ESC_DELAY(20),
/*
  ESC_REG(ST7735_FRMCTR1), 0x0001, 0x002C, 0x002D,
  ESC_REG(ST7735_FRMCTR2), 0x0001, 0x002C, 0x002D,
  ESC_REG(ST7735_FRMCTR3), 0x0001, 0x002C, 0x002D, 0x0001, 0x002C, 0x002D,
  ESC_REG(ST7735_INVCTR), 0x0007,
  ESC_REG(ST7735_PWCTR1), 0x00A2, 0x0002, 0x0084,
  ESC_REG(ST7735_PWCTR2), 0x00C5,
  ESC_REG(ST7735_PWCTR3), 0x000A, 0x0000,
  ESC_REG(ST7735_PWCTR4), 0x008A, 0x002A,
  ESC_REG(ST7735_PWCTR5), 0x008A, 0x00EE,
  ESC_REG(ST7735_VMCTR1), 0x000E,
  ESC_REG(ST7735_INVOFF),
*/
  ESC_REG(ST7735_MADCTL),  ST7735_MADCTL_DATA,
  ESC_REG(ST7735_COLMOD),  0x0005,

  /* Gamma Correction. Colors with 'after-reset' settings are bleak  */
  ESC_REG(ST7735_GMCTRP1), 0x0002, 0x001C, 0x0007, 0x0012, 0x0037, 0x0032, 0x0029, 0x002D, 0x0029, 0x0025, 0x002B, 0x0039, 0x0000, 0x0001, 0x0003, 0x0010,
  ESC_REG(ST7735_GMCTRN1), 0x0003, 0x001D, 0x0007, 0x0006, 0x002E, 0x002C, 0x0029, 0x002D, 0x002E, 0x002E, 0x0037, 0x003F, 0x0000, 0x0000, 0x0002, 0x0010,

  ESC_REG(ST7735_NORON),
  ESC_REG(ST7735_DISPON),
  ESC_END
};
