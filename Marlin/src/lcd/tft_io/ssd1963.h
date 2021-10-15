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

#define SSD1963_MADCTL_MY         0x80 // Row Address Order//行地址顺序
#define SSD1963_MADCTL_MX         0x40 // Column Address Order//列地址顺序
#define SSD1963_MADCTL_MV         0x20 // Row/Column Exchange//行/列交换
#define SSD1963_MADCTL_MH         0x10 // Horizontal Refresh Order//水平刷新顺序
#define SSD1963_MADCTL_BGR        0x08 // RGB-BGR ORDER//RGB-BGR指令
#define SSD1963_MADCTL_RGB        0x00
#define SSD1963_MADCTL_ML         0x04 // Vertical Refresh Order//垂直刷新顺序
#define SSD1963_MADCTL_FH         0x02 // Flip Horizontal//水平翻转
#define SSD1963_MADCTL_FV         0x01 // Flip Vertical//垂直翻转

#define SSD1963_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, SSD1963_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    SSD1963_MADCTL_FH) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    SSD1963_MADCTL_FV)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_RGB
  #define SSD1963_COLOR SSD1963_MADCTL_RGB
#elif TFT_COLOR == TFT_COLOR_BGR
  #define SSD1963_COLOR  SSD1963_MADCTL_BGR
#endif

#define SSD1963_MADCTL_DATA       (SSD1963_ORIENTATION) | (SSD1963_COLOR)

#define SSD1963_NOP               0x00 // No Operation//无操作
#define SSD1963_SWRESET           0x01 // Software reset//软件重置
#define SSD1963_RDDPM             0x0A // Read Display Power Mode//读取显示电源模式
#define SSD1963_RDDMADCTL         0x0B // Read Display MADCTL//读取显示MADCTL
#define SSD1963_RDDCOLMOD         0x0C // Read Display Pixel Format//读取显示像素格式
#define SSD1963_RDDIM             0x0D // Read Display Image Mode//读取显示图像模式
#define SSD1963_RDDSM             0x0E // Read Display Signal Mode//读取显示信号模式
#define SSD1963_SLPIN             0x10 // Sleep In//睡懒觉
#define SSD1963_SLPOUT            0x11 // Sleep Out//露宿
#define SSD1963_PTLON             0x12 // Partial Display Mode On//部分显示模式打开
#define SSD1963_NORON             0x13 // Normal Display Mode On//正常显示模式打开
#define SSD1963_INVOFF            0x20 // Display Inversion Off//显示反转关闭
#define SSD1963_INVON             0x21 // Display Inversion On//显示反转
#define SSD1963_GAMSET            0x26 // Gamma Set//伽马集
#define SSD1963_DISPOFF           0x28 // Display Off//炫耀
#define SSD1963_DISPON            0x29 // Display On//展示
#define SSD1963_CASET             0x2A // Column Address Set//列地址集
#define SSD1963_RASET             0x2B // Row Address Set//行地址集
#define SSD1963_RAMWR             0x2C // Memory Write//内存写入
#define SSD1963_RAMRD             0x2E // Memory Read//内存读取
#define SSD1963_PTLAR             0x30 // Partial Area//局部区域
#define SSD1963_VSCRDEF           0x33 // Vertical Scrolling Definition//垂直滚动定义
#define SSD1963_TEOFF             0x34 // Tearing Effect Line OFF//撕裂效应线
#define SSD1963_TEON              0x35 // Tearing Effect Line ON//线撕裂效应
#define SSD1963_MADCTL            0x36 // Memory Data Access Control//存储器数据访问控制
#define SSD1963_VSCSAD            0x37 // Vertical Scroll Start Address of RAM//RAM的垂直滚动起始地址
#define SSD1963_IDMOFF            0x38 // Idle Mode Off//怠速模式关闭
#define SSD1963_IDMON             0x39 // Idle Mode On//空闲模式打开
#define SSD1963_WRMEMC            0x3C // Write Memory Continue//写入内存继续
#define SSD1963_RDMEMC            0x3E // Read Memory Continue//读存储器继续
#define SSD1963_STE               0x44 // Set Tear Scanline//设置撕裂扫描线
#define SSD1963_GSCAN             0x45 // Get Scanline//获取扫描线
#define SSD1963_WRDISBV           0x51 // Write Display Brightness//写入显示亮度
#define SSD1963_RDDISBV           0x52 // Read Display Brightness//读取显示亮度
#define SSD1963_WRCTRLD           0x53 // Write CTRL Display//写入CTRL显示
#define SSD1963_RDCTRLD           0x54 // Read CTRL Value Display//读取CTRL值显示
#define SSD1963_WRCACE            0x55 // Write Content Adaptive Brightness Control and Color Enhancement//写入内容自适应亮度控制和颜色增强
#define SSD1963_RDCABC            0x56 // Read Content Adaptive Brightness Control//读取内容自适应亮度控制
#define SSD1963_WRCABCMB          0x5E // Write CABC Minimum Brightness//写CABC最小亮度
#define SSD1963_RDCABCMB          0x5F // Read CABC Minimum Brightness//读取CABC最小亮度
#define SSD1963_RDABCSDR          0x68 // Read Automatic Brightness Control Self-Diagnostic Result//读取自动亮度控制自诊断结果
#define SSD1963_RDDDB             0xA1 // Read Device Descriptor Block//读取设备描述符块
#define SSD1963_SLCDMODE          0xB0 // Set the LCD panel mode and resolution//设置LCD面板模式和分辨率
#define SSD1963_SHSYNC            0xB4 // Set HSYNC//设置HSYNC
#define SSD1963_GHSYNC            0xB5 // Get HSYNC//获取HSYNC
#define SSD1963_SVSYNC            0xB6 // Set VSYNC//设置VSYNC
#define SSD1963_GVSYNC            0xB7 // Get VSYNC//获取VSYNC
#define SSD1963_SGPIOCFG          0xB8 // Set GPIO Conf//设置GPIO配置
#define SSD1963_SGPIOV            0xBA // Set GPIO Value//设置GPIO值
#define SSD1963_SPWMCFG           0xBE // Set PWM Conf//设置PWM配置
#define SSD1963_GPWMCFG           0xBF // Get PWM Conf//获取PWM配置
#define SSD1963_SDBCCFG           0xD0 // Set Dynamic Back Light Config//设置动态背光配置
#define SSD1963_GDBCCFG           0xD1 // Get Dynamic Back Light Config//获取动态背光配置
#define SSD1963_PLLON             0xE0 // PLL Enable//锁相环使能
#define SSD1963_PLLMN             0xE2 // Set PLL Multiplier//设置锁相倍增器
#define SSD1963_SLSHIFT           0xE6 // Set the LSHIFT (pixel clock) frequency//设置LSHIFT（像素时钟）频率
#define SSD1963_COLMOD            0xF0 // Interface Pixel Format//接口像素格式

static const uint16_t ssd1963_init[] = {
  DATASIZE_8BIT,
  ESC_REG(SSD1963_PLLMN), 0x0023, 0x0002, 0x0054,
  ESC_REG(SSD1963_PLLON), 0x0001, ESC_DELAY(10),
  ESC_REG(SSD1963_PLLON), 0x0003, ESC_DELAY(10),
  ESC_REG(SSD1963_SWRESET), ESC_DELAY(100),

  ESC_REG(SSD1963_SLSHIFT), 0x0001, 0x001F, 0x00FF,
  ESC_REG(SSD1963_SLCDMODE), 0x0020, 0x0000, 0x0001, 0x00DF, 0x0001, 0x000F, 0x0000,
  ESC_REG(SSD1963_SHSYNC), 0x0002, 0x0013, 0x0000, 0x0008, 0x002B, 0x0000, 0x0002, 0x0000,
  ESC_REG(SSD1963_SVSYNC), 0x0001, 0x0020, 0x0000, 0x0004, 0x000C, 0x0000, 0x0002,
  ESC_REG(SSD1963_SGPIOV), 0x000F,
  ESC_REG(SSD1963_SGPIOCFG), 0x0007, 0x0001,

  ESC_REG(SSD1963_MADCTL), SSD1963_MADCTL_DATA,
  ESC_REG(SSD1963_COLMOD), 0x0003, ESC_DELAY(1),//RBG 565//苏格兰皇家银行565

  ESC_REG(SSD1963_NORON),
  ESC_REG(SSD1963_DISPON),

  ESC_REG(SSD1963_SPWMCFG), 0x0006, 0x00F0, 0x0001, 0x00F0, 0x0000, 0x0000,
  ESC_REG(SSD1963_SDBCCFG), 0x000D,
  ESC_END
};
