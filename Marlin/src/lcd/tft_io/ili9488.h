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

#define ILI9488_MADCTL_MY         0x80 // Row Address Order//行地址顺序
#define ILI9488_MADCTL_MX         0x40 // Column Address Order//列地址顺序
#define ILI9488_MADCTL_MV         0x20 // Row/Column Exchange//行/列交换
#define ILI9488_MADCTL_ML         0x10 // Vertical Refresh Order//垂直刷新顺序
#define ILI9488_MADCTL_BGR        0x08 // RGB-BGR ORDER//RGB-BGR指令
#define ILI9488_MADCTL_RGB        0x00
#define ILI9488_MADCTL_MH         0x04 // Horizontal Refresh Order//水平刷新顺序

#define ILI9488_ORIENTATION_UP    ILI9488_MADCTL_MY                                         // 320x480 ; Cable on the upper side//320x480；电缆在上面
#define ILI9488_ORIENTATION_RIGHT ILI9488_MADCTL_MV                                         // 480x320 ; Cable on the right side//480x320；右侧的电缆
#define ILI9488_ORIENTATION_LEFT  ILI9488_MADCTL_MY | ILI9488_MADCTL_MX | ILI9488_MADCTL_MV // 480x320 ; Cable on the left side//480x320；左侧的电缆
#define ILI9488_ORIENTATION_DOWN  ILI9488_MADCTL_MX                                         // 320x480 ; Cable on the upper side//320x480；电缆在上面

#define ILI9488_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, ILI9488_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    ILI9488_MADCTL_MX) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    ILI9488_MADCTL_MY)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_BGR
  #define ILI9488_COLOR ILI9488_MADCTL_BGR
#elif TFT_COLOR == TFT_COLOR_RGB
  #define ILI9488_COLOR ILI9488_MADCTL_RGB
#endif

#define ILI9488_MADCTL_DATA       (ILI9488_ORIENTATION) | (ILI9488_COLOR)

#define ILI9488_NOP               0x00 // No Operation//无操作
#define ILI9488_SWRESET           0x01 // Software Reset//软件重置
#define ILI9488_RDDIDIF           0x04 // Read Display Identification Information//读取显示识别信息
#define ILI9488_RDNUMED           0x05 // Read Number of the Errors on DSI//读取DSI上的错误数
#define ILI9488_RDDST             0x09 // Read Display Status//读取显示状态
#define ILI9488_RDDPM             0x0A // Read Display Power Mode//读取显示电源模式
#define ILI9488_RDDMADCTL         0x0B // Read Display MADCTL//读取显示MADCTL
#define ILI9488_RDDCOLMOD         0x0C // Read Display COLMOD//读取显示COLMOD
#define ILI9488_RDDIM             0x0D // Read Display Image Mode//读取显示图像模式
#define ILI9488_RDDSM             0x0E // Read Display Signal Mode//读取显示信号模式
#define ILI9488_RDDSDR            0x0F // Read Display Self-Diagnostic Result//读取并显示自诊断结果
#define ILI9488_SLPIN             0x10 // Sleep IN//睡懒觉
#define ILI9488_SLPOUT            0x11 // Sleep OUT//露宿
#define ILI9488_PTLON             0x12 // Partial Mode ON//部分模式打开
#define ILI9488_NORON             0x13 // Normal Display Mode ON//正常显示模式打开
#define ILI9488_INVOFF            0x20 // Display Inversion OFF//显示反转关闭
#define ILI9488_INVON             0x21 // Display Inversion ON//显示反转
#define ILI9488_ALLPOFF           0x22 // All Pixels OFF//所有像素都消失了
#define ILI9488_ALLPON            0x23 // All Pixels ON//所有像素都打开
#define ILI9488_DISOFF            0x28 // Display OFF//炫耀
#define ILI9488_DISON             0x29 // Display ON//展示
#define ILI9488_CASET             0x2A // Column Address Set//列地址集
#define ILI9488_PASET             0x2B // Page Address Set//页面地址集
#define ILI9488_RAMWR             0x2C // Memory Write//内存写入
#define ILI9488_RAMRD             0x2E // Memory Read//内存读取
#define ILI9488_PLTAR             0x30 // Partial Area//局部区域
#define ILI9488_VSCRDEF           0x33 // Vertical Scrolling Definition//垂直滚动定义
#define ILI9488_TEOFF             0x34 // Tearing Effect Line OFF//撕裂效应线
#define ILI9488_TEON              0x35 // Tearing Effect Line ON//线撕裂效应
#define ILI9488_MADCTL            0x36 // Memory Access Control//内存访问控制
#define ILI9488_VSCRSADD          0x37 // Vertical Scrolling Start Address//垂直滚动起始地址
#define ILI9488_IDMOFF            0x38 // Idle Mode OFF//怠速模式关闭
#define ILI9488_IDMON             0x39 // Idle Mode ON//空闲模式打开
#define ILI9488_COLMOD            0x3A // Interface Pixel Format//接口像素格式
#define ILI9488_RAMWRC            0x3C // Memory Write Continue//内存写入继续
#define ILI9488_RAMRDRC           0x3E // Memory Read Continue//内存读取继续
#define ILI9488_TESLWR            0x44 // Write Tear Scan Line//写撕裂扫描线
#define ILI9488_TESLRD            0x45 // Read Scan Line//读取扫描线
#define ILI9488_WRDISBV           0x51 // Write Display Brightness Value//写入显示亮度值
#define ILI9488_RDDISBV           0x52 // Read Display Brightness Value//读取显示亮度值
#define ILI9488_WRCTRLD           0x53 // Write Control Display Value//写控制显示值
#define ILI9488_RDCTRLD           0x54 // Read Control Display Value//读取控制显示值
#define ILI9488_WRCABC            0x55 // Write Content Adaptive Brightness Control Value//写入内容自适应亮度控制值
#define ILI9488_RDCABC            0x56 // Read Content Adaptive Brightness Control Value//读取内容自适应亮度控制值
#define ILI9488_WRCABCMB          0x5E // Write CABC Minimum Brightness//写CABC最小亮度
#define ILI9488_RDCABCMB          0x5F // Read CABC Minimum Brightness//读取CABC最小亮度
#define ILI9488_RDABCSDR          0x68 // Read Automatic Brightness Control Self-diagnostic Result//读取自动亮度控制自诊断结果
#define ILI9488_RDID1             0xDA // Read ID1//读ID1
#define ILI9488_RDID2             0xDB // Read ID2//读ID2
#define ILI9488_RDID3             0xDC // Read ID3//读ID3

#define ILI9488_IFMODE            0xB0 // Interface Mode Control//接口模式控制
#define ILI9488_FRMCTR1           0xB1 // Frame Rate Control (In Normal Mode/Full Colors)//帧速率控制（在正常模式/全彩模式下）
#define ILI9488_FRMCTR2           0xB2 // Frame Rate Control (In Idle Mode/8 Colors)//帧速率控制（空闲模式/8色）
#define ILI9488_FRMCTR3           0xB3 // Frame Rate Control (In Partial Mode/Full Colors)//帧速率控制（部分模式/全色）
#define ILI9488_INVTR             0xB4 // Display Inversion Control//显示反转控制
#define ILI9488_PRCTR             0xB5 // Blanking Porch Control//下料门廊控制
#define ILI9488_DISCTRL           0xB6 // Display Function Control//显示功能控制
#define ILI9488_ETMOD             0xB7 // Entry Mode Set//进入模式集
#define ILI9488_CECTRL1           0xB9 // Color Enhancement Control 1//色彩增强控制1
#define ILI9488_CECTRL2           0xBA // Color Enhancement Control 2//色彩增强控制2
#define ILI9488_HSLCTRL           0xBE // HS Lanes Control//高速行车线管制
#define ILI9488_PWCTRL1           0xC0 // Power Control 1//电源控制1
#define ILI9488_PWCTRL2           0xC1 // Power Control 2//电源控制2
#define ILI9488_PWCTRL3           0xC2 // Power Control 3 (For Normal Mode)//电源控制3（用于正常模式）
#define ILI9488_PWCTRL4           0xC3 // Power Control 4 (For Idle Mode)//功率控制4（用于怠速模式）
#define ILI9488_PWCTRL5           0xC4 // Power Control 5 (For Partial Mode)//电源控制5（用于部分模式）
#define ILI9488_VMCTRL            0xC5 // VCOM Control//VCOM控制
#define ILI9488_CABCCTRL1         0xC6 // CABC Control 1//CABC控制1
#define ILI9488_CABCCTRL2         0xC8 // CABC Control 2//CABC控制2
#define ILI9488_CABCCTRL3         0xC9 // CABC Control 3//CABC控制3
#define ILI9488_CABCCTRL4         0xCA // CABC Control 4//CABC控制4
#define ILI9488_CABCCTRL5         0xCB // CABC Control 5//CABC控制5
#define ILI9488_CABCCTRL6         0xCC // CABC Control 6//CABC控制6
#define ILI9488_CABCCTRL7         0xCD // CABC Control 7//CABC控制7
#define ILI9488_CABCCTRL8         0xCE // CABC Control 8//CABC控制8
#define ILI9488_CABCCTRL9         0xCF // CABC Control 9//CABC控制9
#define ILI9488_NVMWR             0xD0 // NV Memory Write//存储器写入
#define ILI9488_NVMPKEY           0xD1 // NV Memory Protection Key//NV内存保护密钥
#define ILI9488_RDNVM             0xD2 // NV Memory Status Read//NV存储器状态读取
#define ILI9488_RDID4             0xD3 // Read ID4 - 0x009488//读取ID4-0x009488
#define ILI9488_ADJCTL1           0xD7 // Adjust Control 1//调整控制1
#define ILI9488_RDIDV             0xD8 // Read ID Version//读取ID版本
#define ILI9488_PGAMCTRL          0xE0 // Positive Gamma Control//正γ对照
#define ILI9488_NGAMCTRL          0xE1 // Negative Gamma Control//负伽马控制
#define ILI9488_DGAMCTRL1         0xE2 // Ditigal Gamma Control 1//双伽马控制1
#define ILI9488_DGAMCTRL2         0xE3 // Ditigal Gamma Control 2//双igal伽马控制2
#define ILI9488_SETIMAGE          0xE9 // Set Image Function//设置图像函数
#define ILI9488_ADJCTL2           0xF2 // Adjust Control 2//调整控制2
#define ILI9488_ADJCTL3           0xF7 // Adjust Control 3//调整控制3
#define ILI9488_ADJCTL4           0xF8 // Adjust Control 4//调整控制4
#define ILI9488_ADJCTL5           0xF9 // Adjust Control 5//调整控制5
#define ILI9488_RDEXTC            0xFB // Read EXTC command is SPI mode//Read EXTC命令为SPI模式
#define ILI9488_ADJCTL6           0xFC // Adjust Control 6//调整控制装置6
#define ILI9488_ADJCTL7           0xFF // Adjust Control 7//调整控制装置7

static const uint16_t ili9488_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ILI9488_SWRESET), ESC_DELAY(120),
  ESC_REG(ILI9488_SLPOUT), ESC_DELAY(20),

  ESC_REG(ILI9488_MADCTL), ILI9488_MADCTL_DATA,
  ESC_REG(ILI9488_COLMOD), 0x0055,

  /* Gamma Correction. */
  ESC_REG(ILI9488_PGAMCTRL), 0x0000, 0x0003, 0x0009, 0x0008, 0x0016, 0x000A, 0x003F, 0x0078, 0x004C, 0x0009, 0x000A, 0x0008, 0x0016, 0x001A, 0x000F,
  ESC_REG(ILI9488_NGAMCTRL), 0x0000, 0x0016, 0x0019, 0x0003, 0x000F, 0x0005, 0x0032, 0x0045, 0x0046, 0x0004, 0x000E, 0x000D, 0x0035, 0x0037, 0x000F,

  ESC_REG(ILI9488_NORON),
  ESC_REG(ILI9488_DISON),
  ESC_END
};
