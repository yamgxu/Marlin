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

#define ILI9341_MADCTL_MY         0x80 // Row Address Order//行地址顺序
#define ILI9341_MADCTL_MX         0x40 // Column Address Order//列地址顺序
#define ILI9341_MADCTL_MV         0x20 // Row/Column Exchange//行/列交换
#define ILI9341_MADCTL_ML         0x10 // Vertical Refresh Order//垂直刷新顺序
#define ILI9341_MADCTL_BGR        0x08 // RGB-BGR ORDER//RGB-BGR指令
#define ILI9341_MADCTL_RGB        0x00
#define ILI9341_MADCTL_MH         0x04 // Horizontal Refresh Order//水平刷新顺序

#define ILI9341_ORIENTATION_UP    ILI9341_MADCTL_MY                                         // 240x320 ; Cable on the upper side//240x320；电缆在上面
#define ILI9341_ORIENTATION_RIGHT ILI9341_MADCTL_MV                                         // 320x240 ; Cable on the right side//320x240；右侧的电缆
#define ILI9341_ORIENTATION_LEFT  ILI9341_MADCTL_MY | ILI9341_MADCTL_MX | ILI9341_MADCTL_MV // 320x240 ; Cable on the left side//320x240；左侧的电缆
#define ILI9341_ORIENTATION_DOWN  ILI9341_MADCTL_MX                                         // 240x320 ; Cable on the upper side//240x320；电缆在上面

#define ILI9341_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, ILI9341_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    ILI9341_MADCTL_MX) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    ILI9341_MADCTL_MY)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_BGR
  #define ILI9341_COLOR ILI9341_MADCTL_BGR
#elif TFT_COLOR == TFT_COLOR_RGB
  #define ILI9341_COLOR ILI9341_MADCTL_RGB
#endif

#define ILI9341_MADCTL_DATA       (ILI9341_ORIENTATION) | (ILI9341_COLOR)

#define ILI9341_NOP               0x00 // No Operation//无操作
#define ILI9341_SWRESET           0x01 // Software Reset//软件重置
#define ILI9341_RDDIDIF           0x04 // Read display identification information//读取显示识别信息
#define ILI9341_RDDST             0x09 // Read Display Status//读取显示状态
#define ILI9341_RDDPM             0x0A // Read Display Power Mode//读取显示电源模式
#define ILI9341_RDDMADCTL         0x0B // Read Display MADCTL//读取显示MADCTL
#define ILI9341_RDDCOLMOD         0x0C // Read Display Pixel Format//读取显示像素格式
#define ILI9341_RDDIM             0x0D // Read Display Image Format//读取显示图像格式
#define ILI9341_RDDSM             0x0E // Read Display Signal Mode//读取显示信号模式
#define ILI9341_RDDSDR            0x0F // Read Display Self-Diagnostic Result//读取并显示自诊断结果
#define ILI9341_SPLIN             0x10 // Enter Sleep Mode//进入睡眠模式
#define ILI9341_SLPOUT            0x11 // Sleep Out//露宿
#define ILI9341_PTLON             0x12 // Partial Mode ON//部分模式打开
#define ILI9341_NORON             0x13 // Normal Display Mode ON//正常显示模式打开
#define ILI9341_DINVOFF           0x20 // Display Inversion OFF//显示反转关闭
#define ILI9341_DINVON            0x21 // Display Inversion ON//显示反转
#define ILI9341_GAMSET            0x26 // Gamma Set//伽马集
#define ILI9341_DISPOFF           0x28 // Display OFF//炫耀
#define ILI9341_DISPON            0x29 // Display ON//展示
#define ILI9341_CASET             0x2A // Column Address Set//列地址集
#define ILI9341_PASET             0x2B // Page Address Set//页面地址集
#define ILI9341_RAMWR             0x2C // Memory Write//内存写入
#define ILI9341_RGBSET            0x2D // Color Set//颜色集
#define ILI9341_RAMRD             0x2E // Memory Read//内存读取
#define ILI9341_PLTAR             0x30 // Partial Area//局部区域
#define ILI9341_VSCRDEF           0x33 // Vertical Scrolling Definition//垂直滚动定义
#define ILI9341_TEOFF             0x34 // Tearing Effect Line OFF//撕裂效应线
#define ILI9341_TEON              0x35 // Tearing Effect Line ON//线撕裂效应
#define ILI9341_MADCTL            0x36 // Memory Access Control//内存访问控制
#define ILI9341_VSCRSADD          0x37 // Vertical Scrolling Start Address//垂直滚动起始地址
#define ILI9341_IDMOFF            0x38 // Idle Mode OFF//怠速模式关闭
#define ILI9341_IDMON             0x39 // Idle Mode ON//空闲模式打开
#define ILI9341_PIXSET            0x3A // COLMOD: Pixel Format Set//COLMOD：像素格式集
#define ILI9341_WRMEMC            0x3C // Write Memory Continue//写入内存继续
#define ILI9341_RDMEMC            0x3E // Read Memory Continue//读存储器继续
#define ILI9341_STE               0x44 // Set Tear Scanline//设置撕裂扫描线
#define ILI9341_GSCAN             0x45 // Get Scanline//获取扫描线
#define ILI9341_WRDISBV           0x51 // Write Display Brightness//写入显示亮度
#define ILI9341_RDDISBV           0x52 // Read Display Brightness//读取显示亮度
#define ILI9341_WRCTRLD           0x53 // Write CTRL Display//写入CTRL显示
#define ILI9341_RDCTRLD           0x54 // Read CTRL Display//读控制显示
#define ILI9341_WRCABC            0x55 // Write Content Adaptive Brightness Control//写入内容自适应亮度控制
#define ILI9341_RDCABC            0x56 // Read Content Adaptive Brightness Control//读取内容自适应亮度控制
#define ILI9341_WRCABCMB          0x5E // Write CABC Minimum Brightness / Backlight Control 1//写入CABC最小亮度/背光控制1
#define ILI9341_RDCABCMB          0x5F // Read CABC Minimum Brightness / Backlight Control 1//读取CABC最小亮度/背光控制1
#define ILI9341_RDID1             0xDA // Read ID1//读ID1
#define ILI9341_RDID2             0xDB // Read ID2//读ID2
#define ILI9341_RDID3             0xDC // Read ID3//读ID3

#define ILI9341_IFMODE            0xB0 // RGB Interface Signal Control//RGB接口信号控制
#define ILI9341_FRMCTR1           0xB1 // Frame Rate Control (In Normal Mode/Full Colors)//帧速率控制（在正常模式/全彩模式下）
#define ILI9341_FRMCTR2           0xB2 // Frame Rate Control (In Idle Mode/8 colors)//帧速率控制（空闲模式/8色）
#define ILI9341_FRMCTR3           0xB3 // Frame Rate control (In Partial Mode/Full Colors)//帧速率控制（部分模式/全色）
#define ILI9341_INVTR             0xB4 // Display Inversion Control//显示反转控制
#define ILI9341_PRCTR             0xB5 // Blanking Porch Control//下料门廊控制
#define ILI9341_DISCTRL           0xB6 // Display Function Control//显示功能控制
#define ILI9341_ETMOD             0xB7 // Entry Mode Set//进入模式集
#define ILI9341_BLCTL1            0xB8 // Backlight Control 1//背光控制1
#define ILI9341_BLCTL2            0xB9 // Backlight Control 2//背光控制2
#define ILI9341_BLCTL3            0xBA // Backlight Control 3//背光控制3
#define ILI9341_BLCTL4            0xBB // Backlight Control 4//背光控制4
#define ILI9341_BLCTL5            0xBC // Backlight Control 5//背光控制5
#define ILI9341_BLCTL7            0xBE // Backlight Control 7//背光控制7
#define ILI9341_BLCTL8            0xBF // Backlight Control 8//背光控制8
#define ILI9341_PWCTRL1           0xC0 // Power Control 1//电源控制1
#define ILI9341_PWCTRL2           0xC1 // Power Control 2//电源控制2
#define ILI9341_VMCTRL1           0xC5 // VCOM Control 1//VCOM控制1
#define ILI9341_VMCTRL2           0xC7 // VCOM Control 2//VCOM控制2
#define ILI9341_PWCTRLA           0xCB // Power control A//功率控制A
#define ILI9341_PWCTRLB           0xCF // Power control B//功率控制B
#define ILI9341_NVMWR             0xD0 // NV Memory Write//存储器写入
#define ILI9341_NVMPKEY           0xD1 // NV Memory Protection Key//NV内存保护密钥
#define ILI9341_RDNVM             0xD2 // NV Memory Status Read//NV存储器状态读取
#define ILI9341_RDID4             0xD3 // Read ID4 - 0x009341//读取ID4-0x009341
#define ILI9341_PGAMCTRL          0xE0 // Positive Gamma Correction//正伽马校正
#define ILI9341_NGAMCTRL          0xE1 // Negative Gamma Correction//负伽马校正
#define ILI9341_DGAMCTRL1         0xE2 // Digital Gamma Control 1//数字伽马控制1
#define ILI9341_DGAMCTRL2         0xE3 // Digital Gamma Control 2//数字伽马控制2
#define ILI9341_DRVTCTLA1         0xE8 // Driver timing control A//驾驶员正时控制A
#define ILI9341_DRVTCTLA2         0xE9 // Driver timing control A//驾驶员正时控制A
#define ILI9341_DRVTCTLB          0xEA // Driver timing control B//驾驶员正时控制B
#define ILI9341_PONSEQCTL         0xED // Power on sequence control//上电顺序控制
#define ILI9341_EN3G              0xF2 // Enable 3G - 3 gamma control//启用3G-3伽马控制
#define ILI9341_IFCTL             0xF6 // Interface Control//接口控制
#define ILI9341_PUMPRCTL          0xF7 // Pump ratio control//泵比控制


static const uint16_t ili9341_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ILI9341_SWRESET), ESC_DELAY(100),
  ESC_REG(ILI9341_SLPOUT), ESC_DELAY(20),
/*
  ESC_REG(ILI9341_PWCTRLA), 0x0039, 0x002C, 0x0000, 0x0034, 0x0002, // Power control A//功率控制A
  ESC_REG(ILI9341_PWCTRLB), 0x0000, 0x00C1, 0x0030,                 // Power control B//功率控制B
  ESC_REG(ILI9341_DRVTCTLA1), 0x0085, 0x0000, 0x0078,               // Driver timing control A//驾驶员正时控制A
  ESC_REG(ILI9341_DRVTCTLB), 0x0000, 0x0000,                        // Driver timing control B//驾驶员正时控制B
  ESC_REG(ILI9341_PONSEQCTL), 0x0064, 0x0003, 0x0012, 0x0081,       // Power on sequence control//上电顺序控制
  ESC_REG(ILI9341_DISCTRL), 0x0008, 0x0082, 0x0027,                 // Display Function Control//显示功能控制
  ESC_REG(ILI9341_PUMPRCTL), 0x0020,                                // Pump ratio control//泵比控制
  ESC_REG(ILI9341_VMCTRL1), 0x003E, 0x0028,                         // VCOM Control 1//VCOM控制1
  ESC_REG(ILI9341_VMCTRL2), 0x0086,                                 // VCOM Control 2//VCOM控制2
  ESC_REG(ILI9341_FRMCTR1), 0x0000, 0x0018,                         // Frame Rate Control (In Normal Mode/Full Colors)//帧速率控制（在正常模式/全彩模式下）
  ESC_REG(ILI9341_PWCTRL1), 0x0023,                                 // Power Control 1//电源控制1
  ESC_REG(ILI9341_PWCTRL2), 0x0010,                                 // Power Control 2//电源控制2
*/
  ESC_REG(ILI9341_MADCTL), ILI9341_MADCTL_DATA,
  ESC_REG(ILI9341_PIXSET), 0x0055,

  /* Gamma Correction */
  ESC_REG(ILI9341_EN3G), 0x0000,                 // 3Gamma Function Disable//3伽玛函数禁用
  ESC_REG(ILI9341_GAMSET), 0x0001,               // Gamma curve selected//选择伽马曲线
  ESC_REG(ILI9341_PGAMCTRL), 0x000F, 0x0031, 0x002B, 0x000C, 0x000E, 0x0008, 0x004E, 0x00F1, 0x0037, 0x0007, 0x0010, 0x0003, 0x000E, 0x0009, 0x0000,
  ESC_REG(ILI9341_NGAMCTRL), 0x0000, 0x000E, 0x0014, 0x0003, 0x0011, 0x0007, 0x0031, 0x00C1, 0x0048, 0x0008, 0x000F, 0x000C, 0x0031, 0x0036, 0x000F,

  ESC_REG(ILI9341_NORON),
  ESC_REG(ILI9341_DISPON),
  ESC_END
};
