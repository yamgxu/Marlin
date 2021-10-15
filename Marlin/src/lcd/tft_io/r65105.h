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

#define R61505_DRVCTL_SM          0x0400
#define R61505_DRVCTL_SS          0x0100 // Select the shift direction of outputs from the source driver. 0 - from S1 to S720 / 1 - from S720 to S1//选择源驱动器输出的换档方向。0-从S1到S720/1-从S720到S1

#define R61505_ETMOD_TRIGGER      0x8000 // 18-bit RAM when set; 16-bit RAM when not set//设置时为18位RAM；未设置时为16位RAM
#define R61505_ETMOD_DFM          0x4000
#define R61505_ETMOD_BGR          0x1000 // RGB-BGR ORDER//RGB-BGR指令
#define R61505_ETMOD_RGB          0x0000

#define R61505_ETMOD_HWM          0x0020
#define R61505_ETMOD_ORG          0x0080
#define R61505_ETMOD_ID1          0x0020 // 0 - Vertical Decrement / 1 - Vertical Increment//0-垂直递减/1-垂直递增
#define R61505_ETMOD_ID0          0x0010 // 0 - Horizontal Decrement / 1 - Horizontal Increment//0-水平递减/1-水平递增
#define R61505_ETMOD_AM           0x0008 // 0 - Horizontal / 1 - Vertical//0-水平/1-垂直

#define R61505_DRVCTRL_GS         0x8000 // Gate Scan direction//门扫描方向

// MKS Robin TFT v1.1 - 320x240 ; Cable on the left side//MKS Robin TFT v1.1-320x240；左侧的电缆

#if TFT_ROTATION == TFT_ROTATE_180
  #define R61505_DRVCTL_DATA      0x0000
  #define R61505_DRVCTRL_DATA     (0x2700 | R61505_DRVCTRL_GS)
#else
  #define R61505_DRVCTL_DATA      R61505_DRVCTL_SS
  #define R61505_DRVCTRL_DATA     0x2700
#endif

/*
#define R61505_ETMOD_ORIENTATION  IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, R61505_ETMOD_AM)  | \
                                  IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    R61505_ETMOD_ID0) | \
                                  IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    R61505_ETMOD_ID1)
*/

#define R61505_ETMOD_ORIENTATION (R61505_ETMOD_AM | R61505_ETMOD_ID0 | R61505_ETMOD_ID1)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_BGR
  #define R61505_ETMOD_COLOR R61505_ETMOD_BGR
#elif TFT_COLOR == TFT_COLOR_RGB
  #define R61505_ETMOD_COLOR R61505_ETMOD_RGB
#endif

#define R61505_ETMOD_DATA       (R61505_ETMOD_ORIENTATION) | (R61505_ETMOD_COLOR)


#define R61505_RDDID              0x00 // ID code - 0x1505//ID代码-0x1505
#define R61505_DRVCTL             0x01 // Driver Output Control//驱动器输出控制
#define R61505_LCDCTL             0x02 // LCD Driving Wave Control//液晶驱动波控制
#define R61505_ETMOD              0x03 // Entry Mode - Control the GRAM update direction//进入模式-控制GRAM更新方向
#define R61505_RESIZECTL          0x04 // Resizing Control Register//调整控制寄存器大小
#define R61505_DISCTRL1           0x07 // Display Control 1//显示控制1
#define R61505_DISCTRL2           0x08 // Display Control 2//显示控制2
#define R61505_DISCTRL3           0x09 // Display Control 3//显示控制3
#define R61505_DISCTRL4           0x0A // Display Control 4//显示控制4
#define R61505_RGBCTRL1           0x0C // RGB Display Interface Control 1//RGB显示接口控制1
#define R61505_FMARKERPOS         0x0D // Frame Marker Position//帧标记位置
#define R61505_RGBCTRL2           0x0F // RGB Display Interface Control 2//RGB显示界面控制2
#define R61505_PWCTRL1            0x10 // Power Control 1//电源控制1
#define R61505_PWCTRL2            0x11 // Power Control 2//电源控制2
#define R61505_PWCTRL3            0x12 // Power Control 3//电源控制3
#define R61505_PWCTRL4            0x13 // Power Control 4//电源控制4

// With landscape screen orientation 'Horizontal' is Y and 'Vertical' is X//对于横向屏幕方向，“水平”为Y，“垂直”为X
#define R61505_HASET              0x20 // GRAM Horizontal Address Set (0-255)//GRAM水平地址集（0-255）
#define R61505_VASET              0x21 // GRAM Vertical Address Set (0-511)//GRAM垂直地址集（0-511）
#define R61505_RAMWR              0x22 // Write data to GRAM//将数据写入GRAM
#define R61505_RAMRD              0x22 // Read Data from GRAM//从GRAM读取数据

#define R61505_PWCTRL7            0x29 // Power Control 7//功率控制7
#define R61505_GAMCTRL1           0x30 // Gamma Control//伽马控制
#define R61505_GAMCTRL2           0x31 // Gamma Control//伽马控制
#define R61505_GAMCTRL3           0x32 // Gamma Control//伽马控制
#define R61505_GAMCTRL4           0x35 // Gamma Control//伽马控制
#define R61505_GAMCTRL5           0x36 // Gamma Control//伽马控制
#define R61505_GAMCTRL6           0x37 // Gamma Control//伽马控制
#define R61505_GAMCTRL7           0x38 // Gamma Control//伽马控制
#define R61505_GAMCTRL8           0x39 // Gamma Control//伽马控制
#define R61505_GAMCTRL9           0x3C // Gamma Control//伽马控制
#define R61505_GAMCTRLA           0x3D // Gamma Control//伽马控制

// With landscape screen orientation 'Horizontal' is Y and 'Vertical' is X//对于横向屏幕方向，“水平”为Y，“垂直”为X
#define R61505_HASTART            0x50 // Horizontal Address Start Position (0-255)//水平地址起始位置（0-255）
#define R61505_HAEND              0x51 // Horizontal Address End Position (0-255)//水平地址结束位置（0-255）
#define R61505_VASTART            0x52 // Vertical Address Start Position (0-511)//垂直地址起始位置（0-511）
#define R61505_VAEND              0x53 // Vertical Address End Position (0-511)//垂直地址结束位置（0-511）

#define R61505_DRVCTRL            0x60 // Driver Output Control//驱动器输出控制
#define R61505_BASE_IMAGE_CTRL    0x61 // Base Image Display Control//基本图像显示控制
#define R61505_VSCROLL_CTRL       0x6A // Vertical Scroll Control//垂直滚动控制

#define R61505_PLTPOS1            0x80 // Partial Image 1 Display Position//部分图像1显示位置
#define R61505_PLTSTART1          0x81 // Partial Image 1 RAM Start Address//部分映像1 RAM开始地址
#define R61505_PLTEND1            0x82 // Partial Image 1 RAM End Address//部分映像1 RAM端地址
#define R61505_PLTPOS2            0x83 // Partial Image 2 Display Position//部分图像2显示位置
#define R61505_PLTSTART2          0x84 // Partial Image 2 RAM Start Address//部分映像2 RAM起始地址
#define R61505_PLTEND2            0x85 // Partial Image 2 RAM End Address//部分映像2 RAM端地址

#define R61505_IFCTL1             0x90 // Panel Interface Control 1//面板接口控制1
#define R61505_IFCTL2             0x92 // Panel Interface Control 2//面板接口控制2
#define R61505_IFCTL3             0x93 // Panel Interface Control 3//面板接口控制3
#define R61505_IFCTL4             0x95 // Panel Interface Control 4//面板接口控制4
#define R61505_IFCTL5             0x97 // Panel Interface Control 5//面板接口控制5
#define R61505_IFCTL6             0x98 // Panel Interface Control 6//面板接口控制6

#define R61505_OSC_CTRL           0xA4 // Oscillation Control//振荡控制


static const uint16_t r61505_init[] = {
  DATASIZE_16BIT,
  ESC_REG(R61505_DRVCTL), R61505_DRVCTL_DATA,
  ESC_REG(R61505_LCDCTL), 0x0700,             // LCD Driving Wave Control//液晶驱动波控制
  ESC_REG(R61505_ETMOD), R61505_ETMOD_DATA,

  ESC_REG(R61505_RESIZECTL), 0x0000,
  ESC_REG(R61505_DISCTRL1), 0x0173,
  ESC_REG(R61505_DISCTRL2), 0x0202,
  ESC_REG(R61505_DISCTRL3), 0x0000,
  ESC_REG(R61505_DISCTRL4), 0x0000,
  ESC_REG(R61505_RGBCTRL1), 0x0000,
  ESC_REG(R61505_FMARKERPOS), 0x0000,
  ESC_REG(R61505_RGBCTRL2), 0x0000,

  ESC_REG(R61505_PWCTRL1), 0x17B0,
  ESC_REG(R61505_PWCTRL2), 0x0037,
  ESC_REG(R61505_PWCTRL3), 0x0138,
  ESC_REG(R61505_PWCTRL4), 0x1700,
  ESC_REG(R61505_PWCTRL7), 0x000D,

  ESC_REG(R61505_GAMCTRL1), 0x0001,
  ESC_REG(R61505_GAMCTRL2), 0x0606,
  ESC_REG(R61505_GAMCTRL3), 0x0304,
  ESC_REG(R61505_GAMCTRL4), 0x0103,
  ESC_REG(R61505_GAMCTRL5), 0x011D,
  ESC_REG(R61505_GAMCTRL6), 0x0404,
  ESC_REG(R61505_GAMCTRL7), 0x0404,
  ESC_REG(R61505_GAMCTRL8), 0x0404,
  ESC_REG(R61505_GAMCTRL9), 0x0700,
  ESC_REG(R61505_GAMCTRLA), 0x0A1F,

  ESC_REG(R61505_DRVCTRL), R61505_DRVCTRL_DATA,
  ESC_REG(R61505_BASE_IMAGE_CTRL), 0x0001,
  ESC_REG(R61505_VSCROLL_CTRL), 0x0000,

  ESC_REG(R61505_IFCTL1), 0x0010,
  ESC_REG(R61505_IFCTL2), 0x0000,
  ESC_REG(R61505_IFCTL3), 0x0003,
  ESC_REG(R61505_IFCTL4), 0x0101,
  ESC_REG(R61505_IFCTL5), 0x0000,
  ESC_REG(R61505_IFCTL6), 0x0000,
  ESC_END
};
