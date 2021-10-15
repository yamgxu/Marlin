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

#define ST7789V_MADCTL_MY         0x80 // Row Address Order//行地址顺序
#define ST7789V_MADCTL_MX         0x40 // Column Address Order//列地址顺序
#define ST7789V_MADCTL_MV         0x20 // Row/Column Exchange//行/列交换
#define ST7789V_MADCTL_ML         0x10 // Vertical Refresh Order//垂直刷新顺序
#define ST7789V_MADCTL_BGR        0x08 // RGB-BGR ORDER//RGB-BGR指令
#define ST7789V_MADCTL_RGB        0x00
#define ST7789V_MADCTL_MH         0x04 // Horizontal Refresh Order//水平刷新顺序

#define ST7789V_ORIENTATION_UP    ST7789V_MADCTL_MX | ST7789V_MADCTL_MY // 240x320 ; Cable on the upper side//240x320；电缆在上面
#define ST7789V_ORIENTATION_RIGHT ST7789V_MADCTL_MX | ST7789V_MADCTL_MV // 320x240 ; Cable on the right side//320x240；右侧的电缆
#define ST7789V_ORIENTATION_LEFT  ST7789V_MADCTL_MY | ST7789V_MADCTL_MV // 320x240 ; Cable on the left side//320x240；左侧的电缆
#define ST7789V_ORIENTATION_DOWN  0                                     // 240x320 ; Cable on the lower side//240x320；下侧的电缆

#define ST7789V_ORIENTATION IF_0((TFT_ORIENTATION) & TFT_EXCHANGE_XY, ST7789V_MADCTL_MV) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_X,    ST7789V_MADCTL_MX) | \
                            IF_0((TFT_ORIENTATION) & TFT_INVERT_Y,    ST7789V_MADCTL_MY)

#if !defined(TFT_COLOR) || TFT_COLOR == TFT_COLOR_RGB
  #define ST7789V_COLOR ST7789V_MADCTL_RGB
#elif TFT_COLOR == TFT_COLOR_BGR
  #define ST7789V_COLOR ST7789V_MADCTL_BGR
#endif

#define ST7789V_MADCTL_DATA       (ST7789V_ORIENTATION) | (ST7789V_COLOR)

#define ST7789V_NOP               0x00 // No Operation//无操作
#define ST7789V_SWRESET           0x01 // Software reset//软件重置
#define ST7789V_RDDID             0x04 // Read Display ID//读取显示ID
#define ST7789V_RDDST             0x09 // Read Display Status//读取显示状态
#define ST7789V_RDDPM             0x0A // Read Display Power Mode//读取显示电源模式
#define ST7789V_RDDMADCTL         0x0B // Read Display MADCTL//读取显示MADCTL
#define ST7789V_RDDCOLMOD         0x0C // Read Display Pixel Format//读取显示像素格式
#define ST7789V_RDDIM             0x0D // Read Display Image Mode//读取显示图像模式
#define ST7789V_RDDSM             0x0E // Read Display Signal Mode//读取显示信号模式
#define ST7789V_RDDSDR            0x0F // Read Display Self-Diagnostic Result//读取并显示自诊断结果
#define ST7789V_SLPIN             0x10 // Sleep In//睡懒觉
#define ST7789V_SLPOUT            0x11 // Sleep Out//露宿
#define ST7789V_PTLON             0x12 // Partial Display Mode On//部分显示模式打开
#define ST7789V_NORON             0x13 // Normal Display Mode On//正常显示模式打开
#define ST7789V_INVOFF            0x20 // Display Inversion Off//显示反转关闭
#define ST7789V_INVON             0x21 // Display Inversion On//显示反转
#define ST7789V_GAMSET            0x26 // Gamma Set//伽马集
#define ST7789V_DISPOFF           0x28 // Display Off//炫耀
#define ST7789V_DISPON            0x29 // Display On//展示
#define ST7789V_CASET             0x2A // Column Address Set//列地址集
#define ST7789V_RASET             0x2B // Row Address Set//行地址集
#define ST7789V_RAMWR             0x2C // Memory Write//内存写入
#define ST7789V_RAMRD             0x2E // Memory Read//内存读取
#define ST7789V_PTLAR             0x30 // Partial Area//局部区域
#define ST7789V_VSCRDEF           0x33 // Vertical Scrolling Definition//垂直滚动定义
#define ST7789V_TEOFF             0x34 // Tearing Effect Line OFF//撕裂效应线
#define ST7789V_TEON              0x35 // Tearing Effect Line ON//线撕裂效应
#define ST7789V_MADCTL            0x36 // Memory Data Access Control//存储器数据访问控制
#define ST7789V_VSCSAD            0x37 // Vertical Scroll Start Address of RAM//RAM的垂直滚动起始地址
#define ST7789V_IDMOFF            0x38 // Idle Mode Off//怠速模式关闭
#define ST7789V_IDMON             0x39 // Idle Mode On//空闲模式打开
#define ST7789V_COLMOD            0x3A // Interface Pixel Format//接口像素格式
#define ST7789V_WRMEMC            0x3C // Write Memory Continue//写入内存继续
#define ST7789V_RDMEMC            0x3E // Read Memory Continue//读存储器继续
#define ST7789V_STE               0x44 // Set Tear Scanline//设置撕裂扫描线
#define ST7789V_GSCAN             0x45 // Get Scanline//获取扫描线
#define ST7789V_WRDISBV           0x51 // Write Display Brightness//写入显示亮度
#define ST7789V_RDDISBV           0x52 // Read Display Brightness//读取显示亮度
#define ST7789V_WRCTRLD           0x53 // Write CTRL Display//写入CTRL显示
#define ST7789V_RDCTRLD           0x54 // Read CTRL Value Display//读取CTRL值显示
#define ST7789V_WRCACE            0x55 // Write Content Adaptive Brightness Control and Color Enhancement//写入内容自适应亮度控制和颜色增强
#define ST7789V_RDCABC            0x56 // Read Content Adaptive Brightness Control//读取内容自适应亮度控制
#define ST7789V_WRCABCMB          0x5E // Write CABC Minimum Brightness//写CABC最小亮度
#define ST7789V_RDCABCMB          0x5F // Read CABC Minimum Brightness//读取CABC最小亮度
#define ST7789V_RDABCSDR          0x68 // Read Automatic Brightness Control Self-Diagnostic Result//读取自动亮度控制自诊断结果
#define ST7789V_RDID1             0xDA // Read ID1 Value//读取ID1值
#define ST7789V_RDID2             0xDB // Read ID2 Value//读取ID2值
#define ST7789V_RDID3             0xDC // Read ID3 Value//读取ID3值

#define ST7789V_RAMCTRL           0xB0 // RAM Control//RAM控制
#define ST7789V_RGBCTRL           0xB1 // RGB Interface Control//RGB接口控制
#define ST7789V_PORCTRL           0xB2 // Porch Setting//门廊设置
#define ST7789V_FRCTRL1           0xB3 // Frame Rate Control 1 (In partial mode/ idle colors)//帧速率控制1（部分模式/空闲颜色）
#define ST7789V_GCTRL             0xB7 // Gate Control//闸门控制
#define ST7789V_DGMEN             0xBA // Digital Gamma Enable//数字伽马使能
#define ST7789V_VCOMS             0xBB // VCOM Setting//VCOM设置
#define ST7789V_LCMCTRL           0xC0 // LCM Control//LCM控制
#define ST7789V_IDSET             0xC1 // ID Code Setting//ID代码设置
#define ST7789V_VDVVRHEN          0xC2 // VDV and VRH Command Enable//VDV和VRH命令启用
#define ST7789V_VRHS              0xC3 // VRH Set//VRH装置
#define ST7789V_VDVS              0xC4 // VDV Set//VDV装置
#define ST7789V_VCMOFSET          0xC5 // VCOM Offset Set//VCOM偏移集
#define ST7789V_FRCTRL2           0xC6 // Frame Rate Control in Normal Mode//正常模式下的帧速率控制
#define ST7789V_CABCCTRL          0xC7 // CABC Control//CABC控制
#define ST7789V_REGSEL1           0xC8 // Register Value Selection 1//寄存器值选择1
#define ST7789V_REGSEL2           0xCA // Register Value Selection 2//寄存器值选择2
#define ST7789V_PWMFRSEL          0xCC // PWM Frequency Selection//PWM频率选择
#define ST7789V_PWCTRL1           0xD0 // Power Control 1//电源控制1
#define ST7789V_VAPVANEN          0xD2 // Enable VAP/VAN signal output//启用VAP/VAN信号输出
#define ST7789V_CMD2EN            0xDF // Command 2 Enable//命令2启用
#define ST7789V_PVGAMCTRL         0xE0 // Positive Voltage Gamma Control//正电压伽马控制
#define ST7789V_NVGAMCTRL         0xE1 // Negative Voltage Gamma Control//负电压伽马控制
#define ST7789V_DGMLUTR           0xE2 // Digital Gamma Look-up Table for Red//红色数字伽马查找表
#define ST7789V_DGMLUTB           0xE3 // Digital Gamma Look-up Table for Blue//蓝色数字伽马查找表
#define ST7789V_GATECTRL          0xE4 // Gate Control//闸门控制
#define ST7789V_SPI2EN            0xE7 // SPI2 Enable//SPI2使能
#define ST7789V_PWCTRL2           0xE8 // Power Control 2//电源控制2
#define ST7789V_EQCTRL            0xE9 // Equalize time control//均衡时间控制
#define ST7789V_PROMCTRL          0xEC // Program Mode Control//程序模式控制
#define ST7789V_PROMEN            0xFA // Program Mode Enable//程序模式启用
#define ST7789V_NVMSET            0xFC // NVM Setting//NVM设置
#define ST7789V_PROMACT           0xFE // Program action//计划行动

static const uint16_t st7789v_init[] = {
  DATASIZE_8BIT,
  ESC_REG(ST7789V_SWRESET), ESC_DELAY(100),
  ESC_REG(ST7789V_SLPOUT), ESC_DELAY(20),

  ESC_REG(ST7789V_PORCTRL), 0x000C, 0x000C, 0x0000, 0x0033, 0x0033,
  ESC_REG(ST7789V_GCTRL), 0x0035,
  ESC_REG(ST7789V_VCOMS), 0x001F,
  ESC_REG(ST7789V_LCMCTRL), 0x002C,
  ESC_REG(ST7789V_VDVVRHEN), 0x0001, 0x00C3,
  ESC_REG(ST7789V_VDVS), 0x0020,
  ESC_REG(ST7789V_FRCTRL2), 0x000F,
  ESC_REG(ST7789V_PWCTRL1), 0x00A4, 0x00A1,

  ESC_REG(ST7789V_MADCTL), ST7789V_MADCTL_DATA,
  ESC_REG(ST7789V_COLMOD), 0x0055,

  ESC_REG(ST7789V_NORON),
  ESC_REG(ST7789V_DISPON),
  ESC_END
};
