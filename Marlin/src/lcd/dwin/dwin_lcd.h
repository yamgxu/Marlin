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

/********************************************************************************
 * @file     dwin_lcd.h
 * @author   LEO / Creality3D
 * @date     2019/07/18
 * @version  2.0.1
 * @brief    迪文屏控制操作函数
 ********************************************************************************/

#include <stdint.h>

#define RECEIVED_NO_DATA         0x00
#define RECEIVED_SHAKE_HAND_ACK  0x01

#define FHONE                    0xAA

#define DWIN_SCROLL_UP   2
#define DWIN_SCROLL_DOWN 3

#define DWIN_WIDTH  272
#define DWIN_HEIGHT 480

/*-------------------------------------- System variable function --------------------------------------*/

// Handshake (1: Success, 0: Fail)//握手（1:成功，0:失败）
bool DWIN_Handshake(void);

// Common DWIN startup//公共DWIN启动
void DWIN_Startup(void);

// Set the backlight luminance//设置背光亮度
//  luminance: (0x00-0xFF)//亮度：（0x00-0xFF）
void DWIN_Backlight_SetLuminance(const uint8_t luminance);

// Set screen display direction//设置屏幕显示方向
//  dir: 0=0°, 1=90°, 2=180°, 3=270°//方向：0=0°，1=90°，2=180°，3=270°
void DWIN_Frame_SetDir(uint8_t dir);

// Update display//更新显示
void DWIN_UpdateLCD(void);

/*---------------------------------------- Drawing functions ----------------------------------------*/

// Clear screen//清屏
//  color: Clear screen color//颜色：清晰的屏幕颜色
void DWIN_Frame_Clear(const uint16_t color);

// Draw a point//划清界限
//  width: point width   0x01-0x0F//宽度：点宽度0x01-0x0F
//  height: point height 0x01-0x0F//高度：点高度0x01-0x0F
//  x,y: upper left point//x，y：左上角点
void DWIN_Draw_Point(uint8_t width, uint8_t height, uint16_t x, uint16_t y);

// Draw a line//划一条线
//  color: Line segment color//颜色：线段颜色
//  xStart/yStart: Start point//xStart/yStart：起点
//  xEnd/yEnd: End point//结束/结束：结束点
void DWIN_Draw_Line(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

// Draw a Horizontal line//划一条水平线
//  color: Line segment color//颜色：线段颜色
//  xStart/yStart: Start point//xStart/yStart：起点
//  xLength: Line Length//xLength：行长度
inline void DWIN_Draw_HLine(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xLength) {
  DWIN_Draw_Line(color, xStart, yStart, xStart + xLength - 1, yStart);
}

// Draw a Vertical line//画一条垂直线
//  color: Line segment color//颜色：线段颜色
//  xStart/yStart: Start point//xStart/yStart：起点
//  yLength: Line Length//长度：行长度
inline void DWIN_Draw_VLine(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t yLength) {
  DWIN_Draw_Line(color, xStart, yStart, xStart, yStart + yLength - 1);
}

// Draw a rectangle//画一个长方形
//  mode: 0=frame, 1=fill, 2=XOR fill//模式：0=帧，1=填充，2=异或填充
//  color: Rectangle color//颜色：矩形颜色
//  xStart/yStart: upper left point//xStart/yStart：左上角点
//  xEnd/yEnd: lower right point//xEnd/yEnd：右下角点
void DWIN_Draw_Rectangle(uint8_t mode, uint16_t color,
                         uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

// Draw a box//画一个盒子
//  mode: 0=frame, 1=fill, 2=XOR fill//模式：0=帧，1=填充，2=异或填充
//  color: Rectangle color//颜色：矩形颜色
//  xStart/yStart: upper left point//xStart/yStart：左上角点
//  xSize/ySize: box size//xSize/ySize：框大小
inline void DWIN_Draw_Box(uint8_t mode, uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xSize, uint16_t ySize) {
  DWIN_Draw_Rectangle(mode, color, xStart, yStart, xStart + xSize - 1, yStart + ySize - 1);
}

// Move a screen area//移动屏幕区域
//  mode: 0, circle shift; 1, translation//模式：0，循环移位；1、翻译
//  dir: 0=left, 1=right, 2=up, 3=down//方向：0=左，1=右，2=上，3=下
//  dis: Distance//dis：距离
//  color: Fill color//颜色：填充颜色
//  xStart/yStart: upper left point//xStart/yStart：左上角点
//  xEnd/yEnd: bottom right point//xEnd/yEnd：右下角点
void DWIN_Frame_AreaMove(uint8_t mode, uint8_t dir, uint16_t dis,
                         uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

/*---------------------------------------- Text related functions ----------------------------------------*/

// Draw a string//牵线
//  widthAdjust: true=self-adjust character width; false=no adjustment//宽度调整：真=自调整字符宽度；假=无调整
//  bShow: true=display background color; false=don't display background color//bShow:true=显示背景色；false=不显示背景色
//  size: Font size//大小：字体大小
//  color: Character color//颜色：字符颜色
//  bColor: Background color//B颜色：背景色
//  x/y: Upper-left coordinate of the string//x/y：字符串的左上角坐标
//  *string: The string//*字符串：字符串
void DWIN_Draw_String(bool widthAdjust, bool bShow, uint8_t size,
                      uint16_t color, uint16_t bColor, uint16_t x, uint16_t y, char *string);

class __FlashStringHelper;

inline void DWIN_Draw_String(bool widthAdjust, bool bShow, uint8_t size, uint16_t color, uint16_t bColor, uint16_t x, uint16_t y, const __FlashStringHelper *title) {
  DWIN_Draw_String(widthAdjust, bShow, size, color, bColor, x, y, (char *)title);
}

// Draw a positive integer//画一个正整数
//  bShow: true=display background color; false=don't display background color//bShow:true=显示背景色；false=不显示背景色
//  zeroFill: true=zero fill; false=no zero fill//零填充：真=零填充；false=无零填充
//  zeroMode: 1=leading 0 displayed as 0; 0=leading 0 displayed as a space//zeroMode:1=前导0显示为0；0=显示为空格的前导0
//  size: Font size//大小：字体大小
//  color: Character color//颜色：字符颜色
//  bColor: Background color//B颜色：背景色
//  iNum: Number of digits//iNum：位数
//  x/y: Upper-left coordinate//x/y：左上角坐标
//  value: Integer value//值：整数值
void DWIN_Draw_IntValue(uint8_t bShow, bool zeroFill, uint8_t zeroMode, uint8_t size, uint16_t color,
                          uint16_t bColor, uint8_t iNum, uint16_t x, uint16_t y, uint16_t value);

// Draw a floating point number//画一个浮点数
//  bShow: true=display background color; false=don't display background color//bShow:true=显示背景色；false=不显示背景色
//  zeroFill: true=zero fill; false=no zero fill//零填充：真=零填充；false=无零填充
//  zeroMode: 1=leading 0 displayed as 0; 0=leading 0 displayed as a space//zeroMode:1=前导0显示为0；0=显示为空格的前导0
//  size: Font size//大小：字体大小
//  color: Character color//颜色：字符颜色
//  bColor: Background color//B颜色：背景色
//  iNum: Number of whole digits//iNum：整位数
//  fNum: Number of decimal digits//fNum：小数位数
//  x/y: Upper-left point//x/y：左上角点
//  value: Float value//值：浮点值
void DWIN_Draw_FloatValue(uint8_t bShow, bool zeroFill, uint8_t zeroMode, uint8_t size, uint16_t color,
                            uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value);

/*---------------------------------------- Picture related functions ----------------------------------------*/

// Draw JPG and cached in #0 virtual display area//绘制JPG并缓存在#0虚拟显示区中
// id: Picture ID//id：图片id
void DWIN_JPG_ShowAndCache(const uint8_t id);

// Draw an Icon//画一个图标
//  libID: Icon library ID//libID：图标库ID
//  picID: Icon ID//picID:Icon-ID
//  x/y: Upper-left point//x/y：左上角点
void DWIN_ICON_Show(uint8_t libID, uint8_t picID, uint16_t x, uint16_t y);

// Unzip the JPG picture to a virtual display area//将JPG图片解压缩到虚拟显示区域
//  n: Cache index//n：缓存索引
//  id: Picture ID//id：图片id
void DWIN_JPG_CacheToN(uint8_t n, uint8_t id);

// Unzip the JPG picture to virtual display area #1//将JPG图片解压缩到虚拟显示区域#1
//  id: Picture ID//id：图片id
inline void DWIN_JPG_CacheTo1(uint8_t id) { DWIN_JPG_CacheToN(1, id); }

// Copy area from virtual display area to current screen//将虚拟显示区域复制到当前屏幕
//  cacheID: virtual area number//cacheID：虚拟区域号
//  xStart/yStart: Upper-left of virtual area//xStart/yStart：虚拟区域的左上角
//  xEnd/yEnd: Lower-right of virtual area//xEnd/yEnd：虚拟区域的右下角
//  x/y: Screen paste point//x/y：屏幕粘贴点
void DWIN_Frame_AreaCopy(uint8_t cacheID, uint16_t xStart, uint16_t yStart,
                         uint16_t xEnd, uint16_t yEnd, uint16_t x, uint16_t y);

// Animate a series of icons//设置一系列图标的动画
//  animID: Animation ID  up to 16//动画ID：动画ID最多为16
//  animate: animation on or off//动画：打开或关闭动画
//  libID: Icon library ID//libID：图标库ID
//  picIDs: Icon starting ID//picid：图标起始ID
//  picIDe: Icon ending ID//picIDe：图标结束ID
//  x/y: Upper-left point//x/y：左上角点
//  interval: Display time interval, unit 10mS//间隔：显示时间间隔，单位为10mS
void DWIN_ICON_Animation(uint8_t animID, bool animate, uint8_t libID, uint8_t picIDs,
                         uint8_t picIDe, uint16_t x, uint16_t y, uint16_t interval);

// Animation Control//动画控制
//  state: 16 bits, each bit is the state of an animation id//状态：16位，每个位是动画id的状态
void DWIN_ICON_AnimationControl(uint16_t state);
