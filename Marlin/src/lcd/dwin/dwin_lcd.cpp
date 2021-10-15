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

/********************************************************************************
 * @file     dwin_lcd.cpp
 * @author   LEO / Creality3D
 * @date     2019/07/18
 * @version  2.0.1
 * @brief    DWIN screen control functions
 ********************************************************************************/

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_CREALITY_LCD)

#include "../../inc/MarlinConfig.h"

#include "dwin_lcd.h"
#include <string.h> // for memset//用于memset

//#define DEBUG_OUT 1//#定义调试输出1
#include "../../core/debug_out.h"

// Make sure DWIN_SendBuf is large enough to hold the largest string plus draw command and tail.//确保DWIN_SendBuf足够大，可以容纳最大的字符串加上draw命令和tail。
// Assume the narrowest (6 pixel) font and 2-byte gb2312-encoded characters.//采用最窄（6像素）字体和2字节gb2312编码字符。
uint8_t DWIN_SendBuf[11 + DWIN_WIDTH / 6 * 2] = { 0xAA };
uint8_t DWIN_BufTail[4] = { 0xCC, 0x33, 0xC3, 0x3C };
uint8_t databuf[26] = { 0 };
uint8_t receivedType;

int recnum = 0;

inline void DWIN_Byte(size_t &i, const uint16_t bval) {
  DWIN_SendBuf[++i] = bval;
}

inline void DWIN_Word(size_t &i, const uint16_t wval) {
  DWIN_SendBuf[++i] = wval >> 8;
  DWIN_SendBuf[++i] = wval & 0xFF;
}

inline void DWIN_Long(size_t &i, const uint32_t lval) {
  DWIN_SendBuf[++i] = (lval >> 24) & 0xFF;
  DWIN_SendBuf[++i] = (lval >> 16) & 0xFF;
  DWIN_SendBuf[++i] = (lval >>  8) & 0xFF;
  DWIN_SendBuf[++i] = lval & 0xFF;
}

inline void DWIN_String(size_t &i, char * const string) {
  const size_t len = _MIN(sizeof(DWIN_SendBuf) - i, strlen(string));
  memcpy(&DWIN_SendBuf[i+1], string, len);
  i += len;
}

inline void DWIN_String(size_t &i, const __FlashStringHelper * string) {
  if (!string) return;
  const size_t len = strlen_P((PGM_P)string); // cast it to PGM_P, which is basically const char *, and measure it using the _P version of strlen.//将其转换为PGM_P，基本上是const char*，并使用strlen的_P版本进行测量。
  if (len == 0) return;
  memcpy(&DWIN_SendBuf[i+1], string, len);
  i += len;
}

// Send the data in the buffer and the packet end//在缓冲区和数据包端发送数据
inline void DWIN_Send(size_t &i) {
  ++i;
  LOOP_L_N(n, i) { LCD_SERIAL.write(DWIN_SendBuf[n]); delayMicroseconds(1); }
  LOOP_L_N(n, 4) { LCD_SERIAL.write(DWIN_BufTail[n]); delayMicroseconds(1); }
}

/*-------------------------------------- System variable function --------------------------------------*/

// Handshake (1: Success, 0: Fail)//握手（1:成功，0:失败）
bool DWIN_Handshake(void) {
  #ifndef LCD_BAUDRATE
    #define LCD_BAUDRATE 115200
  #endif
  LCD_SERIAL.begin(LCD_BAUDRATE);
  const millis_t serial_connect_timeout = millis() + 1000UL;
  while (!LCD_SERIAL.connected() && PENDING(millis(), serial_connect_timeout)) { /*nada*/ }

  size_t i = 0;
  DWIN_Byte(i, 0x00);
  DWIN_Send(i);

  while (LCD_SERIAL.available() > 0 && recnum < (signed)sizeof(databuf)) {
    databuf[recnum] = LCD_SERIAL.read();
    // ignore the invalid data//忽略无效数据
    if (databuf[0] != FHONE) { // prevent the program from running.//阻止程序运行。
      if (recnum > 0) {
        recnum = 0;
        ZERO(databuf);
      }
      continue;
    }
    delay(10);
    recnum++;
  }

  return ( recnum >= 3
        && databuf[0] == FHONE
        && databuf[1] == '\0'
        && databuf[2] == 'O'
        && databuf[3] == 'K' );
}

// Set the backlight luminance//设置背光亮度
//  luminance: (0x00-0xFF)//亮度：（0x00-0xFF）
void DWIN_Backlight_SetLuminance(const uint8_t luminance) {
  size_t i = 0;
  DWIN_Byte(i, 0x30);
  DWIN_Byte(i, _MAX(luminance, 0x1F));
  DWIN_Send(i);
}

// Set screen display direction//设置屏幕显示方向
//  dir: 0=0°, 1=90°, 2=180°, 3=270°//方向：0=0°，1=90°，2=180°，3=270°
void DWIN_Frame_SetDir(uint8_t dir) {
  size_t i = 0;
  DWIN_Byte(i, 0x34);
  DWIN_Byte(i, 0x5A);
  DWIN_Byte(i, 0xA5);
  DWIN_Byte(i, dir);
  DWIN_Send(i);
}

// Update display//更新显示
void DWIN_UpdateLCD(void) {
  size_t i = 0;
  DWIN_Byte(i, 0x3D);
  DWIN_Send(i);
}

/*---------------------------------------- Drawing functions ----------------------------------------*/

// Clear screen//清屏
//  color: Clear screen color//颜色：清晰的屏幕颜色
void DWIN_Frame_Clear(const uint16_t color) {
  size_t i = 0;
  DWIN_Byte(i, 0x01);
  DWIN_Word(i, color);
  DWIN_Send(i);
}

// Draw a point//划清界限
//  width: point width   0x01-0x0F//宽度：点宽度0x01-0x0F
//  height: point height 0x01-0x0F//高度：点高度0x01-0x0F
//  x,y: upper left point//x，y：左上角点
void DWIN_Draw_Point(uint8_t width, uint8_t height, uint16_t x, uint16_t y) {
  size_t i = 0;
  DWIN_Byte(i, 0x02);
  DWIN_Byte(i, width);
  DWIN_Byte(i, height);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  DWIN_Send(i);
}

// Draw a line//划一条线
//  color: Line segment color//颜色：线段颜色
//  xStart/yStart: Start point//xStart/yStart：起点
//  xEnd/yEnd: End point//结束/结束：结束点
void DWIN_Draw_Line(uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
  size_t i = 0;
  DWIN_Byte(i, 0x03);
  DWIN_Word(i, color);
  DWIN_Word(i, xStart);
  DWIN_Word(i, yStart);
  DWIN_Word(i, xEnd);
  DWIN_Word(i, yEnd);
  DWIN_Send(i);
}

// Draw a rectangle//画一个长方形
//  mode: 0=frame, 1=fill, 2=XOR fill//模式：0=帧，1=填充，2=异或填充
//  color: Rectangle color//颜色：矩形颜色
//  xStart/yStart: upper left point//xStart/yStart：左上角点
//  xEnd/yEnd: lower right point//xEnd/yEnd：右下角点
void DWIN_Draw_Rectangle(uint8_t mode, uint16_t color,
                         uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
  size_t i = 0;
  DWIN_Byte(i, 0x05);
  DWIN_Byte(i, mode);
  DWIN_Word(i, color);
  DWIN_Word(i, xStart);
  DWIN_Word(i, yStart);
  DWIN_Word(i, xEnd);
  DWIN_Word(i, yEnd);
  DWIN_Send(i);
}

// Move a screen area//移动屏幕区域
//  mode: 0, circle shift; 1, translation//模式：0，循环移位；1、翻译
//  dir: 0=left, 1=right, 2=up, 3=down//方向：0=左，1=右，2=上，3=下
//  dis: Distance//dis：距离
//  color: Fill color//颜色：填充颜色
//  xStart/yStart: upper left point//xStart/yStart：左上角点
//  xEnd/yEnd: bottom right point//xEnd/yEnd：右下角点
void DWIN_Frame_AreaMove(uint8_t mode, uint8_t dir, uint16_t dis,
                         uint16_t color, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
  size_t i = 0;
  DWIN_Byte(i, 0x09);
  DWIN_Byte(i, (mode << 7) | dir);
  DWIN_Word(i, dis);
  DWIN_Word(i, color);
  DWIN_Word(i, xStart);
  DWIN_Word(i, yStart);
  DWIN_Word(i, xEnd);
  DWIN_Word(i, yEnd);
  DWIN_Send(i);
}

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
                      uint16_t color, uint16_t bColor, uint16_t x, uint16_t y, char *string) {
  size_t i = 0;
  DWIN_Byte(i, 0x11);
  // Bit 7: widthAdjust//第7位：宽度调整
  // Bit 6: bShow//第6位：bShow
  // Bit 5-4: Unused (0)//第5-4位：未使用（0）
  // Bit 3-0: size//第3-0位：尺寸
  DWIN_Byte(i, (widthAdjust * 0x80) | (bShow * 0x40) | size);
  DWIN_Word(i, color);
  DWIN_Word(i, bColor);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  DWIN_String(i, string);
  DWIN_Send(i);
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
                          uint16_t bColor, uint8_t iNum, uint16_t x, uint16_t y, uint16_t value) {
  size_t i = 0;
  DWIN_Byte(i, 0x14);
  // Bit 7: bshow//第7位：bshow
  // Bit 6: 1 = signed; 0 = unsigned number;//第6位：1=有符号；0=无符号数；
  // Bit 5: zeroFill//第5位：零填充
  // Bit 4: zeroMode//第4位：零模式
  // Bit 3-0: size//第3-0位：尺寸
  DWIN_Byte(i, (bShow * 0x80) | (zeroFill * 0x20) | (zeroMode * 0x10) | size);
  DWIN_Word(i, color);
  DWIN_Word(i, bColor);
  DWIN_Byte(i, iNum);
  DWIN_Byte(i, 0); // fNum//fNum
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  #if 0
    for (char count = 0; count < 8; count++) {
      DWIN_Byte(i, value);
      value >>= 8;
      if (!(value & 0xFF)) break;
    }
  #else
    // Write a big-endian 64 bit integer//写一个大端64位整数
    const size_t p = i + 1;
    for (char count = 8; count--;) { // 7..0// 7..0
      ++i;
      DWIN_SendBuf[p + count] = value;
      value >>= 8;
    }
  #endif

  DWIN_Send(i);
}

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
                            uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value) {
  //uint8_t *fvalue = (uint8_t*)&value;//uint8_t*fvalue=（uint8_t*）和value；
  size_t i = 0;
  DWIN_Byte(i, 0x14);
  DWIN_Byte(i, (bShow * 0x80) | (zeroFill * 0x20) | (zeroMode * 0x10) | size);
  DWIN_Word(i, color);
  DWIN_Word(i, bColor);
  DWIN_Byte(i, iNum);
  DWIN_Byte(i, fNum);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  DWIN_Long(i, value);
  /*
  DWIN_Byte(i, fvalue[3]);
  DWIN_Byte(i, fvalue[2]);
  DWIN_Byte(i, fvalue[1]);
  DWIN_Byte(i, fvalue[0]);
  */
  DWIN_Send(i);
}

/*---------------------------------------- Picture related functions ----------------------------------------*/

// Draw JPG and cached in #0 virtual display area//绘制JPG并缓存在#0虚拟显示区中
// id: Picture ID//id：图片id
void DWIN_JPG_ShowAndCache(const uint8_t id) {
  size_t i = 0;
  DWIN_Word(i, 0x2200);
  DWIN_Byte(i, id);
  DWIN_Send(i);     // AA 23 00 00 00 00 08 00 01 02 03 CC 33 C3 3C//AA 23 00 00 08 00 01 02 03 CC 33 C3 3C
}

// Draw an Icon//画一个图标
//  libID: Icon library ID//libID：图标库ID
//  picID: Icon ID//picID:Icon-ID
//  x/y: Upper-left point//x/y：左上角点
void DWIN_ICON_Show(uint8_t libID, uint8_t picID, uint16_t x, uint16_t y) {
  NOMORE(x, DWIN_WIDTH - 1);
  NOMORE(y, DWIN_HEIGHT - 1); // -- ozy -- srl//--ozy--srl
  size_t i = 0;
  DWIN_Byte(i, 0x23);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  DWIN_Byte(i, 0x80 | libID);
  DWIN_Byte(i, picID);
  DWIN_Send(i);
}

// Unzip the JPG picture to a virtual display area//将JPG图片解压缩到虚拟显示区域
//  n: Cache index//n：缓存索引
//  id: Picture ID//id：图片id
void DWIN_JPG_CacheToN(uint8_t n, uint8_t id) {
  size_t i = 0;
  DWIN_Byte(i, 0x25);
  DWIN_Byte(i, n);
  DWIN_Byte(i, id);
  DWIN_Send(i);
}

// Copy area from virtual display area to current screen//将虚拟显示区域复制到当前屏幕
//  cacheID: virtual area number//cacheID：虚拟区域号
//  xStart/yStart: Upper-left of virtual area//xStart/yStart：虚拟区域的左上角
//  xEnd/yEnd: Lower-right of virtual area//xEnd/yEnd：虚拟区域的右下角
//  x/y: Screen paste point//x/y：屏幕粘贴点
void DWIN_Frame_AreaCopy(uint8_t cacheID, uint16_t xStart, uint16_t yStart,
                         uint16_t xEnd, uint16_t yEnd, uint16_t x, uint16_t y) {
  size_t i = 0;
  DWIN_Byte(i, 0x27);
  DWIN_Byte(i, 0x80 | cacheID);
  DWIN_Word(i, xStart);
  DWIN_Word(i, yStart);
  DWIN_Word(i, xEnd);
  DWIN_Word(i, yEnd);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  DWIN_Send(i);
}

// Animate a series of icons//设置一系列图标的动画
//  animID: Animation ID; 0x00-0x0F//animID：动画ID；0x00-0x0F
//  animate: true on; false off;//动画：在上为真；假关；
//  libID: Icon library ID//libID：图标库ID
//  picIDs: Icon starting ID//picid：图标起始ID
//  picIDe: Icon ending ID//picIDe：图标结束ID
//  x/y: Upper-left point//x/y：左上角点
//  interval: Display time interval, unit 10mS//间隔：显示时间间隔，单位为10mS
void DWIN_ICON_Animation(uint8_t animID, bool animate, uint8_t libID, uint8_t picIDs, uint8_t picIDe, uint16_t x, uint16_t y, uint16_t interval) {
  NOMORE(x, DWIN_WIDTH - 1);
  NOMORE(y, DWIN_HEIGHT - 1); // -- ozy -- srl//--ozy--srl
  size_t i = 0;
  DWIN_Byte(i, 0x28);
  DWIN_Word(i, x);
  DWIN_Word(i, y);
  // Bit 7: animation on or off//位7：动画打开或关闭
  // Bit 6: start from begin or end//第6位：从开始或结束开始
  // Bit 5-4: unused (0)//第5-4位：未使用（0）
  // Bit 3-0: animID//第3-0位：animID
  DWIN_Byte(i, (animate * 0x80) | 0x40 | animID);
  DWIN_Byte(i, libID);
  DWIN_Byte(i, picIDs);
  DWIN_Byte(i, picIDe);
  DWIN_Byte(i, interval);
  DWIN_Send(i);
}

// Animation Control//动画控制
//  state: 16 bits, each bit is the state of an animation id//状态：16位，每个位是动画id的状态
void DWIN_ICON_AnimationControl(uint16_t state) {
  size_t i = 0;
  DWIN_Byte(i, 0x28);
  DWIN_Word(i, state);
  DWIN_Send(i);
}

/*---------------------------------------- Memory functions ----------------------------------------*/
// The LCD has an additional 32KB SRAM and 16KB Flash//LCD有额外的32KB SRAM和16KB闪存

// Data can be written to the sram and save to one of the jpeg page files//数据可以写入sram并保存到其中一个jpeg页面文件

// Write Data Memory//写入数据存储器
//  command 0x31//命令0x31
//  Type: Write memory selection; 0x5A=SRAM; 0xA5=Flash//类型：写入存储器选择；0x5A=SRAM；0xA5=闪光
//  Address: Write data memory address; 0x000-0x7FFF for SRAM; 0x000-0x3FFF for Flash//地址：写入数据存储器地址；用于SRAM的0x000-0x7FFF；用于闪存的0x000-0x3FFF
//  Data: data//数据：数据
////
//  Flash writing returns 0xA5 0x4F 0x4B//闪存写入返回0xA5 0x4F 0x4B

// Read Data Memory//读取数据存储器
//  command 0x32//命令0x32
//  Type: Read memory selection; 0x5A=SRAM; 0xA5=Flash//类型：读存储器选择；0x5A=SRAM；0xA5=闪光
//  Address: Read data memory address; 0x000-0x7FFF for SRAM; 0x000-0x3FFF for Flash//地址：读取数据存储器地址；用于SRAM的0x000-0x7FFF；用于闪存的0x000-0x3FFF
//  Length: leangth of data to read; 0x01-0xF0//长度：要读取的数据的长度；0x01-0xF0
////
//  Response://答复：
//    Type, Address, Length, Data//类型、地址、长度、数据

// Write Picture Memory//写图片存储器
//  Write the contents of the 32KB SRAM data memory into the designated image memory space//将32KB SRAM数据存储器的内容写入指定的图像存储器空间
//  Issued: 0x5A, 0xA5, PIC_ID//发布：0x5A、0xA5，图片编号
//  Response: 0xA5 0x4F 0x4B//响应：0xA5 0x4F 0x4B
////
//  command 0x33//命令0x33
//  0x5A, 0xA5//0x5A，0xA5
//  PicId: Picture Memory location, 0x00-0x0F//PicId：图片内存位置，0x00-0x0F
////
//  Flash writing returns 0xA5 0x4F 0x4B//闪存写入返回0xA5 0x4F 0x4B

#endif // DWIN_CREALITY_LCD//DWIN_CREALITY_液晶显示器
