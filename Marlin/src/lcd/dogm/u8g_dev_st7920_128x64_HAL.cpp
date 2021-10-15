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

/**
 * Based on u8g_dev_st7920_128x64.c
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2011, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list
 *    of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB && DISABLED(TFT_CLASSIC_UI)

#include "HAL_LCD_com_defines.h"

#define PAGE_HEIGHT        8

/* init sequence from https://github.com/adafruit/ST7565-LCD/blob/master/ST7565/ST7565.cpp *///github.com/adafruit/ST7565-LCD/blob/master/ST7565/ST7565.cpp*/
static const uint8_t u8g_dev_st7920_128x64_HAL_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),      // disable chip//禁用芯片
  U8G_ESC_ADR(0),     // instruction mode//教学模式
  U8G_ESC_RST(15),    // do reset low pulse with (15*16)+2 milliseconds (=maximum delay)//用（15*16）+2毫秒（=最大延迟）重置低脉冲
  U8G_ESC_DLY(100),   // 8 Dez 2012: additional delay 100 ms because of reset//8 Dez 2012：重置导致额外延迟100毫秒
  U8G_ESC_CS(1),      // enable chip//使能芯片
  U8G_ESC_DLY(50),    // delay 50 ms//延迟50毫秒

  0x038,              // 8 Bit interface (DL=1), basic instruction set (RE=0)//8位接口（DL=1），基本指令集（RE=0）
  0x00C,              // display on, cursor & blink off; 0x08: all off//显示打开、光标和闪烁关闭；0x08：全部关闭
  0x006,              // Entry mode: Cursor move to right, DDRAM address counter (AC) plus 1, no shift//输入模式：光标向右移动，DDRAM地址计数器（AC）加1，不移位
  0x002,              // disable scroll, enable CGRAM adress//禁用滚动，启用CGRAM地址
  0x001,              // clear RAM, needs 1.6 ms//清除RAM，需要1.6毫秒
  U8G_ESC_DLY(100),   // delay 100 ms//延迟100毫秒

  U8G_ESC_CS(0),      // disable chip//禁用芯片
  U8G_ESC_END         // end of sequence//序列结束
};

void clear_graphics_DRAM(u8g_t *u8g, u8g_dev_t *dev) {
  u8g_SetChipSelect(u8g, dev, 1);
  u8g_Delay(1);
  u8g_SetAddress(u8g, dev, 0);         // cmd mode//命令模式
  u8g_WriteByte(u8g, dev, 0x08);       //display off, cursor+blink off//显示关闭，光标+闪烁关闭
  u8g_WriteByte(u8g, dev, 0x3E);       //extended mode + GDRAM active//扩展模式+GDRAM激活
  LOOP_L_N(y, (LCD_PIXEL_HEIGHT) / 2) { //clear GDRAM//清除GDRAM
    u8g_WriteByte(u8g, dev, 0x80 | y); //set y//设置y
    u8g_WriteByte(u8g, dev, 0x80);     //set x = 0//设置x=0
    u8g_SetAddress(u8g, dev, 1);                  /* data mode */
    LOOP_L_N(i, 2 * (LCD_PIXEL_WIDTH) / 8) //2x width clears both segments//2x宽度清除两个段
      u8g_WriteByte(u8g, dev, 0);
    u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
  }

  u8g_WriteByte(u8g, dev, 0x0C); //display on, cursor+blink off//显示打开，光标+闪烁关闭

  u8g_SetChipSelect(u8g, dev, 0);
}

uint8_t u8g_dev_st7920_128x64_HAL_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  switch (msg) {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_HAL_init_seq);
      clear_graphics_DRAM(u8g, dev);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT: {
      uint8_t y, i;
      uint8_t *ptr;
      u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

      u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
      u8g_SetChipSelect(u8g, dev, 1);
      y = pb->p.page_y0;
      ptr = (uint8_t *)pb->buf;
      for (i = 0; i < 8; i ++) {
        u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
        u8g_WriteByte(u8g, dev, 0x03E );      /* enable extended mode */

        if (y < 32) {
          u8g_WriteByte(u8g, dev, 0x080 | y );      /* y pos  */
          u8g_WriteByte(u8g, dev, 0x080  );      /* set x pos to 0*/
        }
        else {
          u8g_WriteByte(u8g, dev, 0x080 | (y-32) );      /* y pos  */
          u8g_WriteByte(u8g, dev, 0x080 | 8);      /* set x pos to 64*/
        }

        u8g_SetAddress(u8g, dev, 1);                  /* data mode */
        u8g_WriteSequence(u8g, dev, (LCD_PIXEL_WIDTH) / 8, ptr);
        ptr += (LCD_PIXEL_WIDTH) / 8;
        y++;
      }
      u8g_SetChipSelect(u8g, dev, 0);
    }
    break;
  }
  return u8g_dev_pb8h1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_st7920_128x64_HAL_4x_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  switch (msg) {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_400NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_st7920_128x64_HAL_init_seq);
      clear_graphics_DRAM(u8g, dev);
      break;

    case U8G_DEV_MSG_STOP:
      break;

    case U8G_DEV_MSG_PAGE_NEXT: {
      uint8_t y, i;
      uint8_t *ptr;
      u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

      u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
      u8g_SetChipSelect(u8g, dev, 1);
      y = pb->p.page_y0;
      ptr = (uint8_t *)pb->buf;
      for (i = 0; i < 32; i ++) {
        u8g_SetAddress(u8g, dev, 0);           /* cmd mode */
        u8g_WriteByte(u8g, dev, 0x03E );      /* enable extended mode */

        if (y < 32) {
          u8g_WriteByte(u8g, dev, 0x080 | y );      /* y pos  */
          u8g_WriteByte(u8g, dev, 0x080  );      /* set x pos to 0*/
        }
        else {
          u8g_WriteByte(u8g, dev, 0x080 | (y-32) );      /* y pos  */
          u8g_WriteByte(u8g, dev, 0x080 | 8);      /* set x pos to 64*/
        }

        u8g_SetAddress(u8g, dev, 1);                  /* data mode */
        u8g_WriteSequence(u8g, dev, (LCD_PIXEL_WIDTH) / 8, ptr);
        ptr += (LCD_PIXEL_WIDTH) / 8;
        y++;
      }
      u8g_SetChipSelect(u8g, dev, 0);
    }
    break;
  }
  return u8g_dev_pb32h1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_dev_st7920_128x64_HAL_sw_spi, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_HAL_fn, U8G_COM_ST7920_HAL_SW_SPI);

#define QWIDTH ((LCD_PIXEL_WIDTH) * 4)
uint8_t u8g_dev_st7920_128x64_HAL_4x_buf[QWIDTH] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_st7920_128x64_HAL_4x_pb = { { 32, LCD_PIXEL_HEIGHT, 0, 0, 0 }, LCD_PIXEL_WIDTH, u8g_dev_st7920_128x64_HAL_4x_buf};
u8g_dev_t u8g_dev_st7920_128x64_HAL_4x_sw_spi = { u8g_dev_st7920_128x64_HAL_4x_fn, &u8g_dev_st7920_128x64_HAL_4x_pb, U8G_COM_ST7920_HAL_SW_SPI };

U8G_PB_DEV(u8g_dev_st7920_128x64_HAL_hw_spi, LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT, PAGE_HEIGHT, u8g_dev_st7920_128x64_HAL_fn, U8G_COM_ST7920_HAL_HW_SPI);
u8g_dev_t u8g_dev_st7920_128x64_HAL_4x_hw_spi = { u8g_dev_st7920_128x64_HAL_4x_fn, &u8g_dev_st7920_128x64_HAL_4x_pb, U8G_COM_ST7920_HAL_HW_SPI };

#if NONE(__AVR__, ARDUINO_ARCH_STM32, ARDUINO_ARCH_ESP32) || defined(U8G_HAL_LINKS)
  // Also use this device for HAL version of rrd class. This results in the same device being used//也可将此设备用于rrd类的HAL版本。这将导致使用相同的设备
  // for the ST7920 for HAL systems no matter what is selected in marlinui_DOGM.h.//无论marlinui_DOGM.h.中选择了什么，都适用于HAL系统的ST7920。
  u8g_dev_t u8g_dev_st7920_128x64_rrd_sw_spi = { u8g_dev_st7920_128x64_HAL_4x_fn, &u8g_dev_st7920_128x64_HAL_4x_pb, U8G_COM_ST7920_HAL_SW_SPI };
#endif

#endif // HAS_MARLINUI_U8GLIB//马林努伊能说会道吗
