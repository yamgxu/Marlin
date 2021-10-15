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

/**
 * Implementation of the LCD display routines for a TFT GLCD displays with external controller.
 */

#include "../../inc/MarlinConfig.h"

#if IS_TFTGLCD_PANEL

#include "../../libs/duration_t.h"

////////////////////////////////////////////////////////////////////////
// Set up button and encode mappings for each panel (into 'buttons' variable)//为每个面板设置按钮和编码映射（输入“按钮”变量）
////
// This is just to map common functions (across different panels) onto the same//这只是为了将通用功能（跨不同面板）映射到同一个面板上
// macro name. The mapping is independent of whether the button is directly connected or//宏名称。映射与按钮是否直接连接无关
// via a shift/i2c register.//通过移位/i2c寄存器。

////////////////////////////////////////////////////////////////////////
// Create LCD class instance and chipset-specific information//创建LCD类实例和芯片组特定信息
class TFTGLCD {
  private:
  public:
    TFTGLCD();
    void clear_buffer();
    void clr_screen();
    void setCursor(uint8_t col, uint8_t row);
    void write(char c);
    void print(const char *line);
    void print_line();
    void print_screen();
    void redraw_screen();
    void setContrast(uint16_t contrast);
};

extern TFTGLCD lcd;

#include "../fontutils.h"
#include "../lcdprint.h"

// Use panel encoder - free old encoder pins//使用面板编码器-释放旧编码器引脚
#undef BTN_EN1
#undef BTN_EN2
#undef BTN_ENC

#ifndef EN_C
  #define EN_C       4 // for click//点击
#endif

#endif // IS_TFTGLCD_PANEL//是TFTGLCD面板吗
