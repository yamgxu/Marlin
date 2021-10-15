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

#include "../inc/MarlinConfig.h"

#if ((!HAS_ADC_BUTTONS && IS_NEWPANEL) || BUTTONS_EXIST(EN1, EN2)) && !IS_TFTGLCD_PANEL
  #define HAS_ENCODER_WHEEL 1
#endif
#if HAS_ENCODER_WHEEL || ANY_BUTTON(ENC, BACK, UP, DWN, LFT, RT)
  #define HAS_DIGITAL_BUTTONS 1
#endif
#if !HAS_ADC_BUTTONS && (IS_RRW_KEYPAD || (HAS_WIRED_LCD && !IS_NEWPANEL))
  #define HAS_SHIFT_ENCODER 1
#endif

// I2C buttons must be read in the main thread//I2C按钮必须在主线程中读取
#if ANY(LCD_I2C_VIKI, LCD_I2C_PANELOLU2, IS_TFTGLCD_PANEL)
  #define HAS_SLOW_BUTTONS 1
#endif

#if HAS_ENCODER_WHEEL
  #define ENCODER_PHASE_0 0
  #define ENCODER_PHASE_1 2
  #define ENCODER_PHASE_2 3
  #define ENCODER_PHASE_3 1
#endif

#if IS_RRW_KEYPAD
  #define BTN_OFFSET          0 // Bit offset into buttons for shift register values//移位寄存器值按钮的位偏移量

  #define BLEN_KEYPAD_F3      0
  #define BLEN_KEYPAD_F2      1
  #define BLEN_KEYPAD_F1      2
  #define BLEN_KEYPAD_DOWN    3
  #define BLEN_KEYPAD_RIGHT   4
  #define BLEN_KEYPAD_MIDDLE  5
  #define BLEN_KEYPAD_UP      6
  #define BLEN_KEYPAD_LEFT    7

  #define EN_KEYPAD_F1      _BV(BTN_OFFSET + BLEN_KEYPAD_F1)
  #define EN_KEYPAD_F2      _BV(BTN_OFFSET + BLEN_KEYPAD_F2)
  #define EN_KEYPAD_F3      _BV(BTN_OFFSET + BLEN_KEYPAD_F3)
  #define EN_KEYPAD_DOWN    _BV(BTN_OFFSET + BLEN_KEYPAD_DOWN)
  #define EN_KEYPAD_RIGHT   _BV(BTN_OFFSET + BLEN_KEYPAD_RIGHT)
  #define EN_KEYPAD_MIDDLE  _BV(BTN_OFFSET + BLEN_KEYPAD_MIDDLE)
  #define EN_KEYPAD_UP      _BV(BTN_OFFSET + BLEN_KEYPAD_UP)
  #define EN_KEYPAD_LEFT    _BV(BTN_OFFSET + BLEN_KEYPAD_LEFT)

  #define RRK(B) (keypad_buttons & (B))

  #ifdef EN_C
    #define BUTTON_CLICK() ((buttons & EN_C) || RRK(EN_KEYPAD_MIDDLE))
  #else
    #define BUTTON_CLICK() RRK(EN_KEYPAD_MIDDLE)
  #endif
#endif

#if EITHER(HAS_DIGITAL_BUTTONS, DWIN_CREALITY_LCD)
  // Wheel spin pins where BA is 00, 10, 11, 01 (1 bit always changes)//车轮旋转销，其中BA为00、10、11、01（1位始终更改）
  #define BLEN_A 0
  #define BLEN_B 1

  #define EN_A _BV(BLEN_A)
  #define EN_B _BV(BLEN_B)

  #define _BUTTON_PRESSED(BN) !READ(BTN_##BN)

  #if BUTTON_EXISTS(ENC) || HAS_TOUCH_BUTTONS
    #define BLEN_C 2
    #define EN_C _BV(BLEN_C)
  #endif

  #if ENABLED(LCD_I2C_VIKI)
    #include <LiquidTWI2.h>
    #define B_I2C_BTN_OFFSET 3 // (the first three bit positions reserved for EN_A, EN_B, EN_C)//（为EN_A、EN_B、EN_C保留的前三位位置）

    // button and encoder bit positions within 'buttons'//“按钮”内的按钮和编码器位位置
    #define B_LE (BUTTON_LEFT   << B_I2C_BTN_OFFSET)      // The remaining normalized buttons are all read via I2C//其余的标准化按钮都是通过I2C读取的
    #define B_UP (BUTTON_UP     << B_I2C_BTN_OFFSET)
    #define B_MI (BUTTON_SELECT << B_I2C_BTN_OFFSET)
    #define B_DW (BUTTON_DOWN   << B_I2C_BTN_OFFSET)
    #define B_RI (BUTTON_RIGHT  << B_I2C_BTN_OFFSET)

    #if BUTTON_EXISTS(ENC)                                // The pause/stop/restart button is connected to BTN_ENC when used//使用时，暂停/停止/重新启动按钮连接到BTN_ENC
      #define B_ST (EN_C)                                 // Map the pause/stop/resume button into its normalized functional name//将暂停/停止/恢复按钮映射到其规范化的功能名称中
      #define BUTTON_CLICK() (buttons & (B_MI|B_RI|B_ST)) // Pause/stop also acts as click until a proper pause/stop is implemented.//暂停/停止也可作为单击，直到实现正确的暂停/停止。
    #else
      #define BUTTON_CLICK() (buttons & (B_MI|B_RI))
    #endif

    // I2C buttons take too long to read inside an interrupt context and so we read them during lcd_update//I2C按钮在中断上下文中的读取时间太长，因此我们在lcd_更新期间读取它们

  #elif ENABLED(LCD_I2C_PANELOLU2)
    #if !BUTTON_EXISTS(ENC) // Use I2C if not directly connected to a pin//如果未直接连接到引脚，则使用I2C
      #define B_I2C_BTN_OFFSET 3 // (the first three bit positions reserved for EN_A, EN_B, EN_C)//（为EN_A、EN_B、EN_C保留的前三位位置）

      #define B_MI (PANELOLU2_ENCODER_C << B_I2C_BTN_OFFSET) // requires LiquidTWI2 library v1.2.3 or later//需要LiquidTWI2库v1.2.3或更高版本

      #define BUTTON_CLICK() (buttons & B_MI)
    #endif
  #endif
#else
  #undef BUTTON_EXISTS
  #define BUTTON_EXISTS(...) false

  // Dummy button, never pressed//虚拟按钮，从未按下
  #define _BUTTON_PRESSED(BN) false

  // Shift register bits correspond to buttons://移位寄存器位对应于按钮：
  #define BL_LE 7   // Left//左
  #define BL_UP 6   // Up//向上
  #define BL_MI 5   // Middle//中间的
  #define BL_DW 4   // Down//向下
  #define BL_RI 3   // Right//对
  #define BL_ST 2   // Red Button//红色按钮
  #define B_LE _BV(BL_LE)
  #define B_UP _BV(BL_UP)
  #define B_MI _BV(BL_MI)
  #define B_DW _BV(BL_DW)
  #define B_RI _BV(BL_RI)
  #define B_ST _BV(BL_ST)

  #ifndef BUTTON_CLICK
    #define BUTTON_CLICK() (buttons & (B_MI|B_ST))
  #endif
#endif

#ifndef EN_A
  #define EN_A 0
#endif
#ifndef EN_B
  #define EN_B 0
#endif
#ifndef EN_C
  #define EN_C 0
#endif
#if BUTTON_EXISTS(BACK) || EITHER(HAS_TOUCH_BUTTONS, IS_TFTGLCD_PANEL)
  #define BLEN_D 3
  #define EN_D _BV(BLEN_D)
#else
  #define EN_D 0
#endif

#define BUTTON_PRESSED(BN) (_BUTTON_PRESSED_##BN)

#if BUTTON_EXISTS(EN1)
  #define _BUTTON_PRESSED_EN1 _BUTTON_PRESSED(EN1)
#else
  #define _BUTTON_PRESSED_EN1 false
#endif
#if BUTTON_EXISTS(EN2)
  #define _BUTTON_PRESSED_EN2 _BUTTON_PRESSED(EN2)
#else
  #define _BUTTON_PRESSED_EN2 false
#endif
#if BUTTON_EXISTS(ENC_EN)
  #define _BUTTON_PRESSED_ENC_EN _BUTTON_PRESSED(ENC_EN)
#else
  #define _BUTTON_PRESSED_ENC_EN false
#endif
#if BUTTON_EXISTS(ENC)
  #define _BUTTON_PRESSED_ENC _BUTTON_PRESSED(ENC)
#else
  #define _BUTTON_PRESSED_ENC false
#endif
#if BUTTON_EXISTS(UP)
  #define _BUTTON_PRESSED_UP _BUTTON_PRESSED(UP)
#else
  #define _BUTTON_PRESSED_UP false
#endif
#if BUTTON_EXISTS(DWN)
  #define _BUTTON_PRESSED_DWN _BUTTON_PRESSED(DWN)
#else
  #define _BUTTON_PRESSED_DWN false
#endif
#if BUTTON_EXISTS(LFT)
  #define _BUTTON_PRESSED_LFT _BUTTON_PRESSED(LFT)
#else
  #define _BUTTON_PRESSED_LFT false
#endif
#if BUTTON_EXISTS(RT)
  #define _BUTTON_PRESSED_RT _BUTTON_PRESSED(RT)
#else
  #define _BUTTON_PRESSED_RT false
#endif
#if BUTTON_EXISTS(BACK)
  #define _BUTTON_PRESSED_BACK _BUTTON_PRESSED(BACK)
#else
  #define _BUTTON_PRESSED_BACK false
#endif

#ifndef BUTTON_CLICK
  #if EN_C > 0
    #define BUTTON_CLICK() (buttons & EN_C)
  #else
    #define BUTTON_CLICK() false
  #endif
#endif

#if EN_D > 0
  #define LCD_BACK_CLICKED() (buttons & EN_D)
#else
  #define LCD_BACK_CLICKED() false
#endif
