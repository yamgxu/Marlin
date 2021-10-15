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
 * This module is off by default, but can be enabled to facilitate the display of
 * extra debug information during code development.
 *
 * Just connect up 5V and GND to give it power, then connect up the pins assigned
 * in Configuration_adv.h. For example, on the Re-ARM you could use:
 *
 *   #define MAX7219_CLK_PIN   77
 *   #define MAX7219_DIN_PIN   78
 *   #define MAX7219_LOAD_PIN  79
 *
 * max7219.init() is called automatically at startup, and then there are a number of
 * support functions available to control the LEDs in the 8x8 grid.
 *
 * If you are using the Max7219 matrix for firmware debug purposes in time sensitive
 * areas of the code, please be aware that the orientation (rotation) of the display can
 * affect the speed.   The Max7219 can update a single column fairly fast.  It is much
 * faster to do a Max7219_Set_Column() with a rotation of 90 or 270 degrees than to do
 * a Max7219_Set_Row().    The opposite is true for rotations of 0 or 180 degrees.
 */

#ifndef MAX7219_ROTATE
  #define MAX7219_ROTATE 0
#endif
#define _ROT ((MAX7219_ROTATE + 360) % 360)

#ifndef MAX7219_NUMBER_UNITS
  #define MAX7219_NUMBER_UNITS 1
#endif
#define MAX7219_LINES (8 * (MAX7219_NUMBER_UNITS))

////
// MAX7219 registers//MAX7219寄存器
////
#define max7219_reg_noop        0x00
#define max7219_reg_digit0      0x01
#define max7219_reg_digit1      0x02
#define max7219_reg_digit2      0x03
#define max7219_reg_digit3      0x04
#define max7219_reg_digit4      0x05
#define max7219_reg_digit5      0x06
#define max7219_reg_digit6      0x07
#define max7219_reg_digit7      0x08

#define max7219_reg_decodeMode  0x09
#define max7219_reg_intensity   0x0A
#define max7219_reg_scanLimit   0x0B
#define max7219_reg_shutdown    0x0C
#define max7219_reg_displayTest 0x0F

class Max7219 {
public:
  static uint8_t led_line[MAX7219_LINES];

  Max7219() {}

  static void init();
  static void register_setup();
  static void putbyte(uint8_t data);
  static void pulse_load();

  // Set a single register (e.g., a whole native row)//设置单个寄存器（例如，整个本机行）
  static void send(const uint8_t reg, const uint8_t data);

  // Refresh all units//刷新所有单元
  static inline void refresh() { for (uint8_t i = 0; i < 8; i++) refresh_line(i); }

  // Suspend / resume updates to the LED unit//暂停/恢复对LED单元的更新
  // Use these methods to speed up multiple changes//使用这些方法可以加速多个更改
  // or to apply updates from interrupt context.//或者从中断上下文应用更新。
  static inline void suspend() { suspended++; }
  static inline void resume() { suspended--; suspended |= 0x80; }

  // Update a single native line on all units//在所有单元上更新单个本机行
  static void refresh_line(const uint8_t line);

  // Update a single native line on just one unit//仅在一台设备上更新一条本机线路
  static void refresh_unit_line(const uint8_t line);

  #if ENABLED(MAX7219_NUMERIC)
    // Draw an integer with optional leading zeros and optional decimal point//绘制一个带可选前导零和可选小数点的整数
    void print(const uint8_t start, int16_t value, uint8_t size, const bool leadzero=false, bool dec=false);
    // Draw a float with a decimal point and optional digits//绘制带有小数点和可选数字的浮点
    void print(const uint8_t start, const_float_t value, const uint8_t pre_size, const uint8_t post_size, const bool leadzero=false);
  #endif

  // Set a single LED by XY coordinate//通过XY坐标设置单个LED
  static void led_set(const uint8_t x, const uint8_t y, const bool on);
  static void led_on(const uint8_t x, const uint8_t y);
  static void led_off(const uint8_t x, const uint8_t y);
  static void led_toggle(const uint8_t x, const uint8_t y);

  // Set all LEDs in a single column//在一列中设置所有LED
  static void set_column(const uint8_t col, const uint32_t val);
  static void clear_column(const uint8_t col);

  // Set all LEDs in a single row//将所有LED设置为一行
  static void set_row(const uint8_t row, const uint32_t val);
  static void clear_row(const uint8_t row);

  // 16 and 32 bit versions of Row and Column functions//行和列函数的16位和32位版本
  // Multiple rows and columns will be used to display the value if//如果需要，将使用多行和多列来显示值
  // the array of matrix LED's is too narrow to accomplish the goal//矩阵LED阵列太窄，无法实现此目标
  static void set_rows_16bits(const uint8_t y, uint32_t val);
  static void set_rows_32bits(const uint8_t y, uint32_t val);
  static void set_columns_16bits(const uint8_t x, uint32_t val);
  static void set_columns_32bits(const uint8_t x, uint32_t val);

  // Quickly clear the whole matrix//快速清除整个矩阵
  static void clear();

  // Quickly fill the whole matrix//快速填充整个矩阵
  static void fill();

  // Apply custom code to update the matrix//应用自定义代码更新矩阵
  static void idle_tasks();

private:
  static uint8_t suspended;
  static void error(const char * const func, const int32_t v1, const int32_t v2=-1);
  static void noop();
  static void set(const uint8_t line, const uint8_t bits);
  static void send_row(const uint8_t row);
  static void send_column(const uint8_t col);
  static void mark16(const uint8_t y, const uint8_t v1, const uint8_t v2);
  static void range16(const uint8_t y, const uint8_t ot, const uint8_t nt, const uint8_t oh, const uint8_t nh);
  static void quantity16(const uint8_t y, const uint8_t ov, const uint8_t nv);

  #ifdef MAX7219_INIT_TEST
    static void test_pattern();
    static void run_test_pattern();
    static void start_test_pattern();
  #endif
};

extern Max7219 max7219;
