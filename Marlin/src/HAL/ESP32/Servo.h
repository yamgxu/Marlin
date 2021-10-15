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

#include <Arduino.h>
#include "driver/ledc.h"

class Servo {
  static const int MIN_ANGLE =   0,
                   MAX_ANGLE = 180,
                   MIN_PULSE_WIDTH =  544,  // Shortest pulse sent to a servo//发送到伺服系统的最短脉冲
                   MAX_PULSE_WIDTH = 2400,  // Longest pulse sent to a servo//发送到伺服的最长脉冲
                   TAU_MSEC = 20,
                   TAU_USEC = (TAU_MSEC * 1000),
                   MAX_COMPARE = _BV(LEDC_TIMER_14_BIT) - 1, // 65535// 65535
                   CHANNEL_MAX_NUM = LEDC_CHANNEL_MAX;

public:
  Servo();
  int8_t attach(const int pin);   // attach the given pin to the next free channel, set pinMode, return channel number (-1 on fail)//将给定管脚连接到下一个自由通道，设置管脚模式，返回通道编号（-1，故障时）
  void detach();
  void write(int degrees);        // set angle//设定角度
  void move(const int degrees);   // attach the servo, then move to value//连接伺服，然后移动到“值”
  int read();                     // returns current pulse width as an angle between 0 and 180 degrees//以0到180度之间的角度返回当前脉冲宽度

private:
  static int channel_next_free;
  int channel;
  int pin;
  int degrees;
};
