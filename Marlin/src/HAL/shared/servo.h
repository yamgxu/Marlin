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
 * servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 * Copyright (c) 2009 Michael Margolis.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
 * A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 * The servos are pulsed in the background using the value most recently written using the write() method
 *
 * Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 * Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 * The sequence used to seize timers is defined in timers.h
 *
 * The methods are:
 *
 *  Servo - Class for manipulating servo motors connected to Arduino pins.
 *
 *  attach(pin )  - Attaches a servo motor to an i/o pin.
 *  attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
 *  default min is 544, max is 2400
 *
 *  write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
 *  writeMicroseconds() - Sets the servo pulse width in microseconds
 *  read()      - Gets the last written servo pulse width as an angle between 0 and 180.
 *  readMicroseconds() - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
 *  attached()  - Returns true if there is a servo attached.
 *  detach()    - Stops an attached servos from pulsing its i/o pin.
 *  move(angle) - Sequence of attach(0), write(angle),
 *                   With DEACTIVATE_SERVOS_AFTER_MOVE wait SERVO_DELAY and detach.
 */

#if IS_TEENSY32
  #include "../TEENSY31_32/Servo.h"
#elif IS_TEENSY35 || IS_TEENSY36
  #include "../TEENSY35_36/Servo.h"
#elif IS_TEENSY40 || IS_TEENSY41
  #include "../TEENSY40_41/Servo.h"
#elif defined(TARGET_LPC1768)
  #include "../LPC1768/Servo.h"
#elif defined(__STM32F1__) || defined(TARGET_STM32F1)
  #include "../STM32F1/Servo.h"
#elif defined(ARDUINO_ARCH_STM32)
  #include "../STM32/Servo.h"
#elif defined(ARDUINO_ARCH_ESP32)
  #include "../ESP32/Servo.h"
#else
  #include <stdint.h>

  #if defined(__AVR__) || defined(ARDUINO_ARCH_SAM) || defined(__SAMD51__)
    // we're good to go//我们可以走了
  #else
    #error "This library only supports boards with an AVR, SAM3X or SAMD51 processor."
  #endif

  #define Servo_VERSION           2     // software version of this library//此库的软件版本

  class Servo {
    public:
      Servo();
      int8_t attach(const int pin);      // attach the given pin to the next free channel, set pinMode, return channel number (-1 on fail)//将给定管脚连接到下一个自由通道，设置管脚模式，返回通道编号（-1，故障时）
      int8_t attach(const int pin, const int min, const int max); // as above but also sets min and max values for writes.//如上所述，但也设置写入的最小值和最大值。
      void detach();
      void write(int value);             // if value is < 200 it is treated as an angle, otherwise as pulse width in microseconds//如果值小于200，则将其视为角度，否则视为脉冲宽度（以微秒为单位）
      void writeMicroseconds(int value); // write pulse width in microseconds//以微秒为单位写入脉冲宽度
      void move(const int value);        // attach the servo, then move to value//连接伺服，然后移动到“值”
                                         // if value is < 200 it is treated as an angle, otherwise as pulse width in microseconds//如果值小于200，则将其视为角度，否则视为脉冲宽度（以微秒为单位）
                                         // if DEACTIVATE_SERVOS_AFTER_MOVE wait SERVO_DELAY, then detach//如果在移动等待伺服延迟后停用伺服，则分离
      int read();                        // returns current pulse width as an angle between 0 and 180 degrees//以0到180度之间的角度返回当前脉冲宽度
      int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)//返回此伺服的当前脉冲宽度（以微秒为单位）（在第一个版本中已读取）
      bool attached();                   // return true if this servo is attached, otherwise false//如果已连接此伺服，则返回true，否则返回false

    private:
      uint8_t servoIndex;               // index into the channel data for this servo//索引到此伺服的通道数据
      int8_t min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH//最小值是该值乘以4乘以最小脉冲宽度
      int8_t max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH//最大值是该值乘以4加上最大脉冲宽度
  };

#endif
