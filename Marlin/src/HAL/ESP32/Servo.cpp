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
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfig.h"

#if HAS_SERVOS

#include "Servo.h"

// Adjacent channels (0/1, 2/3 etc.) share the same timer and therefore the same frequency and resolution settings on ESP32,//相邻通道（0/1、2/3等）共享相同的计时器，因此ESP32上的频率和分辨率设置相同，
// so we only allocate servo channels up high to avoid side effects with regards to analogWrite (fans, leds, laser pwm etc.)//因此，我们仅将伺服通道分配到较高的位置，以避免与模拟写入相关的副作用（风扇、LED、激光pwm等）
int Servo::channel_next_free = LEDC_CHANNEL_7; // TODO refactor this to share ledc code with Hal.cpp analogwrite//TODO重构此文件以与Hal.cpp analogwrite共享ledc代码

Servo::Servo() {
  channel = channel_next_free++;
}

int8_t Servo::attach(const int inPin) {
  if (channel >= CHANNEL_MAX_NUM) return -1;
  if (inPin > 0) pin = inPin;

  // TODO *80 is a workaround for arduino-esp32 #5375//TODO*80是arduino-esp32#5375的解决方案
  ledcSetup(channel, 50 * 80, LEDC_TIMER_14_BIT); // channel X, 50 Hz, 16-bit depth // todo ESp32-s2 max resolution is 14 bits//通道X，50 Hz，16位深度//todo ESp32-s2最大分辨率为14位
  ledcAttachPin(pin, channel);
  return true;
}

void Servo::detach() { ledcDetachPin(pin); }

int Servo::read() { return degrees; }

void Servo::write(int inDegrees) {
  degrees = constrain(inDegrees, MIN_ANGLE, MAX_ANGLE);
  int us = map(degrees, MIN_ANGLE, MAX_ANGLE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  int duty = map(us, 0, TAU_USEC, 0, MAX_COMPARE);
  ledcWrite(channel, duty);
}

void Servo::move(const int value) {
  constexpr uint16_t servo_delay[] = SERVO_DELAY;
  static_assert(COUNT(servo_delay) == NUM_SERVOS, "SERVO_DELAY must be an array NUM_SERVOS long.");
  if (attach(0) >= 0) {
    write(value);
    safe_delay(servo_delay[channel]);
    TERN_(DEACTIVATE_SERVOS_AFTER_MOVE, detach());
  }
}
#endif // HAS_SERVOS//有伺服系统吗

#endif // ARDUINO_ARCH_ESP32//ARDUINO_ARCH_ESP32
