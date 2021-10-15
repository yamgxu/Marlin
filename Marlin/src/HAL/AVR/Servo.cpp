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
 * servo.cpp - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
 * Copyright (c) 2009 Michael Margolis.  All right reserved.
 */

/**
 * A servo is activated by creating an instance of the Servo class passing the desired pin to the attach() method.
 * The servos are pulsed in the background using the value most recently written using the write() method
 *
 * Note that analogWrite of PWM on pins associated with the timer are disabled when the first servo is attached.
 * Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
 *
 * The methods are:
 *
 * Servo - Class for manipulating servo motors connected to Arduino pins.
 *
 * attach(pin)           - Attach a servo motor to an i/o pin.
 * attach(pin, min, max) - Attach to a pin, setting min and max values in microseconds
 *                         Default min is 544, max is 2400
 *
 * write()               - Set the servo angle in degrees. (Invalid angles —over MIN_PULSE_WIDTH— are treated as µs.)
 * writeMicroseconds()   - Set the servo pulse width in microseconds.
 * move(pin, angle)      - Sequence of attach(pin), write(angle), safe_delay(servo_delay[servoIndex]).
 *                         With DEACTIVATE_SERVOS_AFTER_MOVE it detaches after servo_delay[servoIndex].
 * read()                - Get the last-written servo pulse width as an angle between 0 and 180.
 * readMicroseconds()    - Get the last-written servo pulse width in microseconds.
 * attached()            - Return true if a servo is attached.
 * detach()              - Stop an attached servo from pulsing its i/o pin.
 */

#ifdef __AVR__

#include "../../inc/MarlinConfig.h"

#if HAS_SERVOS

#include <avr/interrupt.h>

#include "../shared/servo.h"
#include "../shared/servo_private.h"

static volatile int8_t Channel[_Nbr_16timers];              // counter for the servo being pulsed for each timer (or -1 if refresh interval)//每个定时器的伺服脉冲计数器（如果刷新间隔为-1）


/************ static functions common to all instances ***********************/

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t* TCNTn, volatile uint16_t* OCRnA) {
  if (Channel[timer] < 0)
    *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer//通道设置为-1表示刷新间隔已完成，因此请重置计时器
  else {
    if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive)
      extDigitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated//如果激活，则将该通道脉冲调低
  }

  Channel[timer]++;    // increment to the next channel//增量到下一个通道
  if (SERVO_INDEX(timer, Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
    *OCRnA = *TCNTn + SERVO(timer, Channel[timer]).ticks;
    if (SERVO(timer, Channel[timer]).Pin.isActive)    // check if activated//检查是否激活
      extDigitalWrite(SERVO(timer, Channel[timer]).Pin.nbr, HIGH); // it's an active channel so pulse it high//这是一个活跃的通道，所以把它调高
  }
  else {
    // finished all channels so wait for the refresh period to expire before starting over//已完成所有通道，请等待刷新周期到期后再重新开始
    if (((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL))    // allow a few ticks to ensure the next OCR1A not missed//允许几次勾选，以确保不会错过下一个OCR1A
      *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
    else
      *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed//至少已过刷新间隔
    Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel//这将在刷新周期结束时递增，以在第一个通道重新开始
  }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform//布线预先定义了信号处理程序，因此不为布线平台定义任何if编译

  // Interrupt handlers for Arduino//Arduino的中断处理程序
  #ifdef _useTimer1
    SIGNAL(TIMER1_COMPA_vect) { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
  #endif

  #ifdef _useTimer3
    SIGNAL(TIMER3_COMPA_vect) { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
  #endif

  #ifdef _useTimer4
    SIGNAL(TIMER4_COMPA_vect) { handle_interrupts(_timer4, &TCNT4, &OCR4A); }
  #endif

  #ifdef _useTimer5
    SIGNAL(TIMER5_COMPA_vect) { handle_interrupts(_timer5, &TCNT5, &OCR5A); }
  #endif

#else // WIRING//接线

  // Interrupt handlers for Wiring//布线中断处理程序
  #ifdef _useTimer1
    void Timer1Service() { handle_interrupts(_timer1, &TCNT1, &OCR1A); }
  #endif
  #ifdef _useTimer3
    void Timer3Service() { handle_interrupts(_timer3, &TCNT3, &OCR3A); }
  #endif

#endif // WIRING//接线

/****************** end of static functions ******************************/

void initISR(timer16_Sequence_t timer) {
  #ifdef _useTimer1
    if (timer == _timer1) {
      TCCR1A = 0;             // normal counting mode//正常计数模式
      TCCR1B = _BV(CS11);     // set prescaler of 8//将预分频器设置为8
      TCNT1 = 0;              // clear the timer count//清除计时器计数
      #if defined(__AVR_ATmega8__) || defined(__AVR_ATmega128__)
        SBI(TIFR, OCF1A);      // clear any pending interrupts;//清除任何挂起的中断；
        SBI(TIMSK, OCIE1A);    // enable the output compare interrupt//启用输出比较中断
      #else
        // here if not ATmega8 or ATmega128//如果不是ATmega8或ATmega128，请点击此处
        SBI(TIFR1, OCF1A);     // clear any pending interrupts;//清除任何挂起的中断；
        SBI(TIMSK1, OCIE1A);   // enable the output compare interrupt//启用输出比较中断
      #endif
      #ifdef WIRING
        timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
      #endif
    }
  #endif

  #ifdef _useTimer3
    if (timer == _timer3) {
      TCCR3A = 0;             // normal counting mode//正常计数模式
      TCCR3B = _BV(CS31);     // set prescaler of 8//将预分频器设置为8
      TCNT3 = 0;              // clear the timer count//清除计时器计数
      #ifdef __AVR_ATmega128__
        SBI(TIFR, OCF3A);     // clear any pending interrupts;//清除任何挂起的中断；
        SBI(ETIMSK, OCIE3A);  // enable the output compare interrupt//启用输出比较中断
      #else
        SBI(TIFR3, OCF3A);   // clear any pending interrupts;//清除任何挂起的中断；
        SBI(TIMSK3, OCIE3A); // enable the output compare interrupt//启用输出比较中断
      #endif
      #ifdef WIRING
        timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only//仅适用于接线平台
      #endif
    }
  #endif

  #ifdef _useTimer4
    if (timer == _timer4) {
      TCCR4A = 0;             // normal counting mode//正常计数模式
      TCCR4B = _BV(CS41);     // set prescaler of 8//将预分频器设置为8
      TCNT4 = 0;              // clear the timer count//清除计时器计数
      TIFR4 = _BV(OCF4A);     // clear any pending interrupts;//清除任何挂起的中断；
      TIMSK4 = _BV(OCIE4A);   // enable the output compare interrupt//启用输出比较中断
    }
  #endif

  #ifdef _useTimer5
    if (timer == _timer5) {
      TCCR5A = 0;             // normal counting mode//正常计数模式
      TCCR5B = _BV(CS51);     // set prescaler of 8//将预分频器设置为8
      TCNT5 = 0;              // clear the timer count//清除计时器计数
      TIFR5 = _BV(OCF5A);     // clear any pending interrupts;//清除任何挂起的中断；
      TIMSK5 = _BV(OCIE5A);   // enable the output compare interrupt//启用输出比较中断
    }
  #endif
}

void finISR(timer16_Sequence_t timer) {
  // Disable use of the given timer//禁用给定计时器的使用
  #ifdef WIRING
    if (timer == _timer1) {
      CBI(
        #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
          TIMSK1
        #else
          TIMSK
        #endif
          , OCIE1A);    // disable timer 1 output compare interrupt//禁用定时器1输出比较中断
      timerDetach(TIMER1OUTCOMPAREA_INT);
    }
    else if (timer == _timer3) {
      CBI(
        #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
          TIMSK3
        #else
          ETIMSK
        #endif
          , OCIE3A);    // disable the timer3 output compare A interrupt//禁用定时器3输出比较中断
      timerDetach(TIMER3OUTCOMPAREA_INT);
    }
  #else // !WIRING// !装电线
    // For arduino - in future: call here to a currently undefined function to reset the timer//对于arduino-将来：在此处调用当前未定义的函数以重置计时器
    UNUSED(timer);
  #endif
}

#endif // HAS_SERVOS//有伺服系统吗

#endif // __AVR__//_uuuavr__
