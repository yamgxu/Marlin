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
 * MarlinSerial.cpp - Hardware serial library for Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Modified 23 November 2006 by David A. Mellis
 * Modified 28 September 2010 by Mark Sproul
 * Modified 14 February 2016 by Andreas Hardtung (added tx buffer)
 * Modified 01 October 2017 by Eduardo José Tagle (added XON/XOFF)
 * Modified 10 June 2018 by Eduardo José Tagle (See #10991)
 * Templatized 01 October 2018 by Eduardo José Tagle to allow multiple instances
 */

#ifdef __AVR__

// Disable HardwareSerial.cpp to support chips without a UART (Attiny, etc.)//禁用HardwareSerial.cpp以支持没有UART（地址等）的芯片

#include "../../inc/MarlinConfig.h"

#if !defined(USBCON) && (defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H))

#include "MarlinSerial.h"
#include "../../MarlinCore.h"

#if ENABLED(DIRECT_STEPPING)
  #include "../../feature/direct_stepping.h"
#endif

template<typename Cfg> typename MarlinSerial<Cfg>::ring_buffer_r MarlinSerial<Cfg>::rx_buffer = { 0, 0, { 0 } };
template<typename Cfg> typename MarlinSerial<Cfg>::ring_buffer_t MarlinSerial<Cfg>::tx_buffer = { 0 };
template<typename Cfg> bool     MarlinSerial<Cfg>::_written = false;
template<typename Cfg> uint8_t  MarlinSerial<Cfg>::xon_xoff_state = MarlinSerial<Cfg>::XON_XOFF_CHAR_SENT | MarlinSerial<Cfg>::XON_CHAR;
template<typename Cfg> uint8_t  MarlinSerial<Cfg>::rx_dropped_bytes = 0;
template<typename Cfg> uint8_t  MarlinSerial<Cfg>::rx_buffer_overruns = 0;
template<typename Cfg> uint8_t  MarlinSerial<Cfg>::rx_framing_errors = 0;
template<typename Cfg> typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::rx_max_enqueued = 0;

// A SW memory barrier, to ensure GCC does not overoptimize loops//软件内存屏障，确保GCC不会过度优化循环
#define sw_barrier() asm volatile("": : :"memory");

#include "../../feature/e_parser.h"

// "Atomically" read the RX head index value without disabling interrupts://在不禁用中断的情况下，“原子”读取RX磁头索引值：
// This MUST be called with RX interrupts enabled, and CAN'T be called//这必须在启用RX中断的情况下调用，不能调用
// from the RX ISR itself!//来自接收ISR本身！
template<typename Cfg>
FORCE_INLINE typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::atomic_read_rx_head() {
  if (Cfg::RX_SIZE > 256) {
    // Keep reading until 2 consecutive reads return the same value,//继续读取，直到两次连续读取返回相同的值，
    // meaning there was no update in-between caused by an interrupt.//这意味着中间没有由中断引起的更新。
    // This works because serial RX interrupts happen at a slower rate//这是因为串行接收中断发生的速度较慢
    // than successive reads of a variable, so 2 consecutive reads with//比一个变量的连续读取多，因此使用
    // the same value means no interrupt updated it.//相同的值表示没有中断更新它。
    ring_buffer_pos_t vold, vnew = rx_buffer.head;
    sw_barrier();
    do {
      vold = vnew;
      vnew = rx_buffer.head;
      sw_barrier();
    } while (vold != vnew);
    return vnew;
  }
  else {
    // With an 8bit index, reads are always atomic. No need for special handling//对于8位索引，读取总是原子的。无需特殊处理
    return rx_buffer.head;
  }
}

template<typename Cfg>
volatile bool MarlinSerial<Cfg>::rx_tail_value_not_stable = false;
template<typename Cfg>
volatile uint16_t MarlinSerial<Cfg>::rx_tail_value_backup = 0;

// Set RX tail index, taking into account the RX ISR could interrupt//考虑到RX ISR可能中断，设置RX尾部索引
//  the write to this variable in the middle - So a backup strategy//在中间写这个变量-所以备份策略
//  is used to ensure reads of the correct values.//用于确保读取正确的值。
//    -Must NOT be called from the RX ISR -//-不得从接收ISR调用-
template<typename Cfg>
FORCE_INLINE void MarlinSerial<Cfg>::atomic_set_rx_tail(typename MarlinSerial<Cfg>::ring_buffer_pos_t value) {
  if (Cfg::RX_SIZE > 256) {
    // Store the new value in the backup//将新值存储在备份中
    rx_tail_value_backup = value;
    sw_barrier();
    // Flag we are about to change the true value//标志我们将要更改真实值
    rx_tail_value_not_stable = true;
    sw_barrier();
    // Store the new value//存储新值
    rx_buffer.tail = value;
    sw_barrier();
    // Signal the new value is completely stored into the value//信号新值已完全存储到值中
    rx_tail_value_not_stable = false;
    sw_barrier();
  }
  else
    rx_buffer.tail = value;
}

// Get the RX tail index, taking into account the read could be//获取RX尾部索引，考虑到读取可能
//  interrupting in the middle of the update of that index value//在该索引值的更新中间中断
//    -Called from the RX ISR -//-从接收ISR呼叫-
template<typename Cfg>
FORCE_INLINE typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::atomic_read_rx_tail() {
  if (Cfg::RX_SIZE > 256) {
    // If the true index is being modified, return the backup value//如果修改的是true索引，则返回备份值
    if (rx_tail_value_not_stable) return rx_tail_value_backup;
  }
  // The true index is stable, return it//真正的索引是稳定的，返回它
  return rx_buffer.tail;
}

// (called with RX interrupts disabled)//（在禁用RX中断的情况下调用）
template<typename Cfg>
FORCE_INLINE void MarlinSerial<Cfg>::store_rxd_char() {

  static EmergencyParser::State emergency_state; // = EP_RESET//=EP_重置

  // This must read the R_UCSRA register before reading the received byte to detect error causes//在读取接收到的字节以检测错误原因之前，必须先读取R_UCSRA寄存器
  if (Cfg::DROPPED_RX && B_DOR && !++rx_dropped_bytes) --rx_dropped_bytes;
  if (Cfg::RX_OVERRUNS && B_DOR && !++rx_buffer_overruns) --rx_buffer_overruns;
  if (Cfg::RX_FRAMING_ERRORS && B_FE && !++rx_framing_errors) --rx_framing_errors;

  // Read the character from the USART//从USART读取字符
  uint8_t c = R_UDR;

  #if ENABLED(DIRECT_STEPPING)
    if (page_manager.maybe_store_rxd_char(c)) return;
  #endif

  // Get the tail - Nothing can alter its value while this ISR is executing, but there's//获取尾部-在执行此ISR时，任何东西都不能改变其值，但是
  // a chance that this ISR interrupted the main process while it was updating the index.//此ISR在更新索引时中断主进程的可能性。
  // The backup mechanism ensures the correct value is always returned.//备份机制确保始终返回正确的值。
  const ring_buffer_pos_t t = atomic_read_rx_tail();

  // Get the head pointer - This ISR is the only one that modifies its value, so it's safe to read here//获取头部指针-此ISR是唯一一个修改其值的ISR，因此在此处阅读是安全的
  ring_buffer_pos_t h = rx_buffer.head;

  // Get the next element//获取下一个元素
  ring_buffer_pos_t i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

  if (Cfg::EMERGENCYPARSER) emergency_parser.update(emergency_state, c);

  // If the character is to be stored at the index just before the tail//如果要将字符存储在尾部之前的索引中
  // (such that the head would advance to the current tail), the RX FIFO is//（这样头部将前进到当前尾部），RX FIFO
  // full, so don't write the character or advance the head.//满了，所以不要写字符或推进头部。
  if (i != t) {
    rx_buffer.buffer[h] = c;
    h = i;
  }
  else if (Cfg::DROPPED_RX && !++rx_dropped_bytes)
    --rx_dropped_bytes;

  if (Cfg::MAX_RX_QUEUED) {
    // Calculate count of bytes stored into the RX buffer//计算存储到RX缓冲区的字节数
    const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

    // Keep track of the maximum count of enqueued bytes//跟踪排队字节的最大计数
    NOLESS(rx_max_enqueued, rx_count);
  }

  if (Cfg::XONOFF) {
    // If the last char that was sent was an XON//如果发送的最后一个字符是XON
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XON_CHAR) {

      // Bytes stored into the RX buffer//存储在RX缓冲区中的字节
      const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

      // If over 12.5% of RX buffer capacity, send XOFF before running out of//如果超过RX缓冲区容量的12.5%，则在用完之前发送XOFF
      // RX buffer space .. 325 bytes @ 250kbits/s needed to let the host react//接收缓冲区空间。。需要325字节@250kbits/s才能让主机做出反应
      // and stop sending bytes. This translates to 13mS propagation time.//并停止发送字节。这相当于13毫秒的传播时间。
      if (rx_count >= (Cfg::RX_SIZE) / 8) {

        // At this point, definitely no TX interrupt was executing, since the TX ISR can't be preempted.//在这一点上，肯定没有执行TX中断，因为TX ISR不能被抢占。
        // Don't enable the TX interrupt here as a means to trigger the XOFF char, because if it happens//不要在这里启用TX中断作为触发XOFF字符的手段，因为如果它发生
        // to be in the middle of trying to disable the RX interrupt in the main program, eventually the//试图在主程序中禁用RX中断，最终
        // enabling of the TX interrupt could be undone. The ONLY reliable thing this can do to ensure//TX中断的启用可以撤消。这是唯一可靠的保证
        // the sending of the XOFF char is to send it HERE AND NOW.//XOFF字符的发送就是在此时此地发送。

        // About to send the XOFF char//即将发送XOFF字符
        xon_xoff_state = XOFF_CHAR | XON_XOFF_CHAR_SENT;

        // Wait until the TX register becomes empty and send it - Here there could be a problem//等待TX寄存器变为空并发送-此处可能有问题
        // - While waiting for the TX register to empty, the RX register could receive a new//-在等待TX寄存器清空时，RX寄存器可接收新的
        //   character. This must also handle that situation!//性格。这也必须处理这种情况！
        while (!B_UDRE) {

          if (B_RXC) {
            // A char arrived while waiting for the TX buffer to be empty - Receive and process it!//在等待TX缓冲区为空时到达一个字符-接收并处理它！

            i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

            // Read the character from the USART//从USART读取字符
            c = R_UDR;

            if (Cfg::EMERGENCYPARSER) emergency_parser.update(emergency_state, c);

            // If the character is to be stored at the index just before the tail//如果要将字符存储在尾部之前的索引中
            // (such that the head would advance to the current tail), the FIFO is//（这样头部将前进到当前尾部），FIFO是
            // full, so don't write the character or advance the head.//满了，所以不要写字符或推进头部。
            if (i != t) {
              rx_buffer.buffer[h] = c;
              h = i;
            }
            else if (Cfg::DROPPED_RX && !++rx_dropped_bytes)
              --rx_dropped_bytes;
          }
          sw_barrier();
        }

        R_UDR = XOFF_CHAR;

        // Clear the TXC bit -- "can be cleared by writing a one to its bit//清除TXC位--“可通过将一写入其位来清除
        // location". This makes sure flush() won't return until the bytes//地点”。这将确保flush（）在返回字节之前不会返回
        // actually got written//真的写了
        B_TXC = 1;

        // At this point there could be a race condition between the write() function//此时，write（）函数之间可能存在竞争条件
        // and this sending of the XOFF char. This interrupt could happen between the//这是XOFF字符的发送。此中断可能发生在
        // wait to be empty TX buffer loop and the actual write of the character. Since//等待TX缓冲区循环和字符的实际写入为空。自从
        // the TX buffer is full because it's sending the XOFF char, the only way to be//TX缓冲区已满，因为它正在发送XOFF字符，这是唯一的恢复方法
        // sure the write() function will succeed is to wait for the XOFF char to be//确保write（）函数成功的方法是等待XOFF字符被删除
        // completely sent. Since an extra character could be received during the wait//完全发送。因为在等待过程中可能会收到一个额外的字符
        // it must also be handled!//它也必须被处理！
        while (!B_UDRE) {

          if (B_RXC) {
            // A char arrived while waiting for the TX buffer to be empty - Receive and process it!//在等待TX缓冲区为空时到达一个字符-接收并处理它！

            i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

            // Read the character from the USART//从USART读取字符
            c = R_UDR;

            if (Cfg::EMERGENCYPARSER)
              emergency_parser.update(emergency_state, c);

            // If the character is to be stored at the index just before the tail//如果要将字符存储在尾部之前的索引中
            // (such that the head would advance to the current tail), the FIFO is//（这样头部将前进到当前尾部），FIFO是
            // full, so don't write the character or advance the head.//满了，所以不要写字符或推进头部。
            if (i != t) {
              rx_buffer.buffer[h] = c;
              h = i;
            }
            else if (Cfg::DROPPED_RX && !++rx_dropped_bytes)
              --rx_dropped_bytes;
          }
          sw_barrier();
        }

        // At this point everything is ready. The write() function won't//现在一切都准备好了。write（）函数将不起作用
        // have any issues writing to the UART TX register if it needs to!//如果需要，写入UART TX寄存器有任何问题！
      }
    }
  }

  // Store the new head value - The main loop will retry until the value is stable//存储新的头值-主循环将重试，直到值稳定
  rx_buffer.head = h;
}

// (called with TX irqs disabled)//（在禁用TX IRQ的情况下调用）
template<typename Cfg>
FORCE_INLINE void MarlinSerial<Cfg>::_tx_udr_empty_irq() {
  if (Cfg::TX_SIZE > 0) {
    // Read positions//读取位置
    uint8_t t = tx_buffer.tail;
    const uint8_t h = tx_buffer.head;

    if (Cfg::XONOFF) {
      // If an XON char is pending to be sent, do it now//如果一个XON字符等待发送，现在就发送
      if (xon_xoff_state == XON_CHAR) {

        // Send the character//发送字符
        R_UDR = XON_CHAR;

        // clear the TXC bit -- "can be cleared by writing a one to its bit//清除TXC位--“可通过将一写入其位来清除
        // location". This makes sure flush() won't return until the bytes//地点”。这将确保flush（）在返回字节之前不会返回
        // actually got written//真的写了
        B_TXC = 1;

        // Remember we sent it.//记得我们送的。
        xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;

        // If nothing else to transmit, just disable TX interrupts.//如果没有其他传输，只需禁用TX中断。
        if (h == t) B_UDRIE = 0; // (Non-atomic, could be reenabled by the main program, but eventually this will succeed)//（非原子的，可以由主程序重新启用，但最终会成功）

        return;
      }
    }

    // If nothing to transmit, just disable TX interrupts. This could//如果没有传输，只需禁用TX中断。这可能
    // happen as the result of the non atomicity of the disabling of RX//由于RX禁用的非原子性而发生
    // interrupts that could end reenabling TX interrupts as a side effect.//作为副作用，可能结束重新烧蚀TX中断的中断。
    if (h == t) {
      B_UDRIE = 0; // (Non-atomic, could be reenabled by the main program, but eventually this will succeed)//（非原子的，可以由主程序重新启用，但最终会成功）
      return;
    }

    // There is something to TX, Send the next byte//有东西要发送，发送下一个字节
    const uint8_t c = tx_buffer.buffer[t];
    t = (t + 1) & (Cfg::TX_SIZE - 1);
    R_UDR = c;
    tx_buffer.tail = t;

    // Clear the TXC bit (by writing a one to its bit location).//清除TXC位（通过将一写入其位位置）。
    // Ensures flush() won't return until the bytes are actually written///确保在实际写入字节之前，flush（）不会返回/
    B_TXC = 1;

    // Disable interrupts if there is nothing to transmit following this byte//如果在该字节后没有要传输的内容，则禁用中断
    if (h == t) B_UDRIE = 0; // (Non-atomic, could be reenabled by the main program, but eventually this will succeed)//（非原子的，可以由主程序重新启用，但最终会成功）
  }
}

// Public Methods//公共方法
template<typename Cfg>
void MarlinSerial<Cfg>::begin(const long baud) {
  uint16_t baud_setting;
  bool useU2X = true;

  #if F_CPU == 16000000UL && SERIAL_PORT == 0
    // Hard-coded exception for compatibility with the bootloader shipped//与所提供的引导加载程序兼容的硬编码异常
    // with the Duemilanove and previous boards, and the firmware on the//使用Duemilanove和以前的板，以及
    // 8U2 on the Uno and Mega 2560.//Uno上的8U2和Mega 2560。
    if (baud == 57600) useU2X = false;
  #endif

  R_UCSRA = 0;
  if (useU2X) {
    B_U2X = 1;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  }
  else
    baud_setting = (F_CPU / 8 / baud - 1) / 2;

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)//分配波特率设置，也称为ubbr（USART波特率寄存器）
  R_UBRRH = baud_setting >> 8;
  R_UBRRL = baud_setting;

  B_RXEN = 1;
  B_TXEN = 1;
  B_RXCIE = 1;
  if (Cfg::TX_SIZE > 0) B_UDRIE = 0;
  _written = false;
}

template<typename Cfg>
void MarlinSerial<Cfg>::end() {
  B_RXEN = 0;
  B_TXEN = 0;
  B_RXCIE = 0;
  B_UDRIE = 0;
}

template<typename Cfg>
int MarlinSerial<Cfg>::peek() {
  const ring_buffer_pos_t h = atomic_read_rx_head(), t = rx_buffer.tail;
  return h == t ? -1 : rx_buffer.buffer[t];
}

template<typename Cfg>
int MarlinSerial<Cfg>::read() {
  const ring_buffer_pos_t h = atomic_read_rx_head();

  // Read the tail. Main thread owns it, so it is safe to directly read it//读尾巴。主线程拥有它，因此直接读取它是安全的
  ring_buffer_pos_t t = rx_buffer.tail;

  // If nothing to read, return now//如果没有要阅读的内容，请立即返回
  if (h == t) return -1;

  // Get the next char//获取下一个字符
  const int v = rx_buffer.buffer[t];
  t = (ring_buffer_pos_t)(t + 1) & (Cfg::RX_SIZE - 1);

  // Advance tail - Making sure the RX ISR will always get an stable value, even//提前跟踪-确保接收ISR始终获得稳定值，即使
  // if it interrupts the writing of the value of that variable in the middle.//如果它中断了中间变量的值的写入。
  atomic_set_rx_tail(t);

  if (Cfg::XONOFF) {
    // If the XOFF char was sent, or about to be sent...//如果XOFF字符已发送或即将发送。。。
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
      // Get count of bytes in the RX buffer//获取接收缓冲区中的字节计数
      const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);
      if (rx_count < (Cfg::RX_SIZE) / 10) {
        if (Cfg::TX_SIZE > 0) {
          // Signal we want an XON character to be sent.//我们希望发送一个XON字符的信号。
          xon_xoff_state = XON_CHAR;
          // Enable TX ISR. Non atomic, but it will eventually enable them//启用发送ISR。非原子的，但它最终将使它们成为可能
          B_UDRIE = 1;
        }
        else {
          // If not using TX interrupts, we must send the XON char now//如果不使用TX中断，我们必须立即发送XON字符
          xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
          while (!B_UDRE) sw_barrier();
          R_UDR = XON_CHAR;
        }
      }
    }
  }

  return v;
}

template<typename Cfg>
typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::available() {
  const ring_buffer_pos_t h = atomic_read_rx_head(), t = rx_buffer.tail;
  return (ring_buffer_pos_t)(Cfg::RX_SIZE + h - t) & (Cfg::RX_SIZE - 1);
}

template<typename Cfg>
void MarlinSerial<Cfg>::flush() {

  // Set the tail to the head://将尾部设置为头部：
  //  - Read the RX head index in a safe way. (See atomic_read_rx_head.)//-以安全的方式读取接收头索引。（参见原子读取接收头。）
  //  - Set the tail, making sure the RX ISR will always get a stable value, even//-设置尾部，确保RX ISR始终获得稳定值，即使
  //    if it interrupts the writing of the value of that variable in the middle.//如果它中断了中间变量的值的写入。
  atomic_set_rx_tail(atomic_read_rx_head());

  if (Cfg::XONOFF) {
    // If the XOFF char was sent, or about to be sent...//如果XOFF字符已发送或即将发送。。。
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
      if (Cfg::TX_SIZE > 0) {
        // Signal we want an XON character to be sent.//我们希望发送一个XON字符的信号。
        xon_xoff_state = XON_CHAR;
        // Enable TX ISR. Non atomic, but it will eventually enable it.//启用发送ISR。非原子的，但它最终会启用它。
        B_UDRIE = 1;
      }
      else {
        // If not using TX interrupts, we must send the XON char now//如果不使用TX中断，我们必须立即发送XON字符
        xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
        while (!B_UDRE) sw_barrier();
        R_UDR = XON_CHAR;
      }
    }
  }
}

template<typename Cfg>
void MarlinSerial<Cfg>::write(const uint8_t c) {
  if (Cfg::TX_SIZE == 0) {

    _written = true;
    while (!B_UDRE) sw_barrier();
    R_UDR = c;

  }
  else {

    _written = true;

    // If the TX interrupts are disabled and the data register//如果TX中断被禁用，并且数据寄存器
    // is empty, just write the byte to the data register and//为空，只需将字节写入数据寄存器并
    // be done. This shortcut helps significantly improve the//完成。此快捷方式有助于显著提高性能
    // effective datarate at high (>500kbit/s) bitrates, where//高（>500kbit/s）比特率下的有效数据速率，其中
    // interrupt overhead becomes a slowdown.//中断开销变慢。
    // Yes, there is a race condition between the sending of the//是的，在发送
    // XOFF char at the RX ISR, but it is properly handled there//RX ISR处的XOFF char，但在那里得到了正确处理
    if (!B_UDRIE && B_UDRE) {
      R_UDR = c;

      // clear the TXC bit -- "can be cleared by writing a one to its bit//清除TXC位--“可通过将一写入其位来清除
      // location". This makes sure flush() won't return until the bytes//地点”。这将确保flush（）在返回字节之前不会返回
      // actually got written//真的写了
      B_TXC = 1;
      return;
    }

    const uint8_t i = (tx_buffer.head + 1) & (Cfg::TX_SIZE - 1);

    // If global interrupts are disabled (as the result of being called from an ISR)...//如果全局中断被禁用（由于从ISR调用）。。。
    if (!ISRS_ENABLED()) {

      // Make room by polling if it is possible to transmit, and do so!//如果可以传输，请通过轮询留出空间，并这样做！
      while (i == tx_buffer.tail) {

        // If we can transmit another byte, do it.//如果我们可以传输另一个字节，就这样做。
        if (B_UDRE) _tx_udr_empty_irq();

        // Make sure compiler rereads tx_buffer.tail//确保编译器重新读取tx_buffer.tail
        sw_barrier();
      }
    }
    else {
      // Interrupts are enabled, just wait until there is space//中断已启用，只需等待有空间
      while (i == tx_buffer.tail) sw_barrier();
    }

    // Store new char. head is always safe to move//存储新字符。头部始终可以安全移动
    tx_buffer.buffer[tx_buffer.head] = c;
    tx_buffer.head = i;

    // Enable TX ISR - Non atomic, but it will eventually enable TX ISR//启用TX ISR-非原子，但最终将启用TX ISR
    B_UDRIE = 1;
  }
}

template<typename Cfg>
void MarlinSerial<Cfg>::flushTX() {

  if (Cfg::TX_SIZE == 0) {
    // No bytes written, no need to flush. This special case is needed since there's//无需写入字节，无需刷新。这个特殊情况是需要的，因为
    // no way to force the TXC (transmit complete) bit to 1 during initialization.//初始化期间，无法强制TXC（传输完成）位为1。
    if (!_written) return;

    // Wait until everything was transmitted//等到一切都传送出去
    while (!B_TXC) sw_barrier();

    // At this point nothing is queued anymore (DRIE is disabled) and//此时，不再排队（DRIE被禁用），并且
    // the hardware finished transmission (TXC is set).//硬件完成传输（设置TXC）。

  }
  else {

    // No bytes written, no need to flush. This special case is needed since there's//无需写入字节，无需刷新。这个特殊情况是需要的，因为
    // no way to force the TXC (transmit complete) bit to 1 during initialization.//初始化期间，无法强制TXC（传输完成）位为1。
    if (!_written) return;

    // If global interrupts are disabled (as the result of being called from an ISR)...//如果全局中断被禁用（由于从ISR调用）。。。
    if (!ISRS_ENABLED()) {

      // Wait until everything was transmitted - We must do polling, as interrupts are disabled//等到一切都被传输了，我们必须进行轮询，因为中断被禁用了
      while (tx_buffer.head != tx_buffer.tail || !B_TXC) {

        // If there is more space, send an extra character//如果有更多的空间，请发送一个额外的字符
        if (B_UDRE) _tx_udr_empty_irq();

        sw_barrier();
      }

    }
    else {
      // Wait until everything was transmitted//等到一切都传送出去
      while (tx_buffer.head != tx_buffer.tail || !B_TXC) sw_barrier();
    }

    // At this point nothing is queued anymore (DRIE is disabled) and//此时，不再排队（DRIE被禁用），并且
    // the hardware finished transmission (TXC is set).//硬件完成传输（设置TXC）。
  }
}

// Hookup ISR handlers//连接ISR处理程序
ISR(SERIAL_REGNAME(USART, SERIAL_PORT, _RX_vect)) {
  MarlinSerial<MarlinSerialCfg<SERIAL_PORT>>::store_rxd_char();
}

ISR(SERIAL_REGNAME(USART, SERIAL_PORT, _UDRE_vect)) {
  MarlinSerial<MarlinSerialCfg<SERIAL_PORT>>::_tx_udr_empty_irq();
}

// Because of the template definition above, it's required to instantiate the template to have all methods generated//由于上面的模板定义，需要实例化模板以生成所有方法
template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT> >;
MSerialT1 customizedSerial1(MSerialT1::HasEmergencyParser);

#ifdef SERIAL_PORT_2

  // Hookup ISR handlers//连接ISR处理程序
  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_2, _RX_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_2>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_2, _UDRE_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_2>>::_tx_udr_empty_irq();
  }

  template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT_2> >;
  MSerialT2 customizedSerial2(MSerialT2::HasEmergencyParser);

#endif // SERIAL_PORT_2//串行端口2

#ifdef SERIAL_PORT_3

  // Hookup ISR handlers//连接ISR处理程序
  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _RX_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, SERIAL_PORT_3, _UDRE_vect)) {
    MarlinSerial<MarlinSerialCfg<SERIAL_PORT_3>>::_tx_udr_empty_irq();
  }

  template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT_3> >;
  MSerialT3 customizedSerial3(MSerialT3::HasEmergencyParser);

#endif // SERIAL_PORT_3//串行端口3

#ifdef MMU2_SERIAL_PORT

  ISR(SERIAL_REGNAME(USART, MMU2_SERIAL_PORT, _RX_vect)) {
    MarlinSerial<MMU2SerialCfg<MMU2_SERIAL_PORT>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, MMU2_SERIAL_PORT, _UDRE_vect)) {
    MarlinSerial<MMU2SerialCfg<MMU2_SERIAL_PORT>>::_tx_udr_empty_irq();
  }

  template class MarlinSerial< MMU2SerialCfg<MMU2_SERIAL_PORT> >;
  MSerialMMU2 mmuSerial(MSerialMMU2::HasEmergencyParser);

#endif // MMU2_SERIAL_PORT//MMU2_串行_端口

#ifdef LCD_SERIAL_PORT

  ISR(SERIAL_REGNAME(USART, LCD_SERIAL_PORT, _RX_vect)) {
    MarlinSerial<LCDSerialCfg<LCD_SERIAL_PORT>>::store_rxd_char();
  }

  ISR(SERIAL_REGNAME(USART, LCD_SERIAL_PORT, _UDRE_vect)) {
    MarlinSerial<LCDSerialCfg<LCD_SERIAL_PORT>>::_tx_udr_empty_irq();
  }

  template class MarlinSerial< LCDSerialCfg<LCD_SERIAL_PORT> >;
  MSerialLCD lcdSerial(MSerialLCD::HasEmergencyParser);

  #if HAS_DGUS_LCD
    template<typename Cfg>
    typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::get_tx_buffer_free() {
      const ring_buffer_pos_t t = tx_buffer.tail,  // next byte to send.//下一个要发送的字节。
                              h = tx_buffer.head;  // next pos for queue.//排队的下一个pos。
      int ret = t - h - 1;
      if (ret < 0) ret += Cfg::TX_SIZE + 1;
      return ret;
    }
  #endif

#endif // LCD_SERIAL_PORT//LCD_串行_端口

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)// !USBCON&（UBRRH | UBRR0H | UBRR1H | UBRR2H | UBRR3H）

// For AT90USB targets use the UART for BT interfacing//对于AT90USB目标，使用UART进行BT接口
#if defined(USBCON) && ENABLED(BLUETOOTH)
  MSerialBT bluetoothSerial(false);
#endif

#endif // __AVR__//_uuuavr__
