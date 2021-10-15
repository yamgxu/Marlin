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
 * MarlinSerial_Due.cpp - Hardware serial library for Arduino DUE
 * Copyright (c) 2017 Eduardo José Tagle. All right reserved
 * Based on MarlinSerial for AVR, copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 */
#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"

#include "MarlinSerial.h"
#include "InterruptVectors.h"
#include "../../MarlinCore.h"

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

// (called with RX interrupts disabled)//（在禁用RX中断的情况下调用）
template<typename Cfg>
FORCE_INLINE void MarlinSerial<Cfg>::store_rxd_char() {

  static EmergencyParser::State emergency_state; // = EP_RESET//=EP_重置

  // Get the tail - Nothing can alter its value while we are at this ISR//获取尾部-当我们处于ISR时，任何东西都不能改变其值
  const ring_buffer_pos_t t = rx_buffer.tail;

  // Get the head pointer//拿到头上的指针
  ring_buffer_pos_t h = rx_buffer.head;

  // Get the next element//获取下一个元素
  ring_buffer_pos_t i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

  // Read the character from the USART//从USART读取字符
  uint8_t c = HWUART->UART_RHR;

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

  const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);
  // Calculate count of bytes stored into the RX buffer//计算存储到RX缓冲区的字节数

  // Keep track of the maximum count of enqueued bytes//跟踪排队字节的最大计数
  if (Cfg::MAX_RX_QUEUED) NOLESS(rx_max_enqueued, rx_count);

  if (Cfg::XONOFF) {
    // If the last char that was sent was an XON//如果发送的最后一个字符是XON
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XON_CHAR) {

      // Bytes stored into the RX buffer//存储在RX缓冲区中的字节
      const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

      // If over 12.5% of RX buffer capacity, send XOFF before running out of//如果超过RX缓冲区容量的12.5%，则在用完之前发送XOFF
      // RX buffer space .. 325 bytes @ 250kbits/s needed to let the host react//接收缓冲区空间。。需要325字节@250kbits/s才能让主机做出反应
      // and stop sending bytes. This translates to 13mS propagation time.//并停止发送字节。这相当于13毫秒的传播时间。
      if (rx_count >= (Cfg::RX_SIZE) / 8) {

        // At this point, definitely no TX interrupt was executing, since the TX isr can't be preempted.//在这一点上，肯定没有执行TX中断，因为TX isr不能被抢占。
        // Don't enable the TX interrupt here as a means to trigger the XOFF char, because if it happens//不要在这里启用TX中断作为触发XOFF字符的手段，因为如果它发生
        // to be in the middle of trying to disable the RX interrupt in the main program, eventually the//试图在主程序中禁用RX中断，最终
        // enabling of the TX interrupt could be undone. The ONLY reliable thing this can do to ensure//TX中断的启用可以撤消。这是唯一可靠的保证
        // the sending of the XOFF char is to send it HERE AND NOW.//XOFF字符的发送就是在此时此地发送。

        // About to send the XOFF char//即将发送XOFF字符
        xon_xoff_state = XOFF_CHAR | XON_XOFF_CHAR_SENT;

        // Wait until the TX register becomes empty and send it - Here there could be a problem//等待TX寄存器变为空并发送-此处可能有问题
        // - While waiting for the TX register to empty, the RX register could receive a new//-在等待TX寄存器清空时，RX寄存器可接收新的
        //   character. This must also handle that situation!//性格。这也必须处理这种情况！
        uint32_t status;
        while (!((status = HWUART->UART_SR) & UART_SR_TXRDY)) {

          if (status & UART_SR_RXRDY) {
            // We received a char while waiting for the TX buffer to be empty - Receive and process it!//我们在等待TX缓冲区为空时收到一个字符-接收并处理它！

            i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

            // Read the character from the USART//从USART读取字符
            c = HWUART->UART_RHR;

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

        HWUART->UART_THR = XOFF_CHAR;

        // At this point there could be a race condition between the write() function//此时，write（）函数之间可能存在竞争条件
        // and this sending of the XOFF char. This interrupt could happen between the//这是XOFF字符的发送。此中断可能发生在
        // wait to be empty TX buffer loop and the actual write of the character. Since//等待TX缓冲区循环和字符的实际写入为空。自从
        // the TX buffer is full because it's sending the XOFF char, the only way to be//TX缓冲区已满，因为它正在发送XOFF字符，这是唯一的恢复方法
        // sure the write() function will succeed is to wait for the XOFF char to be//确保write（）函数成功的方法是等待XOFF字符被删除
        // completely sent. Since an extra character could be received during the wait//完全发送。因为在等待过程中可能会收到一个额外的字符
        // it must also be handled!//它也必须被处理！
        while (!((status = HWUART->UART_SR) & UART_SR_TXRDY)) {

          if (status & UART_SR_RXRDY) {
            // A char arrived while waiting for the TX buffer to be empty - Receive and process it!//在等待TX缓冲区为空时到达一个字符-接收并处理它！

            i = (ring_buffer_pos_t)(h + 1) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);

            // Read the character from the USART//从USART读取字符
            c = HWUART->UART_RHR;

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

        // At this point everything is ready. The write() function won't//现在一切都准备好了。write（）函数将不起作用
        // have any issues writing to the UART TX register if it needs to!//如果需要，写入UART TX寄存器有任何问题！
      }
    }
  }

  // Store the new head value//存储新的头值
  rx_buffer.head = h;
}

template<typename Cfg>
FORCE_INLINE void MarlinSerial<Cfg>::_tx_thr_empty_irq() {
  if (Cfg::TX_SIZE > 0) {
    // Read positions//读取位置
    uint8_t t = tx_buffer.tail;
    const uint8_t h = tx_buffer.head;

    if (Cfg::XONOFF) {
      // If an XON char is pending to be sent, do it now//如果一个XON字符等待发送，现在就发送
      if (xon_xoff_state == XON_CHAR) {

        // Send the character//发送字符
        HWUART->UART_THR = XON_CHAR;

        // Remember we sent it.//记得我们送的。
        xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;

        // If nothing else to transmit, just disable TX interrupts.//如果没有其他传输，只需禁用TX中断。
        if (h == t) HWUART->UART_IDR = UART_IDR_TXRDY;

        return;
      }
    }

    // If nothing to transmit, just disable TX interrupts. This could//如果没有传输，只需禁用TX中断。这可能
    // happen as the result of the non atomicity of the disabling of RX//由于RX禁用的非原子性而发生
    // interrupts that could end reenabling TX interrupts as a side effect.//作为副作用，可能结束重新烧蚀TX中断的中断。
    if (h == t) {
      HWUART->UART_IDR = UART_IDR_TXRDY;
      return;
    }

    // There is something to TX, Send the next byte//有东西要发送，发送下一个字节
    const uint8_t c = tx_buffer.buffer[t];
    t = (t + 1) & (Cfg::TX_SIZE - 1);
    HWUART->UART_THR = c;
    tx_buffer.tail = t;

    // Disable interrupts if there is nothing to transmit following this byte//如果在该字节后没有要传输的内容，则禁用中断
    if (h == t) HWUART->UART_IDR = UART_IDR_TXRDY;
  }
}

template<typename Cfg>
void MarlinSerial<Cfg>::UART_ISR() {
  const uint32_t status = HWUART->UART_SR;

  // Data received?//收到数据了吗？
  if (status & UART_SR_RXRDY) store_rxd_char();

  if (Cfg::TX_SIZE > 0) {
    // Something to send, and TX interrupts are enabled (meaning something to send)?//要发送的内容，以及TX中断是否已启用（表示要发送的内容）？
    if ((status & UART_SR_TXRDY) && (HWUART->UART_IMR & UART_IMR_TXRDY)) _tx_thr_empty_irq();
  }

  // Acknowledge errors//承认错误
  if ((status & UART_SR_OVRE) || (status & UART_SR_FRAME)) {
    if (Cfg::DROPPED_RX && (status & UART_SR_OVRE) && !++rx_dropped_bytes) --rx_dropped_bytes;
    if (Cfg::RX_OVERRUNS && (status & UART_SR_OVRE) && !++rx_buffer_overruns) --rx_buffer_overruns;
    if (Cfg::RX_FRAMING_ERRORS && (status & UART_SR_FRAME) && !++rx_framing_errors) --rx_framing_errors;

    // TODO: error reporting outside ISR//TODO:在ISR外部报告错误
    HWUART->UART_CR = UART_CR_RSTSTA;
  }
}

// Public Methods//公共方法
template<typename Cfg>
void MarlinSerial<Cfg>::begin(const long baud_setting) {

  // Disable UART interrupt in NVIC//在NVIC中禁用UART中断
  NVIC_DisableIRQ( HWUART_IRQ );

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();

  // Disable clock//禁用时钟
  pmc_disable_periph_clk( HWUART_IRQ_ID );

  // Configure PMC//配置PMC
  pmc_enable_periph_clk( HWUART_IRQ_ID );

  // Disable PDC channel//禁用PDC通道
  HWUART->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  // Reset and disable receiver and transmitter//重置和禁用接收器和发射器
  HWUART->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

  // Configure mode: 8bit, No parity, 1 bit stop//配置模式：8位，无奇偶校验，1位停止
  HWUART->UART_MR = UART_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO;

  // Configure baudrate (asynchronous, no oversampling)//配置波特率（异步，无过采样）
  HWUART->UART_BRGR = (SystemCoreClock / (baud_setting << 4));

  // Configure interrupts//配置中断
  HWUART->UART_IDR = 0xFFFFFFFF;
  HWUART->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

  // Install interrupt handler//安装中断处理程序
  install_isr(HWUART_IRQ, UART_ISR);

  // Configure priority. We need a very high priority to avoid losing characters//配置优先级。我们需要一个非常高的优先级，以避免丢失字符
  // and we need to be able to preempt the Stepper ISR and everything else!//我们需要能够抢占步进ISR和其他一切！
  // (this could probably be fixed by using DMA with the Serial port)//（这可能通过使用带有串行端口的DMA来解决）
  NVIC_SetPriority(HWUART_IRQ, 1);

  // Enable UART interrupt in NVIC//在NVIC中启用UART中断
  NVIC_EnableIRQ(HWUART_IRQ);

  // Enable receiver and transmitter//启用接收器和发射器
  HWUART->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

  if (Cfg::TX_SIZE > 0) _written = false;
}

template<typename Cfg>
void MarlinSerial<Cfg>::end() {
  // Disable UART interrupt in NVIC//在NVIC中禁用UART中断
  NVIC_DisableIRQ( HWUART_IRQ );

  // We NEED memory barriers to ensure Interrupts are actually disabled!//我们需要内存屏障来确保中断实际上被禁用！
  // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )// ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
  __DSB();
  __ISB();

  pmc_disable_periph_clk( HWUART_IRQ_ID );
}

template<typename Cfg>
int MarlinSerial<Cfg>::peek() {
  const int v = rx_buffer.head == rx_buffer.tail ? -1 : rx_buffer.buffer[rx_buffer.tail];
  return v;
}

template<typename Cfg>
int MarlinSerial<Cfg>::read() {

  const ring_buffer_pos_t h = rx_buffer.head;
  ring_buffer_pos_t t = rx_buffer.tail;

  if (h == t) return -1;

  int v = rx_buffer.buffer[t];
  t = (ring_buffer_pos_t)(t + 1) & (Cfg::RX_SIZE - 1);

  // Advance tail//前尾
  rx_buffer.tail = t;

  if (Cfg::XONOFF) {
    // If the XOFF char was sent, or about to be sent...//如果XOFF字符已发送或即将发送。。。
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
      // Get count of bytes in the RX buffer//获取接收缓冲区中的字节计数
      const ring_buffer_pos_t rx_count = (ring_buffer_pos_t)(h - t) & (ring_buffer_pos_t)(Cfg::RX_SIZE - 1);
      // When below 10% of RX buffer capacity, send XON before running out of RX buffer bytes//当低于RX缓冲区容量的10%时，在RX缓冲区字节用完之前发送XON
      if (rx_count < (Cfg::RX_SIZE) / 10) {
        if (Cfg::TX_SIZE > 0) {
          // Signal we want an XON character to be sent.//我们希望发送一个XON字符的信号。
          xon_xoff_state = XON_CHAR;
          // Enable TX isr.//启用发送isr。
          HWUART->UART_IER = UART_IER_TXRDY;
        }
        else {
          // If not using TX interrupts, we must send the XON char now//如果不使用TX中断，我们必须立即发送XON字符
          xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
          while (!(HWUART->UART_SR & UART_SR_TXRDY)) sw_barrier();
          HWUART->UART_THR = XON_CHAR;
        }
      }
    }
  }

  return v;
}

template<typename Cfg>
typename MarlinSerial<Cfg>::ring_buffer_pos_t MarlinSerial<Cfg>::available() {
  const ring_buffer_pos_t h = rx_buffer.head, t = rx_buffer.tail;
  return (ring_buffer_pos_t)(Cfg::RX_SIZE + h - t) & (Cfg::RX_SIZE - 1);
}

template<typename Cfg>
void MarlinSerial<Cfg>::flush() {
  rx_buffer.tail = rx_buffer.head;

  if (Cfg::XONOFF) {
    if ((xon_xoff_state & XON_XOFF_CHAR_MASK) == XOFF_CHAR) {
      if (Cfg::TX_SIZE > 0) {
        // Signal we want an XON character to be sent.//我们希望发送一个XON字符的信号。
        xon_xoff_state = XON_CHAR;
        // Enable TX isr.//启用发送isr。
        HWUART->UART_IER = UART_IER_TXRDY;
      }
      else {
        // If not using TX interrupts, we must send the XON char now//如果不使用TX中断，我们必须立即发送XON字符
        xon_xoff_state = XON_CHAR | XON_XOFF_CHAR_SENT;
        while (!(HWUART->UART_SR & UART_SR_TXRDY)) sw_barrier();
        HWUART->UART_THR = XON_CHAR;
      }
    }
  }
}

template<typename Cfg>
size_t MarlinSerial<Cfg>::write(const uint8_t c) {
  _written = true;

  if (Cfg::TX_SIZE == 0) {
    while (!(HWUART->UART_SR & UART_SR_TXRDY)) sw_barrier();
    HWUART->UART_THR = c;
  }
  else {

    // If the TX interrupts are disabled and the data register//如果TX中断被禁用，并且数据寄存器
    // is empty, just write the byte to the data register and//为空，只需将字节写入数据寄存器并
    // be done. This shortcut helps significantly improve the//完成。此快捷方式有助于显著提高性能
    // effective datarate at high (>500kbit/s) bitrates, where//高（>500kbit/s）比特率下的有效数据速率，其中
    // interrupt overhead becomes a slowdown.//中断开销变慢。
    // Yes, there is a race condition between the sending of the//是的，在发送
    // XOFF char at the RX isr, but it is properly handled there//RX isr处的XOFF char，但在那里得到了正确处理
    if (!(HWUART->UART_IMR & UART_IMR_TXRDY) && (HWUART->UART_SR & UART_SR_TXRDY)) {
      HWUART->UART_THR = c;
      return 1;
    }

    const uint8_t i = (tx_buffer.head + 1) & (Cfg::TX_SIZE - 1);

    // If global interrupts are disabled (as the result of being called from an ISR)...//如果全局中断被禁用（由于从ISR调用）。。。
    if (!ISRS_ENABLED()) {

      // Make room by polling if it is possible to transmit, and do so!//如果可以传输，请通过轮询留出空间，并这样做！
      while (i == tx_buffer.tail) {
        // If we can transmit another byte, do it.//如果我们可以传输另一个字节，就这样做。
        if (HWUART->UART_SR & UART_SR_TXRDY) _tx_thr_empty_irq();
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

    // Enable TX isr - Non atomic, but it will eventually enable TX isr//启用TX isr-非原子，但最终将启用TX isr
    HWUART->UART_IER = UART_IER_TXRDY;
  }
  return 1;
}

template<typename Cfg>
void MarlinSerial<Cfg>::flushTX() {
  // TX//德克萨斯州

  if (Cfg::TX_SIZE == 0) {
    // No bytes written, no need to flush. This special case is needed since there's//无需写入字节，无需刷新。这个特殊情况是需要的，因为
    // no way to force the TXC (transmit complete) bit to 1 during initialization.//初始化期间，无法强制TXC（传输完成）位为1。
    if (!_written) return;

    // Wait until everything was transmitted//等到一切都传送出去
    while (!(HWUART->UART_SR & UART_SR_TXEMPTY)) sw_barrier();

    // At this point nothing is queued anymore (DRIE is disabled) and//此时，不再排队（DRIE被禁用），并且
    // the hardware finished transmission (TXC is set).//硬件完成传输（设置TXC）。

  }
  else {
    // If we have never written a byte, no need to flush. This special//如果我们从未写入过一个字节，则无需刷新。这个特别的
    // case is needed since there is no way to force the TXC (transmit//由于无法强制TXC（传输），因此需要外壳
    // complete) bit to 1 during initialization//完成）初始化过程中位为1
    if (!_written) return;

    // If global interrupts are disabled (as the result of being called from an ISR)...//如果全局中断被禁用（由于从ISR调用）。。。
    if (!ISRS_ENABLED()) {

      // Wait until everything was transmitted - We must do polling, as interrupts are disabled//等到一切都被传输了，我们必须进行轮询，因为中断被禁用了
      while (tx_buffer.head != tx_buffer.tail || !(HWUART->UART_SR & UART_SR_TXEMPTY)) {
        // If there is more space, send an extra character//如果有更多的空间，请发送一个额外的字符
        if (HWUART->UART_SR & UART_SR_TXRDY) _tx_thr_empty_irq();
        sw_barrier();
      }

    }
    else {
      // Wait until everything was transmitted//等到一切都传送出去
      while (tx_buffer.head != tx_buffer.tail || !(HWUART->UART_SR & UART_SR_TXEMPTY)) sw_barrier();
    }

    // At this point nothing is queued anymore (DRIE is disabled) and//此时，不再排队（DRIE被禁用），并且
    // the hardware finished transmission (TXC is set).//硬件完成传输（设置TXC）。
  }
}


// If not using the USB port as serial port//如果不使用USB端口作为串行端口
#if defined(SERIAL_PORT) && SERIAL_PORT >= 0
  template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT> >;
  MSerialT1 customizedSerial1(MarlinSerialCfg<SERIAL_PORT>::EMERGENCYPARSER);
#endif

#if defined(SERIAL_PORT_2) && SERIAL_PORT_2 >= 0
  template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT_2> >;
  MSerialT2 customizedSerial2(MarlinSerialCfg<SERIAL_PORT_2>::EMERGENCYPARSER);
#endif

#if defined(SERIAL_PORT_3) && SERIAL_PORT_3 >= 0
  template class MarlinSerial< MarlinSerialCfg<SERIAL_PORT_3> >;
  MSerialT3 customizedSerial3(MarlinSerialCfg<SERIAL_PORT_3>::EMERGENCYPARSER);
#endif

#endif // ARDUINO_ARCH_SAM//阿杜伊诺·阿丘·萨姆
