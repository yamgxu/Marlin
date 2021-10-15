/** translatione by yx */
/****************
 * usb_host.cpp *
 ****************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

/* What follows is a modified version of the MAX3421e originally defined in
 * lib/usbhost.c". This has been rewritten to use SPI routines from the
 * Marlin HAL */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(USB_FLASH_DRIVE_SUPPORT) && DISABLED(USE_UHS3_USB)

#include "Usb.h"
#include "usbhost.h"

uint8_t MAX3421e::vbusState = 0;

// constructor//建造师
void MAX3421e::cs() {
  WRITE(USB_CS_PIN,0);
}

void MAX3421e::ncs() {
  WRITE(USB_CS_PIN,1);
}

// write single byte into MAX3421 register//将单字节写入MAX3421寄存器
void MAX3421e::regWr(uint8_t reg, uint8_t data) {
  cs();
  spiSend(reg | 0x02);
  spiSend(data);
  ncs();
};

// multiple-byte write//多字节写入
// return a pointer to memory position after last written//在最后一次写入后返回指向内存位置的指针
uint8_t* MAX3421e::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  cs();
  spiSend(reg | 0x02);
  while (nbytes--) spiSend(*data_p++);
  ncs();
  return data_p;
}

// GPIO write//GPIO写入
// GPIO byte is split between 2 registers, so two writes are needed to write one byte//GPIO字节在两个寄存器之间拆分，因此写入一个字节需要两次写入

// GPOUT bits are in the low nybble. 0-3 in IOPINS1, 4-7 in IOPINS2//GPOUT位处于低位Nyble。IOPINS1中的0-3，IOPINS2中的4-7
void MAX3421e::gpioWr(uint8_t data) {
  regWr(rIOPINS1, data);
  regWr(rIOPINS2, data >> 4);
}

// single host register read//单主机寄存器读取
uint8_t MAX3421e::regRd(uint8_t reg) {
  cs();
  spiSend(reg);
  uint8_t rv = spiRec();
  ncs();
  return rv;
}
// multiple-byte register read//多字节寄存器读取

// return a pointer to a memory position after last read//在最后一次读取后返回指向内存位置的指针
uint8_t* MAX3421e::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
  cs();
  spiSend(reg);
  while (nbytes--) *data_p++ = spiRec();
  ncs();
  return data_p;
}
// GPIO read. See gpioWr for explanation//GPIO读取。有关说明，请参见gpioWr

// GPIN pins are in high nybbles of IOPINS1, IOPINS2//GPIN引脚位于IOPINS1、IOPINS2的高Nybles中
uint8_t MAX3421e::gpioRd() {
  return (regRd(rIOPINS2) & 0xF0) | // pins 4-7, clean lower nybble//销4-7，清洁下部nybble
         (regRd(rIOPINS1)   >> 4);  // shift low bits and OR with upper from previous operation.//将低位和或与上一操作的高位相移。
}

// reset MAX3421e. Returns false if PLL failed to stabilize 1 second after reset//重置MAX3421e。如果PLL在复位后1秒无法稳定，则返回false
bool MAX3421e::reset() {
  regWr(rUSBCTL, bmCHIPRES);
  regWr(rUSBCTL, 0x00);
  for (uint8_t i = 100; i--;) {
    if (regRd(rUSBIRQ) & bmOSCOKIRQ) return true;
    delay(10);
  }
  return false;
}

// initialize MAX3421e. Set Host mode, pullups, and stuff. Returns 0 if success, -1 if not//初始化MAX3421e。设置主机模式、上拉等。如果成功，则返回0；如果不成功，则返回1
bool MAX3421e::start() {
  // Initialize pins and SPI bus//初始化引脚和SPI总线

  SET_OUTPUT(USB_CS_PIN);
  SET_INPUT_PULLUP(USB_INTR_PIN);
  ncs();
  spiBegin();

  spiInit(SD_SPI_SPEED);

  // MAX3421e - full-duplex, level interrupt, vbus off.//MAX3421e-全双工，电平中断，vbus关闭。
  regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | GPX_VBDET));

  const uint8_t revision = regRd(rREVISION);
  if (revision == 0x00 || revision == 0xFF) {
    SERIAL_ECHOLNPAIR("Revision register appears incorrect on MAX3421e initialization. Got ", revision);
    return false;
  }

  if (!reset()) {
    SERIAL_ECHOLNPGM("OSCOKIRQ hasn't asserted in time");
    return false;
  }

  // Delay a minimum of 1 second to ensure any capacitors are drained.//延迟至少1秒，以确保电容器耗尽。
  // 1 second is required to make sure we do not smoke a Microdrive!//需要1秒来确保我们不吸Microdrive！

  delay(1000);

  regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host//设置下拉菜单，主机
  regWr(rHIEN, bmCONDETIE | bmFRAMEIE); // connection detection//连接检测

  // check if device is connected//检查设备是否已连接
  regWr(rHCTL, bmSAMPLEBUS); // sample USB bus//USB总线示例
  while (!(regRd(rHCTL) & bmSAMPLEBUS)) delay(10); // wait for sample operation to finish//等待样本操作完成

  busprobe(); // check if anything is connected//检查是否有任何连接

  regWr(rHIRQ, bmCONDETIRQ); // clear connection detect interrupt//清除连接检测中断
  regWr(rCPUCTL, 0x01);      // enable interrupt pin//启用中断引脚

  // GPX pin on. This is done here so that busprobe will fail if we have a switch connected.//GPX引脚打开。这是在这里完成的，因此如果我们连接了一个开关，busprobe将失败。
  regWr(rPINCTL, bmFDUPSPI | bmINTLEVEL);

  return true;
}

// Probe bus to determine device presence and speed. Switch host to this speed.//探测总线以确定设备存在和速度。将主机切换到此速度。
void MAX3421e::busprobe() {
  // Switch on just the J & K bits//只打开J&K位
  switch (regRd(rHRSL) & (bmJSTATUS | bmKSTATUS)) {
    case bmJSTATUS:
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
        regWr(rMODE, MODE_FS_HOST); // start full-speed host//启动全速主机
        vbusState = FSHOST;
      }
      else {
        regWr(rMODE, MODE_LS_HOST); // start low-speed host//启动低速主机
        vbusState = LSHOST;
      }
      break;
    case bmKSTATUS:
      if ((regRd(rMODE) & bmLOWSPEED) == 0) {
        regWr(rMODE, MODE_LS_HOST); // start low-speed host//启动低速主机
        vbusState = LSHOST;
      }
      else {
        regWr(rMODE, MODE_FS_HOST); // start full-speed host//启动全速主机
        vbusState = FSHOST;
      }
      break;
    case bmSE1: // illegal state//非法国家
      vbusState = SE1;
      break;
    case bmSE0: // disconnected state//断开状态
      regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);
      vbusState = SE0;
      break;
  }
}

// MAX3421 state change task and interrupt handler//MAX3421状态更改任务和中断处理程序
uint8_t MAX3421e::Task() {
  return READ(USB_INTR_PIN) ? 0 : IntHandler();
}

uint8_t MAX3421e::IntHandler() {
  uint8_t HIRQ = regRd(rHIRQ), // determine interrupt source//确定中断源
          HIRQ_sendback = 0x00;
  if (HIRQ & bmCONDETIRQ) {
    busprobe();
    HIRQ_sendback |= bmCONDETIRQ;
  }
  // End HIRQ interrupts handling, clear serviced IRQs//结束HIRQ中断处理，清除已服务的IRQ
  regWr(rHIRQ, HIRQ_sendback);
  return HIRQ_sendback;
}

#endif // USB_FLASH_DRIVE_SUPPORT//USB闪存驱动器支持
