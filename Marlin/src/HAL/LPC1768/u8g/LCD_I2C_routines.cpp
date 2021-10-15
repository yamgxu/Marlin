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

// adapted from  I2C/master/master.c example//改编自I2C/master/master.c示例
//   https://www-users.cs.york.ac.uk/~pcc/MCP/HAPR-Course-web/CMSIS/examples/html/master_8c_source.html//   https://www-users.cs.york.ac.uk/~pcc/MCP/HAPR课程网站/CMSIS/examples/html/master_8c_source.html

#ifdef TARGET_LPC1768

#include "../include/i2c_util.h"
#include "../../../core/millis_t.h"

extern int millis();

#ifdef __cplusplus
  extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// These two routines are exact copies of the lpc17xx_i2c.c routines.  Couldn't link to//这两个例程是lpc17xx_i2c.c例程的精确副本。无法链接到
// to the lpc17xx_i2c.c routines so had to copy them into this file & rename them.//到lpc17xx_i2c.c例程，因此必须将它们复制到此文件并重命名。

static uint32_t _I2C_Start(LPC_I2C_TypeDef *I2Cx) {
  // Reset STA, STO, SI//重置STA、STO、SI
  I2Cx->I2CONCLR = I2C_I2CONCLR_SIC|I2C_I2CONCLR_STOC|I2C_I2CONCLR_STAC;

  // Enter to Master Transmitter mode//进入主发射机模式
  I2Cx->I2CONSET = I2C_I2CONSET_STA;

  // Wait for complete//等待完成
  while (!(I2Cx->I2CONSET & I2C_I2CONSET_SI));
  I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;
  return (I2Cx->I2STAT & I2C_STAT_CODE_BITMASK);
}

static void _I2C_Stop (LPC_I2C_TypeDef *I2Cx) {
  /* Make sure start bit is not active */
  if (I2Cx->I2CONSET & I2C_I2CONSET_STA)
    I2Cx->I2CONCLR = I2C_I2CONCLR_STAC;

  I2Cx->I2CONSET = I2C_I2CONSET_STO|I2C_I2CONSET_AA;
  I2Cx->I2CONCLR = I2C_I2CONCLR_SIC;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2CDEV_S_ADDR   0x78  // from SSD1306  //actual address is 0x3C - shift left 1 with LSB set to 0 to indicate write//从SSD1306//实际地址为0x3C-左移1，LSB设置为0以指示写入

#define BUFFER_SIZE                     0x1  // only do single byte transfers with LCDs//仅使用LCD进行单字节传输

I2C_M_SETUP_Type transferMCfg;

#define I2C_status (LPC_I2C1->I2STAT & I2C_STAT_CODE_BITMASK)

// Send slave address and write bit//发送从机地址和写入位
uint8_t u8g_i2c_start(const uint8_t sla) {
  // Sometimes TX data ACK or NAK status is returned.  That mean the start state didn't//有时返回TX数据ACK或NAK状态。这意味着开始状态没有改变
  // happen which means only the value of the slave address was send.  Keep looping until//这意味着只发送了从属地址的值。继续循环直到
  // the slave address and write bit are actually sent.//从机地址和写入位实际上已发送。
  do{
    _I2C_Stop(I2CDEV_M); // output stop state on I2C bus//I2C总线上的输出停止状态
    _I2C_Start(I2CDEV_M); // output start state on I2C bus//I2C总线上的输出启动状态
    while ((I2C_status != I2C_I2STAT_M_TX_START)
        && (I2C_status != I2C_I2STAT_M_TX_RESTART)
        && (I2C_status != I2C_I2STAT_M_TX_DAT_ACK)
        && (I2C_status != I2C_I2STAT_M_TX_DAT_NACK));  //wait for start to be asserted//等待start被断言

    LPC_I2C1->I2CONCLR = I2C_I2CONCLR_STAC; // clear start state before tansmitting slave address//在传输从属地址之前清除启动状态
    LPC_I2C1->I2DAT = I2CDEV_S_ADDR & I2C_I2DAT_BITMASK; // transmit slave address & write bit//传输从机地址和写入位
    LPC_I2C1->I2CONSET = I2C_I2CONSET_AA;
    LPC_I2C1->I2CONCLR = I2C_I2CONCLR_SIC;
    while ((I2C_status != I2C_I2STAT_M_TX_SLAW_ACK)
        && (I2C_status != I2C_I2STAT_M_TX_SLAW_NACK)
        && (I2C_status != I2C_I2STAT_M_TX_DAT_ACK)
        && (I2C_status != I2C_I2STAT_M_TX_DAT_NACK));  //wait for slaw to finish//等板鱼吃完
  }while ( (I2C_status == I2C_I2STAT_M_TX_DAT_ACK) ||  (I2C_status == I2C_I2STAT_M_TX_DAT_NACK));
  return 1;
}

void u8g_i2c_init(const uint8_t clock_option) {
  configure_i2c(clock_option);
  u8g_i2c_start(0); // Send slave address and write bit//发送从机地址和写入位
}

uint8_t u8g_i2c_send_byte(uint8_t data) {
  #define I2C_TIMEOUT 3
  LPC_I2C1->I2DAT = data & I2C_I2DAT_BITMASK; // transmit data//传输数据
  LPC_I2C1->I2CONSET = I2C_I2CONSET_AA;
  LPC_I2C1->I2CONCLR = I2C_I2CONCLR_SIC;
  const millis_t timeout = millis() + I2C_TIMEOUT;
  while ((I2C_status != I2C_I2STAT_M_TX_DAT_ACK) && (I2C_status != I2C_I2STAT_M_TX_DAT_NACK) && PENDING(millis(), timeout));  // wait for xmit to finish//等待xmit完成
  // had hangs with SH1106 so added time out - have seen temporary screen corruption when this happens//had与SH1106挂起，因此增加了超时-出现这种情况时，您会看到临时屏幕损坏
  return 1;
}

void u8g_i2c_stop() {
}


#ifdef __cplusplus
  }
#endif

#endif // TARGET_LPC1768//目标为LPC1768
