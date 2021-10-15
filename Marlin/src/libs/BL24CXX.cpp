/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfig.h"

#if ENABLED(IIC_BL24CXX_EEPROM)

/**
 * PersistentStore for Arduino-style EEPROM interface
 * with simple implementations supplied by Marlin.
 */

#include "BL24CXX.h"
#ifdef __STM32F1__
  #include <libmaple/gpio.h>
#else
  #include "../HAL/shared/Delay.h"
  #define delay_us(n) DELAY_US(n)
#endif

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY    10
#endif
#ifndef EEPROM_DEVICE_ADDRESS
  #define EEPROM_DEVICE_ADDRESS (0x50 << 1)
#endif

// IO direction setting//IO方向设置
#ifdef __STM32F1__
  #define SDA_IN()  do{ PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH &= 0XFFFF0FFF; PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH |= 8 << 12; }while(0)
  #define SDA_OUT() do{ PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH &= 0XFFFF0FFF; PIN_MAP[IIC_EEPROM_SDA].gpio_device->regs->CRH |= 3 << 12; }while(0)
#elif STM32F1
  #define SDA_IN()  SET_INPUT(IIC_EEPROM_SDA)
  #define SDA_OUT() SET_OUTPUT(IIC_EEPROM_SDA)
#endif

// IO ops//IO操作
#define IIC_SCL_0()   WRITE(IIC_EEPROM_SCL, LOW)
#define IIC_SCL_1()   WRITE(IIC_EEPROM_SCL, HIGH)
#define IIC_SDA_0()   WRITE(IIC_EEPROM_SDA, LOW)
#define IIC_SDA_1()   WRITE(IIC_EEPROM_SDA, HIGH)
#define READ_SDA()    READ(IIC_EEPROM_SDA)

////
// Simple IIC interface via libmaple//通过libmaple的简单IIC接口
////

// Initialize IIC//初始化IIC
void IIC::init() {
  SET_OUTPUT(IIC_EEPROM_SDA);
  SET_OUTPUT(IIC_EEPROM_SCL);
  IIC_SCL_1();
  IIC_SDA_1();
}

// Generate IIC start signal//产生IIC启动信号
void IIC::start() {
  SDA_OUT();    // SDA line output//SDA线路输出
  IIC_SDA_1();
  IIC_SCL_1();
  delay_us(4);
  IIC_SDA_0();  // START:when CLK is high, DATA change from high to low//开始：CLK高时，数据从高变低
  delay_us(4);
  IIC_SCL_0();  // Clamp the I2C bus, ready to send or receive data//夹紧I2C总线，准备发送或接收数据
}

// Generate IIC stop signal//产生IIC停止信号
void IIC::stop() {
  SDA_OUT();    // SDA line output//SDA线路输出
  IIC_SCL_0();
  IIC_SDA_0();  // STOP:when CLK is high DATA change from low to high//停止：时钟高时，数据从低变高
  delay_us(4);
  IIC_SCL_1();
  IIC_SDA_1();  // Send I2C bus end signal//发送I2C总线结束信号
  delay_us(4);
}

// Wait for the response signal to arrive//等待响应信号到达
// 1 = failed to receive response//1=接收响应失败
// 0 = response received//0=收到响应
uint8_t IIC::wait_ack() {
  uint8_t ucErrTime = 0;
  SDA_IN();      // SDA is set as input//SDA设置为输入
  IIC_SDA_1(); delay_us(1);
  IIC_SCL_1(); delay_us(1);
  while (READ_SDA()) {
    if (++ucErrTime > 250) {
      stop();
      return 1;
    }
  }
  IIC_SCL_0(); // Clock output 0//时钟输出0
  return 0;
}

// Generate ACK response//生成确认响应
void IIC::ack() {
  IIC_SCL_0();
  SDA_OUT();
  IIC_SDA_0();
  delay_us(2);
  IIC_SCL_1();
  delay_us(2);
  IIC_SCL_0();
}

// No ACK response//无应答
void IIC::nAck() {
  IIC_SCL_0();
  SDA_OUT();
  IIC_SDA_1();
  delay_us(2);
  IIC_SCL_1();
  delay_us(2);
  IIC_SCL_0();
}

// Send one IIC byte//发送一个IIC字节
// Return whether the slave responds//返回从属服务器是否响应
// 1 = there is a response//1=有响应
// 0 = no response//0=无响应
void IIC::send_byte(uint8_t txd) {
  SDA_OUT();
  IIC_SCL_0(); // Pull down the clock to start data transmission//拉下时钟开始数据传输
  LOOP_L_N(t, 8) {
    // IIC_SDA = (txd & 0x80) >> 7;//IIC_SDA=（txd&0x80）>>7；
    if (txd & 0x80) IIC_SDA_1(); else IIC_SDA_0();
    txd <<= 1;
    delay_us(2);   // All three delays are necessary for TEA5767//TEA5767需要所有三次延迟
    IIC_SCL_1();
    delay_us(2);
    IIC_SCL_0();
    delay_us(2);
  }
}

// Read 1 byte, when ack=1, send ACK, ack=0, send nACK//读取1字节，当ack=1时，发送ack，ack=0，发送nACK
uint8_t IIC::read_byte(unsigned char ack_chr) {
  unsigned char receive = 0;
  SDA_IN(); // SDA is set as input//SDA设置为输入
  LOOP_L_N(i, 8) {
    IIC_SCL_0();
    delay_us(2);
    IIC_SCL_1();
    receive <<= 1;
    if (READ_SDA()) receive++;
    delay_us(1);
  }
  ack_chr ? ack() : nAck(); // Send ACK / send nACK//发送确认/发送nACK
  return receive;
}

/******************** EEPROM ********************/

// Initialize the IIC interface//初始化IIC接口
void BL24CXX::init() { IIC::init(); }

// Read a byte at the specified address//在指定的地址读取一个字节
// ReadAddr: the address to start reading//ReadAddr：开始读取的地址
// Return: the byte read//Return：读取的字节
uint8_t BL24CXX::readOneByte(uint16_t ReadAddr) {
  uint8_t temp = 0;
  IIC::start();
  if (EE_TYPE > BL24C16) {
    IIC::send_byte(EEPROM_DEVICE_ADDRESS);        // Send write command//发送写命令
    IIC::wait_ack();
    IIC::send_byte(ReadAddr >> 8);                // Send high address//发送高地址
    IIC::wait_ack();
  }
  else
    IIC::send_byte(EEPROM_DEVICE_ADDRESS + ((ReadAddr >> 8) << 1)); // Send device address 0xA0, write data//发送设备地址0xA0，写入数据

  IIC::wait_ack();
  IIC::send_byte(ReadAddr & 0xFF);                // Send low address//发送低位地址
  IIC::wait_ack();
  IIC::start();
  IIC::send_byte(EEPROM_DEVICE_ADDRESS | 0x01);   // Send byte//发送字节
  IIC::wait_ack();
  temp = IIC::read_byte(0);
  IIC::stop();                                    // Generate a stop condition//生成停止条件
  return temp;
}

// Write a data at the address specified by BL24CXX//在BL24CXX指定的地址写入数据
// WriteAddr: The destination address for writing data//WriteAddr：用于写入数据的目标地址
// DataToWrite: the data to be written//DataToWrite：要写入的数据
void BL24CXX::writeOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
  IIC::start();
  if (EE_TYPE > BL24C16) {
    IIC::send_byte(EEPROM_DEVICE_ADDRESS);        // Send write command//发送写命令
    IIC::wait_ack();
    IIC::send_byte(WriteAddr >> 8);               // Send high address//发送高地址
  }
  else
    IIC::send_byte(EEPROM_DEVICE_ADDRESS + ((WriteAddr >> 8) << 1)); // Send device address 0xA0, write data//发送设备地址0xA0，写入数据

  IIC::wait_ack();
  IIC::send_byte(WriteAddr & 0xFF);               // Send low address//发送低位地址
  IIC::wait_ack();
  IIC::send_byte(DataToWrite);                    // Receiving mode//接收方式
  IIC::wait_ack();
  IIC::stop();                                    // Generate a stop condition//生成停止条件
  delay(10);
}

// Start writing data of length Len at the specified address in BL24CXX//开始在BL24CXX中的指定地址写入长度为Len的数据
// This function is used to write 16bit or 32bit data.//此函数用于写入16位或32位数据。
// WriteAddr: the address to start writing//WriteAddr：开始写入的地址
// DataToWrite: the first address of the data array//DataToWrite：数据数组的第一个地址
// Len: The length of the data to be written 2, 4//Len：要写入的数据长度2，4
void BL24CXX::writeLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  LOOP_L_N(t, Len)
    writeOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
}

// Start reading data of length Len from the specified address in BL24CXX//开始从BL24CXX中的指定地址读取长度为Len的数据
// This function is used to read 16bit or 32bit data.//此函数用于读取16位或32位数据。
// ReadAddr: the address to start reading//ReadAddr：开始读取的地址
// Return value: data//返回值：数据
// Len: The length of the data to be read 2,4//Len：要读取的数据长度2,4
uint32_t BL24CXX::readLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint32_t temp = 0;
  LOOP_L_N(t, Len) {
    temp <<= 8;
    temp += readOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

// Check if BL24CXX is normal//检查BL24CXX是否正常
// Return 1: Detection failed//返回1：检测失败
// return 0: detection is successful//返回0：检测成功
#define BL24CXX_TEST_ADDRESS 0x00
#define BL24CXX_TEST_VALUE   0x55

bool BL24CXX::_check() {
  return (readOneByte(BL24CXX_TEST_ADDRESS) != BL24CXX_TEST_VALUE); // false = success!//错误=成功！
}

bool BL24CXX::check() {
  if (_check()) {                                           // Value was written? Good EEPROM!//写了什么值？好的EEPROM！
    writeOneByte(BL24CXX_TEST_ADDRESS, BL24CXX_TEST_VALUE); // Write now and check.//现在写并检查。
    return _check();
  }
  return false; // success!//成功！
}

// Start reading the specified number of data at the specified address in BL24CXX//在BL24CXX中的指定地址开始读取指定数量的数据
// ReadAddr: The address to start reading is 0~255 for 24c02//ReadAddr:对于24c02，开始读取的地址为0~255
// pBuffer: the first address of the data array//pBuffer：数据数组的第一个地址
// NumToRead: the number of data to be read//NumToRead：要读取的数据数
void BL24CXX::read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  for (; NumToRead; NumToRead--)
    *pBuffer++ = readOneByte(ReadAddr++);
}

// Start writing the specified number of data at the specified address in BL24CXX//开始在BL24CXX中的指定地址写入指定数量的数据
// WriteAddr: the address to start writing, 0~255 for 24c02//WriteAddr：开始写入的地址，0~255表示24c02
// pBuffer: the first address of the data array//pBuffer：数据数组的第一个地址
// NumToWrite: the number of data to be written//NumToWrite：要写入的数据数
void BL24CXX::write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  for (; NumToWrite; NumToWrite--, WriteAddr++)
    writeOneByte(WriteAddr, *pBuffer++);
}

#endif // IIC_BL24CXX_EEPROM//IIC_BL24CXX_EEPROM
