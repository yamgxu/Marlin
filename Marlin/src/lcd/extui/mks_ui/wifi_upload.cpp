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
#include "../../../inc/MarlinConfigPre.h"

#if BOTH(HAS_TFT_LVGL_UI, MKS_WIFI_MODULE)

#include "draw_ui.h"
#include "wifi_module.h"
#include "wifi_upload.h"

#include "../../../MarlinCore.h"
#include "../../../sd/cardreader.h"

#define WIFI_SET()        WRITE(WIFI_RESET_PIN, HIGH);
#define WIFI_RESET()      WRITE(WIFI_RESET_PIN, LOW);
#define WIFI_IO1_SET()    WRITE(WIFI_IO1_PIN, HIGH);
#define WIFI_IO1_RESET()  WRITE(WIFI_IO1_PIN, LOW);

extern SZ_USART_FIFO  WifiRxFifo;

extern int readUsartFifo(SZ_USART_FIFO *fifo, int8_t *buf, int32_t len);
extern int writeUsartFifo(SZ_USART_FIFO * fifo, int8_t * buf, int32_t len);
void esp_port_begin(uint8_t interrupt);
extern int usartFifoAvailable(SZ_USART_FIFO *fifo);
void wifi_delay(int n);

#define ARRAY_SIZE(a) sizeof(a) / sizeof((a)[0])

//typedef signed char bool;//typedef签名字符库；

// ESP8266 command codes//ESP8266命令代码
const uint8_t ESP_FLASH_BEGIN = 0x02;
const uint8_t ESP_FLASH_DATA = 0x03;
const uint8_t ESP_FLASH_END = 0x04;
const uint8_t ESP_MEM_BEGIN = 0x05;
const uint8_t ESP_MEM_END = 0x06;
const uint8_t ESP_MEM_DATA = 0x07;
const uint8_t ESP_SYNC = 0x08;
const uint8_t ESP_WRITE_REG = 0x09;
const uint8_t ESP_READ_REG = 0x0A;

// MAC address storage locations//MAC地址存储位置
const uint32_t ESP_OTP_MAC0 = 0x3FF00050;
const uint32_t ESP_OTP_MAC1 = 0x3FF00054;
const uint32_t ESP_OTP_MAC2 = 0x3FF00058;
const uint32_t ESP_OTP_MAC3 = 0x3FF0005C;

const size_t EspFlashBlockSize = 0x0400;      // 1K byte blocks//1K字节块

const uint8_t ESP_IMAGE_MAGIC = 0xE9;
const uint8_t ESP_CHECKSUM_MAGIC = 0xEF;

const uint32_t ESP_ERASE_CHIP_ADDR = 0x40004984;  // &SPIEraseChip//间谍
const uint32_t ESP_SEND_PACKET_ADDR = 0x40003C80; // &send_packet//发送数据包（&U）
const uint32_t ESP_SPI_READ_ADDR = 0x40004B1C;    // &SPIRead//&斯皮里德
const uint32_t ESP_UNKNOWN_ADDR = 0x40001121;   // not used//不用
const uint32_t ESP_USER_DATA_RAM_ADDR = 0x3FFE8000; // &user data ram//&用户数据ram
const uint32_t ESP_IRAM_ADDR = 0x40100000;      // instruction RAM//指令RAM
const uint32_t ESP_FLASH_ADDR = 0x40200000;     // address of start of Flash//闪存启动地址

UPLOAD_STRUCT esp_upload;

static const unsigned int retriesPerReset = 3;
static const uint32_t connectAttemptInterval = 50;
static const unsigned int percentToReportIncrement = 5; // how often we report % complete//我们报告完成百分比的频率
static const uint32_t defaultTimeout = 500;
static const uint32_t eraseTimeout = 15000;
static const uint32_t blockWriteTimeout = 200;
static const uint32_t blockWriteInterval = 15;      // 15ms is long enough, 10ms is mostly too short//15毫秒足够长，10毫秒大多太短
static SdFile update_file, *update_curDir;

// Messages corresponding to result codes, should make sense when followed by " error"//与结果代码相对应的消息在后跟“error”时应该有意义
const char *resultMessages[] = {
  "no",
  "timeout",
  "comm write",
  "connect",
  "bad reply",
  "file read",
  "empty file",
  "response header",
  "slip frame",
  "slip state",
  "slip data"
};

// A note on baud rates.//关于波特率的注记。
// The ESP8266 supports 921600, 460800, 230400, 115200, 74880 and some lower baud rates.//ESP8266支持921600、460800、230400、115200、74880和一些更低的波特率。
// 921600b is not reliable because even though it sometimes succeeds in connecting, we get a bad response during uploading after a few blocks.//921600b是不可靠的，因为即使它有时连接成功，但在上传过程中，经过几个块后，我们会收到错误的响应。
// Probably our UART ISR cannot receive bytes fast enough, perhaps because of the latency of the system tick ISR.//可能我们的UART ISR接收字节的速度不够快，可能是因为系统ISR的延迟。
// 460800b doesn't always manage to connect, but if it does then uploading appears to be reliable.//460800b并不总是能够连接，但如果能够连接，那么上传似乎是可靠的。
// 230400b always manages to connect.//230400b始终能够连接。
static const uint32_t uploadBaudRates[] = { 460800, 230400, 115200, 74880 };

signed char IsReady() {
  return esp_upload.state == upload_idle;
}

void uploadPort_write(const uint8_t *buf, const size_t len) {
  for (size_t i = 0; i < len; i++)
    WIFISERIAL.write(*(buf + i));
}

char uploadPort_read() {
  uint8_t retChar;
  retChar = WIFISERIAL.read();
  return _MAX(retChar, 0);
}

int uploadPort_available() {
  return usartFifoAvailable(&WifiRxFifo);
}

void uploadPort_begin() {
  esp_port_begin(1);
}

void uploadPort_close() {
  //WIFI_COM.end();//WIFI_COM.end（）；
  //WIFI_COM.begin(115200, true);//WIFI_COM.begin（115200，正确）；
  esp_port_begin(0);
}

void flushInput() {
  while (uploadPort_available() != 0) {
    (void)uploadPort_read();
    //IWDG_ReloadCounter();//IWDG_重载计数器（）；
  }
}

// Extract 1-4 bytes of a value in little-endian order from a buffer beginning at a specified offset//从指定偏移量开始的缓冲区中以小尾端顺序提取1-4字节的值
uint32_t getData(unsigned byteCnt, const uint8_t *buf, int ofst) {
  uint32_t val = 0;
  if (buf && byteCnt) {
    unsigned int shiftCnt = 0;
    NOMORE(byteCnt, 4U);
    do {
      val |= (uint32_t)buf[ofst++] << shiftCnt;
      shiftCnt += 8;
    } while (--byteCnt);
  }
  return val;
}

// Put 1-4 bytes of a value in little-endian order into a buffer beginning at a specified offset.//将一个值的1-4字节以小尾端顺序放入缓冲区，从指定的偏移量开始。
void putData(uint32_t val, unsigned byteCnt, uint8_t *buf, int ofst) {
  if (buf && byteCnt) {
    NOMORE(byteCnt, 4U);
    do {
      buf[ofst++] = (uint8_t)(val & 0xFF);
      val >>= 8;
    } while (--byteCnt);
  }
}

// Read a byte optionally performing SLIP decoding.  The return values are://读取一个字节，可选择执行滑动解码。返回值为：
////
//  2 - an escaped byte was read successfully//2-已成功读取转义字节
//  1 - a non-escaped byte was read successfully//1-已成功读取非转义字节
//  0 - no data was available//0-没有可用的数据
//   -1 - the value 0xC0 was encountered (shouldn't happen)//-1-遇到值0xC0（不应发生）
//   -2 - a SLIP escape byte was found but the following byte wasn't available//-2-找到了滑动转义字节，但以下字节不可用
//   -3 - a SLIP escape byte was followed by an invalid byte//-3-滑动转义字节后跟无效字节
int ReadByte(uint8_t *data, signed char slipDecode) {
  if (uploadPort_available() == 0) return 0;

  // At least one byte is available//至少有一个字节可用
  *data = uploadPort_read();

  if (!slipDecode) return 1;

  if (*data == 0xC0) return -1; // This shouldn't happen//这不应该发生
  if (*data != 0xDB) return 1;  // If not the SLIP escape, we're done//如果不是逃跑，我们就完了

  // SLIP escape, check availability of subsequent byte//滑动转义，检查后续字节的可用性
  if (uploadPort_available() == 0) return -2;

  // process the escaped byte//处理转义字节
  *data = uploadPort_read();
  if (*data == 0xDC) { *data = 0xC0; return 2; }
  if (*data == 0xDD) { *data = 0xDB; return 2; }

  return -3; // invalid//无效的
}
// When we write a sync packet, there must be no gaps between most of the characters.//当我们编写同步数据包时，大多数字符之间必须没有间隔。
// So use this function, which does a block write to the UART buffer in the latest CoreNG.//所以使用这个函数，它在最新的coren中对UART缓冲区进行块写入。
void _writePacketRaw(const uint8_t *buf, size_t len) {
  uploadPort_write(buf, len);
}

// Write a byte to the serial port optionally SLIP encoding. Return the number of bytes actually written.//向串行端口写入一个字节（可选滑动编码）。返回实际写入的字节数。
void WriteByteRaw(uint8_t b) {
  uploadPort_write((const uint8_t *)&b, 1);
}

// Write a byte to the serial port optionally SLIP encoding. Return the number of bytes actually written.//向串行端口写入一个字节（可选滑动编码）。返回实际写入的字节数。
void WriteByteSlip(const uint8_t b) {
  if (b == 0xC0) {
    WriteByteRaw(0xDB);
    WriteByteRaw(0xDC);
  }
  else if (b == 0xDB) {
    WriteByteRaw(0xDB);
    WriteByteRaw(0xDD);
  }
  else
    uploadPort_write((const uint8_t *)&b, 1);
}

// Wait for a data packet to be returned.  If the body of the packet is//等待返回数据包。如果数据包的主体是
// non-zero length, return an allocated buffer indirectly containing the//非零长度，返回间接包含
// data and return the data length. Note that if the pointer for returning//并返回数据长度。请注意，如果返回的指针
// the data buffer is nullptr, the response is expected to be two bytes of zero.//数据缓冲区为nullptr，响应应为两个字节的零。
////
// If an error occurs, return a negative value.  Otherwise, return the number//如果发生错误，则返回负值。否则，请返回号码
// of bytes in the response (or zero if the response was not the standard "two bytes of zero").//响应中的字节数（如果响应不是标准的“零的两个字节”，则为零）。
EspUploadResult readPacket(uint8_t op, uint32_t *valp, size_t *bodyLen, uint32_t msTimeout) {
  typedef enum {
    begin = 0,
    header,
    body,
    end,
    done
  } PacketState;

  uint8_t resp, opRet;

  const size_t headerLength = 8;

  uint32_t startTime = getWifiTick();
  uint8_t hdr[headerLength];
  uint16_t hdrIdx = 0;

  uint16_t bodyIdx = 0;
  uint8_t respBuf[2];

  // wait for the response//等待回应
  uint16_t needBytes = 1;

  PacketState state = begin;

  *bodyLen = 0;

  while (state != done) {
    uint8_t c;
    EspUploadResult stat;

    //IWDG_ReloadCounter();//IWDG_重载计数器（）；
    watchdog_refresh();

    if (getWifiTickDiff(startTime, getWifiTick()) > msTimeout)
      return timeout;

    if (uploadPort_available() < needBytes) {
      // insufficient data available//可用数据不足
      // preferably, return to Spin() here//最好在这里返回到Spin（）
      continue;
    }

    // sufficient bytes have been received for the current state, process them//已收到当前状态的足够字节，请处理它们
    switch (state) {
      case begin: // expecting frame start//预期帧开始
        c = uploadPort_read();
        if (c == (uint8_t)0xC0) break;
        state = header;
        needBytes = 2;
        break;
      case end:   // expecting frame end//预期帧结束
        c = uploadPort_read();
        if (c != (uint8_t)0xC0) return slipFrame;
        state = done;
        break;

      case header:  // reading an 8-byte header//读取8字节头
      case body: {  // reading the response body//阅读回应体
        int rslt;
        // retrieve a byte with SLIP decoding//使用滑动解码检索字节
        rslt = ReadByte(&c, 1);
        if (rslt != 1 && rslt != 2) {
          // some error occurred//发生了一些错误
          stat = (rslt == 0 || rslt == -2) ? slipData : slipFrame;
          return stat;
        }
        else if (state == header) {
          //store the header byte//存储头字节
          hdr[hdrIdx++] = c;
          if (hdrIdx >= headerLength) {
            // get the body length, prepare a buffer for it//获取主体长度，为其准备缓冲区
            *bodyLen = (uint16_t)getData(2, hdr, 2);

            // extract the value, if requested//如果需要，请提取该值
            if (valp)
              *valp = getData(4, hdr, 4);

            if (*bodyLen != 0)
              state = body;
            else {
              needBytes = 1;
              state = end;
            }
          }
        }
        else {
          // Store the response body byte, check for completion//存储响应正文字节，检查是否完成
          if (bodyIdx < ARRAY_SIZE(respBuf))
            respBuf[bodyIdx] = c;

          if (++bodyIdx >= *bodyLen) {
            needBytes = 1;
            state = end;
          }
        }
      } break;

      default: return slipState;  // this shouldn't happen//这不应该发生
    }
  }

  // Extract elements from the header//从标题中提取元素
  resp = (uint8_t)getData(1, hdr, 0);
  opRet = (uint8_t)getData(1, hdr, 1);

  // Sync packets often provoke a response with a zero opcode instead of ESP_SYNC//同步数据包通常以零操作码而不是ESP_Sync引发响应
  if (resp != 0x01 || opRet != op) return respHeader;

  return success;
}

// Send a block of data performing SLIP encoding of the content.//发送数据块，对内容执行滑动编码。
void _writePacket(const uint8_t *data, size_t len) {
  unsigned char outBuf[2048] = {0};
  unsigned int outIndex = 0;
  while (len != 0) {
    if (*data == 0xC0) {
      outBuf[outIndex++] = 0xDB;
      outBuf[outIndex++] = 0xDC;
    }
    else if (*data == 0xDB) {
      outBuf[outIndex++] = 0xDB;
      outBuf[outIndex++] = 0xDD;
    }
    else {
      outBuf[outIndex++] = *data;
    }
    data++;
    --len;
  }
  uploadPort_write((const uint8_t *)outBuf, outIndex);
}

// Send a packet to the serial port while performing SLIP framing. The packet data comprises a header and an optional data block.//执行滑动帧时向串行端口发送数据包。分组数据包括报头和可选数据块。
// A SLIP packet begins and ends with 0xC0.  The data encapsulated has the bytes//SLIP数据包以0xC0开始和结束。封装的数据包含字节
// 0xC0 and 0xDB replaced by the two-byte sequences {0xDB, 0xDC} and {0xDB, 0xDD} respectively.//0xC0和0xDB分别替换为两个字节序列{0xDB，0xDC}和{0xDB，0xDD}。

void writePacket(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) {
  WriteByteRaw(0xC0);           // send the packet start character//发送数据包起始字符
  _writePacket(hdr, hdrLen);    // send the header//发送标题
  _writePacket(data, dataLen);  // send the data block//发送数据块
  WriteByteRaw(0xC0);           // send the packet end character//发送数据包结束字符
}

// Send a packet to the serial port while performing SLIP framing. The packet data comprises a header and an optional data block.//执行滑动帧时向串行端口发送数据包。分组数据包括报头和可选数据块。
// This is like writePacket except that it does a fast block write for both the header and the main data with no SLIP encoding. Used to send sync commands.//这与writePacket类似，只是它对头和主数据都执行快速块写入，无需滑动编码。用于发送同步命令。
void writePacketRaw(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) {
  WriteByteRaw(0xC0);             // send the packet start character//发送数据包起始字符
  _writePacketRaw(hdr, hdrLen);   // send the header//发送标题
  _writePacketRaw(data, dataLen); // send the data block in raw mode//以原始模式发送数据块
  WriteByteRaw(0xC0);             // send the packet end character//发送数据包结束字符
}

// Send a command to the attached device together with the supplied data, if any.//将命令与提供的数据（如果有）一起发送到连接的设备。
// The data is supplied via a list of one or more segments.//数据通过一个或多个段的列表提供。
void sendCommand(uint8_t op, uint32_t checkVal, const uint8_t *data, size_t dataLen) {
  // populate the header//填充标题
  uint8_t hdr[8];
  putData(0, 1, hdr, 0);
  putData(op, 1, hdr, 1);
  putData(dataLen, 2, hdr, 2);
  putData(checkVal, 4, hdr, 4);

  // send the packet//发送数据包
  if (op == ESP_SYNC)
    writePacketRaw(hdr, sizeof(hdr), data, dataLen);
  else
    writePacket(hdr, sizeof(hdr), data, dataLen);
}

// Send a command to the attached device together with the supplied data, if any, and get the response//将命令与提供的数据（如果有）一起发送到连接的设备，并获取响应
EspUploadResult doCommand(uint8_t op, const uint8_t *data, size_t dataLen, uint32_t checkVal, uint32_t *valp, uint32_t msTimeout) {
  size_t bodyLen;
  EspUploadResult stat;

  sendCommand(op, checkVal, data, dataLen);

  stat = readPacket(op, valp, &bodyLen, msTimeout);
  if (stat == success && bodyLen != 2)
    stat = badReply;

  return stat;
}

// Send a synchronising packet to the serial port in an attempt to induce//向串行端口发送一个同步数据包，试图诱导
// the ESP8266 to auto-baud lock on the baud rate.//ESP8266用于自动锁定波特率。
EspUploadResult Sync(uint16_t timeout) {
  uint8_t buf[36];
  EspUploadResult stat;
  int i ;

  // compose the data for the sync attempt//为同步尝试撰写数据
  memset(buf, 0x55, sizeof(buf));
  buf[0] = 0x07;
  buf[1] = 0x07;
  buf[2] = 0x12;
  buf[3] = 0x20;

  stat = doCommand(ESP_SYNC, buf, sizeof(buf), 0, 0, timeout);

  // If we got a response other than sync, discard it and wait for a sync response. This happens at higher baud rates.//如果得到的响应不是同步，请放弃它并等待同步响应。这在波特率较高时发生。
  for (i = 0; i < 10 && stat == respHeader; ++i) {
    size_t bodyLen;
    stat = readPacket(ESP_SYNC, 0, &bodyLen, timeout);
  }

  if (stat == success) {
    // Read and discard additional replies//阅读并放弃其他回复
    for (;;) {
      size_t bodyLen;
      EspUploadResult rc = readPacket(ESP_SYNC, 0, &bodyLen, defaultTimeout);
      watchdog_refresh();
      if (rc != success || bodyLen != 2) break;
    }
  }
  //DEBUG//调试
  //else debug//printf("stat=%d\n", (int)stat);//else debug//printf（“stat=%d\n”，（int）stat）；
  return stat;
}

// Send a command to the device to begin the Flash process.//向设备发送命令以开始闪存过程。
EspUploadResult flashBegin(uint32_t addr, uint32_t size) {
  // determine the number of blocks represented by the size//确定由大小表示的块数
  uint32_t blkCnt;
  uint8_t buf[16];
  uint32_t timeout;

  blkCnt = (size + EspFlashBlockSize - 1) / EspFlashBlockSize;

  // ensure that the address is on a block boundary//确保地址位于块边界上
  addr &= ~(EspFlashBlockSize - 1);

  // begin the Flash process//开始闪存过程
  putData(size, 4, buf, 0);
  putData(blkCnt, 4, buf, 4);
  putData(EspFlashBlockSize, 4, buf, 8);
  putData(addr, 4, buf, 12);

  timeout = (size != 0) ? eraseTimeout : defaultTimeout;
  return doCommand(ESP_FLASH_BEGIN, buf, sizeof(buf), 0, 0, timeout);
}

// Send a command to the device to terminate the Flash process//向设备发送命令以终止闪存进程
EspUploadResult flashFinish(signed char reboot) {
  uint8_t buf[4];
  putData(reboot ? 0 : 1, 4, buf, 0);
  return doCommand(ESP_FLASH_END, buf, sizeof(buf), 0, 0, defaultTimeout);
}

// Compute the checksum of a block of data//计算数据块的校验和
uint16_t checksum(const uint8_t *data, uint16_t dataLen, uint16_t cksum) {
  if (data) while (dataLen--) cksum ^= (uint16_t)*data++;
  return cksum;
}

EspUploadResult flashWriteBlock(uint16_t flashParmVal, uint16_t flashParmMask) {
  const uint32_t blkSize = EspFlashBlockSize;
  int i;

  // Allocate a data buffer for the combined header and block data//为组合的标头和块数据分配数据缓冲区
  const uint16_t hdrOfst = 0;
  const uint16_t dataOfst = 16;
  const uint16_t blkBufSize = dataOfst + blkSize;
  uint32_t blkBuf32[blkBufSize/4];
  uint8_t * const blkBuf = (uint8_t*)(blkBuf32);
  uint32_t cnt;
  uint16_t cksum;
  EspUploadResult stat;

  // Prepare the header for the block//准备块的标题
  putData(blkSize, 4, blkBuf, hdrOfst + 0);
  putData(esp_upload.uploadBlockNumber, 4, blkBuf, hdrOfst + 4);
  putData(0, 4, blkBuf, hdrOfst + 8);
  putData(0, 4, blkBuf, hdrOfst + 12);

  // Get the data for the block//获取块的数据
  cnt = update_file.read(blkBuf + dataOfst, blkSize); //->Read(reinterpret_cast<char *>(blkBuf + dataOfst), blkSize);//->读取（重新解释cast<char*>（blkBuf+dataOfst），blkSize）；
  if (cnt != blkSize) {
    if (update_file.curPosition() == esp_upload.fileSize) {
      // partial last block, fill the remainder//部分最后一块，填充剩余部分
      memset(blkBuf + dataOfst + cnt, 0xFF, blkSize - cnt);
    }
    else
      return fileRead;
  }

  // Patch the flash parameters into the first block if it is loaded at address 0//如果闪存参数加载在地址0处，则将其修补到第一个块中
  if (esp_upload.uploadBlockNumber == 0 && esp_upload.uploadAddress == 0 && blkBuf[dataOfst] == ESP_IMAGE_MAGIC && flashParmMask != 0) {
    // update the Flash parameters//更新闪存参数
    uint32_t flashParm = getData(2, blkBuf + dataOfst + 2, 0) & ~(uint32_t)flashParmMask;
    putData(flashParm | flashParmVal, 2, blkBuf + dataOfst + 2, 0);
  }

  // Calculate the block checksum//计算块校验和
  cksum = checksum(blkBuf + dataOfst, blkSize, ESP_CHECKSUM_MAGIC);

  for (i = 0; i < 3; i++)
    if ((stat = doCommand(ESP_FLASH_DATA, blkBuf, blkBufSize, cksum, 0, blockWriteTimeout)) == success)
      break;
  return stat;
}

void upload_spin() {

  switch (esp_upload.state) {
    case resetting:
      if (esp_upload.connectAttemptNumber == 9) {
        esp_upload.uploadResult = connected;
        esp_upload.state = done;
      }
      else {
        uploadPort_begin();
        wifi_delay(2000);
        flushInput();
        esp_upload.lastAttemptTime = esp_upload.lastResetTime = getWifiTick();
        esp_upload.state = connecting;
      }
      break;

    case connecting:
      if ((getWifiTickDiff(esp_upload.lastAttemptTime, getWifiTick()) >= connectAttemptInterval) && (getWifiTickDiff(esp_upload.lastResetTime, getWifiTick()) >= 500)) {
        EspUploadResult res = Sync(5000);
        esp_upload.lastAttemptTime = getWifiTick();
        if (res == success)
          esp_upload.state = erasing;
        else {
          esp_upload.connectAttemptNumber++;
          if (esp_upload.connectAttemptNumber % retriesPerReset == 0)
            esp_upload.state = resetting;
        }
      }
      break;

    case erasing:
      if (getWifiTickDiff(esp_upload.lastAttemptTime, getWifiTick()) >= blockWriteInterval) {
        uint32_t eraseSize;
        const uint32_t sectorsPerBlock = 16;
        const uint32_t sectorSize = 4096;
        const uint32_t numSectors = (esp_upload.fileSize + sectorSize - 1)/sectorSize;
        const uint32_t startSector = esp_upload.uploadAddress/sectorSize;

        uint32_t headSectors = sectorsPerBlock - (startSector % sectorsPerBlock);
        NOMORE(headSectors, numSectors);

        eraseSize = (numSectors < 2 * headSectors)
                  ? (numSectors + 1) / 2 * sectorSize
                  : (numSectors - headSectors) * sectorSize;

        esp_upload.uploadResult = flashBegin(esp_upload.uploadAddress, eraseSize);
        if (esp_upload.uploadResult == success) {
          esp_upload.uploadBlockNumber = 0;
          esp_upload.uploadNextPercentToReport = percentToReportIncrement;
          esp_upload.lastAttemptTime = getWifiTick();
          esp_upload.state = uploading;
        }
        else
          esp_upload.state = done;
      }
      break;

    case uploading:
      // The ESP needs several milliseconds to recover from one packet before it will accept another//ESP需要几毫秒才能从一个数据包中恢复，然后才能接受另一个数据包
      if (getWifiTickDiff(esp_upload.lastAttemptTime, getWifiTick()) >= 15) {
        unsigned int percentComplete;
        const uint32_t blkCnt = (esp_upload.fileSize + EspFlashBlockSize - 1) / EspFlashBlockSize;
        if (esp_upload.uploadBlockNumber < blkCnt) {
          esp_upload.uploadResult = flashWriteBlock(0, 0);
          esp_upload.lastAttemptTime = getWifiTick();
          if (esp_upload.uploadResult != success)
            esp_upload.state = done;
          percentComplete = (100 * esp_upload.uploadBlockNumber)/blkCnt;
          ++esp_upload.uploadBlockNumber;
          if (percentComplete >= esp_upload.uploadNextPercentToReport)
            esp_upload.uploadNextPercentToReport += percentToReportIncrement;
        }
        else
          esp_upload.state = done;
      }
      break;

    case done:
      update_file.close();
      esp_upload.state = upload_idle;
      break;

    default: break;
  }
}

// Try to upload the given file at the given address//尝试在给定的地址上载给定的文件
void SendUpdateFile(const char *file, uint32_t address) {
  const char * const fname = card.diveToFile(false, update_curDir, ESP_FIRMWARE_FILE);
  if (!update_file.open(update_curDir, fname, O_READ)) return;

  esp_upload.fileSize = update_file.fileSize();

  if (esp_upload.fileSize == 0) {
    update_file.close();
    return;
  }

  esp_upload.uploadAddress = address;
  esp_upload.connectAttemptNumber = 0;
  esp_upload.state = resetting;
}

static const uint32_t FirmwareAddress = 0x00000000, WebFilesAddress = 0x00100000;

void ResetWiFiForUpload(int begin_or_end) {
  //#if 0//#如果0
  uint32_t start, now;

  start = getWifiTick();
  now = start;

  if (begin_or_end == 0) {
    SET_OUTPUT(WIFI_IO0_PIN);
    WRITE(WIFI_IO0_PIN, LOW);
  }
  else
    SET_INPUT_PULLUP(WIFI_IO0_PIN);

  WIFI_RESET();
  while (getWifiTickDiff(start, now) < 500) now = getWifiTick();
  WIFI_SET();
  //#endif//#恩迪夫
}

int32_t wifi_upload(int type) {
  esp_upload.retriesPerBaudRate = 9;

  ResetWiFiForUpload(0);

  switch (type) {
    case 0: SendUpdateFile(ESP_FIRMWARE_FILE, FirmwareAddress); break;
    case 1: SendUpdateFile(ESP_WEB_FIRMWARE_FILE, FirmwareAddress); break;
    case 2: SendUpdateFile(ESP_WEB_FILE, WebFilesAddress); break;
    default: return -1;
  }

  while (esp_upload.state != upload_idle) {
    upload_spin();
    watchdog_refresh();
  }

  ResetWiFiForUpload(1);

  return esp_upload.uploadResult == success ? 0 : -1;
}

#endif // HAS_TFT_LVGL_UI && MKS_WIFI_MODULE//具有TFT LVGL UI和MKS WIFI模块
