/** translatione by yx */
/**
 * STM32F1: MMCv3/SDv1/SDv2 (SPI mode) control module
 *
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2019 BigTreeTech [https://github.com/bigtreetech]
 * Copyright (C) 2015, ChaN, all right reserved.
 *
 * This software is a free software and there is NO WARRANTY.
 * No restriction on use. You can use, modify and redistribute it for
 * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 * Redistributions of source code must retain the above copyright notice.
 */

#ifdef __STM32F1__

#include "../../inc/MarlinConfig.h"

#if SD_CONNECTION_IS(ONBOARD)

#include "onboard_sd.h"
#include "SPI.h"
#include "fastio.h"

#ifndef ONBOARD_SPI_DEVICE
  #define ONBOARD_SPI_DEVICE SPI_DEVICE
#endif

#if HAS_SD_HOST_DRIVE
  #define ONBOARD_SD_SPI SPI
#else
  SPIClass OnboardSPI(ONBOARD_SPI_DEVICE);
  #define ONBOARD_SD_SPI OnboardSPI
#endif

#if ONBOARD_SPI_DEVICE == 1
  #define SPI_CLOCK_MAX SPI_BAUD_PCLK_DIV_4
#else
  #define SPI_CLOCK_MAX SPI_BAUD_PCLK_DIV_2
#endif

#define CS_LOW()  WRITE(ONBOARD_SD_CS_PIN, LOW)  // Set OnboardSPI cs low//设置板载SPI cs低
#define CS_HIGH() WRITE(ONBOARD_SD_CS_PIN, HIGH) // Set OnboardSPI cs high//设置板载SPI cs高

#define FCLK_FAST() ONBOARD_SD_SPI.setClockDivider(SPI_CLOCK_MAX)
#define FCLK_SLOW() ONBOARD_SD_SPI.setClockDivider(SPI_BAUD_PCLK_DIV_256)

/*--------------------------------------------------------------------------
   Module Private Functions
---------------------------------------------------------------------------*/

/* MMC/SD command */
#define CMD0  (0)         // GO_IDLE_STATE//进入空闲状态
#define CMD1  (1)         // SEND_OP_COND (MMC)//发送操作条件（MMC）
#define ACMD41  (0x80+41) // SEND_OP_COND (SDC)//发送操作条件（SDC）
#define CMD8  (8)         // SEND_IF_COND//如果有条件，请发送
#define CMD9  (9)         // SEND_CSD//发送到CSD
#define CMD10 (10)        // SEND_CID//发送CID
#define CMD12 (12)        // STOP_TRANSMISSION//停止传输
#define ACMD13  (0x80+13) // SD_STATUS (SDC)//SD_状态（SDC）
#define CMD16 (16)        // SET_BLOCKLEN//设置块
#define CMD17 (17)        // READ_SINGLE_BLOCK//读单块
#define CMD18 (18)        // READ_MULTIPLE_BLOCK//读取多个块
#define CMD23 (23)        // SET_BLOCK_COUNT (MMC)//设置块计数（MMC）
#define ACMD23  (0x80+23) // SET_WR_BLK_ERASE_COUNT (SDC)//设置擦除计数（SDC）
#define CMD24 (24)        // WRITE_BLOCK//写块
#define CMD25 (25)        // WRITE_MULTIPLE_BLOCK//写入多个块
#define CMD32 (32)        // ERASE_ER_BLK_START//抹去你的黑色开始
#define CMD33 (33)        // ERASE_ER_BLK_END//擦去你的头发
#define CMD38 (38)        // ERASE//抹去
#define CMD48 (48)        // READ_EXTR_SINGLE//读取外部数据
#define CMD49 (49)        // WRITE_EXTR_SINGLE//单写
#define CMD55 (55)        // APP_CMD//APP_CMD
#define CMD58 (58)        // READ_OCR//读取光学字符识别

static volatile DSTATUS Stat = STA_NOINIT;  // Physical drive status//物理驱动器状态
static volatile UINT timeout;
static BYTE CardType;                       // Card type flags//卡片类型标志

/*-----------------------------------------------------------------------*/
/* Send/Receive data to the MMC  (Platform dependent)                    */
/*-----------------------------------------------------------------------*/

/* Exchange a byte */
static BYTE xchg_spi (
  BYTE dat  // Data to send//要发送的数据
) {
  BYTE returnByte = ONBOARD_SD_SPI.transfer(dat);
  return returnByte;
}

/* Receive multiple byte */
static void rcvr_spi_multi (
  BYTE *buff,   // Pointer to data buffer//指向数据缓冲区的指针
  UINT btr      // Number of bytes to receive (16, 64 or 512)//要接收的字节数（16、64或512）
) {
  ONBOARD_SD_SPI.dmaTransfer(0, const_cast<uint8_t*>(buff), btr);
}

#if _DISKIO_WRITE

  // Send multiple bytes//发送多个字节
  static void xmit_spi_multi (
    const BYTE *buff, // Pointer to the data//指向数据的指针
    UINT btx          // Number of bytes to send (multiple of 16)//要发送的字节数（16的倍数）
  ) {
    ONBOARD_SD_SPI.dmaSend(const_cast<uint8_t*>(buff), btx);
  }

#endif // _DISKIO_WRITE//_DISKIO_写入

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static int wait_ready ( // 1:Ready, 0:Timeout//1:准备就绪，0:超时
  UINT wt               // Timeout [ms]//超时[毫秒]
) {
  BYTE d;
  timeout = millis() + wt;
  do {
    d = xchg_spi(0xFF);
    // This loop takes a while. Insert rot_rdq() here for multitask environment.//这个循环需要一段时间。对于多任务环境，在此处插入rot_rdq（）。
  } while (d != 0xFF && (timeout > millis()));  // Wait for card goes ready or timeout//等待卡准备就绪或超时

  return (d == 0xFF) ? 1 : 0;
}

/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static void deselect() {
  CS_HIGH();      // CS = H//CS=H
  xchg_spi(0xFF); // Dummy clock (force DO hi-z for multiple slave SPI)//虚拟时钟（多从SPI的强制DO hi-z）
}

/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static int select() {             // 1:OK, 0:Timeout//1:好，0:超时
  CS_LOW();                       // CS = L//CS=L
  xchg_spi(0xFF);                 // Dummy clock (force DO enabled)//虚拟时钟（强制DO启用）

  if (wait_ready(500)) return 1;  // Leading busy check: Wait for card ready//引导忙检查：等待卡就绪

  deselect();                     // Timeout//超时
  return 0;
}

/*-----------------------------------------------------------------------*/
/* Control SPI module (Platform dependent)                               */
/*-----------------------------------------------------------------------*/

// Enable SSP module and attach it to I/O pads//启用SSP模块并将其连接到I/O焊盘
static void sd_power_on() {
  ONBOARD_SD_SPI.setModule(ONBOARD_SPI_DEVICE);
  ONBOARD_SD_SPI.begin();
  ONBOARD_SD_SPI.setBitOrder(MSBFIRST);
  ONBOARD_SD_SPI.setDataMode(SPI_MODE0);
  OUT_WRITE(ONBOARD_SD_CS_PIN, HIGH); // Set CS# high//将CS设置为高
}

// Disable SPI function//禁用SPI功能
static void sd_power_off() {
  select();                           // Wait for card ready//等待卡片准备好
  deselect();
}

/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/

static int rcvr_datablock (   // 1:OK, 0:Error//1:好，0:错误
  BYTE *buff,                 // Data buffer//数据缓冲区
  UINT btr                    // Data block length (byte)//数据块长度（字节）
) {
  BYTE token;

  timeout = millis() + 200;
  do {                            // Wait for DataStart token in timeout of 200ms//等待DataStart令牌，超时时间为200ms
    token = xchg_spi(0xFF);
                                  // This loop will take a while. Insert rot_rdq() here for multitask environment.//这个循环需要一段时间。对于多任务环境，在此处插入rot_rdq（）。
  } while ((token == 0xFF) && (timeout > millis()));
  if (token != 0xFE) return 0;    // Function fails if invalid DataStart token or timeout//如果DataStart令牌无效或超时，函数将失败

  rcvr_spi_multi(buff, btr);      // Store trailing data to the buffer//将尾部数据存储到缓冲区
  xchg_spi(0xFF); xchg_spi(0xFF); // Discard CRC//丢弃CRC

  return 1;                       // Function succeeded//功能成功
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if _DISKIO_WRITE

  static int xmit_datablock(  // 1:OK, 0:Failed//1:正常，0:失败
    const BYTE *buff,         // Pointer to 512 byte data to be sent//指向要发送的512字节数据的指针
    BYTE token                // Token//代币
  ) {
    BYTE resp;

    if (!wait_ready(500)) return 0;       // Leading busy check: Wait for card ready to accept data block//引导忙检查：等待卡准备好接受数据块

    xchg_spi(token);                      // Send token//发送令牌
    if (token == 0xFD) return 1;          // Do not send data if token is StopTran//如果令牌是StopTran，则不发送数据

    xmit_spi_multi(buff, 512);            // Data//资料
    xchg_spi(0xFF); xchg_spi(0xFF);       // Dummy CRC//伪CRC

    resp = xchg_spi(0xFF);                // Receive data resp//接收数据响应

    return (resp & 0x1F) == 0x05 ? 1 : 0; // Data was accepted or not//数据是否被接受

    // Busy check is done at next transmission//忙检查在下一次传输时完成
  }

#endif // _DISKIO_WRITE//_DISKIO_写入

/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd( // Return value: R1 resp (bit7==1:Failed to send)//返回值：R1 resp（位7==1:发送失败）
  BYTE cmd,           // Command index//命令索引
  DWORD arg           // Argument//论据
) {
  BYTE n, res;

  if (cmd & 0x80) {   // Send a CMD55 prior to ACMD<n>//在ACMD之前发送CMD55<n>
    cmd &= 0x7F;
    res = send_cmd(CMD55, 0);
    if (res > 1) return res;
  }

  // Select the card and wait for ready except to stop multiple block read//选择卡并等待就绪，除非停止多块读取
  if (cmd != CMD12) {
    deselect();
    if (!select()) return 0xFF;
  }

  // Send command packet//发送命令包
  xchg_spi(0x40 | cmd);         // Start + command index//开始+命令索引
  xchg_spi((BYTE)(arg >> 24));  // Argument[31..24]//论点[31..24]
  xchg_spi((BYTE)(arg >> 16));  // Argument[23..16]//论点[23..16]
  xchg_spi((BYTE)(arg >> 8));   // Argument[15..8]//论点[15..8]
  xchg_spi((BYTE)arg);          // Argument[7..0]//参数[7..0]
  n = 0x01;                     // Dummy CRC + Stop//虚拟CRC+停止
  if (cmd == CMD0) n = 0x95;    // Valid CRC for CMD0(0)//CMD0（0）的有效CRC
  if (cmd == CMD8) n = 0x87;    // Valid CRC for CMD8(0x1AA)//CMD8的有效CRC（0x1AA）
  xchg_spi(n);

  // Receive command response//接收命令响应
  if (cmd == CMD12) xchg_spi(0xFF); // Discard the following byte when CMD12//当使用CMD12时，放弃以下字节
  n = 10;                           // Wait for response (10 bytes max)//等待响应（最多10字节）
  do
    res = xchg_spi(0xFF);
  while ((res & 0x80) && --n);

  return res;                   // Return received response//返回收到的响应
}

/*--------------------------------------------------------------------------
   Public Functions
---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
  BYTE drv                                                            // Physical drive number (0)//物理驱动器号（0）
) {
  BYTE n, cmd, ty, ocr[4];

  if (drv) return STA_NOINIT;                                         // Supports only drive 0//仅支持驱动器0
  sd_power_on();                                                      // Initialize SPI//初始化SPI

  if (Stat & STA_NODISK) return Stat;                                 // Is a card existing in the soket?//soket中是否有卡？

  FCLK_SLOW();
  for (n = 10; n; n--) xchg_spi(0xFF);                                // Send 80 dummy clocks//发送80个假时钟

  ty = 0;
  if (send_cmd(CMD0, 0) == 1) {                                       // Put the card SPI state//将卡置于SPI状态
    timeout = millis() + 1000;                                        // Initialization timeout = 1 sec//初始化超时=1秒
    if (send_cmd(CMD8, 0x1AA) == 1) {                                 // Is the catd SDv2?//是有线电视SDv2吗？
      for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);                // Get 32 bit return value of R7 resp//获取R7 resp的32位返回值
      if (ocr[2] == 0x01 && ocr[3] == 0xAA) {                         // Does the card support 2.7-3.6V?//该卡是否支持2.7-3.6V电压？
        while ((timeout > millis()) && send_cmd(ACMD41, 1UL << 30));  // Wait for end of initialization with ACMD41(HCS)//等待ACMD41（HCS）初始化结束
        if ((timeout > millis()) && send_cmd(CMD58, 0) == 0) {        // Check CCS bit in the OCR//检查OCR中的CCS位
          for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
          ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;          // Check if the card is SDv2//检查卡是否为SDv2
        }
      }
    }
    else {                                                            // Not an SDv2 card//不是SDv2卡
      if (send_cmd(ACMD41, 0) <= 1)   {                               // SDv1 or MMCv3?//SDv1还是MMCv3？
        ty = CT_SD1; cmd = ACMD41;                                    // SDv1 (ACMD41(0))//SDv1（ACMD41（0））
      }
      else {
        ty = CT_MMC; cmd = CMD1;                                      // MMCv3 (CMD1(0))//MMCv3（CMD1（0））
      }
      while ((timeout > millis()) && send_cmd(cmd, 0));               // Wait for the card leaves idle state//等待卡离开空闲状态
      if (!(timeout > millis()) || send_cmd(CMD16, 512) != 0)         // Set block length: 512//设置块长度：512
        ty = 0;
    }
  }
  CardType = ty;                                                      // Card type//卡片类型
  deselect();

  if (ty) {                                                           // OK//嗯
    FCLK_FAST();                                                      // Set fast clock//设置快时钟
    Stat &= ~STA_NOINIT;                                              // Clear STA_NOINIT flag//清除STA_NOINIT旗
  }
  else {                                                              // Failed//失败
    sd_power_off();
    Stat = STA_NOINIT;
  }

  return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
  BYTE drv                    // Physical drive number (0)//物理驱动器号（0）
) {
  if (drv) return STA_NOINIT; // Supports only drive 0//仅支持驱动器0
  return Stat;                // Return disk status//返回磁盘状态
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
  BYTE drv,     // Physical drive number (0)//物理驱动器号（0）
  BYTE *buff,   // Pointer to the data buffer to store read data//指向存储读取数据的数据缓冲区的指针
  DWORD sector, // Start sector number (LBA)//起始扇区号（LBA）
  UINT count    // Number of sectors to read (1..128)//要读取的扇区数（1..128）
) {
  BYTE cmd;

  if (drv || !count) return RES_PARERR;       // Check parameter//检查参数
  if (Stat & STA_NOINIT) return RES_NOTRDY;   // Check if drive is ready//检查驱动器是否准备就绪
  if (!(CardType & CT_BLOCK)) sector *= 512;  // LBA ot BA conversion (byte addressing cards)//LBA-ot-BA转换（字节寻址卡）
  FCLK_FAST();
  cmd = count > 1 ? CMD18 : CMD17;            //  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK//读取多块：读取单块
  if (send_cmd(cmd, sector) == 0) {
    do {
      if (!rcvr_datablock(buff, 512)) break;
      buff += 512;
    } while (--count);
    if (cmd == CMD18) send_cmd(CMD12, 0);     // STOP_TRANSMISSION//停止传输
  }
  deselect();

  return count ? RES_ERROR : RES_OK;          // Return result//返回结果
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _DISKIO_WRITE

  DRESULT disk_write(
    BYTE drv,                                   // Physical drive number (0)//物理驱动器号（0）
    const BYTE *buff,                           // Pointer to the data to write//指向要写入的数据的指针
    DWORD sector,                               // Start sector number (LBA)//起始扇区号（LBA）
    UINT count                                  // Number of sectors to write (1..128)//要写入的扇区数（1..128）
  ) {
    if (drv || !count) return RES_PARERR;       // Check parameter//检查参数
    if (Stat & STA_NOINIT) return RES_NOTRDY;   // Check drive status//检查驱动器状态
    if (Stat & STA_PROTECT) return RES_WRPRT;   // Check write protect//检查写保护
    FCLK_FAST();
    if (!(CardType & CT_BLOCK)) sector *= 512;  // LBA ==> BA conversion (byte addressing cards)//LBA==>BA转换（字节寻址卡）

    if (count == 1) {                           // Single sector write//单扇区写入
      if ((send_cmd(CMD24, sector) == 0)        // WRITE_BLOCK//写块
        && xmit_datablock(buff, 0xFE)) {
        count = 0;
      }
    }
    else {                                            // Multiple sector write//多扇区写入
      if (CardType & CT_SDC) send_cmd(ACMD23, count); // Predefine number of sectors//预定义扇区数
      if (send_cmd(CMD25, sector) == 0) {             // WRITE_MULTIPLE_BLOCK//写入多个块
        do {
          if (!xmit_datablock(buff, 0xFC)) break;
          buff += 512;
        } while (--count);
        if (!xmit_datablock(0, 0xFD)) count = 1;      // STOP_TRAN token//停止传送令牌
      }
    }
    deselect();

    return count ? RES_ERROR : RES_OK;                // Return result//返回结果
  }

#endif // _DISKIO_WRITE//_DISKIO_写入

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

#if _DISKIO_IOCTL

  DRESULT disk_ioctl (
    BYTE drv,   // Physical drive number (0)//物理驱动器号（0）
    BYTE cmd,   // Control command code//控制命令代码
    void *buff  // Pointer to the conrtol data//指向conrtol数据的指针
  ) {
    DRESULT res;
    BYTE n, csd[16], *ptr = (BYTE *)buff;
    DWORD *dp, st, ed, csize;
    #if _DISKIO_ISDIO
      SDIO_CMD *sdio = buff;
      BYTE rc, *buf;
      UINT dc;
    #endif

    if (drv) return RES_PARERR;                 // Check parameter//检查参数
    if (Stat & STA_NOINIT) return RES_NOTRDY;   // Check if drive is ready//检查驱动器是否准备就绪

    res = RES_ERROR;
    FCLK_FAST();
    switch (cmd) {
      case CTRL_SYNC:                           // Wait for end of internal write process of the drive//等待驱动器内部写入过程结束
        if (select()) res = RES_OK;
        break;

      case GET_SECTOR_COUNT:                    // Get drive capacity in unit of sector (DWORD)//以扇区为单位获取驱动器容量（DWORD）
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
          if ((csd[0] >> 6) == 1) {             // SDC ver 2.00//SDC 2.00版
            csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
            *(DWORD*)buff = csize << 10;
          }
          else {                                // SDC ver 1.XX or MMC ver 3//SDC版本1.XX或MMC版本3
            n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
            csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
            *(DWORD*)buff = csize << (n - 9);
          }
          res = RES_OK;
        }
        break;

      case GET_BLOCK_SIZE:                              // Get erase block size in unit of sector (DWORD)//获取以扇区（DWORD）为单位的擦除块大小
        if (CardType & CT_SD2) {                        // SDC ver 2.00//SDC 2.00版
          if (send_cmd(ACMD13, 0) == 0) {               // Read SD status//读取SD状态
            xchg_spi(0xFF);
            if (rcvr_datablock(csd, 16)) {              // Read partial block//读取部分块
              for (n = 64 - 16; n; n--) xchg_spi(0xFF); // Purge trailing data//清除尾随数据
              *(DWORD*)buff = 16UL << (csd[10] >> 4);
              res = RES_OK;
            }
          }
        }
        else {                                                        // SDC ver 1.XX or MMC//SDC版本1.XX或MMC
          if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {  // Read CSD//读CSD
            if (CardType & CT_SD1) {                                  // SDC ver 1.XX//SDC 1.XX版
              *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
            }
            else {                                                    // MMC//MMC
              *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
            }
            res = RES_OK;
          }
        }
        break;

      case CTRL_TRIM:                                   // Erase a block of sectors (used when _USE_TRIM in ffconf.h is 1)//擦除扇区块（当ffconf.h中的_USE_TRIM为1时使用）
        if (!(CardType & CT_SDC)) break;                // Check if the card is SDC//检查卡是否为SDC
        if (disk_ioctl(drv, MMC_GET_CSD, csd)) break;   // Get CSD//获得CSD
        if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break; // Check if sector erase can be applied to the card//检查是否可以对卡应用扇区擦除
        dp = (DWORD *)buff; st = dp[0]; ed = dp[1];     // Load sector block//加载扇区块
        if (!(CardType & CT_BLOCK)) {
          st *= 512; ed *= 512;
        }
        if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) { // Erase sector block//擦除扇区块
          res = RES_OK; // FatFs does not check result of this command//FatFs不检查此命令的结果
        }
        break;

      // The following commands are never used by FatFs module//FatFs模块从不使用以下命令

      case MMC_GET_TYPE:    // Get MMC/SDC type (BYTE)//获取MMC/SDC类型（字节）
        *ptr = CardType;
        res = RES_OK;
        break;

      case MMC_GET_CSD:     // Read CSD (16 bytes)//读取CSD（16字节）
        if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(ptr, 16)) {
          res = RES_OK;
        }
        break;

      case MMC_GET_CID:     // Read CID (16 bytes)//读取CID（16字节）
        if (send_cmd(CMD10, 0) == 0 && rcvr_datablock(ptr, 16)) {
          res = RES_OK;
        }
        break;

      case MMC_GET_OCR:     // Read OCR (4 bytes)//读取OCR（4字节）
        if (send_cmd(CMD58, 0) == 0) {
          for (n = 4; n; n--) *ptr++ = xchg_spi(0xFF);
          res = RES_OK;
        }
        break;

      case MMC_GET_SDSTAT:  // Read SD status (64 bytes)//读取SD状态（64字节）
        if (send_cmd(ACMD13, 0) == 0) {
          xchg_spi(0xFF);
          if (rcvr_datablock(ptr, 64)) res = RES_OK;
        }
        break;

      #if _DISKIO_ISDIO

        case ISDIO_READ:
          sdio = buff;
          if (send_cmd(CMD48, 0x80000000 | sdio->func << 28 | sdio->addr << 9 | ((sdio->ndata - 1) & 0x1FF)) == 0) {
            for (Timer1 = 1000; (rc = xchg_spi(0xFF)) == 0xFF && Timer1; ) ;
            if (rc == 0xFE) {
              for (buf = sdio->data, dc = sdio->ndata; dc; dc--) *buf++ = xchg_spi(0xFF);
              for (dc = 514 - sdio->ndata; dc; dc--) xchg_spi(0xFF);
              res = RES_OK;
            }
          }
          break;
        case ISDIO_WRITE:
          sdio = buff;
          if (send_cmd(CMD49, 0x80000000 | sdio->func << 28 | sdio->addr << 9 | ((sdio->ndata - 1) & 0x1FF)) == 0) {
            xchg_spi(0xFF); xchg_spi(0xFE);
            for (buf = sdio->data, dc = sdio->ndata; dc; dc--) xchg_spi(*buf++);
            for (dc = 514 - sdio->ndata; dc; dc--) xchg_spi(0xFF);
            if ((xchg_spi(0xFF) & 0x1F) == 0x05) res = RES_OK;
          }
          break;
        case ISDIO_MRITE:
          sdio = buff;
          if (send_cmd(CMD49, 0x84000000 | sdio->func << 28 | sdio->addr << 9 | sdio->ndata >> 8) == 0) {
            xchg_spi(0xFF); xchg_spi(0xFE);
            xchg_spi(sdio->ndata);
            for (dc = 513; dc; dc--) xchg_spi(0xFF);
            if ((xchg_spi(0xFF) & 0x1F) == 0x05) res = RES_OK;
          }
          break;

      #endif // _DISKIO_ISDIO//_DISKIO_ISDIO

      default: res = RES_PARERR;
    }

    deselect();
    return res;
  }

#endif // _DISKIO_IOCTL//_DISKIO_IOCTL

#endif // SD_CONNECTION_IS(ONBOARD)//SD_连接_（车载）
#endif // __STM32F1__//_uustm32f1__
