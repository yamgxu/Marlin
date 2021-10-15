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
 * \file
 * \brief Sd2Card class for V2 SD/SDHC cards
 */

/**
 * Arduino Sd2Card Library
 * Copyright (c) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "SdFatConfig.h"
#include "SdInfo.h"
#include "disk_io_driver.h"

#include <stdint.h>

uint16_t const SD_INIT_TIMEOUT  = 2000,  // (ms) Init timeout//（毫秒）初始化超时
               SD_ERASE_TIMEOUT = 10000, // (ms) Erase timeout//（毫秒）擦除超时
               SD_READ_TIMEOUT  = 300,   // (ms) Read timeout//（毫秒）读取超时
               SD_WRITE_TIMEOUT = 600;   // (ms) Write timeout//（毫秒）写入超时

// SD card errors//SD卡错误
typedef enum : uint8_t {
  SD_CARD_ERROR_CMD0               = 0x01, // Timeout error for command CMD0 (initialize card in SPI mode)//命令CMD0超时错误（在SPI模式下初始化卡）
  SD_CARD_ERROR_CMD8               = 0x02, // CMD8 was not accepted - not a valid SD card//未接受CMD8-不是有效的SD卡
  SD_CARD_ERROR_CMD12              = 0x03, // Card returned an error response for CMD12 (write stop)//卡返回了CMD12的错误响应（写停止）
  SD_CARD_ERROR_CMD17              = 0x04, // Card returned an error response for CMD17 (read block)//卡返回了CMD17的错误响应（读取块）
  SD_CARD_ERROR_CMD18              = 0x05, // Card returned an error response for CMD18 (read multiple block)//卡返回了CMD18的错误响应（读取多个块）
  SD_CARD_ERROR_CMD24              = 0x06, // Card returned an error response for CMD24 (write block)//卡返回了CMD24的错误响应（写入块）
  SD_CARD_ERROR_CMD25              = 0x07, // WRITE_MULTIPLE_BLOCKS command failed//写入多个块命令失败
  SD_CARD_ERROR_CMD58              = 0x08, // Card returned an error response for CMD58 (read OCR)//卡返回了CMD58的错误响应（读取OCR）
  SD_CARD_ERROR_ACMD23             = 0x09, // SET_WR_BLK_ERASE_COUNT failed//设置\u WR\u BLK\u擦除\u计数失败
  SD_CARD_ERROR_ACMD41             = 0x0A, // ACMD41 initialization process timeout//ACMD41初始化进程超时
  SD_CARD_ERROR_BAD_CSD            = 0x0B, // Card returned a bad CSR version field//卡返回了错误的CSR版本字段
  SD_CARD_ERROR_ERASE              = 0x0C, // Erase block group command failed//擦除块组命令失败
  SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0x0D, // Card not capable of single block erase//不能进行单块擦除的卡
  SD_CARD_ERROR_ERASE_TIMEOUT      = 0x0E, // Erase sequence timed out//擦除序列超时
  SD_CARD_ERROR_READ               = 0x0F, // Card returned an error token instead of read data//卡返回错误令牌，而不是读取数据
  SD_CARD_ERROR_READ_REG           = 0x10, // Read CID or CSD failed//读取CID或CSD失败
  SD_CARD_ERROR_READ_TIMEOUT       = 0x11, // Timeout while waiting for start of read data//等待开始读取数据时超时
  SD_CARD_ERROR_STOP_TRAN          = 0x12, // Card did not accept STOP_TRAN_TOKEN//卡未接受STOP_TRAN_令牌
  SD_CARD_ERROR_WRITE              = 0x13, // Card returned an error token as a response to a write operation//卡返回一个错误令牌作为对写入操作的响应
  SD_CARD_ERROR_WRITE_BLOCK_ZERO   = 0x14, // REMOVE - not used ... attempt to write protected block zero//删除-未使用。。。尝试写入受保护的块零
  SD_CARD_ERROR_WRITE_MULTIPLE     = 0x15, // Card did not go ready for a multiple block write//卡未准备好进行多块写入
  SD_CARD_ERROR_WRITE_PROGRAMMING  = 0x16, // Card returned an error to a CMD13 status check after a write//写入后，卡向CMD13状态检查返回错误
  SD_CARD_ERROR_WRITE_TIMEOUT      = 0x17, // Timeout occurred during write programming//写入编程期间发生超时
  SD_CARD_ERROR_SCK_RATE           = 0x18, // Incorrect rate selected//选择的费率不正确
  SD_CARD_ERROR_INIT_NOT_CALLED    = 0x19, // Init() not called//未调用Init（）
  // 0x1A is unused now, it was: card returned an error for CMD59 (CRC_ON_OFF)//0x1A现在未使用，它是：卡为CMD59返回了一个错误（CRC\u开\u关）
  SD_CARD_ERROR_READ_CRC = 0x1B             // Invalid read CRC//无效读CRC
} sd_error_code_t;

// card types//卡片类型
uint8_t const SD_CARD_TYPE_SD1  = 1,        // Standard capacity V1 SD card//标准容量V1 SD卡
              SD_CARD_TYPE_SD2  = 2,        // Standard capacity V2 SD card//标准容量V2 SD卡
              SD_CARD_TYPE_SDHC = 3;        // High Capacity SD card//大容量SD卡

/**
 * Define SOFTWARE_SPI to use bit-bang SPI
 */
#if EITHER(MEGA_SOFT_SPI, USE_SOFTWARE_SPI)
  #define SOFTWARE_SPI
#endif

#if IS_TEENSY_35_36 || IS_TEENSY_40_41
  #include "NXP_SDHC.h"
  #define BUILTIN_SDCARD 254
#endif

/**
 * \class Sd2Card
 * \brief Raw access to SD and SDHC flash memory cards.
 */
class DiskIODriver_SPI_SD : public DiskIODriver {
public:

  DiskIODriver_SPI_SD() : errorCode_(SD_CARD_ERROR_INIT_NOT_CALLED), type_(0) {}

  bool erase(uint32_t firstBlock, uint32_t lastBlock);
  bool eraseSingleBlockEnable();

  /**
   *  Set SD error code.
   *  \param[in] code value for error code.
   */
  void error(const uint8_t code);

  /**
   * \return error code for last error. See Sd2Card.h for a list of error codes.
   */
  inline int errorCode() const { return errorCode_; }

  /** \return error data for last error. */
  inline int errorData() const { return status_; }

  /**
   * Initialize an SD flash memory card with default clock rate and chip
   * select pin.  See sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin).
   *
   * \return true for success or false for failure.
   */
  bool init(const uint8_t sckRateID, const pin_t chipSelectPin) override;

  bool setSckRate(const uint8_t sckRateID);

  /**
   * Return the card type: SD V1, SD V2 or SDHC
   * \return 0 - SD V1, 1 - SD V2, or 3 - SDHC.
   */
  int type() const { return type_; }

  /**
   * Read a card's CID register. The CID contains card identification
   * information such as Manufacturer ID, Product name, Product serial
   * number and Manufacturing date.
   *
   * \param[out] cid pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  bool readCID(cid_t *cid) { return readRegister(CMD10, cid); }

  /**
   * Read a card's CSD register. The CSD contains Card-Specific Data that
   * provides information regarding access to the card's contents.
   *
   * \param[out] csd pointer to area for returned data.
   *
   * \return true for success or false for failure.
   */
  inline bool readCSD(csd_t *csd) override { return readRegister(CMD9, csd); }

  bool readData(uint8_t *dst) override;
  bool readStart(uint32_t blockNumber) override;
  bool readStop() override;

  bool writeData(const uint8_t *src) override;
  bool writeStart(const uint32_t blockNumber, const uint32_t eraseCount) override;
  bool writeStop() override;

  bool readBlock(uint32_t block, uint8_t *dst) override;
  bool writeBlock(uint32_t blockNumber, const uint8_t *src) override;

  uint32_t cardSize() override;

  bool isReady() override { return ready; };

  void idle() override {}

private:
  bool ready = false;
  uint8_t chipSelectPin_,
          errorCode_,
          spiRate_,
          status_,
          type_;

  // private functions//私人职能
  inline uint8_t cardAcmd(const uint8_t cmd, const uint32_t arg) {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  uint8_t cardCommand(const uint8_t cmd, const uint32_t arg);

  bool readData(uint8_t *dst, const uint16_t count);
  bool readRegister(const uint8_t cmd, void *buf);
  void chipDeselect();
  void chipSelect();
  inline void type(const uint8_t value) { type_ = value; }
  bool waitNotBusy(const millis_t timeout_ms);
  bool writeData(const uint8_t token, const uint8_t *src);
};
