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
 * sd/SdVolume.h
 *
 * Arduino SdFat Library
 * Copyright (c) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include <stdint.h>

#include "../inc/MarlinConfigPre.h"

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #include "usb_flashdrive/Sd2Card_FlashDrive.h"
#endif

#if NEED_SD2CARD_SDIO
  #include "Sd2Card_sdio.h"
#elif NEED_SD2CARD_SPI
  #include "Sd2Card.h"
#endif

#include "SdFatConfig.h"
#include "SdFatStructs.h"

//==============================================================================//==============================================================================
// SdVolume class//SdVolume类

/**
 * \brief Cache for an SD data block
 */
union cache_t {
  uint8_t         data[512];  // Used to access cached file data blocks.//用于访问缓存的文件数据块。
  uint16_t        fat16[256]; // Used to access cached FAT16 entries.//用于访问缓存的FAT16条目。
  uint32_t        fat32[128]; // Used to access cached FAT32 entries.//用于访问缓存的FAT32条目。
  dir_t           dir[16];    // Used to access cached directory entries.//用于访问缓存的目录项。
  mbr_t           mbr;        // Used to access a cached Master Boot Record.//用于访问缓存的主引导记录。
  fat_boot_t      fbs;        // Used to access to a cached FAT boot sector.//用于访问缓存的FAT引导扇区。
  fat32_boot_t    fbs32;      // Used to access to a cached FAT32 boot sector.//用于访问缓存的FAT32引导扇区。
  fat32_fsinfo_t  fsinfo;     // Used to access to a cached FAT32 FSINFO sector.//用于访问缓存的FAT32 FSINFO扇区。
};

/**
 * \class SdVolume
 * \brief Access FAT16 and FAT32 volumes on SD and SDHC cards.
 */
class SdVolume {
 public:
  // Create an instance of SdVolume//创建SdVolume的实例
  SdVolume() : fatType_(0) {}
  /**
   * Clear the cache and returns a pointer to the cache.  Used by the WaveRP
   * recorder to do raw write to the SD card.  Not for normal apps.
   * \return A pointer to the cache buffer or zero if an error occurs.
   */
  cache_t* cacheClear() {
    if (!cacheFlush()) return 0;
    cacheBlockNumber_ = 0xFFFFFFFF;
    return &cacheBuffer_;
  }

  /**
   * Initialize a FAT volume.  Try partition one first then try super
   * floppy format.
   *
   * \param[in] dev The DiskIODriver where the volume is located.
   *
   * \return true for success, false for failure.
   * Reasons for failure include not finding a valid partition, not finding
   * a valid FAT file system or an I/O error.
   */
  bool init(DiskIODriver *dev) { return init(dev, 1) || init(dev, 0); }
  bool init(DiskIODriver *dev, uint8_t part);

  // inline functions that return volume info//返回卷信息的内联函数
  uint8_t blocksPerCluster() const { return blocksPerCluster_; } //> \return The volume's cluster size in blocks.//>\返回卷的群集大小（以块为单位）。
  uint32_t blocksPerFat() const { return blocksPerFat_; }        //> \return The number of blocks in one FAT.//>\返回一个FAT中的块数。
  uint32_t clusterCount() const { return clusterCount_; }        //> \return The total number of clusters in the volume.//>\返回卷中群集的总数。
  uint8_t clusterSizeShift() const { return clusterSizeShift_; } //> \return The shift count required to multiply by blocksPerCluster.//>\返回与blocksPerCluster相乘所需的移位计数。
  uint32_t dataStartBlock() const { return dataStartBlock_; }    //> \return The logical block number for the start of file data.//>\返回文件数据开头的逻辑块号。
  uint8_t fatCount() const { return fatCount_; }                 //> \return The number of FAT structures on the volume.//>\返回卷上FAT结构的数量。
  uint32_t fatStartBlock() const { return fatStartBlock_; }      //> \return The logical block number for the start of the first FAT.//>\返回第一个FAT开头的逻辑块号。
  uint8_t fatType() const { return fatType_; }                   //> \return The FAT type of the volume. Values are 12, 16 or 32.//>\返回卷的FAT类型。值为12、16或32。
  int32_t freeClusterCount();
  uint32_t rootDirEntryCount() const { return rootDirEntryCount_; } /** \return The number of entries in the root directory for FAT16 volumes. */

  /**
   * \return The logical block number for the start of the root directory
   *   on FAT16 volumes or the first cluster number on FAT32 volumes.
   */
  uint32_t rootDirStart() const { return rootDirStart_; }

  /**
   * DiskIODriver object for this volume
   * \return pointer to DiskIODriver object.
   */
  DiskIODriver* sdCard() { return sdCard_; }

  /**
   * Debug access to FAT table
   *
   * \param[in] n cluster number.
   * \param[out] v value of entry
   * \return true for success or false for failure
   */
  bool dbgFat(uint32_t n, uint32_t *v) { return fatGet(n, v); }

 private:
  // Allow SdBaseFile access to SdVolume private data.//允许SdBaseFile访问SdVolume专用数据。
  friend class SdBaseFile;

  // value for dirty argument in cacheRawBlock to indicate read from cache//cacheRawBlock中脏参数的值，用于指示从缓存读取
  static bool const CACHE_FOR_READ = false;
  // value for dirty argument in cacheRawBlock to indicate write to cache//cacheRawBlock中脏参数的值，用于指示写入缓存
  static bool const CACHE_FOR_WRITE = true;

  #if USE_MULTIPLE_CARDS
    cache_t cacheBuffer_;        // 512 byte cache for device blocks//设备块的512字节缓存
    uint32_t cacheBlockNumber_;  // Logical number of block in the cache//缓存中的逻辑块数
    DiskIODriver *sdCard_;       // DiskIODriver object for cache//用于缓存的DiskIODriver对象
    bool cacheDirty_;            // cacheFlush() will write block if true//如果为true，cacheFlush（）将写入块
    uint32_t cacheMirrorBlock_;  // block number for mirror FAT//镜像FAT的块编号
  #else
    static cache_t cacheBuffer_;        // 512 byte cache for device blocks//设备块的512字节缓存
    static uint32_t cacheBlockNumber_;  // Logical number of block in the cache//缓存中的逻辑块数
    static DiskIODriver *sdCard_;       // DiskIODriver object for cache//用于缓存的DiskIODriver对象
    static bool cacheDirty_;            // cacheFlush() will write block if true//如果为true，cacheFlush（）将写入块
    static uint32_t cacheMirrorBlock_;  // block number for mirror FAT//镜像FAT的块编号
  #endif

  uint32_t allocSearchStart_;   // start cluster for alloc search//启动alloc搜索的群集
  uint8_t blocksPerCluster_;    // cluster size in blocks//以块为单位的簇大小
  uint32_t blocksPerFat_;       // FAT size in blocks//脂肪块大小
  uint32_t clusterCount_;       // clusters in one FAT//一脂成簇
  uint8_t clusterSizeShift_;    // shift to convert cluster count to block count//shift将群集计数转换为块计数
  uint32_t dataStartBlock_;     // first data block number//第一数据块编号
  uint8_t fatCount_;            // number of FATs on volume//体积上脂肪的数量
  uint32_t fatStartBlock_;      // start block for first FAT//第一个脂肪的起始块
  uint8_t fatType_;             // volume type (12, 16, OR 32)//卷类型（12、16或32）
  uint16_t rootDirEntryCount_;  // number of entries in FAT16 root dir//FAT16根目录中的条目数
  uint32_t rootDirStart_;       // root start block for FAT16, cluster for FAT32//FAT16的根起始块，FAT32的群集

  bool allocContiguous(uint32_t count, uint32_t *curCluster);
  uint8_t blockOfCluster(uint32_t position) const { return (position >> 9) & (blocksPerCluster_ - 1); }
  uint32_t clusterStartBlock(uint32_t cluster) const { return dataStartBlock_ + ((cluster - 2) << clusterSizeShift_); }
  uint32_t blockNumber(uint32_t cluster, uint32_t position) const { return clusterStartBlock(cluster) + blockOfCluster(position); }

  cache_t* cache() { return &cacheBuffer_; }
  uint32_t cacheBlockNumber() const { return cacheBlockNumber_; }

  #if USE_MULTIPLE_CARDS
    bool cacheFlush();
    bool cacheRawBlock(uint32_t blockNumber, bool dirty);
  #else
    static bool cacheFlush();
    static bool cacheRawBlock(uint32_t blockNumber, bool dirty);
  #endif

  // used by SdBaseFile write to assign cache to SD location//SdBaseFile写入用于将缓存分配给SD位置
  void cacheSetBlockNumber(uint32_t blockNumber, bool dirty) {
    cacheDirty_ = dirty;
    cacheBlockNumber_  = blockNumber;
  }
  void cacheSetDirty() { cacheDirty_ |= CACHE_FOR_WRITE; }
  bool chainSize(uint32_t beginCluster, uint32_t *size);
  bool fatGet(uint32_t cluster, uint32_t *value);
  bool fatPut(uint32_t cluster, uint32_t value);
  bool fatPutEOC(uint32_t cluster) { return fatPut(cluster, 0x0FFFFFFF); }
  bool freeChain(uint32_t cluster);
  bool isEOC(uint32_t cluster) const {
    if (FAT12_SUPPORT && fatType_ == 12) return  cluster >= FAT12EOC_MIN;
    if (fatType_ == 16) return cluster >= FAT16EOC_MIN;
    return  cluster >= FAT32EOC_MIN;
  }
  bool readBlock(uint32_t block, uint8_t *dst) { return sdCard_->readBlock(block, dst); }
  bool writeBlock(uint32_t block, const uint8_t *dst) { return sdCard_->writeBlock(block, dst); }
};
