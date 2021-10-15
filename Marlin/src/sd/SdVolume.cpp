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
 * sd/SdVolume.cpp
 *
 * Arduino SdFat Library
 * Copyright (c) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(SDSUPPORT)

#include "SdVolume.h"

#include "../MarlinCore.h"

#if !USE_MULTIPLE_CARDS
  // raw block cache//原始块缓存
  uint32_t SdVolume::cacheBlockNumber_;  // current block number//当前区块编号
  cache_t  SdVolume::cacheBuffer_;       // 512 byte cache for Sd2Card//Sd2Card的512字节缓存
  DiskIODriver *SdVolume::sdCard_;       // pointer to SD card object//指向SD卡对象的指针
  bool     SdVolume::cacheDirty_;        // cacheFlush() will write block if true//如果为true，cacheFlush（）将写入块
  uint32_t SdVolume::cacheMirrorBlock_;  // mirror  block for second FAT//第二脂肪镜块
#endif

// find a contiguous group of clusters//查找一组连续的群集
bool SdVolume::allocContiguous(uint32_t count, uint32_t *curCluster) {
  if (ENABLED(SDCARD_READONLY)) return false;

  // start of group//小组开始
  uint32_t bgnCluster;
  // end of group//小组结束
  uint32_t endCluster;
  // last cluster of FAT//最后一簇脂肪
  uint32_t fatEnd = clusterCount_ + 1;

  // flag to save place to start next search//保存位置以开始下一次搜索的标志
  bool setStart;

  // set search start cluster//设置搜索开始群集
  if (*curCluster) {
    // try to make file contiguous//尝试使文件连续
    bgnCluster = *curCluster + 1;

    // don't save new start location//不保存新的开始位置
    setStart = false;
  }
  else {
    // start at likely place for free cluster//从可用群集的可能位置开始
    bgnCluster = allocSearchStart_;

    // save next search start if one cluster//如果有一个群集，则保存下一个搜索开始
    setStart = count == 1;
  }
  // end of group//小组结束
  endCluster = bgnCluster;

  // search the FAT for free clusters//在FAT中搜索自由簇
  for (uint32_t n = 0;; n++, endCluster++) {
    // can't find space checked all clusters//找不到已选中的所有群集的空间
    if (n >= clusterCount_) return false;

    // past end - start from beginning of FAT//过去的结束-从脂肪的开始开始
    if (endCluster > fatEnd) {
      bgnCluster = endCluster = 2;
    }
    uint32_t f;
    if (!fatGet(endCluster, &f)) return false;

    if (f != 0) {
      // cluster in use try next cluster as bgnCluster//正在使用的群集作为bgnCluster尝试下一个群集
      bgnCluster = endCluster + 1;
    }
    else if ((endCluster - bgnCluster + 1) == count) {
      // done - found space//完成-找到空间
      break;
    }
  }
  // mark end of chain//标记链条末端
  if (!fatPutEOC(endCluster)) return false;

  // link clusters//链接群集
  while (endCluster > bgnCluster) {
    if (!fatPut(endCluster - 1, endCluster)) return false;
    endCluster--;
  }
  if (*curCluster != 0) {
    // connect chains//连接链
    if (!fatPut(*curCluster, bgnCluster)) return false;
  }
  // return first cluster number to caller//将第一个群集号返回给调用者
  *curCluster = bgnCluster;

  // remember possible next free cluster//还记得下一个可用集群吗
  if (setStart) allocSearchStart_ = bgnCluster + 1;

  return true;
}

bool SdVolume::cacheFlush() {
  #if DISABLED(SDCARD_READONLY)
    if (cacheDirty_) {
      if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data))
        return false;

      // mirror FAT tables//镜像胖表
      if (cacheMirrorBlock_) {
        if (!sdCard_->writeBlock(cacheMirrorBlock_, cacheBuffer_.data))
          return false;
        cacheMirrorBlock_ = 0;
      }
      cacheDirty_ = 0;
    }
  #endif
  return true;
}

bool SdVolume::cacheRawBlock(uint32_t blockNumber, bool dirty) {
  if (cacheBlockNumber_ != blockNumber) {
    if (!cacheFlush()) return false;
    if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) return false;
    cacheBlockNumber_ = blockNumber;
  }
  if (dirty) cacheDirty_ = true;
  return true;
}

// return the size in bytes of a cluster chain//返回群集链的大小（以字节为单位）
bool SdVolume::chainSize(uint32_t cluster, uint32_t *size) {
  uint32_t s = 0;
  do {
    if (!fatGet(cluster, &cluster)) return false;
    s += 512UL << clusterSizeShift_;
  } while (!isEOC(cluster));
  *size = s;
  return true;
}

// Fetch a FAT entry//获取一个大条目
bool SdVolume::fatGet(uint32_t cluster, uint32_t *value) {
  uint32_t lba;
  if (cluster > (clusterCount_ + 1)) return false;
  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) return false;
    index &= 0x1FF;
    uint16_t tmp = cacheBuffer_.data[index];
    index++;
    if (index == 512) {
      if (!cacheRawBlock(lba + 1, CACHE_FOR_READ)) return false;
      index = 0;
    }
    tmp |= cacheBuffer_.data[index] << 8;
    *value = cluster & 1 ? tmp >> 4 : tmp & 0xFFF;
    return true;
  }

  if (fatType_ == 16)
    lba = fatStartBlock_ + (cluster >> 8);
  else if (fatType_ == 32)
    lba = fatStartBlock_ + (cluster >> 7);
  else
    return false;

  if (lba != cacheBlockNumber_ && !cacheRawBlock(lba, CACHE_FOR_READ))
    return false;

  *value = (fatType_ == 16) ? cacheBuffer_.fat16[cluster & 0xFF] : (cacheBuffer_.fat32[cluster & 0x7F] & FAT32MASK);
  return true;
}

// Store a FAT entry//存储一个胖条目
bool SdVolume::fatPut(uint32_t cluster, uint32_t value) {
  if (ENABLED(SDCARD_READONLY)) return false;

  uint32_t lba;
  // error if reserved cluster//保留群集时出错
  if (cluster < 2) return false;

  // error if not in FAT//如果不是在FAT中，则出错
  if (cluster > (clusterCount_ + 1)) return false;

  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) return false;
    // mirror second FAT//镜像第二脂肪
    if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
    index &= 0x1FF;
    uint8_t tmp = value;
    if (cluster & 1) {
      tmp = (cacheBuffer_.data[index] & 0xF) | tmp << 4;
    }
    cacheBuffer_.data[index] = tmp;
    index++;
    if (index == 512) {
      lba++;
      index = 0;
      if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) return false;
      // mirror second FAT//镜像第二脂肪
      if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
    }
    tmp = value >> 4;
    if (!(cluster & 1)) {
      tmp = ((cacheBuffer_.data[index] & 0xF0)) | tmp >> 4;
    }
    cacheBuffer_.data[index] = tmp;
    return true;
  }

  if (fatType_ == 16)
    lba = fatStartBlock_ + (cluster >> 8);
  else if (fatType_ == 32)
    lba = fatStartBlock_ + (cluster >> 7);
  else
    return false;

  if (!cacheRawBlock(lba, CACHE_FOR_WRITE)) return false;

  // store entry//商店入口
  if (fatType_ == 16)
    cacheBuffer_.fat16[cluster & 0xFF] = value;
  else
    cacheBuffer_.fat32[cluster & 0x7F] = value;

  // mirror second FAT//镜像第二脂肪
  if (fatCount_ > 1) cacheMirrorBlock_ = lba + blocksPerFat_;
  return true;
}

// free a cluster chain//释放集群链
bool SdVolume::freeChain(uint32_t cluster) {
  // clear free cluster location//清除空闲群集位置
  allocSearchStart_ = 2;

  do {
    uint32_t next;
    if (!fatGet(cluster, &next)) return false;

    // free cluster//自由簇
    if (!fatPut(cluster, 0)) return false;

    cluster = next;
  } while (!isEOC(cluster));

  return true;
}

/** Volume free space in clusters.
 *
 * \return Count of free clusters for success or -1 if an error occurs.
 */
int32_t SdVolume::freeClusterCount() {
  uint32_t free = 0;
  uint16_t n;
  uint32_t todo = clusterCount_ + 2;

  if (fatType_ == 16)
    n = 256;
  else if (fatType_ == 32)
    n = 128;
  else // put FAT12 here//把FAT12放在这里
    return -1;

  for (uint32_t lba = fatStartBlock_; todo; todo -= n, lba++) {
    if (!cacheRawBlock(lba, CACHE_FOR_READ)) return -1;
    NOMORE(n, todo);
    if (fatType_ == 16) {
      for (uint16_t i = 0; i < n; i++)
        if (cacheBuffer_.fat16[i] == 0) free++;
    }
    else {
      for (uint16_t i = 0; i < n; i++)
        if (cacheBuffer_.fat32[i] == 0) free++;
    }
    #ifdef ESP32
      // Needed to reset the idle task watchdog timer on ESP32 as reading the complete FAT may easily//需要重置ESP32上的空闲任务看门狗计时器，因为读取完整FAT可能很容易
      // block for 10+ seconds. yield() is insufficient since it blocks lower prio tasks (e.g., idle).//阻挡10秒以上。yield（）不够，因为它会阻止较低优先级的任务（例如空闲）。
      static millis_t nextTaskTime = 0;
      const millis_t ms = millis();
      if (ELAPSED(ms, nextTaskTime)) {
        vTaskDelay(1);            // delay 1 tick (Minimum. Usually 10 or 1 ms depending on skdconfig.h)//延迟1滴答声（最小值。通常为10或1毫秒，取决于skdconfig.h）
        nextTaskTime = ms + 1000; // tickle the task manager again in 1 second//在1秒内再次挠痒痒任务管理器
      }
    #endif // ESP32//ESP32
  }
  return free;
}

/** Initialize a FAT volume.
 *
 * \param[in] dev The SD card where the volume is located.
 *
 * \param[in] part The partition to be used.  Legal values for \a part are
 * 1-4 to use the corresponding partition on a device formatted with
 * a MBR, Master Boot Record, or zero if the device is formatted as
 * a super floppy with the FAT boot sector in block zero.
 *
 * \return true for success, false for failure.
 * Reasons for failure include not finding a valid partition, not finding a valid
 * FAT file system in the specified partition or an I/O error.
 */
bool SdVolume::init(DiskIODriver* dev, uint8_t part) {
  uint32_t totalBlocks, volumeStartBlock = 0;
  fat32_boot_t *fbs;

  sdCard_ = dev;
  fatType_ = 0;
  allocSearchStart_ = 2;
  cacheDirty_ = 0;  // cacheFlush() will write block if true//如果为true，cacheFlush（）将写入块
  cacheMirrorBlock_ = 0;
  cacheBlockNumber_ = 0xFFFFFFFF;

  // if part == 0 assume super floppy with FAT boot sector in block zero//如果part==0，则假定超级软盘在块0中具有FAT引导扇区
  // if part > 0 assume mbr volume with partition table//如果部分>0，则假定mbr卷具有分区表
  if (part) {
    if (part > 4) return false;
    if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) return false;
    part_t *p = &cacheBuffer_.mbr.part[part - 1];
    if ((p->boot & 0x7F) != 0  || p->totalSectors < 100 || p->firstSector == 0)
      return false; // not a valid partition//不是有效的分区
    volumeStartBlock = p->firstSector;
  }
  if (!cacheRawBlock(volumeStartBlock, CACHE_FOR_READ)) return false;
  fbs = &cacheBuffer_.fbs32;
  if (fbs->bytesPerSector != 512 ||
      fbs->fatCount == 0 ||
      fbs->reservedSectorCount == 0 ||
      fbs->sectorsPerCluster == 0) {
    // not valid FAT volume//无效脂肪体积
    return false;
  }
  fatCount_ = fbs->fatCount;
  blocksPerCluster_ = fbs->sectorsPerCluster;
  // determine shift that is same as multiply by blocksPerCluster_//确定与乘以blocksPerCluster相同的移位_
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != _BV(clusterSizeShift_)) {
    // error if not power of 2//如果不是2的幂，则出错
    if (clusterSizeShift_++ > 7) return false;
  }
  blocksPerFat_ = fbs->sectorsPerFat16 ?
                  fbs->sectorsPerFat16 : fbs->sectorsPerFat32;

  fatStartBlock_ = volumeStartBlock + fbs->reservedSectorCount;

  // count for FAT16 zero for FAT32//FAT16计数为零，FAT32计数为零
  rootDirEntryCount_ = fbs->rootDirEntryCount;

  // directory start for FAT16 dataStart for FAT32//FAT32的FAT16数据启动目录
  rootDirStart_ = fatStartBlock_ + fbs->fatCount * blocksPerFat_;

  // data start for FAT16 and FAT32//FAT16和FAT32的数据启动
  dataStartBlock_ = rootDirStart_ + ((32 * fbs->rootDirEntryCount + 511) / 512);

  // total blocks for FAT16 or FAT32//FAT16或FAT32的总块数
  totalBlocks = fbs->totalSectors16 ?
                fbs->totalSectors16 : fbs->totalSectors32;

  // total data blocks//总数据块
  clusterCount_ = totalBlocks - (dataStartBlock_ - volumeStartBlock);

  // divide by cluster size to get cluster count//除以群集大小以获得群集计数
  clusterCount_ >>= clusterSizeShift_;

  // FAT type is determined by cluster count//FAT类型由群集计数确定
  if (clusterCount_ < 4085) {
    fatType_ = 12;
    if (!FAT12_SUPPORT) return false;
  }
  else if (clusterCount_ < 65525)
    fatType_ = 16;
  else {
    rootDirStart_ = fbs->fat32RootCluster;
    fatType_ = 32;
  }
  return true;
}

#endif // SDSUPPORT//SDSUPPORT
