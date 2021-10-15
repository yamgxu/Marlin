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

#if HAS_TFT_LVGL_UI

#include "../../../inc/MarlinConfig.h"
#include "SPIFlashStorage.h"

extern W25QXXFlash W25QXX;

uint8_t SPIFlashStorage::m_pageData[SPI_FLASH_PageSize];
uint32_t SPIFlashStorage::m_currentPage;
uint16_t SPIFlashStorage::m_pageDataUsed;
uint32_t SPIFlashStorage::m_startAddress;

#if HAS_SPI_FLASH_COMPRESSION

  uint8_t SPIFlashStorage::m_compressedData[SPI_FLASH_PageSize];
  uint16_t SPIFlashStorage::m_compressedDataUsed;

  template <typename T>
  static uint32_t rle_compress(T *output, uint32_t outputLength, T *input, uint32_t inputLength, uint32_t& inputProcessed) {
    uint32_t count = 0, out = 0, index, i;
    T pixel;
    //32767 for uint16_t//uint16的32767
    //127 for uint16_t//127用于uint16\u t
    //calculated at compile time//在编译时计算
    constexpr T max = (0xFFFFFFFF >> (8 * (4 - sizeof(T)))) / 2;

    inputProcessed = 0;
    while (count < inputLength && out < outputLength) {
      index = count;
      pixel = input[index++];
      while (index < inputLength && index - count < max && input[index] == pixel)
        index++;
      if (index - count == 1) {
        /*
         * Failed to "replicate" the current pixel. See how many to copy.
         * Avoid a replicate run of only 2-pixels after a literal run. There
         * is no gain in this, and there is a risK of loss if the run after
         * the two identical pixels is another literal run. So search for
         * 3 identical pixels.
         */
        while (index < inputLength && index - count < max && (input[index] != input[index - 1] || (index > 1 && input[index] != input[index - 2])))
          index++;
        /*
         * Check why this run stopped. If it found two identical pixels, reset
         * the index so we can add a run. Do this twice: the previous run
         * tried to detect a replicate run of at least 3 pixels. So we may be
         * able to back up two pixels if such a replicate run was found.
         */
        while (index < inputLength && input[index] == input[index - 1])
          index--;
        // If the output buffer could overflow, stop at the remaining bytes//如果输出缓冲区可能溢出，请在剩余字节处停止
        NOMORE(index, count + outputLength - out - 1);
        output[out++] = (uint16_t)(count - index);
        for (i = count; i < index; i++)
          output[out++] = input[i];
      }
      else {
        // Need at least more 2 spaces//至少需要2个以上的空间
        if (out > outputLength - 2) break;
        output[out++] = (uint16_t)(index - count);
        output[out++] = pixel;
      }
      count = index;
    }
    inputProcessed = count;

    // Padding//填充物
    if (out == outputLength - 1) output[out++] = 0;

    return out;
  }

  template <typename UT, typename T>
  static uint32_t rle_uncompress(UT *output, uint32_t outputLength, UT *input, uint32_t inputLength, uint32_t &outputFilled) {
    T count;
    UT i;
    uint32_t processedBytes = 0;
    outputFilled = 0;

    while (outputLength > 0 && inputLength > 0) {
      processedBytes++;
      count = static_cast<T>(*input++);
      inputLength--;
      if (count > 0) { // Replicate run//复制运行
        for (i = 0; i < count && outputLength > i; i++)
          output[i] = *input;
        outputFilled += i;
        // If copy incomplete, change the input buffer to start with remaining data in the next call//如果复制不完整，则在下一次调用中更改输入缓冲区以剩余数据开始
        if (i < count) {
          // Change to process the difference in the next call//更改以在下一次调用中处理差异
          *(input - 1) = static_cast<UT>(count - i);
          return processedBytes - 1;
        }
        input++;
        inputLength--;
        processedBytes++;
      }
      else if (count < 0) { // literal run//文字运行
        count = static_cast<T>(-count);
        // Copy, validating if the output have enough space//复制，验证输出是否有足够的空间
        for (i = 0; i < count && outputLength > i; i++)
          output[i] = input[i];
        outputFilled += i;
        // If copy incomplete, change the input buffer to start with remaining data in the next call//如果复制不完整，则在下一次调用中更改输入缓冲区以剩余数据开始
        if (i < count) {
          input[i - 1] = static_cast<UT>((count - i) * -1);
          // Back one//后面一个
          return processedBytes + i - 1;
        }
        input += count;
        inputLength -= count;
        processedBytes += count;
      }
      output += count;
      outputLength -= count;
    }

    return processedBytes;
  }

#endif // HAS_SPI_FLASH_COMPRESSION//有SPI闪存压缩吗

void SPIFlashStorage::beginWrite(uint32_t startAddress) {
  m_pageDataUsed = 0;
  m_currentPage = 0;
  m_startAddress = startAddress;
  #if HAS_SPI_FLASH_COMPRESSION
    // Restart the compressed buffer, keep the pointers of the uncompressed buffer//重新启动压缩缓冲区，保留未压缩缓冲区的指针
    m_compressedDataUsed = 0;
  #endif
}


void SPIFlashStorage::endWrite() {
  // Flush remaining data//刷新剩余数据
  #if HAS_SPI_FLASH_COMPRESSION
    if (m_compressedDataUsed > 0) {
      flushPage();
      savePage(m_compressedData);
    }
  #else
    if (m_pageDataUsed > 0) flushPage();
  #endif
}

void SPIFlashStorage::savePage(uint8_t *buffer) {
  W25QXX.SPI_FLASH_BufferWrite(buffer, m_startAddress + (SPI_FLASH_PageSize * m_currentPage), SPI_FLASH_PageSize);
  // Test env//测试环境
  // char fname[256];//字符fname[256]；
  // snprintf(fname, sizeof(fname), "./pages/page-%03d.data", m_currentPage);//snprintf（fname，sizeof（fname），“/pages/页-%03d.数据”，m_currentPage）；
  // FILE *fp = fopen(fname, "wb");//文件*fp=fopen（fname，“wb”）；
  // fwrite(buffer, 1, m_compressedDataUsed, fp);//fwrite（缓冲区，1，m_压缩，fp）；
  // fclose(fp);//fclose（fp）；
}

void SPIFlashStorage::loadPage(uint8_t *buffer) {
  W25QXX.SPI_FLASH_BufferRead(buffer, m_startAddress + (SPI_FLASH_PageSize * m_currentPage), SPI_FLASH_PageSize);
  // Test env//测试环境
  // char fname[256];//字符fname[256]；
  // snprintf(fname, sizeof(fname), "./pages/page-%03d.data", m_currentPage);//snprintf（fname，sizeof（fname），“/pages/页-%03d.数据”，m_currentPage）；
  // FILE *fp = fopen(fname, "rb");//文件*fp=fopen（fname，“rb”）；
  // if (fp) {//if（fp）{
  //     fread(buffer, 1, SPI_FLASH_PageSize, fp);//fread（缓冲区，1，SPI_FLASH_页面大小，fp）；
  //     fclose(fp);//fclose（fp）；
  // }// }
}

void SPIFlashStorage::flushPage() {
  #if HAS_SPI_FLASH_COMPRESSION
    // Work com with compressed in memory//在内存中使用压缩的com
    uint32_t inputProcessed;
    uint32_t compressedSize = rle_compress<uint16_t>((uint16_t *)(m_compressedData + m_compressedDataUsed), compressedDataFree() / 2, (uint16_t *)m_pageData, m_pageDataUsed / 2, inputProcessed) * 2;
    inputProcessed *= 2;
    m_compressedDataUsed += compressedSize;

    // Space remaining in the compressed buffer?//压缩缓冲区中剩余的空间？
    if (compressedDataFree() > 0) {
      // Free the uncompressed buffer//释放未压缩的缓冲区
      m_pageDataUsed = 0;
      return;
    }

    // Part of the m_pageData was compressed, so ajust the pointers, freeing what was processed, shift the buffer//部分m_页面数据被压缩，因此调整指针，释放处理内容，移动缓冲区
    // TODO: To avoid this copy, use a circular buffer//TODO:要避免此复制，请使用循环缓冲区
    memmove(m_pageData, m_pageData + inputProcessed, m_pageDataUsed - inputProcessed);
    m_pageDataUsed -= inputProcessed;

    // No? So flush page with compressed data!!//没有？所以用压缩数据刷新页面！！
    uint8_t *buffer = m_compressedData;
  #else
    uint8_t *buffer = m_pageData;
  #endif

  savePage(buffer);

  #if HAS_SPI_FLASH_COMPRESSION
    // Restart the compressed buffer, keep the pointers of the uncompressed buffer//重新启动压缩缓冲区，保留未压缩缓冲区的指针
    m_compressedDataUsed = 0;
  #else
    m_pageDataUsed = 0;
  #endif
  m_currentPage++;
}

void SPIFlashStorage::readPage() {
  #if HAS_SPI_FLASH_COMPRESSION
    if (compressedDataFree() == 0) {
      loadPage(m_compressedData);
      m_currentPage++;
      m_compressedDataUsed = 0;
    }

    // Need to uncompress data//需要解压缩数据吗
    if (pageDataFree() == 0) {
      m_pageDataUsed = 0;
      uint32_t outpuProcessed = 0;
      uint32_t inputProcessed = rle_uncompress<uint16_t, int16_t>((uint16_t *)(m_pageData + m_pageDataUsed), pageDataFree() / 2, (uint16_t *)(m_compressedData + m_compressedDataUsed), compressedDataFree() / 2, outpuProcessed);
      inputProcessed *= 2;
      outpuProcessed *= 2;
      if (outpuProcessed < pageDataFree()) {
        m_pageDataUsed = SPI_FLASH_PageSize - outpuProcessed;
        // TODO: To avoid this copy, use a circular buffer//TODO:要避免此复制，请使用循环缓冲区
        memmove(m_pageData + m_pageDataUsed, m_pageData, outpuProcessed);
      }

      m_compressedDataUsed += inputProcessed;
    }
  #else
    loadPage(m_pageData);
    m_pageDataUsed = 0;
    m_currentPage++;
  #endif
}

uint16_t SPIFlashStorage::inData(uint8_t *data, uint16_t size) {
  // Don't write more than we can//不要写得太多
  NOMORE(size, pageDataFree());
  memcpy(m_pageData + m_pageDataUsed, data, size);
  m_pageDataUsed += size;
  return size;
}

void SPIFlashStorage::writeData(uint8_t *data, uint16_t size) {
  // Flush a page if needed//如果需要，刷新页面
  if (pageDataFree() == 0) flushPage();

  while (size > 0) {
    uint16_t written = inData(data, size);
    size -= written;
    // Need to write more? Flush page and continue!//需要写更多吗？刷新页面并继续！
    if (size > 0) {
      flushPage();
      data += written;
    }
  }
}

void SPIFlashStorage::beginRead(uint32_t startAddress) {
  m_startAddress = startAddress;
  m_currentPage = 0;
  // Nothing in memory now//现在什么都不记得了
  m_pageDataUsed = SPI_FLASH_PageSize;
  #if HAS_SPI_FLASH_COMPRESSION
    m_compressedDataUsed = sizeof(m_compressedData);
  #endif
}

uint16_t SPIFlashStorage::outData(uint8_t *data, uint16_t size) {
  // Don't read more than we have//不要比我们读的多
  NOMORE(size, pageDataFree());
  memcpy(data, m_pageData + m_pageDataUsed, size);
  m_pageDataUsed += size;
  return size;
}

void SPIFlashStorage::readData(uint8_t *data, uint16_t size) {
  // Read a page if needed//如果需要，读一页
  if (pageDataFree() == 0) readPage();

  while (size > 0) {
    uint16_t read = outData(data, size);
    size -= read;
    // Need to write more? Flush page and continue!//需要写更多吗？刷新页面并继续！
    if (size > 0) {
      readPage();
      data += read;
    }
  }
}

SPIFlashStorage SPIFlash;

#endif // HAS_TFT_LVGL_UI//有TFT\U LVGL\U用户界面
