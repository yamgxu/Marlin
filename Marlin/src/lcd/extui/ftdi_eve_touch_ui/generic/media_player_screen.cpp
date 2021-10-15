/** translatione by yx */
/***************************
 * media_player_screen.cpp *
 ***************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
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

#include "../config.h"
#include "../screens.h"

/**
 * The MediaPlayerScreen allows an AVI to be played.
 *
 * It requires a special AVI file. The following video
 * and audio codecs must be used:
 *
 *    -vcodec mjpeg -pix_fmt yuvj420p
 *    -acodec adpcm_ima_wav
 *
 * To generate a 2 second static screen from a png file:
 *
 *   ffmpeg -i startup.png -vcodec mjpeg -pix_fmt yuvj420p -r 1 video.avi
 *   sox -n -r 44100 -b 8 -c 2 -L silence.wav trim 0.0 2.000
 *   ffmpeg -i silence.wav -acodec adpcm_ima_wav silence.avi
 *   ffmpeg -i video.avi -i silence.wav -c copy -map 0:v:0 -map 1:a:0 startup.avi
 */

#ifdef FTDI_MEDIA_PLAYER_SCREEN

#include "../archim2-flash/flash_storage.h"
#include "../archim2-flash/media_file_reader.h"

using namespace FTDI;

void MediaPlayerScreen::onEntry() {
  BaseScreen::onEntry();
  CLCD::turn_on_backlight();
  SoundPlayer::set_volume(255);
}

void MediaPlayerScreen::onRedraw(draw_mode_t) {
}

bool MediaPlayerScreen::playCardMedia() {
  #if ENABLED(SDSUPPORT)
    char fname[15];
    strcpy_P(fname, PSTR("STARTUP.AVI"));

    MediaFileReader reader;
    if (!reader.open(fname))
      return false;

    SERIAL_ECHO_MSG("Starting to play STARTUP.AVI");
    playStream(&reader, MediaFileReader::read);
    reader.close();
  #endif
  return true;
}

// Attempt to play media from the onboard SPI flash chip//尝试从板载SPI闪存芯片播放媒体
bool MediaPlayerScreen::playBootMedia() {
  UIFlashStorage::BootMediaReader reader;
  if (!reader.isAvailable()) return false;

  SERIAL_ECHO_MSG("Starting to play boot video");
  playStream(&reader, UIFlashStorage::BootMediaReader::read);
  return true;
}

void MediaPlayerScreen::playStream(void *obj, media_streamer_func_t *data_stream) {
  #if FTDI_API_LEVEL >= 810
    if (FTDI::ftdi_chip >= 810) {
      // Set up the media FIFO on the end of RAMG, as the top of RAMG//在RAMG的末端设置媒体FIFO，作为RAMG的顶部
      // will be used as the framebuffer.//将用作帧缓冲区。

      uint8_t        buf[512];
      const uint32_t block_size = 512;
      const uint32_t fifo_size  = block_size * 2;
      const uint32_t fifo_start = CLCD::MAP::RAM_G + CLCD::MAP::RAM_G_SIZE - fifo_size;

      CommandProcessor cmd;
      cmd.cmd(CMD_DLSTART)
         .cmd(CLEAR_COLOR_RGB(0x000000))
         .cmd(CLEAR(true,true,true))
         .cmd(DL::DL_DISPLAY)
         .cmd(CMD_SWAP)
         .execute()
         .cmd(CMD_DLSTART)
         .mediafifo(fifo_start, fifo_size)
         .playvideo(OPT_FULLSCREEN | OPT_MEDIAFIFO | OPT_NOTEAR | OPT_SOUND)
         .cmd(DL::DL_DISPLAY)
         .cmd(CMD_SWAP)
         .execute();

      uint32_t writePtr = 0;
      int16_t  nBytes;

      uint32_t t = millis();
      uint8_t timeouts;

      spiInit(SPI_HALF_SPEED); // Boost SPI speed for video playback//为视频播放提高SPI速度

      do {
        // Write block n//写入块n
        nBytes = (*data_stream)(obj, buf, block_size);
        if (nBytes == -1) break;

        if (millis() - t > 10) {
          ExtUI::yield();
          t = millis();
        }

        CLCD::mem_write_bulk (fifo_start + writePtr, buf, nBytes);

        // Wait for FTDI810 to finish playing block n-1//等待FTDI810完成播放区块n-1
        timeouts = 20;
        do {
          if (millis() - t > 10) {
            ExtUI::yield();
            t = millis();
            timeouts--;
            if (timeouts == 0) {
              SERIAL_ECHO_MSG("Timeout playing video");
              cmd.reset();
              goto exit;
            }
          }
        } while (CLCD::mem_read_32(CLCD::REG::MEDIAFIFO_READ) != writePtr);

        // Start playing block n//开始玩n区
        writePtr = (writePtr + nBytes) % fifo_size;
        CLCD::mem_write_32(CLCD::REG::MEDIAFIFO_WRITE, writePtr);
      } while (nBytes == block_size);

      SERIAL_ECHO_MSG("Done playing video");

    exit:
      spiInit(SD_SPI_SPEED); // Restore default speed//恢复默认速度

      // Since playing media overwrites RAMG, we need to reinitialize//因为播放媒体会覆盖RAMG，所以我们需要重新初始化
      // everything that is stored in RAMG.//存储在RAMG中的所有内容。
      cmd.cmd(CMD_DLSTART).execute();
      DLCache::init();
      StatusScreen::loadBitmaps();
    }
  #else
    UNUSED(obj);
    UNUSED(data_stream);
  #endif // FTDI_API_LEVEL >= 810//FTDI_API_等级>=810
}

#endif // FTDI_MEDIA_PLAYER_SCREEN//FTDI_媒体_播放器_屏幕
