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
 * lcd/extui/anycubic_chiron/FileNavigator.cpp
 *
 * Extensible_UI implementation for Anycubic Chiron
 * Written By Nick Wells, 2020 [https://github.com/SwiftNick]
 *  (not affiliated with Anycubic, Ltd.)
 *
 * The AC panel wants files in block of 4 and can only display a flat list
 * This library allows full folder traversal or flat file display and supports both standerd and new style panels.
 *
 * ## Old Style TFT panel
 * Supported chars     {}[]-+=_"$%^&*()~<>|
 * Max display length  22 chars
 * Max path len        29 chars
 * (DOS 8.3 filepath max 29chars)
 * (long filepath Max 22)
 *
 * ## New TFT Panel Format file display format
 * Supported chars     {}[]-+=_!"$%^&*()~<>\|
 * Max display length  26 chars
 * Max path len        29 chars
 * (DOS 8.3 filepath must end '.GCO')
 * (long filepath must end '.gcode')
 *
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(ANYCUBIC_LCD_CHIRON)
#include "FileNavigator.h"
#include "chiron_tft.h"

using namespace ExtUI;

#define DEBUG_OUT ACDEBUG(AC_FILE)
#include "../../../core/debug_out.h"

namespace Anycubic {

FileNavigator filenavigator;
FileList  FileNavigator::filelist;                          // Instance of the Marlin file API//Marlin文件API的实例
uint16_t  FileNavigator::lastpanelindex;
uint16_t  FileNavigator::currentindex;                      // override the panel request//覆盖面板请求
uint8_t   FileNavigator::currentfolderdepth;
uint16_t  FileNavigator::currentfolderindex[MAX_FOLDER_DEPTH];   // track folder pos for iteration//跟踪迭代的文件夹位置
char      FileNavigator::currentfoldername[MAX_PATH_LEN + 1];   // Current folder path//当前文件夹路径

FileNavigator::FileNavigator() { reset(); }

void FileNavigator::reset() {
  DEBUG_ECHOLNPGM("reset()");
  currentfoldername[0] = '\0';
  currentfolderdepth = 0;
  currentindex = 0;
  lastpanelindex = 0;
  ZERO(currentfolderindex);

  // Start at root folder//从根文件夹开始
  while (!filelist.isAtRootDir()) filelist.upDir();
  refresh();
}

void FileNavigator::refresh() { filelist.refresh(); }

void FileNavigator::changeDIR(const char *folder) {
  if (currentfolderdepth >= MAX_FOLDER_DEPTH) return; // limit the folder depth//限制文件夹深度
  DEBUG_ECHOLNPAIR("FD:" , folderdepth, " FP:",currentindex, " currentfolder:", currentfoldername, " enter:", folder);
  currentfolderindex[currentfolderdepth] = currentindex;
  strcat(currentfoldername, folder);
  strcat(currentfoldername, "/");
  filelist.changeDir(folder);
  currentfolderdepth++;
  currentindex = 0;
}

void FileNavigator::upDIR() {
  DEBUG_ECHOLNPAIR("upDIR() from D:", currentfolderdepth, " N:", currentfoldername);
  if (!filelist.isAtRootDir()) {
    filelist.upDir();
    currentfolderdepth--;
    currentindex = currentfolderindex[currentfolderdepth]; // restore last position in the folder//还原文件夹中的最后一个位置
    filelist.seek(currentindex); // restore file information//还原文件信息
  }

  // Remove the child folder from the stored path//从存储路径中删除子文件夹
  if (currentfolderdepth == 0)
    currentfoldername[0] = '\0';
  else {
    char * const pos = strchr(currentfoldername, '/');
    *(pos + 1) = '\0';
  }
}

void FileNavigator::skiptofileindex(uint16_t skip) {
  if (skip == 0) return;
  while (skip > 0) {
    if (filelist.seek(currentindex)) {
      DEBUG_ECHOLNPAIR("CI:", currentindex, " FD:", currentfolderdepth, " N:", skip, " ", filelist.longFilename());
      if (!filelist.isDir()) {
        skip--;
        currentindex++;
      }
      else
        changeDIR(filelist.shortFilename());
    } // valid file//有效文件
    if (currentindex == filelist.count()) {
      if (currentfolderdepth > 0) {
        upDIR();
        currentindex++;
      }
      else break; // end of root folder//根文件夹的结尾
    } // end of folder//文件夹末尾
  } // files needed//需要的文件
  // No more files available.//没有更多可用文件。
}

#if ENABLED(AC_SD_FOLDER_VIEW) // SD Folder navigation//SD文件夹导航

  void FileNavigator::getFiles(uint16_t index, panel_type_t paneltype, uint8_t filesneeded) {
    if (index == 0) currentindex = 0;
    // Each time we change folder we reset the file index to 0 and keep track//每次更改文件夹时，我们都会将文件索引重置为0并保持跟踪
    // of the current position, since the TFT panel isn't aware of folder trees.//当前位置，因为TFT面板不知道文件夹树。
    if (index > 0) {
      --currentindex; // go back a file to take account of the .. we added to the root.//返回一个文件以考虑。。我们添加到根。
      if (index > lastpanelindex)
        currentindex += filesneeded;
      else
        currentindex = currentindex < 4 ? 0 : currentindex - filesneeded;
    }
    lastpanelindex = index;

    DEBUG_ECHOLNPAIR("index=", index, " currentindex=", currentindex);

    if (currentindex == 0 && currentfolderdepth > 0) { // Add a link to go up a folder//添加指向文件夹的链接
      // The new panel ignores entries that don't end in .GCO or .gcode so add and pad them.//新面板将忽略不以.GCO或.gcode结尾的条目，因此添加并填充它们。
      if (paneltype == AC_panel_new) {
        TFTSer.println("<<.GCO");
        Chiron.SendtoTFTLN(PSTR("..                  .gcode"));
      }
      else {
        TFTSer.println("<<");
        TFTSer.println("..");
      }
      filesneeded--;
    }

    for (uint16_t seek = currentindex; seek < currentindex + filesneeded; seek++) {
      if (filelist.seek(seek)) {
        sendFile(paneltype);
        DEBUG_ECHOLNPAIR("-", seek, " '", filelist.longFilename(), "' '", currentfoldername, "", filelist.shortFilename(), "'");
      }
    }
  }

  void FileNavigator::sendFile(panel_type_t paneltype) {
    if (filelist.isDir()) {
      // Add mandatory tags for new panel otherwise lines are ignored.//为新面板添加必需的标记，否则将忽略行。
      if (paneltype == AC_panel_new) {
        TFTSer.print(filelist.shortFilename());
        TFTSer.println(".GCO");
        TFTSer.print(filelist.shortFilename());
        TFTSer.write('/');
        // Make sure we fill all 29 chars of the display line to clear the text buffer otherwise the last line is still visible//确保填充显示行的所有29个字符以清除文本缓冲区，否则最后一行仍然可见
        for (int8_t i = strlen(filelist.shortFilename()); i < 19; i++)
          TFTSer.write(' ');
        TFTSer.println(".gcode");
      }
      else {
        TFTSer.println(filelist.shortFilename());
        TFTSer.print(filelist.shortFilename());
        TFTSer.write('/');
        TFTSer.println();
      }
    }
    else { // Not DIR//不直接
      TFTSer.write('/');
      if (currentfolderdepth > 0) TFTSer.print(currentfoldername);
      TFTSer.println(filelist.shortFilename());
      TFTSer.print(filelist.longFilename());

      // Make sure we fill all 29 chars of the display line to clear the text buffer otherwise the last line is still visible//确保填充显示行的所有29个字符以清除文本缓冲区，否则最后一行仍然可见
      if (paneltype == AC_panel_new)
        for (int8_t i = strlen(filelist.longFilename()); i < 26; i++)
          TFTSer.write(' ');

      TFTSer.println();
    }
  }  // AC_SD_FOLDER_VIEW//AC_SD_文件夹_视图

#else // Flat file list//平面文件列表

  void FileNavigator::getFiles(uint16_t index, panel_type_t paneltype, uint8_t filesneeded) {
    DEBUG_ECHOLNPAIR("getFiles() I:", index," L:", lastpanelindex);
    // if we're searching backwards, jump back to start and search forward//如果我们在向后搜索，请跳回开始并向前搜索
    if (index < lastpanelindex) {
      reset();
      skiptofileindex(index);
    }
    lastpanelindex = index;

    while (filesneeded > 0) {
      if (filelist.seek(currentindex)) {
        if (!filelist.isDir()) {
          sendFile(paneltype);
          filesneeded--;
          currentindex++;
        }
        else
          changeDIR(filelist.shortFilename());
      } // valid file//有效文件

      if (currentindex == filelist.count()) {
        if (currentfolderdepth > 0) {
          upDIR();
          currentindex++;
        }
        else break; // end of root folder//根文件夹的结尾
      } // end of folder//文件夹末尾
    } // files needed//需要的文件
    // No more files available.//没有更多可用文件。
  }

  void FileNavigator::sendFile(panel_type_t paneltype) {
    TFTSer.write('/');
    if (currentfolderdepth > 0) TFTSer.print(currentfoldername);
    TFTSer.println(filelist.shortFilename());
    if (currentfolderdepth > 0) TFTSer.print(currentfoldername);
    TFTSer.println(filelist.longFilename());
    DEBUG_ECHOLNPAIR("/", currentfoldername, "", filelist.shortFilename(), " ", filelist.longFilename());
  }

#endif // Flat file list//平面文件列表

} // Anycubic namespace//任意立方名称空间

#endif // ANYCUBIC_LCD_CHIRON//任意立方_液晶_凯龙
