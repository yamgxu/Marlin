/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/* ****************************************
 * lcd/extui/nextion/nextion_tft_defs.h
 * ****************************************
 * Extensible_UI implementation for Nextion
 * https://github.com/Skorpi08//github.com/Skorpi08
 * ***************************************/

#include "../../../inc/MarlinConfigPre.h"

//#define NEXDEBUGLEVEL 255//#定义级别255
#if NEXDEBUGLEVEL
  // Bit-masks for selective debug://用于选择性调试的位掩码：
  enum NexDebugMask : uint8_t {
    N_INFO   = _BV(0),
    N_ACTION = _BV(1),
    N_FILE   = _BV(2),
    N_PANEL  = _BV(3),
    N_MARLIN = _BV(4),
    N_SOME   = _BV(5),
    N_ALL    = _BV(6)
  };
  #define NEXDEBUG(M) (((M) & NEXDEBUGLEVEL) == M)  // Debug flag macro//调试标志宏
#else
  #define NEXDEBUG(M) false
#endif

#define MAX_FOLDER_DEPTH                4    // Limit folder depth TFT has a limit for the file path//限制文件夹深度TFT对文件路径有限制
#define MAX_CMND_LEN                   16 * MAX_FOLDER_DEPTH // Maximum Length for a Panel command//面板命令的最大长度
#define MAX_PATH_LEN                   16 * MAX_FOLDER_DEPTH // Maximum number of characters in a SD file path//SD文件路径中的最大字符数

 // TFT panel commands//TFT面板命令
#define  msg_welcome                MACHINE_NAME " Ready."

#define SEND_TEMP(x,y,t,z)  (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR(".txt=\"")), LCD_SERIAL.print(y), nextion.SendtoTFT(PSTR(t)), LCD_SERIAL.print(z), nextion.SendtoTFT(PSTR("\"\xFF\xFF\xFF")))
#define SEND_VAL(x,y)       (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR(".val=")),   LCD_SERIAL.print(y), nextion.SendtoTFT(PSTR("\xFF\xFF\xFF")))
#define SEND_TXT(x,y)       (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR(".txt=\"")), nextion.SendtoTFT(PSTR(y)),  nextion.SendtoTFT(PSTR("\"\xFF\xFF\xFF")))
#define SEND_TXT_P(x,y)     (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR(".txt=\"")), nextion.SendtoTFT(y), nextion.SendtoTFT(PSTR("\"\xFF\xFF\xFF")))
#define SEND_VALasTXT(x,y)  (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR(".txt=\"")), LCD_SERIAL.print(y), nextion.SendtoTFT(PSTR("\"\xFF\xFF\xFF")))
#define SEND_TXT_END(x)     (nextion.SendtoTFT(PSTR(x)), nextion.SendtoTFT(PSTR("\xFF\xFF\xFF")))
#define SEND_PCO2(x,y,z)    (nextion.SendtoTFT(PSTR(x)), LCD_SERIAL.print(y), nextion.SendtoTFT(PSTR(".pco=")), nextion.SendtoTFT(PSTR(z)), nextion.SendtoTFT(PSTR("\xFF\xFF\xFF")))
