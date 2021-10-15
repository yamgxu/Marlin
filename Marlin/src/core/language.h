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

#include "../inc/MarlinConfig.h"

#define _UxGT(a) a

// Fallback if no language is set. DON'T CHANGE//如果未设置语言，则返回。不要改变
#ifndef LCD_LANGUAGE
  #define LCD_LANGUAGE en
#endif

// For character-based LCD controllers (DISPLAY_CHARSET_HD44780)//用于基于字符的LCD控制器（显示字符集HD44780）
#define JAPANESE 1
#define WESTERN  2
#define CYRILLIC 3

// NOTE: IF YOU CHANGE LANGUAGE FILES OR MERGE A FILE WITH CHANGES//注意：如果更改语言文件或将文件与更改合并
////
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRA_LCD" / "SDSUPPORT" #define IN "Configuration.h"//==>始终尝试编译带/不带“ULTIPANEL”/“ULTRA_LCD”/“SDSUPPORT”#在“Configuration.h”中定义的MARLIN
//   ==> ALSO TRY ALL AVAILABLE LANGUAGE OPTIONS//==>还可以尝试所有可用的语言选项
// See also https://marlinfw.org/docs/development/lcd_language.html//另见https://marlinfw.org/docs/development/lcd_language.html

// Languages//语言
// an         Aragonese//阿拉贡人
// bg         Bulgarian//保加利亚语
// ca         Catalan//加泰罗尼亚
// cz         Czech//捷克
// da         Danish//丹麦语
// de         German//德德语
// el         Greek//希腊语
// el_gr      Greek (Greece)//希腊语（希腊）
// en         English//英语
// es         Spanish//西班牙语
// eu         Basque-Euskera//欧盟巴斯克尤斯凯拉酒店
// fi         Finnish//芬兰语
// fr         French//法语
// gl         Galician//加利西亚
// hr         Croatian//克罗地亚人
// hu         Hungarian//匈牙利语
// it         Italian//是意大利人吗
// jp_kana    Japanese//日本假名
// ko_KR      Korean (South Korea)//ko_KR Korean（韩国）
// nl         Dutch//荷兰
// pl         Polish//光抛光
// pt         Portuguese//葡萄牙
// pt_br      Portuguese (Brazilian)//葡萄牙（巴西）
// ro         Romanian//罗马尼亚人
// ru         Russian//俄罗斯
// sk         Slovak//斯洛伐克语
// sv         Swedish//瑞典语
// tr         Turkish//土耳其
// uk         Ukrainian//英国乌克兰人
// vi         Vietnamese//六越南语
// zh_CN      Chinese (Simplified)//中文（简体）
// zh_TW      Chinese (Traditional)//中文（繁体）

#ifdef DEFAULT_SOURCE_CODE_URL
  #undef  SOURCE_CODE_URL
  #define SOURCE_CODE_URL DEFAULT_SOURCE_CODE_URL
#endif

#ifdef CUSTOM_MACHINE_NAME
  #undef  MACHINE_NAME
  #define MACHINE_NAME CUSTOM_MACHINE_NAME
#elif defined(DEFAULT_MACHINE_NAME)
  #undef  MACHINE_NAME
  #define MACHINE_NAME DEFAULT_MACHINE_NAME
#endif

#ifndef MACHINE_UUID
  #define MACHINE_UUID DEFAULT_MACHINE_UUID
#endif

#define MARLIN_WEBSITE_URL "marlinfw.org"

//#if !defined(STRING_SPLASH_LINE3) && defined(WEBSITE_URL)//#如果！已定义（字符串\u启动\u行3）和已定义（网站\u URL）
//  #define STRING_SPLASH_LINE3 WEBSITE_URL//#定义字符串_SPLASH_LINE3网站_URL
//#endif//#恩迪夫

////
// Common Serial Console Messages//通用串行控制台消息
// Don't change these strings because serial hosts look for them.//不要更改这些字符串，因为串行主机会查找它们。
////

#define STR_ENQUEUEING                      "enqueueing \""
#define STR_POWERUP                         "PowerUp"
#define STR_EXTERNAL_RESET                  " External Reset"
#define STR_BROWNOUT_RESET                  " Brown out Reset"
#define STR_WATCHDOG_RESET                  " Watchdog Reset"
#define STR_SOFTWARE_RESET                  " Software Reset"
#define STR_FREE_MEMORY                     " Free Memory: "
#define STR_PLANNER_BUFFER_BYTES            "  PlannerBufferBytes: "
#define STR_OK                              "ok"
#define STR_WAIT                            "wait"
#define STR_STATS                           "Stats: "
#define STR_FILE_SAVED                      "Done saving file."
#define STR_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "
#define STR_ERR_CHECKSUM_MISMATCH           "checksum mismatch, Last Line: "
#define STR_ERR_NO_CHECKSUM                 "No Checksum with line number, Last Line: "
#define STR_FILE_PRINTED                    "Done printing file"
#define STR_NO_MEDIA                        "No media"
#define STR_BEGIN_FILE_LIST                 "Begin file list"
#define STR_END_FILE_LIST                   "End file list"
#define STR_INVALID_EXTRUDER                "Invalid extruder"
#define STR_INVALID_E_STEPPER               "Invalid E stepper"
#define STR_E_STEPPER_NOT_SPECIFIED         "E stepper not specified"
#define STR_INVALID_SOLENOID                "Invalid solenoid"
#define STR_COUNT_X                         " Count X:"
#define STR_COUNT_A                         " Count A:"
#define STR_WATCHDOG_FIRED                  "Watchdog timeout. Reset required."
#define STR_ERR_KILLED                      "Printer halted. kill() called!"
#define STR_FLOWMETER_FAULT                 "Coolant flow fault. Flowmeter safety is active. Attention required."
#define STR_ERR_STOPPED                     "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define STR_ERR_SERIAL_MISMATCH             "Serial status mismatch"
#define STR_BUSY_PROCESSING                 "busy: processing"
#define STR_BUSY_PAUSED_FOR_USER            "busy: paused for user"
#define STR_BUSY_PAUSED_FOR_INPUT           "busy: paused for input"
#define STR_Z_MOVE_COMP                     "Z_move_comp"
#define STR_RESEND                          "Resend: "
#define STR_UNKNOWN_COMMAND                 "Unknown command: \""
#define STR_ACTIVE_EXTRUDER                 "Active Extruder: "

#define STR_PROBE_OFFSET                    "Probe Offset"
#define STR_SKEW_MIN                        "min_skew_factor: "
#define STR_SKEW_MAX                        "max_skew_factor: "
#define STR_ERR_MATERIAL_INDEX              "M145 S<index> out of range (0-1)"
#define STR_ERR_M421_PARAMETERS             "M421 incorrect parameter usage"
#define STR_ERR_BAD_PLANE_MODE              "G5 requires XY plane mode"
#define STR_ERR_MESH_XY                     "Mesh point out of range"
#define STR_ERR_ARC_ARGS                    "G2/G3 bad parameters"
#define STR_ERR_PROTECTED_PIN               "Protected Pin"
#define STR_ERR_M420_FAILED                 "Failed to enable Bed Leveling"
#define STR_ERR_M428_TOO_FAR                "Too far from reference point"
#define STR_ERR_M303_DISABLED               "PIDTEMP disabled"
#define STR_M119_REPORT                     "Reporting endstop status"
#define STR_ON                              "ON"
#define STR_OFF                             "OFF"
#define STR_ENDSTOP_HIT                     "TRIGGERED"
#define STR_ENDSTOP_OPEN                    "open"
#define STR_HOTEND_OFFSET                   "Hotend offsets:"
#define STR_DUPLICATION_MODE                "Duplication mode: "
#define STR_SOFT_ENDSTOPS                   "Soft endstops: "
#define STR_SOFT_MIN                        "  Min: "
#define STR_SOFT_MAX                        "  Max: "

#define STR_SAVED_POS                       "Position saved"
#define STR_RESTORING_POS                   "Restoring position"
#define STR_INVALID_POS_SLOT                "Invalid slot. Total: "

#define STR_SD_CANT_OPEN_SUBDIR             "Cannot open subdir "
#define STR_SD_INIT_FAIL                    "No SD card"
#define STR_SD_VOL_INIT_FAIL                "volume.init failed"
#define STR_SD_OPENROOT_FAIL                "openRoot failed"
#define STR_SD_CARD_OK                      "SD card ok"
#define STR_SD_WORKDIR_FAIL                 "workDir open failed"
#define STR_SD_OPEN_FILE_FAIL               "open failed, File: "
#define STR_SD_FILE_OPENED                  "File opened: "
#define STR_SD_SIZE                         " Size: "
#define STR_SD_FILE_SELECTED                "File selected"
#define STR_SD_WRITE_TO_FILE                "Writing to file: "
#define STR_SD_PRINTING_BYTE                "SD printing byte "
#define STR_SD_NOT_PRINTING                 "Not SD printing"
#define STR_SD_ERR_WRITE_TO_FILE            "error writing to file"
#define STR_SD_ERR_READ                     "SD read error"
#define STR_SD_CANT_ENTER_SUBDIR            "Cannot enter subdir: "

#define STR_ENDSTOPS_HIT                    "endstops hit: "
#define STR_ERR_COLD_EXTRUDE_STOP           " cold extrusion prevented"
#define STR_ERR_LONG_EXTRUDE_STOP           " too long extrusion prevented"
#define STR_ERR_HOTEND_TOO_COLD             "Hotend too cold"
#define STR_ERR_EEPROM_WRITE                "Error writing to EEPROM!"

#define STR_FILAMENT_CHANGE_HEAT_LCD        "Press button to heat nozzle"
#define STR_FILAMENT_CHANGE_INSERT_LCD      "Insert filament and press button"
#define STR_FILAMENT_CHANGE_WAIT_LCD        "Press button to resume"
#define STR_FILAMENT_CHANGE_HEAT_M108       "Send M108 to heat nozzle"
#define STR_FILAMENT_CHANGE_INSERT_M108     "Insert filament and send M108"
#define STR_FILAMENT_CHANGE_WAIT_M108       "Send M108 to resume"

#define STR_STOP_BLTOUCH                    "!! STOP called because of BLTouch error - restart with M999"
#define STR_STOP_UNHOMED                    "!! STOP called because of unhomed error - restart with M999"
#define STR_KILL_INACTIVE_TIME              "!! KILL caused by too much inactive time - current command: "
#define STR_KILL_BUTTON                     "!! KILL caused by KILL button/pin"

// temperature.cpp strings//温度.cpp字符串
#define STR_PID_AUTOTUNE_START              "PID Autotune start"
#define STR_PID_BAD_HEATER_ID               "PID Autotune failed! Bad heater id"
#define STR_PID_TEMP_TOO_HIGH               "PID Autotune failed! Temperature too high"
#define STR_PID_TIMEOUT                     "PID Autotune failed! timeout"
#define STR_BIAS                            " bias: "
#define STR_D_COLON                         " d: "
#define STR_T_MIN                           " min: "
#define STR_T_MAX                           " max: "
#define STR_KU                              " Ku: "
#define STR_TU                              " Tu: "
#define STR_CLASSIC_PID                     " Classic PID "
#define STR_KP                              " Kp: "
#define STR_KI                              " Ki: "
#define STR_KD                              " Kd: "
#define STR_PID_AUTOTUNE_FINISHED           "PID Autotune finished! Put the last Kp, Ki and Kd constants from below into Configuration.h"
#define STR_PID_DEBUG                       " PID_DEBUG "
#define STR_PID_DEBUG_INPUT                 ": Input "
#define STR_PID_DEBUG_OUTPUT                " Output "
#define STR_PID_DEBUG_PTERM                 " pTerm "
#define STR_PID_DEBUG_ITERM                 " iTerm "
#define STR_PID_DEBUG_DTERM                 " dTerm "
#define STR_PID_DEBUG_CTERM                 " cTerm "
#define STR_INVALID_EXTRUDER_NUM            " - Invalid extruder number !"

#define STR_HEATER_BED                      "bed"
#define STR_HEATER_CHAMBER                  "chamber"
#define STR_COOLER                          "cooler"
#define STR_LASER_TEMP                      "laser temperature"

#define STR_STOPPED_HEATER                  ", system stopped! Heater_ID: "
#define STR_REDUNDANCY                      "Heater switched off. Temperature difference between temp sensors is too high !"
#define STR_T_HEATING_FAILED                "Heating failed"
#define STR_T_THERMAL_RUNAWAY               "Thermal Runaway"
#define STR_T_MAXTEMP                       "MAXTEMP triggered"
#define STR_T_MINTEMP                       "MINTEMP triggered"
#define STR_ERR_PROBING_FAILED              "Probing Failed"
#define STR_ZPROBE_OUT_SER                  "Z Probe Past Bed"

// Debug//调试
#define STR_DEBUG_PREFIX                    "DEBUG:"
#define STR_DEBUG_OFF                       "off"
#define STR_DEBUG_ECHO                      "ECHO"
#define STR_DEBUG_INFO                      "INFO"
#define STR_DEBUG_ERRORS                    "ERRORS"
#define STR_DEBUG_DRYRUN                    "DRYRUN"
#define STR_DEBUG_COMMUNICATION             "COMMUNICATION"
#define STR_DEBUG_LEVELING                  "LEVELING"

#define STR_PRINTER_LOCKED                  "Printer locked! (Unlock with M511 or LCD)"
#define STR_WRONG_PASSWORD                  "Incorrect Password"
#define STR_PASSWORD_TOO_LONG               "Password too long"
#define STR_PASSWORD_REMOVED                "Password removed"
#define STR_REMINDER_SAVE_SETTINGS          "Remember to save!"
#define STR_PASSWORD_SET                    "Password is "

////
// Endstop Names used by Endstops::report_states//Endstops:：report_states使用的Endstop名称
////
#define STR_X_MIN                           "x_min"
#define STR_X_MAX                           "x_max"
#define STR_X2_MIN                          "x2_min"
#define STR_X2_MAX                          "x2_max"

#if HAS_Y_AXIS
  #define STR_Y_MIN                         "y_min"
  #define STR_Y_MAX                         "y_max"
  #define STR_Y2_MIN                        "y2_min"
  #define STR_Y2_MAX                        "y2_max"
#endif

#if HAS_Z_AXIS
  #define STR_Z_MIN                         "z_min"
  #define STR_Z_MAX                         "z_max"
  #define STR_Z2_MIN                        "z2_min"
  #define STR_Z2_MAX                        "z2_max"
  #define STR_Z3_MIN                        "z3_min"
  #define STR_Z3_MAX                        "z3_max"
  #define STR_Z4_MIN                        "z4_min"
  #define STR_Z4_MAX                        "z4_max"
#endif

#define STR_Z_PROBE                         "z_probe"
#define STR_PROBE_EN                        "probe_en"
#define STR_FILAMENT_RUNOUT_SENSOR          "filament"

// General axis names//通用轴名称
#define STR_X "X"
#define STR_Y "Y"
#define STR_Z "Z"
#define STR_I AXIS4_STR
#define STR_J AXIS5_STR
#define STR_K AXIS6_STR
#define STR_E "E"
#if IS_KINEMATIC
  #define STR_A "A"
  #define STR_B "B"
  #define STR_C "C"
#else
  #define STR_A "X"
  #define STR_B "Y"
  #define STR_C "Z"
#endif
#define STR_X2 "X2"
#define STR_Y2 "Y2"
#define STR_Z2 "Z2"
#define STR_Z3 "Z3"
#define STR_Z4 "Z4"

#define LCD_STR_A STR_A
#define LCD_STR_B STR_B
#define LCD_STR_C STR_C
#define LCD_STR_I STR_I
#define LCD_STR_J STR_J
#define LCD_STR_K STR_K
#define LCD_STR_E STR_E

// Extra Axis and Endstop Names//额外的轴和结束点名称
#if LINEAR_AXES >= 4
  #if AXIS4_NAME == 'A'
    #define AXIS4_STR "A"
    #define STR_I_MIN "a_min"
    #define STR_I_MAX "a_max"
  #elif AXIS4_NAME == 'B'
    #define AXIS4_STR "B"
    #define STR_I_MIN "b_min"
    #define STR_I_MAX "b_max"
  #elif AXIS4_NAME == 'C'
    #define AXIS4_STR "C"
    #define STR_I_MIN "c_min"
    #define STR_I_MAX "c_max"
  #elif AXIS4_NAME == 'U'
    #define AXIS4_STR "U"
    #define STR_I_MIN "u_min"
    #define STR_I_MAX "u_max"
  #elif AXIS4_NAME == 'V'
    #define AXIS4_STR "V"
    #define STR_I_MIN "v_min"
    #define STR_I_MAX "v_max"
  #elif AXIS4_NAME == 'W'
    #define AXIS4_STR "W"
    #define STR_I_MIN "w_min"
    #define STR_I_MAX "w_max"
  #else
    #define AXIS4_STR "A"
    #define STR_I_MIN "a_min"
    #define STR_I_MAX "a_max"
  #endif
#else
  #define AXIS4_STR   ""
#endif

#if LINEAR_AXES >= 5
  #if AXIS5_NAME == 'A'
    #define AXIS5_STR "A"
    #define STR_J_MIN "a_min"
    #define STR_J_MAX "a_max"
  #elif AXIS5_NAME == 'B'
    #define AXIS5_STR "B"
    #define STR_J_MIN "b_min"
    #define STR_J_MAX "b_max"
  #elif AXIS5_NAME == 'C'
    #define AXIS5_STR "C"
    #define STR_J_MIN "c_min"
    #define STR_J_MAX "c_max"
  #elif AXIS5_NAME == 'U'
    #define AXIS5_STR "U"
    #define STR_J_MIN "u_min"
    #define STR_J_MAX "u_max"
  #elif AXIS5_NAME == 'V'
    #define AXIS5_STR "V"
    #define STR_J_MIN "v_min"
    #define STR_J_MAX "v_max"
  #elif AXIS5_NAME == 'W'
    #define AXIS5_STR "W"
    #define STR_J_MIN "w_min"
    #define STR_J_MAX "w_max"
  #else
    #define AXIS5_STR "B"
    #define STR_J_MIN "b_min"
    #define STR_J_MAX "b_max"
  #endif
#else
  #define AXIS5_STR   ""
#endif

#if LINEAR_AXES >= 6
  #if AXIS6_NAME == 'A'
    #define AXIS6_STR "A"
    #define STR_K_MIN "a_min"
    #define STR_K_MAX "a_max"
  #elif AXIS6_NAME == 'B'
    #define AXIS6_STR "B"
    #define STR_K_MIN "b_min"
    #define STR_K_MAX "b_max"
  #elif AXIS6_NAME == 'C'
    #define AXIS6_STR "C"
    #define STR_K_MIN "c_min"
    #define STR_K_MAX "c_max"
  #elif AXIS6_NAME == 'U'
    #define AXIS6_STR "U"
    #define STR_K_MIN "u_min"
    #define STR_K_MAX "u_max"
  #elif AXIS6_NAME == 'V'
    #define AXIS6_STR "V"
    #define STR_K_MIN "v_min"
    #define STR_K_MAX "v_max"
  #elif AXIS6_NAME == 'W'
    #define AXIS6_STR "W"
    #define STR_K_MIN "w_min"
    #define STR_K_MAX "w_max"
  #else
    #define AXIS6_STR "C"
    #define STR_K_MIN "c_min"
    #define STR_K_MAX "c_max"
  #endif
#else
  #define AXIS6_STR   ""
#endif

#if EITHER(HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL)

  // Custom characters defined in the first 8 characters of the LCD//LCD前8个字符中定义的自定义字符
  #define LCD_STR_BEDTEMP     "\x00" // Print only as a char. This will have 'unexpected' results when used in a string!//仅打印为字符。在字符串中使用时，将产生“意外”结果！
  #define LCD_STR_DEGREE      "\x01"
  #define LCD_STR_THERMOMETER "\x02" // Still used with string concatenation//仍然与字符串连接一起使用
  #define LCD_STR_UPLEVEL     "\x03"
  #define LCD_STR_REFRESH     "\x04"
  #define LCD_STR_FOLDER      "\x05"
  #define LCD_STR_FEEDRATE    "\x06"
  #define LCD_STR_CLOCK       "\x07"
  #define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */

#else
  ////
  // Custom characters from Marlin_symbols.fon which was merged into ISO10646-0-3.bdf//Marlin_symbols.fon中的自定义字符，已合并到ISO10646-0-3.bdf中
  // \x00 intentionally skipped to avoid problems in strings//\x00故意跳过以避免字符串中出现问题
  ////
  #define LCD_STR_REFRESH     "\x01"
  #define LCD_STR_FOLDER      "\x02"
  #define LCD_STR_ARROW_RIGHT "\x03"
  #define LCD_STR_UPLEVEL     "\x04"
  #define LCD_STR_CLOCK       "\x05"
  #define LCD_STR_FEEDRATE    "\x06"
  #define LCD_STR_BEDTEMP     "\x07"
  #define LCD_STR_THERMOMETER "\x08"
  #define LCD_STR_DEGREE      "\x09"

  #define LCD_STR_SPECIAL_MAX '\x09'
  // Maximum here is 0x1F because 0x20 is ' ' (space) and the normal charsets begin.//这里的最大值是0x1F，因为0x20是“”（空格），并且正常字符集开始。
  // Better stay below 0x10 because DISPLAY_CHARSET_HD44780_WESTERN begins here.//最好保持在0x10以下，因为显示从这里开始。

  // Symbol characters//符号字符
  #define LCD_STR_FILAM_DIA   "\xF8"
  #define LCD_STR_FILAM_MUL   "\xA4"

#endif

/**
 * Tool indexes for LCD display only
 *
 * By convention the LCD shows "E1" for the first extruder.
 * However, internal to Marlin E0/T0 is the first tool, and
 * most board silkscreens say "E0." Zero-based labels will
 * make these indexes consistent but this defies expectation.
 */
#if ENABLED(NUMBER_TOOLS_FROM_0)
  #define LCD_FIRST_TOOL 0
  #define LCD_STR_N0 "0"
  #define LCD_STR_N1 "1"
  #define LCD_STR_N2 "2"
  #define LCD_STR_N3 "3"
  #define LCD_STR_N4 "4"
  #define LCD_STR_N5 "5"
  #define LCD_STR_N6 "6"
  #define LCD_STR_N7 "7"
#else
  #define LCD_FIRST_TOOL 1
  #define LCD_STR_N0 "1"
  #define LCD_STR_N1 "2"
  #define LCD_STR_N2 "3"
  #define LCD_STR_N3 "4"
  #define LCD_STR_N4 "5"
  #define LCD_STR_N5 "6"
  #define LCD_STR_N6 "7"
  #define LCD_STR_N7 "8"
#endif

#define LCD_STR_E0 "E" LCD_STR_N0
#define LCD_STR_E1 "E" LCD_STR_N1
#define LCD_STR_E2 "E" LCD_STR_N2
#define LCD_STR_E3 "E" LCD_STR_N3
#define LCD_STR_E4 "E" LCD_STR_N4
#define LCD_STR_E5 "E" LCD_STR_N5
#define LCD_STR_E6 "E" LCD_STR_N6
#define LCD_STR_E7 "E" LCD_STR_N7

// Include localized LCD Menu Messages//包括本地化的LCD菜单消息

#define LANGUAGE_DATA_INCL_(M) STRINGIFY_(fontdata/langdata_##M.h)
#define LANGUAGE_DATA_INCL(M) LANGUAGE_DATA_INCL_(M)

#define LANGUAGE_INCL_(M) STRINGIFY_(../lcd/language/language_##M.h)
#define LANGUAGE_INCL(M) LANGUAGE_INCL_(M)

// Use superscripts, if possible. Evaluated at point of use.//如果可能的话，使用上标。在使用点进行评估。
#define SUPERSCRIPT_TWO   TERN(NOT_EXTENDED_ISO10646_1_5X7, "^2", "²")
#define SUPERSCRIPT_THREE TERN(NOT_EXTENDED_ISO10646_1_5X7, "^3", "³")

#include "multi_language.h"   // Allow multiple languages//允许使用多种语言

#include "../lcd/language/language_en.h"
#include LANGUAGE_INCL(LCD_LANGUAGE)
#include LANGUAGE_INCL(LCD_LANGUAGE_2)
#include LANGUAGE_INCL(LCD_LANGUAGE_3)
#include LANGUAGE_INCL(LCD_LANGUAGE_4)
#include LANGUAGE_INCL(LCD_LANGUAGE_5)

#if NONE(DISPLAY_CHARSET_ISO10646_1, \
         DISPLAY_CHARSET_ISO10646_5, \
         DISPLAY_CHARSET_ISO10646_KANA, \
         DISPLAY_CHARSET_ISO10646_GREEK, \
         DISPLAY_CHARSET_ISO10646_CN, \
         DISPLAY_CHARSET_ISO10646_TR, \
         DISPLAY_CHARSET_ISO10646_PL, \
         DISPLAY_CHARSET_ISO10646_CZ, \
         DISPLAY_CHARSET_ISO10646_SK)
  #define DISPLAY_CHARSET_ISO10646_1 // use the better font on full graphic displays.//在全图形显示上使用更好的字体。
#endif
