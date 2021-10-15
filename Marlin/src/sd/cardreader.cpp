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

#include "../inc/MarlinConfig.h"

#if ENABLED(SDSUPPORT)

//#define DEBUG_CARDREADER//#定义调试卡读卡器

#include "cardreader.h"

#include "../MarlinCore.h"
#include "../lcd/marlinui.h"

#if ENABLED(DWIN_CREALITY_LCD)
  #include "../lcd/dwin/e3v2/dwin.h"
#endif

#include "../module/planner.h"        // for synchronize//用于同步
#include "../module/printcounter.h"
#include "../gcode/queue.h"
#include "../module/settings.h"
#include "../module/stepper/indirection.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../feature/e_parser.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../feature/powerloss.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../feature/pause.h"
#endif

#define DEBUG_OUT EITHER(DEBUG_CARDREADER, MARLIN_DEV_MODE)
#include "../core/debug_out.h"
#include "../libs/hex_print.h"

// extern//外行

PGMSTR(M21_STR, "M21");
PGMSTR(M23_STR, "M23 %s");
PGMSTR(M24_STR, "M24");

// public://公众：

card_flags_t CardReader::flag;
char CardReader::filename[FILENAME_LENGTH], CardReader::longFilename[LONG_FILENAME_LENGTH];

IF_DISABLED(NO_SD_AUTOSTART, uint8_t CardReader::autofile_index); // = 0// = 0

#if BOTH(HAS_MULTI_SERIAL, BINARY_FILE_TRANSFER)
  serial_index_t CardReader::transfer_port_index;
#endif

// private://私人：

SdFile CardReader::root, CardReader::workDir, CardReader::workDirParents[MAX_DIR_DEPTH];
uint8_t CardReader::workDirDepth;

#if ENABLED(SDCARD_SORT_ALPHA)

  uint16_t CardReader::sort_count;
  #if ENABLED(SDSORT_GCODE)
    bool CardReader::sort_alpha;
    int CardReader::sort_folders;
    //bool CardReader::sort_reverse;//布尔读卡器：：排序_反向；
  #endif

  #if ENABLED(SDSORT_DYNAMIC_RAM)
    uint8_t *CardReader::sort_order;
  #else
    uint8_t CardReader::sort_order[SDSORT_LIMIT];
  #endif

  #if ENABLED(SDSORT_USES_RAM)

    #if ENABLED(SDSORT_CACHE_NAMES)
      uint16_t CardReader::nrFiles; // Cached total file count//缓存的文件总数
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        char **CardReader::sortshort, **CardReader::sortnames;
      #else
        char CardReader::sortshort[SDSORT_LIMIT][FILENAME_LENGTH];
        char CardReader::sortnames[SDSORT_LIMIT][SORTED_LONGNAME_STORAGE];
      #endif
    #elif DISABLED(SDSORT_USES_STACK)
      char CardReader::sortnames[SDSORT_LIMIT][SORTED_LONGNAME_STORAGE];
    #endif

    #if HAS_FOLDER_SORTING
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        uint8_t *CardReader::isDir;
      #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
        uint8_t CardReader::isDir[(SDSORT_LIMIT+7)>>3];
      #endif
      #define IS_DIR(n) TEST(isDir[(n) >> 3], (n) & 0x07)
    #endif

  #endif // SDSORT_USES_RAM//SDSORT_使用_RAM

#endif // SDCARD_SORT_ALPHA//SDCARD_SORT_ALPHA

#if HAS_USB_FLASH_DRIVE
  DiskIODriver_USBFlash CardReader::media_driver_usbFlash;
#endif

#if NEED_SD2CARD_SDIO || NEED_SD2CARD_SPI
  CardReader::sdcard_driver_t CardReader::media_driver_sdcard;
#endif

DiskIODriver* CardReader::driver = nullptr;
SdVolume CardReader::volume;
SdFile CardReader::file;

#if HAS_MEDIA_SUBCALLS
  uint8_t CardReader::file_subcall_ctr;
  uint32_t CardReader::filespos[SD_PROCEDURE_DEPTH];
  char CardReader::proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
#endif

uint32_t CardReader::filesize, CardReader::sdpos;

CardReader::CardReader() {
  changeMedia(&
    #if HAS_USB_FLASH_DRIVE && !SHARED_VOLUME_IS(SD_ONBOARD)
      media_driver_usbFlash
    #else
      media_driver_sdcard
    #endif
  );

  #if ENABLED(SDCARD_SORT_ALPHA)
    sort_count = 0;
    #if ENABLED(SDSORT_GCODE)
      sort_alpha = true;
      sort_folders = FOLDER_SORTING;
      //sort_reverse = false;//sort_reverse=false；
    #endif
  #endif

  flag.sdprinting = flag.sdprintdone = flag.mounted = flag.saving = flag.logging = false;
  filesize = sdpos = 0;

  TERN_(HAS_MEDIA_SUBCALLS, file_subcall_ctr = 0);

  IF_DISABLED(NO_SD_AUTOSTART, autofile_cancel());

  workDirDepth = 0;
  ZERO(workDirParents);

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
  #endif

  #if PIN_EXISTS(SDPOWER)
    OUT_WRITE(SDPOWER_PIN, HIGH); // Power the SD reader//为SD读卡器供电
  #endif
}

////
// Get a DOS 8.3 filename in its useful form//以有用的形式获取DOS 8.3文件名
////
char *createFilename(char * const buffer, const dir_t &p) {
  char *pos = buffer;
  LOOP_L_N(i, 11) {
    if (p.name[i] == ' ') continue;
    if (i == 8) *pos++ = '.';
    *pos++ = p.name[i];
  }
  *pos++ = 0;
  return buffer;
}

////
// Return 'true' if the item is a folder or G-code file//如果项目是文件夹或G代码文件，则返回“true”
////
bool CardReader::is_dir_or_gcode(const dir_t &p) {
  //uint8_t pn0 = p.name[0];//uint8_t pn0=p.name[0]；

  if ( (p.attributes & DIR_ATT_HIDDEN)                  // Hidden by attribute//按属性隐藏
    // When readDir() > 0 these must be false://当readDir（）大于0时，这些值必须为false：
    //|| pn0 == DIR_NAME_FREE || pn0 == DIR_NAME_DELETED  // Clear or Deleted entry//||pn0==DIR_NAME_FREE | | pn0==DIR_NAME_DELETED//清除或删除条目
    //|| pn0 == '.' || longFilename[0] == '.'             // Hidden file//||pn0='.| | longFilename[0]='.//隐藏文件
    //|| !DIR_IS_FILE_OR_SUBDIR(&p)                       // Not a File or Directory//|| !DIR\u是文件或子目录（&p）//不是文件或目录
  ) return false;

  flag.filenameIsDir = DIR_IS_SUBDIR(&p);               // We know it's a File or Folder//我们知道这是一个文件或文件夹

  return (
    flag.filenameIsDir                                  // All Directories are ok//所有目录都正常
    || (p.name[8] == 'G' && p.name[9] != '~')           // Non-backup *.G* files are accepted//接受非备份*.G*文件
  );
}

////
// Get the number of (compliant) items in the folder//获取文件夹中的（兼容）项目数
////
int CardReader::countItems(SdFile dir) {
  dir_t p;
  int c = 0;
  while (dir.readDir(&p, longFilename) > 0)
    c += is_dir_or_gcode(p);

  #if ALL(SDCARD_SORT_ALPHA, SDSORT_USES_RAM, SDSORT_CACHE_NAMES)
    nrFiles = c;
  #endif

  return c;
}

////
// Get file/folder info for an item by index//按索引获取项目的文件/文件夹信息
////
void CardReader::selectByIndex(SdFile dir, const uint8_t index) {
  dir_t p;
  for (uint8_t cnt = 0; dir.readDir(&p, longFilename) > 0;) {
    if (is_dir_or_gcode(p)) {
      if (cnt == index) {
        createFilename(filename, p);
        return;  // 0 based index//基于0的索引
      }
      cnt++;
    }
  }
}

////
// Get file/folder info for an item by name//按名称获取项目的文件/文件夹信息
////
void CardReader::selectByName(SdFile dir, const char * const match) {
  dir_t p;
  for (uint8_t cnt = 0; dir.readDir(&p, longFilename) > 0; cnt++) {
    if (is_dir_or_gcode(p)) {
      createFilename(filename, p);
      if (strcasecmp(match, filename) == 0) return;
    }
  }
}

////
// Recursive method to print all files within a folder in flat//在平面中打印文件夹中所有文件的递归方法
// DOS 8.3 format. This style of listing is the most compatible//DOS 8.3格式。这种类型的列表是最兼容的
// with legacy hosts.//使用遗留主机。
////
// This method recurses to unlimited depth and lists every//此方法递归到无限深度，并列出每个
// G-code file within the given parent. If the hierarchy is//给定父级中的G代码文件。如果层次结构是
// very deep this can blow up the stack, so a 'depth' parameter//非常深，这可能会炸毁堆栈，因此需要一个“深度”参数
// (as with printListingJSON) would be a good addition.//（与printListingJSON一样）将是一个很好的补充。
////
void CardReader::printListing(SdFile parent, const char * const prepend/*=nullptr*/) {
  dir_t p;
  while (parent.readDir(&p, longFilename) > 0) {
    if (DIR_IS_SUBDIR(&p)) {

      // Get the short name for the item, which we know is a folder//获取项目的短名称，我们知道它是一个文件夹
      char dosFilename[FILENAME_LENGTH];
      createFilename(dosFilename, p);

      // Allocate enough stack space for the full path to a folder, trailing slash, and nul//为文件夹的完整路径、尾随斜杠和nul分配足够的堆栈空间
      const bool prepend_is_empty = (!prepend || prepend[0] == '\0');
      const int len = (prepend_is_empty ? 1 : strlen(prepend)) + strlen(dosFilename) + 1 + 1;
      char path[len];

      // Append the FOLDERNAME12/ to the passed string.//将FOLDERNAME12/附加到传递的字符串。
      // It contains the full path to the "parent" argument.//它包含“parent”参数的完整路径。
      // We now have the full path to the item in this folder.//我们现在有了此文件夹中项目的完整路径。
      strcpy(path, prepend_is_empty ? "/" : prepend); // root slash if prepend is empty//如果prepend为空，则使用根斜杠
      strcat(path, dosFilename);                      // FILENAME_LENGTH characters maximum//文件名\u最大长度字符数
      strcat(path, "/");                              // 1 character//1个字符

      // Serial.print(path);//串行打印（路径）；

      // Get a new directory object using the full path//使用完整路径获取新的目录对象
      // and dive recursively into it.//然后递归地潜入其中。
      SdFile child; // child.close() in destructor//析构函数中的child.close（）
      if (child.open(&parent, dosFilename, O_READ))
        printListing(child, path);
      else {
        SERIAL_ECHO_MSG(STR_SD_CANT_OPEN_SUBDIR, dosFilename);
        return;
      }
    }
    else if (is_dir_or_gcode(p)) {
      if (prepend) SERIAL_ECHO(prepend);
      SERIAL_ECHO(createFilename(filename, p));
      SERIAL_CHAR(' ');
      SERIAL_ECHOLN(p.fileSize);
    }
  }
}

////
// List all files on the SD card//列出SD卡上的所有文件
////
void CardReader::ls() {
  if (flag.mounted) {
    root.rewind();
    printListing(root);
  }
}

#if ENABLED(LONG_FILENAME_HOST_SUPPORT)

  ////
  // Get a long pretty path based on a DOS 8.3 path//在DOS8.3路径的基础上获得一个长而漂亮的路径
  ////
  void CardReader::printLongPath(char * const path) {

    int i, pathLen = strlen(path);

    // SERIAL_ECHOPGM("Full Path: "); SERIAL_ECHOLN(path);//串行ECHOPGM（“完整路径：”）；串行_ECHOLN（路径）；

    // Zero out slashes to make segments//零斜杠以生成线段
    for (i = 0; i < pathLen; i++) if (path[i] == '/') path[i] = '\0';

    SdFile diveDir = root; // start from the root for segment 1//从段1的根开始
    for (i = 0; i < pathLen;) {

      if (path[i] == '\0') i++; // move past a single nul//越过一个nul

      char *segment = &path[i]; // The segment after most slashes//大多数斜杠后的线段

      // If a segment is empty (extra-slash) then exit//如果段为空（额外斜杠），则退出
      if (!*segment) break;

      // Go to the next segment//转到下一节
      while (path[++i]) { }

      //SERIAL_ECHOLNPAIR("Looking for segment: ", segment);//序列回波对（“查找段：”，段）；

      // Find the item, setting the long filename//查找项目，设置长文件名
      diveDir.rewind();
      selectByName(diveDir, segment);

      // Print /LongNamePart to serial output//打印/LongNamePart到串行输出
      SERIAL_CHAR('/');
      SERIAL_ECHO(longFilename[0] ? longFilename : "???");

      // If the filename was printed then that's it//如果文件名已打印，则仅此而已
      if (!flag.filenameIsDir) break;

      // SERIAL_ECHOPGM("Opening dir: "); SERIAL_ECHOLN(segment);//系列ECHOPGM（“开始目录：”）；序列号（段）；

      // Open the sub-item as the new dive parent//将子项作为新的父项打开
      SdFile dir;
      if (!dir.open(&diveDir, segment, O_READ)) {
        SERIAL_EOL();
        SERIAL_ECHO_START();
        SERIAL_ECHOPAIR(STR_SD_CANT_OPEN_SUBDIR, segment);
        break;
      }

      diveDir.close();
      diveDir = dir;

    } // while i<pathLen//而我是帕斯伦

    SERIAL_EOL();
  }

#endif // LONG_FILENAME_HOST_SUPPORT//长\u文件名\u主机\u支持

////
// Echo the DOS 8.3 filename (and long filename, if any)//回显DOS 8.3文件名（和长文件名，如果有的话）
////
void CardReader::printSelectedFilename() {
  if (file.isOpen()) {
    char dosFilename[FILENAME_LENGTH];
    file.getDosName(dosFilename);
    SERIAL_ECHO(dosFilename);
    #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
      selectFileByName(dosFilename);
      if (longFilename[0]) {
        SERIAL_CHAR(' ');
        SERIAL_ECHO(longFilename);
      }
    #endif
  }
  else
    SERIAL_ECHOPGM("(no file)");

  SERIAL_EOL();
}

void CardReader::mount() {
  flag.mounted = false;
  if (root.isOpen()) root.close();

  if (!driver->init(SD_SPI_SPEED, SDSS)
    #if defined(LCD_SDSS) && (LCD_SDSS != SDSS)
      && !driver->init(SD_SPI_SPEED, LCD_SDSS)
    #endif
  ) SERIAL_ECHO_MSG(STR_SD_INIT_FAIL);
  else if (!volume.init(driver))
    SERIAL_ERROR_MSG(STR_SD_VOL_INIT_FAIL);
  else if (!root.openRoot(&volume))
    SERIAL_ERROR_MSG(STR_SD_OPENROOT_FAIL);
  else {
    flag.mounted = true;
    SERIAL_ECHO_MSG(STR_SD_CARD_OK);
  }

  if (flag.mounted)
    cdroot();
  #if ENABLED(USB_FLASH_DRIVE_SUPPORT) || PIN_EXISTS(SD_DETECT)
    else if (marlin_state != MF_INITIALIZING)
      ui.set_status_P(GET_TEXT(MSG_SD_INIT_FAIL), -1);
  #endif

  ui.refresh();
}

/**
 * Handle SD card events
 */
#if MB(FYSETC_CHEETAH, FYSETC_AIO_II)
  #include "../module/stepper.h"
#endif

void CardReader::manage_media() {
  static uint8_t prev_stat = 2;       // First call, no prior state//第一次呼叫，无先前状态
  uint8_t stat = uint8_t(IS_SD_INSERTED());
  if (stat == prev_stat) return;

  DEBUG_ECHOLNPAIR("SD: Status changed from ", prev_stat, " to ", stat);

  flag.workDirIsRoot = true;          // Return to root on mount/release//装载/释放时返回到root

  if (ui.detected()) {

    uint8_t old_stat = prev_stat;
    prev_stat = stat;                 // Change now to prevent re-entry//现在更改以防止重新进入

    if (stat) {                       // Media Inserted//插入媒体
      safe_delay(500);                // Some boards need a delay to get settled//一些董事会需要延迟才能达成和解
      if (TERN1(SD_IGNORE_AT_STARTUP, old_stat != 2))
        mount();                      // Try to mount the media//尝试装入媒体
      #if MB(FYSETC_CHEETAH, FYSETC_CHEETAH_V12, FYSETC_AIO_II)
        reset_stepper_drivers();      // Workaround for Cheetah bug//猎豹虫的解决方法
      #endif
      if (!isMounted()) stat = 0;     // Not mounted?//没有上马？
    }
    else {
      #if PIN_EXISTS(SD_DETECT)
        release();                    // Card is released//信用卡已发行
      #endif
    }

    ui.media_changed(old_stat, stat); // Update the UI//更新用户界面

    if (stat) {
      TERN_(SDCARD_EEPROM_EMULATION, settings.first_load());
      if (old_stat == 2) {            // First mount?//第一架？
        DEBUG_ECHOLNPGM("First mount.");
        #if ENABLED(POWER_LOSS_RECOVERY)
          recovery.check();           // Check for PLR file. (If not there then call autofile_begin)//检查PLR文件。（如果没有，则调用autofile_begin）
        #elif DISABLED(NO_SD_AUTOSTART)
          autofile_begin();           // Look for auto0.g on the next loop//在下一个循环中查找auto0.g
        #endif
      }
    }
  }
  else
    DEBUG_ECHOLNPGM("SD: No UI Detected.");
}

/**
 * "Release" the media by clearing the 'mounted' flag.
 * Used by M22, "Release Media", manage_media.
 */
void CardReader::release() {
  // Card removed while printing? Abort!//打印时是否移除卡片？中止
  if (IS_SD_PRINTING())
    abortFilePrintSoon();
  else
    endFilePrintNow();

  flag.mounted = false;
  flag.workDirIsRoot = true;
  #if ALL(SDCARD_SORT_ALPHA, SDSORT_USES_RAM, SDSORT_CACHE_NAMES)
    nrFiles = 0;
  #endif
}

/**
 * Open a G-code file and set Marlin to start processing it.
 * Enqueues M23 and M24 commands to initiate a media print.
 */
void CardReader::openAndPrintFile(const char *name) {
  char cmd[4 + strlen(name) + 1 + 3 + 1]; // Room for "M23 ", filename, "\n", "M24", and null//容纳“M23”、文件名“\n”、“M24”和null的空间
  sprintf_P(cmd, M23_STR, name);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  strcat_P(cmd, PSTR("\nM24"));
  queue.inject(cmd);
}

/**
 * Start or resume a media print by setting the sdprinting flag.
 * The file browser pre-sort is also purged to free up memory,
 * since you cannot browse files during active printing.
 * Used by M24 and anywhere Start / Resume applies.
 */
void CardReader::startOrResumeFilePrinting() {
  if (isMounted()) {
    flag.sdprinting = true;
    flag.sdprintdone = false;
    TERN_(SD_RESORT, flush_presort());
  }
}

////
// Run tasks upon finishing or aborting a file print.//完成或中止文件打印后运行任务。
////
void CardReader::endFilePrintNow(TERN_(SD_RESORT, const bool re_sort/*=false*/)) {
  TERN_(ADVANCED_PAUSE_FEATURE, did_pause_print = 0);
  TERN_(DWIN_CREALITY_LCD, HMI_flag.print_finish = flag.sdprinting);
  flag.abort_sd_printing = false;
  if (isFileOpen()) file.close();
  TERN_(SD_RESORT, if (re_sort) presort());
}

void CardReader::abortFilePrintNow(TERN_(SD_RESORT, const bool re_sort/*=false*/)) {
  flag.sdprinting = flag.sdprintdone = false;
  endFilePrintNow(TERN_(SD_RESORT, re_sort));
}

void CardReader::openLogFile(const char * const path) {
  flag.logging = DISABLED(SDCARD_READONLY);
  IF_DISABLED(SDCARD_READONLY, openFileWrite(path));
}

////
// Get the root-relative DOS path of the selected file//获取所选文件的根相对DOS路径
////
void CardReader::getAbsFilenameInCWD(char *dst) {
  *dst++ = '/';
  uint8_t cnt = 1;

  auto appendAtom = [&](SdFile &file) {
    file.getDosName(dst);
    while (*dst && cnt < MAXPATHNAMELENGTH) { dst++; cnt++; }
    if (cnt < MAXPATHNAMELENGTH) { *dst = '/'; dst++; cnt++; }
  };

  LOOP_L_N(i, workDirDepth)                // Loop down to current work dir//向下循环到当前工作目录
    appendAtom(workDirParents[i]);

  if (cnt < MAXPATHNAMELENGTH - (FILENAME_LENGTH) - 1) {    // Leave room for filename and nul//为文件名和nul留出空间
    appendAtom(file);
    --dst;
  }
  *dst = '\0';
}

void openFailed(const char * const fname) {
  SERIAL_ECHOLNPAIR(STR_SD_OPEN_FILE_FAIL, fname, ".");
}

void announceOpen(const uint8_t doing, const char * const path) {
  if (doing) {
    PORT_REDIRECT(SerialMask::All);
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Now ");
    SERIAL_ECHOPGM_P(doing == 1 ? PSTR("doing") : PSTR("fresh"));
    SERIAL_ECHOLNPAIR(" file: ", path);
  }
}

////
// Open a file by DOS path for read//通过DOS路径打开文件进行读取
// The 'subcall_type' flag indicates...//“subcall_type”标志表示。。。
//   - 0 : Standard open from host or user interface.//-0：标准从主机或用户界面打开。
//   - 1 : (file open) Opening a new sub-procedure.//-1：（文件打开）打开新的子过程。
//   - 1 : (no file open) Opening a macro (M98).//-1：（无文件打开）打开宏（M98）。
//   - 2 : Resuming from a sub-procedure//-2：从子程序恢复
////
void CardReader::openFileRead(const char * const path, const uint8_t subcall_type/*=0*/) {
  if (!isMounted()) return;

  switch (subcall_type) {
    case 0:      // Starting a new print. "Now fresh file: ..."//开始新的打印。“现在刷新文件：…”
      announceOpen(2, path);
      TERN_(HAS_MEDIA_SUBCALLS, file_subcall_ctr = 0);
      break;

    #if HAS_MEDIA_SUBCALLS

      case 1:      // Starting a sub-procedure//启动子过程

        // With no file is open it's a simple macro. "Now doing file: ..."//没有打开任何文件，这是一个简单的宏。“正在处理文件：…”
        if (!isFileOpen()) { announceOpen(1, path); break; }

        // Too deep? The firmware has to bail.//太深了？固件必须退出。
        if (file_subcall_ctr > SD_PROCEDURE_DEPTH - 1) {
          SERIAL_ERROR_MSG("Exceeded max SUBROUTINE depth:", SD_PROCEDURE_DEPTH);
          kill(GET_TEXT(MSG_KILL_SUBCALL_OVERFLOW));
          return;
        }

        // Store current filename (based on workDirParents) and position//存储当前文件名（基于workDirParents）和位置
        getAbsFilenameInCWD(proc_filenames[file_subcall_ctr]);
        filespos[file_subcall_ctr] = sdpos;

        // For sub-procedures say 'SUBROUTINE CALL target: "..." parent: "..." pos12345'//对于子过程，说‘子程序调用目标：“…”父：“…”pos12345'
        SERIAL_ECHO_MSG("SUBROUTINE CALL target:\"", path, "\" parent:\"", proc_filenames[file_subcall_ctr], "\" pos", sdpos);
        file_subcall_ctr++;
        break;

      case 2:      // Resuming previous file after sub-procedure//在子过程之后恢复上一个文件
        SERIAL_ECHO_MSG("END SUBROUTINE");
        break;

    #endif
  }

  abortFilePrintNow();

  SdFile *diveDir;
  const char * const fname = diveToFile(true, diveDir, path);
  if (!fname) return;

  if (file.open(diveDir, fname, O_READ)) {
    filesize = file.fileSize();
    sdpos = 0;

    { // Don't remove this block, as the PORT_REDIRECT is a RAII//不要删除此块，因为端口_重定向是RAII
      PORT_REDIRECT(SerialMask::All);
      SERIAL_ECHOLNPAIR(STR_SD_FILE_OPENED, fname, STR_SD_SIZE, filesize);
      SERIAL_ECHOLNPGM(STR_SD_FILE_SELECTED);
    }

    selectFileByName(fname);
    ui.set_status(longFilename[0] ? longFilename : fname);
  }
  else
    openFailed(fname);
}

inline void echo_write_to_file(const char * const fname) {
  SERIAL_ECHOLNPAIR(STR_SD_WRITE_TO_FILE, fname);
}

////
// Open a file by DOS path for write//通过DOS路径打开文件进行写入
////
void CardReader::openFileWrite(const char * const path) {
  if (!isMounted()) return;

  announceOpen(2, path);
  TERN_(HAS_MEDIA_SUBCALLS, file_subcall_ctr = 0);

  abortFilePrintNow();

  SdFile *diveDir;
  const char * const fname = diveToFile(false, diveDir, path);
  if (!fname) return;

  #if ENABLED(SDCARD_READONLY)
    openFailed(fname);
  #else
    if (file.open(diveDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
      flag.saving = true;
      selectFileByName(fname);
      TERN_(EMERGENCY_PARSER, emergency_parser.disable());
      echo_write_to_file(fname);
      ui.set_status(fname);
    }
    else
      openFailed(fname);
  #endif
}

////
// Check if a file exists by absolute or workDir-relative path//检查文件是否按绝对路径或相对路径存在
// If the file exists, the long name can also be fetched.//如果文件存在，也可以提取长名称。
////
bool CardReader::fileExists(const char * const path) {
  if (!isMounted()) return false;

  DEBUG_ECHOLNPAIR("fileExists: ", path);

  // Dive to the file's directory and get the base name//跳转到文件目录并获取基本名称
  SdFile *diveDir = nullptr;
  const char * const fname = diveToFile(false, diveDir, path);
  if (!fname) return false;

  // Get the longname of the checked file//获取选中文件的longname
  //diveDir->rewind();//diveDir->倒带（）；
  //selectByName(*diveDir, fname);//按名称选择（*diveDir，fname）；
  //diveDir->close();//diveDir->close（）；

  // Try to open the file and return the result//尝试打开文件并返回结果
  SdFile tmpFile;
  const bool success = tmpFile.open(diveDir, fname, O_READ);
  if (success) tmpFile.close();
  return success;
}

////
// Delete a file by name in the working directory//在工作目录中按名称删除文件
////
void CardReader::removeFile(const char * const name) {
  if (!isMounted()) return;

  //abortFilePrintNow();//abortFilePrintNow（）；

  SdFile *itsDirPtr;
  const char * const fname = diveToFile(false, itsDirPtr, name);
  if (!fname) return;

  #if ENABLED(SDCARD_READONLY)
    SERIAL_ECHOLNPAIR("Deletion failed (read-only), File: ", fname, ".");
  #else
    if (file.remove(itsDirPtr, fname)) {
      SERIAL_ECHOLNPAIR("File deleted:", fname);
      sdpos = 0;
      TERN_(SDCARD_SORT_ALPHA, presort());
    }
    else
      SERIAL_ECHOLNPAIR("Deletion failed, File: ", fname, ".");
  #endif
}

void CardReader::report_status() {
  if (isPrinting()) {
    SERIAL_ECHOPAIR(STR_SD_PRINTING_BYTE, sdpos);
    SERIAL_CHAR('/');
    SERIAL_ECHOLN(filesize);
  }
  else
    SERIAL_ECHOLNPGM(STR_SD_NOT_PRINTING);
}

void CardReader::write_command(char * const buf) {
  char *begin = buf,
       *npos = nullptr,
       *end = buf + strlen(buf) - 1;

  file.writeError = false;
  if ((npos = strchr(buf, 'N'))) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);

  if (file.writeError) SERIAL_ERROR_MSG(STR_SD_ERR_WRITE_TO_FILE);
}

#if DISABLED(NO_SD_AUTOSTART)
  /**
   * Run all the auto#.g files. Called:
   * - On boot after successful card init.
   * - From the LCD command to Run Auto Files
   */
  void CardReader::autofile_begin() {
    autofile_index = 1;
    (void)autofile_check();
  }

  /**
   * Run the next auto#.g file. Called:
   *   - On boot after successful card init
   *   - After finishing the previous auto#.g file
   *   - From the LCD command to begin the auto#.g files
   *
   * Return 'true' if an auto file was started
   */
  bool CardReader::autofile_check() {
    if (!autofile_index) return false;

    if (!isMounted())
      mount();
    else if (ENABLED(SDCARD_EEPROM_EMULATION))
      settings.first_load();

    // Don't run auto#.g when a PLR file exists//当PLR文件存在时，不要运行auto#.g
    if (isMounted() && TERN1(POWER_LOSS_RECOVERY, !recovery.valid())) {
      char autoname[10];
      sprintf_P(autoname, PSTR("/auto%c.g"), '0' + autofile_index - 1);
      if (fileExists(autoname)) {
        cdroot();
        openAndPrintFile(autoname);
        autofile_index++;
        return true;
      }
    }
    autofile_cancel();
    return false;
  }
#endif

void CardReader::closefile(const bool store_location/*=false*/) {
  file.sync();
  file.close();
  flag.saving = flag.logging = false;
  sdpos = 0;
  TERN_(EMERGENCY_PARSER, emergency_parser.enable());

  if (store_location) {
    //future: store printer state, filename and position for continuing a stopped print//未来：存储打印机状态、文件名和位置，以便继续停止打印
    // so one can unplug the printer and continue printing the next day.//这样就可以拔掉打印机的插头，第二天继续打印。
  }
}

////
// Get info for a file in the working directory by index//按索引获取工作目录中文件的信息
////
void CardReader::selectFileByIndex(const uint16_t nr) {
  #if ENABLED(SDSORT_CACHE_NAMES)
    if (nr < sort_count) {
      strcpy(filename, sortshort[nr]);
      strcpy(longFilename, sortnames[nr]);
      flag.filenameIsDir = IS_DIR(nr);
      return;
    }
  #endif
  workDir.rewind();
  selectByIndex(workDir, nr);
}

////
// Get info for a file in the working directory by DOS name//按DOS名称获取工作目录中文件的信息
////
void CardReader::selectFileByName(const char * const match) {
  #if ENABLED(SDSORT_CACHE_NAMES)
    for (uint16_t nr = 0; nr < sort_count; nr++)
      if (strcasecmp(match, sortshort[nr]) == 0) {
        strcpy(filename, sortshort[nr]);
        strcpy(longFilename, sortnames[nr]);
        flag.filenameIsDir = IS_DIR(nr);
        return;
      }
  #endif
  workDir.rewind();
  selectByName(workDir, match);
}

uint16_t CardReader::countFilesInWorkDir() {
  workDir.rewind();
  return countItems(workDir);
}

/**
 * Dive to the given DOS 8.3 file path, with optional echo of the dive paths.
 *
 * On entry:
 *  - The workDir points to the last-set navigation target by cd, cdup, cdroot, or diveToFile(true, ...)
 *
 * On exit:
 *  - Your curDir pointer contains an SdFile reference to the file's directory.
 *  - If update_cwd was 'true' the workDir now points to the file's directory.
 *
 * Returns a pointer to the last segment (filename) of the given DOS 8.3 path.
 * On exit, inDirPtr contains an SdFile reference to the file's directory.
 *
 * A nullptr result indicates an unrecoverable error.
 *
 * NOTE: End the path with a slash to dive to a folder. In this case the
 *       returned filename will be blank (points to the end of the path).
 */
const char* CardReader::diveToFile(const bool update_cwd, SdFile* &inDirPtr, const char * const path, const bool echo/*=false*/) {
  DEBUG_SECTION(est, "diveToFile", true);

  // Track both parent and subfolder//跟踪父文件夹和子文件夹
  static SdFile newDir1, newDir2;
  SdFile *sub = &newDir1, *startDirPtr;

  // Parsing the path string//解析路径字符串
  const char *atom_ptr = path;

  DEBUG_ECHOLNPAIR(" path = '", path, "'");

  if (path[0] == '/') {               // Starting at the root directory?//从根目录开始？
    inDirPtr = &root;
    atom_ptr++;
    DEBUG_ECHOLNPAIR(" CWD to root: ", hex_address((void*)inDirPtr));
    if (update_cwd) workDirDepth = 0; // The cwd can be updated for the benefit of sub-programs//cwd可以为子程序的利益而更新
  }
  else
    inDirPtr = &workDir;              // Dive from workDir (as set by the UI)//从workDir跳转（由UI设置）

  startDirPtr = inDirPtr;

  DEBUG_ECHOLNPAIR(" startDirPtr = ", hex_address((void*)startDirPtr));

  while (atom_ptr) {
    // Find next subdirectory delimiter//查找下一个子目录分隔符
    char * const name_end = strchr(atom_ptr, '/');

    // Last atom in the path? Item found.//路径上的最后一个原子？找到的项目。
    if (name_end <= atom_ptr) break;

    // Isolate the next subitem name//隔离下一个子项名称
    const uint8_t len = name_end - atom_ptr;
    char dosSubdirname[len + 1];
    strncpy(dosSubdirname, atom_ptr, len);
    dosSubdirname[len] = 0;

    if (echo) SERIAL_ECHOLN(dosSubdirname);

    DEBUG_ECHOLNPAIR(" sub = ", hex_address((void*)sub));

    // Open inDirPtr (closing first)//打开inDirPtr（先关闭）
    sub->close();
    if (!sub->open(inDirPtr, dosSubdirname, O_READ)) {
      openFailed(dosSubdirname);
      atom_ptr = nullptr;
      break;
    }

    // Close inDirPtr if not at starting-point//如果不在起始点，则关闭inDirPtr
    if (inDirPtr != startDirPtr) {
      DEBUG_ECHOLNPAIR(" closing inDirPtr: ", hex_address((void*)inDirPtr));
      inDirPtr->close();
    }

    // inDirPtr now subDir//inDirPtr now子目录
    inDirPtr = sub;
    DEBUG_ECHOLNPAIR(" inDirPtr = sub: ", hex_address((void*)inDirPtr));

    // Update workDirParents and workDirDepth//更新workDirParents和workDirDepth
    if (update_cwd) {
      DEBUG_ECHOLNPAIR(" update_cwd");
      if (workDirDepth < MAX_DIR_DEPTH)
        workDirParents[workDirDepth++] = *inDirPtr;
    }

    // Point sub at the other scratch object//将sub指向另一个划痕对象
    sub = (inDirPtr != &newDir1) ? &newDir1 : &newDir2;
    DEBUG_ECHOLNPAIR(" swapping sub = ", hex_address((void*)sub));

    // Next path atom address//下一路径原子地址
    atom_ptr = name_end + 1;
  }

  if (update_cwd) {
    workDir = *inDirPtr;
    DEBUG_ECHOLNPAIR(" final workDir = ", hex_address((void*)inDirPtr));
    flag.workDirIsRoot = (workDirDepth == 0);
    TERN_(SDCARD_SORT_ALPHA, presort());
  }

  DEBUG_ECHOLNPAIR(" returning string ", atom_ptr ?: "nullptr");
  return atom_ptr;
}

void CardReader::cd(const char * relpath) {
  SdFile newDir, *parent = &getWorkDir();

  if (newDir.open(parent, relpath, O_READ)) {
    workDir = newDir;
    flag.workDirIsRoot = false;
    if (workDirDepth < MAX_DIR_DEPTH)
      workDirParents[workDirDepth++] = workDir;
    TERN_(SDCARD_SORT_ALPHA, presort());
  }
  else
    SERIAL_ECHO_MSG(STR_SD_CANT_ENTER_SUBDIR, relpath);
}

int8_t CardReader::cdup() {
  if (workDirDepth > 0) {                                               // At least 1 dir has been saved//至少保存了1个目录
    workDir = --workDirDepth ? workDirParents[workDirDepth - 1] : root; // Use parent, or root if none//使用父级，如果没有，则使用根级
    TERN_(SDCARD_SORT_ALPHA, presort());
  }
  if (!workDirDepth) flag.workDirIsRoot = true;
  return workDirDepth;
}

void CardReader::cdroot() {
  workDir = root;
  flag.workDirIsRoot = true;
  TERN_(SDCARD_SORT_ALPHA, presort());
}

#if ENABLED(SDCARD_SORT_ALPHA)

  /**
   * Get the name of a file in the working directory by sort-index
   */
  void CardReader::getfilename_sorted(const uint16_t nr) {
    selectFileByIndex(TERN1(SDSORT_GCODE, sort_alpha) && (nr < sort_count)
      ? sort_order[nr] : nr);
  }

  #if ENABLED(SDSORT_USES_RAM)
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      // Use dynamic method to copy long filename//使用动态方法复制长文件名
      #define SET_SORTNAME(I) (sortnames[I] = strdup(longest_filename()))
      #if ENABLED(SDSORT_CACHE_NAMES)
        // When caching also store the short name, since//缓存时还存储短名称，因为
        // we're replacing the selectFileByIndex() behavior.//我们正在替换selectFileByIndex（）行为。
        #define SET_SORTSHORT(I) (sortshort[I] = strdup(filename))
      #else
        #define SET_SORTSHORT(I) NOOP
      #endif
    #else
      // Copy filenames into the static array//将文件名复制到静态数组中
      #define _SET_SORTNAME(I) strncpy(sortnames[I], longest_filename(), SORTED_LONGNAME_MAXLEN)
      #if SORTED_LONGNAME_MAXLEN == LONG_FILENAME_LENGTH
        // Short name sorting always use LONG_FILENAME_LENGTH with no trailing nul//短名称排序始终使用长\u文件名\u长度，不带尾随nul
        #define SET_SORTNAME(I) _SET_SORTNAME(I)
      #else
        // Copy multiple name blocks. Add a nul for the longest case.//复制多个名称块。为最长的案例添加nul。
        #define SET_SORTNAME(I) do{ _SET_SORTNAME(I); sortnames[I][SORTED_LONGNAME_MAXLEN] = '\0'; }while(0)
      #endif
      #if ENABLED(SDSORT_CACHE_NAMES)
        #define SET_SORTSHORT(I) strcpy(sortshort[I], filename)
      #else
        #define SET_SORTSHORT(I) NOOP
      #endif
    #endif
  #endif

  /**
   * Read all the files and produce a sort key
   *
   * We can do this in 3 ways...
   *  - Minimal RAM: Read two filenames at a time sorting along...
   *  - Some RAM: Buffer the directory just for this sort
   *  - Most RAM: Buffer the directory and return filenames from RAM
   */
  void CardReader::presort() {

    // Throw away old sort index//扔掉旧的排序索引
    flush_presort();

    // Sorting may be turned off//可能会关闭排序
    if (TERN0(SDSORT_GCODE, !sort_alpha)) return;

    // If there are files, sort up to the limit//如果有文件，请按限制进行排序
    uint16_t fileCnt = countFilesInWorkDir();
    if (fileCnt > 0) {

      // Never sort more than the max allowed//排序不得超过允许的最大值
      // If you use folders to organize, 20 may be enough//如果你用文件夹来组织，20个就足够了
      NOMORE(fileCnt, uint16_t(SDSORT_LIMIT));

      // Sort order is always needed. May be static or dynamic.//总是需要排序顺序。可以是静态的，也可以是动态的。
      TERN_(SDSORT_DYNAMIC_RAM, sort_order = new uint8_t[fileCnt]);

      // Use RAM to store the entire directory during pre-sort.//在预排序期间，使用RAM存储整个目录。
      // SDSORT_LIMIT should be set to prevent over-allocation.//应设置SDU限制以防止过度分配。
      #if ENABLED(SDSORT_USES_RAM)

        // If using dynamic ram for names, allocate on the heap.//如果使用动态ram作为名称，请在堆上进行分配。
        #if ENABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortshort = new char*[fileCnt];
            sortnames = new char*[fileCnt];
          #endif
        #elif ENABLED(SDSORT_USES_STACK)
          char sortnames[fileCnt][SORTED_LONGNAME_STORAGE];
        #endif

        // Folder sorting needs 1 bit per entry for flags.//文件夹排序每个条目需要1位标志。
        #if HAS_FOLDER_SORTING
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            isDir = new uint8_t[(fileCnt + 7) >> 3];
          #elif ENABLED(SDSORT_USES_STACK)
            uint8_t isDir[(fileCnt + 7) >> 3];
          #endif
        #endif

      #else // !SDSORT_USES_RAM// !SDSORT_使用_RAM

        // By default re-read the names from SD for every compare//默认情况下，每次比较都从SD重新读取名称
        // retaining only two filenames at a time. This is very//一次只保留两个文件名。这是非常重要的
        // slow but is safest and uses minimal RAM.//速度慢但最安全，使用的内存最少。
        char name1[LONG_FILENAME_LENGTH];

      #endif

      if (fileCnt > 1) {

        // Init sort order.//初始化排序顺序。
        for (uint16_t i = 0; i < fileCnt; i++) {
          sort_order[i] = i;
          // If using RAM then read all filenames now.//如果使用RAM，那么现在读取所有文件名。
          #if ENABLED(SDSORT_USES_RAM)
            selectFileByIndex(i);
            SET_SORTNAME(i);
            SET_SORTSHORT(i);
            // char out[30];//字符输出[30]；
            // sprintf_P(out, PSTR("---- %i %s %s"), i, flag.filenameIsDir ? "D" : " ", sortnames[i]);//sprintf_P（out，PSTR（---%i%s%s）），i，flag.filenameidir？“D”：“”，sortnames[i]）；
            // SERIAL_ECHOLN(out);//序列号（输出）；
            #if HAS_FOLDER_SORTING
              const uint16_t bit = i & 0x07, ind = i >> 3;
              if (bit == 0) isDir[ind] = 0x00;
              if (flag.filenameIsDir) SBI(isDir[ind], bit);
            #endif
          #endif
        }

        // Bubble Sort//气泡排序
        for (uint16_t i = fileCnt; --i;) {
          bool didSwap = false;
          uint8_t o1 = sort_order[0];
          #if DISABLED(SDSORT_USES_RAM)
            selectFileByIndex(o1);              // Pre-fetch the first entry and save it//预取第一个条目并保存它
            strcpy(name1, longest_filename());  // so the loop only needs one fetch//因此，循环只需要一次获取
            #if ENABLED(HAS_FOLDER_SORTING)
              bool dir1 = flag.filenameIsDir;
            #endif
          #endif

          for (uint16_t j = 0; j < i; ++j) {
            const uint16_t o2 = sort_order[j + 1];

            // Compare names from the array or just the two buffered names//比较数组中的名称或仅比较两个缓冲名称
            #if ENABLED(SDSORT_USES_RAM)
              #define _SORT_CMP_NODIR() (strcasecmp(sortnames[o1], sortnames[o2]) > 0)
            #else
              #define _SORT_CMP_NODIR() (strcasecmp(name1, name2) > 0)
            #endif

            #if HAS_FOLDER_SORTING
              #if ENABLED(SDSORT_USES_RAM)
                // Folder sorting needs an index and bit to test for folder-ness.//文件夹排序需要一个索引和位来测试文件夹的可靠性。
                #define _SORT_CMP_DIR(fs) (IS_DIR(o1) == IS_DIR(o2) ? _SORT_CMP_NODIR() : IS_DIR(fs > 0 ? o1 : o2))
              #else
                #define _SORT_CMP_DIR(fs) ((dir1 == flag.filenameIsDir) ? _SORT_CMP_NODIR() : (fs > 0 ? dir1 : !dir1))
              #endif
            #endif

            // The most economical method reads names as-needed//最经济的方法是根据需要读取名称
            // throughout the loop. Slow if there are many.//在整个循环中。如果有很多，就慢一点。
            #if DISABLED(SDSORT_USES_RAM)
              selectFileByIndex(o2);
              const bool dir2 = flag.filenameIsDir;
              char * const name2 = longest_filename(); // use the string in-place//在适当的位置使用字符串
            #endif // !SDSORT_USES_RAM// !SDSORT_使用_RAM

            // Sort the current pair according to settings.//根据设置对当前对进行排序。
            if (
              #if HAS_FOLDER_SORTING
                #if ENABLED(SDSORT_GCODE)
                  sort_folders ? _SORT_CMP_DIR(sort_folders) : _SORT_CMP_NODIR()
                #else
                  _SORT_CMP_DIR(FOLDER_SORTING)
                #endif
              #else
                _SORT_CMP_NODIR()
              #endif
            ) {
              // Reorder the index, indicate that sorting happened//重新排序索引，指示已进行排序
              // Note that the next o1 will be the current o1. No new fetch needed.//请注意，下一个o1将是当前o1。不需要新的抓取。
              sort_order[j] = o2;
              sort_order[j + 1] = o1;
              didSwap = true;
            }
            else {
              // The next o1 is the current o2. No new fetch needed.//下一个o1是当前的o2。不需要新的抓取。
              o1 = o2;
              #if DISABLED(SDSORT_USES_RAM)
                TERN_(HAS_FOLDER_SORTING, dir1 = dir2);
                strcpy(name1, name2);
              #endif
            }
          }
          if (!didSwap) break;
        }
        // Using RAM but not keeping names around//使用RAM但不保留名称
        #if ENABLED(SDSORT_USES_RAM) && DISABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            for (uint16_t i = 0; i < fileCnt; ++i) free(sortnames[i]);
            TERN_(HAS_FOLDER_SORTING, free(isDir));
          #endif
        #endif
      }
      else {
        sort_order[0] = 0;
        #if BOTH(SDSORT_USES_RAM, SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortnames = new char*[1];
            sortshort = new char*[1];
            isDir = new uint8_t[1];
          #endif
          selectFileByIndex(0);
          SET_SORTNAME(0);
          SET_SORTSHORT(0);
          isDir[0] = flag.filenameIsDir;
        #endif
      }

      sort_count = fileCnt;
    }
  }

  void CardReader::flush_presort() {
    if (sort_count > 0) {
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        delete sort_order;
        #if ENABLED(SDSORT_CACHE_NAMES)
          LOOP_L_N(i, sort_count) {
            free(sortshort[i]); // strdup//标准
            free(sortnames[i]); // strdup//标准
          }
          delete sortshort;
          delete sortnames;
        #endif
      #endif
      sort_count = 0;
    }
  }

#endif // SDCARD_SORT_ALPHA//SDCARD_SORT_ALPHA

uint16_t CardReader::get_num_Files() {
  if (!isMounted()) return 0;
  return (
    #if ALL(SDCARD_SORT_ALPHA, SDSORT_USES_RAM, SDSORT_CACHE_NAMES)
      nrFiles // no need to access the SD card for filenames//无需访问SD卡获取文件名
    #else
      countFilesInWorkDir()
    #endif
  );
}

////
// Return from procedure or close out the Print Job//从程序返回或结束打印作业
////
void CardReader::fileHasFinished() {
  file.close();
  #if HAS_MEDIA_SUBCALLS
    if (file_subcall_ctr > 0) { // Resume calling file after closing procedure//关闭过程后继续调用文件
      file_subcall_ctr--;
      openFileRead(proc_filenames[file_subcall_ctr], 2); // 2 = Returning from sub-procedure//2=从子程序返回
      setIndex(filespos[file_subcall_ctr]);
      startOrResumeFilePrinting();
      return;
    }
  #endif

  endFilePrintNow(TERN_(SD_RESORT, true));

  flag.sdprintdone = true;        // Stop getting bytes from the SD card//停止从SD卡获取字节
  marlin_state = MF_SD_COMPLETE;  // Tell Marlin to enqueue M1001 soon//告诉马林尽快让M1001排队
}

#if ENABLED(AUTO_REPORT_SD_STATUS)
  AutoReporter<CardReader::AutoReportSD> CardReader::auto_reporter;
#endif

#if ENABLED(POWER_LOSS_RECOVERY)

  bool CardReader::jobRecoverFileExists() {
    const bool exists = recovery.file.open(&root, recovery.filename, O_READ);
    if (exists) recovery.file.close();
    return exists;
  }

  void CardReader::openJobRecoveryFile(const bool read) {
    if (!isMounted()) return;
    if (recovery.file.isOpen()) return;
    if (!recovery.file.open(&root, recovery.filename, read ? O_READ : O_CREAT | O_WRITE | O_TRUNC | O_SYNC))
      openFailed(recovery.filename);
    else if (!read)
      echo_write_to_file(recovery.filename);
  }

  // Removing the job recovery file currently requires closing//删除作业恢复文件当前需要关闭
  // the file being printed, so during SD printing the file should//正在打印的文件，因此在SD打印期间，文件应
  // be zeroed and written instead of deleted.//被归零并写入，而不是删除。
  void CardReader::removeJobRecoveryFile() {
    if (jobRecoverFileExists()) {
      recovery.init();
      removeFile(recovery.filename);
      #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
        SERIAL_ECHOPGM("Power-loss file delete");
        SERIAL_ECHOPGM_P(jobRecoverFileExists() ? PSTR(" failed.\n") : PSTR("d.\n"));
      #endif
    }
  }

#endif // POWER_LOSS_RECOVERY//功率损失恢复

#endif // SDSUPPORT//SDSUPPORT
