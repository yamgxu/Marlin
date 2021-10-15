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

#if ENABLED(SDSUPPORT)

extern const char M23_STR[], M24_STR[];

#if BOTH(SDCARD_SORT_ALPHA, SDSORT_DYNAMIC_RAM)
  #define SD_RESORT 1
#endif

#if ENABLED(SDCARD_RATHERRECENTFIRST) && DISABLED(SDCARD_SORT_ALPHA)
  #define SD_ORDER(N,C) ((C) - 1 - (N))
#else
  #define SD_ORDER(N,C) N
#endif

#define MAX_DIR_DEPTH     10       // Maximum folder depth//最大文件夹深度
#define MAXDIRNAMELENGTH   8       // DOS folder name size//DOS文件夹名称大小
#define MAXPATHNAMELENGTH  (1 + (MAXDIRNAMELENGTH + 1) * (MAX_DIR_DEPTH) + 1 + FILENAME_LENGTH) // "/" + N * ("ADIRNAME/") + "filename.ext"//“/”+N*（“ADIRNAME/”）+“filename.ext”

#include "SdFile.h"
#include "disk_io_driver.h"

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #include "usb_flashdrive/Sd2Card_FlashDrive.h"
#endif

#if NEED_SD2CARD_SDIO
  #include "Sd2Card_sdio.h"
#elif NEED_SD2CARD_SPI
  #include "Sd2Card.h"
#endif

#if ENABLED(MULTI_VOLUME)
  #define SV_SD_ONBOARD      1
  #define SV_USB_FLASH_DRIVE 2
  #define _VOLUME_ID(N) _CAT(SV_, N)
  #define SHARED_VOLUME_IS(N) (DEFAULT_SHARED_VOLUME == _VOLUME_ID(N))
  #if !SHARED_VOLUME_IS(SD_ONBOARD) && !SHARED_VOLUME_IS(USB_FLASH_DRIVE)
    #error "DEFAULT_SHARED_VOLUME must be either SD_ONBOARD or USB_FLASH_DRIVE."
  #endif
#else
  #define SHARED_VOLUME_IS(...) 0
#endif

typedef struct {
  bool saving:1,
       logging:1,
       sdprinting:1,
       sdprintdone:1,
       mounted:1,
       filenameIsDir:1,
       workDirIsRoot:1,
       abort_sd_printing:1
       #if ENABLED(BINARY_FILE_TRANSFER)
         , binary_mode:1
       #endif
    ;
} card_flags_t;

#if ENABLED(AUTO_REPORT_SD_STATUS)
  #include "../libs/autoreport.h"
#endif

class CardReader {
public:
  static card_flags_t flag;                         // Flags (above)//旗帜（上图）
  static char filename[FILENAME_LENGTH],            // DOS 8.3 filename of the selected item//DOS 8.3所选项目的文件名
              longFilename[LONG_FILENAME_LENGTH];   // Long name of the selected item//所选项目的长名称

  // Fast! binary file transfer//快！二进制文件传输
  #if ENABLED(BINARY_FILE_TRANSFER)
    #if HAS_MULTI_SERIAL
      static serial_index_t transfer_port_index;
    #else
      static constexpr serial_index_t transfer_port_index = 0;
    #endif
  #endif

  // // // Methods // // ///////方法/////

  CardReader();

  static void changeMedia(DiskIODriver *_driver) { driver = _driver; }

  static SdFile getroot() { return root; }

  static void mount();
  static void release();
  static inline bool isMounted() { return flag.mounted; }

  // Handle media insert/remove//处理介质插入/取出
  static void manage_media();

  // SD Card Logging//SD卡记录
  static void openLogFile(const char * const path);
  static void write_command(char * const buf);

  #if DISABLED(NO_SD_AUTOSTART)     // Auto-Start auto#.g file handling//自动启动自动#.g文件处理
    static uint8_t autofile_index;  // Next auto#.g index to run, plus one. Ignored by autofile_check when zero.//下一个要运行的自动#.g索引，加上一个。当为零时，自动文件检查忽略。
    static void autofile_begin();   // Begin check. Called automatically after boot-up.//开始检查。启动后自动调用。
    static bool autofile_check();   // Check for the next auto-start file and run it.//检查下一个自动启动文件并运行它。
    static inline void autofile_cancel() { autofile_index = 0; }
  #endif

  // Basic file ops//基本文件操作
  static void openFileRead(const char * const path, const uint8_t subcall=0);
  static void openFileWrite(const char * const path);
  static void closefile(const bool store_location=false);
  static bool fileExists(const char * const name);
  static void removeFile(const char * const name);

  static inline char* longest_filename() { return longFilename[0] ? longFilename : filename; }
  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    static void printLongPath(char * const path);   // Used by M33//由M33使用
  #endif

  // Working Directory for SD card menu//SD卡菜单的工作目录
  static void cdroot();
  static void cd(const char *relpath);
  static int8_t cdup();
  static uint16_t countFilesInWorkDir();
  static uint16_t get_num_Files();

  // Select a file//选择一个文件
  static void selectFileByIndex(const uint16_t nr);
  static void selectFileByName(const char * const match);  // (working directory only)//（仅适用于工作目录）

  // Print job//打印作业
  static void report_status();
  static void getAbsFilenameInCWD(char *dst);
  static void printSelectedFilename();
  static void openAndPrintFile(const char *name);   // (working directory or full path)//（工作目录或完整路径）
  static void startOrResumeFilePrinting();
  static void endFilePrintNow(TERN_(SD_RESORT, const bool re_sort=false));
  static void abortFilePrintNow(TERN_(SD_RESORT, const bool re_sort=false));
  static void fileHasFinished();
  static inline void abortFilePrintSoon() { flag.abort_sd_printing = true; }
  static inline void pauseSDPrint()       { flag.sdprinting = false; }
  static inline bool isPrinting()         { return flag.sdprinting; }
  static inline bool isPaused()           { return isFileOpen() && !isPrinting(); }
  #if HAS_PRINT_PROGRESS_PERMYRIAD
    static inline uint16_t permyriadDone() {
      if (flag.sdprintdone) return 10000;
      if (isFileOpen() && filesize) return sdpos / ((filesize + 9999) / 10000);
      return 0;
    }
  #endif
  static inline uint8_t percentDone() {
    if (flag.sdprintdone) return 100;
    if (isFileOpen() && filesize) return sdpos / ((filesize + 99) / 100);
    return 0;
  }

  /**
   * Dive down to a relative or absolute path.
   * Relative paths apply to the workDir.
   *
   * update_cwd: Pass 'true' to update the workDir on success.
   *   inDirPtr: On exit your pointer points to the target SdFile.
   *             A nullptr indicates failure.
   *       path: Start with '/' for abs path. End with '/' to get a folder ref.
   *       echo: Set 'true' to print the path throughout the loop.
   */
  static const char* diveToFile(const bool update_cwd, SdFile* &inDirPtr, const char * const path, const bool echo=false);

  #if ENABLED(SDCARD_SORT_ALPHA)
    static void presort();
    static void getfilename_sorted(const uint16_t nr);
    #if ENABLED(SDSORT_GCODE)
      FORCE_INLINE static void setSortOn(bool b)        { sort_alpha   = b; presort(); }
      FORCE_INLINE static void setSortFolders(int i)    { sort_folders = i; presort(); }
      //FORCE_INLINE static void setSortReverse(bool b) { sort_reverse = b; }//FORCE_INLINE static void setSortReverse（bool b）{sort_reverse=b；}
    #endif
  #else
    FORCE_INLINE static void getfilename_sorted(const uint16_t nr) { selectFileByIndex(nr); }
  #endif

  static void ls();

  #if ENABLED(POWER_LOSS_RECOVERY)
    static bool jobRecoverFileExists();
    static void openJobRecoveryFile(const bool read);
    static void removeJobRecoveryFile();
  #endif

  // Current Working Dir - Set by cd, cdup, cdroot, and diveToFile(true, ...)//当前工作目录-由cd、cdup、cdroot和diveToFile设置（true，…）
  static inline char* getWorkDirName()  { workDir.getDosName(filename); return filename; }
  static inline SdFile& getWorkDir()    { return workDir.isOpen() ? workDir : root; }

  // Print File stats//打印文件统计信息
  static inline uint32_t getFileSize()  { return filesize; }
  static inline uint32_t getIndex()     { return sdpos; }
  static inline bool isFileOpen()       { return isMounted() && file.isOpen(); }
  static inline bool eof()              { return getIndex() >= getFileSize(); }

  // File data operations//文件数据操作
  static inline int16_t get()                            { int16_t out = (int16_t)file.read(); sdpos = file.curPosition(); return out; }
  static inline int16_t read(void *buf, uint16_t nbyte)  { return file.isOpen() ? file.read(buf, nbyte) : -1; }
  static inline int16_t write(void *buf, uint16_t nbyte) { return file.isOpen() ? file.write(buf, nbyte) : -1; }
  static inline void setIndex(const uint32_t index)      { file.seekSet((sdpos = index)); }

  // TODO: rename to diskIODriver()//TODO:重命名为diskIODriver（）
  static DiskIODriver* diskIODriver() { return driver; }

  #if ENABLED(AUTO_REPORT_SD_STATUS)
    ////
    // SD Auto Reporting//SD自动报告
    ////
    struct AutoReportSD { static void report() { report_status(); } };
    static AutoReporter<AutoReportSD> auto_reporter;
  #endif

  #if SHARED_VOLUME_IS(USB_FLASH_DRIVE) || ENABLED(USB_FLASH_DRIVE_SUPPORT)
    #define HAS_USB_FLASH_DRIVE 1
    static DiskIODriver_USBFlash media_driver_usbFlash;
  #endif

  #if NEED_SD2CARD_SDIO || NEED_SD2CARD_SPI
    typedef TERN(NEED_SD2CARD_SDIO, DiskIODriver_SDIO, DiskIODriver_SPI_SD) sdcard_driver_t;
    static sdcard_driver_t media_driver_sdcard;
  #endif

private:
  ////
  // Working directory and parents//工作目录和父目录
  ////
  static SdFile root, workDir, workDirParents[MAX_DIR_DEPTH];
  static uint8_t workDirDepth;

  ////
  // Alphabetical file and folder sorting//按字母顺序排列的文件和文件夹排序
  ////
  #if ENABLED(SDCARD_SORT_ALPHA)
    static uint16_t sort_count;   // Count of sorted items in the current directory//当前目录中已排序项目的计数
    #if ENABLED(SDSORT_GCODE)
      static bool sort_alpha;     // Flag to enable / disable the feature//启用/禁用功能的标志
      static int sort_folders;    // Folder sorting before/none/after//文件夹排序前/无/后
      //static bool sort_reverse; // Flag to enable / disable reverse sorting//静态布尔排序_反向；//启用/禁用反向排序的标志
    #endif

    // By default the sort index is static//默认情况下，排序索引是静态的
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      static uint8_t *sort_order;
    #else
      static uint8_t sort_order[SDSORT_LIMIT];
    #endif

    #if BOTH(SDSORT_USES_RAM, SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)
      #define SORTED_LONGNAME_MAXLEN (SDSORT_CACHE_VFATS) * (FILENAME_LENGTH)
      #define SORTED_LONGNAME_STORAGE (SORTED_LONGNAME_MAXLEN + 1)
    #else
      #define SORTED_LONGNAME_MAXLEN LONG_FILENAME_LENGTH
      #define SORTED_LONGNAME_STORAGE SORTED_LONGNAME_MAXLEN
    #endif

    // Cache filenames to speed up SD menus.//缓存文件名以加速SD菜单。
    #if ENABLED(SDSORT_USES_RAM)

      // If using dynamic ram for names, allocate on the heap.//如果使用动态ram作为名称，请在堆上进行分配。
      #if ENABLED(SDSORT_CACHE_NAMES)
        static uint16_t nrFiles; // Cache the total count//缓存总计数
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          static char **sortshort, **sortnames;
        #else
          static char sortshort[SDSORT_LIMIT][FILENAME_LENGTH];
        #endif
      #endif

      #if (ENABLED(SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)) || NONE(SDSORT_CACHE_NAMES, SDSORT_USES_STACK)
        static char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_STORAGE];
      #endif

      // Folder sorting uses an isDir array when caching items.//文件夹排序在缓存项目时使用isDir数组。
      #if HAS_FOLDER_SORTING
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          static uint8_t *isDir;
        #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
          static uint8_t isDir[(SDSORT_LIMIT + 7) >> 3];
        #endif
      #endif

    #endif // SDSORT_USES_RAM//SDSORT_使用_RAM

  #endif // SDCARD_SORT_ALPHA//SDCARD_SORT_ALPHA

  static DiskIODriver *driver;
  static SdVolume volume;
  static SdFile file;

  static uint32_t filesize, // Total size of the current file, in bytes//当前文件的总大小（字节）
                  sdpos;    // Index most recently read (one behind file.getPos)//最近读取的索引（文件.getPos后面的一个）

  ////
  // Procedure calls to other files//对其他文件的过程调用
  ////
  #if HAS_MEDIA_SUBCALLS
    static uint8_t file_subcall_ctr;
    static uint32_t filespos[SD_PROCEDURE_DEPTH];
    static char proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  #endif

  ////
  // Directory items//目录项
  ////
  static bool is_dir_or_gcode(const dir_t &p);
  static int countItems(SdFile dir);
  static void selectByIndex(SdFile dir, const uint8_t index);
  static void selectByName(SdFile dir, const char * const match);
  static void printListing(SdFile parent, const char * const prepend=nullptr);

  #if ENABLED(SDCARD_SORT_ALPHA)
    static void flush_presort();
  #endif
};

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #define IS_SD_INSERTED() DiskIODriver_USBFlash::isInserted()
#elif PIN_EXISTS(SD_DETECT)
  #define IS_SD_INSERTED() (READ(SD_DETECT_PIN) == SD_DETECT_STATE)
#else
  // No card detect line? Assume the card is inserted.//没有卡检测线？假设卡已插入。
  #define IS_SD_INSERTED() true
#endif

#define IS_SD_PRINTING()  (card.flag.sdprinting && !card.flag.abort_sd_printing)
#define IS_SD_FETCHING()  (!card.flag.sdprintdone && IS_SD_PRINTING())
#define IS_SD_PAUSED()    card.isPaused()
#define IS_SD_FILE_OPEN() card.isFileOpen()

extern CardReader card;

#else // !SDSUPPORT// !SDSUPPORT

#define IS_SD_PRINTING()  false
#define IS_SD_FETCHING()  false
#define IS_SD_PAUSED()    false
#define IS_SD_FILE_OPEN() false

#define LONG_FILENAME_LENGTH 0

#endif // !SDSUPPORT// !SDSUPPORT
