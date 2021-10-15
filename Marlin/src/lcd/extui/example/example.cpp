/** translatione by yx */
/*********************
 * example.cpp *
 *********************/

/****************************************************************************
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

#include "../../../inc/MarlinConfigPre.h"

#if BOTH(EXTUI_EXAMPLE, EXTENSIBLE_UI)

#include "../ui_api.h"

// To implement a new UI, complete the functions below and//要实现新的UI，请完成以下功能并
// read or update Marlin's state using the methods in the//使用中的方法读取或更新Marlin的状态
// ExtUI methods in "../ui_api.h"//“./ui\u api.h”中的ExtUI方法
////
// Although it may be possible to access other state//尽管可以访问其他状态
// variables from Marlin, using the API here possibly//来自Marlin的变量，可能在这里使用API
// helps ensure future compatibility.//有助于确保将来的兼容性。

namespace ExtUI {
  void onStartup() {
    /* Initialize the display module here. The following
     * routines are available for access to the GPIO pins:
     *
     *   SET_OUTPUT(pin)
     *   SET_INPUT_PULLUP(pin)
     *   SET_INPUT(pin)
     *   WRITE(pin,value)
     *   READ(pin)
     */
  }
  void onIdle() {}
  void onPrinterKilled(PGM_P const error, PGM_P const component) {}
  void onMediaInserted() {}
  void onMediaError() {}
  void onMediaRemoved() {}
  void onPlayTone(const uint16_t frequency, const uint16_t duration) {}
  void onPrintTimerStarted() {}
  void onPrintTimerPaused() {}
  void onPrintTimerStopped() {}
  void onFilamentRunout(const extruder_t extruder) {}
  void onUserConfirmRequired(const char * const msg) {}
  void onStatusChanged(const char * const msg) {}

  void onHomingStart() {}
  void onHomingComplete() {}
  void onPrintFinished() {}

  void onFactoryReset() {}

  void onStoreSettings(char *buff) {
    // Called when saving to EEPROM (i.e. M500). If the ExtUI needs//保存到EEPROM（即M500）时调用。如果ExtUI需要
    // permanent data to be stored, it can write up to eeprom_data_size bytes//永久性存储数据，可写入eeprom_数据_大小字节
    // into buff.//变成浅黄色。

    // Example://例如：
    //  static_assert(sizeof(myDataStruct) <= eeprom_data_size);//静态断言（sizeof（myDataStruct）<=eeprom数据大小）；
    //  memcpy(buff, &myDataStruct, sizeof(myDataStruct));//memcpy（buff和myDataStruct，sizeof（myDataStruct））；
  }

  void onLoadSettings(const char *buff) {
    // Called while loading settings from EEPROM. If the ExtUI//从EEPROM加载设置时调用。如果ExtUI
    // needs to retrieve data, it should copy up to eeprom_data_size bytes//需要检索数据时，它应最多复制eeprom_数据_大小字节
    // from buff//发福

    // Example://例如：
    //  static_assert(sizeof(myDataStruct) <= eeprom_data_size);//静态断言（sizeof（myDataStruct）<=eeprom数据大小）；
    //  memcpy(&myDataStruct, buff, sizeof(myDataStruct));//memcpy（&myDataStruct，buff，sizeof（myDataStruct））；
  }

  void onPostprocessSettings() {
    // Called after loading or resetting stored settings//加载或重置存储设置后调用
  }

  void onConfigurationStoreWritten(bool success) {
    // Called after the entire EEPROM has been written,//在写入整个EEPROM后调用，
    // whether successful or not.//不管成功与否。
  }

  void onConfigurationStoreRead(bool success) {
    // Called after the entire EEPROM has been read,//读取整个EEPROM后调用，
    // whether successful or not.//不管成功与否。
  }

  #if HAS_MESH
    void onMeshLevelingStart() {}

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
      // Called when any mesh points are updated//更新任何网格点时调用
    }

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const probe_state_t state) {
      // Called to indicate a special condition//被调用以指示特殊情况
    }
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void onPowerLossResume() {
      // Called on resume from power-loss//因断电而恢复通话
    }
  #endif

  #if HAS_PID_HEATING
    void onPidTuning(const result_t rst) {
      // Called for temperature PID tuning result//调用温度PID调整结果
      switch (rst) {
        case PID_BAD_EXTRUDER_NUM: break;
        case PID_TEMP_TOO_HIGH:    break;
        case PID_TUNING_TIMEOUT:   break;
        case PID_DONE:             break;
      }
    }
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled()  {}
}

#endif // EXTUI_EXAMPLE && EXTENSIBLE_UI//EXTUI\u示例和可扩展\u UI
