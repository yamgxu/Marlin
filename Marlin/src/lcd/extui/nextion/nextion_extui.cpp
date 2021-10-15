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

/**
 * lcd/extui/nextion_lcd.cpp
 *
 * Nextion TFT support for Marlin
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(NEXTION_TFT)

#include "../ui_api.h"
#include "nextion_tft.h"

namespace ExtUI {

  void onStartup()                                   { nextion.Startup();  }
  void onIdle()                                      { nextion.IdleLoop(); }
  void onPrinterKilled(PGM_P const error, PGM_P const component) { nextion.PrinterKilled(error,component); }
  void onMediaInserted() {}
  void onMediaError()    {}
  void onMediaRemoved()  {}
  void onPlayTone(const uint16_t frequency, const uint16_t duration) {}
  void onPrintTimerStarted() {}
  void onPrintTimerPaused()  {}
  void onPrintTimerStopped() {}
  void onFilamentRunout(const extruder_t)            {}
  void onUserConfirmRequired(const char * const msg) { nextion.ConfirmationRequest(msg); }
  void onStatusChanged(const char * const msg)       { nextion.StatusChange(msg);        }

  void onHomingStart()    {}
  void onHomingComplete() {}
  void onPrintFinished()                             { nextion.PrintFinished(); }

  void onFactoryReset()   {}

  void onStoreSettings(char *buff) {
    // Called when saving to EEPROM (i.e. M500). If the ExtUI needs//保存到EEPROM（即M500）时调用。如果ExtUI需要
    // permanent data to be stored, it can write up to eeprom_data_size bytes//永久性存储数据，可写入eeprom_数据_大小字节
    // into buff.//变成浅黄色。

    // Example://例如：
    //  static_assert(sizeof(myDataStruct) <= ExtUI::eeprom_data_size);//静态断言（sizeof（myDataStruct）<=ExtUI:：eeprom\u数据大小）；
    //  memcpy(buff, &myDataStruct, sizeof(myDataStruct));//memcpy（buff和myDataStruct，sizeof（myDataStruct））；
  }

  void onLoadSettings(const char *buff) {
    // Called while loading settings from EEPROM. If the ExtUI//从EEPROM加载设置时调用。如果ExtUI
    // needs to retrieve data, it should copy up to eeprom_data_size bytes//需要检索数据时，它应最多复制eeprom_数据_大小字节
    // from buff//发福

    // Example://例如：
    //  static_assert(sizeof(myDataStruct) <= ExtUI::eeprom_data_size);//静态断言（sizeof（myDataStruct）<=ExtUI:：eeprom\u数据大小）；
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

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const float zval) {
      // Called when any mesh points are updated//更新任何网格点时调用
    }

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const ExtUI::probe_state_t state) {
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
      nextion.PanelInfo(37);
    }
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled()  {}
}

#endif // NEXTION_TFT//NEXTION_TFT
