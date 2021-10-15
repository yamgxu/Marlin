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
 * lcd/extui/malyan/malyan_extui.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(MALYAN_LCD)

#include "../ui_api.h"
#include "malyan.h"

//#include "../../marlinui.h"//#包括“../../marlinui.h”
//#include "../../../sd/cardreader.h"//#包括“../../sd/cardreader.h”
//#include "../../../module/temperature.h"//#包括“../../../module/temperature.h”
//#include "../../../module/stepper.h"//#包括“../../../module/stepper.h”
//#include "../../../module/motion.h"//#包括“../../../module/motion.h”
//#include "../../../libs/duration_t.h"//#包括“../../../libs/duration\u t.h”
//#include "../../../module/printcounter.h"//#包括“../../../module/printcounter.h”
//#include "../../../gcode/queue.h"//#包括“../../../gcode/queue.h”

namespace ExtUI {
  void onStartup() {
    /**
     * The Malyan LCD actually runs as a separate MCU on Serial 1.
     * This code's job is to siphon the weird curly-brace commands from
     * it and translate into ExtUI operations where possible.
     */
    inbound_count = 0;

    #ifndef LCD_BAUDRATE
      #define LCD_BAUDRATE 500000
    #endif
    LCD_SERIAL.begin(LCD_BAUDRATE);

    // Signal init//信号初始化
    write_to_lcd_P(PSTR("{SYS:STARTED}\r\n"));

    // send a version that says "unsupported"//发送一个显示“不支持”的版本
    write_to_lcd_P(PSTR("{VER:99}\r\n"));

    // No idea why it does this twice.//不知道为什么它会这样做两次。
    write_to_lcd_P(PSTR("{SYS:STARTED}\r\n"));
    update_usb_status(true);
  }

  void onIdle() {
    /**
     * - from printer on startup:
     * {SYS:STARTED}{VER:29}{SYS:STARTED}{R:UD}
     */

    // First report USB status.//首先报告USB状态。
    update_usb_status(false);

    // now drain commands...//现在排水命令。。。
    while (LCD_SERIAL.available())
      parse_lcd_byte((byte)LCD_SERIAL.read());

    #if ENABLED(SDSUPPORT)
      // The way last printing status works is simple://上次打印状态的工作方式很简单：
      // The UI needs to see at least one TQ which is not 100%//UI需要至少看到一个不是100%的TQ
      // and then when the print is complete, one which is.//然后当打印完成时，一个是。
      static uint8_t last_percent_done = 100;

      // If there was a print in progress, we need to emit the final//如果有打印正在进行，我们需要发出最终结果
      // print status as {TQ:100}. Reset last percent done so a new print will//打印状态为{TQ:100}。重置完成的最后百分比，以便重新打印
      // issue a percent of 0.//发出0的百分比。
      const uint8_t percent_done = (ExtUI::isPrinting() || ExtUI::isPrintingFromMediaPaused()) ? ExtUI::getProgress_percent() : last_printing_status ? 100 : 0;
      if (percent_done != last_percent_done) {
        char message_buffer[16];
        sprintf_P(message_buffer, PSTR("{TQ:%03i}"), percent_done);
        write_to_lcd(message_buffer);
        last_percent_done = percent_done;
        last_printing_status = ExtUI::isPrinting();
      }
    #endif
  }

  void onPrinterKilled(PGM_P const error, PGM_P const component) {
    set_lcd_error_P(error, component);
  }

  #if HAS_PID_HEATING

    void onPidTuning(const result_t rst) {
      // Called for temperature PID tuning result//调用温度PID调整结果
      //SERIAL_ECHOLNPAIR("OnPidTuning:", rst);//串行回波对（“onpidtunning:”，rst）；
      switch (rst) {
        case PID_BAD_EXTRUDER_NUM:
          set_lcd_error_P(GET_TEXT(MSG_PID_BAD_EXTRUDER_NUM));
          break;
        case PID_TEMP_TOO_HIGH:
          set_lcd_error_P(GET_TEXT(MSG_PID_TEMP_TOO_HIGH));
          break;
        case PID_TUNING_TIMEOUT:
          set_lcd_error_P(GET_TEXT(MSG_PID_TIMEOUT));
          break;
        case PID_DONE:
          set_lcd_error_P(GET_TEXT(MSG_PID_AUTOTUNE_DONE));
          break;
      }
    }

  #endif

  void onPrintTimerStarted() { write_to_lcd_P(PSTR("{SYS:BUILD}")); }
  void onPrintTimerPaused() {}
  void onPrintTimerStopped() { write_to_lcd_P(PSTR("{TQ:100}")); }

  // Not needed for Malyan LCD//Malyan LCD不需要
  void onStatusChanged(const char * const) {}
  void onMediaInserted() {}
  void onMediaError() {}
  void onMediaRemoved() {}
  void onPlayTone(const uint16_t, const uint16_t) {}
  void onFilamentRunout(const extruder_t extruder) {}
  void onUserConfirmRequired(const char * const) {}
  void onHomingStart() {}
  void onHomingComplete() {}
  void onPrintFinished() {}
  void onFactoryReset() {}
  void onStoreSettings(char*) {}
  void onLoadSettings(const char*) {}
  void onPostprocessSettings() {}
  void onConfigurationStoreWritten(bool) {}
  void onConfigurationStoreRead(bool) {}

  #if HAS_MESH
    void onMeshLevelingStart() {}
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {}
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const ExtUI::probe_state_t state) {}
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void onPowerLossResume() {}
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled()  {}
}

#endif // MALYAN_LCD//MALYAN_液晶显示器
