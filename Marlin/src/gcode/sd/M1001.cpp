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

#include "../../inc/MarlinConfig.h"

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/printcounter.h"
#include "../../sd/cardreader.h"

#ifdef SD_FINISHED_RELEASECOMMAND
  #include "../queue.h"
#endif

#if EITHER(LCD_SET_PROGRESS_MANUALLY, SD_REPRINT_LAST_SELECTED_FILE)
  #include "../../lcd/marlinui.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../feature/powerloss.h"
#endif

#if HAS_LEDS_OFF_FLAG
  #include "../../MarlinCore.h" // for wait_for_user_response()//等待用户响应（）
  #include "../../feature/leds/printer_event_leds.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../feature/host_actions.h"
#endif

#ifndef PE_LEDS_COMPLETED_TIME
  #define PE_LEDS_COMPLETED_TIME (30*60)
#endif

/**
 * M1001: Execute actions for SD print completion
 */
void GcodeSuite::M1001() {
  planner.synchronize();

  // SD Printing is finished when the queue reaches M1001//当队列到达M1001时，SD打印完成
  card.flag.sdprinting = card.flag.sdprintdone = false;

  // If there's another auto#.g file to run...//如果要运行另一个auto#.g文件。。。
  if (TERN(NO_SD_AUTOSTART, false, card.autofile_check())) return;

  // Purge the recovery file...//清除恢复文件。。。
  TERN_(POWER_LOSS_RECOVERY, recovery.purge());

  // Report total print time//报告总打印时间
  const bool long_print = print_job_timer.duration() > 60;
  if (long_print) gcode.process_subcommands_now_P(PSTR("M31"));

  // Stop the print job timer//停止打印作业计时器
  gcode.process_subcommands_now_P(PSTR("M77"));

  // Set the progress bar "done" state//将进度条设置为“完成”状态
  TERN_(LCD_SET_PROGRESS_MANUALLY, ui.set_progress_done());

  // Announce SD file completion//宣布SD文件完成
  {
    PORT_REDIRECT(SerialMask::All);
    SERIAL_ECHOLNPGM(STR_FILE_PRINTED);
  }

  // Update the status LED color//更新状态LED颜色
  #if HAS_LEDS_OFF_FLAG
    if (long_print) {
      printerEventLEDs.onPrintCompleted();
      TERN_(EXTENSIBLE_UI, ExtUI::onUserConfirmRequired_P(GET_TEXT(MSG_PRINT_DONE)));
      TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_USER_CONTINUE, GET_TEXT(MSG_PRINT_DONE), CONTINUE_STR));
      wait_for_user_response(SEC_TO_MS(TERN(HAS_LCD_MENU, PE_LEDS_COMPLETED_TIME, 30)));
      printerEventLEDs.onResumeAfterWait();
    }
  #endif

  // Inject SD_FINISHED_RELEASECOMMAND, if any//注入SD_FINISHED_release命令（如果有）
  #ifdef SD_FINISHED_RELEASECOMMAND
    gcode.process_subcommands_now_P(PSTR(SD_FINISHED_RELEASECOMMAND));
  #endif

  TERN_(EXTENSIBLE_UI, ExtUI::onPrintFinished());

  // Re-select the last printed file in the UI//重新选择UI中最后一个打印的文件
  TERN_(SD_REPRINT_LAST_SELECTED_FILE, ui.reselect_last_file());
}

#endif // SDSUPPORT//SDSUPPORT
