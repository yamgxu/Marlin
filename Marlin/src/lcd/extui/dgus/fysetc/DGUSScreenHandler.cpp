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

#include "../../../../inc/MarlinConfigPre.h"

#if ENABLED(DGUS_LCD_UI_FYSETC)

#include "../DGUSScreenHandler.h"

#include "../../../../MarlinCore.h"
#include "../../../../gcode/queue.h"
#include "../../../../libs/duration_t.h"
#include "../../../../module/settings.h"
#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"
#include "../../../../module/printcounter.h"
#include "../../../../sd/cardreader.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../../feature/powerloss.h"
#endif

#if ENABLED(SDSUPPORT)

  void DGUSScreenHandler::DGUSLCD_SD_FileSelected(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t touched_nr = (int16_t)swap16(*(uint16_t*)val_ptr) + top_file;
    if (touched_nr > filelist.count()) return;
    if (!filelist.seek(touched_nr)) return;

    if (filelist.isDir()) {
      filelist.changeDir(filelist.filename());
      top_file = 0;
      ForceCompleteUpdate();
      return;
    }

    #if ENABLED(DGUS_PRINT_FILENAME)
      // Send print filename//发送打印文件名
      dgusdisplay.WriteVariable(VP_SD_Print_Filename, filelist.filename(), VP_SD_FileName_LEN, true);
    #endif

    // Setup Confirmation screen//设置确认屏幕
    file_to_print = touched_nr;

    HandleUserConfirmationPopUp(VP_SD_FileSelectConfirm, nullptr, PSTR("Print file"), filelist.filename(), PSTR("from SD Card?"), true, true, false, true);
  }

  void DGUSScreenHandler::DGUSLCD_SD_StartPrint(DGUS_VP_Variable &var, void *val_ptr) {
    if (!filelist.seek(file_to_print)) return;
    ExtUI::printFile(filelist.shortFilename());
    GotoScreen(DGUSLCD_SCREEN_SDPRINTMANIPULATION);
  }

  void DGUSScreenHandler::DGUSLCD_SD_ResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr) {

    if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.//当用户停留在此菜单中且打印机完成时，避免竞争条件。
    switch (swap16(*(uint16_t*)val_ptr)) {
      case 0: { // Resume//恢复
        if (ExtUI::isPrintingFromMediaPaused()) {
          ExtUI::resumePrint();
        }
      } break;

      case 1: // Pause//停顿

        GotoScreen(MKSLCD_SCREEN_PAUSE);
        if (!ExtUI::isPrintingFromMediaPaused()) {
          ExtUI::pausePrint();
          //ExtUI::mks_pausePrint();//ExtUI:：mks_pausePrint（）；
        }
        break;
      case 2: // Abort//流产
        HandleUserConfirmationPopUp(VP_SD_AbortPrintConfirmed, nullptr, PSTR("Abort printing"), filelist.filename(), PSTR("?"), true, true, false, true);
        break;
    }
  }

  void DGUSScreenHandler::DGUSLCD_SD_SendFilename(DGUS_VP_Variable& var) {
    uint16_t target_line = (var.VP - VP_SD_FileName0) / VP_SD_FileName_LEN;
    if (target_line > DGUS_SD_FILESPERSCREEN) return;
    char tmpfilename[VP_SD_FileName_LEN + 1] = "";
    var.memadr = (void*)tmpfilename;

    if (filelist.seek(top_file + target_line)) {
      snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s%c"), filelist.filename(), filelist.isDir() ? '/' : 0); // snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s"), filelist.filename());//snprintf_P（tmpfilename，VP_SD_FileName_LEN，PSTR（“%s”），filelist.FileName（））；
    }
    DGUSLCD_SendStringToDisplay(var);
  }

  void DGUSScreenHandler::SDCardInserted() {
    top_file = 0;
    filelist.refresh();
    auto cs = getCurrentScreen();
    if (cs == DGUSLCD_SCREEN_MAIN || cs == DGUSLCD_SCREEN_STATUS)
      GotoScreen(DGUSLCD_SCREEN_SDFILELIST);
  }

  void DGUSScreenHandler::SDCardRemoved() {
    if (current_screen == DGUSLCD_SCREEN_SDFILELIST
        || (current_screen == DGUSLCD_SCREEN_CONFIRM && (ConfirmVP == VP_SD_AbortPrintConfirmed || ConfirmVP == VP_SD_FileSelectConfirm))
        || current_screen == DGUSLCD_SCREEN_SDPRINTMANIPULATION
    ) GotoScreen(DGUSLCD_SCREEN_MAIN);
  }

#endif // SDSUPPORT//SDSUPPORT

void DGUSScreenHandler::ScreenChangeHook(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t *tmp = (uint8_t*)val_ptr;

  // The keycode in target is coded as <from-frame><to-frame>, so 0x0100A means//目标中的键码编码为<from frame><to frame>，因此0x0100A表示
  // from screen 1 (main) to 10 (temperature). DGUSLCD_SCREEN_POPUP is special,//从屏幕1（主）到10（温度）。DGUSLCD_屏幕_弹出窗口是特殊的，
  // meaning "return to previous screen"//意思是“返回上一屏幕”
  DGUSLCD_Screens target = (DGUSLCD_Screens)tmp[1];

  DEBUG_ECHOLNPAIR("\n DEBUG target", target);

  if (target == DGUSLCD_SCREEN_POPUP) {
    // Special handling for popup is to return to previous menu//弹出窗口的特殊处理是返回上一个菜单
    if (current_screen == DGUSLCD_SCREEN_POPUP && confirm_action_cb) confirm_action_cb();
    PopToOldScreen();
    return;
  }

  UpdateNewScreen(target);

  #ifdef DEBUG_DGUSLCD
    if (!DGUSLCD_FindScreenVPMapList(target)) DEBUG_ECHOLNPAIR("WARNING: No screen Mapping found for ", target);
  #endif
}

void DGUSScreenHandler::HandleManualMove(DGUS_VP_Variable &var, void *val_ptr) {
  DEBUG_ECHOLNPGM("HandleManualMove");

  int16_t movevalue = swap16(*(uint16_t*)val_ptr);
  #if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
    if (movevalue) {
      const uint16_t choice = *(uint16_t*)var.memadr;
      movevalue = movevalue < 0 ? -choice : choice;
    }
  #endif
  char axiscode;
  unsigned int speed = 1500; // FIXME: get default feedrate for manual moves, dont hardcode.//修正：获取手动移动的默认进给速度，不要硬编码。

  switch (var.VP) {
    default: return;

    case VP_MOVE_X:
      axiscode = 'X';
      if (!ExtUI::canMove(ExtUI::axis_t::X)) goto cannotmove;
      break;

    case VP_MOVE_Y:
      axiscode = 'Y';
      if (!ExtUI::canMove(ExtUI::axis_t::Y)) goto cannotmove;
      break;

    case VP_MOVE_Z:
      axiscode = 'Z';
      speed = 300; // default to 5mm/s//默认为5毫米/秒
      if (!ExtUI::canMove(ExtUI::axis_t::Z)) goto cannotmove;
      break;

    case VP_HOME_ALL: // only used for homing//仅用于归航
      axiscode  = '\0';
      movevalue = 0; // ignore value sent from display, this VP is _ONLY_ for homing.//忽略显示器发送的值，此VP仅用于归位。
      break;
  }

  if (!movevalue) {
    // homing//归巢
    DEBUG_ECHOPAIR(" homing ", AS_CHAR(axiscode));
    char buf[6] = "G28 X";
    buf[4] = axiscode;
    //DEBUG_ECHOPAIR(" ", buf);//调试回声对（“，buf）；
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓");//调试_ECHOLNPGM（“✓");
    ForceCompleteUpdate();
    return;
  }
  else {
    // movement//运动
    DEBUG_ECHOPAIR(" move ", AS_CHAR(axiscode));
    bool old_relative_mode = relative_mode;
    if (!relative_mode) {
      //DEBUG_ECHOPGM(" G91");//调试ECHOPGM（“G91”）；
      queue.enqueue_now_P(PSTR("G91"));
      //DEBUG_ECHOPGM(" ✓ ");//调试ECHOPGM（“✓ ");
    }
    char buf[32]; // G1 X9999.99 F12345//G1 X9999.99 F12345
    unsigned int backup_speed = MMS_TO_MMM(feedrate_mm_s);
    char sign[] = "\0";
    int16_t value = movevalue / 100;
    if (movevalue < 0) { value = -value; sign[0] = '-'; }
    int16_t fraction = ABS(movevalue) % 100;
    snprintf_P(buf, 32, PSTR("G0 %c%s%d.%02d F%d"), axiscode, sign, value, fraction, speed);
    //DEBUG_ECHOPAIR(" ", buf);//调试回声对（“，buf）；
    queue.enqueue_one_now(buf);
    //DEBUG_ECHOLNPGM(" ✓ ");//调试_ECHOLNPGM（“✓ ");
    if (backup_speed != speed) {
      snprintf_P(buf, 32, PSTR("G0 F%d"), backup_speed);
      queue.enqueue_one_now(buf);
      //DEBUG_ECHOPAIR(" ", buf);//调试回声对（“，buf）；
    }
    // while (!enqueue_and_echo_command(buf)) idle();//当（！enqueue_和_echo_命令（buf））空闲时（）；
    //DEBUG_ECHOLNPGM(" ✓ ");//调试_ECHOLNPGM（“✓ ");
    if (!old_relative_mode) {
      //DEBUG_ECHOPGM("G90");//调试ECHOPGM（“G90”）；
      queue.enqueue_now_P(PSTR("G90"));
      //DEBUG_ECHOPGM(" ✓ ");//调试ECHOPGM（“✓ ");
    }
  }

  ForceCompleteUpdate();
  DEBUG_ECHOLNPGM("manmv done.");
  return;

  cannotmove:
    DEBUG_ECHOLNPAIR(" cannot move ", AS_CHAR(axiscode));
    return;
}

#if HAS_PID_HEATING
  void DGUSScreenHandler::HandleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t rawvalue = swap16(*(uint16_t*)val_ptr);
    DEBUG_ECHOLNPAIR("V1:", rawvalue);
    float value = (float)rawvalue / 10;
    DEBUG_ECHOLNPAIR("V2:", value);
    float newvalue = 0;

    switch (var.VP) {
      default: return;
        #if HAS_HOTEND
          case VP_E0_PID_P: newvalue = value; break;
          case VP_E0_PID_I: newvalue = scalePID_i(value); break;
          case VP_E0_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HOTENDS >= 2
          case VP_E1_PID_P: newvalue = value; break;
          case VP_E1_PID_I: newvalue = scalePID_i(value); break;
          case VP_E1_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HAS_HEATED_BED
          case VP_BED_PID_P: newvalue = value; break;
          case VP_BED_PID_I: newvalue = scalePID_i(value); break;
          case VP_BED_PID_D: newvalue = scalePID_d(value); break;
        #endif
    }

    DEBUG_ECHOLNPAIR_F("V3:", newvalue);
    *(float *)var.memadr = newvalue;

    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel//不要在下次更新时覆盖该值，因为显示可能会并行自动递增
  }
#endif // HAS_PID_HEATING//有没有电加热

#if ENABLED(BABYSTEPPING)
  void DGUSScreenHandler::HandleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleLiveAdjustZ");
    int16_t flag  = swap16(*(uint16_t*)val_ptr),
            steps = flag ? -20 : 20;
    ExtUI::smartAdjustAxis_steps(steps, ExtUI::axis_t::Z, true);
    ForceCompleteUpdate();
  }
#endif

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)

  void DGUSScreenHandler::HandleFilamentOption(DGUS_VP_Variable &var, void *val_ptr) {
    DEBUG_ECHOLNPGM("HandleFilamentOption");

    uint8_t e_temp = 0;
    filament_data.heated = false;
    uint16_t preheat_option = swap16(*(uint16_t*)val_ptr);
    if (preheat_option <= 8) {      // Load filament type//负载灯丝类型
      filament_data.action = 1;
    }
    else if (preheat_option >= 10) { // Unload filament type//卸载灯丝类型
      preheat_option -= 10;
      filament_data.action = 2;
      filament_data.purge_length = DGUS_FILAMENT_PURGE_LENGTH;
    }
    else {                          // Cancel filament operation//取消灯丝操作
      filament_data.action = 0;
    }

    switch (preheat_option) {
      case 0: // Load PLA//装载PLA
        #ifdef PREHEAT_1_TEMP_HOTEND
          e_temp = PREHEAT_1_TEMP_HOTEND;
        #endif
        break;
      case 1: // Load ABS//负载ABS
        TERN_(PREHEAT_2_TEMP_HOTEND, e_temp = PREHEAT_2_TEMP_HOTEND);
        break;
      case 2: // Load PET//装载宠物
        #ifdef PREHEAT_3_TEMP_HOTEND
          e_temp = PREHEAT_3_TEMP_HOTEND;
        #endif
        break;
      case 3: // Load FLEX//负载弹性
        #ifdef PREHEAT_4_TEMP_HOTEND
          e_temp = PREHEAT_4_TEMP_HOTEND;
        #endif
        break;
      case 9: // Cool down//冷静下来
      default:
        e_temp = 0;
        break;
    }

    if (filament_data.action == 0) { // Go back to utility screen//返回实用程序屏幕
      #if HAS_HOTEND
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
      #endif
      #if HOTENDS >= 2
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E1);
      #endif
      GotoScreen(DGUSLCD_SCREEN_UTILITY);
    }
    else { // Go to the preheat screen to show the heating progress//转到预热屏幕以显示加热进度
      switch (var.VP) {
        default: return;
          #if HAS_HOTEND
            case VP_E0_FILAMENT_LOAD_UNLOAD:
              filament_data.extruder = ExtUI::extruder_t::E0;
              thermalManager.setTargetHotend(e_temp, filament_data.extruder);
              break;
          #endif
          #if HAS_MULTI_EXTRUDER
            case VP_E1_FILAMENT_LOAD_UNLOAD:
              filament_data.extruder = ExtUI::extruder_t::E1;
              thermalManager.setTargetHotend(e_temp, filament_data.extruder);
              break;
          #endif
      }
      GotoScreen(DGUSLCD_SCREEN_FILAMENT_HEATING);
    }
  }

  void DGUSScreenHandler::HandleFilamentLoadUnload(DGUS_VP_Variable &var) {
    DEBUG_ECHOLNPGM("HandleFilamentLoadUnload");
    if (filament_data.action <= 0) return;

    // If we close to the target temperature, we can start load or unload the filament//如果我们接近目标温度，我们可以开始加载或卸载灯丝
    if (thermalManager.hotEnoughToExtrude(filament_data.extruder) && \
        thermalManager.targetHotEnoughToExtrude(filament_data.extruder)) {
      float movevalue = DGUS_FILAMENT_LOAD_LENGTH_PER_TIME;

      if (filament_data.action == 1) { // load filament//负荷灯丝
        if (!filament_data.heated) {
          //GotoScreen(DGUSLCD_SCREEN_FILAMENT_LOADING);//GotoScreen（DGUSLCD屏幕灯丝加载）；
          filament_data.heated = true;
        }
        movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
      }
      else { // unload filament//卸载灯丝
        if (!filament_data.heated) {
          GotoScreen(DGUSLCD_SCREEN_FILAMENT_UNLOADING);
          filament_data.heated = true;
        }
        // Before unloading extrude to prevent jamming//卸料前挤出，防止堵塞
        if (filament_data.purge_length >= 0) {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
          filament_data.purge_length -= movevalue;
        }
        else {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) - movevalue;
        }
      }
      ExtUI::setAxisPosition_mm(movevalue, filament_data.extruder);
    }
  }
#endif // DGUS_FILAMENT_LOADUNLOAD//DGUS_灯丝_装载卸载

bool DGUSScreenHandler::loop() {
  dgusdisplay.loop();

  const millis_t ms = millis();
  static millis_t next_event_ms = 0;

  if (!IsScreenComplete() || ELAPSED(ms, next_event_ms)) {
    next_event_ms = ms + DGUS_UPDATE_INTERVAL_MS;
    UpdateScreenVPData();
  }

  #if ENABLED(SHOW_BOOTSCREEN)
    static bool booted = false;

    if (!booted && TERN0(POWER_LOSS_RECOVERY, recovery.valid()))
      booted = true;

    if (!booted && ELAPSED(ms, TERN(USE_MKS_GREEN_UI, 1000, BOOTSCREEN_TIMEOUT)))
      booted = true;
  #endif
  return IsScreenComplete();
}

#endif // DGUS_LCD_UI_FYSETC//DGUS_LCD_UI_FYSETC
