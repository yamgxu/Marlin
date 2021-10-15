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
 * lcd/extui/anycubic_chiron/chiron_tft.cpp
 *
 * Extensible_UI implementation for Anycubic Chiron
 * Written By Nick Wells, 2020 [https://github.com/SwiftNick]
 *  (not affiliated with Anycubic, Ltd.)
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(ANYCUBIC_LCD_CHIRON)

#include "chiron_tft.h"
#include "Tunes.h"
#include "FileNavigator.h"

#include "../../../gcode/queue.h"
#include "../../../sd/cardreader.h"
#include "../../../libs/numtostr.h"
#include "../../../MarlinCore.h"

namespace Anycubic {

ChironTFT Chiron;
#if AUTO_DETECT_CHIRON_TFT
  panel_type_t   ChironTFT::panel_type = AC_panel_unknown;
#endif
last_error_t     ChironTFT::last_error;
printer_state_t  ChironTFT::printer_state;
paused_state_t   ChironTFT::pause_state;
heater_state_t   ChironTFT::hotend_state;
heater_state_t   ChironTFT::hotbed_state;
xy_uint8_t       ChironTFT::selectedmeshpoint;
char             ChironTFT::selectedfile[MAX_PATH_LEN + 1];
char             ChironTFT::panel_command[MAX_CMND_LEN + 1];
uint8_t          ChironTFT::command_len;
float            ChironTFT::live_Zoffset;
file_menu_t      ChironTFT::file_menu;

void ChironTFT::Startup() {
  selectedfile[0]   = '\0';
  panel_command[0]  = '\0';
  command_len       = 0;
  last_error        = AC_error_none;
  printer_state     = AC_printer_idle;
  pause_state       = AC_paused_idle;
  hotend_state      = AC_heater_off;
  hotbed_state      = AC_heater_off;
  live_Zoffset      = 0.0;
  file_menu         = AC_menu_file;

  // Setup pins for powerloss detection//用于断电检测的设置引脚
  // Two IO pins are connected on the Trigorilla Board//Trigorilla板上连接有两个IO引脚
  // On a power interruption the OUTAGECON_PIN goes low.//电源中断时，OUTAGECON_引脚变低。

  #if ENABLED(POWER_LOSS_RECOVERY)
    OUT_WRITE(OUTAGECON_PIN, HIGH);
  #endif

  // Filament runout is handled by Marlin settings in Configuration.h//灯丝跳动由配置h中的Marlin设置处理
  // opt_set    FIL_RUNOUT_STATE HIGH  // Pin state indicating that filament is NOT present.//opt_set FIL_RUNOUT_STATE HIGH//Pin STATE表示灯丝不存在。
  // opt_enable FIL_RUNOUT_PULLUP//选择\启用FIL\ U跳动\上拉
  TFTSer.begin(115200);

  // wait for the TFT panel to initialise and finish the animation//等待TFT面板初始化并完成动画
  delay_ms(250);

  // There are different panels for the Chiron with slightly different commands//凯龙有不同的面板，命令略有不同
  // So we need to know what we are working with.//所以我们需要知道我们在做什么。

  // Panel type can be defined otherwise detect it automatically//可以定义面板类型，否则会自动检测
  if (panel_type == AC_panel_unknown) DetectPanelType();

  // Signal Board has reset//信号板已复位
  SendtoTFTLN(AC_msg_main_board_has_reset);

  // Enable leveling and Disable end stops during print//在打印期间启用调平和禁用结束停止
  // as Z home places nozzle above the bed so we need to allow it past the end stops//因为Z home将喷嘴放在床的上方，所以我们需要让它通过末端止动块
  injectCommands_P(AC_cmnd_enable_leveling);

  // Startup tunes are defined in Tunes.h//启动调谐在tunes.h中定义
  PlayTune(BEEPER_PIN, TERN(AC_DEFAULT_STARTUP_TUNE, Anycubic_PowerOn, GB_PowerOn), 1);

  #if ACDEBUGLEVEL
    SERIAL_ECHOLNPAIR("AC Debug Level ", ACDEBUGLEVEL);
  #endif
  SendtoTFTLN(AC_msg_ready);
}

void ChironTFT::DetectPanelType() {
  #if AUTO_DETECT_CHIRON_TFT
    // Send a query to the TFT//向TFT发送查询
    SendtoTFTLN(AC_Test_for_OldPanel); // The panel will respond with 'SXY 480 320'//面板将响应“SXY 480 320”
    SendtoTFTLN(AC_Test_for_NewPanel); // the panel will respond with '[0]=0   ' to '[19]=0   '//面板将以“[0]=0”到“[19]=0”进行响应
  #endif
}

void ChironTFT::IdleLoop()  {
  if (ReadTFTCommand()) {
    ProcessPanelRequest();
    command_len = 0;
  }
  CheckHeaters();
}

void ChironTFT::PrinterKilled(PGM_P error,PGM_P component)  {
  SendtoTFTLN(AC_msg_kill_lcd);
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("PrinterKilled()\nerror: ", error , "\ncomponent: ", component);
  #endif
}

void ChironTFT::MediaEvent(media_event_t event)  {
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("ProcessMediaStatus() ", event);
  #endif
  switch (event) {
    case AC_media_inserted:
      SendtoTFTLN(AC_msg_sd_card_inserted);
      break;

    case AC_media_removed:
      SendtoTFTLN(AC_msg_sd_card_removed);
      break;

    case AC_media_error:
      last_error = AC_error_noSD;
      SendtoTFTLN(AC_msg_no_sd_card);
      break;
  }
}

void ChironTFT::TimerEvent(timer_event_t event)  {
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("TimerEvent() ", event);
    SERIAL_ECHOLNPAIR("Printer State: ", printer_state);
  #endif

  switch (event) {
    case AC_timer_started: {
      live_Zoffset = 0.0; // reset print offset//重置打印偏移量
      setSoftEndstopState(false);  // disable endstops to print//禁用结束停止打印
      printer_state = AC_printer_printing;
      SendtoTFTLN(AC_msg_print_from_sd_card);
    } break;

    case AC_timer_paused: {
      printer_state = AC_printer_paused;
      pause_state   = AC_paused_idle;
      SendtoTFTLN(AC_msg_paused);
    } break;

    case AC_timer_stopped: {
      if (printer_state != AC_printer_idle) {
        printer_state = AC_printer_stopping;
        SendtoTFTLN(AC_msg_print_complete);
      }
      setSoftEndstopState(true); // enable endstops//启用结束停止
    } break;
  }
}

void ChironTFT::FilamentRunout()  {
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("FilamentRunout() printer_state ", printer_state);
  #endif
  // 1 Signal filament out//1信号灯丝断开
  last_error = AC_error_filament_runout;
  SendtoTFTLN(isPrintingFromMedia() ? AC_msg_filament_out_alert : AC_msg_filament_out_block);
  PlayTune(BEEPER_PIN, FilamentOut, 1);
}

void ChironTFT::ConfirmationRequest(const char * const msg)  {
  // M108 continue//M108继续
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("ConfirmationRequest() ", msg, " printer_state:", printer_state);
  #endif
  switch (printer_state) {
    case AC_printer_pausing: {
      if (strcmp_P(msg, MARLIN_msg_print_paused) == 0 || strcmp_P(msg, MARLIN_msg_nozzle_parked) == 0) {
        SendtoTFTLN(AC_msg_paused); // enable continue button//启用继续按钮
        printer_state = AC_printer_paused;
      }
    } break;

    case AC_printer_resuming_from_power_outage:
    case AC_printer_printing:
    case AC_printer_paused: {
      // Heater timout, send acknowledgement//加热器超时，发送确认
      if (strcmp_P(msg, MARLIN_msg_heater_timeout) == 0) {
        pause_state = AC_paused_heater_timed_out;
        SendtoTFTLN(AC_msg_paused); // enable continue button//启用继续按钮
        PlayTune(BEEPER_PIN,Heater_Timedout,1);
      }
      // Reheat finished, send acknowledgement//重新加热完成，发送确认
      else if (strcmp_P(msg, MARLIN_msg_reheat_done) == 0) {
        pause_state = AC_paused_idle;
        SendtoTFTLN(AC_msg_paused); // enable continue button//启用继续按钮
      }
      // Filament Purging, send acknowledgement enter run mode//灯丝吹扫，发送确认，进入运行模式
      else if (strcmp_P(msg, MARLIN_msg_filament_purging) == 0) {
        pause_state = AC_paused_purging_filament;
        SendtoTFTLN(AC_msg_paused); // enable continue button//启用继续按钮
      }
    } break;
    default:
    break;
  }
}

void ChironTFT::StatusChange(const char * const msg)  {
  #if ACDEBUG(AC_MARLIN)
    SERIAL_ECHOLNPAIR("StatusChange() ", msg);
    SERIAL_ECHOLNPAIR("printer_state:", printer_state);
  #endif
  bool msg_matched = false;
  // The only way to get printer status is to parse messages//获取打印机状态的唯一方法是解析消息
  // Use the state to minimise the work we do here.//利用州政府最大限度地减少我们在这里所做的工作。
  switch (printer_state) {
    case AC_printer_probing: {
      // If probing completes ok save the mesh and park//如果探测完成，请保存网格并停驻
      // Ignore the custom machine name//忽略自定义计算机名
      if (strcmp_P(msg + strlen(CUSTOM_MACHINE_NAME), MARLIN_msg_ready) == 0) {
        injectCommands_P(PSTR("M500\nG27"));
        SendtoTFTLN(AC_msg_probing_complete);
        printer_state = AC_printer_idle;
        msg_matched = true;
      }
      // If probing fails dont save the mesh raise the probe above the bad point//如果探测失败，请不要保存网格，将探测器升高到坏点以上
      if (strcmp_P(msg, MARLIN_msg_probing_failed) == 0) {
        PlayTune(BEEPER_PIN, BeepBeepBeeep, 1);
        injectCommands_P(PSTR("G1 Z50 F500"));
        SendtoTFTLN(AC_msg_probing_complete);
        printer_state = AC_printer_idle;
        msg_matched = true;
      }
    } break;

    case AC_printer_printing: {
      if (strcmp_P(msg, MARLIN_msg_reheating) == 0) {
        SendtoTFTLN(AC_msg_paused); // enable continue button//启用继续按钮
        msg_matched = true;
       }
    } break;

    case AC_printer_pausing: {
      if (strcmp_P(msg, MARLIN_msg_print_paused) == 0) {
        SendtoTFTLN(AC_msg_paused);
        printer_state = AC_printer_paused;
        pause_state = AC_paused_idle;
        msg_matched = true;
       }
    } break;

    case AC_printer_stopping: {
      if (strcmp_P(msg, MARLIN_msg_print_aborted) == 0) {
        SendtoTFTLN(AC_msg_stop);
        printer_state = AC_printer_idle;
        msg_matched = true;
      }
    } break;
    default:
    break;
  }

  // If not matched earlier see if this was a heater message//如果之前不匹配，请查看这是否是加热器消息
  if (!msg_matched) {
    if (strcmp_P(msg, MARLIN_msg_extruder_heating) == 0) {
      SendtoTFTLN(AC_msg_nozzle_heating);
      hotend_state = AC_heater_temp_set;
    }
    else if (strcmp_P(msg, MARLIN_msg_bed_heating) == 0) {
      SendtoTFTLN(AC_msg_bed_heating);
      hotbed_state = AC_heater_temp_set;
    }
    else if (strcmp_P(msg, MARLIN_msg_EEPROM_version) == 0) {
      last_error = AC_error_EEPROM;
    }
  }
}

void ChironTFT::PowerLossRecovery()  {
  printer_state = AC_printer_resuming_from_power_outage; // Play tune to notify user we can recover.//播放tune以通知用户我们可以恢复。
  last_error = AC_error_powerloss;
  PlayTune(BEEPER_PIN, SOS, 1);
  SERIAL_ECHOLNPGM_P(AC_msg_powerloss_recovery);
}

void ChironTFT::PrintComplete() {
  SendtoTFT(AC_msg_print_complete);
  printer_state = AC_printer_idle;
  setSoftEndstopState(true); // enable endstops//启用结束停止
}

void ChironTFT::SendtoTFT(PGM_P str) {  // A helper to print PROGMEM string to the panel//将PROGMEM字符串打印到面板的助手
  #if ACDEBUG(AC_SOME)
    SERIAL_ECHOPGM_P(str);
  #endif
  while (const char c = pgm_read_byte(str++)) TFTSer.write(c);
}

void ChironTFT::SendtoTFTLN(PGM_P str = nullptr) {
  if (str) {
    #if ACDEBUG(AC_SOME)
      SERIAL_ECHOPGM("> ");
    #endif
    SendtoTFT(str);
    #if ACDEBUG(AC_SOME)
      SERIAL_EOL();
    #endif
  }
  TFTSer.println();
}

bool ChironTFT::ReadTFTCommand() {
  bool command_ready = false;
  while (TFTSer.available() > 0 && command_len < MAX_CMND_LEN) {
    panel_command[command_len] = TFTSer.read();
    if (panel_command[command_len] == '\n') {
      command_ready = true;
      break;
    }
    command_len++;
  }

  if (command_ready || command_len == MAX_CMND_LEN) {
    panel_command[command_len] = '\0';
    #if ACDEBUG(AC_ALL)
      SERIAL_ECHOLNPAIR("len(",command_len,") < ", panel_command);
    #endif
    command_ready = true;
  }
  return command_ready;
}

int8_t ChironTFT::FindToken(char c) {
  int8_t pos = 0;
  do {
    if (panel_command[pos] == c) {
      #if ACDEBUG(AC_INFO)
        SERIAL_ECHOLNPAIR("Tpos:", pos, " ", c);
      #endif
      return pos;
    }
  } while(++pos < command_len);
  #if ACDEBUG(AC_INFO)
    SERIAL_ECHOLNPAIR("Not found: ", c);
  #endif
  return -1;
}

void ChironTFT::CheckHeaters() {
  uint8_t faultDuration = 0;

  // if the hotend temp is abnormal, confirm state before signalling panel//如果热端温度异常，请在信号面板前确认状态
  celsius_float_t temp = getActualTemp_celsius(E0);
  while (!WITHIN(temp, HEATER_0_MINTEMP, HEATER_0_MAXTEMP)) {
    faultDuration++;
    if (faultDuration >= AC_HEATER_FAULT_VALIDATION_TIME) {
      SendtoTFTLN(AC_msg_nozzle_temp_abnormal);
      last_error = AC_error_abnormal_temp_t0;
      SERIAL_ECHOLNPAIR("Extruder temp abnormal! : ", temp);
      break;
    }
    delay_ms(500);
    temp = getActualTemp_celsius(E0);
  }

  // If the hotbed temp is abnormal, confirm state before signaling panel//如果温床温度异常，在信号面板前确认状态
  faultDuration = 0;
  temp = getActualTemp_celsius(BED);
  while (!WITHIN(temp, BED_MINTEMP, BED_MAXTEMP)) {
    faultDuration++;
    if (faultDuration >= AC_HEATER_FAULT_VALIDATION_TIME) {
      SendtoTFTLN(AC_msg_nozzle_temp_abnormal);
      last_error = AC_error_abnormal_temp_bed;
      SERIAL_ECHOLNPAIR("Bed temp abnormal! : ", temp);
      break;
    }
    delay_ms(500);
    temp = getActualTemp_celsius(E0);
  }

  // Update panel with hotend heater status//使用热端加热器状态更新面板
  if (hotend_state != AC_heater_temp_reached) {
    if (WITHIN(getActualTemp_celsius(E0) - getTargetTemp_celsius(E0), -(TEMP_WINDOW), TEMP_WINDOW)) {
      SendtoTFTLN(AC_msg_nozzle_heating_done);
      hotend_state = AC_heater_temp_reached;
    }
  }

  // Update panel with bed heater status//使用床加热器状态更新面板
  if (hotbed_state != AC_heater_temp_reached) {
    if (WITHIN(getActualTemp_celsius(BED) - getTargetTemp_celsius(BED), -(TEMP_BED_WINDOW), TEMP_BED_WINDOW)) {
      SendtoTFTLN(AC_msg_bed_heating_done);
      hotbed_state = AC_heater_temp_reached;
    }
  }
}

void ChironTFT::SendFileList(int8_t startindex) {
  // Respond to panel request for 4 files starting at index//响应从索引开始的4个文件的面板请求
  #if ACDEBUG(AC_INFO)
    SERIAL_ECHOLNPAIR("## SendFileList ## ", startindex);
  #endif
  SendtoTFTLN(PSTR("FN "));
  filenavigator.getFiles(startindex, panel_type, 4);
  SendtoTFTLN(PSTR("END"));
}

void ChironTFT::SelectFile() {
  if (panel_type == AC_panel_new) {
    strncpy(selectedfile, panel_command + 4, command_len - 3);
    selectedfile[command_len - 4] = '\0';
  }
  else {
    strncpy(selectedfile, panel_command + 4, command_len - 4);
    selectedfile[command_len - 5] = '\0';
  }
  #if ACDEBUG(AC_FILE)
    SERIAL_ECHOLNPAIR(" Selected File: ",selectedfile);
  #endif
  switch (selectedfile[0]) {
    case '/':   // Valid file selected//已选择有效文件
      SendtoTFTLN(AC_msg_sd_file_open_success);
      break;

    case '<':   // .. (go up folder level)// .. （上升到文件夹级别）
      filenavigator.upDIR();
      SendtoTFTLN(AC_msg_sd_file_open_failed);
      SendFileList( 0 );
      break;
    default:   // enter sub folder//输入子文件夹
      // for new panel remove the '.GCO' tag that was added to the end of the path//对于新面板，请删除添加到路径末尾的“.GCO”标记
      if (panel_type == AC_panel_new)
        selectedfile[strlen(selectedfile) - 4] = '\0';
      filenavigator.changeDIR(selectedfile);
      SendtoTFTLN(AC_msg_sd_file_open_failed);
      SendFileList( 0 );
      break;
  }
}

void ChironTFT::ProcessPanelRequest() {
  // Break these up into logical blocks // as its easier to navigate than one huge switch case!//将它们分解为逻辑块//因为它比一个巨大的交换机外壳更容易导航！
  int8_t tpos = FindToken('A');
  // Panel request are 'A0' - 'A36'//面板请求为“A0”-“A36”
  if (tpos != -1) {
    const int8_t req = atoi(&panel_command[tpos+1]);

    // Information requests A0 - A8 and A33//信息请求A0-A8和A33
    if (req <= 8 || req == 33) PanelInfo(req);

    // Simple Actions A9 - A28//简单动作A9-A28
    else if (req <= 28) PanelAction(req);

    // Process Initiation//进程启动
    else if (req <= 36) PanelProcess(req);
  }
  else {
    #if AUTO_DETECT_CHIRON_TFT
      // This may be a response to a panel type detection query//这可能是对面板类型检测查询的响应
      if (panel_type == AC_panel_unknown) {
        tpos = FindToken('S'); // old panel will respond to 'SIZE' with 'SXY 480 320'//旧面板将以“SXY 480 320”响应“尺寸”
        if (tpos != -1) {
          if (panel_command[tpos+1]== 'X' && panel_command[tpos+2]=='Y') {
            panel_type = AC_panel_standard;
            SERIAL_ECHOLNPGM_P(AC_msg_old_panel_detected);
          }
        }
        else {
          tpos = FindToken('['); // new panel will respond to 'J200' with '[0]=0'//新面板将以“[0]=0”响应“J200”
          if (tpos != -1) {
            if (panel_command[tpos+1]== '0' && panel_command[tpos+2]==']') {
              panel_type = AC_panel_new;
              SERIAL_ECHOLNPGM_P(AC_msg_new_panel_detected);
            }
          }
        }
        return;
      }
    #endif

    SendtoTFTLN(); // Ignore unknown requests//忽略未知请求
  }
}

void ChironTFT::PanelInfo(uint8_t req) {
  // information requests A0-A8 and A33//信息请求A0-A8和A33
  switch (req) {
    case 0:   // A0 Get HOTEND Temp//A0获得热端温度
      SendtoTFT(PSTR("A0V "));
      TFTSer.println(getActualTemp_celsius(E0));
      break;

    case 1:   // A1 Get HOTEND Target Temp//A1获取热端目标温度
      SendtoTFT(PSTR("A1V "));
      TFTSer.println(getTargetTemp_celsius(E0));
      break;

    case 2:   // A2 Get BED Temp//A2卧床温度
      SendtoTFT(PSTR("A2V "));
      TFTSer.println(getActualTemp_celsius(BED));
      break;

    case 3:   // A3 Get BED Target Temp//A3获得床位目标温度
      SendtoTFT(PSTR("A3V "));
      TFTSer.println(getTargetTemp_celsius(BED));
      break;

    case 4:   // A4 Get FAN Speed//A4获得风扇转速
      SendtoTFT(PSTR("A4V "));
      TFTSer.println(getActualFan_percent(FAN0));
      break;

    case 5:   // A5 Get Current Coordinates//A5获取当前坐标
      SendtoTFT(PSTR("A5V X: "));
      TFTSer.print(getAxisPosition_mm(X));
      SendtoTFT(PSTR(" Y: "));
      TFTSer.print(getAxisPosition_mm(Y));
      SendtoTFT(PSTR(" Z: "));
      TFTSer.println(getAxisPosition_mm(Z));
      break;

    case 6:   // A6 Get printing progress//A6获得打印进度
      if (isPrintingFromMedia()) {
        SendtoTFT(PSTR("A6V "));
        TFTSer.println(ui8tostr2(getProgress_percent()));
      }
      else
        SendtoTFTLN(PSTR("A6V ---"));
      break;

    case 7: { // A7 Get Printing Time//A7获得打印时间
      uint32_t time = getProgress_seconds_elapsed() / 60;
      SendtoTFT(PSTR("A7V "));
      TFTSer.print(ui8tostr2(time / 60));
      SendtoTFT(PSTR(" H "));
      TFTSer.print(ui8tostr2(time % 60));
      SendtoTFT(PSTR(" M"));
      #if ACDEBUG(AC_ALL)
        SERIAL_ECHOLNPAIR("Print time ", ui8tostr2(time / 60), ":", ui8tostr2(time % 60));
      #endif
    } break;

    case 8:   // A8 Get SD Card list A8 S0//A8获取SD卡列表A8 S0
      if (!isMediaInserted()) safe_delay(500);
      if (!isMediaInserted())   // Make sure the card is removed//确保卡已卸下
        SendtoTFTLN(AC_msg_no_sd_card);
      else if (panel_command[3] == 'S')
        SendFileList( atoi( &panel_command[4] ) );
      break;

    case 33:   // A33 Get firmware info//A33获取固件信息
      SendtoTFT(PSTR("J33 "));
      // If there is an error recorded, show that instead of the FW version//如果记录了错误，则显示该错误，而不是FW版本
      if (!GetLastError()) SendtoTFTLN(PSTR(SHORT_BUILD_VERSION));
      break;
  }
}

void ChironTFT::PanelAction(uint8_t req) {
  switch (req) {
    case  9:   // A9 Pause SD print//A9暂停SD打印
      if (isPrintingFromMedia()) {
        SendtoTFTLN(AC_msg_pause);
        pausePrint();
        printer_state = AC_printer_pausing;
      }
      else
        SendtoTFTLN(AC_msg_stop);
      break;

    case 10: // A10 Resume SD Print//A10简历SD打印
      if (pause_state == AC_paused_idle || printer_state == AC_printer_resuming_from_power_outage)
        resumePrint();
      else
        setUserConfirmed();
      break;

    case 11:   // A11 Stop SD print//A11停止SD打印
      if (isPrintingFromMedia()) {
        printer_state = AC_printer_stopping;
        stopPrint();
      }
      else {
        if (printer_state == AC_printer_resuming_from_power_outage)
          injectCommands_P(PSTR("M1000 C")); // Cancel recovery//取消恢复
        SendtoTFTLN(AC_msg_stop);
        printer_state = AC_printer_idle;
      }
      break;

    case 12:   // A12 Kill printer//A12压井打印机
      kill();  // from marlincore.h//来自marlincore.h
      break;

    case 13:   // A13 Select file//A13选择文件
      SelectFile();
      break;

    case 14: { // A14 Start Printing//A14开始打印
      // Allows printer to restart the job if we dont want to recover//如果我们不想恢复，允许打印机重新启动作业
      if (printer_state == AC_printer_resuming_from_power_outage) {
        injectCommands_P(PSTR("M1000 C")); // Cancel recovery//取消恢复
        printer_state = AC_printer_idle;
      }
      #if ACDebugLevel >= 1
        SERIAL_ECHOLNPAIR_F("Print: ", selectedfile);
      #endif
      printFile(selectedfile);
      SendtoTFTLN(AC_msg_print_from_sd_card);
    } break;

    case 15:   // A15 Resuming from outage//A15从大修中恢复
      if (printer_state == AC_printer_resuming_from_power_outage) {
        // Need to home here to restore the Z position//需要回到原点才能恢复Z位置
        injectCommands_P(AC_cmnd_power_loss_recovery);
        injectCommands_P(PSTR("M1000"));  // home and start recovery//回家并开始恢复
      }
      break;

    case 16: { // A16 Set HotEnd temp  A17 S170//A16设置热端温度A17 S170
      const float set_Htemp = atof(&panel_command[5]);
      hotend_state = set_Htemp ? AC_heater_temp_set : AC_heater_off;
      switch ((char)panel_command[4]) {
        // Set Temp//设定温度
        case 'S': case 'C': setTargetTemp_celsius(set_Htemp, E0);
      }
    } break;

    case 17: { // A17 Set bed temp//A17设定床温
      const float set_Btemp = atof(&panel_command[5]);
      hotbed_state = set_Btemp ? AC_heater_temp_set : AC_heater_off;
      if (panel_command[4] == 'S')
        setTargetTemp_celsius(set_Btemp, BED);
    } break;

    case 18:   // A18 Set Fan Speed//A18设定风扇转速
      if (panel_command[4] == 'S')
        setTargetFan_percent(atof(&panel_command[5]), FAN0);
      break;

    case 19:   // A19 Motors off//A19电机关闭
      if (!isPrinting()) {
        disable_all_steppers(); // from marlincore.h//来自marlincore.h
        SendtoTFTLN(AC_msg_ready);
      }
      break;

    case 20:   // A20 Read/write print speed//A20读/写打印速度
      if (panel_command[4] == 'S')
        setFeedrate_percent(atoi(&panel_command[5]));
      else {
        SendtoTFT(PSTR("A20V "));
        TFTSer.println(getFeedrate_percent());
      }
      break;

    case 21:   // A21 Home Axis  A21 X//A21主轴A21 X
      if (!isPrinting()) {
        switch ((char)panel_command[4]) {
          case 'X': injectCommands_P(PSTR("G28X")); break;
          case 'Y': injectCommands_P(PSTR("G28Y")); break;
          case 'Z': injectCommands_P(PSTR("G28Z")); break;
          case 'C': injectCommands_P(G28_STR); break;
        }
      }
      break;

    case 22: {   // A22 Move Axis//A22移动轴
      // The commands have changed on the new panel//新面板上的命令已更改
      // Old TFT A22 X -1F1500      A22 X +1F1500//旧TFT A22 X-1F1500 A22 X+1F1500
      // New TFT A22 X-1.0 F1500    A22 X1.0 F1500//新TFT A22 X-1.0 F1500 A22 X1.0 F1500

      // lets just wrap this in a gcode relative nonprint move and let the controller deal with it//让我们把它包装在一个gcode相对非打印移动中，让控制器来处理它
      // G91 G0 <panel command> G90//G91 G0<面板命令>G90

      if (!isPrinting()) { // Ignore request if printing//如果打印，则忽略请求
        char MoveCmnd[30];
        sprintf_P(MoveCmnd, PSTR("G91\nG0%s\nG90"), panel_command + 3);
        #if ACDEBUG(AC_ACTION)
          SERIAL_ECHOLNPAIR("Move: ", MoveCmnd);
        #endif
        setSoftEndstopState(true);  // enable endstops//启用结束停止
        injectCommands(MoveCmnd);
      }
    } break;

    case 23:   // A23 Preheat PLA//A23预热聚乳酸
      // Ignore request if printing//如果打印，则忽略请求
      if (!isPrinting()) {
        // Temps defined in configuration.h//在configuration.h中定义的temp
        setTargetTemp_celsius(PREHEAT_1_TEMP_BED, BED);
        setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, E0);
        SendtoTFTLN();
        hotbed_state = AC_heater_temp_set;
        hotend_state = AC_heater_temp_set;
      }
      break;

    case 24:   // A24 Preheat ABS//A24预热ABS
      // Ignore request if printing//如果打印，则忽略请求
      if (!isPrinting()) {
        setTargetTemp_celsius(PREHEAT_2_TEMP_BED, BED);
        setTargetTemp_celsius(PREHEAT_2_TEMP_HOTEND, E0);
        SendtoTFTLN();
        hotbed_state = AC_heater_temp_set;
        hotend_state = AC_heater_temp_set;
      }
      break;

    case 25:   // A25 Cool Down//A25冷却
      // Ignore request if printing//如果打印，则忽略请求
      if (!isPrinting()) {
        setTargetTemp_celsius(0, E0);
        setTargetTemp_celsius(0, BED);
        SendtoTFTLN(AC_msg_ready);
        hotbed_state = AC_heater_off;
        hotend_state = AC_heater_off;
      }
      break;

    case 26:   // A26 Refresh SD//A26刷新SD
      if (card.isMounted())card.release();
      card.mount();
      safe_delay(500);
      filenavigator.reset();
      break;

    case 27:   // A27 Servo Angles adjust//A27伺服角度调整
      break;

    case 28:   // A28 Filament set A28 O/C//A28灯丝组件A28 O/C
      // Ignore request if printing//如果打印，则忽略请求
      if (isPrinting()) break;
      SendtoTFTLN();
      break;
  }
}

void ChironTFT::PanelProcess(uint8_t req) {
  switch (req) {
    case 29: { // A29 Read Mesh Point A29 X1 Y1//A29读取网格点A29 X1 Y1
      xy_uint8_t pos;
      float pos_z;
      pos.x = atoi(&panel_command[FindToken('X')+1]);
      pos.y = atoi(&panel_command[FindToken('Y')+1]);
      pos_z = getMeshPoint(pos);

      SendtoTFT(PSTR("A29V "));
      TFTSer.println(pos_z * 100);
      if (!isPrinting()) {
        setSoftEndstopState(true);  // disable endstops//禁用结束停止
        // If the same meshpoint is selected twice in a row, move the head to that ready for adjustment//如果连续两次选择同一网格点，请将头部移动到准备进行调整的位置
        if ((selectedmeshpoint.x == pos.x) && (selectedmeshpoint.y == pos.y)) {
          if (!isPositionKnown())
            injectCommands_P(G28_STR); // home//家

          if (isPositionKnown()) {
            #if ACDEBUG(AC_INFO)
              SERIAL_ECHOLNPAIR("Moving to mesh point at x: ", pos.x, " y: ", pos.y, " z: ", pos_z);
            #endif
            // Go up before moving//先上去再搬家
            setAxisPosition_mm(3.0,Z);

            setAxisPosition_mm(17 + (93 * pos.x), X);
            setAxisPosition_mm(20 + (93 * pos.y), Y);
            setAxisPosition_mm(0.0, Z);
            #if ACDEBUG(AC_INFO)
              SERIAL_ECHOLNPAIR("Current Z: ", getAxisPosition_mm(Z));
            #endif
          }
        }
        selectedmeshpoint.x = pos.x;
        selectedmeshpoint.y = pos.y;
      }
    } break;

    case 30: {   // A30 Auto leveling//A30自动调平
      if (FindToken('S') != -1) { // Start probing New panel adds spaces..//开始探测新面板添加空间。。
        // Ignore request if printing//如果打印，则忽略请求
        if (isPrinting())
          SendtoTFTLN(AC_msg_probing_not_allowed); // forbid auto leveling//禁止自动调平
        else {


          SendtoTFTLN(AC_msg_start_probing);
          injectCommands_P(PSTR("G28\nG29"));
          printer_state = AC_printer_probing;
        }
      }
      else {
        SendtoTFTLN(AC_msg_start_probing); // Just enter levelling menu//只需进入平层菜单
      }
    } break;

    case 31: { // A31 Adjust all Probe Points//A31调整所有探头点
      // The tokens can occur in different places on the new panel so we need to find it.//标记可以出现在新面板上的不同位置，因此我们需要找到它。

      if (FindToken('C') != -1) { // Restore and apply original offsets//恢复并应用原始偏移
        if (!isPrinting()) {
          injectCommands_P(PSTR("M501\nM420 S1"));
          selectedmeshpoint.x = selectedmeshpoint.y = 99;
          SERIAL_ECHOLNPGM_P(AC_msg_mesh_changes_abandoned);
        }
      }

      else if (FindToken('D') != -1) { // Save Z Offset tables and restore leveling state//保存Z偏移表并恢复水平状态
        if (!isPrinting()) {
          setAxisPosition_mm(1.0,Z); // Lift nozzle before any further movements are made//在进行任何进一步移动之前，提起喷嘴
          injectCommands_P(PSTR("M500"));
          SERIAL_ECHOLNPGM_P(AC_msg_mesh_changes_saved);
          selectedmeshpoint.x = selectedmeshpoint.y = 99;
        }
      }

      else if (FindToken('G') != -1) { // Get current offset//获取当前偏移量
        SendtoTFT(PSTR("A31V "));
        // When printing use the live z Offset position//打印时，请使用活动z偏移位置
        // we will use babystepping to move the print head//我们将使用babystepping移动打印头
        if (isPrinting())
          TFTSer.println(live_Zoffset);
        else {
          TFTSer.println(getZOffset_mm());
          selectedmeshpoint.x = selectedmeshpoint.y = 99;
        }
      }

      else {
        int8_t tokenpos = FindToken('S');
        if (tokenpos != -1) { // Set offset (adjusts all points by value)//设置偏移（按值调整所有点）
          float Zshift = atof(&panel_command[tokenpos+1]);
          setSoftEndstopState(false);  // disable endstops//禁用结束停止
          // Allow temporary Z position nudging during print//打印期间允许临时Z位置微移
          // From the leveling panel use the all points UI to adjust the print pos.//从“调平”面板使用“所有点”用户界面调整打印位置。
          if (isPrinting()) {
            #if ACDEBUG(AC_INFO)
              SERIAL_ECHOLNPAIR("Change Zoffset from:", live_Zoffset, " to ", live_Zoffset + Zshift);
            #endif
            if (isAxisPositionKnown(Z)) {
              #if ACDEBUG(AC_INFO)
                const float currZpos = getAxisPosition_mm(Z);
                SERIAL_ECHOLNPAIR("Nudge Z pos from ", currZpos, " to ", currZpos + constrain(Zshift, -0.05, 0.05));
              #endif
              // Use babystepping to adjust the head position//使用babystepping调整头部位置
              int16_t steps = mmToWholeSteps(constrain(Zshift,-0.05,0.05), Z);
              #if ACDEBUG(AC_INFO)
                SERIAL_ECHOLNPAIR("Steps to move Z: ", steps);
              #endif
              babystepAxis_steps(steps, Z);
              live_Zoffset += Zshift;
            }
            SendtoTFT(PSTR("A31V "));
            TFTSer.println(live_Zoffset);
          }
          else {
            GRID_LOOP(x, y) {
              const xy_uint8_t pos { x, y };
              const float currval = getMeshPoint(pos);
              setMeshPoint(pos, constrain(currval + Zshift, AC_LOWEST_MESHPOINT_VAL, 2));
              #if ACDEBUG(AC_INFO)
                SERIAL_ECHOLNPAIR("Change mesh point X", x," Y",y ," from ", currval, " to ", getMeshPoint(pos) );
              #endif
            }
            const float currZOffset = getZOffset_mm();
            #if ACDEBUG(AC_INFO)
              SERIAL_ECHOLNPAIR("Change probe offset from ", currZOffset, " to  ", currZOffset + Zshift);
            #endif

            setZOffset_mm(currZOffset + Zshift);
            SendtoTFT(PSTR("A31V "));
            TFTSer.println(getZOffset_mm());

            if (isAxisPositionKnown(Z)) {
              // Move Z axis//移动Z轴
              const float currZpos = getAxisPosition_mm(Z);
              #if ACDEBUG(AC_INFO)
                SERIAL_ECHOLNPAIR("Move Z pos from ", currZpos, " to ", currZpos + constrain(Zshift, -0.05, 0.05));
              #endif
              setAxisPosition_mm(currZpos+constrain(Zshift,-0.05,0.05),Z);
            }
          }
        }
      }
    } break;

    case 32: { // A32 clean leveling beep flag//A32清洁调平嘟嘟声标志
      // Ignore request if printing//如果打印，则忽略请求
      //if (isPrinting()) break;//如果（isPrinting（））中断；
      //injectCommands_P(PSTR("M500\nM420 S1\nG1 Z10 F240\nG1 X0 Y0 F6000"));//（PSTR（“M500\nM420 S1\nG1 Z10 F240\nG1 X0 Y0 F6000”）；
      //TFTSer.println();//TFTSer.println（）；
    } break;

    // A33 firmware info request see PanelInfo()//A33固件信息请求见PanelInfo（）

    case 34: {  // A34 Adjust single mesh point A34 C/S X1 Y1 V123//A34调整单网格点A34 C/S X1 Y1 V123
      if (panel_command[3] == 'C') { // Restore original offsets//恢复原始偏移
        injectCommands_P(PSTR("M501\nM420 S1"));
        selectedmeshpoint.x = selectedmeshpoint.y = 99;
        //printer_state = AC_printer_idle;//打印机\u状态=AC\u打印机\u空闲；
      }
      else {
        xy_uint8_t pos;
        pos.x = atoi(&panel_command[5]);
        pos.y = atoi(&panel_command[8]);

        float currmesh = getMeshPoint(pos);
        float newval   = atof(&panel_command[11])/100;
        #if ACDEBUG(AC_INFO)
          SERIAL_ECHOLNPAIR("Change mesh point x:", pos.x, " y:", pos.y);
          SERIAL_ECHOLNPAIR("from ", currmesh, " to ", newval);
        #endif
        // Update Meshpoint//更新网格点
        setMeshPoint(pos,newval);
        if (printer_state == AC_printer_idle || printer_state == AC_printer_probing /*!isPrinting()*/) {
          // if we are at the current mesh point indicated on the panel Move Z pos +/- 0.05mm//如果我们位于面板上指示的当前网格点，移动Z位置+/-0.05毫米
          // (The panel changes the mesh value by +/- 0.05mm on each button press)//（每次按下按钮时，面板将网格值更改+/-0.05 mm）
          if (selectedmeshpoint.x == pos.x && selectedmeshpoint.y == pos.y) {
            setSoftEndstopState(false);
            float currZpos = getAxisPosition_mm(Z);
            #if ACDEBUG(AC_INFO)
              SERIAL_ECHOLNPAIR("Move Z pos from ", currZpos, " to ", currZpos + constrain(newval - currmesh, -0.05, 0.05));
            #endif
            setAxisPosition_mm(currZpos + constrain(newval - currmesh, -0.05, 0.05), Z);
          }
        }
      }
    }  break;

    case 36:    // A36 Auto leveling for new TFT bet that was a typo in the panel code!//A36自动调平用于新TFT，面板代码中有一个输入错误！
      SendtoTFTLN(AC_msg_start_probing);
      break;
  }
}

bool ChironTFT::GetLastError() {
  switch (last_error) {
    case AC_error_abnormal_temp_bed: SendtoTFTLN(AC_msg_error_bed_temp);    break;
    case AC_error_abnormal_temp_t0:  SendtoTFTLN(AC_msg_error_hotend_temp); break;
    case AC_error_noSD:              SendtoTFTLN(AC_msg_error_sd_card);     break;
    case AC_error_powerloss:         SendtoTFTLN(AC_msg_power_loss);        break;
    case AC_error_EEPROM:            SendtoTFTLN(AC_msg_eeprom_version);    break;
    case AC_error_filament_runout:   SendtoTFTLN(AC_msg_filament_out);      break;
    default: return false;
  }
  last_error = AC_error_none;
  return true;
}

} // Anycubic namespace//任意立方名称空间

#endif // ANYCUBIC_LCD_CHIRON//任意立方_液晶_凯龙
