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

#include "../DGUSDisplay.h"
#include "../DGUSVPVariable.h"
#include "../DGUSDisplayDef.h"

#include "../../../../inc/MarlinConfig.h"

enum DGUSLCD_Screens : uint8_t;

class DGUSScreenHandler {
public:
  DGUSScreenHandler() = default;

  static bool loop();

  // Send all 4 strings that are displayed on the infoscreen, confirmation screen and kill screen//发送信息屏幕、确认屏幕和终止屏幕上显示的所有4个字符串
  // The bools specifing whether the strings are in RAM or FLASH.//指定字符串是在RAM中还是在FLASH中的布尔值。
  static void sendinfoscreen(const char *line1, const char *line2, const char *line3, const char *line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash);

  static void HandleUserConfirmationPopUp(uint16_t ConfirmVP, const char *line1, const char *line2, const char *line3, const char *line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash);

  #if 0
  static void sendinfoscreen_ch_mks(const uint16_t *line1, const uint16_t *line2, const uint16_t *line3, const uint16_t *line4);
  static void sendinfoscreen_en_mks(const char *line1, const char *line2, const char *line3, const char *line4) ;
  static void sendinfoscreen_mks(const void *line1, const void *line2, const void *line3, const void *line4, uint16_t language);
  #endif

  // "M117" Message -- msg is a RAM ptr.//“M117”消息——msg是一个RAM ptr。
  static void setstatusmessage(const char *msg);
  // The same for messages from Flash//来自Flash的消息也是如此
  static void setstatusmessagePGM(PGM_P const msg);
  // Callback for VP "Display wants to change screen on idle printer"//VP“显示器想更改闲置打印机上的屏幕”的回调
  static void ScreenChangeHookIfIdle(DGUS_VP_Variable &var, void *val_ptr);
  // Callback for VP "Screen has been changed"//VP“屏幕已更改”的回调
  static void ScreenChangeHook(DGUS_VP_Variable &var, void *val_ptr);

  static void ScreenBackChange(DGUS_VP_Variable &var, void *val_ptr);

  // Callback for VP "All Heaters Off"//回拨VP“所有加热器关闭”
  static void HandleAllHeatersOff(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for "Change this temperature"//勾选“改变此温度”
  static void HandleTemperatureChanged(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for "Change Flowrate"//用于“改变流量”的挂钩
  static void HandleFlowRateChanged(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
    // Hook for manual move option//手动移动选项的挂钩
    static void HandleManualMoveOption(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  static void EEPROM_CTRL(DGUS_VP_Variable &var, void *val_ptr);
  static void LanguageChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void GetOffsetValue(DGUS_VP_Variable &var, void *val_ptr);
  static void Level_Ctrl_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void MeshLevel(DGUS_VP_Variable &var, void *val_ptr);
  static void MeshLevelDistanceConfig(DGUS_VP_Variable &var, void *val_ptr);
  static void ManualAssistLeveling(DGUS_VP_Variable &var, void *val_ptr);
  static void ZoffsetConfirm(DGUS_VP_Variable &var, void *val_ptr);
  static void Z_offset_select(DGUS_VP_Variable &var, void *val_ptr);
  static void GetManualMovestep(DGUS_VP_Variable &var, void *val_ptr);
  static void GetZoffsetDistance(DGUS_VP_Variable &var, void *val_ptr);
  static void GetMinExtrudeTemp(DGUS_VP_Variable &var, void *val_ptr);
  static void GetParkPos_MKS(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    static void HandleGetExMinTemp_MKS(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  static void DGUS_LanguageDisplay(uint8_t var);
  static void TMC_ChangeConfig(DGUS_VP_Variable &var, void *val_ptr);
  static void GetTurnOffCtrl(DGUS_VP_Variable &var, void *val_ptr);
  static void LanguagePInit(void);
  static void DGUS_Runout_Idle(void);
  static void DGUS_RunoutInit(void);
  static void DGUS_ExtrudeLoadInit(void);
  static void LCD_BLK_Adjust(DGUS_VP_Variable &var, void *val_ptr);
  static void SD_FileBack(DGUS_VP_Variable &var, void *val_ptr);

  // Hook for manual move.//用于手动移动的挂钩。
  static void HandleManualMove(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for manual extrude.//手动挤压的挂钩。
  static void HandleManualExtrude(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for motor lock and unlook//电机锁钩和解锁钩
  static void HandleMotorLockUnlock(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(POWER_LOSS_RECOVERY)
    // Hook for power loss recovery.//电源损耗恢复挂钩。
    static void HandlePowerLossRecovery(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  // Hook for settings//设置挂钩
  static void HandleSettings(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr);

  static void HandleStepPerMMChanged_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleStepPerMMExtruderChanged_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleMaxSpeedChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleExtruderMaxSpeedChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleAccChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleMaxAccChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleExtruderAccChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleChangeLevelPoint_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleTravelAccChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleFeedRateMinChange_MKS(DGUS_VP_Variable &var, void *val_ptr);
  static void HandleMin_T_F_MKS(DGUS_VP_Variable &var, void *val_ptr);

  #if HAS_PID_HEATING
    // Hook for "Change this temperature PID para"//“更改此温度PID参数”的挂钩
    static void HandleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr);
    // Hook for PID autotune//PID自动调谐挂钩
    static void HandlePIDAutotune(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if HAS_BED_PROBE
    // Hook for "Change probe offset z"//“更改探针偏移z”的挂钩
    static void HandleProbeOffsetZChanged(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if ENABLED(BABYSTEPPING)
    // Hook for live z adjust action//活z调整动作挂钩
    static void HandleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if HAS_FAN
    // Hook for fan control//风扇控制钩
    static void HandleFanControl(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  // Hook for heater control//加热器控制钩
  static void HandleHeaterControl(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(DGUS_PREHEAT_UI)
    // Hook for preheat//预热钩
    static void HandlePreheat(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
    // Hook for filament load and unload filament option//灯丝装载和卸载灯丝选件的挂钩
    static void HandleFilamentOption(DGUS_VP_Variable &var, void *val_ptr);
    // Hook for filament load and unload//灯丝装卸挂钩
    static void HandleFilamentLoadUnload(DGUS_VP_Variable &var);

    static void MKS_FilamentLoadUnload(DGUS_VP_Variable &var, void *val_ptr, const int filamentDir);
    static void MKS_FilamentLoad(DGUS_VP_Variable &var, void *val_ptr);
    static void MKS_FilamentUnLoad(DGUS_VP_Variable &var, void *val_ptr);
    static void MKS_LOAD_UNLOAD_IDLE();
    static void MKS_LOAD_Cancle(DGUS_VP_Variable &var, void *val_ptr);
    static void GetManualFilament(DGUS_VP_Variable &var, void *val_ptr);
    static void GetManualFilamentSpeed(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  #if ENABLED(SDSUPPORT)
    // Callback for VP "Display wants to change screen when there is a SD card"//VP“当有SD卡时显示器想改变屏幕”的回调
    static void ScreenChangeHookIfSD(DGUS_VP_Variable &var, void *val_ptr);
    // Scroll buttons on the file listing screen.//文件列表屏幕上的滚动按钮。
    static void DGUSLCD_SD_ScrollFilelist(DGUS_VP_Variable &var, void *val_ptr);
    // File touched.//文件被触动了。
    static void DGUSLCD_SD_FileSelected(DGUS_VP_Variable &var, void *val_ptr);
    // start print after confirmation received.//收到确认后开始打印。
    static void DGUSLCD_SD_StartPrint(DGUS_VP_Variable &var, void *val_ptr);
    // User hit the pause, resume or abort button.//用户点击暂停、恢复或中止按钮。
    static void DGUSLCD_SD_ResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr);
    // User confirmed the abort action//用户确认了中止操作
    static void DGUSLCD_SD_ReallyAbort(DGUS_VP_Variable &var, void *val_ptr);
    // User hit the tune button//用户点击调谐按钮
    static void DGUSLCD_SD_PrintTune(DGUS_VP_Variable &var, void *val_ptr);
    // Send a single filename to the display.//向显示器发送单个文件名。
    static void DGUSLCD_SD_SendFilename(DGUS_VP_Variable &var);
    // Marlin informed us that a new SD has been inserted.//Marlin通知我们已插入一个新的SD。
    static void SDCardInserted();
    // Marlin informed us that the SD Card has been removed().//Marlin通知我们SD卡已被移除（）。
    static void SDCardRemoved();
    // Marlin informed us about a bad SD Card.//马林告诉我们SD卡坏了。
    static void SDCardError();
    // Marlin informed us about SD print completion.//马林通知我们SD打印完成。
    static void SDPrintingFinished();
  #else
    static void PrintReturn(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  // OK Button on the Confirm screen.//确认屏幕上的OK按钮。
  static void ScreenConfirmedOK(DGUS_VP_Variable &var, void *val_ptr);

  // Update data after going to a new screen (by display or by GotoScreen)//转到新屏幕后更新数据（通过显示或GotoScreen）
  // remember: store the last-displayed screen, so it can be returned to.//记住：存储最后显示的屏幕，以便将其返回到。
  // (e.g for popup messages)//（例如，对于弹出消息）
  static void UpdateNewScreen(DGUSLCD_Screens newscreen, bool popup=false);

  // Recall the remembered screen.//回忆记忆中的屏幕。
  static void PopToOldScreen();

  // Make the display show the screen and update all VPs in it.//使显示屏显示屏幕并更新其中的所有VP。
  static void GotoScreen(DGUSLCD_Screens screen, bool ispopup = false);

  static void UpdateScreenVPData();

  // Helpers to convert and transfer data to the display.//用于将数据转换和传输到显示器的助手。
  static void DGUSLCD_SendWordValueToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendStringToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendStringToDisplayPGM(DGUS_VP_Variable &var);
  static void DGUSLCD_SendTemperaturePID(DGUS_VP_Variable &var);
  static void DGUSLCD_SendPercentageToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendPrintProgressToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendPrintTimeToDisplay(DGUS_VP_Variable &var);

  static void DGUSLCD_SendPrintTimeToDisplay_MKS(DGUS_VP_Variable &var);
  static void DGUSLCD_SendBabyStepToDisplay_MKS(DGUS_VP_Variable &var);
  static void DGUSLCD_SendFanToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendGbkToDisplay(DGUS_VP_Variable &var);
  static void DGUSLCD_SendStringToDisplay_Language_MKS(DGUS_VP_Variable &var);
  static void DGUSLCD_SendTMCStepValue(DGUS_VP_Variable &var);

  #if ENABLED(PRINTCOUNTER)
    static void DGUSLCD_SendPrintAccTimeToDisplay(DGUS_VP_Variable &var);
    static void DGUSLCD_SendPrintsTotalToDisplay(DGUS_VP_Variable &var);
  #endif
  #if HAS_FAN
    static void DGUSLCD_SendFanStatusToDisplay(DGUS_VP_Variable &var);
  #endif
  static void DGUSLCD_SendHeaterStatusToDisplay(DGUS_VP_Variable &var);
  #if ENABLED(DGUS_UI_WAITING)
    static void DGUSLCD_SendWaitingStatusToDisplay(DGUS_VP_Variable &var);
  #endif

  // Send a value from 0..100 to a variable with a range from 0..255//将0..100的值发送到范围为0..255的变量
  static void DGUSLCD_PercentageToUint8(DGUS_VP_Variable &var, void *val_ptr);

  static void DGUSLCD_SetUint8(DGUS_VP_Variable &var, void *val_ptr);

  template<typename T>
  static void DGUSLCD_SetValueDirectly(DGUS_VP_Variable &var, void *val_ptr) {
    if (!var.memadr) return;
    union { unsigned char tmp[sizeof(T)]; T t; } x;
    unsigned char *ptr = (unsigned char*)val_ptr;
    LOOP_L_N(i, sizeof(T)) x.tmp[i] = ptr[sizeof(T) - i - 1];
    *(T*)var.memadr = x.t;
  }

  // Send a float value to the display.//向显示器发送浮点值。
  // Display will get a 4-byte integer scaled to the number of digits://显示器将获得一个按位数缩放的4字节整数：
  // Tell the display the number of digits and it cheats by displaying a dot between...//告诉显示器数字的数量，它通过在…之间显示一个点来作弊。。。
  template<unsigned int decimals>
  static void DGUSLCD_SendFloatAsLongValueToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      float f = *(float *)var.memadr;
      f *= cpow(10, decimals);
      dgusdisplay.WriteVariable(var.VP, (long)f);
    }
  }

  // Send a float value to the display.//向显示器发送浮点值。
  // Display will get a 2-byte integer scaled to the number of digits://显示器将获得一个按位数缩放的2字节整数：
  // Tell the display the number of digits and it cheats by displaying a dot between...//告诉显示器数字的数量，它通过在…之间显示一个点来作弊。。。
  template<unsigned int decimals>
  static void DGUSLCD_SendFloatAsIntValueToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      float f = *(float *)var.memadr;
      DEBUG_ECHOLNPAIR_F(" >> ", f, 6);
      f *= cpow(10, decimals);
      dgusdisplay.WriteVariable(var.VP, (int16_t)f);
    }
  }

  // Force an update of all VP on the current screen.//强制更新当前屏幕上的所有VP。
  static inline void ForceCompleteUpdate() { update_ptr = 0; ScreenComplete = false; }
  // Has all VPs sent to the screen//是否已将所有VP发送到屏幕
  static inline bool IsScreenComplete() { return ScreenComplete; }

  static inline DGUSLCD_Screens getCurrentScreen() { return current_screen; }

  static inline void SetupConfirmAction( void (*f)()) { confirm_action_cb = f; }

private:
  static DGUSLCD_Screens current_screen;  //< currently on screen//<当前在屏幕上
  static constexpr uint8_t NUM_PAST_SCREENS = 4;
  static DGUSLCD_Screens past_screens[NUM_PAST_SCREENS]; //< LIFO with past screens for the "back" button.//<后进先出与“后退”按钮的过去屏幕。

  static uint8_t update_ptr;      //< Last sent entry in the VPList for the actual screen.//<实际屏幕的VPList中最后发送的条目。
  static uint16_t skipVP;         //< When updating the screen data, skip this one, because the user is interacting with it.//<更新屏幕数据时，跳过此数据，因为用户正在与之交互。
  static bool ScreenComplete;     //< All VPs sent to screen?//<是否将所有VP发送到屏幕？

  static uint16_t ConfirmVP;      //< context for confirm screen (VP that will be emulated-sent on "OK").//<确认屏幕的上下文（将在“确定”时发送模拟的VP）。

  #if ENABLED(SDSUPPORT)
    static int16_t top_file;      //< file on top of file chooser//<文件选择器顶部的文件
    static int16_t file_to_print; //< touched file to be confirmed//<待确认的文件
  #endif

  static void (*confirm_action_cb)();
};

#define MKS_Language_Choose   0x00
#define MKS_Language_NoChoose 0x01

#define MKS_SimpleChinese     0
#define MKS_English           1
extern uint8_t mks_language_index;
extern bool DGUSAutoTurnOff;

#if ENABLED(POWER_LOSS_RECOVERY)
  #define PLR_SCREEN_RECOVER MKSLCD_SCREEN_PRINT
  #define PLR_SCREEN_CANCEL MKSLCD_SCREEN_HOME
#endif
