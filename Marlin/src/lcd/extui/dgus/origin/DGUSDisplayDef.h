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

#include "../DGUSDisplayDef.h"

enum DGUSLCD_Screens : uint8_t {
  DGUSLCD_SCREEN_BOOT                = 0,
  DGUSLCD_SCREEN_MAIN                = 10,
  DGUSLCD_SCREEN_TEMPERATURE         = 20,
  DGUSLCD_SCREEN_STATUS              = 30,
  DGUSLCD_SCREEN_STATUS2             = 32,
  DGUSLCD_SCREEN_MANUALMOVE          = 40,
  DGUSLCD_SCREEN_MANUALEXTRUDE       = 42,
  DGUSLCD_SCREEN_FANANDFEEDRATE      = 44,
  DGUSLCD_SCREEN_FLOWRATES           = 46,
  DGUSLCD_SCREEN_SDFILELIST          = 50,
  DGUSLCD_SCREEN_SDPRINTMANIPULATION = 52,
  DGUSLCD_SCREEN_POWER_LOSS          = 100,
  DGUSLCD_SCREEN_PREHEAT             = 120,
  DGUSLCD_SCREEN_UTILITY             = 110,
  DGUSLCD_SCREEN_FILAMENT_HEATING    = 146,
  DGUSLCD_SCREEN_FILAMENT_LOADING    = 148,
  DGUSLCD_SCREEN_FILAMENT_UNLOADING  = 158,
  DGUSLCD_SCREEN_SDPRINTTUNE         = 170,
  DGUSLCD_SCREEN_CONFIRM             = 240,
  DGUSLCD_SCREEN_KILL                = 250, ///< Kill Screen. Must always be 250 (to be able to display "Error wrong LCD Version")///<Kill屏幕。必须始终为250（才能显示“错误LCD版本”）
  DGUSLCD_SCREEN_WAITING             = 251,
  DGUSLCD_SCREEN_POPUP               = 252, ///< special target, popup screen will also return this code to say "return to previous screen"///<特殊目标，弹出屏幕也将返回此代码，显示“返回上一屏幕”
  DGUSLCD_SCREEN_UNUSED              = 255
};

// Display Memory layout used (T5UID)//使用的显示内存布局（T5UID）
// Except system variables this is arbitrary, just to organize stuff....//除了系统变量，这是任意的，只是为了组织东西。。。。

// 0x0000 .. 0x0FFF  -- System variables and reserved by the display//0x0000。。0x0FFF——系统变量，由显示器保留
// 0x1000 .. 0x1FFF  -- Variables to never change location, regardless of UI Version//0x1000。。0x1FFF——无论UI版本如何，都不会更改位置的变量
// 0x2000 .. 0x2FFF  -- Controls (VPs that will trigger some action)//0x2000。。0x2FF—控件（将触发某些操作的VP）
// 0x3000 .. 0x4FFF  -- Marlin Data to be displayed//0x3000。。0x4FFF——要显示的马林鱼数据
// 0x5000 ..         -- SPs (if we want to modify display elements, e.g change color or like) -- currently unused//0x5000..--SPs（如果我们想修改显示元素，例如更改颜色或类似内容）--当前未使用

// As there is plenty of space (at least most displays have >8k RAM), we do not pack them too tight,//由于有足够的空间（至少大多数显示器的RAM大于8k），我们不会将它们包装得太紧，
// so that we can keep variables nicely together in the address space.//这样我们就可以在地址空间中将变量很好地组合在一起。

// UI Version always on 0x1000...0x1002 so that the firmware can check this and bail out.//UI版本始终在0x1000…0x1002上，以便固件可以检查并退出。
constexpr uint16_t VP_UI_VERSION_MAJOR = 0x1000;  // Major -- incremented when incompatible//Major--不兼容时递增
constexpr uint16_t VP_UI_VERSION_MINOR = 0x1001;  // Minor -- incremented on new features, but compatible//次要--在新功能上增加，但兼容
constexpr uint16_t VP_UI_VERSION_PATCH = 0x1002;  // Patch -- fixed which do not change functionality.//修补程序--修复了不更改功能的修补程序。
constexpr uint16_t VP_UI_FLAVOUR       = 0x1010;  // lets reserve 16 bytes here to determine if UI is suitable for this Marlin. tbd.//让我们在这里保留16个字节，以确定UI是否适合此Marlin。待定。

// Storage space for the Killscreen messages. 0x1100 - 0x1200 . Reused for the popup.//屏幕消息的存储空间。0x1100-0x1200。重新用于弹出窗口。
constexpr uint16_t VP_MSGSTR1 = 0x1100;
constexpr uint8_t VP_MSGSTR1_LEN = 0x20;  // might be more place for it...//可能是更适合它的地方。。。
constexpr uint16_t VP_MSGSTR2 = 0x1140;
constexpr uint8_t VP_MSGSTR2_LEN = 0x20;
constexpr uint16_t VP_MSGSTR3 = 0x1180;
constexpr uint8_t VP_MSGSTR3_LEN = 0x20;
constexpr uint16_t VP_MSGSTR4 = 0x11C0;
constexpr uint8_t VP_MSGSTR4_LEN = 0x20;

// Screenchange request for screens that only make sense when printer is idle.//仅当打印机空闲时才有意义的屏幕的屏幕更改请求。
// e.g movement is only allowed if printer is not printing.//例如，仅当打印机未打印时才允许移动。
// Marlin must confirm by setting the screen manually.//Marlin必须通过手动设置屏幕进行确认。
constexpr uint16_t VP_SCREENCHANGE_ASK = 0x2000;
constexpr uint16_t VP_SCREENCHANGE = 0x2001;   // Key-Return button to new menu pressed. Data contains target screen in low byte and info in high byte.//按下返回新菜单的按键。数据包含低字节的目标屏幕和高字节的信息。
constexpr uint16_t VP_TEMP_ALL_OFF = 0x2002;   // Turn all heaters off. Value arbitrary ;)=//关掉所有加热器。任意值；）=
constexpr uint16_t VP_SCREENCHANGE_WHENSD = 0x2003; // "Print" Button touched -- go only there if there is an SD Card.//点击“打印”按钮——只有在有SD卡的情况下才去那里。

constexpr uint16_t VP_CONFIRMED = 0x2010; // OK on confirm screen.//确认屏幕上的OK。

// Buttons on the SD-Card File listing.//SD卡文件列表上的按钮。
constexpr uint16_t VP_SD_ScrollEvent = 0x2020; // Data: 0 for "up a directory", numbers are the amount to scroll, e.g -1 one up, 1 one down//数据：0表示“向上移动目录”，数字是要滚动的数量，例如-1向上移动，1向下移动
constexpr uint16_t VP_SD_FileSelected = 0x2022; // Number of file field selected.//所选文件字段的数目。
constexpr uint16_t VP_SD_FileSelectConfirm = 0x2024; // (This is a virtual VP and emulated by the Confirm Screen when a file has been confirmed)//（这是一个虚拟VP，在确认文件后由确认屏幕模拟）

constexpr uint16_t VP_SD_ResumePauseAbort = 0x2026; // Resume(Data=0), Pause(Data=1), Abort(Data=2) SD Card prints//恢复（数据=0）、暂停（数据=1）、中止（数据=2）SD卡打印
constexpr uint16_t VP_SD_AbortPrintConfirmed = 0x2028; // Abort print confirmation (virtual, will be injected by the confirm dialog)//中止打印确认（虚拟，将由确认对话框注入）
constexpr uint16_t VP_SD_Print_Setting = 0x2040;
constexpr uint16_t VP_SD_Print_LiveAdjustZ = 0x2050; // Data: 0 down, 1 up//数据：0向下，1向上

// Controls for movement (we can't use the incremental / decremental feature of the display at this feature works only with 16 bit values//运动控制（我们不能使用显示器的递增/递减功能，此功能仅适用于16位值
// (which would limit us to 655.35mm, which is likely not a problem for common setups, but i don't want to rule out hangprinters support)//（这将把我们限制在655.35mm，这对于普通设置来说可能不是问题，但我不想排除hangprinters支持）
// A word about the coding: The VP will be per axis and the return code will be an signed 16 bit value in 0.01 mm resolution, telling us//关于编码的一句话：VP将是每轴的，返回码将是一个有符号的16位值，分辨率为0.01 mm，告诉我们
// the relative travel amount t he user wants to do. So eg. if the display sends us VP=2100 with value 100, the user wants us to move X by +1 mm.//用户想要做的相对移动量。例如，如果显示器发送给我们VP=2100，值为100，用户希望我们移动X+1毫米。
constexpr uint16_t VP_MOVE_X = 0x2100;
constexpr uint16_t VP_MOVE_Y = 0x2102;
constexpr uint16_t VP_MOVE_Z = 0x2104;
constexpr uint16_t VP_MOVE_E0 = 0x2110;
constexpr uint16_t VP_MOVE_E1 = 0x2112;
//constexpr uint16_t VP_MOVE_E2 = 0x2114;//constexpr uint16\u t VP\u MOVE\u E2=0x2114；
//constexpr uint16_t VP_MOVE_E3 = 0x2116;//constexpr uint16\u t VP\u MOVE\u E3=0x2116；
//constexpr uint16_t VP_MOVE_E4 = 0x2118;//constexpr uint16\u t VP\u MOVE\u E4=0x2118；
//constexpr uint16_t VP_MOVE_E5 = 0x211A;//constexpr uint16\u t VP\u MOVE\u E5=0x211A；
constexpr uint16_t VP_HOME_ALL = 0x2120;
constexpr uint16_t VP_MOTOR_LOCK_UNLOK = 0x2130;

// Power loss recovery//功率损耗恢复
constexpr uint16_t VP_POWER_LOSS_RECOVERY = 0x2180;

// Fan Control Buttons , switch between "off" and "on"//风扇控制按钮，在“关闭”和“打开”之间切换
constexpr uint16_t VP_FAN0_CONTROL = 0x2200;
constexpr uint16_t VP_FAN1_CONTROL = 0x2202;
//constexpr uint16_t VP_FAN2_CONTROL = 0x2204;//constexpr uint16\u t VP\u FAN2\u CONTROL=0x2204；
//constexpr uint16_t VP_FAN3_CONTROL = 0x2206;//constexpr uint16\u t VP\u FAN3\u CONTROL=0x2206；

// Heater Control Buttons , triged between "cool down" and "heat PLA" state//加热器控制按钮，在“冷却”和“加热”状态之间触发
constexpr uint16_t VP_E0_CONTROL = 0x2210;
constexpr uint16_t VP_E1_CONTROL = 0x2212;
//constexpr uint16_t VP_E2_CONTROL = 0x2214;//constexpr uint16\u t VP\u E2\u CONTROL=0x2214；
//constexpr uint16_t VP_E3_CONTROL = 0x2216;//constexpr uint16\u t VP\u E3\u CONTROL=0x2216；
//constexpr uint16_t VP_E4_CONTROL = 0x2218;//constexpr uint16\u t VP\u E4\u CONTROL=0x2218；
//constexpr uint16_t VP_E5_CONTROL = 0x221A;//constexpr uint16_t VP_E5_CONTROL=0x221A；
constexpr uint16_t VP_BED_CONTROL = 0x221C;

// Preheat//预热
constexpr uint16_t VP_E0_BED_PREHEAT = 0x2220;
constexpr uint16_t VP_E1_BED_CONTROL = 0x2222;
//constexpr uint16_t VP_E2_BED_CONTROL = 0x2224;//constexpr uint16\u t VP\u E2\u BED\u CONTROL=0x2224；
//constexpr uint16_t VP_E3_BED_CONTROL = 0x2226;//constexpr uint16\u t VP\u E3\u BED\u CONTROL=0x2226；
//constexpr uint16_t VP_E4_BED_CONTROL = 0x2228;//constexpr uint16\u t VP\u E4\u BED\u CONTROL=0x2228；
//constexpr uint16_t VP_E5_BED_CONTROL = 0x222A;//constexpr uint16\u t VP\u E5\u BED\u CONTROL=0x222A；

// Filament load and unload//灯丝装卸
constexpr uint16_t VP_E0_FILAMENT_LOAD_UNLOAD = 0x2300;
constexpr uint16_t VP_E1_FILAMENT_LOAD_UNLOAD = 0x2302;

// Settings store , reset//设置存储，重置
constexpr uint16_t VP_SETTINGS = 0x2400;

// PID autotune//PID自动调谐
constexpr uint16_t VP_PID_AUTOTUNE_E0 = 0x2410;
//constexpr uint16_t VP_PID_AUTOTUNE_E1 = 0x2412;//constexpr uint16\u t VP\u PID\u AUTOTUNE\u E1=0x2412；
//constexpr uint16_t VP_PID_AUTOTUNE_E2 = 0x2414;//constexpr uint16\u t VP\u PID\u AUTOTUNE\u E2=0x2414；
//constexpr uint16_t VP_PID_AUTOTUNE_E3 = 0x2416;//constexpr uint16\u t VP\u PID\u AUTOTUNE\u E3=0x2416；
//constexpr uint16_t VP_PID_AUTOTUNE_E4 = 0x2418;//constexpr uint16\u t VP\u PID\u AUTOTUNE\u E4=0x2418；
//constexpr uint16_t VP_PID_AUTOTUNE_E5 = 0x241A;//constexpr uint16\u t VP\u PID\u AUTOTUNE\u E5=0x241A；
constexpr uint16_t VP_PID_AUTOTUNE_BED = 0x2420;

// Firmware version on the boot screen.//启动屏幕上的固件版本。
constexpr uint16_t VP_MARLIN_VERSION = 0x3000;
constexpr uint8_t VP_MARLIN_VERSION_LEN = 16;   // there is more space on the display, if needed.//如果需要，显示屏上有更多空间。

// Place for status messages.//放置状态消息。
constexpr uint16_t VP_M117 = 0x3020;
constexpr uint8_t VP_M117_LEN = 0x20;

// Temperatures.//温度。
constexpr uint16_t VP_T_E0_Is = 0x3060;  // 4 Byte Integer//4字节整数
constexpr uint16_t VP_T_E0_Set = 0x3062; // 2 Byte Integer//2字节整数
constexpr uint16_t VP_T_E1_Is = 0x3064;  // 4 Byte Integer//4字节整数

// reserved to support up to 6 Extruders://预留可支持多达6台挤出机：
//constexpr uint16_t VP_T_E1_Set = 0x3066; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E1\u Set=0x3066；//2字节整数
//constexpr uint16_t VP_T_E2_Is = 0x3068;  // 4 Byte Integer//constexpr uint16\u t VP\u t\u E2\u Is=0x3068；//4字节整数
//constexpr uint16_t VP_T_E2_Set = 0x306A; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E2\u Set=0x306A；//2字节整数
//constexpr uint16_t VP_T_E3_Is = 0x306C;  // 4 Byte Integer//constexpr uint16\u t VP\u t\u E3\u Is=0x306C；//4字节整数
//constexpr uint16_t VP_T_E3_Set = 0x306E; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E3\u Set=0x306E；//2字节整数
//constexpr uint16_t VP_T_E4_Is = 0x3070;  // 4 Byte Integer//constexpr uint16\u t VP\u t\u E4\u Is=0x3070；//4字节整数
//constexpr uint16_t VP_T_E4_Set = 0x3072; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E4\u Set=0x3072；//2字节整数
//constexpr uint16_t VP_T_E4_Is = 0x3074;  // 4 Byte Integer//constexpr uint16\u t VP\u t\u E4\u Is=0x3074；//4字节整数
//constexpr uint16_t VP_T_E4_Set = 0x3076; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E4\u集=0x3076；//2字节整数
//constexpr uint16_t VP_T_E5_Is = 0x3078;  // 4 Byte Integer//constexpr uint16\u t VP\u t\u E5\u Is=0x3078；//4字节整数
//constexpr uint16_t VP_T_E5_Set = 0x307A; // 2 Byte Integer//constexpr uint16\u t VP\u t\u E5\u集=0x307A；//2字节整数

constexpr uint16_t VP_T_Bed_Is = 0x3080;  // 4 Byte Integer//4字节整数
constexpr uint16_t VP_T_Bed_Set = 0x3082; // 2 Byte Integer//2字节整数

constexpr uint16_t VP_Flowrate_E0 = 0x3090; // 2 Byte Integer//2字节整数
constexpr uint16_t VP_Flowrate_E1 = 0x3092; // 2 Byte Integer//2字节整数

// reserved for up to 6 Extruders://最多可用于6台挤出机：
//constexpr uint16_t VP_Flowrate_E2 = 0x3094;//constexpr uint16\u t VP\u流量=0x3094；
//constexpr uint16_t VP_Flowrate_E3 = 0x3096;//constexpr uint16\u t VP\u流量=0x3096；
//constexpr uint16_t VP_Flowrate_E4 = 0x3098;//constexpr uint16\u t VP\u流量=0x3098；
//constexpr uint16_t VP_Flowrate_E5 = 0x309A;//constexpr uint16\u t VP\u流量=0x309A；

constexpr uint16_t VP_Fan0_Percentage = 0x3100;  // 2 Byte Integer (0..100)//2字节整数（0..100）
constexpr uint16_t VP_Fan1_Percentage = 0x33A2;  // 2 Byte Integer (0..100)//2字节整数（0..100）
//constexpr uint16_t VP_Fan2_Percentage = 0x33A4;  // 2 Byte Integer (0..100)//constexpr uint16\u t VP\u Fan2\u百分比=0x33A4；//2字节整数（0..100）
//constexpr uint16_t VP_Fan3_Percentage = 0x33A6;  // 2 Byte Integer (0..100)//constexpr uint16\u t VP\u Fan3\u百分比=0x33A6；//2字节整数（0..100）

constexpr uint16_t VP_Feedrate_Percentage = 0x3102; // 2 Byte Integer (0..100)//2字节整数（0..100）
constexpr uint16_t VP_PrintProgress_Percentage = 0x3104; // 2 Byte Integer (0..100)//2字节整数（0..100）

constexpr uint16_t VP_PrintTime = 0x3106;
constexpr uint16_t VP_PrintTime_LEN = 10;

constexpr uint16_t VP_PrintAccTime = 0x3160;
constexpr uint16_t VP_PrintAccTime_LEN = 32;

constexpr uint16_t VP_PrintsTotal = 0x3180;
constexpr uint16_t VP_PrintsTotal_LEN = 16;

// Actual Position//实际位置
constexpr uint16_t VP_XPos = 0x3110;  // 4 Byte Fixed point number; format xxx.yy//4字节固定点号；格式xxx.yy
constexpr uint16_t VP_YPos = 0x3112;  // 4 Byte Fixed point number; format xxx.yy//4字节固定点号；格式xxx.yy
constexpr uint16_t VP_ZPos = 0x3114;  // 4 Byte Fixed point number; format xxx.yy//4字节固定点号；格式xxx.yy

constexpr uint16_t VP_EPos = 0x3120;  // 4 Byte Fixed point number; format xxx.yy//4字节固定点号；格式xxx.yy

// SDCard File Listing//SD卡文件列表
constexpr uint16_t VP_SD_FileName_LEN = 32; // LEN is shared for all entries.//所有条目都共享LEN。
constexpr uint16_t DGUS_SD_FILESPERSCREEN = 5; // FIXME move that info to the display and read it from there.//FIXME将该信息移动到显示屏上并从那里读取。
constexpr uint16_t VP_SD_FileName0 = 0x3200;
constexpr uint16_t VP_SD_FileName1 = 0x3220;
constexpr uint16_t VP_SD_FileName2 = 0x3240;
constexpr uint16_t VP_SD_FileName3 = 0x3260;
constexpr uint16_t VP_SD_FileName4 = 0x3280;

constexpr uint16_t VP_SD_Print_ProbeOffsetZ = 0x32A0; ////
constexpr uint16_t VP_SD_Print_Filename = 0x32C0; ////

// Fan status//风扇状态
constexpr uint16_t VP_FAN0_STATUS = 0x3300;
constexpr uint16_t VP_FAN1_STATUS = 0x3302;
//constexpr uint16_t VP_FAN2_STATUS = 0x3304;//constexpr uint16\u t VP\u FAN2\u STATUS=0x3304；
//constexpr uint16_t VP_FAN3_STATUS = 0x3306;//constexpr uint16\u t VP\u FAN3\u STATUS=0x3306；

// Heater status//加热器状态
constexpr uint16_t VP_E0_STATUS = 0x3310;
//constexpr uint16_t VP_E1_STATUS = 0x3312;//constexpr uint16\u t VP\u E1\u STATUS=0x3312；
//constexpr uint16_t VP_E2_STATUS = 0x3314;//constexpr uint16\u t VP\u E2\u STATUS=0x3314；
//constexpr uint16_t VP_E3_STATUS = 0x3316;//constexpr uint16\u t VP\u E3\u STATUS=0x3316；
//constexpr uint16_t VP_E4_STATUS = 0x3318;//constexpr uint16\u t VP\u E4\u STATUS=0x3318；
//constexpr uint16_t VP_E5_STATUS = 0x331A;//constexpr uint16_t VP_E5_STATUS=0x331A；
constexpr uint16_t VP_BED_STATUS = 0x331C;

constexpr uint16_t VP_MOVE_OPTION = 0x3400;

// Step per mm//每毫米步数
constexpr uint16_t VP_X_STEP_PER_MM = 0x3600; // at the moment , 2 byte unsigned int , 0~1638.4//目前，2字节无符号整数，0~1638.4
//constexpr uint16_t VP_X2_STEP_PER_MM = 0x3602;//constexpr uint16\u t VP\u X2\u步长每毫米=0x3602；
constexpr uint16_t VP_Y_STEP_PER_MM = 0x3604;
//constexpr uint16_t VP_Y2_STEP_PER_MM = 0x3606;//constexpr uint16 VP Y2步进每毫米=0x3606；
constexpr uint16_t VP_Z_STEP_PER_MM = 0x3608;
//constexpr uint16_t VP_Z2_STEP_PER_MM = 0x360A;//constexpr uint16_t VP_Z2_STEP_PER_MM=0x360A；
constexpr uint16_t VP_E0_STEP_PER_MM = 0x3610;
//constexpr uint16_t VP_E1_STEP_PER_MM = 0x3612;//constexpr uint16 VP E1每毫米阶梯=0x3612；
//constexpr uint16_t VP_E2_STEP_PER_MM = 0x3614;//constexpr uint16\u t VP\u E2\u步长每毫米=0x3614；
//constexpr uint16_t VP_E3_STEP_PER_MM = 0x3616;//constexpr uint16\u t VP\u E3\u步长每毫米=0x3616；
//constexpr uint16_t VP_E4_STEP_PER_MM = 0x3618;//constexpr uint16\u t VP\u E4\u步长每毫米=0x3618；
//constexpr uint16_t VP_E5_STEP_PER_MM = 0x361A;//constexpr uint16 VP E5步进每毫米=0x361A；

// PIDs//皮德斯
constexpr uint16_t VP_E0_PID_P = 0x3700; // at the moment , 2 byte unsigned int , 0~1638.4//目前，2字节无符号整数，0~1638.4
constexpr uint16_t VP_E0_PID_I = 0x3702;
constexpr uint16_t VP_E0_PID_D = 0x3704;
constexpr uint16_t VP_BED_PID_P = 0x3710;
constexpr uint16_t VP_BED_PID_I = 0x3712;
constexpr uint16_t VP_BED_PID_D = 0x3714;

// Wating screen status//等待屏幕状态
constexpr uint16_t VP_WAITING_STATUS = 0x3800;

// SPs for certain variables...//某些变量的SPs。。。
// located at 0x5000 and up//位于0x5000及以上
// Not used yet!//还没用！
// This can be used e.g to make controls / data display invisible//这可用于例如使控件/数据显示不可见
constexpr uint16_t SP_T_E0_Is = 0x5000;
constexpr uint16_t SP_T_E0_Set = 0x5010;
constexpr uint16_t SP_T_E1_Is = 0x5020;
constexpr uint16_t SP_T_Bed_Is = 0x5030;
constexpr uint16_t SP_T_Bed_Set = 0x5040;
