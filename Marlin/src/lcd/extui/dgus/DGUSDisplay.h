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

/**
 * lcd/extui/dgus/DGUSDisplay.h
 */

#include "../../../inc/MarlinConfigPre.h"

#include <stdlib.h>    // size_t//尺寸

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif
#include "DGUSVPVariable.h"

enum DGUSLCD_Screens : uint8_t;

//#define DEBUG_DGUSLCD//#定义DEBUG_DGUSLCD
#define DEBUG_OUT ENABLED(DEBUG_DGUSLCD)
#include "../../../core/debug_out.h"

typedef enum : uint8_t {
  DGUS_IDLE,           //< waiting for DGUS_HEADER1.//<等待DGUS_负责人1。
  DGUS_HEADER1_SEEN,   //< DGUS_HEADER1 received//<DGUS_头1已收到
  DGUS_HEADER2_SEEN,   //< DGUS_HEADER2 received//<DGUS_头2已收到
  DGUS_WAIT_TELEGRAM,  //< LEN received, Waiting for to receive all bytes.//<LEN已接收，等待接收所有字节。
} rx_datagram_state_t;

// Low-Level access to the display.//显示器的低电平访问。
class DGUSDisplay {
public:

  DGUSDisplay() = default;

  static void InitDisplay();

  // Variable access.//可变访问。
  static void WriteVariable(uint16_t adr, const void *values, uint8_t valueslen, bool isstr=false);
  static void WriteVariablePGM(uint16_t adr, const void *values, uint8_t valueslen, bool isstr=false);
  static void WriteVariable(uint16_t adr, int16_t value);
  static void WriteVariable(uint16_t adr, uint16_t value);
  static void WriteVariable(uint16_t adr, uint8_t value);
  static void WriteVariable(uint16_t adr, int8_t value);
  static void WriteVariable(uint16_t adr, long value);
  static void MKS_WriteVariable(uint16_t adr, uint8_t value);


  // Utility functions for bridging ui_api and dbus//用于桥接ui_api和DBU的实用程序函数
  template<typename T, float(*Getter)(const T), T selector, typename WireType=uint16_t>
  static void SetVariable(DGUS_VP_Variable &var) {
    WriteVariable(var.VP, (WireType)Getter(selector));
  }

  template<typename T, void(*Setter)(const float V, const T), T selector>
  static void GetVariable(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t newvalue = swap16(*(uint16_t*)val_ptr);
    Setter(newvalue, selector);
  }

  // Until now I did not need to actively read from the display. That's why there is no ReadVariable//直到现在，我还不需要从显示器上主动阅读。这就是为什么没有ReadVariable
  // (I extensively use the auto upload of the display)//（我广泛使用显示器的自动上传功能）

  // Force display into another screen.//强制显示到另一个屏幕。
  // (And trigger update of containing VPs)//（并触发包含VPs的更新）
  // (to implement a pop up message, which may not be nested)//（实现一个弹出消息，该消息不能嵌套）
  static void RequestScreen(DGUSLCD_Screens screen);

  // Periodic tasks, eg. Rx-Queue handling.//定期任务，例如Rx队列处理。
  static void loop();

public:
  // Helper for users of this class to estimate if an interaction would be blocking.//帮助此类用户估计交互是否会阻塞。
  static size_t GetFreeTxBuffer();

  // Checks two things: Can we confirm the presence of the display and has we initiliazed it.//检查两件事：我们是否可以确认显示器的存在以及是否初始化了它。
  // (both boils down that the display answered to our chatting)//（两者都归结为显示器响应了我们的聊天）
  static inline bool isInitialized() { return Initialized; }

private:
  static void WriteHeader(uint16_t adr, uint8_t cmd, uint8_t payloadlen);
  static void WritePGM(const char str[], uint8_t len);
  static void ProcessRx();

  static inline uint16_t swap16(const uint16_t value) { return (value & 0xFFU) << 8U | (value >> 8U); }
  static rx_datagram_state_t rx_datagram_state;
  static uint8_t rx_datagram_len;
  static bool Initialized, no_reentrance;
};

#define GET_VARIABLE(f, t, V...) (&DGUSDisplay::GetVariable<decltype(t), f, t, ##V>)
#define SET_VARIABLE(f, t, V...) (&DGUSDisplay::SetVariable<decltype(t), f, t, ##V>)

extern DGUSDisplay dgusdisplay;

// compile-time x^y//编译时x^y
constexpr float cpow(const float x, const int y) { return y == 0 ? 1.0 : x * cpow(x, y - 1); }

/// Find the flash address of a DGUS_VP_Variable for the VP.///查找VP的DGUS_VP_变量的闪存地址。
const DGUS_VP_Variable* DGUSLCD_FindVPVar(const uint16_t vp);

/// Helper to populate a DGUS_VP_Variable for a given VP. Return false if not found.///帮助器为给定VP填充DGUS_VP_变量。如果未找到，则返回false。
bool populate_VPVar(const uint16_t VP, DGUS_VP_Variable * const ramcopy);
