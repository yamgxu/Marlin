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
 * feature/pause.h - Pause feature support functions
 * This may be combined with related G-codes if features are consolidated.
 */

typedef struct {
  float unload_length, load_length;
} fil_change_settings_t;

#include "../inc/MarlinConfigPre.h"

#if ENABLED(ADVANCED_PAUSE_FEATURE)

#include "../libs/nozzle.h"

enum PauseMode : char {
  PAUSE_MODE_SAME,
  PAUSE_MODE_PAUSE_PRINT,
  PAUSE_MODE_CHANGE_FILAMENT,
  PAUSE_MODE_LOAD_FILAMENT,
  PAUSE_MODE_UNLOAD_FILAMENT
};

enum PauseMessage : char {
  PAUSE_MESSAGE_PARKING,
  PAUSE_MESSAGE_CHANGING,
  PAUSE_MESSAGE_WAITING,
  PAUSE_MESSAGE_UNLOAD,
  PAUSE_MESSAGE_INSERT,
  PAUSE_MESSAGE_LOAD,
  PAUSE_MESSAGE_PURGE,
  PAUSE_MESSAGE_OPTION,
  PAUSE_MESSAGE_RESUME,
  PAUSE_MESSAGE_STATUS,
  PAUSE_MESSAGE_HEAT,
  PAUSE_MESSAGE_HEATING
};

#if M600_PURGE_MORE_RESUMABLE
  enum PauseMenuResponse : char {
    PAUSE_RESPONSE_WAIT_FOR,
    PAUSE_RESPONSE_EXTRUDE_MORE,
    PAUSE_RESPONSE_RESUME_PRINT
  };
  extern PauseMenuResponse pause_menu_response;
  extern PauseMode pause_mode;
#endif

extern fil_change_settings_t fc_settings[EXTRUDERS];

extern uint8_t did_pause_print;

#if ENABLED(DUAL_X_CARRIAGE)
  #define DXC_PARAMS , const int8_t DXC_ext=-1
  #define DXC_ARGS   , const int8_t DXC_ext
  #define DXC_PASS   , DXC_ext
  #define DXC_SAY    , " dxc:", int(DXC_ext)
#else
  #define DXC_PARAMS
  #define DXC_ARGS
  #define DXC_PASS
  #define DXC_SAY
#endif

// Pause the print. If unload_length is set, do a Filament Unload//暂停打印。如果设置了卸载长度，则进行灯丝卸载
bool pause_print(
  const_float_t   retract,                                    // (mm) Retraction length//（mm）回缩长度
  const xyz_pos_t &park_point,                                // Parking XY Position and Z Raise//驻车XY位置和Z升起
  const bool      show_lcd=false,                             // Set LCD status messages?//设置LCD状态信息？
  const_float_t   unload_length=0                             // (mm) Filament Change Unload Length - 0 to skip//（mm）灯丝更换卸载长度-0跳过
  DXC_PARAMS                                                  // Dual-X-Carriage extruder index//双X机架挤出机指数
);

void wait_for_confirmation(
  const bool      is_reload=false,                            // Reload Filament? (otherwise Resume Print)//重新装填灯丝？（否则继续打印）
  const int8_t    max_beep_count=0                            // Beep alert for attention//发出嘟嘟声提醒注意
  DXC_PARAMS                                                  // Dual-X-Carriage extruder index//双X机架挤出机指数
);

void resume_print(
  const_float_t   slow_load_length=0,                         // (mm) Slow Load Length for finishing move//（mm）终饰移动的慢速负载长度
  const_float_t   fast_load_length=0,                         // (mm) Fast Load Length for initial move//（mm）初始移动的快速负载长度
  const_float_t   extrude_length=ADVANCED_PAUSE_PURGE_LENGTH, // (mm) Purge length//（mm）吹扫长度
  const int8_t    max_beep_count=0,                           // Beep alert for attention//发出嘟嘟声提醒注意
  const celsius_t targetTemp=0                                // (°C) A target temperature for the hotend//（°C）热端的目标温度
  DXC_PARAMS                                                  // Dual-X-Carriage extruder index//双X机架挤出机指数
);

bool load_filament(
  const_float_t   slow_load_length=0,                         // (mm) Slow Load Length for finishing move//（mm）终饰移动的慢速负载长度
  const_float_t   fast_load_length=0,                         // (mm) Fast Load Length for initial move//（mm）初始移动的快速负载长度
  const_float_t   extrude_length=0,                           // (mm) Purge length//（mm）吹扫长度
  const int8_t    max_beep_count=0,                           // Beep alert for attention//发出嘟嘟声提醒注意
  const bool      show_lcd=false,                             // Set LCD status messages?//设置LCD状态信息？
  const bool      pause_for_user=false,                       // Pause for user before returning?//用户返回前是否暂停？
  const PauseMode mode=PAUSE_MODE_PAUSE_PRINT                 // Pause Mode to apply//暂停模式以应用
  DXC_PARAMS                                                  // Dual-X-Carriage extruder index//双X机架挤出机指数
);

bool unload_filament(
  const_float_t   unload_length,                              // (mm) Filament Unload Length - 0 to skip//（mm）灯丝卸载长度-0跳过
  const bool      show_lcd=false,                             // Set LCD status messages?//设置LCD状态信息？
  const PauseMode mode=PAUSE_MODE_PAUSE_PRINT                 // Pause Mode to apply//暂停模式以应用
  #if BOTH(FILAMENT_UNLOAD_ALL_EXTRUDERS, MIXING_EXTRUDER)
    , const_float_t mix_multiplier=1.0f                       // Extrusion multiplier (for a Mixing Extruder)//挤出倍增器（用于混合挤出机）
  #endif
);

#else // !ADVANCED_PAUSE_FEATURE// !高级暂停功能

  constexpr uint8_t did_pause_print = 0;

#endif // !ADVANCED_PAUSE_FEATURE// !高级暂停功能
