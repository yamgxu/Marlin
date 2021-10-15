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
 * fwretract.h - Define firmware-based retraction interface
 */

#include "../inc/MarlinConfigPre.h"

typedef struct {
       float retract_length;                      // M207 S - G10 Retract length//M207 S-G10回缩长度
  feedRate_t retract_feedrate_mm_s;               // M207 F - G10 Retract feedrate//M207 F-G10回缩进给速度
       float retract_zraise,                      // M207 Z - G10 Retract hop size//M207 Z-G10缩回跃点尺寸
             retract_recover_extra;               // M208 S - G11 Recover length//M208 S-G11恢复长度
  feedRate_t retract_recover_feedrate_mm_s;       // M208 F - G11 Recover feedrate//M208 F-G11恢复进给速度
       float swap_retract_length,                 // M207 W - G10 Swap Retract length//M207 W-G10交换回缩长度
             swap_retract_recover_extra;          // M208 W - G11 Swap Recover length//M208 W-G11交换恢复长度
  feedRate_t swap_retract_recover_feedrate_mm_s;  // M208 R - G11 Swap Recover feedrate//M208 R-G11交换恢复进给速度
} fwretract_settings_t;

#if ENABLED(FWRETRACT)

class FWRetract {
private:
  #if HAS_MULTI_EXTRUDER
    static bool retracted_swap[EXTRUDERS];         // Which extruders are swap-retracted//哪些挤出机是换装缩回的
  #endif

public:
  static fwretract_settings_t settings;

  #if ENABLED(FWRETRACT_AUTORETRACT)
    static bool autoretract_enabled;               // M209 S - Autoretract switch//M209 S-自动复位开关
  #else
    static constexpr bool autoretract_enabled = false;
  #endif

  static bool retracted[EXTRUDERS];                // Which extruders are currently retracted//目前收回了哪些挤出机
  static float current_retract[EXTRUDERS],         // Retract value used by planner//收回计划器使用的值
               current_hop;                        // Hop value used by planner//规划器使用的跃点值

  FWRetract() { reset(); }

  static void reset();

  static void refresh_autoretract() {
    LOOP_L_N(i, EXTRUDERS) retracted[i] = false;
  }

  static void enable_autoretract(const bool enable) {
    #if ENABLED(FWRETRACT_AUTORETRACT)
      autoretract_enabled = enable;
      refresh_autoretract();
    #endif
  }

  static void retract(const bool retracting OPTARG(HAS_MULTI_EXTRUDER, bool swapping = false));

  static void M207();
  static void M207_report(const bool forReplay=false);
  static void M208();
  static void M208_report(const bool forReplay=false);
  #if ENABLED(FWRETRACT_AUTORETRACT)
    static void M209();
    static void M209_report(const bool forReplay=false);
  #endif
};

extern FWRetract fwretract;

#endif // FWRETRACT//收回
