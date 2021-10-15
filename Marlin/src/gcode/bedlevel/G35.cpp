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

#if ENABLED(ASSISTED_TRAMMING)

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

////
// Define tramming point names.//定义电车点名称。
////

#include "../../feature/tramming.h"

/**
 * G35: Read bed corners to help adjust bed screws
 *
 *   S<screw_thread>
 *
 * Screw thread: 30 - Clockwise M3
 *               31 - Counter-Clockwise M3
 *               40 - Clockwise M4
 *               41 - Counter-Clockwise M4
 *               50 - Clockwise M5
 *               51 - Counter-Clockwise M5
 **/
void GcodeSuite::G35() {
  DEBUG_SECTION(log_G35, "G35", DEBUGGING(LEVELING));

  if (DEBUGGING(LEVELING)) log_machine_info();

  float z_measured[G35_PROBE_COUNT] = { 0 };

  const uint8_t screw_thread = parser.byteval('S', TRAMMING_SCREW_THREAD);
  if (!WITHIN(screw_thread, 30, 51) || screw_thread % 10 > 1) {
    SERIAL_ECHOLNPGM("?(S)crew thread must be 30, 31, 40, 41, 50, or 51.");
    return;
  }

  // Wait for planner moves to finish!//等待计划者移动完成！
  planner.synchronize();

  // Disable the leveling matrix before auto-aligning//自动对齐前禁用调平矩阵
  #if HAS_LEVELING
    #if ENABLED(RESTORE_LEVELING_AFTER_G35)
      const bool leveling_was_active = planner.leveling_active;
    #endif
    set_bed_leveling_enabled(false);
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif

  // Always home with tool 0 active//始终在工具0处于活动状态时返回原位
  #if HAS_MULTI_HOTEND
    const uint8_t old_tool_index = active_extruder;
    tool_change(0, true);
  #endif

  // Disable duplication mode on homing//禁用重设原点时的复制模式
  TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

  // Home only Z axis when X and Y is trusted, otherwise all axes, if needed before this procedure//当X和Y受信任时，仅原点为Z轴，否则为所有轴（如果在此过程之前需要）
  if (!all_axes_trusted()) process_subcommands_now_P(PSTR("G28Z"));

  bool err_break = false;

  // Probe all positions//探测所有位置
  LOOP_L_N(i, G35_PROBE_COUNT) {

    // In BLTOUCH HS mode, the probe travels in a deployed state.//在BLTOUCH HS模式下，探头以展开状态移动。
    // Users of G35 might have a badly misaligned bed, so raise Z by the//G35的用户可能有一个严重错位的床，所以将Z调高
    // length of the deployed pin (BLTOUCH stroke < 7mm)//展开销的长度（BLTOUCH行程<7mm）
    do_blocking_move_to_z(SUM_TERN(BLTOUCH_HS_MODE, Z_CLEARANCE_BETWEEN_PROBES, 7));
    const float z_probed_height = probe.probe_at_point(screws_tilt_adjust_pos[i], PROBE_PT_RAISE, 0, true);

    if (isnan(z_probed_height)) {
      SERIAL_ECHOPAIR("G35 failed at point ", i, " (");
      SERIAL_ECHOPGM_P((char *)pgm_read_ptr(&tramming_point_name[i]));
      SERIAL_CHAR(')');
      SERIAL_ECHOLNPAIR_P(SP_X_STR, screws_tilt_adjust_pos[i].x, SP_Y_STR, screws_tilt_adjust_pos[i].y);
      err_break = true;
      break;
    }

    if (DEBUGGING(LEVELING)) {
      DEBUG_ECHOPAIR("Probing point ", i, " (");
      DEBUG_ECHOPGM_P((char *)pgm_read_ptr(&tramming_point_name[i]));
      DEBUG_CHAR(')');
      DEBUG_ECHOLNPAIR_P(SP_X_STR, screws_tilt_adjust_pos[i].x, SP_Y_STR, screws_tilt_adjust_pos[i].y, SP_Z_STR, z_probed_height);
    }

    z_measured[i] = z_probed_height;
  }

  if (!err_break) {
    const float threads_factor[] = { 0.5, 0.7, 0.8 };

    // Calculate adjusts//计算调整
    LOOP_S_L_N(i, 1, G35_PROBE_COUNT) {
      const float diff = z_measured[0] - z_measured[i],
                  adjust = abs(diff) < 0.001f ? 0 : diff / threads_factor[(screw_thread - 30) / 10];

      const int full_turns = trunc(adjust);
      const float decimal_part = adjust - float(full_turns);
      const int minutes = trunc(decimal_part * 60.0f);

      SERIAL_ECHOPGM("Turn ");
      SERIAL_ECHOPGM_P((char *)pgm_read_ptr(&tramming_point_name[i]));
      SERIAL_ECHOPAIR(" ", (screw_thread & 1) == (adjust > 0) ? "CCW" : "CW", " by ", abs(full_turns), " turns");
      if (minutes) SERIAL_ECHOPAIR(" and ", abs(minutes), " minutes");
      if (ENABLED(REPORT_TRAMMING_MM)) SERIAL_ECHOPAIR(" (", -diff, "mm)");
      SERIAL_EOL();
    }
  }
  else
    SERIAL_ECHOLNPGM("G35 aborted.");

  // Restore the active tool after homing//归位后恢复激活的刀具
  #if HAS_MULTI_HOTEND
    tool_change(old_tool_index, DISABLED(PARKING_EXTRUDER)); // Fetch previous toolhead if not PARKING_EXTRUDER//如果挤出机未停机，取上一个工具头
  #endif

  #if BOTH(HAS_LEVELING, RESTORE_LEVELING_AFTER_G35)
    set_bed_leveling_enabled(leveling_was_active);
  #endif

  // Stow the probe, as the last call to probe.probe_at_point(...) left//收起探测器，作为对探测器的最后一次调用。探测器位于左（…）点
  // the probe deployed if it was successful.//如果成功，将部署探测器。
  probe.stow();

  move_to_tramming_wait_pos();

  // After this operation the Z position needs correction//此操作后，Z位置需要校正
  set_axis_never_homed(Z_AXIS);
}

#endif // ASSISTED_TRAMMING//辅助缆车
