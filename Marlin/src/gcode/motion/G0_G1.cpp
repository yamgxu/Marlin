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

#include "../gcode.h"
#include "../../module/motion.h"

#include "../../MarlinCore.h"

#if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)
  #include "../../feature/fwretract.h"
#endif

#include "../../sd/cardreader.h"

#if ENABLED(NANODLP_Z_SYNC)
  #include "../../module/stepper.h"
#endif

extern xyze_pos_t destination;

#if ENABLED(VARIABLE_G0_FEEDRATE)
  feedRate_t fast_move_feedrate = MMM_TO_MMS(G0_FEEDRATE);
#endif

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
void GcodeSuite::G0_G1(TERN_(HAS_FAST_MOVES, const bool fast_move/*=false*/)) {

  if (IsRunning()
    #if ENABLED(NO_MOTION_BEFORE_HOMING)
      && !homing_needed_error(
        LINEAR_AXIS_GANG(
            (parser.seen_test('X') ? _BV(X_AXIS) : 0),
          | (parser.seen_test('Y') ? _BV(Y_AXIS) : 0),
          | (parser.seen_test('Z') ? _BV(Z_AXIS) : 0),
          | (parser.seen_test(AXIS4_NAME) ? _BV(I_AXIS) : 0),
          | (parser.seen_test(AXIS5_NAME) ? _BV(J_AXIS) : 0),
          | (parser.seen_test(AXIS6_NAME) ? _BV(K_AXIS) : 0))
      )
    #endif
  ) {
    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

    #ifdef G0_FEEDRATE
      feedRate_t old_feedrate;
      #if ENABLED(VARIABLE_G0_FEEDRATE)
        if (fast_move) {
          old_feedrate = feedrate_mm_s;             // Back up the (old) motion mode feedrate//备份（旧）运动模式进给速率
          feedrate_mm_s = fast_move_feedrate;       // Get G0 feedrate from last usage//从上次使用中获取G0进给率
        }
      #endif
    #endif

    get_destination_from_command();                 // Get X Y Z E F (and set cutter power)//获取X Y Z E F（并设置刀具功率）

    #ifdef G0_FEEDRATE
      if (fast_move) {
        #if ENABLED(VARIABLE_G0_FEEDRATE)
          fast_move_feedrate = feedrate_mm_s;       // Save feedrate for the next G0//保存下一个G0的进给速度
        #else
          old_feedrate = feedrate_mm_s;             // Back up the (new) motion mode feedrate//备份（新）运动模式进给速率
          feedrate_mm_s = MMM_TO_MMS(G0_FEEDRATE);  // Get the fixed G0 feedrate//获得固定的G0进给速度
        #endif
      }
    #endif

    #if BOTH(FWRETRACT, FWRETRACT_AUTORETRACT)

      if (MIN_AUTORETRACT <= MAX_AUTORETRACT) {
        // When M209 Autoretract is enabled, convert E-only moves to firmware retract/recover moves//启用M209 Autoretract后，将仅电子移动转换为固件收回/恢复移动
        if (fwretract.autoretract_enabled && parser.seen_test('E')
          && !parser.seen(LINEAR_AXIS_GANG("X", "Y", "Z", AXIS4_STR, AXIS5_STR, AXIS6_STR))
        ) {
          const float echange = destination.e - current_position.e;
          // Is this a retract or recover move?//这是收回还是收回？
          if (WITHIN(ABS(echange), MIN_AUTORETRACT, MAX_AUTORETRACT) && fwretract.retracted[active_extruder] == (echange > 0.0)) {
            current_position.e = destination.e;       // Hide a G1-based retract/recover from calculations//从计算中隐藏基于G1的收回/恢复
            sync_plan_position_e();                   // AND from the planner//从策划者那里
            return fwretract.retract(echange < 0.0);  // Firmware-based retract/recover (double-retract ignored)//基于固件的收回/恢复（忽略双收回）
          }
        }
      }

    #endif // FWRETRACT//收回

    #if IS_SCARA
      fast_move ? prepare_fast_move_to_destination() : prepare_line_to_destination();
    #else
      prepare_line_to_destination();
    #endif

    #ifdef G0_FEEDRATE
      // Restore the motion mode feedrate//恢复运动模式进给速度
      if (fast_move) feedrate_mm_s = old_feedrate;
    #endif

    #if ENABLED(NANODLP_Z_SYNC)
      #if ENABLED(NANODLP_ALL_AXIS)
        #define _MOVE_SYNC parser.seenval('X') || parser.seenval('Y') || parser.seenval('Z')  // For any move wait and output sync message//对于任何移动，请等待并输出同步消息
      #else
        #define _MOVE_SYNC parser.seenval('Z')  // Only for Z move//仅限Z移动
      #endif
      if (_MOVE_SYNC) {
        planner.synchronize();
        SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);
      }
      TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
    #else
      TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
    #endif
  }
}
