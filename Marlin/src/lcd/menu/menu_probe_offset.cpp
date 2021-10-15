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

////
// Calibrate Probe offset menu.//校准探头偏移菜单。
////

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(PROBE_OFFSET_WIZARD)

#include "menu_item.h"
#include "menu_addon.h"
#include "../../gcode/queue.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/probe.h"

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

// Global storage//全局存储
float z_offset_backup, calculated_z_offset, z_offset_ref;

#if HAS_LEVELING
  bool leveling_was_active;
#endif

inline void z_clearance_move() {
  do_z_clearance(
    #ifdef Z_AFTER_HOMING
      Z_AFTER_HOMING
    #elif defined(Z_HOMING_HEIGHT)
      Z_HOMING_HEIGHT
    #else
      10
    #endif
  );
}

void set_offset_and_go_back(const_float_t z) {
  probe.offset.z = z;
  SET_SOFT_ENDSTOP_LOOSE(false);
  TERN_(HAS_LEVELING, set_bed_leveling_enabled(leveling_was_active));
  ui.goto_previous_screen_no_defer();
}

void _goto_manual_move_z(const_float_t scale) {
  ui.manual_move.menu_scale = scale;
  ui.goto_screen(lcd_move_z);
}

void probe_offset_wizard_menu() {
  START_MENU();
  calculated_z_offset = probe.offset.z + current_position.z - z_offset_ref;

  if (LCD_HEIGHT >= 4)
    STATIC_ITEM(MSG_MOVE_NOZZLE_TO_BED, SS_CENTER|SS_INVERT);

  STATIC_ITEM_P(PSTR("Z="), SS_CENTER, ftostr42_52(current_position.z));
  STATIC_ITEM(MSG_ZPROBE_ZOFFSET, SS_LEFT, ftostr42_52(calculated_z_offset));

  SUBMENU(MSG_MOVE_1MM,  []{ _goto_manual_move_z( 1);    });
  SUBMENU(MSG_MOVE_01MM, []{ _goto_manual_move_z( 0.1f); });

  if ((FINE_MANUAL_MOVE) > 0.0f && (FINE_MANUAL_MOVE) < 0.1f) {
    char tmp[20], numstr[10];
    // Determine digits needed right of decimal//确定小数点右边所需的位数
    const uint8_t digs = !UNEAR_ZERO((FINE_MANUAL_MOVE) * 1000 - int((FINE_MANUAL_MOVE) * 1000)) ? 4 :
                         !UNEAR_ZERO((FINE_MANUAL_MOVE) *  100 - int((FINE_MANUAL_MOVE) *  100)) ? 3 : 2;
    sprintf_P(tmp, GET_TEXT(MSG_MOVE_N_MM), dtostrf(FINE_MANUAL_MOVE, 1, digs, numstr));
    #if DISABLED(HAS_GRAPHICAL_TFT)
      SUBMENU_P(NUL_STR, []{ _goto_manual_move_z(float(FINE_MANUAL_MOVE)); });
      MENU_ITEM_ADDON_START(0 + ENABLED(HAS_MARLINUI_HD44780));
      lcd_put_u8str(tmp);
      MENU_ITEM_ADDON_END();
    #else
      SUBMENU_P(tmp, []{ _goto_manual_move_z(float(FINE_MANUAL_MOVE)); });
    #endif
  }

  ACTION_ITEM(MSG_BUTTON_DONE, []{
    set_offset_and_go_back(calculated_z_offset);
    current_position.z = z_offset_ref;  // Set Z to z_offset_ref, as we can expect it is at probe height//将Z设置为Z_offset_ref，因为我们可以预期它位于探头高度
    sync_plan_position();
    z_clearance_move();                 // Raise Z as if it was homed//把Z调高，就好像它是原点一样
  });

  ACTION_ITEM(MSG_BUTTON_CANCEL, []{
    set_offset_and_go_back(z_offset_backup);
    // If wizard-homing was done by probe with PROBE_OFFSET_WIZARD_START_Z//如果向导归位是由带探测器的探测器完成的，则为探测器偏移量向导开始
    #if HOMING_Z_WITH_PROBE && defined(PROBE_OFFSET_WIZARD_START_Z)
      set_axis_never_homed(Z_AXIS); // On cancel the Z position needs correction//取消时，Z位置需要校正
      queue.inject_P(PSTR("G28Z"));
    #else // Otherwise do a Z clearance move like after Homing//否则，Z间隙会像归位后那样移动吗
      z_clearance_move();
    #endif
  });

  END_MENU();
}

void prepare_for_probe_offset_wizard() {
  #if defined(PROBE_OFFSET_WIZARD_XY_POS) || !HOMING_Z_WITH_PROBE
    if (ui.should_draw()) MenuItem_static::draw(1, GET_TEXT(MSG_PROBE_WIZARD_PROBING));

    if (ui.wait_for_move) return;

    #ifndef PROBE_OFFSET_WIZARD_XY_POS
      #define PROBE_OFFSET_WIZARD_XY_POS XY_CENTER
    #endif
    // Get X and Y from configuration, or use center//从配置中获取X和Y，或使用中心
    constexpr xy_pos_t wizard_pos = PROBE_OFFSET_WIZARD_XY_POS;

    // Probe for Z reference//Z参考探针
    ui.wait_for_move = true;
    z_offset_ref = probe.probe_at_point(wizard_pos, PROBE_PT_RAISE, 0, true);
    ui.wait_for_move = false;

    // Stow the probe, as the last call to probe.probe_at_point(...) left//收起探测器，作为对探测器的最后一次调用。探测器位于左（…）点
    // the probe deployed if it was successful.//如果成功，将部署探测器。
    probe.stow();
  #else
    if (ui.wait_for_move) return;
  #endif

  // Move Nozzle to Probing/Homing Position//将喷嘴移至探测/归位位置
  ui.wait_for_move = true;
  current_position += probe.offset_xy;
  line_to_current_position(MMM_TO_MMS(XY_PROBE_FEEDRATE));
  ui.synchronize(GET_TEXT(MSG_PROBE_WIZARD_MOVING));
  ui.wait_for_move = false;

  SET_SOFT_ENDSTOP_LOOSE(true); // Disable soft endstops for free Z movement//禁用软止动块以实现自由Z向移动

  // Go to Calibration Menu//进入校准菜单
  ui.goto_screen(probe_offset_wizard_menu);
  ui.defer_status_screen();
}

void goto_probe_offset_wizard() {
  ui.defer_status_screen();
  set_all_unhomed();

  // Store probe.offset.z for Case: Cancel//案例的存储探针.offset.z:取消
  z_offset_backup = probe.offset.z;

  #ifdef PROBE_OFFSET_WIZARD_START_Z
    probe.offset.z = PROBE_OFFSET_WIZARD_START_Z;
  #endif

  // Store Bed-Leveling-State and disable//存储床位调平状态并禁用
  #if HAS_LEVELING
    leveling_was_active = planner.leveling_active;
    set_bed_leveling_enabled(false);
  #endif

  // Home all axes//所有轴的原点
  queue.inject_P(G28_STR);

  ui.goto_screen([]{
    _lcd_draw_homing();
    if (all_axes_homed()) {
      z_offset_ref = 0;             // Set Z Value for Wizard Position to 0//将向导位置的Z值设置为0
      ui.goto_screen(prepare_for_probe_offset_wizard);
      ui.defer_status_screen();
    }
  });

}

#endif // PROBE_OFFSET_WIZARD//探测偏移向导
