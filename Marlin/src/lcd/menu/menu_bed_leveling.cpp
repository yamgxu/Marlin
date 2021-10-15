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
// Bed Leveling Menus//床位调整菜单
////

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(LCD_BED_LEVELING)

#include "menu_item.h"
#include "../../module/planner.h"
#include "../../feature/bedlevel/bedlevel.h"

#if HAS_BED_PROBE && DISABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "../../module/probe.h"
#endif

#if HAS_GRAPHICAL_TFT
  #include "../tft/tft.h"
  #if ENABLED(TOUCH_SCREEN)
    #include "../tft/touch.h"
  #endif
#endif

#if EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)

  #include "../../module/motion.h"
  #include "../../gcode/queue.h"

  ////
  // Motion > Level Bed handlers//运动>水平床处理器
  ////

  static uint8_t manual_probe_index;

  // LCD probed points are from defaults//LCD探测点来自默认值
  constexpr uint8_t total_probe_points = TERN(AUTO_BED_LEVELING_3POINT, 3, GRID_MAX_POINTS);

  ////
  // Bed leveling is done. Wait for G29 to complete.//河床平整完成。等待G29完成。
  // A flag is used so that this can release control//使用一个标志，这样可以释放控制
  // and allow the command queue to be processed.//并允许处理命令队列。
  ////
  // When G29 finishes the last move://G29完成最后一步时：
  // - Raise Z to the "Z after probing" height//-将Z提升至“探测后Z”高度
  // - Don't return until done.//-完成之前不要返回。
  ////
  // ** This blocks the command queue! **//**这将阻止命令队列**
  ////
  void _lcd_level_bed_done() {
    if (!ui.wait_for_move) {
      #if Z_AFTER_PROBING > 0 && DISABLED(MESH_BED_LEVELING)
        // Display "Done" screen and wait for moves to complete//显示“完成”屏幕并等待移动完成
        line_to_z(Z_AFTER_PROBING);
        ui.synchronize(GET_TEXT(MSG_LEVEL_BED_DONE));
      #endif
      ui.goto_previous_screen_no_defer();
      ui.completion_feedback();
    }
    if (ui.should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4, GET_TEXT(MSG_LEVEL_BED_DONE));
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }

  void _lcd_level_goto_next_point();

  ////
  // Step 7: Get the Z coordinate, click goes to the next point or exits//步骤7：获取Z坐标，单击转到下一点或退出
  ////
  void _lcd_level_bed_get_z() {

    if (ui.use_click()) {

      ////
      // Save the current Z position and move//保存当前Z位置并移动
      ////

      // If done...//如果完成。。。
      if (++manual_probe_index >= total_probe_points) {
        ////
        // The last G29 records the point and enables bed leveling//最后一个G29记录该点并启用河床平整
        ////
        ui.wait_for_move = true;
        ui.goto_screen(_lcd_level_bed_done);
        #if ENABLED(MESH_BED_LEVELING)
          queue.inject_P(PSTR("G29S2"));
        #elif ENABLED(PROBE_MANUALLY)
          queue.inject_P(PSTR("G29V1"));
        #endif
      }
      else
        _lcd_level_goto_next_point();

      return;
    }

    ////
    // Encoder knob or keypad buttons adjust the Z position//编码器旋钮或键盘按钮可调整Z位置
    ////
    if (ui.encoderPosition) {
      const float z = current_position.z + float(int32_t(ui.encoderPosition)) * (MESH_EDIT_Z_STEP);
      line_to_z(constrain(z, -(LCD_PROBE_Z_RANGE) * 0.5f, (LCD_PROBE_Z_RANGE) * 0.5f));
      ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      ui.encoderPosition = 0;
    }

    ////
    // Draw on first display, then only on Z change//在第一个显示上绘制，然后仅在Z更改上绘制
    ////
    if (ui.should_draw()) {
      const float v = current_position.z;
      MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_MOVE_Z), ftostr43sign(v + (v < 0 ? -0.0001f : 0.0001f), '+'));
    }
  }

  ////
  // Step 6: Display "Next point: 1 / 9" while waiting for move to finish//步骤6：在等待移动完成时显示“下一点：1/9”
  ////
  void _lcd_level_bed_moving() {
    if (ui.should_draw()) {
      char msg[10];
      sprintf_P(msg, PSTR("%i / %u"), int(manual_probe_index + 1), total_probe_points);
      MenuEditItemBase::draw_edit_screen(GET_TEXT(MSG_LEVEL_BED_NEXT_POINT), msg);
    }
    ui.refresh(LCDVIEW_CALL_NO_REDRAW);
    if (!ui.wait_for_move) ui.goto_screen(_lcd_level_bed_get_z);
  }

  ////
  // Step 5: Initiate a move to the next point//步骤5：开始移动到下一点
  ////
  void _lcd_level_goto_next_point() {
    ui.goto_screen(_lcd_level_bed_moving);

    // G29 Records Z, moves, and signals when it pauses//G29在暂停时记录Z、移动和信号
    ui.wait_for_move = true;
    #if ENABLED(MESH_BED_LEVELING)
      queue.inject_P(manual_probe_index ? PSTR("G29S2") : PSTR("G29S1"));
    #elif ENABLED(PROBE_MANUALLY)
      queue.inject_P(PSTR("G29V1"));
    #endif
  }

  ////
  // Step 4: Display "Click to Begin", wait for click//步骤4：显示“单击开始”，等待单击
  //         Move to the first probe position//移动到第一个探针位置
  ////
  void _lcd_level_bed_homing_done() {
    if (ui.should_draw()) {
      MenuItem_static::draw(1, GET_TEXT(MSG_LEVEL_BED_WAITING));
      // Color UI needs a control to detect a touch//彩色用户界面需要一个控件来检测触摸
      #if BOTH(TOUCH_SCREEN, HAS_GRAPHICAL_TFT)
        touch.add_control(CLICK, 0, 0, TFT_WIDTH, TFT_HEIGHT);
      #endif
    }
    if (ui.use_click()) {
      manual_probe_index = 0;
      _lcd_level_goto_next_point();
    }
  }

  ////
  // Step 3: Display "Homing XYZ" - Wait for homing to finish//步骤3：显示“归位XYZ”-等待归位完成
  ////
  void _lcd_level_bed_homing() {
    _lcd_draw_homing();
    if (all_axes_homed()) ui.goto_screen(_lcd_level_bed_homing_done);
  }

  #if ENABLED(PROBE_MANUALLY)
    extern bool g29_in_progress;
  #endif

  ////
  // Step 2: Continue Bed Leveling...//第2步：继续床位平整。。。
  ////
  void _lcd_level_bed_continue() {
    ui.defer_status_screen();
    set_all_unhomed();
    ui.goto_screen(_lcd_level_bed_homing);
    queue.inject_P(G28_STR);
  }

#endif // PROBE_MANUALLY || MESH_BED_LEVELING//探头| | |网|床|找平

#if ENABLED(MESH_EDIT_MENU)

  inline void refresh_planner() {
    set_current_from_steppers_for_axis(ALL_AXES_ENUM);
    sync_plan_position();
  }

  void menu_edit_mesh() {
    static uint8_t xind, yind; // =0// =0
    START_MENU();
    BACK_ITEM(MSG_BED_LEVELING);
    EDIT_ITEM(uint8, MSG_MESH_X, &xind, 0, (GRID_MAX_POINTS_X) - 1);
    EDIT_ITEM(uint8, MSG_MESH_Y, &yind, 0, (GRID_MAX_POINTS_Y) - 1);
    EDIT_ITEM_FAST(float43, MSG_MESH_EDIT_Z, &Z_VALUES(xind, yind), -(LCD_PROBE_Z_RANGE) * 0.5, (LCD_PROBE_Z_RANGE) * 0.5, refresh_planner);
    END_MENU();
  }

#endif // MESH_EDIT_MENU//网格编辑菜单

/**
 * Step 1: Bed Level entry-point
 *
 * << Motion
 *    Auto Home           (if homing needed)
 *    Leveling On/Off     (if data exists, and homed)
 *    Fade Height: ---    (Req: ENABLE_LEVELING_FADE_HEIGHT)
 *    Mesh Z Offset: ---  (Req: MESH_BED_LEVELING)
 *    Z Probe Offset: --- (Req: HAS_BED_PROBE, Opt: BABYSTEP_ZPROBE_OFFSET)
 *    Level Bed >
 *    Level Corners >     (if homed)
 *    Load Settings       (Req: EEPROM_SETTINGS)
 *    Save Settings       (Req: EEPROM_SETTINGS)
 */
void menu_bed_leveling() {
  const bool is_homed = all_axes_trusted(),
             is_valid = leveling_is_valid();

  START_MENU();
  BACK_ITEM(MSG_MOTION);

  // Auto Home if not using manual probing//如果不使用手动探测，则自动回家
  #if NONE(PROBE_MANUALLY, MESH_BED_LEVELING)
    if (!is_homed) GCODES_ITEM(MSG_AUTO_HOME, G28_STR);
  #endif

  // Level Bed//水平床
  #if EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)
    // Manual leveling uses a guided procedure//手动调平使用引导程序
    SUBMENU(MSG_LEVEL_BED, _lcd_level_bed_continue);
  #else
    // Automatic leveling can just run the G-code//自动调平可以只运行G代码
    GCODES_ITEM(MSG_LEVEL_BED, is_homed ? PSTR("G29") : PSTR("G29N"));
  #endif

  #if ENABLED(MESH_EDIT_MENU)
    if (is_valid) SUBMENU(MSG_EDIT_MESH, menu_edit_mesh);
  #endif

  // Homed and leveling is valid? Then leveling can be toggled.//原点和水平是否有效？然后可以切换调平。
  if (is_homed && is_valid) {
    bool show_state = planner.leveling_active;
    EDIT_ITEM(bool, MSG_BED_LEVELING, &show_state, _lcd_toggle_bed_leveling);
  }

  // Z Fade Height//Z衰减高度
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    // Shadow for editing the fade height//用于编辑淡入高度的阴影
    editable.decimal = planner.z_fade_height;
    EDIT_ITEM_FAST(float3, MSG_Z_FADE_HEIGHT, &editable.decimal, 0, 100, []{ set_z_fade_height(editable.decimal); });
  #endif

  ////
  // Mesh Bed Leveling Z-Offset//网床找平Z向偏移
  ////
  #if ENABLED(MESH_BED_LEVELING)
    EDIT_ITEM(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
  #endif

  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    SUBMENU(MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
  #elif HAS_BED_PROBE
    EDIT_ITEM(LCD_Z_OFFSET_TYPE, MSG_ZPROBE_ZOFFSET, &probe.offset.z, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
  #endif

  #if ENABLED(LEVEL_BED_CORNERS)
    SUBMENU(MSG_LEVEL_CORNERS, _lcd_level_bed_corners);
  #endif

  #if ENABLED(EEPROM_SETTINGS)
    ACTION_ITEM(MSG_LOAD_EEPROM, ui.load_settings);
    ACTION_ITEM(MSG_STORE_EEPROM, ui.store_settings);
  #endif
  END_MENU();
}

#endif // LCD_BED_LEVELING//液晶显示器\u床\u调平
