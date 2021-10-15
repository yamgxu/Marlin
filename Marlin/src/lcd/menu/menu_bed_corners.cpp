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
// Level Bed Corners menu//水平床角落菜单
////

#include "../../inc/MarlinConfigPre.h"

#if BOTH(HAS_LCD_MENU, LEVEL_BED_CORNERS)

#include "menu_item.h"
#include "../../module/motion.h"
#include "../../module/planner.h"

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#ifndef LEVEL_CORNERS_Z_HOP
  #define LEVEL_CORNERS_Z_HOP 4.0
#endif
#ifndef LEVEL_CORNERS_HEIGHT
  #define LEVEL_CORNERS_HEIGHT 0.0
#endif

#if ENABLED(LEVEL_CORNERS_USE_PROBE)
  #include "../../module/probe.h"
  #include "../../module/endstops.h"
  #if ENABLED(BLTOUCH)
    #include "../../feature/bltouch.h"
  #endif
  #ifndef LEVEL_CORNERS_PROBE_TOLERANCE
    #define LEVEL_CORNERS_PROBE_TOLERANCE 0.2
  #endif
  float last_z;
  int good_points;
  bool corner_probing_done, wait_for_probe;

  #if HAS_MARLINUI_U8GLIB
    #include "../dogm/marlinui_DOGM.h"
  #endif
  #define GOOD_POINTS_TO_STR(N) ui8tostr2(N)
  #define LAST_Z_TO_STR(N) ftostr53_63(N) //ftostr42_52(N)//FTU 42_52（北）

#endif

static_assert(LEVEL_CORNERS_Z_HOP >= 0, "LEVEL_CORNERS_Z_HOP must be >= 0. Please update your configuration.");

#if HAS_LEVELING
  static bool leveling_was_active = false;
#endif

#ifndef LEVEL_CORNERS_LEVELING_ORDER
  #define LEVEL_CORNERS_LEVELING_ORDER { LF, RF, LB, RB } // Default//违约
  //#define LEVEL_CORNERS_LEVELING_ORDER { LF, LB, RF  }  // 3 hard-coded points//#定义标高角点标高顺序{LF，LB，RF}//3个硬编码点
  //#define LEVEL_CORNERS_LEVELING_ORDER { LF, RF }       // 3-Point tramming - Rear//#定义标高(转角)(调平)顺序{LF，RF}//3点轨道-后
  //#define LEVEL_CORNERS_LEVELING_ORDER { LF, LB }       // 3-Point tramming - Right//#定义标高\转角\找平\顺序{LF，LB}//3点轨道-右侧
  //#define LEVEL_CORNERS_LEVELING_ORDER { RF, RB }       // 3-Point tramming - Left//#定义标高\转角\找平\顺序{RF，RB}//3点轨道-左侧
  //#define LEVEL_CORNERS_LEVELING_ORDER { LB, RB }       // 3-Point tramming - Front//#定义标高_转角_调平_顺序{LB，RB}//3点轨道-前
#endif

#define LF 1
#define RF 2
#define RB 3
#define LB 4
constexpr int lco[] = LEVEL_CORNERS_LEVELING_ORDER;
constexpr bool level_corners_3_points = COUNT(lco) == 2;
static_assert(level_corners_3_points || COUNT(lco) == 4, "LEVEL_CORNERS_LEVELING_ORDER must have exactly 2 or 4 corners.");

constexpr int lcodiff = abs(lco[0] - lco[1]);
static_assert(COUNT(lco) == 4 || lcodiff == 1 || lcodiff == 3, "The first two LEVEL_CORNERS_LEVELING_ORDER corners must be on the same edge.");

constexpr int nr_edge_points = level_corners_3_points ? 3 : 4;
constexpr int available_points = nr_edge_points + ENABLED(LEVEL_CENTER_TOO);
constexpr int center_index = TERN(LEVEL_CENTER_TOO, available_points - 1, -1);
constexpr float inset_lfrb[4] = LEVEL_CORNERS_INSET_LFRB;
constexpr xy_pos_t lf { (X_MIN_BED) + inset_lfrb[0], (Y_MIN_BED) + inset_lfrb[1] },
                   rb { (X_MAX_BED) - inset_lfrb[2], (Y_MAX_BED) - inset_lfrb[3] };

static int8_t bed_corner;

/**
 * Select next corner coordinates
 */
static void _lcd_level_bed_corners_get_next_position() {

  if (level_corners_3_points) {
    if (bed_corner >= available_points) bed_corner = 0; // Above max position -> move back to first corner//高于最大位置->移回第一个角
    switch (bed_corner) {
      case 0 ... 1:
        // First two corners set explicitly by the configuration//前两个角由配置明确设置
        current_position = lf;                       // Left front//左前
        switch (lco[bed_corner]) {
          case RF: current_position.x = rb.x; break; // Right Front//右前方
          case RB: current_position   = rb;   break; // Right Back//右后卫
          case LB: current_position.y = rb.y; break; // Left Back//左后卫
        }
        break;

      case 2:
        // Determine which edge to probe for 3rd point//确定第三点要探测的边
        current_position.set(lf.x + (rb.x - lf.x) / 2, lf.y + (rb.y - lf.y) / 2);
        if ((lco[0] == LB && lco[1] == RB) || (lco[0] == RB && lco[1] == LB)) current_position.y = lf.y; // Front Center//前中心
        if ((lco[0] == LF && lco[1] == LB) || (lco[0] == LB && lco[1] == LF)) current_position.x = rb.x; // Center Right//中右翼
        if ((lco[0] == RF && lco[1] == RB) || (lco[0] == RB && lco[1] == RF)) current_position.x = lf.x; // Left Center//左中
        if ((lco[0] == LF && lco[1] == RF) || (lco[0] == RF && lco[1] == LF)) current_position.y = rb.y; // Center Back//中后卫
        #if DISABLED(LEVEL_CENTER_TOO) && ENABLED(LEVEL_CORNERS_USE_PROBE)
          bed_corner++;  // Must increment the count to ensure it resets the loop if the 3rd point is out of tolerance//必须增加计数，以确保在第三点超出公差时重置回路
        #endif
        break;

      #if ENABLED(LEVEL_CENTER_TOO)
        case 3:
          current_position.set(X_CENTER, Y_CENTER);
          break;
      #endif
    }
  }
  else {
    // Four-Corner Bed Tramming with optional center//带可选中心的四角床电车
    if (TERN0(LEVEL_CENTER_TOO, bed_corner == center_index)) {
      current_position.set(X_CENTER, Y_CENTER);
      TERN_(LEVEL_CORNERS_USE_PROBE, good_points--); // Decrement to allow one additional probe point//减小以允许一个额外的探测点
    }
    else {
      current_position = lf;                       // Left front//左前
      switch (lco[bed_corner]) {
        case RF: current_position.x = rb.x; break; // Right front//右前方
        case RB: current_position   = rb;   break; // Right rear//右后方
        case LB: current_position.y = rb.y; break; // Left rear//左后方
      }
    }
  }
}

/**
 * Level corners, starting in the front-left corner.
 */
#if ENABLED(LEVEL_CORNERS_USE_PROBE)

  #define VALIDATE_POINT(X, Y, STR) static_assert(Probe::build_time::can_reach((X), (Y)), \
    "LEVEL_CORNERS_INSET_LFRB " STR " inset is not reachable with the default NOZZLE_TO_PROBE offset and PROBING_MARGIN.")
  VALIDATE_POINT(lf.x, Y_CENTER, "left"); VALIDATE_POINT(X_CENTER, lf.y, "front");
  VALIDATE_POINT(rb.x, Y_CENTER, "right"); VALIDATE_POINT(X_CENTER, rb.y, "back");

  #ifndef PAGE_CONTAINS
    #define PAGE_CONTAINS(...) true
  #endif

  void _lcd_draw_probing() {
    if (!ui.should_draw()) return;

    TERN_(HAS_MARLINUI_U8GLIB, ui.set_font(FONT_MENU)); // Set up the font for extra info//为额外信息设置字体

    MenuItem_static::draw(0, GET_TEXT(MSG_PROBING_MESH), SS_INVERT); // "Probing Mesh" heading//“探测网格”标题

    uint8_t cy = TERN(TFT_COLOR_UI, 3, LCD_HEIGHT - 1), y = LCD_ROW_Y(cy);

    // Display # of good points found vs total needed//显示找到的好分数与需要的总分数之比
    if (PAGE_CONTAINS(y - (MENU_FONT_HEIGHT), y)) {
      SETCURSOR(TERN(TFT_COLOR_UI, 2, 0), cy);
      lcd_put_u8str_P(GET_TEXT(MSG_LEVEL_CORNERS_GOOD_POINTS));
      IF_ENABLED(TFT_COLOR_UI, lcd_moveto(12, cy));
      lcd_put_u8str(GOOD_POINTS_TO_STR(good_points));
      lcd_put_wchar('/');
      lcd_put_u8str(GOOD_POINTS_TO_STR(nr_edge_points));
    }

    --cy;
    y -= MENU_FONT_HEIGHT;

    // Display the Last Z value//显示最后一个Z值
    if (PAGE_CONTAINS(y - (MENU_FONT_HEIGHT), y)) {
      SETCURSOR(TERN(TFT_COLOR_UI, 2, 0), cy);
      lcd_put_u8str_P(GET_TEXT(MSG_LEVEL_CORNERS_LAST_Z));
      IF_ENABLED(TFT_COLOR_UI, lcd_moveto(12, 2));
      lcd_put_u8str(LAST_Z_TO_STR(last_z));
    }
  }

  void _lcd_draw_raise() {
    if (!ui.should_draw()) return;
    MenuItem_confirm::select_screen(
      GET_TEXT(MSG_BUTTON_DONE), GET_TEXT(MSG_BUTTON_SKIP)
      , []{ corner_probing_done = true; wait_for_probe = false; }
      , []{ wait_for_probe = false; }
      , GET_TEXT(MSG_LEVEL_CORNERS_RAISE)
      , (const char*)nullptr, NUL_STR
    );
  }

  void _lcd_draw_level_prompt() {
    if (!ui.should_draw()) return;
    MenuItem_confirm::confirm_screen(
      []{ queue.inject_P(TERN(HAS_LEVELING, PSTR("G29N"), G28_STR));
          ui.return_to_status();
      }
      , []{ ui.goto_previous_screen_no_defer(); }
      , GET_TEXT(MSG_LEVEL_CORNERS_IN_RANGE)
      , (const char*)nullptr, PSTR("?")
    );
  }

  bool _lcd_level_bed_corners_probe(bool verify=false) {
    if (verify) do_blocking_move_to_z(current_position.z + LEVEL_CORNERS_Z_HOP); // do clearance if needed//如有需要，进行清关
    TERN_(BLTOUCH_SLOW_MODE, bltouch.deploy()); // Deploy in LOW SPEED MODE on every probe action//在每次探测动作中以低速模式部署
    do_blocking_move_to_z(last_z - LEVEL_CORNERS_PROBE_TOLERANCE, MMM_TO_MMS(Z_PROBE_FEEDRATE_SLOW)); // Move down to lower tolerance//向下移动到较低的公差
    if (TEST(endstops.trigger_state(), Z_MIN_PROBE)) { // check if probe triggered//检查探针是否触发
      endstops.hit_on_purpose();
      set_current_from_steppers_for_axis(Z_AXIS);
      sync_plan_position();
      TERN_(BLTOUCH_SLOW_MODE, bltouch.stow()); // Stow in LOW SPEED MODE on every trigger//在每个触发器上以低速模式收起
      // Triggered outside tolerance range?//触发超出公差范围？
      if (ABS(current_position.z - last_z) > LEVEL_CORNERS_PROBE_TOLERANCE) {
        last_z = current_position.z; // Above tolerance. Set a new Z for subsequent corners.//超出公差。为后续角点设置新的Z。
        good_points = 0;             // ...and start over//…然后重新开始
      }
      return true; // probe triggered//探针触发
    }
    do_blocking_move_to_z(last_z); // go back to tolerance middle point before raise//升起前返回公差中点
    return false; // probe not triggered//探测器未触发
  }

  bool _lcd_level_bed_corners_raise() {
    bool probe_triggered = false;
    corner_probing_done = false;
    wait_for_probe = true;
    ui.goto_screen(_lcd_draw_raise); // show raise screen//显示提升屏幕
    ui.set_selection(true);
    while (wait_for_probe && !probe_triggered) { // loop while waiting to bed raise and probe trigger//等待床上升起和探测触发器时循环
      probe_triggered = PROBE_TRIGGERED();
      if (probe_triggered) {
        endstops.hit_on_purpose();
        TERN_(LEVEL_CORNERS_AUDIO_FEEDBACK, ui.buzz(200, 600));
      }
      idle();
    }
    TERN_(BLTOUCH_SLOW_MODE, bltouch.stow());
    ui.goto_screen(_lcd_draw_probing);
    return (probe_triggered);
  }

  void _lcd_test_corners() {
    bed_corner = TERN(LEVEL_CENTER_TOO, center_index, 0);
    last_z = LEVEL_CORNERS_HEIGHT;
    endstops.enable_z_probe(true);
    good_points = 0;
    ui.goto_screen(_lcd_draw_probing);
    do {
      ui.refresh(LCDVIEW_REDRAW_NOW);
      _lcd_draw_probing();                                // update screen with # of good points//用#个好点更新屏幕
      do_blocking_move_to_z(SUM_TERN(BLTOUCH_HS_MODE, current_position.z + LEVEL_CORNERS_Z_HOP, 7)); // clearance//净空

      _lcd_level_bed_corners_get_next_position();         // Select next corner coordinates//选择下一个角点坐标
      current_position -= probe.offset_xy;                // Account for probe offsets//探针偏移的帐户
      do_blocking_move_to_xy(current_position);           // Goto corner//转到角落

      TERN_(BLTOUCH_HS_MODE, bltouch.deploy());           // Deploy in HIGH SPEED MODE//以高速模式部署
      if (!_lcd_level_bed_corners_probe()) {              // Probe down to tolerance//深入到公差
        if (_lcd_level_bed_corners_raise()) {             // Prompt user to raise bed if needed//如果需要，提示用户升起床
          #if ENABLED(LEVEL_CORNERS_VERIFY_RAISED)        // Verify//核实
            while (!_lcd_level_bed_corners_probe(true)) { // Loop while corner verified//边验证边循环
              if (!_lcd_level_bed_corners_raise()) {      // Prompt user to raise bed if needed//如果需要，提示用户升起床
                if (corner_probing_done) return;          // Done was selected//已选择“完成”
                break;                                    // Skip was selected//已选择跳过
              }
            }
          #endif
        }
        else if (corner_probing_done)                     // Done was selected//已选择“完成”
          return;
      }

      if (bed_corner != center_index) good_points++; // ignore center//忽略中心
      if (++bed_corner > 3) bed_corner = 0;

    } while (good_points < nr_edge_points); // loop until all points within tolerance//循环，直到所有点都在公差范围内

    #if ENABLED(BLTOUCH_HS_MODE)
      // In HIGH SPEED MODE do clearance and stow at the very end//在高速模式下，在最末端进行间隙和收起
      do_blocking_move_to_z(current_position.z + LEVEL_CORNERS_Z_HOP);
      bltouch.stow();
    #endif

    ui.goto_screen(_lcd_draw_level_prompt); // prompt for bed leveling//河床平整提示
    ui.set_selection(true);
  }

#else // !LEVEL_CORNERS_USE_PROBE// !标高\u转角\u使用\u探头

  static void _lcd_goto_next_corner() {
    line_to_z(LEVEL_CORNERS_Z_HOP);

    // Select next corner coordinates//选择下一个角点坐标
    _lcd_level_bed_corners_get_next_position();

    line_to_current_position(manual_feedrate_mm_s.x);
    line_to_z(LEVEL_CORNERS_HEIGHT);
    if (++bed_corner >= available_points) bed_corner = 0;
  }

#endif // !LEVEL_CORNERS_USE_PROBE// !标高\u转角\u使用\u探头

static void _lcd_level_bed_corners_homing() {
  _lcd_draw_homing();
  if (!all_axes_homed()) return;
  #if ENABLED(LEVEL_CORNERS_USE_PROBE)
    _lcd_test_corners();
    if (corner_probing_done) ui.goto_previous_screen_no_defer();
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(leveling_was_active));
    endstops.enable_z_probe(false);
  #else
    bed_corner = 0;
    ui.goto_screen([]{
      MenuItem_confirm::select_screen(
          GET_TEXT(MSG_BUTTON_NEXT), GET_TEXT(MSG_BUTTON_DONE)
        , _lcd_goto_next_corner
        , []{
            line_to_z(LEVEL_CORNERS_Z_HOP); // Raise Z off the bed when done//完成后将Z抬离床
            TERN_(HAS_LEVELING, set_bed_leveling_enabled(leveling_was_active));
            ui.goto_previous_screen_no_defer();
          }
        , GET_TEXT(TERN(LEVEL_CENTER_TOO, MSG_LEVEL_BED_NEXT_POINT, MSG_NEXT_CORNER))
        , (const char*)nullptr, PSTR("?")
      );
    });
    ui.set_selection(true);
    _lcd_goto_next_corner();
  #endif
}

void _lcd_level_bed_corners() {
  ui.defer_status_screen();
  if (!all_axes_trusted()) {
    set_all_unhomed();
    queue.inject_P(G28_STR);
  }

  // Disable leveling so the planner won't mess with us//禁用水平调整，这样计划员就不会打扰我们了
  #if HAS_LEVELING
    leveling_was_active = planner.leveling_active;
    set_bed_leveling_enabled(false);
  #endif

  ui.goto_screen(_lcd_level_bed_corners_homing);
}

#endif // HAS_LCD_MENU && LEVEL_BED_CORNERS//有LCD菜单和水平床角
