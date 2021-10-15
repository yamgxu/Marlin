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

#if ENABLED(ARC_SUPPORT)

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"

#if ENABLED(DELTA)
  #include "../../module/delta.h"
#elif ENABLED(SCARA)
  #include "../../module/scara.h"
#endif

#if N_ARC_CORRECTION < 1
  #undef N_ARC_CORRECTION
  #define N_ARC_CORRECTION 1
#endif

/**
 * Plan an arc in 2 dimensions, with optional linear motion in a 3rd dimension
 *
 * The arc is traced by generating many small linear segments, as configured by
 * MM_PER_ARC_SEGMENT (Default 1mm). In the future we hope more slicers will include
 * an option to generate G2/G3 arcs for curved surfaces, as this will allow faster
 * boards to produce much smoother curved surfaces.
 */
void plan_arc(
  const xyze_pos_t &cart,   // Destination position//目的地位置
  const ab_float_t &offset, // Center of rotation relative to current_position//相对于当前位置的旋转中心
  const bool clockwise,     // Clockwise?//顺时针？
  const uint8_t circles     // Take the scenic route//走风景优美的路线
) {
  #if ENABLED(CNC_WORKSPACE_PLANES)
    AxisEnum p_axis, q_axis, l_axis;
    switch (gcode.workspace_plane) {
      default:
      case GcodeSuite::PLANE_XY: p_axis = X_AXIS; q_axis = Y_AXIS; l_axis = Z_AXIS; break;
      case GcodeSuite::PLANE_YZ: p_axis = Y_AXIS; q_axis = Z_AXIS; l_axis = X_AXIS; break;
      case GcodeSuite::PLANE_ZX: p_axis = Z_AXIS; q_axis = X_AXIS; l_axis = Y_AXIS; break;
    }
  #else
    constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS OPTARG(HAS_Z_AXIS, l_axis = Z_AXIS);
  #endif

  // Radius vector from center to current location//从中心到当前位置的半径向量
  ab_float_t rvec = -offset;

  const float radius = HYPOT(rvec.a, rvec.b),
              center_P = current_position[p_axis] - rvec.a,
              center_Q = current_position[q_axis] - rvec.b,
              rt_X = cart[p_axis] - center_P,
              rt_Y = cart[q_axis] - center_Q
              OPTARG(HAS_Z_AXIS, start_L = current_position[l_axis]);

  #ifdef MIN_ARC_SEGMENTS
    uint16_t min_segments = MIN_ARC_SEGMENTS;
  #else
    constexpr uint16_t min_segments = 1;
  #endif

  // Angle of rotation between position and target from the circle center.//从圆心到位置和目标之间的旋转角度。
  float angular_travel;

  // Do a full circle if starting and ending positions are "identical"//如果起始和结束位置“相同”，则做一整圈
  if (NEAR(current_position[p_axis], cart[p_axis]) && NEAR(current_position[q_axis], cart[q_axis])) {
    // Preserve direction for circles//保留圆的方向
    angular_travel = clockwise ? -RADIANS(360) : RADIANS(360);
  }
  else {
    // Calculate the angle//计算角度
    angular_travel = ATAN2(rvec.a * rt_Y - rvec.b * rt_X, rvec.a * rt_X + rvec.b * rt_Y);

    // Angular travel too small to detect? Just return.//角行程太小而无法检测？回来吧。
    if (!angular_travel) return;

    // Make sure angular travel over 180 degrees goes the other way around.//确保超过180度的角行程反过来。
    switch (((angular_travel < 0) << 1) | clockwise) {
      case 1: angular_travel -= RADIANS(360); break; // Positive but CW? Reverse direction.//是的，但是CW？反向。
      case 2: angular_travel += RADIANS(360); break; // Negative but CCW? Reverse direction.//是的，但是《特定常规武器公约》？反向。
    }

    #ifdef MIN_ARC_SEGMENTS
      min_segments = CEIL(min_segments * ABS(angular_travel) / RADIANS(360));
      NOLESS(min_segments, 1U);
    #endif
  }

  #if HAS_Z_AXIS
    float linear_travel = cart[l_axis] - start_L;
  #endif
  #if HAS_EXTRUDERS
    float extruder_travel = cart.e - current_position.e;
  #endif

  // If circling around...//如果绕着…转。。。
  if (ENABLED(ARC_P_CIRCLES) && circles) {
    const float total_angular = angular_travel + circles * RADIANS(360),  // Total rotation with all circles and remainder//带所有圆和余数的总旋转
              part_per_circle = RADIANS(360) / total_angular;             // Each circle's part of the total//每个圆都是总的一部分

    #if HAS_Z_AXIS
      const float l_per_circle = linear_travel * part_per_circle;         // L movement per circle//L每圈移动
    #endif
    #if HAS_EXTRUDERS
      const float e_per_circle = extruder_travel * part_per_circle;       // E movement per circle//E每圈移动量
    #endif

    xyze_pos_t temp_position = current_position;                          // for plan_arc to compare to current_position//用于将平面弧与当前位置进行比较
    for (uint16_t n = circles; n--;) {
      TERN_(HAS_EXTRUDERS, temp_position.e += e_per_circle);              // Destination E axis//目的地E轴
      TERN_(HAS_Z_AXIS, temp_position[l_axis] += l_per_circle);           // Destination L axis//目的地L轴
      plan_arc(temp_position, offset, clockwise, 0);                      // Plan a single whole circle//计划一整圈
    }
    TERN_(HAS_Z_AXIS, linear_travel = cart[l_axis] - current_position[l_axis]);
    TERN_(HAS_EXTRUDERS, extruder_travel = cart.e - current_position.e);
  }

  const float flat_mm = radius * angular_travel,
              mm_of_travel = TERN_(HAS_Z_AXIS, linear_travel ? HYPOT(flat_mm, linear_travel) :) ABS(flat_mm);
  if (mm_of_travel < 0.001f) return;

  const feedRate_t scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);

  // Start with a nominal segment length//从标称段长度开始
  float seg_length = (
    #ifdef ARC_SEGMENTS_PER_R
      constrain(MM_PER_ARC_SEGMENT * radius, MM_PER_ARC_SEGMENT, ARC_SEGMENTS_PER_R)
    #elif ARC_SEGMENTS_PER_SEC
      _MAX(scaled_fr_mm_s * RECIPROCAL(ARC_SEGMENTS_PER_SEC), MM_PER_ARC_SEGMENT)
    #else
      MM_PER_ARC_SEGMENT
    #endif
  );
  // Divide total travel by nominal segment length//将总行程除以标称段长度
  uint16_t segments = FLOOR(mm_of_travel / seg_length);
  NOLESS(segments, min_segments);         // At least some segments//至少有一些部分
  seg_length = mm_of_travel / segments;

  /**
   * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
   * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
   *     r_T = [cos(phi) -sin(phi);
   *            sin(phi)  cos(phi)] * r ;
   *
   * For arc generation, the center of the circle is the axis of rotation and the radius vector is
   * defined from the circle center to the initial position. Each line segment is formed by successive
   * vector rotations. This requires only two cos() and sin() computations to form the rotation
   * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
   * all double numbers are single precision on the Arduino. (True double precision will not have
   * round off issues for CNC applications.) Single precision error can accumulate to be greater than
   * tool precision in some cases. Therefore, arc path correction is implemented.
   *
   * Small angle approximation may be used to reduce computation overhead further. This approximation
   * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
   * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
   * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
   * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
   * issue for CNC machines with the single precision Arduino calculations.
   *
   * This approximation also allows plan_arc to immediately insert a line segment into the planner
   * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
   * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
   * This is important when there are successive arc motions.
   */
  // Vector rotation matrix values//向量旋转矩阵值
  xyze_pos_t raw;
  const float theta_per_segment = angular_travel / segments,
              sq_theta_per_segment = sq(theta_per_segment),
              sin_T = theta_per_segment - sq_theta_per_segment * theta_per_segment / 6,
              cos_T = 1 - 0.5f * sq_theta_per_segment; // Small angle approximation//小角度近似

  #if HAS_Z_AXIS && DISABLED(AUTO_BED_LEVELING_UBL)
    const float linear_per_segment = linear_travel / segments;
  #endif
  #if HAS_EXTRUDERS
    const float extruder_per_segment = extruder_travel / segments;
  #endif

  // Initialize the linear axis//初始化线性轴
  TERN_(HAS_Z_AXIS, raw[l_axis] = current_position[l_axis]);

  // Initialize the extruder axis//初始化挤出机轴
  TERN_(HAS_EXTRUDERS, raw.e = current_position.e);

  #if ENABLED(SCARA_FEEDRATE_SCALING)
    const float inv_duration = scaled_fr_mm_s / seg_length;
  #endif

  millis_t next_idle_ms = millis() + 200UL;

  #if N_ARC_CORRECTION > 1
    int8_t arc_recalc_count = N_ARC_CORRECTION;
  #endif

  for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times//迭代（段-1）次

    thermalManager.manage_heater();
    if (ELAPSED(millis(), next_idle_ms)) {
      next_idle_ms = millis() + 200UL;
      idle();
    }

    #if N_ARC_CORRECTION > 1
      if (--arc_recalc_count) {
        // Apply vector rotation matrix to previous rvec.a / 1//将矢量旋转矩阵应用于上一个rvec.a/1
        const float r_new_Y = rvec.a * sin_T + rvec.b * cos_T;
        rvec.a = rvec.a * cos_T - rvec.b * sin_T;
        rvec.b = r_new_Y;
      }
      else
    #endif
    {
      #if N_ARC_CORRECTION > 1
        arc_recalc_count = N_ARC_CORRECTION;
      #endif

      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.//对半径向量进行圆弧校正。仅每N_弧_校正增量计算一次。
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).//通过从初始半径向量（=偏移量）应用变换矩阵计算精确位置。
      // To reduce stuttering, the sin and cos could be computed at different times.//为了减少口吃，可以在不同的时间计算sin和cos。
      // For now, compute both at the same time.//现在，请同时计算两者。
      const float cos_Ti = cos(i * theta_per_segment), sin_Ti = sin(i * theta_per_segment);
      rvec.a = -offset[0] * cos_Ti + offset[1] * sin_Ti;
      rvec.b = -offset[0] * sin_Ti - offset[1] * cos_Ti;
    }

    // Update raw location//更新原始位置
    raw[p_axis] = center_P + rvec.a;
    raw[q_axis] = center_Q + rvec.b;
    #if HAS_Z_AXIS
      raw[l_axis] = TERN(AUTO_BED_LEVELING_UBL, start_L, raw[l_axis] + linear_per_segment);
    #endif

    TERN_(HAS_EXTRUDERS, raw.e += extruder_per_segment);

    apply_motion_limits(raw);

    #if HAS_LEVELING && !PLANNER_LEVELING
      planner.apply_leveling(raw);
    #endif

    if (!planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, 0
      OPTARG(SCARA_FEEDRATE_SCALING, inv_duration)
    )) break;
  }

  // Ensure last segment arrives at target location.//确保最后一段到达目标位置。
  raw = cart;
  TERN_(AUTO_BED_LEVELING_UBL, TERN_(HAS_Z_AXIS, raw[l_axis] = start_L));

  apply_motion_limits(raw);

  #if HAS_LEVELING && !PLANNER_LEVELING
    planner.apply_leveling(raw);
  #endif

  planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, 0
    OPTARG(SCARA_FEEDRATE_SCALING, inv_duration)
  );

  TERN_(AUTO_BED_LEVELING_UBL, TERN_(HAS_Z_AXIS, raw[l_axis] = start_L));
  current_position = raw;

} // plan_arc//平面图

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 *
 * This command has two forms: IJ-form (JK, KI) and R-form.
 *
 *  - Depending on the current Workspace Plane orientation,
 *    use parameters IJ/JK/KI to specify the XY/YZ/ZX offsets.
 *    At least one of the IJ/JK/KI parameters is required.
 *    XY/YZ/ZX can be omitted to do a complete circle.
 *    The given XY/YZ/ZX is not error-checked. The arc ends
 *    based on the angle of the destination.
 *    Mixing IJ/JK/KI with R will throw an error.
 *
 *  - R specifies the radius. X or Y (Y or Z / Z or X) is required.
 *      Omitting both XY/YZ/ZX will throw an error.
 *      XY/YZ/ZX must differ from the current XY/YZ/ZX.
 *      Mixing R with IJ/JK/KI will throw an error.
 *
 *  - P specifies the number of full circles to do
 *      before the specified arc move.
 *
 *  Examples:
 *
 *    G2 I10           ; CW circle centered at X+10
 *    G3 X20 Y12 R14   ; CCW circle with r=14 ending at X20 Y12
 */
void GcodeSuite::G2_G3(const bool clockwise) {
  if (MOTION_CONDITIONS) {

    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_RUNNING));

    #if ENABLED(SF_ARC_FIX)
      const bool relative_mode_backup = relative_mode;
      relative_mode = true;
    #endif

    get_destination_from_command();   // Get X Y Z E F (and set cutter power)//获取X Y Z E F（并设置刀具功率）

    TERN_(SF_ARC_FIX, relative_mode = relative_mode_backup);

    ab_float_t arc_offset = { 0, 0 };
    if (parser.seenval('R')) {
      const float r = parser.value_linear_units();
      if (r) {
        const xy_pos_t p1 = current_position, p2 = destination;
        if (p1 != p2) {
          const xy_pos_t d2 = (p2 - p1) * 0.5f;          // XY vector to midpoint of move from current//XY矢量到从当前移动的中点
          const float e = clockwise ^ (r < 0) ? -1 : 1,  // clockwise -1/1, counterclockwise 1/-1//顺时针-1/1，逆时针1/-1
                      len = d2.magnitude(),              // Distance to mid-point of move from current//从当前位置到移动中点的距离
                      h2 = (r - len) * (r + len),        // factored to reduce rounding error//减少舍入误差的因素
                      h = (h2 >= 0) ? SQRT(h2) : 0.0f;   // Distance to the arc pivot-point from midpoint//从中点到圆弧轴点的距离
          const xy_pos_t s = { -d2.y, d2.x };            // Perpendicular bisector. (Divide by len for unit vector.)//垂直平分线。（单位向量除以len。）
          arc_offset = d2 + s / len * e * h;             // The calculated offset (mid-point if |r| <= len)//计算的偏移量（如果| r |<=len，则为中点）
        }
      }
    }
    else {
      #if ENABLED(CNC_WORKSPACE_PLANES)
        char achar, bchar;
        switch (gcode.workspace_plane) {
          default:
          case GcodeSuite::PLANE_XY: achar = 'I'; bchar = 'J'; break;
          case GcodeSuite::PLANE_YZ: achar = 'J'; bchar = 'K'; break;
          case GcodeSuite::PLANE_ZX: achar = 'K'; bchar = 'I'; break;
        }
      #else
        constexpr char achar = 'I', bchar = 'J';
      #endif
      if (parser.seenval(achar)) arc_offset.a = parser.value_linear_units();
      if (parser.seenval(bchar)) arc_offset.b = parser.value_linear_units();
    }

    if (arc_offset) {

      #if ENABLED(ARC_P_CIRCLES)
        // P indicates number of circles to do//P表示要做的圈数
        const int8_t circles_to_do = parser.byteval('P');
        if (!WITHIN(circles_to_do, 0, 100))
          SERIAL_ERROR_MSG(STR_ERR_ARC_ARGS);
      #else
        constexpr uint8_t circles_to_do = 0;
      #endif

      // Send the arc to the planner//将弧发送给计划员
      plan_arc(destination, arc_offset, clockwise, circles_to_do);
      reset_stepper_timeout();
    }
    else
      SERIAL_ERROR_MSG(STR_ERR_ARC_ARGS);

    TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
  }
}

#endif // ARC_SUPPORT//弧形支架
