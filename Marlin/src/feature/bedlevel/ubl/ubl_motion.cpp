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
#include "../../../inc/MarlinConfig.h"

#if ENABLED(AUTO_BED_LEVELING_UBL)

#include "../bedlevel.h"
#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../module/motion.h"

#if ENABLED(DELTA)
  #include "../../../module/delta.h"
#endif

#include "../../../MarlinCore.h"
#include <math.h>

#if !UBL_SEGMENTED

  void unified_bed_leveling::line_to_destination_cartesian(const_feedRate_t scaled_fr_mm_s, const uint8_t extruder) {
    /**
     * Much of the nozzle movement will be within the same cell. So we will do as little computation
     * as possible to determine if this is the case. If this move is within the same cell, we will
     * just do the required Z-Height correction, call the Planner's buffer_line() routine, and leave
     */
    #if HAS_POSITION_MODIFIERS
      xyze_pos_t start = current_position, end = destination;
      planner.apply_modifiers(start);
      planner.apply_modifiers(end);
    #else
      const xyze_pos_t &start = current_position, &end = destination;
    #endif

    const xy_int8_t istart = cell_indexes(start), iend = cell_indexes(end);

    // A move within the same cell needs no splitting//同一单元格内的移动不需要拆分
    if (istart == iend) {

      FINAL_MOVE:

      // When UBL_Z_RAISE_WHEN_OFF_MESH is disabled Z correction is extrapolated from the edge of the mesh//禁用“禁用网格”时，UBL_Z_RAISE_时，Z校正将从网格边缘外推
      #ifdef UBL_Z_RAISE_WHEN_OFF_MESH
        // For a move off the UBL mesh, use a constant Z raise//要从UBL网格移出，请使用恒定的Z向上移动
        if (!cell_index_x_valid(end.x) || !cell_index_y_valid(end.y)) {

          // Note: There is no Z Correction in this case. We are off the mesh and don't know what//注：在这种情况下没有Z校正。我们脱离了网格，不知道发生了什么
          // a reasonable correction would be, UBL_Z_RAISE_WHEN_OFF_MESH will be used instead of//一个合理的修正是，将使用UBL_Z_RAISE_，而不是
          // a calculated (Bi-Linear interpolation) correction.//计算（双线性插值）校正。

          end.z += UBL_Z_RAISE_WHEN_OFF_MESH;
          planner.buffer_segment(end, scaled_fr_mm_s, extruder);
          current_position = destination;
          return;
        }
      #endif

      // The distance is always MESH_X_DIST so multiply by the constant reciprocal.//距离始终是网格X距离，因此乘以常数倒数。
      const float xratio = (end.x - mesh_index_to_xpos(iend.x)) * RECIPROCAL(MESH_X_DIST),
                  yratio = (end.y - mesh_index_to_ypos(iend.y)) * RECIPROCAL(MESH_Y_DIST),
                  z1 = z_values[iend.x][iend.y    ] + xratio * (z_values[iend.x + 1][iend.y    ] - z_values[iend.x][iend.y    ]),
                  z2 = z_values[iend.x][iend.y + 1] + xratio * (z_values[iend.x + 1][iend.y + 1] - z_values[iend.x][iend.y + 1]);

      // X cell-fraction done. Interpolate the two Z offsets with the Y fraction for the final Z offset.//X细胞部分完成。使用最终Z偏移的Y分数插值两个Z偏移。
      const float z0 = (z1 + (z2 - z1) * yratio) * planner.fade_scaling_factor_for_z(end.z);

      // Undefined parts of the Mesh in z_values[][] are NAN.//z_值[][]中未定义的网格部分为NAN。
      // Replace NAN corrections with 0.0 to prevent NAN propagation.//将NAN校正替换为0.0以防止NAN传播。
      if (!isnan(z0)) end.z += z0;
      planner.buffer_segment(end, scaled_fr_mm_s, extruder);
      current_position = destination;
      return;
    }

    /**
     * Past this point the move is known to cross one or more mesh lines. Check for the most common
     * case - crossing only one X or Y line - after details are worked out to reduce computation.
     */

    const xy_float_t dist = end - start;
    const xy_bool_t neg { dist.x < 0, dist.y < 0 };
    const xy_int8_t ineg { int8_t(neg.x), int8_t(neg.y) };
    const xy_float_t sign { neg.x ? -1.0f : 1.0f, neg.y ? -1.0f : 1.0f };
    const xy_int8_t iadd { int8_t(iend.x == istart.x ? 0 : sign.x), int8_t(iend.y == istart.y ? 0 : sign.y) };

    /**
     * Compute the extruder scaling factor for each partial move, checking for
     * zero-length moves that would result in an infinite scaling factor.
     * A float divide is required for this, but then it just multiplies.
     * Also select a scaling factor based on the larger of the X and Y
     * components. The larger of the two is used to preserve precision.
     */

    const xy_float_t ad = sign * dist;
    const bool use_x_dist = ad.x > ad.y;

    float on_axis_distance = use_x_dist ? dist.x : dist.y;

    const float z_normalized_dist = (end.z - start.z) / on_axis_distance; // Allow divide by zero//允许被零除
    #if HAS_EXTRUDERS
      const float e_normalized_dist = (end.e - start.e) / on_axis_distance;
      const bool inf_normalized_flag = isinf(e_normalized_dist);
    #endif

    xy_int8_t icell = istart;

    const float ratio = dist.y / dist.x,        // Allow divide by zero//允许被零除
                c = start.y - ratio * start.x;

    const bool inf_ratio_flag = isinf(ratio);

    xyze_pos_t dest; // Stores XYZE for segmented moves//存储分段移动的XYZE

    /**
     * Handle vertical lines that stay within one column.
     * These need not be perfectly vertical.
     */
    if (iadd.x == 0) {        // Vertical line?//垂直线？
      icell.y += ineg.y;      // Line going down? Just go to the bottom.//电话线断了？到下面去。
      while (icell.y != iend.y + ineg.y) {
        icell.y += iadd.y;
        const float next_mesh_line_y = mesh_index_to_ypos(icell.y);

        /**
         * Skip the calculations for an infinite slope.
         * For others the next X is the same so this can continue.
         * Calculate X at the next Y mesh line.
         */
        dest.x = inf_ratio_flag ? start.x : (next_mesh_line_y - c) / ratio;

        float z0 = z_correction_for_x_on_horizontal_mesh_line(dest.x, icell.x, icell.y)
                   * planner.fade_scaling_factor_for_z(end.z);

        // Undefined parts of the Mesh in z_values[][] are NAN.//z_值[][]中未定义的网格部分为NAN。
        // Replace NAN corrections with 0.0 to prevent NAN propagation.//将NAN校正替换为0.0以防止NAN传播。
        if (isnan(z0)) z0 = 0.0;

        dest.y = mesh_index_to_ypos(icell.y);

        /**
         * Without this check, it's possible to generate a zero length move, as in the case where
         * the line is heading down, starting exactly on a mesh line boundary. Since this is rare
         * it might be fine to remove this check and let planner.buffer_segment() filter it out.
         */
        if (dest.y != start.y) {
          if (!inf_normalized_flag) { // fall-through faster than branch//落空比树枝快
            on_axis_distance = use_x_dist ? dest.x - start.x : dest.y - start.y;
            TERN_(HAS_EXTRUDERS, dest.e = start.e + on_axis_distance * e_normalized_dist);
            dest.z = start.z + on_axis_distance * z_normalized_dist;
          }
          else {
            TERN_(HAS_EXTRUDERS, dest.e = end.e);
            dest.z = end.z;
          }

          dest.z += z0;
          planner.buffer_segment(dest, scaled_fr_mm_s, extruder);

        } //else printf("FIRST MOVE PRUNED  ");//else printf（“第一步修剪”）；
      }

      // At the final destination? Usually not, but when on a Y Mesh Line it's completed.//在最终目的地？通常不会，但当在Y网格线上时，它已完成。
      if (xy_pos_t(current_position) != xy_pos_t(end))
        goto FINAL_MOVE;

      current_position = destination;
      return;
    }

    /**
     * Handle horizontal lines that stay within one row.
     * These need not be perfectly horizontal.
     */
    if (iadd.y == 0) {      // Horizontal line?//水平线？
      icell.x += ineg.x;     // Heading left? Just go to the left edge of the cell for the first move.//向左转？第一步只需走到单元格的左边缘。

      while (icell.x != iend.x + ineg.x) {
        icell.x += iadd.x;
        dest.x = mesh_index_to_xpos(icell.x);
        dest.y = ratio * dest.x + c;    // Calculate Y at the next X mesh line//在下一条X网格线上计算Y

        float z0 = z_correction_for_y_on_vertical_mesh_line(dest.y, icell.x, icell.y)
                     * planner.fade_scaling_factor_for_z(end.z);

        // Undefined parts of the Mesh in z_values[][] are NAN.//z_值[][]中未定义的网格部分为NAN。
        // Replace NAN corrections with 0.0 to prevent NAN propagation.//将NAN校正替换为0.0以防止NAN传播。
        if (isnan(z0)) z0 = 0.0;

        /**
         * Without this check, it's possible to generate a zero length move, as in the case where
         * the line is heading left, starting exactly on a mesh line boundary. Since this is rare
         * it might be fine to remove this check and let planner.buffer_segment() filter it out.
         */
        if (dest.x != start.x) {
          if (!inf_normalized_flag) {
            on_axis_distance = use_x_dist ? dest.x - start.x : dest.y - start.y;
            TERN_(HAS_EXTRUDERS, dest.e = start.e + on_axis_distance * e_normalized_dist); // Based on X or Y because the move is horizontal//基于X或Y，因为移动是水平的
            dest.z = start.z + on_axis_distance * z_normalized_dist;
          }
          else {
            TERN_(HAS_EXTRUDERS, dest.e = end.e);
            dest.z = end.z;
          }

          dest.z += z0;
          if (!planner.buffer_segment(dest, scaled_fr_mm_s, extruder)) break;

        } //else printf("FIRST MOVE PRUNED  ");//else printf（“第一步修剪”）；
      }

      if (xy_pos_t(current_position) != xy_pos_t(end))
        goto FINAL_MOVE;

      current_position = destination;
      return;
    }

    /**
     * Generic case of a line crossing both X and Y Mesh lines.
     */

    xy_int8_t cnt = (istart - iend).ABS();

    icell += ineg;

    while (cnt) {

      const float next_mesh_line_x = mesh_index_to_xpos(icell.x + iadd.x),
                  next_mesh_line_y = mesh_index_to_ypos(icell.y + iadd.y);

      dest.y = ratio * next_mesh_line_x + c;    // Calculate Y at the next X mesh line//在下一条X网格线上计算Y
      dest.x = (next_mesh_line_y - c) / ratio;  // Calculate X at the next Y mesh line//计算下一条Y网格线处的X
                                                // (No need to worry about ratio == 0.//（无需担心比率==0。
                                                //  In that case, it was already detected//在这种情况下，它已经被检测到
                                                //  as a vertical line move above.)//当垂直线向上移动时。）

      if (neg.x == (dest.x > next_mesh_line_x)) { // Check if we hit the Y line first//检查我们是否先到达Y线
        // Yes!  Crossing a Y Mesh Line next//对!！下一步穿过Y网格线
        float z0 = z_correction_for_x_on_horizontal_mesh_line(dest.x, icell.x - ineg.x, icell.y + iadd.y)
                   * planner.fade_scaling_factor_for_z(end.z);

        // Undefined parts of the Mesh in z_values[][] are NAN.//z_值[][]中未定义的网格部分为NAN。
        // Replace NAN corrections with 0.0 to prevent NAN propagation.//将NAN校正替换为0.0以防止NAN传播。
        if (isnan(z0)) z0 = 0.0;

        dest.y = next_mesh_line_y;

        if (!inf_normalized_flag) {
          on_axis_distance = use_x_dist ? dest.x - start.x : dest.y - start.y;
          TERN_(HAS_EXTRUDERS, dest.e = start.e + on_axis_distance * e_normalized_dist);
          dest.z = start.z + on_axis_distance * z_normalized_dist;
        }
        else {
          TERN_(HAS_EXTRUDERS, dest.e = end.e);
          dest.z = end.z;
        }

        dest.z += z0;
        if (!planner.buffer_segment(dest, scaled_fr_mm_s, extruder)) break;

        icell.y += iadd.y;
        cnt.y--;
      }
      else {
        // Yes!  Crossing a X Mesh Line next//对!！下一步穿过X网格线
        float z0 = z_correction_for_y_on_vertical_mesh_line(dest.y, icell.x + iadd.x, icell.y - ineg.y)
                   * planner.fade_scaling_factor_for_z(end.z);

        // Undefined parts of the Mesh in z_values[][] are NAN.//z_值[][]中未定义的网格部分为NAN。
        // Replace NAN corrections with 0.0 to prevent NAN propagation.//将NAN校正替换为0.0以防止NAN传播。
        if (isnan(z0)) z0 = 0.0;

        dest.x = next_mesh_line_x;

        if (!inf_normalized_flag) {
          on_axis_distance = use_x_dist ? dest.x - start.x : dest.y - start.y;
          TERN_(HAS_EXTRUDERS, dest.e = start.e + on_axis_distance * e_normalized_dist);
          dest.z = start.z + on_axis_distance * z_normalized_dist;
        }
        else {
          TERN_(HAS_EXTRUDERS, dest.e = end.e);
          dest.z = end.z;
        }

        dest.z += z0;
        if (!planner.buffer_segment(dest, scaled_fr_mm_s, extruder)) break;

        icell.x += iadd.x;
        cnt.x--;
      }

      if (cnt.x < 0 || cnt.y < 0) break; // Too far! Exit the loop and go to FINAL_MOVE//太远了！退出循环并转到最后一步
    }

    if (xy_pos_t(current_position) != xy_pos_t(end))
      goto FINAL_MOVE;

    current_position = destination;
  }

#else // UBL_SEGMENTED//分段的

  #if IS_SCARA
    #define DELTA_SEGMENT_MIN_LENGTH 0.25 // SCARA minimum segment size is 0.25mm//SCARA最小分段尺寸为0.25mm
  #elif ENABLED(DELTA)
    #define DELTA_SEGMENT_MIN_LENGTH 0.10 // mm (still subject to DELTA_SEGMENTS_PER_SECOND)//毫米（仍以每秒增量段为准）
  #else // CARTESIAN//笛卡尔
    #ifdef LEVELED_SEGMENT_LENGTH
      #define DELTA_SEGMENT_MIN_LENGTH LEVELED_SEGMENT_LENGTH
    #else
      #define DELTA_SEGMENT_MIN_LENGTH 1.00 // mm (similar to G2/G3 arc segmentation)//mm（类似于G2/G3圆弧分段）
    #endif
  #endif

  /**
   * Prepare a segmented linear move for DELTA/SCARA/CARTESIAN with UBL and FADE semantics.
   * This calls planner.buffer_segment multiple times for small incremental moves.
   * Returns true if did NOT move, false if moved (requires current_position update).
   */

  bool _O2 unified_bed_leveling::line_to_destination_segmented(const_feedRate_t scaled_fr_mm_s) {

    if (!position_is_reachable(destination))  // fail if moving outside reachable boundary//如果移动到可达边界之外，则失败
      return true;                            // did not move, so current_position still accurate//没有移动，因此当前位置仍然准确

    const xyze_pos_t total = destination - current_position;

    const float cart_xy_mm_2 = HYPOT2(total.x, total.y),
                cart_xy_mm = SQRT(cart_xy_mm_2);                                     // Total XY distance//总XY距离

    #if IS_KINEMATIC
      const float seconds = cart_xy_mm / scaled_fr_mm_s;                             // Duration of XY move at requested rate//按请求速率移动XY的持续时间
      uint16_t segments = LROUND(segments_per_second * seconds),                     // Preferred number of segments for distance @ feedrate//进给速度下距离的首选段数
               seglimit = LROUND(cart_xy_mm * RECIPROCAL(DELTA_SEGMENT_MIN_LENGTH)); // Number of segments at minimum segment length//最小段长度的段数
      NOMORE(segments, seglimit);                                                    // Limit to minimum segment length (fewer segments)//限制为最小段长度（较少段）
    #else
      uint16_t segments = LROUND(cart_xy_mm * RECIPROCAL(DELTA_SEGMENT_MIN_LENGTH)); // Cartesian fixed segment length//笛卡尔固定段长度
    #endif

    NOLESS(segments, 1U);                                                            // Must have at least one segment//必须至少有一个段
    const float inv_segments = 1.0f / segments,                                      // Reciprocal to save calculation//保存计算的倒数
                segment_xyz_mm = SQRT(cart_xy_mm_2 + sq(total.z)) * inv_segments;    // Length of each segment//每段的长度

    #if ENABLED(SCARA_FEEDRATE_SCALING)
      const float inv_duration = scaled_fr_mm_s / segment_xyz_mm;
    #endif

    xyze_float_t diff = total * inv_segments;

    // Note that E segment distance could vary slightly as z mesh height//请注意，E段距离可能随z网格高度略有变化
    // changes for each segment, but small enough to ignore.//每个段的更改，但小到可以忽略。

    xyze_pos_t raw = current_position;

    // Just do plain segmentation if UBL is inactive or the target is above the fade height//如果UBL处于非活动状态或目标高于淡入度高度，只需执行普通分割
    if (!planner.leveling_active || !planner.leveling_active_at_z(destination.z)) {
      while (--segments) {
        raw += diff;
        planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, segment_xyz_mm
          OPTARG(SCARA_FEEDRATE_SCALING, inv_duration)
        );
      }
      planner.buffer_line(destination, scaled_fr_mm_s, active_extruder, segment_xyz_mm
        OPTARG(SCARA_FEEDRATE_SCALING, inv_duration)
      );
      return false; // Did not set current from destination//未从目标设置当前值
    }

    // Otherwise perform per-segment leveling//否则，执行每段调平

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      const float fade_scaling_factor = planner.fade_scaling_factor_for_z(destination.z);
    #endif

    // Move to first segment destination//移动到第一段目的地
    raw += diff;

    for (;;) {  // for each mesh cell encountered during the move//对于移动过程中遇到的每个网格单元

      // Compute mesh cell invariants that remain constant for all segments within cell.//计算网格单元不变量，该不变量对于单元内的所有线段保持不变。
      // Note for cell index, if point is outside the mesh grid (in MESH_INSET perimeter)//注意：如果点位于网格网格外（在网格内插入周长），则为单元索引
      // the bilinear interpolation from the adjacent cell within the mesh will still work.//来自网格内相邻单元的双线性插值仍然有效。
      // Inner loop will exit each time (because out of cell bounds) but will come back//内部循环每次都会退出（因为超出单元格边界），但会返回
      // in top of loop and again re-find same adjacent cell and use it, just less efficient//在循环的顶部，再次找到相同的相邻单元并使用它，只是效率较低
      // for mesh inset area.//用于网格嵌入区域。

      xy_int8_t icell = {
        int8_t((raw.x - (MESH_MIN_X)) * RECIPROCAL(MESH_X_DIST)),
        int8_t((raw.y - (MESH_MIN_Y)) * RECIPROCAL(MESH_Y_DIST))
      };
      LIMIT(icell.x, 0, GRID_MAX_CELLS_X);
      LIMIT(icell.y, 0, GRID_MAX_CELLS_Y);

      float z_x0y0 = z_values[icell.x  ][icell.y  ],  // z at lower left corner//左下角的z
            z_x1y0 = z_values[icell.x+1][icell.y  ],  // z at upper left corner//左上角的z
            z_x0y1 = z_values[icell.x  ][icell.y+1],  // z at lower right corner//z位于右下角
            z_x1y1 = z_values[icell.x+1][icell.y+1];  // z at upper right corner//z位于右上角

      if (isnan(z_x0y0)) z_x0y0 = 0;              // ideally activating planner.leveling_active (G29 A)//理想情况下激活planner.leveling_激活（G29 A）
      if (isnan(z_x1y0)) z_x1y0 = 0;              //   should refuse if any invalid mesh points//如果存在任何无效网格点，则应拒绝
      if (isnan(z_x0y1)) z_x0y1 = 0;              //   in order to avoid isnan tests per cell,//为了避免每个单元进行isnan测试，
      if (isnan(z_x1y1)) z_x1y1 = 0;              //   thus guessing zero for undefined points//因此，对于未定义的点，猜测零

      const xy_pos_t pos = { mesh_index_to_xpos(icell.x), mesh_index_to_ypos(icell.y) };
      xy_pos_t cell = raw - pos;

      const float z_xmy0 = (z_x1y0 - z_x0y0) * RECIPROCAL(MESH_X_DIST),   // z slope per x along y0 (lower left to lower right)//沿y0每x的z斜率（从左下到右下）
                  z_xmy1 = (z_x1y1 - z_x0y1) * RECIPROCAL(MESH_X_DIST);   // z slope per x along y1 (upper left to upper right)//沿y1每x的z坡度（左上至右上）

            float z_cxy0 = z_x0y0 + z_xmy0 * cell.x;        // z height along y0 at cell.x (changes for each cell.x in cell)//沿y0的z高度在单元格.x处（单元格中每个单元格.x的变化）

      const float z_cxy1 = z_x0y1 + z_xmy1 * cell.x,        // z height along y1 at cell.x//x单元沿y1的z高度
                  z_cxyd = z_cxy1 - z_cxy0;                 // z height difference along cell.x from y0 to y1//从y0到y1沿x单元格的z高度差

            float z_cxym = z_cxyd * RECIPROCAL(MESH_Y_DIST); // z slope per y along cell.x from pos.y to y1 (changes for each cell.x in cell)//沿单元格x从位置y到y1的z斜率（单元格中每个单元格x的变化）

      //    float z_cxcy = z_cxy0 + z_cxym * cell.y;        // interpolated mesh z height along cell.x at cell.y (do inside the segment loop)//float z_cxcy=z_cxy0+z_cxym*cell.y；//沿cell.x在cell.y处插值网格z高度（在段循环内执行）

      // As subsequent segments step through this cell, the z_cxy0 intercept will change//随着后续段逐步通过该单元，z_cxy0截距将发生变化
      // and the z_cxym slope will change, both as a function of cell.x within the cell, and//z_cxym斜率将发生变化，既作为单元格内单元格.x的函数，也作为
      // each change by a constant for fixed segment lengths.//对于固定的管段长度，每一个变化都是一个常数。

      const float z_sxy0 = z_xmy0 * diff.x,                                       // per-segment adjustment to z_cxy0//对z_cxy0的每段调整
                  z_sxym = (z_xmy1 - z_xmy0) * RECIPROCAL(MESH_Y_DIST) * diff.x;  // per-segment adjustment to z_cxym//对z_cxym的每段调整

      for (;;) {  // for all segments within this mesh cell//用于此网格单元内的所有线段

        if (--segments == 0) raw = destination;     // if this is last segment, use destination for exact//如果这是最后一段，请使用目的地进行精确搜索

        const float z_cxcy = (z_cxy0 + z_cxym * cell.y) // interpolated mesh z height along cell.x at cell.y//沿cell.x在cell.y处插值网格z高度
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            * fade_scaling_factor                   // apply fade factor to interpolated mesh height//将淡入因子应用于插值网格高度
          #endif
        ;

        const float oldz = raw.z; raw.z += z_cxcy;
        planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, segment_xyz_mm OPTARG(SCARA_FEEDRATE_SCALING, inv_duration) );
        raw.z = oldz;

        if (segments == 0)                        // done with last segment//完成最后一段
          return false;                           // didn't set current from destination//未从目标设置当前值

        raw += diff;
        cell += diff;

        if (!WITHIN(cell.x, 0, MESH_X_DIST) || !WITHIN(cell.y, 0, MESH_Y_DIST))    // done within this cell, break to next//在此单元格内完成，请转到下一个单元格
          break;

        // Next segment still within same mesh cell, adjust the per-segment//下一段仍在同一网格单元内，请调整“每段”
        // slope and intercept to compute next z height.//坡度和截距以计算下一个z高度。

        z_cxy0 += z_sxy0;   // adjust z_cxy0 by per-segment z_sxy0//按段z_sxy0调整z_cxy0
        z_cxym += z_sxym;   // adjust z_cxym by per-segment z_sxym//每段z_sxym调整z_cxym

      } // segment loop//段循环
    } // cell loop//细胞环

    return false; // caller will update current_position//调用者将更新当前位置
  }

#endif // UBL_SEGMENTED//分段的

#endif // AUTO_BED_LEVELING_UBL//自动调平床
