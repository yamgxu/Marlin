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

#include "../inc/MarlinConfigPre.h"

#if ENABLED(BACKLASH_COMPENSATION)

#include "backlash.h"

#include "../module/motion.h"
#include "../module/planner.h"

#ifdef BACKLASH_DISTANCE_MM
  #if ENABLED(BACKLASH_GCODE)
    xyz_float_t Backlash::distance_mm = BACKLASH_DISTANCE_MM;
  #else
    const xyz_float_t Backlash::distance_mm = BACKLASH_DISTANCE_MM;
  #endif
#endif

#if ENABLED(BACKLASH_GCODE)
  uint8_t Backlash::correction = (BACKLASH_CORRECTION) * 0xFF;
  #ifdef BACKLASH_SMOOTHING_MM
    float Backlash::smoothing_mm = BACKLASH_SMOOTHING_MM;
  #endif
#endif

#if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
  xyz_float_t Backlash::measured_mm{0};
  xyz_uint8_t Backlash::measured_count{0};
#endif

Backlash backlash;

/**
 * To minimize seams in the printed part, backlash correction only adds
 * steps to the current segment (instead of creating a new segment, which
 * causes discontinuities and print artifacts).
 *
 * With a non-zero BACKLASH_SMOOTHING_MM value the backlash correction is
 * spread over multiple segments, smoothing out artifacts even more.
 */

void Backlash::add_correction_steps(const int32_t &da, const int32_t &db, const int32_t &dc, const uint8_t dm, block_t * const block) {
  static uint8_t last_direction_bits;
  uint8_t changed_dir = last_direction_bits ^ dm;
  // Ignore direction change unless steps are taken in that direction//忽略方向更改，除非在该方向上采取步骤
  #if DISABLED(CORE_BACKLASH) || ENABLED(MARKFORGED_XY)
    if (!da) CBI(changed_dir, X_AXIS);
    if (!db) CBI(changed_dir, Y_AXIS);
    if (!dc) CBI(changed_dir, Z_AXIS);
  #elif CORE_IS_XY
    if (!(da + db)) CBI(changed_dir, X_AXIS);
    if (!(da - db)) CBI(changed_dir, Y_AXIS);
    if (!dc)        CBI(changed_dir, Z_AXIS);
  #elif CORE_IS_XZ
    if (!(da + dc)) CBI(changed_dir, X_AXIS);
    if (!(da - dc)) CBI(changed_dir, Z_AXIS);
    if (!db)        CBI(changed_dir, Y_AXIS);
  #elif CORE_IS_YZ
    if (!(db + dc)) CBI(changed_dir, Y_AXIS);
    if (!(db - dc)) CBI(changed_dir, Z_AXIS);
    if (!da)        CBI(changed_dir, X_AXIS);
  #endif
  last_direction_bits ^= changed_dir;

  if (correction == 0) return;

  #ifdef BACKLASH_SMOOTHING_MM
    // The segment proportion is a value greater than 0.0 indicating how much residual_error//段比例是一个大于0.0的值，表示剩余误差的大小
    // is corrected for in this segment. The contribution is based on segment length and the//在该段中已更正。贡献基于段长度和长度
    // smoothing distance. Since the computation of this proportion involves a floating point//平滑距离。因为这个比例的计算涉及到一个浮点数
    // division, defer computation until needed.//除法，将计算推迟到需要时。
    float segment_proportion = 0;

    // Residual error carried forward across multiple segments, so correction can be applied//剩余误差在多段间结转，因此可以进行校正
    // to segments where there is no direction change.//到没有方向更改的线段。
    static xyz_long_t residual_error{0};
  #else
    // No direction change, no correction.//没有方向改变，就没有修正。
    if (!changed_dir) return;
    // No leftover residual error from segment to segment//段与段之间无剩余误差
    xyz_long_t residual_error{0};
  #endif

  const float f_corr = float(correction) / 255.0f;

  LOOP_LINEAR_AXES(axis) {
    if (distance_mm[axis]) {
      const bool reversing = TEST(dm,axis);

      // When an axis changes direction, add axis backlash to the residual error//当轴改变方向时，将轴齿隙添加到残余误差中
      if (TEST(changed_dir, axis))
        residual_error[axis] += (reversing ? -f_corr : f_corr) * distance_mm[axis] * planner.settings.axis_steps_per_mm[axis];

      // Decide how much of the residual error to correct in this segment//决定此段中要纠正的剩余误差的大小
      int32_t error_correction = residual_error[axis];
      #ifdef BACKLASH_SMOOTHING_MM
        if (error_correction && smoothing_mm != 0) {
          // Take up a portion of the residual_error in this segment, but only when//在该段中，仅当
          // the current segment travels in the same direction as the correction//当前段的移动方向与校正方向相同
          if (reversing == (error_correction < 0)) {
            if (segment_proportion == 0) segment_proportion = _MIN(1.0f, block->millimeters / smoothing_mm);
            error_correction = CEIL(segment_proportion * error_correction);
          }
          else
            error_correction = 0; // Don't take up any backlash in this segment, as it would subtract steps//在这一段中不要采取任何反冲，因为这会减少步数
        }
      #endif

      // This correction reduces the residual error and adds block steps//这种校正减少了残余误差并增加了块步数
      if (error_correction) {
        block->steps[axis] += ABS(error_correction);
        #if ENABLED(CORE_BACKLASH)
          switch (axis) {
            case CORE_AXIS_1:
              //block->steps[CORE_AXIS_2] += influence_distance_mm[axis] * planner.settings.axis_steps_per_mm[CORE_AXIS_2];//块->步数[CORE_AXIS_2]+=影响距离[AXIS]*planner.settings.AXIS_steps_per_mm[CORE_AXIS_2]；
              //SERIAL_ECHOLNPAIR("CORE_AXIS_1 dir change. distance=", distance_mm[axis], " r.err=", residual_error[axis],//串行回波对（“芯轴1方向改变。距离=”，距离毫米[轴]，“r.err=”，残余误差[轴]，
              //  " da=", da, " db=", db, " block->steps[axis]=", block->steps[axis], " err_corr=", error_correction);//“da=”，da，“db=”，db，“块->步数[轴]=”，块->步数[轴]，“err_corr=”，error_corr=”，error_correction）；
              break;
            case CORE_AXIS_2:
              //block->steps[CORE_AXIS_1] += influence_distance_mm[axis] * planner.settings.axis_steps_per_mm[CORE_AXIS_1];;//块->步数[CORE_AXIS_1]+=影响距离[AXIS]*planner.settings.AXIS_steps_per_mm[CORE_AXIS_1]；；
              //SERIAL_ECHOLNPAIR("CORE_AXIS_2 dir change. distance=", distance_mm[axis], " r.err=", residual_error[axis],//串行回波对（“芯轴方向改变。距离=”，距离毫米[轴]，“r.err=”，残余误差[轴]，
              //  " da=", da, " db=", db, " block->steps[axis]=", block->steps[axis], " err_corr=", error_correction);//“da=”，da，“db=”，db，“块->步数[轴]=”，块->步数[轴]，“err_corr=”，error_corr=”，error_correction）；
              break;
            case NORMAL_AXIS: break;
          }
          residual_error[axis] = 0; // No residual_error needed for next CORE block, I think...//我认为，下一个核心块不需要任何剩余的_错误。。。
        #else
          residual_error[axis] -= error_correction;
        #endif
      }
    }
  }
}

#if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)

  #include "../module/probe.h"

  // Measure Z backlash by raising nozzle in increments until probe deactivates//通过逐渐升高喷嘴测量Z齿隙，直到探针停用
  void Backlash::measure_with_probe() {
    if (measured_count.z == 255) return;

    const float start_height = current_position.z;
    while (current_position.z < (start_height + BACKLASH_MEASUREMENT_LIMIT) && PROBE_TRIGGERED())
      do_blocking_move_to_z(current_position.z + BACKLASH_MEASUREMENT_RESOLUTION, MMM_TO_MMS(BACKLASH_MEASUREMENT_FEEDRATE));

    // The backlash from all probe points is averaged, so count the number of measurements//所有探针点的齿隙均为平均值，因此计算测量次数
    measured_mm.z += current_position.z - start_height;
    measured_count.z++;
  }

#endif

#endif // BACKLASH_COMPENSATION//齿隙补偿
