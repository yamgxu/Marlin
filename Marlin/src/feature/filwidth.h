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

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"
#include "../module/thermistor/thermistors.h"

class FilamentWidthSensor {
public:
  static constexpr int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;
  static bool enabled;              // (M405-M406) Filament Width Sensor ON/OFF.//（M405-M406）灯丝宽度传感器开/关。
  static uint32_t accum;            // ADC accumulator//ADC累加器
  static uint16_t raw;              // Measured filament diameter - one extruder only//测量的长丝直径-仅一台挤出机
  static float nominal_mm,          // (M104) Nominal filament width//（M104）标称灯丝宽度
               measured_mm,         // Measured filament diameter//测得的灯丝直径
               e_count, delay_dist;
  static uint8_t meas_delay_cm;     // Distance delay setting//距离延迟设置
  static int8_t ratios[MMD_CM],     // Ring buffer to delay measurement. (Extruder factor minus 100)//环形缓冲区延迟测量。（挤出机系数减100）
                index_r, index_w;   // Indexes into ring buffer//索引到环形缓冲区

  FilamentWidthSensor() { init(); }
  static void init();

  static inline void enable(const bool ena) { enabled = ena; }

  static inline void set_delay_cm(const uint8_t cm) {
    meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
  }

  /**
   * Convert Filament Width (mm) to an extrusion ratio
   * and reduce to an 8 bit value.
   *
   * A nominal width of 1.75 and measured width of 1.73
   * gives (100 * 1.75 / 1.73) for a ratio of 101 and
   * a return value of 1.
   */
  static int8_t sample_to_size_ratio() {
    return ABS(nominal_mm - measured_mm) <= FILWIDTH_ERROR_MARGIN
           ? int(100.0f * nominal_mm / measured_mm) - 100 : 0;
  }

  // Apply a single ADC reading to the raw value//对原始值应用单个ADC读数
  static void accumulate(const uint16_t adc) {
    if (adc > 102)  // Ignore ADC under 0.5 volts//忽略0.5伏以下的ADC
      accum += (uint32_t(adc) << 7) - (accum >> 7);
  }

  // Convert raw measurement to mm//将原始测量值转换为毫米
  static inline float raw_to_mm(const uint16_t v) { return v * 5.0f * RECIPROCAL(float(MAX_RAW_THERMISTOR_VALUE)); }
  static inline float raw_to_mm() { return raw_to_mm(raw); }

  // A scaled reading is ready//刻度读数已准备就绪
  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach//因为我们使用了1/128 IIR滤波器方法，所以将其除以以获得0-16384的范围
  static inline void reading_ready() { raw = accum >> 10; }

  // Update mm from the raw measurement//从原始测量值更新mm
  static inline void update_measured_mm() { measured_mm = raw_to_mm(); }

  // Update ring buffer used to delay filament measurements//更新用于延迟灯丝测量的环形缓冲器
  static inline void advance_e(const_float_t e_move) {

    // Increment counters with the E distance//使用E距离递增计数器
    e_count += e_move;
    delay_dist += e_move;

    // Only get new measurements on forward E movement//仅获取前进E运动的新测量值
    if (!UNEAR_ZERO(e_count)) {

      // Loop the delay distance counter (modulus by the mm length)//循环延迟距离计数器（模数乘以mm长度）
      while (delay_dist >= MMD_MM) delay_dist -= MMD_MM;

      // Convert into an index (cm) into the measurement array//将索引（cm）转换为测量阵列
      index_r = int8_t(delay_dist * 0.1f);

      // If the ring buffer is not full...//如果环形缓冲区未满。。。
      if (index_r != index_w) {
        e_count = 0;                            // Reset the E movement counter//重置E移动计数器
        const int8_t meas_sample = sample_to_size_ratio();
        do {
          if (++index_w >= MMD_CM) index_w = 0; // The next unused slot//下一个未使用的插槽
          ratios[index_w] = meas_sample;        // Store the measurement//存储测量值
        } while (index_r != index_w);           // More slots to fill?//还要填补更多的空缺吗？
      }
    }
  }

  // Dynamically set the volumetric multiplier based on the delayed width measurement.//基于延迟宽度测量动态设置体积倍增器。
  static inline void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r - meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed//如果需要，循环缓冲区
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
      planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }

};

extern FilamentWidthSensor filwidth;
