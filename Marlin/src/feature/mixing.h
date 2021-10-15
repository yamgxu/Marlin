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

//#define MIXER_NORMALIZER_DEBUG//#定义混频器\规格化器\调试

#ifndef __AVR__ // || HAS_DUAL_MIXING//| |有|双|混合
  // Use 16-bit (or fastest) data for the integer mix factors//使用16位（或最快）数据作为整数混合因子
  typedef uint_fast16_t mixer_comp_t;
  typedef uint_fast16_t mixer_accu_t;
  #define COLOR_A_MASK 0x8000
  #define COLOR_MASK 0x7FFF
#else
  // Use 8-bit data for the integer mix factors//使用8位数据作为整数混合因子
  // Exactness is sacrificed for speed//精确性是为了速度而牺牲的
  #define MIXER_ACCU_SIGNED
  typedef uint8_t mixer_comp_t;
  typedef int8_t mixer_accu_t;
  #define COLOR_A_MASK 0x80
  #define COLOR_MASK 0x7F
#endif

typedef int8_t mixer_perc_t;

#ifndef MIXING_VIRTUAL_TOOLS
  #define MIXING_VIRTUAL_TOOLS 1
#endif

enum MixTool {
    FIRST_USER_VIRTUAL_TOOL = 0
  , LAST_USER_VIRTUAL_TOOL = MIXING_VIRTUAL_TOOLS - 1
  , NR_USER_VIRTUAL_TOOLS
  , MIXER_DIRECT_SET_TOOL = NR_USER_VIRTUAL_TOOLS
  #if HAS_MIXER_SYNC_CHANNEL
    , MIXER_AUTORETRACT_TOOL
  #endif
  , NR_MIXING_VIRTUAL_TOOLS
};

#define MAX_VTOOLS TERN(HAS_MIXER_SYNC_CHANNEL, 254, 255)
static_assert(NR_MIXING_VIRTUAL_TOOLS <= MAX_VTOOLS, "MIXING_VIRTUAL_TOOLS must be <= " STRINGIFY(MAX_VTOOLS) "!");

#define MIXER_STEPPER_LOOP(VAR) for (uint_fast8_t VAR = 0; VAR < MIXING_STEPPERS; VAR++)

#if ENABLED(GRADIENT_MIX)

  typedef struct {
    bool enabled;                         // This gradient is enabled//此渐变已启用
    mixer_comp_t color[MIXING_STEPPERS];  // The current gradient color//当前渐变颜色
    float start_z, end_z;                 // Region for gradient//梯度区域
    int8_t start_vtool, end_vtool;        // Start and end virtual tools//开始和结束虚拟工具
    mixer_perc_t start_mix[MIXING_STEPPERS],  // Start and end mixes from those tools//从这些工具开始和结束混合
                 end_mix[MIXING_STEPPERS];
    #if ENABLED(GRADIENT_VTOOL)
      int8_t vtool_index;                 // Use this virtual tool number as index//使用此虚拟刀具编号作为索引
    #endif
  } gradient_t;

#endif

/**
 * @brief Mixer class
 * @details Contains data and behaviors for a Mixing Extruder
 */
class Mixer {
  public:

  static float collector[MIXING_STEPPERS];    // M163 components, also editable from LCD//M163组件，也可从LCD编辑

  static void init(); // Populate colors at boot time//在启动时填充颜色

  static void reset_vtools();
  static void refresh_collector(const float proportion=1.0, const uint8_t t=selected_vtool, float (&c)[MIXING_STEPPERS]=collector);

  // Used up to Planner level//用于计划员级别
  FORCE_INLINE static void set_collector(const uint8_t c, const float f) { collector[c] = _MAX(f, 0.0f); }

  static void normalize(const uint8_t tool_index);
  FORCE_INLINE static void normalize() { normalize(selected_vtool); }

  FORCE_INLINE static uint8_t get_current_vtool() { return selected_vtool; }

  FORCE_INLINE static void T(const uint_fast8_t c) {
    selected_vtool = c;
    TERN_(GRADIENT_VTOOL, refresh_gradient());
    TERN_(HAS_DUAL_MIXING, update_mix_from_vtool());
  }

  // Used when dealing with blocks//在处理块时使用
  FORCE_INLINE static void populate_block(mixer_comp_t b_color[MIXING_STEPPERS]) {
    #if ENABLED(GRADIENT_MIX)
      if (gradient.enabled) {
        MIXER_STEPPER_LOOP(i) b_color[i] = gradient.color[i];
        return;
      }
    #endif
    MIXER_STEPPER_LOOP(i) b_color[i] = color[selected_vtool][i];
  }

  FORCE_INLINE static void stepper_setup(mixer_comp_t b_color[MIXING_STEPPERS]) {
    MIXER_STEPPER_LOOP(i) s_color[i] = b_color[i];
  }

  #if EITHER(HAS_DUAL_MIXING, GRADIENT_MIX)

    static mixer_perc_t mix[MIXING_STEPPERS];  // Scratch array for the Mix in proportion to 100//按比例为100的混合物的刮擦阵列

    static inline void copy_mix_to_color(mixer_comp_t (&tcolor)[MIXING_STEPPERS]) {
      // Scale each component to the largest one in terms of COLOR_A_MASK//根据颜色和遮罩将每个组件缩放为最大的组件
      // So the largest component will be COLOR_A_MASK and the other will be in proportion to it//因此，最大的组成部分将是彩色遮罩，另一个将与之成比例
      const float scale = (COLOR_A_MASK) * RECIPROCAL(_MAX(
        LIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5])
      ));

      // Scale all values so their maximum is COLOR_A_MASK//缩放所有值，使其最大值为颜色遮罩
      MIXER_STEPPER_LOOP(i) tcolor[i] = mix[i] * scale;

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOPGM("Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]);
        SERIAL_ECHOPGM(" ] to Color [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, tcolor[0], tcolor[1], tcolor[2], tcolor[3], tcolor[4], tcolor[5]);
        SERIAL_ECHOLNPGM(" ]");
      #endif
    }

    static inline void update_mix_from_vtool(const uint8_t j=selected_vtool) {
      float ctot = 0;
      MIXER_STEPPER_LOOP(i) ctot += color[j][i];
      //MIXER_STEPPER_LOOP(i) mix[i] = 100.0f * color[j][i] / ctot;//混合器步进器环路（i）混合[i]=100.0f*颜色[j][i]/ctot；
      MIXER_STEPPER_LOOP(i) mix[i] = mixer_perc_t(100.0f * color[j][i] / ctot);

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOPAIR("V-tool ", j, " [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, color[j][0], color[j][1], color[j][2], color[j][3], color[j][4], color[j][5]);
        SERIAL_ECHOPGM(" ] to Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]);
        SERIAL_ECHOLNPGM(" ]");
      #endif
    }

  #endif // HAS_DUAL_MIXING || GRADIENT_MIX//具有|双|混合|梯度|混合

  #if HAS_DUAL_MIXING

    // Update the virtual tool from an edited mix//从已编辑的组合更新虚拟工具
    static inline void update_vtool_from_mix() {
      copy_mix_to_color(color[selected_vtool]);
      TERN_(GRADIENT_MIX, refresh_gradient());
      // MIXER_STEPPER_LOOP(i) collector[i] = mix[i];//混频器步进器环路（i）采集器[i]=mix[i]；
      // normalize();//规范化（）；
    }

  #endif // HAS_DUAL_MIXING//有双重混合吗

  #if ENABLED(GRADIENT_MIX)

    static gradient_t gradient;
    static float prev_z;

    // Update the current mix from the gradient for a given Z//根据给定Z的渐变更新当前混合
    static void update_gradient_for_z(const_float_t z);
    static void update_gradient_for_planner_z();
    static inline void gradient_control(const_float_t z) {
      if (gradient.enabled) {
        if (z >= gradient.end_z)
          T(gradient.end_vtool);
        else
          update_gradient_for_z(z);
      }
    }

    static inline void update_mix_from_gradient() {
      float ctot = 0;
      MIXER_STEPPER_LOOP(i) ctot += gradient.color[i];
      MIXER_STEPPER_LOOP(i) mix[i] = (mixer_perc_t)CEIL(100.0f * gradient.color[i] / ctot);

      #ifdef MIXER_NORMALIZER_DEBUG
        SERIAL_ECHOPGM("Gradient [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, gradient.color[0], gradient.color[1], gradient.color[2], gradient.color[3], gradient.color[4], gradient.color[5]);
        SERIAL_ECHOPGM(" ] to Mix [ ");
        SERIAL_ECHOLIST_N(MIXING_STEPPERS, mix[0], mix[1], mix[2], mix[3], mix[4], mix[5]);
        SERIAL_ECHOLNPGM(" ]");
      #endif
    }

    // Refresh the gradient after a change//更改后刷新渐变
    static void refresh_gradient() {
      #if ENABLED(GRADIENT_VTOOL)
        const bool is_grd = (gradient.vtool_index == -1 || selected_vtool == (uint8_t)gradient.vtool_index);
      #else
        constexpr bool is_grd = true;
      #endif
      gradient.enabled = is_grd && gradient.start_vtool != gradient.end_vtool && gradient.start_z < gradient.end_z;
      if (gradient.enabled) {
        mixer_perc_t mix_bak[MIXING_STEPPERS];
        COPY(mix_bak, mix);
        update_mix_from_vtool(gradient.start_vtool);
        COPY(gradient.start_mix, mix);
        update_mix_from_vtool(gradient.end_vtool);
        COPY(gradient.end_mix, mix);
        update_gradient_for_planner_z();
        COPY(mix, mix_bak);
        prev_z = -1;
      }
    }

  #endif // GRADIENT_MIX//梯度混合

  // Used in Stepper//用于步进电机
  FORCE_INLINE static uint8_t get_stepper() { return runner; }
  FORCE_INLINE static uint8_t get_next_stepper() {
    for (;;) {
      if (--runner < 0) runner = MIXING_STEPPERS - 1;
      accu[runner] += s_color[runner];
      if (
        #ifdef MIXER_ACCU_SIGNED
          accu[runner] < 0
        #else
          accu[runner] & COLOR_A_MASK
        #endif
      ) {
        accu[runner] &= COLOR_MASK;
        return runner;
      }
    }
  }

  private:

  // Used up to Planner level//用于计划员级别
  static uint_fast8_t selected_vtool;
  static mixer_comp_t color[NR_MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];

  // Used in Stepper//用于步进电机
  static int_fast8_t  runner;
  static mixer_comp_t s_color[MIXING_STEPPERS];
  static mixer_accu_t accu[MIXING_STEPPERS];
};

extern Mixer mixer;
