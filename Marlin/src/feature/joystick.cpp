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

/**
 * joystick.cpp - joystick input / jogging
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(JOYSTICK)

#include "joystick.h"

#include "../inc/MarlinConfig.h"  // for pins//别针
#include "../module/planner.h"

Joystick joystick;

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#if HAS_JOY_ADC_X
  temp_info_t Joystick::x; // = { 0 }// = { 0 }
  #if ENABLED(INVERT_JOY_X)
    #define JOY_X(N) (16383 - (N))
  #else
    #define JOY_X(N) (N)
  #endif
#endif
#if HAS_JOY_ADC_Y
  temp_info_t Joystick::y; // = { 0 }// = { 0 }
  #if ENABLED(INVERT_JOY_Y)
    #define JOY_Y(N) (16383 - (N))
  #else
    #define JOY_Y(N) (N)
  #endif
#endif
#if HAS_JOY_ADC_Z
  temp_info_t Joystick::z; // = { 0 }// = { 0 }
  #if ENABLED(INVERT_JOY_Z)
    #define JOY_Z(N) (16383 - (N))
  #else
    #define JOY_Z(N) (N)
  #endif
#endif

#if ENABLED(JOYSTICK_DEBUG)
  void Joystick::report() {
    SERIAL_ECHOPGM("Joystick");
    #if HAS_JOY_ADC_X
      SERIAL_ECHOPAIR_P(SP_X_STR, JOY_X(x.raw));
    #endif
    #if HAS_JOY_ADC_Y
      SERIAL_ECHOPAIR_P(SP_Y_STR, JOY_Y(y.raw));
    #endif
    #if HAS_JOY_ADC_Z
      SERIAL_ECHOPAIR_P(SP_Z_STR, JOY_Z(z.raw));
    #endif
    #if HAS_JOY_ADC_EN
      SERIAL_ECHO_TERNARY(READ(JOY_EN_PIN), " EN=", "HIGH (dis", "LOW (en", "abled)");
    #endif
    SERIAL_EOL();
  }
#endif

#if HAS_JOY_ADC_X || HAS_JOY_ADC_Y || HAS_JOY_ADC_Z

  void Joystick::calculate(xyz_float_t &norm_jog) {
    // Do nothing if enable pin (active-low) is not LOW//如果启用引脚（低电平有效）不低，则不执行任何操作
    #if HAS_JOY_ADC_EN
      if (READ(JOY_EN_PIN)) return;
    #endif

    auto _normalize_joy = [](float &axis_jog, const int16_t raw, const int16_t (&joy_limits)[4]) {
      if (WITHIN(raw, joy_limits[0], joy_limits[3])) {
        // within limits, check deadzone//在限制范围内，检查死区
        if (raw > joy_limits[2])
          axis_jog = (raw - joy_limits[2]) / float(joy_limits[3] - joy_limits[2]);
        else if (raw < joy_limits[1])
          axis_jog = (raw - joy_limits[1]) / float(joy_limits[1] - joy_limits[0]);  // negative value//负值
        // Map normal to jog value via quadratic relationship//通过二次关系将法线映射到jog值
        axis_jog = SIGN(axis_jog) * sq(axis_jog);
      }
    };

    #if HAS_JOY_ADC_X
      static constexpr int16_t joy_x_limits[4] = JOY_X_LIMITS;
      _normalize_joy(norm_jog.x, JOY_X(x.raw), joy_x_limits);
    #endif
    #if HAS_JOY_ADC_Y
      static constexpr int16_t joy_y_limits[4] = JOY_Y_LIMITS;
      _normalize_joy(norm_jog.y, JOY_Y(y.raw), joy_y_limits);
    #endif
    #if HAS_JOY_ADC_Z
      static constexpr int16_t joy_z_limits[4] = JOY_Z_LIMITS;
      _normalize_joy(norm_jog.z, JOY_Z(z.raw), joy_z_limits);
    #endif
  }

#endif

#if ENABLED(POLL_JOG)

  void Joystick::inject_jog_moves() {
    // Recursion barrier//递归屏障
    static bool injecting_now; // = false;//=假；
    if (injecting_now) return;

    #if ENABLED(NO_MOTION_BEFORE_HOMING)
      if (TERN0(HAS_JOY_ADC_X, axis_should_home(X_AXIS)) || TERN0(HAS_JOY_ADC_Y, axis_should_home(Y_AXIS)) || TERN0(HAS_JOY_ADC_Z, axis_should_home(Z_AXIS)))
        return;
    #endif

    static constexpr int QUEUE_DEPTH = 5;                                // Insert up to this many movements//最多插入这么多动作
    static constexpr float target_lag = 0.25f,                           // Aim for 1/4 second lag//以1/4秒的延迟为目标
                           seg_time = target_lag / QUEUE_DEPTH;          // 0.05 seconds, short segments inserted every 1/20th of a second//0.05秒，每1/20秒插入一小段
    static constexpr millis_t timer_limit_ms = millis_t(seg_time * 500); // 25 ms minimum delay between insertions//插入之间的最小延迟为25毫秒

    // The planner can merge/collapse small moves, so the movement queue is unreliable to control the lag//计划者可以合并/折叠小的移动，因此移动队列不可靠，无法控制延迟
    static millis_t next_run = 0;
    if (PENDING(millis(), next_run)) return;
    next_run = millis() + timer_limit_ms;

    // Only inject a command if the planner has fewer than 5 moves and there are no unparsed commands//仅当规划器的移动次数少于5次且没有未分析的命令时，才注入命令
    if (planner.movesplanned() >= QUEUE_DEPTH || queue.has_commands_queued())
      return;

    // Normalized jog values are 0 for no movement and -1 or +1 for as max feedrate (nonlinear relationship)//标准化jog值为0表示无移动，为-1或+1表示as最大进给速率（非线性关系）
    // Jog are initialized to zero and handling input can update values but doesn't have to//Jog初始化为零，处理输入可以更新值，但不必更新
    // You could use a two-axis joystick and a one-axis keypad and they might work together//你可以使用双轴操纵杆和单轴键盘，它们可以一起工作
    xyz_float_t norm_jog{0};

    // Use ADC values and defined limits. The active zone is normalized: -1..0 (dead) 0..1//使用ADC值和定义的限制。活动区域已标准化：-1..0（死）0..1
    #if HAS_JOY_ADC_X || HAS_JOY_ADC_Y || HAS_JOY_ADC_Z
      joystick.calculate(norm_jog);
    #endif

    // Other non-joystick poll-based jogging could be implemented here//其他非操纵杆民意测验的慢跑可以在这里实施
    // with "jogging" encapsulated as a more general class.//将“慢跑”封装为更一般的类。

    TERN_(EXTENSIBLE_UI, ExtUI::_joystick_update(norm_jog));

    // norm_jog values of [-1 .. 1] maps linearly to [-feedrate .. feedrate]//[-1..1]的norm_jog值与[-feedrate..feedrate]呈线性映射
    xyz_float_t move_dist{0};
    float hypot2 = 0;
    LOOP_LINEAR_AXES(i) if (norm_jog[i]) {
      move_dist[i] = seg_time * norm_jog[i] * TERN(EXTENSIBLE_UI, manual_feedrate_mm_s, planner.settings.max_feedrate_mm_s)[i];
      hypot2 += sq(move_dist[i]);
    }

    if (!UNEAR_ZERO(hypot2)) {
      current_position += move_dist;
      apply_motion_limits(current_position);
      const float length = sqrt(hypot2);
      injecting_now = true;
      planner.buffer_line(current_position, length / seg_time, active_extruder, length);
      injecting_now = false;
    }
  }

#endif // POLL_JOG//慢跑

#endif // JOYSTICK//操纵杆
