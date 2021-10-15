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

/**
 * planner.h
 *
 * Buffer movement commands and manage the acceleration profile plan
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 */

#include "../MarlinCore.h"

#if ENABLED(JD_HANDLE_SMALL_SEGMENTS)
  // Enable this option for perfect accuracy but maximum//启用此选项可获得最佳精度，但最大
  // computation. Should be fine on ARM processors.//计算。在ARM处理器上应该可以。
  //#define JD_USE_MATH_ACOS//#定义JD\u使用\u数学\u ACOS

  // Disable this option to save 120 bytes of PROGMEM,//禁用此选项可保存120字节的程序，
  // but incur increased computation and a reduction//但会导致计算量增加和减少
  // in accuracy.//准确地说。
  #define JD_USE_LOOKUP_TABLE
#endif

#include "motion.h"
#include "../gcode/queue.h"

#if ENABLED(DELTA)
  #include "delta.h"
#endif

#if ABL_PLANAR
  #include "../libs/vector_3.h" // for matrix_3x3//对于矩阵_3x3
#endif

#if ENABLED(FWRETRACT)
  #include "../feature/fwretract.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if HAS_CUTTER
  #include "../feature/spindle_laser_types.h"
#endif

#if ENABLED(DIRECT_STEPPING)
  #include "../feature/direct_stepping.h"
  #define IS_PAGE(B) TEST(B->flag, BLOCK_BIT_IS_PAGE)
#else
  #define IS_PAGE(B) false
#endif

// Feedrate for manual moves//手动移动的进给速度
#ifdef MANUAL_FEEDRATE
  constexpr xyze_feedrate_t _mf = MANUAL_FEEDRATE,
           manual_feedrate_mm_s = LOGICAL_AXIS_ARRAY(_mf.e / 60.0f,
                                                     _mf.x / 60.0f, _mf.y / 60.0f, _mf.z / 60.0f,
                                                     _mf.i / 60.0f, _mf.j / 60.0f, _mf.k / 60.0f);
#endif

#if IS_KINEMATIC && HAS_JUNCTION_DEVIATION
  #define HAS_DIST_MM_ARG 1
#endif

enum BlockFlagBit : char {
  // Recalculate trapezoids on entry junction. For optimization.//重新计算入口连接处的梯形。用于优化。
  BLOCK_BIT_RECALCULATE,

  // Nominal speed always reached.//始终达到标称速度。
  // i.e., The segment is long enough, so the nominal speed is reachable if accelerating//也就是说，该段足够长，因此如果加速，可以达到标称速度
  // from a safe speed (in consideration of jerking from zero speed).//从安全速度（考虑从零速度猛拉）。
  BLOCK_BIT_NOMINAL_LENGTH,

  // The block is segment 2+ of a longer move//该块是较长移动的第2+段
  BLOCK_BIT_CONTINUED,

  // Sync the stepper counts from the block//从块同步步进器计数
  BLOCK_BIT_SYNC_POSITION

  // Direct stepping page//直接步进页面
  #if ENABLED(DIRECT_STEPPING)
    , BLOCK_BIT_IS_PAGE
  #endif

  // Sync the fan speeds from the block//从块同步风扇速度
  #if ENABLED(LASER_SYNCHRONOUS_M106_M107)
    , BLOCK_BIT_SYNC_FANS
  #endif
};

enum BlockFlag : char {
    BLOCK_FLAG_RECALCULATE          = _BV(BLOCK_BIT_RECALCULATE)
  , BLOCK_FLAG_NOMINAL_LENGTH       = _BV(BLOCK_BIT_NOMINAL_LENGTH)
  , BLOCK_FLAG_CONTINUED            = _BV(BLOCK_BIT_CONTINUED)
  , BLOCK_FLAG_SYNC_POSITION        = _BV(BLOCK_BIT_SYNC_POSITION)
  #if ENABLED(DIRECT_STEPPING)
    , BLOCK_FLAG_IS_PAGE            = _BV(BLOCK_BIT_IS_PAGE)
  #endif
  #if ENABLED(LASER_SYNCHRONOUS_M106_M107)
    , BLOCK_FLAG_SYNC_FANS          = _BV(BLOCK_BIT_SYNC_FANS)
  #endif
};

#define BLOCK_MASK_SYNC ( BLOCK_FLAG_SYNC_POSITION | TERN0(LASER_SYNCHRONOUS_M106_M107, BLOCK_FLAG_SYNC_FANS) )

#if ENABLED(LASER_POWER_INLINE)

  typedef struct {
    bool isPlanned:1;
    bool isEnabled:1;
    bool dir:1;
    bool Reserved:6;
  } power_status_t;

  typedef struct {
    power_status_t status;    // See planner settings for meaning//请参见规划器设置以了解其含义
    uint8_t power;            // Ditto; When in trapezoid mode this is nominal power//同上；在梯形模式下，这是标称功率
    #if ENABLED(LASER_POWER_INLINE_TRAPEZOID)
      uint8_t   power_entry;  // Entry power for the laser//激光器的进入功率
      #if DISABLED(LASER_POWER_INLINE_TRAPEZOID_CONT)
        uint8_t   power_exit; // Exit power for the laser//激光器的输出功率
        uint32_t  entry_per,  // Steps per power increment (to avoid floats in stepper calcs)//每功率增量步数（避免步进计算中出现浮动）
                  exit_per;   // Steps per power decrement//每功率递减步数
      #endif
    #endif
  } block_laser_t;

#endif

/**
 * struct block_t
 *
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 *
 * The "nominal" values are as-specified by gcode, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct block_t {

  volatile uint8_t flag;                    // Block flags (See BlockFlag enum above) - Modified by ISR and main thread!//块标志（请参阅上面的块标志枚举）-由ISR和主线程修改！

  //运动规划器用于管理加速度的字段//运动规划器用于管理加速度的字段
  float nominal_speed_sqr,                  // 该块的标称速度 (mm/sec)^2// 该块的标称速度 （毫米/秒）^2
        entry_speed_sqr,                    // 前一个当前结的进入速度（mm/sec）^2// 前一个当前结的进入速度（毫米/秒）^2
        max_entry_speed_sqr,                // 最大允许的结入口速度(mm/sec)^2// 最大允许的结入口速度（毫米/秒）^2
        millimeters,                        // 这个块的总行程，单位为毫米// 这个块的总行程，单位为毫米
        acceleration;                       // 加速度mm/sec ^ 2// 加速度毫米/秒^2

  union {
    abce_ulong_t steps;                     // 沿每个轴进行步长计数// 沿每个轴进行步长计数
    abce_long_t position;                   //执行同步块时要强制执行的新位置//执行同步块时要强制执行的新位置
  };
  uint32_t step_event_count;                // 完成此块所需的步骤事件数// 完成此块所需的步骤事件数

  #if HAS_MULTI_EXTRUDER //有多个挤出机//有多个挤出机
    uint8_t extruder;                       // The extruder to move (if E move)//要移动的挤出机（如果E移动）
  #else
    static constexpr uint8_t extruder = 0;
  #endif

  #if ENABLED(MIXING_EXTRUDER)//混合挤出机//混合挤出机
    mixer_comp_t b_color[MIXING_STEPPERS];  // Normalized color for the mixing steppers//混合步进器的标准化颜色
  #endif

  // 梯形发生器的设置// 梯形发生器的设置
  uint32_t accelerate_until,                //停止加速的步骤事件的索引//停止加速的步骤事件的索引
           decelerate_after;                //开始减速的步骤事件的索引//开始减速的步骤事件的索引

  #if ENABLED(S_CURVE_ACCELERATION)
    uint32_t cruise_rate,                   // The actual cruise rate to use, between end of the acceleration phase and start of deceleration phase//在加速阶段结束和减速阶段开始之间使用的实际巡航速度
             acceleration_time,             // Acceleration time and deceleration time in STEP timer counts//步进计时器计数中的加速时间和减速时间
             deceleration_time,
             acceleration_time_inverse,     // Inverse of acceleration and deceleration periods, expressed as integer. Scale depends on CPU being used//加速和减速周期的倒数，表示为整数。规模取决于所使用的CPU
             deceleration_time_inverse;
  #else
    uint32_t acceleration_rate;             // 用于加速度计算的加速度// 用于加速度计算的加速度
  #endif

  uint8_t direction_bits;                   //为该块设置的方向位（参考 config.h 中的 _DIRECTION_BIT）//为该块设置的方向位（参考 配置.h中的 _方向（U位）

  // 超前挤压// 超前挤压
  #if ENABLED(LIN_ADVANCE)
    bool use_advance_lead;
    uint16_t advance_speed,                 // STEP timer value for extruder speed offset ISR//挤出机速度偏移ISR的步进定时器值
             max_adv_steps,                 // max. advance steps to get cruising speed pressure (not always nominal_speed!)//获得巡航速度压力的最大前进步数（不总是标称速度！）
             final_adv_steps;               // advance steps due to exit speed//因退出速度而前进的步骤
    float e_D_ratio;
  #endif

  uint32_t nominal_rate,                    // The nominal step rate for this block in step_events/sec//此块的标称步进速率，单位为步进事件/秒
           initial_rate,                    // The jerk-adjusted step rate at start of block//试块开始时的挺举调整步进率
           final_rate,                      // The minimal rate at exit//退出时的最小速率
           acceleration_steps_per_s2;       // acceleration steps/sec^2//加速步数/秒^2

  #if ENABLED(DIRECT_STEPPING)
    page_idx_t page_idx;                    // Page index used for direct stepping//用于直接步进的页面索引
  #endif

  #if HAS_CUTTER
    cutter_power_t cutter_power;            // Power level for Spindle, Laser, etc.//主轴、激光器等的功率水平。
  #endif

  #if HAS_FAN
    uint8_t fan_speed[FAN_COUNT];
  #endif

  #if ENABLED(BARICUDA)
    uint8_t valve_pressure, e_to_p_pressure;
  #endif

  #if HAS_WIRED_LCD
    uint32_t segment_time_us;
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    uint32_t sdpos;
  #endif

  #if ENABLED(LASER_POWER_INLINE)
    block_laser_t laser;
  #endif

} block_t;

#if ANY(LIN_ADVANCE, SCARA_FEEDRATE_SCALING, GRADIENT_MIX, LCD_SHOW_E_TOTAL)
  #define HAS_POSITION_FLOAT 1
#endif

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

#if ENABLED(LASER_POWER_INLINE)
  typedef struct {
    /**
     * Laser status flags
     */
    power_status_t status;
    /**
     * Laser power: 0 or 255 in case of PWM-less laser,
     * or the OCR (oscillator count register) value;
     *
     * Using OCR instead of raw power, because it avoids
     * floating point operations during the move loop.
     */
    uint8_t power;
  } laser_state_t;
#endif

typedef struct {
   uint32_t max_acceleration_mm_per_s2[DISTINCT_AXES], // (mm/s^2) M201 XYZE//（毫米/秒^2）M201 XYZE
            min_segment_time_us;                // (µs) M205 B//（µs）M205 B
      float axis_steps_per_mm[DISTINCT_AXES];   // (steps) M92 XYZE - Steps per millimeter//（步数）M92 XYZE-每毫米步数
 feedRate_t max_feedrate_mm_s[DISTINCT_AXES];   // (mm/s) M203 XYZE - Max speeds//（毫米/秒）M203 XYZE-最大速度
      float acceleration,                       // (mm/s^2) M204 S - Normal acceleration. DEFAULT ACCELERATION for all printing moves.//（毫米/秒^2）M204秒-正常加速度。所有打印移动的默认加速。
            retract_acceleration,               // (mm/s^2) M204 R - Retract acceleration. Filament pull-back and push-forward while standing still in the other axes//（毫米/秒^2）M204 R-缩回加速度。灯丝向后拉并向前推，同时在其他轴上静止不动
            travel_acceleration;                // (mm/s^2) M204 T - Travel acceleration. DEFAULT ACCELERATION for all NON printing moves.//（毫米/秒^2）M204 T-行驶加速度。所有非打印移动的默认加速度。
 feedRate_t min_feedrate_mm_s,                  // (mm/s) M205 S - Minimum linear feedrate//（mm/s）M205 s-最小线性进给速度
            min_travel_feedrate_mm_s;           // (mm/s) M205 T - Minimum travel feedrate//（mm/s）M205 T-最小行程进给率
} planner_settings_t;

#if DISABLED(SKEW_CORRECTION)
  #define XY_SKEW_FACTOR 0
  #define XZ_SKEW_FACTOR 0
  #define YZ_SKEW_FACTOR 0
#endif

typedef struct {
  #if ENABLED(SKEW_CORRECTION_GCODE)
    float xy;
    #if ENABLED(SKEW_CORRECTION_FOR_Z)
      float xz, yz;
    #else
      const float xz = XZ_SKEW_FACTOR, yz = YZ_SKEW_FACTOR;
    #endif
  #else
    const float xy = XY_SKEW_FACTOR,
                xz = XZ_SKEW_FACTOR, yz = YZ_SKEW_FACTOR;
  #endif
} skew_factor_t;

#if ENABLED(DISABLE_INACTIVE_EXTRUDER)
  typedef IF<(BLOCK_BUFFER_SIZE > 64), uint16_t, uint8_t>::type last_move_t;
#endif

class Planner {
  public:

    /**
     * The move buffer, calculated in stepper steps
     *
     * block_buffer is a ring buffer...
     *
     *             head,tail : indexes for write,read
     *            head==tail : the buffer is empty
     *            head!=tail : blocks are in the buffer
     *   head==(tail-1)%size : the buffer is full
     *
     *  Writer of head is Planner::buffer_segment().
     *  Reader of tail is Stepper::isr(). Always consider tail busy / read-only
     */
    static block_t block_buffer[BLOCK_BUFFER_SIZE];
    static volatile uint8_t block_buffer_head,      // Index of the next block to be pushed//要推送的下一个块的索引
                            block_buffer_nonbusy,   // Index of the first non busy block//第一个非忙块的索引
                            block_buffer_planned,   // Index of the optimally planned block//最优规划块的索引
                            block_buffer_tail;      // Index of the busy block, if any//忙块的索引（如果有）
    static uint16_t cleaning_buffer_counter;        // A counter to disable queuing of blocks//用于禁用块排队的计数器
    static uint8_t delay_before_delivering;         // This counter delays delivery of blocks when queue becomes empty to allow the opportunity of merging blocks//当队列变空时，此计数器延迟块的传递，以便有机会合并块


    #if ENABLED(DISTINCT_E_FACTORS)
      static uint8_t last_extruder;                 // Respond to extruder change//对挤出机变化的响应
    #endif

    #if ENABLED(DIRECT_STEPPING)
      static uint32_t last_page_step_rate;          // Last page step rate given//最后一页步进率给定
      static xyze_bool_t last_page_dir;             // Last page direction given//最后一页给出了方向
    #endif

    #if HAS_EXTRUDERS
      static int16_t flow_percentage[EXTRUDERS];    // Extrusion factor for each extruder//每台挤出机的挤出系数
      static float e_factor[EXTRUDERS];             // The flow percentage and volumetric multiplier combine to scale E movement//流量百分比和体积倍增器结合起来可缩放E移动
    #endif

    #if DISABLED(NO_VOLUMETRICS)
      static float filament_size[EXTRUDERS],          // diameter of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder//灯丝直径（以毫米为单位），通常约为1.75或2.85，0禁用挤出机的体积计算
                   volumetric_area_nominal,           // Nominal cross-sectional area//标称横截面积
                   volumetric_multiplier[EXTRUDERS];  // Reciprocal of cross-sectional area of filament (in mm^2). Pre-calculated to reduce computation in the planner//灯丝横截面积的倒数（单位：mm^2）。预先计算以减少计划器中的计算
                                                      // May be auto-adjusted by a filament width sensor//可通过灯丝宽度传感器自动调整
    #endif

    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      static float volumetric_extruder_limit[EXTRUDERS],          // Maximum mm^3/sec the extruder can handle//挤出机可处理的最大速度为mm^3/秒
                   volumetric_extruder_feedrate_limit[EXTRUDERS]; // Feedrate limit (mm/s) calculated from volume limit//根据体积限制计算的进给速度限制（mm/s）
    #endif

    static planner_settings_t settings;

    #if ENABLED(LASER_POWER_INLINE)
      static laser_state_t laser_inline;
    #endif

    static uint32_t max_acceleration_steps_per_s2[DISTINCT_AXES]; // (steps/s^2) Derived from mm_per_s2//（步数/s^2）从mm_/秒2派生
    static float steps_to_mm[DISTINCT_AXES];          // Millimeters per step//每步毫米数

    #if HAS_JUNCTION_DEVIATION
      static float junction_deviation_mm;             // (mm) M205 J//（毫米）M205 J
      #if HAS_LINEAR_E_JERK
        static float max_e_jerk[DISTINCT_E];          // Calculated from junction_deviation_mm//根据连接处偏差计算
      #endif
    #endif

    #if HAS_CLASSIC_JERK
      // (mm/s^2) M205 XYZ(E) - The largest speed change requiring no acceleration.//（mm/s^2）M205 XYZ（E）-无需加速的最大速度变化。
      static TERN(HAS_LINEAR_E_JERK, xyz_pos_t, xyze_pos_t) max_jerk;
    #endif

    #if HAS_LEVELING
      static bool leveling_active;          // Flag that bed leveling is enabled//启用河床平整的标志
      #if ABL_PLANAR
        static matrix_3x3 bed_level_matrix; // Transform to compensate for bed level//变换以补偿床面高度
      #endif
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        static float z_fade_height, inverse_z_fade_height;
      #endif
    #else
      static constexpr bool leveling_active = false;
    #endif

    #if ENABLED(LIN_ADVANCE)
      static float extruder_advance_K[EXTRUDERS];
    #endif

    /**
     * The current position of the tool in absolute steps
     * Recalculated if any axis_steps_per_mm are changed by gcode
     */
    static xyze_long_t position;

    #if HAS_POSITION_FLOAT
      static xyze_pos_t position_float;
    #endif

    #if IS_KINEMATIC
      static xyze_pos_t position_cart;
    #endif

    static skew_factor_t skew_factor;

    #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
      static bool abort_on_endstop_hit;
    #endif
    #ifdef XY_FREQUENCY_LIMIT
      static int8_t xy_freq_limit_hz;         // Minimum XY frequency setting//最小XY频率设置
      static float xy_freq_min_speed_factor;  // Minimum speed factor setting//最小速度系数设置
      static int32_t xy_freq_min_interval_us; // Minimum segment time based on xy_freq_limit_hz//基于xy\u频率\u限制\u hz的最小分段时间
      static inline void refresh_frequency_limit() {
        //xy_freq_min_interval_us = xy_freq_limit_hz ?: LROUND(1000000.0f / xy_freq_limit_hz);//xy\u频率\u最小间隔\u us=xy\u频率限制\u hz？：LROUND（1000000.0f/xy\u频率限制\u hz）；
        if (xy_freq_limit_hz)
          xy_freq_min_interval_us = LROUND(1000000.0f / xy_freq_limit_hz);
      }
      static inline void set_min_speed_factor_u8(const uint8_t v255) {
        xy_freq_min_speed_factor = float(ui8_to_percent(v255)) / 100;
      }
      static inline void set_frequency_limit(const uint8_t hz) {
        xy_freq_limit_hz = constrain(hz, 0, 100);
        refresh_frequency_limit();
      }
    #endif

  private:

    /**
     * Speed of previous path line segment
     */
    static xyze_float_t previous_speed;

    /**
     * Nominal speed of previous path line segment (mm/s)^2
     */
    static float previous_nominal_speed_sqr;

    /**
     * Limit where 64bit math is necessary for acceleration calculation
     */
    static uint32_t acceleration_long_cutoff;

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      static float last_fade_z;
    #endif

    #if ENABLED(DISABLE_INACTIVE_EXTRUDER)
      // Counters to manage disabling inactive extruder steppers//用于管理禁用非活动挤出机步进机的计数器
      static last_move_t g_uc_extruder_last_move[E_STEPPERS];
    #endif

    #if HAS_WIRED_LCD
      volatile static uint32_t block_buffer_runtime_us; // Theoretical block buffer runtime in µs//以µs为单位的理论块缓冲运行时间
    #endif

  public:

    /**
     * Instance Methods
     */

    Planner();

    void init();

    /**
     * Static (class) Methods
     */

    // Recalculate steps/s^2 accelerations based on mm/s^2 settings//根据mm/s^2设置重新计算步长/s^2加速度
    static void reset_acceleration_rates();

    /**
     * Recalculate 'position' and 'steps_to_mm'.
     * Must be called whenever settings.axis_steps_per_mm changes!
     */
    static void refresh_positioning();

    // For an axis set the Maximum Acceleration in mm/s^2//对于轴，设置最大加速度，单位为mm/s^2
    static void set_max_acceleration(const uint8_t axis, float inMaxAccelMMS2);

    // For an axis set the Maximum Feedrate in mm/s//对于轴，以毫米/秒为单位设置最大进给速度
    static void set_max_feedrate(const uint8_t axis, float inMaxFeedrateMMS);

    // For an axis set the Maximum Jerk (instant change) in mm/s//对于轴，以毫米/秒为单位设置最大脉动（瞬时变化）
    #if HAS_CLASSIC_JERK
      static void set_max_jerk(const AxisEnum axis, float inMaxJerkMMS);
    #else
      static inline void set_max_jerk(const AxisEnum, const_float_t) {}
    #endif

    #if HAS_EXTRUDERS
      FORCE_INLINE static void refresh_e_factor(const uint8_t e) {
        e_factor[e] = flow_percentage[e] * 0.01f * TERN(NO_VOLUMETRICS, 1.0f, volumetric_multiplier[e]);
      }

      static inline void set_flow(const uint8_t e, const int16_t flow) {
        flow_percentage[e] = flow;
        refresh_e_factor(e);
      }

    #endif

    // Manage fans, paste pressure, etc.//管理风扇、粘贴压力等。
    static void check_axes_activity();

    // Apply fan speeds//应用风扇转速
    #if HAS_FAN
      static void sync_fan_speeds(uint8_t (&fan_speed)[FAN_COUNT]);
      #if FAN_KICKSTART_TIME
        static void kickstart_fan(uint8_t (&fan_speed)[FAN_COUNT], const millis_t &ms, const uint8_t f);
      #else
        FORCE_INLINE static void kickstart_fan(uint8_t (&)[FAN_COUNT], const millis_t &, const uint8_t) {}
      #endif
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      void apply_filament_width_sensor(const int8_t encoded_ratio);

      static inline float volumetric_percent(const bool vol) {
        return 100.0f * (vol
            ? volumetric_area_nominal / volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
            : volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
        );
      }
    #endif

    #if DISABLED(NO_VOLUMETRICS)

      // Update multipliers based on new diameter measurements//根据新的直径测量值更新乘数
      static void calculate_volumetric_multipliers();

      #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
        // Update pre calculated extruder feedrate limits based on volumetric values//根据体积值更新预先计算的挤出机进给速度限制
        static void calculate_volumetric_extruder_limit(const uint8_t e);
        static void calculate_volumetric_extruder_limits();
      #endif

      FORCE_INLINE static void set_filament_size(const uint8_t e, const_float_t v) {
        filament_size[e] = v;
        if (v > 0) volumetric_area_nominal = CIRCLE_AREA(v * 0.5); //TODO: should it be per extruder//TODO:应该是每个挤出机吗
        // make sure all extruders have some sane value for the filament size//确保所有挤出机的长丝尺寸都有合理的值
        LOOP_L_N(i, COUNT(filament_size))
          if (!filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
      }

    #endif

    #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
      FORCE_INLINE static void set_volumetric_extruder_limit(const uint8_t e, const_float_t v) {
        volumetric_extruder_limit[e] = v;
        calculate_volumetric_extruder_limit(e);
      }
    #endif

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)

      /**
       * Get the Z leveling fade factor based on the given Z height,
       * re-calculating only when needed.
       *
       *  Returns 1.0 if planner.z_fade_height is 0.0.
       *  Returns 0.0 if Z is past the specified 'Fade Height'.
       */
      static inline float fade_scaling_factor_for_z(const_float_t rz) {
        static float z_fade_factor = 1;
        if (!z_fade_height) return 1;
        if (rz >= z_fade_height) return 0;
        if (last_fade_z != rz) {
          last_fade_z = rz;
          z_fade_factor = 1 - rz * inverse_z_fade_height;
        }
        return z_fade_factor;
      }

      FORCE_INLINE static void force_fade_recalc() { last_fade_z = -999.999f; }

      FORCE_INLINE static void set_z_fade_height(const_float_t zfh) {
        z_fade_height = zfh > 0 ? zfh : 0;
        inverse_z_fade_height = RECIPROCAL(z_fade_height);
        force_fade_recalc();
      }

      FORCE_INLINE static bool leveling_active_at_z(const_float_t rz) {
        return !z_fade_height || rz < z_fade_height;
      }

    #else

      FORCE_INLINE static float fade_scaling_factor_for_z(const_float_t) { return 1; }

      FORCE_INLINE static bool leveling_active_at_z(const_float_t) { return true; }

    #endif

    #if ENABLED(SKEW_CORRECTION)

      FORCE_INLINE static void skew(float &cx, float &cy, const_float_t cz) {
        if (COORDINATE_OKAY(cx, X_MIN_POS + 1, X_MAX_POS) && COORDINATE_OKAY(cy, Y_MIN_POS + 1, Y_MAX_POS)) {
          const float sx = cx - cy * skew_factor.xy - cz * (skew_factor.xz - (skew_factor.xy * skew_factor.yz)),
                      sy = cy - cz * skew_factor.yz;
          if (COORDINATE_OKAY(sx, X_MIN_POS, X_MAX_POS) && COORDINATE_OKAY(sy, Y_MIN_POS, Y_MAX_POS)) {
            cx = sx; cy = sy;
          }
        }
      }
      FORCE_INLINE static void skew(xyz_pos_t &raw) { skew(raw.x, raw.y, raw.z); }

      FORCE_INLINE static void unskew(float &cx, float &cy, const_float_t cz) {
        if (COORDINATE_OKAY(cx, X_MIN_POS, X_MAX_POS) && COORDINATE_OKAY(cy, Y_MIN_POS, Y_MAX_POS)) {
          const float sx = cx + cy * skew_factor.xy + cz * skew_factor.xz,
                      sy = cy + cz * skew_factor.yz;
          if (COORDINATE_OKAY(sx, X_MIN_POS, X_MAX_POS) && COORDINATE_OKAY(sy, Y_MIN_POS, Y_MAX_POS)) {
            cx = sx; cy = sy;
          }
        }
      }
      FORCE_INLINE static void unskew(xyz_pos_t &raw) { unskew(raw.x, raw.y, raw.z); }

    #endif // SKEW_CORRECTION//倾斜校正

    #if HAS_LEVELING
      /**
       * Apply leveling to transform a cartesian position
       * as it will be given to the planner and steppers.
       */
      static void apply_leveling(xyz_pos_t &raw);
      static void unapply_leveling(xyz_pos_t &raw);
      FORCE_INLINE static void force_unapply_leveling(xyz_pos_t &raw) {
        leveling_active = true;
        unapply_leveling(raw);
        leveling_active = false;
      }
    #else
      FORCE_INLINE static void apply_leveling(xyz_pos_t&) {}
      FORCE_INLINE static void unapply_leveling(xyz_pos_t&) {}
    #endif

    #if ENABLED(FWRETRACT)
      static void apply_retract(float &rz, float &e);
      FORCE_INLINE static void apply_retract(xyze_pos_t &raw) { apply_retract(raw.z, raw.e); }
      static void unapply_retract(float &rz, float &e);
      FORCE_INLINE static void unapply_retract(xyze_pos_t &raw) { unapply_retract(raw.z, raw.e); }
    #endif

    #if HAS_POSITION_MODIFIERS
      FORCE_INLINE static void apply_modifiers(xyze_pos_t &pos, bool leveling=ENABLED(PLANNER_LEVELING)) {
        TERN_(SKEW_CORRECTION, skew(pos));
        if (leveling) apply_leveling(pos);
        TERN_(FWRETRACT, apply_retract(pos));
      }

      FORCE_INLINE static void unapply_modifiers(xyze_pos_t &pos, bool leveling=ENABLED(PLANNER_LEVELING)) {
        TERN_(FWRETRACT, unapply_retract(pos));
        if (leveling) unapply_leveling(pos);
        TERN_(SKEW_CORRECTION, unskew(pos));
      }
    #endif // HAS_POSITION_MODIFIERS//有位置修改器

    // Number of moves currently in the planner including the busy block, if any//当前在计划器中的移动数，包括忙区（如果有）
    FORCE_INLINE static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail); }

    // Number of nonbusy moves currently in the planner//当前在计划器中的非繁忙移动数
    FORCE_INLINE static uint8_t nonbusy_movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_nonbusy); }

    // Remove all blocks from the buffer//从缓冲区中删除所有块
    FORCE_INLINE static void clear_block_buffer() { block_buffer_nonbusy = block_buffer_planned = block_buffer_head = block_buffer_tail = 0; }

    // Check if movement queue is full//检查移动队列是否已满
    FORCE_INLINE static bool is_full() { return block_buffer_tail == next_block_index(block_buffer_head); }

    // Get count of movement slots free//获得可用的移动槽数
    FORCE_INLINE static uint8_t moves_free() { return BLOCK_BUFFER_SIZE - 1 - movesplanned(); }

    /**
     * Planner::get_next_free_block
     *
     * - Get the next head indices (passed by reference)
     * - Wait for the number of spaces to open up in the planner
     * - Return the first head block
     */
    FORCE_INLINE static block_t* get_next_free_block(uint8_t &next_buffer_head, const uint8_t count=1) {

      // Wait until there are enough slots free//等待，直到有足够的空闲插槽
      while (moves_free() < count) { idle(); }

      // Return the first available block//返回第一个可用的块
      next_buffer_head = next_block_index(block_buffer_head);
      return &block_buffer[block_buffer_head];
    }

    /**
     * Planner::_buffer_steps
     *
     * Add a new linear movement to the buffer (in terms of steps).
     *
     *  target      - target position in steps units
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     *
     * Returns true if movement was buffered, false otherwise
     */
    static bool _buffer_steps(const xyze_long_t &target
      OPTARG(HAS_POSITION_FLOAT, const xyze_pos_t &target_float)
      OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
      , feedRate_t fr_mm_s, const uint8_t extruder, const_float_t millimeters=0.0
    );

    /**
     * Planner::_populate_block
     *
     * Fills a new linear movement in the block (in terms of steps).
     *
     *  target      - target position in steps units
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     *
     * Returns true is movement is acceptable, false otherwise
     */
    static bool _populate_block(block_t * const block, bool split_move, const xyze_long_t &target
      OPTARG(HAS_POSITION_FLOAT, const xyze_pos_t &target_float)
      OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
      , feedRate_t fr_mm_s, const uint8_t extruder, const_float_t millimeters=0.0
    );

    /**
     * Planner::buffer_sync_block
     * Add a block to the buffer that just updates the position or in
     * case of LASER_SYNCHRONOUS_M106_M107 the fan pwm
     */
    static void buffer_sync_block(
      TERN_(LASER_SYNCHRONOUS_M106_M107, uint8_t sync_flag=BLOCK_FLAG_SYNC_POSITION)
    );

  #if IS_KINEMATIC
    private:

      // Allow do_homing_move to access internal functions, such as buffer_segment.//允许do_homing_move访问内部功能，如缓冲段。
      friend void do_homing_move(const AxisEnum, const float, const feedRate_t, const bool);
  #endif

    /**
     * Planner::buffer_segment
     *
     * Add a new linear movement to the buffer in axis units.
     *
     * Leveling and kinematics should be applied ahead of calling this.
     *
     *  a,b,c,e     - target positions in mm and/or degrees
     *  fr_mm_s     - (target) speed of the move
     *  extruder    - target extruder
     *  millimeters - the length of the movement, if known
     */
    static bool buffer_segment(const abce_pos_t &abce
      OPTARG(HAS_DIST_MM_ARG, const xyze_float_t &cart_dist_mm)
      , const_feedRate_t fr_mm_s, const uint8_t extruder=active_extruder, const_float_t millimeters=0.0
    );

  public:

    /**
     * Add a new linear movement to the buffer.
     * The target is cartesian. It's translated to
     * delta/scara if needed.
     *
     *  cart         - target position in mm or degrees
     *  fr_mm_s      - (target) speed of the move (mm/s)
     *  extruder     - target extruder
     *  millimeters  - the length of the movement, if known
     *  inv_duration - the reciprocal if the duration of the movement, if known (kinematic only if feeedrate scaling is enabled)
     */
    static bool buffer_line(const xyze_pos_t &cart, const_feedRate_t fr_mm_s, const uint8_t extruder=active_extruder, const float millimeters=0.0
      OPTARG(SCARA_FEEDRATE_SCALING, const_float_t inv_duration=0.0)
    );

    #if ENABLED(DIRECT_STEPPING)
      static void buffer_page(const page_idx_t page_idx, const uint8_t extruder, const uint16_t num_steps);
    #endif

    /**
     * Set the planner.position and individual stepper positions.
     * Used by G92, G28, G29, and other procedures.
     *
     * The supplied position is in the cartesian coordinate space and is
     * translated in to machine space as needed. Modifiers such as leveling
     * and skew are also applied.
     *
     * Multiplies by axis_steps_per_mm[] and does necessary conversion
     * for COREXY / COREXZ / COREYZ to set the corresponding stepper positions.
     *
     * Clears previous speed values.
     */
    static void set_position_mm(const xyze_pos_t &xyze);

    #if HAS_EXTRUDERS
      static void set_e_position_mm(const_float_t e);
    #endif

    /**
     * Set the planner.position and individual stepper positions.
     *
     * The supplied position is in machine space, and no additional
     * conversions are applied.
     */
    static void set_machine_position_mm(const abce_pos_t &abce);

    /**
     * Get an axis position according to stepper position(s)
     * For CORE machines apply translation from ABC to XYZ.
     */
    static float get_axis_position_mm(const AxisEnum axis);

    static inline abce_pos_t get_axis_positions_mm() {
      const abce_pos_t out = LOGICAL_AXIS_ARRAY(
        get_axis_position_mm(E_AXIS),
        get_axis_position_mm(A_AXIS), get_axis_position_mm(B_AXIS), get_axis_position_mm(C_AXIS),
        get_axis_position_mm(I_AXIS), get_axis_position_mm(J_AXIS), get_axis_position_mm(K_AXIS)
      );
      return out;
    }

    // SCARA AB axes are in degrees, not mm//圣甲虫AB轴的单位是度，而不是毫米
    #if IS_SCARA
      FORCE_INLINE static float get_axis_position_degrees(const AxisEnum axis) { return get_axis_position_mm(axis); }
    #endif

    // Called to force a quick stop of the machine (for example, when//调用以强制快速停止机器（例如，当
    // a Full Shutdown is required, or when endstops are hit)//（需要完全关闭，或当碰到止动块时）
    static void quick_stop();

    #if ENABLED(REALTIME_REPORTING_COMMANDS)
      // Force a quick pause of the machine (e.g., when a pause is required in the middle of move)./强迫机器的快速停顿（例如，在移动中间需要暂停）。
      // NOTE: Hard-stops will lose steps so encoders are highly recommended if using these!//注意：硬停止将丢失步数，因此强烈建议使用编码器！
      static void quick_pause();
      static void quick_resume();
    #endif

    // Called when an endstop is triggered. Causes the machine to stop inmediately//触发结束停止时调用。使机器立即停止
    static void endstop_triggered(const AxisEnum axis);

    // Triggered position of an axis in mm (not core-savvy)//轴的触发位置，单位为mm（非核心能力）
    static float triggered_position_mm(const AxisEnum axis);

    // Block until all buffered steps are executed / cleaned//阻塞，直到执行/清除所有缓冲步骤
    static void synchronize();

    // Wait for moves to finish and disable all steppers//等待移动完成并禁用所有步进器
    static void finish_and_disable();

    // Periodic handler to manage the cleaning buffer counter//用于管理清理缓冲区计数器的定期处理程序
    // Called from the Temperature ISR at ~1kHz//从~1kHz的温度ISR调用
    static void isr() { if (cleaning_buffer_counter) --cleaning_buffer_counter; }

    /**
     * Does the buffer have any blocks queued?
     */
    FORCE_INLINE static bool has_blocks_queued() { return (block_buffer_head != block_buffer_tail); }

    /**
     * Get the current block for processing
     * and mark the block as busy.
     * Return nullptr if the buffer is empty
     * or if there is a first-block delay.
     *
     * WARNING: Called from Stepper ISR context!
     */
    static block_t* get_current_block();

    /**
     * "Release" the current block so its slot can be reused.
     * Called when the current block is no longer needed.
     */
    FORCE_INLINE static void release_current_block() {
      if (has_blocks_queued())
        block_buffer_tail = next_block_index(block_buffer_tail);
    }

    #if HAS_WIRED_LCD
      static uint16_t block_buffer_runtime();
      static void clear_block_buffer_runtime();
    #endif

    #if ENABLED(AUTOTEMP)
      static celsius_t autotemp_min, autotemp_max;
      static float autotemp_factor;
      static bool autotemp_enabled;
      static void autotemp_update();
      static void autotemp_M104_M109();
      static void autotemp_task();
    #endif

    #if HAS_LINEAR_E_JERK
      FORCE_INLINE static void recalculate_max_e_jerk() {
        const float prop = junction_deviation_mm * SQRT(0.5) / (1.0f - SQRT(0.5));
        LOOP_L_N(i, EXTRUDERS)
          max_e_jerk[E_INDEX_N(i)] = SQRT(prop * settings.max_acceleration_mm_per_s2[E_INDEX_N(i)]);
      }
    #endif

  private:

    #if ENABLED(AUTOTEMP)
      #if ENABLED(AUTOTEMP_PROPORTIONAL)
        static void _autotemp_update_from_hotend();
      #else
        static inline void _autotemp_update_from_hotend() {}
      #endif
    #endif

    /**
     * Get the index of the next / previous block in the ring buffer
     */
    static constexpr uint8_t next_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index + 1); }
    static constexpr uint8_t prev_block_index(const uint8_t block_index) { return BLOCK_MOD(block_index - 1); }

    /**
     * Calculate the distance (not time) it takes to accelerate
     * from initial_rate to target_rate using the given acceleration:
     */
    static float estimate_acceleration_distance(const_float_t initial_rate, const_float_t target_rate, const_float_t accel) {
      if (accel == 0) return 0; // accel was 0, set acceleration distance to 0//加速度为0，将加速度距离设置为0
      return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
    }

    /**
     * Return the point at which you must start braking (at the rate of -'accel') if
     * you start at 'initial_rate', accelerate (until reaching the point), and want to end at
     * 'final_rate' after traveling 'distance'.
     *
     * This is used to compute the intersection point between acceleration and deceleration
     * in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
     */
    static float intersection_distance(const_float_t initial_rate, const_float_t final_rate, const_float_t accel, const_float_t distance) {
      if (accel == 0) return 0; // accel was 0, set intersection distance to 0//加速度为0，将交点距离设置为0
      return (accel * 2 * distance - sq(initial_rate) + sq(final_rate)) / (accel * 4);
    }

    /**
     * Calculate the maximum allowable speed squared at this point, in order
     * to reach 'target_velocity_sqr' using 'acceleration' within a given
     * 'distance'.
     */
    static float max_allowable_speed_sqr(const_float_t accel, const_float_t target_velocity_sqr, const_float_t distance) {
      return target_velocity_sqr - 2 * accel * distance;
    }

    #if ENABLED(S_CURVE_ACCELERATION)
      /**
       * Calculate the speed reached given initial speed, acceleration and distance
       */
      static float final_speed(const_float_t initial_velocity, const_float_t accel, const_float_t distance) {
        return SQRT(sq(initial_velocity) + 2 * accel * distance);
      }
    #endif

    static void calculate_trapezoid_for_block(block_t * const block, const_float_t entry_factor, const_float_t exit_factor);

    static void reverse_pass_kernel(block_t * const current, const block_t * const next);
    static void forward_pass_kernel(const block_t * const previous, block_t * const current, uint8_t block_index);

    static void reverse_pass();
    static void forward_pass();

    static void recalculate_trapezoids();

    static void recalculate();

    #if HAS_JUNCTION_DEVIATION

      FORCE_INLINE static void normalize_junction_vector(xyze_float_t &vector) {
        float magnitude_sq = 0;
        LOOP_LOGICAL_AXES(idx) if (vector[idx]) magnitude_sq += sq(vector[idx]);
        vector *= RSQRT(magnitude_sq);
      }

      FORCE_INLINE static float limit_value_by_axis_maximum(const_float_t max_value, xyze_float_t &unit_vec) {
        float limit_value = max_value;
        LOOP_LOGICAL_AXES(idx) {
          if (unit_vec[idx]) {
            if (limit_value * ABS(unit_vec[idx]) > settings.max_acceleration_mm_per_s2[idx])
              limit_value = ABS(settings.max_acceleration_mm_per_s2[idx] / unit_vec[idx]);
          }
        }
        return limit_value;
      }

    #endif // !CLASSIC_JERK// !经典挺举
};

#define PLANNER_XY_FEEDRATE() _MIN(planner.settings.max_feedrate_mm_s[X_AXIS], planner.settings.max_feedrate_mm_s[Y_AXIS])

extern Planner planner;
