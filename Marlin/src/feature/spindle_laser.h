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
 * feature/spindle_laser.h
 * Support for Laser Power or Spindle Power & Direction
 */

#include "../inc/MarlinConfig.h"

#include "spindle_laser_types.h"

#if USE_BEEPER
  #include "../libs/buzzer.h"
#endif

#if ENABLED(LASER_POWER_INLINE)
  #include "../module/planner.h"
#endif

#define PCT_TO_PWM(X) ((X) * 255 / 100)
#define PCT_TO_SERVO(X) ((X) * 180 / 100)

#ifndef SPEED_POWER_INTERCEPT
  #define SPEED_POWER_INTERCEPT 0
#endif

// #define _MAP(N,S1,S2,D1,D2) ((N)*_MAX((D2)-(D1),0)/_MAX((S2)-(S1),1)+(D1))//定义映射（N，S1，S2，D1，D2）（（N）*_MAX（（D2）-（D1），0）/_MAX（（S2）-（S1），1）+（D1））

class SpindleLaser {
public:
  static constexpr float
    min_pct = TERN(CUTTER_POWER_RELATIVE, 0, TERN(SPINDLE_FEATURE, round(100.0f * (SPEED_POWER_MIN) / (SPEED_POWER_MAX)), SPEED_POWER_MIN)),
    max_pct = TERN(SPINDLE_FEATURE, 100, SPEED_POWER_MAX);

  static const inline uint8_t pct_to_ocr(const_float_t pct) { return uint8_t(PCT_TO_PWM(pct)); }

  // cpower = configured values (e.g., SPEED_POWER_MAX)//cpower=配置值（例如速度\功率\最大值）

  // Convert configured power range to a percentage//将配置的功率范围转换为百分比
  static const inline uint8_t cpwr_to_pct(const cutter_cpower_t cpwr) {
    constexpr cutter_cpower_t power_floor = TERN(CUTTER_POWER_RELATIVE, SPEED_POWER_MIN, 0),
                              power_range = SPEED_POWER_MAX - power_floor;
    return cpwr ? round(100.0f * (cpwr - power_floor) / power_range) : 0;
  }

  // Convert a cpower (e.g., SPEED_POWER_STARTUP) to unit power (upwr, upower),//将cpower（例如速度功率启动）转换为单位功率（upwr，upower），
  // which can be PWM, Percent, Servo angle, or RPM (rel/abs).//可以是PWM、百分比、伺服角度或RPM（rel/abs）。
  static const inline cutter_power_t cpwr_to_upwr(const cutter_cpower_t cpwr) { // STARTUP power to Unit power//启动功率到单位功率
    const cutter_power_t upwr = (
      #if ENABLED(SPINDLE_FEATURE)
        // Spindle configured values are in RPM//主轴配置值以RPM为单位
        #if CUTTER_UNIT_IS(RPM)
          cpwr                            // to RPM//到RPM
        #elif CUTTER_UNIT_IS(PERCENT)     // to PCT//到PCT
          cpwr_to_pct(cpwr)
        #elif CUTTER_UNIT_IS(SERVO)       // to SERVO angle//伺服角
          PCT_TO_SERVO(cpwr_to_pct(cpwr))
        #else                             // to PWM//脉宽调制
          PCT_TO_PWM(cpwr_to_pct(cpwr))
        #endif
      #else
        // Laser configured values are in PCT//激光器配置值以PCT为单位
        #if CUTTER_UNIT_IS(PWM255)
          PCT_TO_PWM(cpwr)
        #else
          cpwr                            // to RPM/PCT//至RPM/PCT
        #endif
      #endif
    );
    return upwr;
  }

  static const cutter_power_t mpower_min() { return cpwr_to_upwr(SPEED_POWER_MIN); }
  static const cutter_power_t mpower_max() { return cpwr_to_upwr(SPEED_POWER_MAX); }

  #if ENABLED(LASER_FEATURE)
    static cutter_test_pulse_t testPulse; // Test fire Pulse ms value//测试火脉冲ms值
  #endif

  static bool isReady;                    // Ready to apply power setting from the UI to OCR//准备好从UI向OCR应用电源设置了吗
  static uint8_t power;

  #if ENABLED(MARLIN_DEV_MODE)
    static cutter_frequency_t frequency;  // Set PWM frequency; range: 2K-50K//设置PWM频率；范围：2K-50K
  #endif

  static cutter_power_t menuPower,        // Power as set via LCD menu in PWM, Percentage or RPM//通过PWM、百分比或RPM中的LCD菜单设置的功率
                        unitPower;        // Power as displayed status in PWM, Percentage or RPM//以PWM、百分比或RPM显示的功率状态

  static void init();

  #if ENABLED(MARLIN_DEV_MODE)
    static inline void refresh_frequency() { set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), frequency); }
  #endif

  // Modifying this function should update everywhere//修改此函数应在任何地方更新
  static inline bool enabled(const cutter_power_t opwr) { return opwr > 0; }
  static inline bool enabled() { return enabled(power); }

  static void apply_power(const uint8_t inpow);

  FORCE_INLINE static void refresh() { apply_power(power); }
  FORCE_INLINE static void set_power(const uint8_t upwr) { power = upwr; refresh(); }

  #if ENABLED(SPINDLE_LASER_PWM)

    private:

    static void _set_ocr(const uint8_t ocr);

    public:

    static void set_ocr(const uint8_t ocr);
    static inline void set_ocr_power(const uint8_t ocr) { power = ocr; set_ocr(ocr); }
    static void ocr_off();
    // Used to update output for power->OCR translation//用于更新power->OCR翻译的输出
    static inline uint8_t upower_to_ocr(const cutter_power_t upwr) {
      return (
        #if CUTTER_UNIT_IS(PWM255)
          uint8_t(upwr)
        #elif CUTTER_UNIT_IS(PERCENT)
          pct_to_ocr(upwr)
        #else
          uint8_t(pct_to_ocr(cpwr_to_pct(upwr)))
        #endif
      );
    }

    // Correct power to configured range//正确的功率到配置的范围
    static inline cutter_power_t power_to_range(const cutter_power_t pwr) {
      return power_to_range(pwr, (
        #if CUTTER_UNIT_IS(PWM255)
          0
        #elif CUTTER_UNIT_IS(PERCENT)
          1
        #elif CUTTER_UNIT_IS(RPM)
          2
        #else
          #error "CUTTER_UNIT_IS(unknown)"
        #endif
      ));
    }
    static inline cutter_power_t power_to_range(const cutter_power_t pwr, const uint8_t pwrUnit) {
      if (pwr <= 0) return 0;
      cutter_power_t upwr;
      switch (pwrUnit) {
        case 0:                                                 // PWM//脉宽调制
          upwr = cutter_power_t(
              (pwr < pct_to_ocr(min_pct)) ? pct_to_ocr(min_pct) // Use minimum if set below//如果设置如下，则使用最小值
            : (pwr > pct_to_ocr(max_pct)) ? pct_to_ocr(max_pct) // Use maximum if set above//如果以上设置，则使用最大值
            :  pwr
          );
          break;
        case 1:                                                 // PERCENT//百分比
          upwr = cutter_power_t(
              (pwr < min_pct) ? min_pct                         // Use minimum if set below//如果设置如下，则使用最小值
            : (pwr > max_pct) ? max_pct                         // Use maximum if set above//如果以上设置，则使用最大值
            :  pwr                                              // PCT//PCT
          );
          break;
        case 2:                                                 // RPM//转速
          upwr = cutter_power_t(
              (pwr < SPEED_POWER_MIN) ? SPEED_POWER_MIN         // Use minimum if set below//如果设置如下，则使用最小值
            : (pwr > SPEED_POWER_MAX) ? SPEED_POWER_MAX         // Use maximum if set above//如果以上设置，则使用最大值
            : pwr                                               // Calculate OCR value//计算OCR值
          );
          break;
        default: break;
      }
      return upwr;
    }

  #endif // SPINDLE_LASER_PWM//主轴激光脉宽调制

  static inline void set_enabled(const bool enable) {
    set_power(enable ? TERN(SPINDLE_LASER_PWM, (power ?: (unitPower ? upower_to_ocr(cpwr_to_upwr(SPEED_POWER_STARTUP)) : 0)), 255) : 0);
  }

  // Wait for spindle to spin up or spin down//等待主轴向上或向下旋转
  static inline void power_delay(const bool on) {
    #if DISABLED(LASER_POWER_INLINE)
      safe_delay(on ? SPINDLE_LASER_POWERUP_DELAY : SPINDLE_LASER_POWERDOWN_DELAY);
    #endif
  }

  #if ENABLED(SPINDLE_CHANGE_DIR)
    static void set_reverse(const bool reverse);
    static bool is_reverse() { return READ(SPINDLE_DIR_PIN) == SPINDLE_INVERT_DIR; }
  #else
    static inline void set_reverse(const bool) {}
    static bool is_reverse() { return false; }
  #endif

  #if ENABLED(AIR_EVACUATION)
    static void air_evac_enable();         // Turn On Cutter Vacuum or Laser Blower motor//打开切割机真空或激光鼓风机电机
    static void air_evac_disable();        // Turn Off Cutter Vacuum or Laser Blower motor//关闭切割机真空或激光鼓风机电机
    static void air_evac_toggle();         // Toggle Cutter Vacuum or Laser Blower motor//切换切割机真空或激光鼓风机电机
    static inline bool air_evac_state() {  // Get current state//获取当前状态
      return (READ(AIR_EVACUATION_PIN) == AIR_EVACUATION_ACTIVE);
    }
  #endif

  #if ENABLED(AIR_ASSIST)
    static void air_assist_enable();         // Turn on air assist//打开空气辅助
    static void air_assist_disable();        // Turn off air assist//关闭空气辅助
    static void air_assist_toggle();         // Toggle air assist//切换空气辅助
    static inline bool air_assist_state() {  // Get current state//获取当前状态
      return (READ(AIR_ASSIST_PIN) == AIR_ASSIST_ACTIVE);
    }
  #endif

  static inline void disable() { isReady = false; set_enabled(false); }

  #if HAS_LCD_MENU
    static inline void enable_with_dir(const bool reverse) {
      isReady = true;
      const uint8_t ocr = TERN(SPINDLE_LASER_PWM, upower_to_ocr(menuPower), 255);
      if (menuPower)
        power = ocr;
      else
        menuPower = cpwr_to_upwr(SPEED_POWER_STARTUP);
      unitPower = menuPower;
      set_reverse(reverse);
      set_enabled(true);
    }
    FORCE_INLINE static void enable_forward() { enable_with_dir(false); }
    FORCE_INLINE static void enable_reverse() { enable_with_dir(true); }
    FORCE_INLINE static void enable_same_dir() { enable_with_dir(is_reverse()); }

    #if ENABLED(SPINDLE_LASER_PWM)
      static inline void update_from_mpower() {
        if (isReady) power = upower_to_ocr(menuPower);
        unitPower = menuPower;
      }
    #endif

    #if ENABLED(LASER_FEATURE)
      /**
       * Test fire the laser using the testPulse ms duration
       * Also fires with any PWM power that was previous set
       * If not set defaults to 80% power
       */
      static inline void test_fire_pulse() {
        TERN_(USE_BEEPER, buzzer.tone(30, 3000));
        enable_forward();                  // Turn Laser on (Spindle speak but same funct)//打开激光器（主轴说话，但功能相同）
        delay(testPulse);                  // Delay for time set by user in pulse ms menu screen.//用户在脉冲ms菜单屏幕中设置的时间延迟。
        disable();                         // Turn laser off//关闭激光器
      }
    #endif

  #endif // HAS_LCD_MENU//有LCD菜单吗

  #if ENABLED(LASER_POWER_INLINE)
    /**
     * Inline power adds extra fields to the planner block
     * to handle laser power and scale to movement speed.
     */

    // Force disengage planner power control//力计功率控制
    static inline void inline_disable() {
      isReady = false;
      unitPower = 0;
      planner.laser_inline.status.isPlanned = false;
      planner.laser_inline.status.isEnabled = false;
      planner.laser_inline.power = 0;
    }

    // Inline modes of all other functions; all enable planner inline power control//所有其他函数的内联模式；全部启用planner内联电源控制
    static inline void set_inline_enabled(const bool enable) {
      if (enable)
        inline_power(255);
      else {
        isReady = false;
        unitPower = menuPower = 0;
        planner.laser_inline.status.isPlanned = false;
        TERN(SPINDLE_LASER_PWM, inline_ocr_power, inline_power)(0);
      }
    }

    // Set the power for subsequent movement blocks//设置后续移动块的功率
    static void inline_power(const cutter_power_t upwr) {
      unitPower = menuPower = upwr;
      #if ENABLED(SPINDLE_LASER_PWM)
        #if ENABLED(SPEED_POWER_RELATIVE) && !CUTTER_UNIT_IS(RPM) // relative mode does not turn laser off at 0, except for RPM//相对模式不会在0时关闭激光器，RPM除外
          planner.laser_inline.status.isEnabled = true;
          planner.laser_inline.power = upower_to_ocr(upwr);
          isReady = true;
        #else
          inline_ocr_power(upower_to_ocr(upwr));
        #endif
      #else
        planner.laser_inline.status.isEnabled = enabled(upwr);
        planner.laser_inline.power = upwr;
        isReady = enabled(upwr);
      #endif
    }

    static inline void inline_direction(const bool) { /* never */ }

    #if ENABLED(SPINDLE_LASER_PWM)
      static inline void inline_ocr_power(const uint8_t ocrpwr) {
        isReady = ocrpwr > 0;
        planner.laser_inline.status.isEnabled = ocrpwr > 0;
        planner.laser_inline.power = ocrpwr;
      }
    #endif
  #endif  // LASER_POWER_INLINE//激光功率在线

  static inline void kill() {
    TERN_(LASER_POWER_INLINE, inline_disable());
    disable();
  }
};

extern SpindleLaser cutter;
