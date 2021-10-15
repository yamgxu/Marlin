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
 * feature/spindle_laser.cpp
 */

#include "../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "spindle_laser.h"

#if ENABLED(SPINDLE_SERVO)
  #include "../module/servo.h"
#endif

#if ENABLED(I2C_AMMETER)
  #include "../feature/ammeter.h"
#endif

SpindleLaser cutter;
uint8_t SpindleLaser::power;
#if ENABLED(LASER_FEATURE)
  cutter_test_pulse_t SpindleLaser::testPulse = 50;                   // Test fire Pulse time ms value.//测试点火脉冲时间ms值。
#endif
bool SpindleLaser::isReady;                                           // Ready to apply power setting from the UI to OCR//准备好从UI向OCR应用电源设置了吗
cutter_power_t SpindleLaser::menuPower,                               // Power set via LCD menu in PWM, PERCENT, or RPM//通过PWM、百分比或RPM中的LCD菜单设置功率
               SpindleLaser::unitPower;                               // LCD status power in PWM, PERCENT, or RPM//以PWM、百分比或RPM表示的LCD状态功率

#if ENABLED(MARLIN_DEV_MODE)
  cutter_frequency_t SpindleLaser::frequency;                         // PWM frequency setting; range: 2K - 50K//PWM频率设置；范围：2K-50K
#endif
#define SPINDLE_LASER_PWM_OFF TERN(SPINDLE_LASER_PWM_INVERT, 255, 0)

////
// Init the cutter to a safe OFF state//使切割机处于安全关闭状态
////
void SpindleLaser::init() {
  #if ENABLED(SPINDLE_SERVO)
    MOVE_SERVO(SPINDLE_SERVO_NR, SPINDLE_SERVO_MIN);
  #else
    OUT_WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE);    // Init spindle to off//初始主轴关闭
  #endif
  #if ENABLED(SPINDLE_CHANGE_DIR)
    OUT_WRITE(SPINDLE_DIR_PIN, SPINDLE_INVERT_DIR ? 255 : 0);         // Init rotation to clockwise (M3)//初始顺时针旋转（M3）
  #endif
  #if ENABLED(SPINDLE_LASER_PWM)
    SET_PWM(SPINDLE_LASER_PWM_PIN);
    analogWrite(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_PWM_OFF); // Set to lowest speed//设置为最低速度
  #endif
  #if ENABLED(HAL_CAN_SET_PWM_FREQ) && defined(SPINDLE_LASER_FREQUENCY)
    set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_FREQUENCY);
    TERN_(MARLIN_DEV_MODE, frequency = SPINDLE_LASER_FREQUENCY);
  #endif
  #if ENABLED(AIR_EVACUATION)
    OUT_WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE);            // Init Vacuum/Blower OFF//初始真空/鼓风机关闭
  #endif
  #if ENABLED(AIR_ASSIST)
    OUT_WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_ACTIVE);                    // Init Air Assist OFF//初始空气辅助关闭
  #endif
  #if ENABLED(I2C_AMMETER)
    ammeter.init();                                                   // Init I2C Ammeter//初始I2C电流表
  #endif
}

#if ENABLED(SPINDLE_LASER_PWM)
  /**
   * Set the cutter PWM directly to the given ocr value
   */
  void SpindleLaser::_set_ocr(const uint8_t ocr) {
    #if NEEDS_HARDWARE_PWM && SPINDLE_LASER_FREQUENCY
      set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), TERN(MARLIN_DEV_MODE, frequency, SPINDLE_LASER_FREQUENCY));
      set_pwm_duty(pin_t(SPINDLE_LASER_PWM_PIN), ocr ^ SPINDLE_LASER_PWM_OFF);
    #else
      analogWrite(pin_t(SPINDLE_LASER_PWM_PIN), ocr ^ SPINDLE_LASER_PWM_OFF);
    #endif
  }

  void SpindleLaser::set_ocr(const uint8_t ocr) {
    WRITE(SPINDLE_LASER_ENA_PIN,  SPINDLE_LASER_ACTIVE_STATE); // Cutter ON//开刀
    _set_ocr(ocr);
  }

  void SpindleLaser::ocr_off() {
    WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE); // Cutter OFF//切断
    _set_ocr(0);
  }
#endif

////
// Set cutter ON/OFF state (and PWM) to the given cutter power value//将刀具开/关状态（和PWM）设置为给定的刀具功率值
////
void SpindleLaser::apply_power(const uint8_t opwr) {
  static uint8_t last_power_applied = 0;
  if (opwr == last_power_applied) return;
  last_power_applied = opwr;
  power = opwr;
  #if ENABLED(SPINDLE_LASER_PWM)
    if (cutter.unitPower == 0 && CUTTER_UNIT_IS(RPM)) {
      ocr_off();
      isReady = false;
    }
    else if (ENABLED(CUTTER_POWER_RELATIVE) || enabled()) {
      set_ocr(power);
      isReady = true;
    }
    else {
      ocr_off();
      isReady = false;
    }
  #elif ENABLED(SPINDLE_SERVO)
    MOVE_SERVO(SPINDLE_SERVO_NR, power);
  #else
    WRITE(SPINDLE_LASER_ENA_PIN, enabled() ? SPINDLE_LASER_ACTIVE_STATE : !SPINDLE_LASER_ACTIVE_STATE);
    isReady = true;
  #endif
}

#if ENABLED(SPINDLE_CHANGE_DIR)
  ////
  // Set the spindle direction and apply immediately//设置主轴方向并立即应用
  // Stop on direction change if SPINDLE_STOP_ON_DIR_CHANGE is enabled//如果启用主轴停止方向改变，则停止方向改变
  ////
  void SpindleLaser::set_reverse(const bool reverse) {
    const bool dir_state = (reverse == SPINDLE_INVERT_DIR); // Forward (M3) HIGH when not inverted//未反转时向前（M3）高
    if (TERN0(SPINDLE_STOP_ON_DIR_CHANGE, enabled()) && READ(SPINDLE_DIR_PIN) != dir_state) disable();
    WRITE(SPINDLE_DIR_PIN, dir_state);
  }
#endif

#if ENABLED(AIR_EVACUATION)

  // Enable / disable Cutter Vacuum or Laser Blower motor//启用/禁用切割机真空或激光鼓风机电机
  void SpindleLaser::air_evac_enable()  { WRITE(AIR_EVACUATION_PIN,  AIR_EVACUATION_ACTIVE); } // Turn ON//打开

  void SpindleLaser::air_evac_disable() { WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE); } // Turn OFF//关掉

  void SpindleLaser::air_evac_toggle()  { TOGGLE(AIR_EVACUATION_PIN); } // Toggle state//切换状态

#endif // AIR_EVACUATION//空中疏散

#if ENABLED(AIR_ASSIST)

  // Enable / disable air assist//启用/禁用空气辅助
  void SpindleLaser::air_assist_enable()  { WRITE(AIR_ASSIST_PIN,  AIR_ASSIST_PIN); } // Turn ON//打开

  void SpindleLaser::air_assist_disable() { WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_PIN); } // Turn OFF//关掉

  void SpindleLaser::air_assist_toggle()  { TOGGLE(AIR_ASSIST_PIN); } // Toggle state//切换状态

#endif // AIR_ASSIST//空气辅助

#endif // HAS_CUTTER//切割器
