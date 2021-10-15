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
 * module/probe.cpp
 */

#include "../inc/MarlinConfig.h"

#if HAS_BED_PROBE

#include "probe.h"

#include "../libs/buzzer.h"
#include "motion.h"
#include "temperature.h"
#include "endstops.h"

#include "../gcode/gcode.h"
#include "../lcd/marlinui.h"

#include "../MarlinCore.h" // for stop(), disable_e_steppers(), wait_for_user_response()//对于stop（），禁用步进器（），等待用户响应（）

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(DELTA)
  #include "delta.h"
#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "planner.h"
#endif

#if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
  #include "../feature/backlash.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h" // for PROMPT_USER_CONTINUE//对于提示用户，请继续
#endif

#if HAS_Z_SERVO_PROBE
  #include "servo.h"
#endif

#if ENABLED(SENSORLESS_PROBING)
  #include "stepper.h"
  #include "../feature/tmc_util.h"
#endif

#if HAS_QUIET_PROBING
  #include "stepper/indirection.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

Probe probe;

xyz_pos_t Probe::offset; // Initialized by settings.load()//由settings.load（）初始化

#if HAS_PROBE_XY_OFFSET
  const xy_pos_t &Probe::offset_xy = Probe::offset;
#endif

#if ENABLED(Z_PROBE_SLED)

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * stow[in]     If false, move to MAX_X and engage the solenoid
   *              If true, move to MAX_X and release the solenoid
   */
  static void dock_sled(const bool stow) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("dock_sled(", stow, ")");

    // Dock sled a bit closer to ensure proper capturing//将雪橇靠得更近一点，以确保正确捕获
    do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));

    #if HAS_SOLENOID_1 && DISABLED(EXT_SOLENOID)
      WRITE(SOL1_PIN, !stow); // switch solenoid//开关电磁阀
    #endif
  }

#elif ENABLED(TOUCH_MI_PROBE)

  // Move to the magnet to unlock the probe//移动到磁铁以解锁探头
  inline void run_deploy_moves_script() {
    #ifndef TOUCH_MI_DEPLOY_XPOS
      #define TOUCH_MI_DEPLOY_XPOS X_MIN_POS
    #elif TOUCH_MI_DEPLOY_XPOS > X_MAX_BED
      TemporaryGlobalEndstopsState unlock_x(false);
    #endif
    #if TOUCH_MI_DEPLOY_YPOS > Y_MAX_BED
      TemporaryGlobalEndstopsState unlock_y(false);
    #endif

    #if ENABLED(TOUCH_MI_MANUAL_DEPLOY)

      const screenFunc_t prev_screen = ui.currentScreen;
      LCD_MESSAGEPGM(MSG_MANUAL_DEPLOY_TOUCHMI);
      ui.return_to_status();

      TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Deploy TouchMI"), CONTINUE_STR));
      wait_for_user_response();
      ui.reset_status();
      ui.goto_screen(prev_screen);

    #elif defined(TOUCH_MI_DEPLOY_XPOS) && defined(TOUCH_MI_DEPLOY_YPOS)
      do_blocking_move_to_xy(TOUCH_MI_DEPLOY_XPOS, TOUCH_MI_DEPLOY_YPOS);
    #elif defined(TOUCH_MI_DEPLOY_XPOS)
      do_blocking_move_to_x(TOUCH_MI_DEPLOY_XPOS);
    #elif defined(TOUCH_MI_DEPLOY_YPOS)
      do_blocking_move_to_y(TOUCH_MI_DEPLOY_YPOS);
    #endif
  }

  // Move down to the bed to stow the probe//向下移动到床上以收起探头
  inline void run_stow_moves_script() {
    const xyz_pos_t oldpos = current_position;
    endstops.enable_z_probe(false);
    do_blocking_move_to_z(TOUCH_MI_RETRACT_Z, homing_feedrate(Z_AXIS));
    do_blocking_move_to(oldpos, homing_feedrate(Z_AXIS));
  }

#elif ENABLED(Z_PROBE_ALLEN_KEY)

  inline void run_deploy_moves_script() {
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_1
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t deploy_1 = Z_PROBE_ALLEN_KEY_DEPLOY_1;
      do_blocking_move_to(deploy_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_2
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t deploy_2 = Z_PROBE_ALLEN_KEY_DEPLOY_2;
      do_blocking_move_to(deploy_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_3
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t deploy_3 = Z_PROBE_ALLEN_KEY_DEPLOY_3;
      do_blocking_move_to(deploy_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_4
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t deploy_4 = Z_PROBE_ALLEN_KEY_DEPLOY_4;
      do_blocking_move_to(deploy_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_5
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t deploy_5 = Z_PROBE_ALLEN_KEY_DEPLOY_5;
      do_blocking_move_to(deploy_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE));
    #endif
  }

  inline void run_stow_moves_script() {
    #ifdef Z_PROBE_ALLEN_KEY_STOW_1
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t stow_1 = Z_PROBE_ALLEN_KEY_STOW_1;
      do_blocking_move_to(stow_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_2
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t stow_2 = Z_PROBE_ALLEN_KEY_STOW_2;
      do_blocking_move_to(stow_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_3
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t stow_3 = Z_PROBE_ALLEN_KEY_STOW_3;
      do_blocking_move_to(stow_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_4
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t stow_4 = Z_PROBE_ALLEN_KEY_STOW_4;
      do_blocking_move_to(stow_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_5
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE 0.0
      #endif
      constexpr xyz_pos_t stow_5 = Z_PROBE_ALLEN_KEY_STOW_5;
      do_blocking_move_to(stow_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE));
    #endif
  }

#endif // Z_PROBE_ALLEN_KEY//Z_探头_艾伦_键

#if HAS_QUIET_PROBING

  #ifndef DELAY_BEFORE_PROBING
    #define DELAY_BEFORE_PROBING 25
  #endif

  void Probe::set_probing_paused(const bool dopause) {
    TERN_(PROBING_HEATERS_OFF, thermalManager.pause_heaters(dopause));
    TERN_(PROBING_FANS_OFF, thermalManager.set_fans_paused(dopause));
    TERN_(PROBING_ESTEPPERS_OFF, if (dopause) disable_e_steppers());
    #if ENABLED(PROBING_STEPPERS_OFF)
      IF_DISABLED(DELTA, static uint8_t old_trusted);
      if (dopause) {
        #if DISABLED(DELTA)
          old_trusted = axis_trusted;
          DISABLE_AXIS_X();
          DISABLE_AXIS_Y();
        #endif
        IF_DISABLED(PROBING_ESTEPPERS_OFF, disable_e_steppers());
      }
      else {
        #if DISABLED(DELTA)
          if (TEST(old_trusted, X_AXIS)) ENABLE_AXIS_X();
          if (TEST(old_trusted, Y_AXIS)) ENABLE_AXIS_Y();
        #endif
        axis_trusted = old_trusted;
      }
    #endif
    if (dopause) safe_delay(_MAX(DELAY_BEFORE_PROBING, 25));
  }

#endif // HAS_QUIET_PROBING//你安静了吗

/**
 * Raise Z to a minimum height to make room for a probe to move
 */
void Probe::do_z_raise(const float z_raise) {
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("Probe::do_z_raise(", z_raise, ")");
  float z_dest = z_raise;
  if (offset.z < 0) z_dest -= offset.z;
  do_z_clearance(z_dest);
}

FORCE_INLINE void probe_specific_action(const bool deploy) {
  #if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)
    do {
      #if ENABLED(PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED)
        if (deploy == PROBE_TRIGGERED()) break;
      #endif

      BUZZ(100, 659);
      BUZZ(100, 698);

      PGM_P const ds_str = deploy ? GET_TEXT(MSG_MANUAL_DEPLOY) : GET_TEXT(MSG_MANUAL_STOW);
      ui.return_to_status();       // To display the new status message//显示新状态消息的步骤
      ui.set_status_P(ds_str, 99);
      SERIAL_ECHOLNPGM_P(ds_str);

      TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_USER_CONTINUE, PSTR("Stow Probe"), CONTINUE_STR));
      TERN_(EXTENSIBLE_UI, ExtUI::onUserConfirmRequired_P(PSTR("Stow Probe")));

      wait_for_user_response();
      ui.reset_status();

    } while (ENABLED(PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED));

  #endif // PAUSE_BEFORE_DEPLOY_STOW//在部署前暂停

  #if ENABLED(SOLENOID_PROBE)

    #if HAS_SOLENOID_1
      WRITE(SOL1_PIN, deploy);
    #endif

  #elif ENABLED(Z_PROBE_SLED)

    dock_sled(!deploy);

  #elif ENABLED(BLTOUCH)

    deploy ? bltouch.deploy() : bltouch.stow();

  #elif HAS_Z_SERVO_PROBE

    MOVE_SERVO(Z_PROBE_SERVO_NR, servo_angles[Z_PROBE_SERVO_NR][deploy ? 0 : 1]);

  #elif EITHER(TOUCH_MI_PROBE, Z_PROBE_ALLEN_KEY)

    deploy ? run_deploy_moves_script() : run_stow_moves_script();

  #elif ENABLED(RACK_AND_PINION_PROBE)

    do_blocking_move_to_x(deploy ? Z_PROBE_DEPLOY_X : Z_PROBE_RETRACT_X);

  #elif DISABLED(PAUSE_BEFORE_DEPLOY_STOW)

    UNUSED(deploy);

  #endif
}

#if EITHER(PREHEAT_BEFORE_PROBING, PREHEAT_BEFORE_LEVELING)

  #if ENABLED(PREHEAT_BEFORE_PROBING)
    #ifndef PROBING_NOZZLE_TEMP
      #define PROBING_NOZZLE_TEMP 0
    #endif
    #ifndef PROBING_BED_TEMP
      #define PROBING_BED_TEMP 0
    #endif
  #endif

  /**
   * Do preheating as required before leveling or probing.
   *  - If a preheat input is higher than the current target, raise the target temperature.
   *  - If a preheat input is higher than the current temperature, wait for stabilization.
   */
  void Probe::preheat_for_probing(const celsius_t hotend_temp, const celsius_t bed_temp) {
    #if HAS_HOTEND && (PROBING_NOZZLE_TEMP || LEVELING_NOZZLE_TEMP)
      #define WAIT_FOR_NOZZLE_HEAT
    #endif
    #if HAS_HEATED_BED && (PROBING_BED_TEMP || LEVELING_BED_TEMP)
      #define WAIT_FOR_BED_HEAT
    #endif

    DEBUG_ECHOPGM("Preheating ");

    #if ENABLED(WAIT_FOR_NOZZLE_HEAT)
      const celsius_t hotendPreheat = hotend_temp > thermalManager.degTargetHotend(0) ? hotend_temp : 0;
      if (hotendPreheat) {
        DEBUG_ECHOPAIR("hotend (", hotendPreheat, ")");
        thermalManager.setTargetHotend(hotendPreheat, 0);
      }
    #elif ENABLED(WAIT_FOR_BED_HEAT)
      constexpr celsius_t hotendPreheat = 0;
    #endif

    #if ENABLED(WAIT_FOR_BED_HEAT)
      const celsius_t bedPreheat = bed_temp > thermalManager.degTargetBed() ? bed_temp : 0;
      if (bedPreheat) {
        if (hotendPreheat) DEBUG_ECHOPGM(" and ");
        DEBUG_ECHOPAIR("bed (", bedPreheat, ")");
        thermalManager.setTargetBed(bedPreheat);
      }
    #endif

    DEBUG_EOL();

    TERN_(WAIT_FOR_NOZZLE_HEAT, if (hotend_temp > thermalManager.wholeDegHotend(0) + (TEMP_WINDOW)) thermalManager.wait_for_hotend(0));
    TERN_(WAIT_FOR_BED_HEAT,    if (bed_temp    > thermalManager.wholeDegBed() + (TEMP_BED_WINDOW)) thermalManager.wait_for_bed_heating());
  }

#endif

/**
 * Attempt to deploy or stow the probe
 *
 * Return TRUE if the probe could not be deployed/stowed
 */
bool Probe::set_deployed(const bool deploy) {

  if (DEBUGGING(LEVELING)) {
    DEBUG_POS("Probe::set_deployed", current_position);
    DEBUG_ECHOLNPAIR("deploy: ", deploy);
  }

  if (endstops.z_probe_enabled == deploy) return false;

  // Make room for probe to deploy (or stow)//为探头的展开（或存放）留出空间
  // Fix-mounted probe should only raise for deploy//固定式探头仅应在展开时升起
  // unless PAUSE_BEFORE_DEPLOY_STOW is enabled//除非在启用部署前暂停存储
  #if EITHER(FIX_MOUNTED_PROBE, NOZZLE_AS_PROBE) && DISABLED(PAUSE_BEFORE_DEPLOY_STOW)
    const bool z_raise_wanted = deploy;
  #else
    constexpr bool z_raise_wanted = true;
  #endif

  if (z_raise_wanted)
    do_z_raise(_MAX(Z_CLEARANCE_BETWEEN_PROBES, Z_CLEARANCE_DEPLOY_PROBE));

  #if EITHER(Z_PROBE_SLED, Z_PROBE_ALLEN_KEY)
    if (homing_needed_error(TERN_(Z_PROBE_SLED, _BV(X_AXIS)))) {
      SERIAL_ERROR_MSG(STR_STOP_UNHOMED);
      stop();
      return true;
    }
  #endif

  const xy_pos_t old_xy = current_position;

  #if ENABLED(PROBE_TRIGGERED_WHEN_STOWED_TEST)

    // Only deploy/stow if needed//仅在需要时展开/收起
    if (PROBE_TRIGGERED() == deploy) {
      if (!deploy) endstops.enable_z_probe(false); // Switch off triggered when stowed probes early//提前收起探头时触发关闭
                                                   // otherwise an Allen-Key probe can't be stowed.//否则，内六角扳手探头无法收起。
      probe_specific_action(deploy);
    }

    if (PROBE_TRIGGERED() == deploy) {             // Unchanged after deploy/stow action?//展开/收起操作后是否保持不变？
      if (IsRunning()) {
        SERIAL_ERROR_MSG("Z-Probe failed");
        LCD_ALERTMESSAGEPGM_P(PSTR("Err: ZPROBE"));
      }
      stop();
      return true;
    }

  #else

    probe_specific_action(deploy);

  #endif

  // If preheating is required before any probing...//如果在任何探测之前需要预热。。。
  TERN_(PREHEAT_BEFORE_PROBING, if (deploy) preheat_for_probing(PROBING_NOZZLE_TEMP, PROBING_BED_TEMP));

  do_blocking_move_to(old_xy);
  endstops.enable_z_probe(deploy);
  return false;
}

/**
 * @brief Used by run_z_probe to do a single Z probe move.
 *
 * @param  z        Z destination
 * @param  fr_mm_s  Feedrate in mm/s
 * @return true to indicate an error
 */

/**
 * @brief Move down until the probe triggers or the low limit is reached
 *
 * @details Used by run_z_probe to get each bed Z height measurement.
 *          Sets current_position.z to the height where the probe triggered
 *          (according to the Z stepper count). The float Z is propagated
 *          back to the planner.position to preempt any rounding error.
 *
 * @return TRUE if the probe failed to trigger.
 */
bool Probe::probe_down_to_z(const_float_t z, const_feedRate_t fr_mm_s) {
  DEBUG_SECTION(log_probe, "Probe::probe_down_to_z", DEBUGGING(LEVELING));

  #if BOTH(HAS_HEATED_BED, WAIT_FOR_BED_HEATER)
    thermalManager.wait_for_bed_heating();
  #endif

  #if BOTH(HAS_TEMP_HOTEND, WAIT_FOR_HOTEND)
    thermalManager.wait_for_hotend_heating(active_extruder);
  #endif

  if (TERN0(BLTOUCH_SLOW_MODE, bltouch.deploy())) return true; // Deploy in LOW SPEED MODE on every probe action//在每次探测动作中以低速模式部署

  // Disable stealthChop if used. Enable diag1 pin on driver.//禁用隐形斩波（如果使用）。启用驱动器上的diag1引脚。
  #if ENABLED(SENSORLESS_PROBING)
    sensorless_t stealth_states { false };
    #if ENABLED(DELTA)
      stealth_states.x = tmc_enable_stallguard(stepperX);
      stealth_states.y = tmc_enable_stallguard(stepperY);
    #endif
    stealth_states.z = tmc_enable_stallguard(stepperZ);
    endstops.enable(true);
  #endif

  TERN_(HAS_QUIET_PROBING, set_probing_paused(true));

  // Move down until the probe is triggered//向下移动，直到触发探针
  do_blocking_move_to_z(z, fr_mm_s);

  // Check to see if the probe was triggered//检查探针是否已触发
  const bool probe_triggered =
    #if BOTH(DELTA, SENSORLESS_PROBING)
      endstops.trigger_state() & (_BV(X_MAX) | _BV(Y_MAX) | _BV(Z_MAX))
    #else
      TEST(endstops.trigger_state(), Z_MIN_PROBE)
    #endif
  ;

  TERN_(HAS_QUIET_PROBING, set_probing_paused(false));

  // Re-enable stealthChop if used. Disable diag1 pin on driver.//重新启用隐身斩波（如果使用）。禁用驱动器上的diag1引脚。
  #if ENABLED(SENSORLESS_PROBING)
    endstops.not_homing();
    #if ENABLED(DELTA)
      tmc_disable_stallguard(stepperX, stealth_states.x);
      tmc_disable_stallguard(stepperY, stealth_states.y);
    #endif
    tmc_disable_stallguard(stepperZ, stealth_states.z);
  #endif

  if (probe_triggered && TERN0(BLTOUCH_SLOW_MODE, bltouch.stow())) // Stow in LOW SPEED MODE on every trigger//在每个触发器上以低速模式收起
    return true;

  // Clear endstop flags//清除结束停止标志
  endstops.hit_on_purpose();

  // Get Z where the steppers were interrupted//在步进器被中断的地方得到Z
  set_current_from_steppers_for_axis(Z_AXIS);

  // Tell the planner where we actually are//告诉计划者我们的实际位置
  sync_plan_position();

  return !probe_triggered;
}

#if ENABLED(PROBE_TARE)

  /**
   * @brief Init the tare pin
   *
   * @details Init tare pin to ON state for a strain gauge, otherwise OFF
   */
  void Probe::tare_init() {
    OUT_WRITE(PROBE_TARE_PIN, !PROBE_TARE_STATE);
  }

  /**
   * @brief Tare the Z probe
   *
   * @details Signal to the probe to tare itself
   *
   * @return TRUE if the tare cold not be completed
   */
  bool Probe::tare() {
    #if BOTH(PROBE_ACTIVATION_SWITCH, PROBE_TARE_ONLY_WHILE_INACTIVE)
      if (endstops.probe_switch_activated()) {
        SERIAL_ECHOLNPGM("Cannot tare an active probe");
        return true;
      }
    #endif

    SERIAL_ECHOLNPGM("Taring probe");
    WRITE(PROBE_TARE_PIN, PROBE_TARE_STATE);
    delay(PROBE_TARE_TIME);
    WRITE(PROBE_TARE_PIN, !PROBE_TARE_STATE);
    delay(PROBE_TARE_DELAY);

    endstops.hit_on_purpose();
    return false;
  }
#endif

/**
 * @brief Probe at the current XY (possibly more than once) to find the bed Z.
 *
 * @details Used by probe_at_point to get the bed Z height at the current XY.
 *          Leaves current_position.z at the height where the probe triggered.
 *
 * @return The Z position of the bed at the current XY or NAN on error.
 */
float Probe::run_z_probe(const bool sanity_check/*=true*/) {
  DEBUG_SECTION(log_probe, "Probe::run_z_probe", DEBUGGING(LEVELING));

  auto try_to_probe = [&](PGM_P const plbl, const_float_t z_probe_low_point, const feedRate_t fr_mm_s, const bool scheck, const float clearance) -> bool {
    // Tare the probe, if supported//如有支撑，给探头去皮
    if (TERN0(PROBE_TARE, tare())) return true;

    // Do a first probe at the fast speed//以最快的速度进行第一次探测
    const bool probe_fail = probe_down_to_z(z_probe_low_point, fr_mm_s),            // No probe trigger?//没有探针触发器？
               early_fail = (scheck && current_position.z > -offset.z + clearance); // Probe triggered too high?//探头触发过高？
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING) && (probe_fail || early_fail)) {
        DEBUG_ECHOPGM_P(plbl);
        DEBUG_ECHOPGM(" Probe fail! -");
        if (probe_fail) DEBUG_ECHOPGM(" No trigger.");
        if (early_fail) DEBUG_ECHOPGM(" Triggered early.");
        DEBUG_EOL();
      }
    #else
      UNUSED(plbl);
    #endif
    return probe_fail || early_fail;
  };

  // Stop the probe before it goes too low to prevent damage.//在探头过低之前停止探头，以防损坏。
  // If Z isn't known then probe to -10mm.//如果Z未知，则探测至-10mm。
  const float z_probe_low_point = axis_is_trusted(Z_AXIS) ? -offset.z + Z_PROBE_LOW_POINT : -10.0;

  // Double-probing does a fast probe followed by a slow probe//双重探测是先进行快速探测，然后进行慢速探测
  #if TOTAL_PROBING == 2

    // Attempt to tare the probe//试图给探头去皮
    if (TERN0(PROBE_TARE, tare())) return NAN;

    // Do a first probe at the fast speed//以最快的速度进行第一次探测
    if (try_to_probe(PSTR("FAST"), z_probe_low_point, z_probe_fast_mm_s,
                     sanity_check, Z_CLEARANCE_BETWEEN_PROBES) ) return NAN;

    const float first_probe_z = current_position.z;

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("1st Probe Z:", first_probe_z);

    // Raise to give the probe clearance//升起以给探针留出间隙
    do_blocking_move_to_z(current_position.z + Z_CLEARANCE_MULTI_PROBE, z_probe_fast_mm_s);

  #elif Z_PROBE_FEEDRATE_FAST != Z_PROBE_FEEDRATE_SLOW

    // If the nozzle is well over the travel height then//如果喷嘴远远超过行程高度，则
    // move down quickly before doing the slow probe//在进行慢速探测之前，快速向下移动
    const float z = Z_CLEARANCE_DEPLOY_PROBE + 5.0 + (offset.z < 0 ? -offset.z : 0);
    if (current_position.z > z) {
      // Probe down fast. If the probe never triggered, raise for probe clearance//快速向下探测。如果探针从未触发，则升高探针间隙
      if (!probe_down_to_z(z, z_probe_fast_mm_s))
        do_blocking_move_to_z(current_position.z + Z_CLEARANCE_BETWEEN_PROBES, z_probe_fast_mm_s);
    }
  #endif

  #if EXTRA_PROBING > 0
    float probes[TOTAL_PROBING];
  #endif

  #if TOTAL_PROBING > 2
    float probes_z_sum = 0;
    for (
      #if EXTRA_PROBING > 0
        uint8_t p = 0; p < TOTAL_PROBING; p++
      #else
        uint8_t p = TOTAL_PROBING; p--;
      #endif
    )
  #endif
    {
      // If the probe won't tare, return//如果探针无法去皮，则返回
      if (TERN0(PROBE_TARE, tare())) return true;

      // Probe downward slowly to find the bed//慢慢向下探，找到床
      if (try_to_probe(PSTR("SLOW"), z_probe_low_point, MMM_TO_MMS(Z_PROBE_FEEDRATE_SLOW),
                       sanity_check, Z_CLEARANCE_MULTI_PROBE) ) return NAN;

      TERN_(MEASURE_BACKLASH_WHEN_PROBING, backlash.measure_with_probe());

      const float z = current_position.z;

      #if EXTRA_PROBING > 0
        // Insert Z measurement into probes[]. Keep it sorted ascending.//将Z测量插入探针[]。按升序排序。
        LOOP_LE_N(i, p) {                            // Iterate the saved Zs to insert the new Z//迭代保存的Z以插入新的Z
          if (i == p || probes[i] > z) {                              // Last index or new Z is smaller than this Z//最后一个索引或新Z小于此Z
            for (int8_t m = p; --m >= i;) probes[m + 1] = probes[m];  // Shift items down after the insertion point//在插入点后向下移动项目
            probes[i] = z;                                            // Insert the new Z measurement//插入新的Z测量值
            break;                                                    // Only one to insert. Done!//只需插入一个。完成！
          }
        }
      #elif TOTAL_PROBING > 2
        probes_z_sum += z;
      #else
        UNUSED(z);
      #endif

      #if TOTAL_PROBING > 2
        // Small Z raise after all but the last probe//除了最后一个探针之外，小Z升高
        if (p
          #if EXTRA_PROBING > 0
            < TOTAL_PROBING - 1
          #endif
        ) do_blocking_move_to_z(z + Z_CLEARANCE_MULTI_PROBE, z_probe_fast_mm_s);
      #endif
    }

  #if TOTAL_PROBING > 2

    #if EXTRA_PROBING > 0
      // Take the center value (or average the two middle values) as the median//取中心值（或两个中间值的平均值）作为中间值
      static constexpr int PHALF = (TOTAL_PROBING - 1) / 2;
      const float middle = probes[PHALF],
                  median = ((TOTAL_PROBING) & 1) ? middle : (middle + probes[PHALF + 1]) * 0.5f;

      // Remove values farthest from the median//删除距离中间值最远的值
      uint8_t min_avg_idx = 0, max_avg_idx = TOTAL_PROBING - 1;
      for (uint8_t i = EXTRA_PROBING; i--;)
        if (ABS(probes[max_avg_idx] - median) > ABS(probes[min_avg_idx] - median))
          max_avg_idx--; else min_avg_idx++;

      // Return the average value of all remaining probes.//返回所有剩余探针的平均值。
      LOOP_S_LE_N(i, min_avg_idx, max_avg_idx)
        probes_z_sum += probes[i];

    #endif

    const float measured_z = probes_z_sum * RECIPROCAL(MULTIPLE_PROBING);

  #elif TOTAL_PROBING == 2

    const float z2 = current_position.z;

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("2nd Probe Z:", z2, " Discrepancy:", first_probe_z - z2);

    // Return a weighted average of the fast and slow probes//返回快速和慢速探针的加权平均值
    const float measured_z = (z2 * 3.0 + first_probe_z * 2.0) * 0.2;

  #else

    // Return the single probe result//返回单探针结果
    const float measured_z = current_position.z;

  #endif

  return measured_z;
}

/**
 * - Move to the given XY
 * - Deploy the probe, if not already deployed
 * - Probe the bed, get the Z position
 * - Depending on the 'stow' flag
 *   - Stow the probe, or
 *   - Raise to the BETWEEN height
 * - Return the probed Z position
 */
float Probe::probe_at_point(const_float_t rx, const_float_t ry, const ProbePtRaise raise_after/*=PROBE_PT_NONE*/, const uint8_t verbose_level/*=0*/, const bool probe_relative/*=true*/, const bool sanity_check/*=true*/) {
  DEBUG_SECTION(log_probe, "Probe::probe_at_point", DEBUGGING(LEVELING));

  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOLNPAIR(
      "...(", LOGICAL_X_POSITION(rx), ", ", LOGICAL_Y_POSITION(ry),
      ", ", raise_after == PROBE_PT_RAISE ? "raise" : raise_after == PROBE_PT_STOW ? "stow" : "none",
      ", ", verbose_level,
      ", ", probe_relative ? "probe" : "nozzle", "_relative)"
    );
    DEBUG_POS("", current_position);
  }

  #if BOTH(BLTOUCH, BLTOUCH_HS_MODE)
    if (bltouch.triggered()) bltouch._reset();
  #endif

  // On delta keep Z below clip height or do_blocking_move_to will abort//在delta上，将Z保持在剪辑高度以下或执行阻止移动将中止
  xyz_pos_t npos = { rx, ry, _MIN(TERN(DELTA, delta_clip_start_height, current_position.z), current_position.z) };
  if (probe_relative) {                                     // The given position is in terms of the probe//给定的位置是在探针方面
    if (!can_reach(npos)) {
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Position Not Reachable");
      return NAN;
    }
    npos -= offset_xy;                                      // Get the nozzle position//获取喷嘴位置
  }
  else if (!position_is_reachable(npos)) return NAN;        // The given position is in terms of the nozzle//给定的位置与喷嘴有关

  // Move the probe to the starting XYZ//将探头移动到起始XYZ
  do_blocking_move_to(npos, feedRate_t(XY_PROBE_FEEDRATE_MM_S));

  float measured_z = NAN;
  if (!deploy()) measured_z = run_z_probe(sanity_check) + offset.z;
  if (!isnan(measured_z)) {
    const bool big_raise = raise_after == PROBE_PT_BIG_RAISE;
    if (big_raise || raise_after == PROBE_PT_RAISE)
      do_blocking_move_to_z(current_position.z + (big_raise ? 25 : Z_CLEARANCE_BETWEEN_PROBES), z_probe_fast_mm_s);
    else if (raise_after == PROBE_PT_STOW)
      if (stow()) measured_z = NAN;   // Error on stow?//积载错误？

    if (verbose_level > 2)
      SERIAL_ECHOLNPAIR("Bed X: ", LOGICAL_X_POSITION(rx), " Y: ", LOGICAL_Y_POSITION(ry), " Z: ", measured_z);
  }

  if (isnan(measured_z)) {
    stow();
    LCD_MESSAGEPGM(MSG_LCD_PROBING_FAILED);
    #if DISABLED(G29_RETRY_AND_RECOVER)
      SERIAL_ERROR_MSG(STR_ERR_PROBING_FAILED);
    #endif
  }

  return measured_z;
}

#if HAS_Z_SERVO_PROBE

  void Probe::servo_probe_init() {
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     */
    STOW_Z_SERVO();
  }

#endif // HAS_Z_SERVO_PROBE//有_Z_伺服_探头

#endif // HAS_BED_PROBE//你有床吗
