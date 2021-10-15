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

#include "tool_change.h"

#include "probe.h"
#include "motion.h"
#include "planner.h"
#include "temperature.h"

#include "../MarlinCore.h"

//#define DEBUG_TOOL_CHANGE//#定义调试工具更改

#define DEBUG_OUT ENABLED(DEBUG_TOOL_CHANGE)
#include "../core/debug_out.h"

#if HAS_MULTI_EXTRUDER
  toolchange_settings_t toolchange_settings;  // Initialized by settings.load()//由settings.load（）初始化
#endif

#if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
  migration_settings_t migration = migration_defaults;
  bool enable_first_prime;
#endif

#if ENABLED(TOOLCHANGE_FS_INIT_BEFORE_SWAP)
  bool toolchange_extruder_ready[EXTRUDERS];
#endif

#if EITHER(MAGNETIC_PARKING_EXTRUDER, TOOL_SENSOR) \
  || defined(EVENT_GCODE_TOOLCHANGE_T0) || defined(EVENT_GCODE_TOOLCHANGE_T1) || defined(EVENT_GCODE_AFTER_TOOLCHANGE) \
  || (ENABLED(PARKING_EXTRUDER) && PARKING_EXTRUDER_SOLENOIDS_DELAY > 0)
  #include "../gcode/gcode.h"
#endif

#if ENABLED(TOOL_SENSOR)
  #include "../lcd/marlinui.h"
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  #include "stepper.h"
#endif

#if ANY(SWITCHING_EXTRUDER, SWITCHING_NOZZLE, SWITCHING_TOOLHEAD)
  #include "servo.h"
#endif

#if ENABLED(EXT_SOLENOID) && DISABLED(PARKING_EXTRUDER)
  #include "../feature/solenoid.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if HAS_FANMUX
  #include "../feature/fanmux.h"
#endif

#if HAS_PRUSA_MMU1
  #include "../feature/mmu/mmu.h"
#elif HAS_PRUSA_MMU2
  #include "../feature/mmu/mmu2.h"
#endif

#if HAS_LCD_MENU
  #include "../lcd/marlinui.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../feature/pause.h"
#endif

#if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
  #include "../gcode/gcode.h"
  #if TOOLCHANGE_FS_WIPE_RETRACT <= 0
    #undef TOOLCHANGE_FS_WIPE_RETRACT
    #define TOOLCHANGE_FS_WIPE_RETRACT 0
  #endif
#endif

#if DO_SWITCH_EXTRUDER

  #if EXTRUDERS > 3
    #define _SERVO_NR(E) ((E) < 2 ? SWITCHING_EXTRUDER_SERVO_NR : SWITCHING_EXTRUDER_E23_SERVO_NR)
  #else
    #define _SERVO_NR(E) SWITCHING_EXTRUDER_SERVO_NR
  #endif

  void move_extruder_servo(const uint8_t e) {
    planner.synchronize();
    if ((EXTRUDERS & 1) && e < EXTRUDERS - 1) {
      MOVE_SERVO(_SERVO_NR(e), servo_angles[_SERVO_NR(e)][e & 1]);
      safe_delay(500);
    }
  }

#endif // DO_SWITCH_EXTRUDER//请勿切换挤出机

#if ENABLED(SWITCHING_NOZZLE)

  #if SWITCHING_NOZZLE_TWO_SERVOS

    inline void _move_nozzle_servo(const uint8_t e, const uint8_t angle_index) {
      constexpr int8_t  sns_index[2] = { SWITCHING_NOZZLE_SERVO_NR, SWITCHING_NOZZLE_E1_SERVO_NR };
      constexpr int16_t sns_angles[2] = SWITCHING_NOZZLE_SERVO_ANGLES;
      planner.synchronize();
      MOVE_SERVO(sns_index[e], sns_angles[angle_index]);
      safe_delay(500);
    }

    void lower_nozzle(const uint8_t e) { _move_nozzle_servo(e, 0); }
    void raise_nozzle(const uint8_t e) { _move_nozzle_servo(e, 1); }

  #else

    void move_nozzle_servo(const uint8_t angle_index) {
      planner.synchronize();
      MOVE_SERVO(SWITCHING_NOZZLE_SERVO_NR, servo_angles[SWITCHING_NOZZLE_SERVO_NR][angle_index]);
      safe_delay(500);
    }

  #endif

#endif // SWITCHING_NOZZLE//切换喷嘴

void _line_to_current(const AxisEnum fr_axis, const float fscale=1) {
  line_to_current_position(planner.settings.max_feedrate_mm_s[fr_axis] * fscale);
}
void slow_line_to_current(const AxisEnum fr_axis) { _line_to_current(fr_axis, 0.2f); }
void fast_line_to_current(const AxisEnum fr_axis) { _line_to_current(fr_axis, 0.5f); }

#if ENABLED(MAGNETIC_PARKING_EXTRUDER)

  float parkingposx[2],           // M951 R L//M951 R L
        parkinggrabdistance,      // M951 I//M951 I
        parkingslowspeed,         // M951 J//M951 J
        parkinghighspeed,         // M951 H//M951H
        parkingtraveldistance,    // M951 D//M951 D
        compensationmultiplier;

  inline void magnetic_parking_extruder_tool_change(const uint8_t new_tool) {

    const float oldx = current_position.x,
                grabpos = mpe_settings.parking_xpos[new_tool] + (new_tool ? mpe_settings.grab_distance : -mpe_settings.grab_distance),
                offsetcompensation = TERN0(HAS_HOTEND_OFFSET, hotend_offset[active_extruder].x * mpe_settings.compensation_factor);

    if (homing_needed_error(_BV(X_AXIS))) return;

    /**
     * Z Lift and Nozzle Offset shift ar defined in caller method to work equal with any Multi Hotend realization
     *
     * Steps:
     *   1. Move high speed to park position of new extruder
     *   2. Move to couple position of new extruder (this also discouple the old extruder)
     *   3. Move to park position of new extruder
     *   4. Move high speed to approach park position of old extruder
     *   5. Move to park position of old extruder
     *   6. Move to starting position
     */

    // STEP 1//第一步

    current_position.x = mpe_settings.parking_xpos[new_tool] + offsetcompensation;

    DEBUG_ECHOPAIR("(1) Move extruder ", new_tool);
    DEBUG_POS(" to new extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    // STEP 2//步骤2

    current_position.x = grabpos + offsetcompensation;

    DEBUG_ECHOPAIR("(2) Couple extruder ", new_tool);
    DEBUG_POS(" to new extruder GrabPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // Delay before moving tool, to allow magnetic coupling//移动工具前的延迟，以允许磁耦合
    gcode.dwell(150);

    // STEP 3//步骤3

    current_position.x = mpe_settings.parking_xpos[new_tool] + offsetcompensation;

    DEBUG_ECHOPAIR("(3) Move extruder ", new_tool);
    DEBUG_POS(" back to new extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // STEP 4//步骤4

    current_position.x = mpe_settings.parking_xpos[active_extruder] + (active_extruder == 0 ? MPE_TRAVEL_DISTANCE : -MPE_TRAVEL_DISTANCE) + offsetcompensation;

    DEBUG_ECHOPAIR("(4) Move extruder ", new_tool);
    DEBUG_POS(" close to old extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    // STEP 5//步骤5

    current_position.x = mpe_settings.parking_xpos[active_extruder] + offsetcompensation;

    DEBUG_ECHOPAIR("(5) Park extruder ", new_tool);
    DEBUG_POS(" at old extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // STEP 6//步骤6

    current_position.x = oldx;

    DEBUG_ECHOPAIR("(6) Move extruder ", new_tool);
    DEBUG_POS(" to starting position", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    DEBUG_ECHOLNPGM("Autopark done.");
  }

#elif ENABLED(PARKING_EXTRUDER)

  void pe_solenoid_init() {
    LOOP_LE_N(n, 1) pe_solenoid_set_pin_state(n, !PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
  }

  void pe_solenoid_set_pin_state(const uint8_t extruder_num, const uint8_t state) {
    switch (extruder_num) {
      case 1: OUT_WRITE(SOL1_PIN, state); break;
      default: OUT_WRITE(SOL0_PIN, state); break;
    }
    #if PARKING_EXTRUDER_SOLENOIDS_DELAY > 0
      gcode.dwell(PARKING_EXTRUDER_SOLENOIDS_DELAY);
    #endif
  }

  bool extruder_parked = true, do_solenoid_activation = true;

  // Modifies tool_change() behavior based on homing side//基于归位侧修改工具的\u change（）行为
  bool parking_extruder_unpark_after_homing(const uint8_t final_tool, bool homed_towards_final_tool) {
    do_solenoid_activation = false; // Tell parking_extruder_tool_change to skip solenoid activation//告诉停车场\u挤出机\u刀具\u更换以跳过电磁阀激活

    if (!extruder_parked) return false; // nothing to do//无事可做

    if (homed_towards_final_tool) {
      pe_solenoid_magnet_off(1 - final_tool);
      DEBUG_ECHOLNPAIR("Disengage magnet", 1 - final_tool);
      pe_solenoid_magnet_on(final_tool);
      DEBUG_ECHOLNPAIR("Engage magnet", final_tool);
      parking_extruder_set_parked(false);
      return false;
    }

    return true;
  }

  inline void parking_extruder_tool_change(const uint8_t new_tool, bool no_move) {
    if (!no_move) {

      constexpr float parkingposx[] = PARKING_EXTRUDER_PARKING_X;

      #if HAS_HOTEND_OFFSET
        const float x_offset = hotend_offset[active_extruder].x;
      #else
        constexpr float x_offset = 0;
      #endif

      const float midpos = (parkingposx[0] + parkingposx[1]) * 0.5f + x_offset,
                  grabpos = parkingposx[new_tool] + (new_tool ? PARKING_EXTRUDER_GRAB_DISTANCE : -(PARKING_EXTRUDER_GRAB_DISTANCE)) + x_offset;

      /**
       * 1. Move to park position of old extruder
       * 2. Disengage magnetic field, wait for delay
       * 3. Move near new extruder
       * 4. Engage magnetic field for new extruder
       * 5. Move to parking incl. offset of new extruder
       * 6. Lower Z-Axis
       */

      // STEP 1//第一步

      DEBUG_POS("Start PE Tool-Change", current_position);

      // Don't park the active_extruder unless unparked//除非断开，否则不要停放活动的_挤出机
      if (!extruder_parked) {
        current_position.x = parkingposx[active_extruder] + x_offset;

        DEBUG_ECHOLNPAIR("(1) Park extruder ", active_extruder);
        DEBUG_POS("Moving ParkPos", current_position);

        fast_line_to_current(X_AXIS);

        // STEP 2//步骤2

        planner.synchronize();
        DEBUG_ECHOLNPGM("(2) Disengage magnet");
        pe_solenoid_magnet_off(active_extruder);

        // STEP 3//步骤3

        current_position.x += active_extruder ? -10 : 10; // move 10mm away from parked extruder//从停放的挤出机移开10mm

        DEBUG_ECHOLNPGM("(3) Move near new extruder");
        DEBUG_POS("Move away from parked extruder", current_position);

        fast_line_to_current(X_AXIS);
      }

      // STEP 4//步骤4

      planner.synchronize();
      DEBUG_ECHOLNPGM("(4) Engage magnetic field");

      // Just save power for inverted magnets//只需为倒置的磁铁省电即可
      TERN_(PARKING_EXTRUDER_SOLENOIDS_INVERT, pe_solenoid_magnet_on(active_extruder));
      pe_solenoid_magnet_on(new_tool);

      // STEP 5//步骤5

      current_position.x = grabpos + (new_tool ? -10 : 10);
      fast_line_to_current(X_AXIS);

      current_position.x = grabpos;

      DEBUG_SYNCHRONIZE();
      DEBUG_POS("(5) Unpark extruder", current_position);

      slow_line_to_current(X_AXIS);

      // STEP 6//步骤6

      current_position.x = DIFF_TERN(HAS_HOTEND_OFFSET, midpos, hotend_offset[new_tool].x);

      DEBUG_SYNCHRONIZE();
      DEBUG_POS("(6) Move midway between hotends", current_position);

      fast_line_to_current(X_AXIS);
      planner.synchronize(); // Always sync the final move//始终同步最后一步

      DEBUG_POS("PE Tool-Change done.", current_position);
      parking_extruder_set_parked(false);
    }
    else if (do_solenoid_activation) {
      // Deactivate current extruder solenoid//停用当前挤出机电磁阀
      pe_solenoid_set_pin_state(active_extruder, !PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
      // Engage new extruder magnetic field//接合新的挤出机磁场
      pe_solenoid_set_pin_state(new_tool, PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
    }

    do_solenoid_activation = true; // Activate solenoid for subsequent tool_change()//启动电磁阀以进行后续的工具更换（）
  }

#endif // PARKING_EXTRUDER//挤压机

#if ENABLED(SWITCHING_TOOLHEAD)

  // Return a bitmask of tool sensor states//返回刀具传感器状态的位掩码
  inline uint8_t poll_tool_sensor_pins() {
    return (0
      #if ENABLED(TOOL_SENSOR)
        #if PIN_EXISTS(TOOL_SENSOR1)
          | (READ(TOOL_SENSOR1_PIN) << 0)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR2)
          | (READ(TOOL_SENSOR2_PIN) << 1)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR3)
          | (READ(TOOL_SENSOR3_PIN) << 2)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR4)
          | (READ(TOOL_SENSOR4_PIN) << 3)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR5)
          | (READ(TOOL_SENSOR5_PIN) << 4)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR6)
          | (READ(TOOL_SENSOR6_PIN) << 5)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR7)
          | (READ(TOOL_SENSOR7_PIN) << 6)
        #endif
        #if PIN_EXISTS(TOOL_SENSOR8)
          | (READ(TOOL_SENSOR8_PIN) << 7)
        #endif
      #endif
    );
  }

  #if ENABLED(TOOL_SENSOR)

    bool tool_sensor_disabled; // = false//=错误

    uint8_t check_tool_sensor_stats(const uint8_t tool_index, const bool kill_on_error/*=false*/, const bool disable/*=false*/) {
      static uint8_t sensor_tries; // = 0// = 0
      for (;;) {
        if (poll_tool_sensor_pins() == _BV(tool_index)) {
          sensor_tries = 0;
          return tool_index;
        }
        else if (kill_on_error && (!tool_sensor_disabled || disable)) {
          sensor_tries++;
          if (sensor_tries > 10) kill(PSTR("Tool Sensor error"));
          safe_delay(5);
        }
        else {
          sensor_tries++;
          if (sensor_tries > 10) return -1;
          safe_delay(5);
        }
      }
    }

  #endif

  inline void switching_toolhead_lock(const bool locked) {
    #ifdef SWITCHING_TOOLHEAD_SERVO_ANGLES
      const uint16_t swt_angles[2] = SWITCHING_TOOLHEAD_SERVO_ANGLES;
      MOVE_SERVO(SWITCHING_TOOLHEAD_SERVO_NR, swt_angles[locked ? 0 : 1]);
    #elif PIN_EXISTS(SWT_SOLENOID)
      OUT_WRITE(SWT_SOLENOID_PIN, locked);
      gcode.dwell(10);
    #else
      #error "No toolhead locking mechanism configured."
    #endif
  }

  #include <bitset>

  void swt_init() {
    switching_toolhead_lock(true);

    #if ENABLED(TOOL_SENSOR)
      // Init tool sensors//初始化工具传感器
      #if PIN_EXISTS(TOOL_SENSOR1)
        SET_INPUT_PULLUP(TOOL_SENSOR1_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR2)
        SET_INPUT_PULLUP(TOOL_SENSOR2_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR3)
        SET_INPUT_PULLUP(TOOL_SENSOR3_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR4)
        SET_INPUT_PULLUP(TOOL_SENSOR4_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR5)
        SET_INPUT_PULLUP(TOOL_SENSOR5_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR6)
        SET_INPUT_PULLUP(TOOL_SENSOR6_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR7)
        SET_INPUT_PULLUP(TOOL_SENSOR7_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR8)
        SET_INPUT_PULLUP(TOOL_SENSOR8_PIN);
      #endif

      if (check_tool_sensor_stats(0)) {
        ui.set_status_P("TC error");
        switching_toolhead_lock(false);
        while (check_tool_sensor_stats(0)) { /* nada */ }
        switching_toolhead_lock(true);
      }
      ui.set_status_P("TC Success");
    #endif
  }

  inline void switching_toolhead_tool_change(const uint8_t new_tool, bool no_move/*=false*/) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS;
    const float placexpos = toolheadposx[active_extruder],
                grabxpos = toolheadposx[new_tool];

    (void)check_tool_sensor_stats(active_extruder, true);

    /**
     * 1. Move to switch position of current toolhead
     * 2. Unlock tool and drop it in the dock
     * 3. Move to the new toolhead
     * 4. Grab and lock the new toolhead
     */

    // 1. Move to switch position of current toolhead// 1. 移动到当前刀头的开关位置

    DEBUG_POS("Start ST Tool-Change", current_position);

    current_position.x = placexpos;

    DEBUG_ECHOLNPAIR("(1) Place old tool ", active_extruder);
    DEBUG_POS("Move X SwitchPos", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS - (SWITCHING_TOOLHEAD_Y_SECURITY);

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    slow_line_to_current(Y_AXIS);

    // 2. Unlock tool and drop it in the dock// 2. 解锁工具并将其放入坞中
    TERN_(TOOL_SENSOR, tool_sensor_disabled = true);

    planner.synchronize();
    DEBUG_ECHOLNPGM("(2) Unlock and Place Toolhead");
    switching_toolhead_lock(false);
    safe_delay(500);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;
    DEBUG_POS("Move Y SwitchPos", current_position);
    slow_line_to_current(Y_AXIS);

    // Wait for move to complete, then another 0.2s//等待移动完成，然后再等待0.2秒
    planner.synchronize();
    safe_delay(200);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    slow_line_to_current(Y_AXIS); // move away from docked toolhead//远离停靠的工具头

    (void)check_tool_sensor_stats(active_extruder);

    // 3. Move to the new toolhead// 3. 移动到新的工具头

    current_position.x = grabxpos;

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(3) Move to new toolhead position");
    DEBUG_POS("Move to new toolhead X", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS - (SWITCHING_TOOLHEAD_Y_SECURITY);

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    slow_line_to_current(Y_AXIS);

    // 4. Grab and lock the new toolhead// 4. 抓住并锁定新的工具头

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(4) Grab and lock new toolhead");
    DEBUG_POS("Move Y SwitchPos", current_position);

    slow_line_to_current(Y_AXIS);

    // Wait for move to finish, pause 0.2s, move servo, pause 0.5s//等待移动完成，暂停0.2秒，移动伺服，暂停0.5秒
    planner.synchronize();
    safe_delay(200);

    (void)check_tool_sensor_stats(new_tool, true, true);

    switching_toolhead_lock(true);
    safe_delay(500);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    slow_line_to_current(Y_AXIS); // Move away from docked toolhead//远离停靠的工具头
    planner.synchronize();        // Always sync the final move//始终同步最后一步

    (void)check_tool_sensor_stats(new_tool, true, true);

    DEBUG_POS("ST Tool-Change done.", current_position);
  }

#elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)

  inline void magnetic_switching_toolhead_tool_change(const uint8_t new_tool, bool no_move/*=false*/) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS,
                    toolheadclearx[] = SWITCHING_TOOLHEAD_X_SECURITY;

    const float placexpos = toolheadposx[active_extruder],
                placexclear = toolheadclearx[active_extruder],
                grabxpos = toolheadposx[new_tool],
                grabxclear = toolheadclearx[new_tool];

    /**
     * 1. Move to switch position of current toolhead
     * 2. Release and place toolhead in the dock
     * 3. Move to the new toolhead
     * 4. Grab the new toolhead and move to security position
     */

    DEBUG_POS("Start MST Tool-Change", current_position);

    // 1. Move to switch position current toolhead// 1. 移动到当前刀头的开关位置

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR;

    SERIAL_ECHOLNPAIR("(1) Place old tool ", active_extruder);
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    fast_line_to_current(Y_AXIS);

    current_position.x = placexclear;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move X SwitchPos + Security", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos", current_position);

    fast_line_to_current(Y_AXIS);

    current_position.x = placexpos;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move X SwitchPos", current_position);

    line_to_current_position(planner.settings.max_feedrate_mm_s[X_AXIS] * 0.25f);

    // 2. Release and place toolhead in the dock// 2. 释放工具头并将其放入坞中

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(2) Release and Place Toolhead");

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_RELEASE;
    DEBUG_POS("Move Y SwitchPos + Release", current_position);
    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS] * 0.1f);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_SECURITY;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS]);

    // 3. Move to new toolhead position// 3. 移动到新的刀头位置

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(3) Move to new toolhead position");

    current_position.x = grabxpos;
    DEBUG_POS("Move to new toolhead X", current_position);
    fast_line_to_current(X_AXIS);

    // 4. Grab the new toolhead and move to security position// 4. 抓住新工具头并移动到安全位置

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(4) Grab new toolhead, move to security position");

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_RELEASE;
    DEBUG_POS("Move Y SwitchPos + Release", current_position);
    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS]);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos", current_position);

    _line_to_current(Y_AXIS, 0.2f);

    #if ENABLED(PRIME_BEFORE_REMOVE) && (SWITCHING_TOOLHEAD_PRIME_MM || SWITCHING_TOOLHEAD_RETRACT_MM)
      #if SWITCHING_TOOLHEAD_PRIME_MM
        current_position.e += SWITCHING_TOOLHEAD_PRIME_MM;
        planner.buffer_line(current_position, MMM_TO_MMS(SWITCHING_TOOLHEAD_PRIME_FEEDRATE), new_tool);
      #endif
      #if SWITCHING_TOOLHEAD_RETRACT_MM
        current_position.e -= SWITCHING_TOOLHEAD_RETRACT_MM;
        planner.buffer_line(current_position, MMM_TO_MMS(SWITCHING_TOOLHEAD_RETRACT_FEEDRATE), new_tool);
      #endif
    #else
      planner.synchronize();
      safe_delay(100); // Give switch time to settle//给转换时间来解决
    #endif

    current_position.x = grabxclear;
    DEBUG_POS("Move to new toolhead X + Security", current_position);
    _line_to_current(X_AXIS, 0.1f);
    planner.synchronize();
    safe_delay(100); // Give switch time to settle//给转换时间来解决

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    fast_line_to_current(Y_AXIS); // move away from docked toolhead//远离停靠的工具头
    planner.synchronize(); // Always sync last tool-change move//始终同步上次换刀移动

    DEBUG_POS("MST Tool-Change done.", current_position);
  }

#elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)

  inline void est_activate_solenoid()   { OUT_WRITE(SOL0_PIN, HIGH); }
  inline void est_deactivate_solenoid() { OUT_WRITE(SOL0_PIN, LOW); }
  void est_init() { est_activate_solenoid(); }

  inline void em_switching_toolhead_tool_change(const uint8_t new_tool, bool no_move) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS;
    const float placexpos = toolheadposx[active_extruder],
                grabxpos = toolheadposx[new_tool];
    const xyz_pos_t &hoffs = hotend_offset[active_extruder];

    /**
     * 1. Raise Z-Axis to give enough clearance
     * 2. Move to position near active extruder parking
     * 3. Move gently to park position of active extruder
     * 4. Disengage magnetic field, wait for delay
     * 5. Leave extruder and move to position near new extruder parking
     * 6. Move gently to park position of new extruder
     * 7. Engage magnetic field for new extruder parking
     * 8. Unpark extruder
     * 9. Apply Z hotend offset to current position
     */

    DEBUG_POS("Start EMST Tool-Change", current_position);

    // 1. Raise Z-Axis to give enough clearance// 1. 提升Z轴以提供足够的间隙

    current_position.z += SWITCHING_TOOLHEAD_Z_HOP;
    DEBUG_POS("(1) Raise Z-Axis ", current_position);
    fast_line_to_current(Z_AXIS);

    // 2. Move to position near active extruder parking// 2. 移动到主动挤出机停车位附近的位置

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPAIR("(2) Move near active extruder parking", active_extruder);
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.set(hoffs.x + placexpos,
                         hoffs.y + SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR);
    fast_line_to_current(X_AXIS);

    // 3. Move gently to park position of active extruder// 3. 轻轻移动到主动挤出机的停车位置

    DEBUG_SYNCHRONIZE();
    SERIAL_ECHOLNPAIR("(3) Move gently to park position of active extruder", active_extruder);
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    slow_line_to_current(Y_AXIS);

    // 4. Disengage magnetic field, wait for delay// 4. 断开磁场，等待延迟

    planner.synchronize();
    DEBUG_ECHOLNPGM("(4) Disengage magnet");
    est_deactivate_solenoid();

    // 5. Leave extruder and move to position near new extruder parking// 5. 离开挤出机，移动到新挤出机停车场附近的位置

    DEBUG_ECHOLNPGM("(5) Move near new extruder parking");
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    slow_line_to_current(Y_AXIS);
    current_position.set(hoffs.x + grabxpos,
                         hoffs.y + SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR);
    fast_line_to_current(X_AXIS);

    // 6. Move gently to park position of new extruder// 6. 轻轻移动到新挤出机的停车位置

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    if (DEBUGGING(LEVELING)) {
      planner.synchronize();
      DEBUG_ECHOLNPGM("(6) Move near new extruder");
    }
    slow_line_to_current(Y_AXIS);

    // 7. Engage magnetic field for new extruder parking// 7. 为新挤出机停车接合磁场

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(7) Engage magnetic field");
    est_activate_solenoid();

    // 8. Unpark extruder// 8. Unpark挤出机

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_ECHOLNPGM("(8) Unpark extruder");
    slow_line_to_current(X_AXIS);
    planner.synchronize(); // Always sync the final move//始终同步最后一步

    // 9. Apply Z hotend offset to current position// 9. 将Z热端偏移应用于当前位置

    DEBUG_POS("(9) Applying Z-offset", current_position);
    current_position.z += hoffs.z - hotend_offset[new_tool].z;

    DEBUG_POS("EMST Tool-Change done.", current_position);
  }

#endif // ELECTROMAGNETIC_SWITCHING_TOOLHEAD//电磁开关工具头

#if HAS_EXTRUDERS
  inline void invalid_extruder_error(const uint8_t e) {
    SERIAL_ECHO_START();
    SERIAL_CHAR('T'); SERIAL_ECHO(e);
    SERIAL_CHAR(' '); SERIAL_ECHOLNPGM(STR_INVALID_EXTRUDER);
  }
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * @brief Dual X Tool Change
   * @details Change tools, with extra behavior based on current mode
   *
   * @param new_tool Tool index to activate
   * @param no_move Flag indicating no moves should take place
   */
  inline void dualx_tool_change(const uint8_t new_tool, bool &no_move) {

    DEBUG_ECHOPGM("Dual X Carriage Mode ");
    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE: DEBUG_ECHOLNPGM("FULL_CONTROL"); break;
      case DXC_AUTO_PARK_MODE:    DEBUG_ECHOLNPGM("AUTO_PARK");    break;
      case DXC_DUPLICATION_MODE:  DEBUG_ECHOLNPGM("DUPLICATION");  break;
      case DXC_MIRRORED_MODE:     DEBUG_ECHOLNPGM("MIRRORED");     break;
    }

    // Get the home position of the currently-active tool//获取当前激活刀具的原始位置
    const float xhome = x_home_pos(active_extruder);

    if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE                  // If Auto-Park mode is enabled//如果启用自动驻车模式
        && IsRunning() && !no_move                                  // ...and movement is permitted//…并且允许移动
        && (delayed_move_time || current_position.x != xhome)       // ...and delayed_move_time is set OR not "already parked"...//…并且延迟移动时间已设置或未“已驻车”。。。
    ) {
      DEBUG_ECHOLNPAIR("MoveX to ", xhome);
      current_position.x = xhome;
      line_to_current_position(planner.settings.max_feedrate_mm_s[X_AXIS]);   // Park the current head//停在当前的头上
      planner.synchronize();
    }

    // Activate the new extruder ahead of calling set_axis_is_at_home!//在调用set_axis_is_at_home之前激活新挤出机！
    active_extruder = new_tool;

    // This function resets the max/min values - the current position may be overwritten below.//此功能重置最大/最小值-当前位置可能会被覆盖。
    set_axis_is_at_home(X_AXIS);

    DEBUG_POS("New Extruder", current_position);

    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
        // New current position is the position of the activated extruder//新的当前位置是激活挤出机的位置
        current_position.x = inactive_extruder_x;
        // Save the inactive extruder's position (from the old current_position)//保存非活动挤出机的位置（从旧的当前位置）
        inactive_extruder_x = destination.x;
        DEBUG_ECHOLNPAIR("DXC Full Control curr.x=", current_position.x, " dest.x=", destination.x);
        break;
      case DXC_AUTO_PARK_MODE:
        idex_set_parked();
        break;
      default:
        break;
    }

    // Ensure X axis DIR pertains to the correct carriage//确保X轴方向与正确的托架相关
    stepper.set_directions();

    DEBUG_ECHOLNPAIR("Active extruder parked: ", active_extruder_parked ? "yes" : "no");
    DEBUG_POS("New extruder (parked)", current_position);
  }

#endif // DUAL_X_CARRIAGE//双_X_车厢

/**
 * Prime active tool using TOOLCHANGE_FILAMENT_SWAP settings
 */
#if ENABLED(TOOLCHANGE_FILAMENT_SWAP)

  void tool_change_prime() {
    if (toolchange_settings.extra_prime > 0
      && TERN(PREVENT_COLD_EXTRUSION, !thermalManager.targetTooColdToExtrude(active_extruder), 1)
    ) {
      destination = current_position; // Remember the old position//还记得以前的位置吗

      const bool ok = TERN1(TOOLCHANGE_PARK, all_axes_homed() && toolchange_settings.enable_park);

      #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
        // Store and stop fan. Restored on any exit.//储存并停止风扇。在任何出口恢复。
        REMEMBER(fan, thermalManager.fan_speed[TOOLCHANGE_FS_FAN], 0);
      #endif

      // Z raise//Z升起
      if (ok) {
        // Do a small lift to avoid the workpiece in the move back (below)//进行小幅度提升，以避免工件向后移动（下图）
        current_position.z += toolchange_settings.z_raise;
        #if HAS_SOFTWARE_ENDSTOPS
          NOMORE(current_position.z, soft_endstop.max.z);
        #endif
        fast_line_to_current(Z_AXIS);
        planner.synchronize();
      }

      // Park//停车场
      #if ENABLED(TOOLCHANGE_PARK)
        if (ok) {
          IF_DISABLED(TOOLCHANGE_PARK_Y_ONLY, current_position.x = toolchange_settings.change_point.x);
          IF_DISABLED(TOOLCHANGE_PARK_X_ONLY, current_position.y = toolchange_settings.change_point.y);
          planner.buffer_line(current_position, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE), active_extruder);
          planner.synchronize();
        }
      #endif

      // Prime (All distances are added and slowed down to ensure secure priming in all circumstances)//充注（所有距离都增加并减慢，以确保在所有情况下都能安全充注）
      unscaled_e_move(toolchange_settings.swap_length + toolchange_settings.extra_prime, MMM_TO_MMS(toolchange_settings.prime_speed));

      // Cutting retraction//切削收缩
      #if TOOLCHANGE_FS_WIPE_RETRACT
        unscaled_e_move(-(TOOLCHANGE_FS_WIPE_RETRACT), MMM_TO_MMS(toolchange_settings.retract_speed));
      #endif

      // Cool down with fan//用风扇冷却
      #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
        thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = toolchange_settings.fan_speed;
        gcode.dwell(SEC_TO_MS(toolchange_settings.fan_time));
        thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = 0;
      #endif

      // Move back//退后
      #if ENABLED(TOOLCHANGE_PARK)
        if (ok) {
          #if ENABLED(TOOLCHANGE_NO_RETURN)
            destination.set(current_position.x, current_position.y);
            prepare_internal_move_to_destination(planner.settings.max_feedrate_mm_s[Z_AXIS]);
          #else
            prepare_internal_move_to_destination(MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE));
          #endif
        }
      #endif

      // Cutting recover//切割回收
      unscaled_e_move(toolchange_settings.extra_resume + TOOLCHANGE_FS_WIPE_RETRACT, MMM_TO_MMS(toolchange_settings.unretract_speed));

      // Resume at the old E position//恢复到原来的E位置
      current_position.e = destination.e;
      sync_plan_position_e();
    }
  }

#endif // TOOLCHANGE_FILAMENT_SWAP//更换工具\u灯丝\u更换

/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t new_tool, bool no_move/*=false*/) {

  if (TERN0(MAGNETIC_SWITCHING_TOOLHEAD, new_tool == active_extruder))
    return;

  #if ENABLED(MIXING_EXTRUDER)

    UNUSED(no_move);

    if (new_tool >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(new_tool);

    #if MIXING_VIRTUAL_TOOLS > 1
      // T0-Tnnn: Switch virtual tool by changing the index to the mix//T0 Tnnn：通过将索引更改为混合来切换虚拟工具
      mixer.T(new_tool);
    #endif

  #elif HAS_PRUSA_MMU2

    UNUSED(no_move);

    mmu2.tool_change(new_tool);

  #elif EXTRUDERS == 0

    // Nothing to do//无事可做
    UNUSED(new_tool); UNUSED(no_move);

  #elif EXTRUDERS < 2

    UNUSED(no_move);

    if (new_tool) invalid_extruder_error(new_tool);
    return;

  #elif HAS_MULTI_EXTRUDER

    planner.synchronize();

    #if ENABLED(DUAL_X_CARRIAGE)  // Only T0 allowed if the Printer is in DXC_DUPLICATION_MODE or DXC_MIRRORED_MODE//仅当打印机处于DXC_复制模式或DXC_镜像模式时才允许T0
      if (new_tool != 0 && idex_is_duplicating())
         return invalid_extruder_error(new_tool);
    #endif

    if (new_tool >= EXTRUDERS)
      return invalid_extruder_error(new_tool);

    if (!no_move && homing_needed()) {
      no_move = true;
      DEBUG_ECHOLNPGM("No move (not homed)");
    }

    TERN_(HAS_LCD_MENU, if (!no_move) ui.update());

    #if ENABLED(DUAL_X_CARRIAGE)
      const bool idex_full_control = dual_x_carriage_mode == DXC_FULL_CONTROL_MODE;
    #else
      constexpr bool idex_full_control = false;
    #endif

    const uint8_t old_tool = active_extruder;
    const bool can_move_away = !no_move && !idex_full_control;

    #if HAS_LEVELING
      // Set current position to the physical position//将当前位置设置为物理位置
      TEMPORARY_BED_LEVELING_STATE(false);
    #endif

    // First tool priming. To prime again, reboot the machine.//第一个工具启动。要重新启动，请重新启动计算机。
    #if ENABLED(TOOLCHANGE_FS_PRIME_FIRST_USED)
      static bool first_tool_is_primed = false;
      if (new_tool == old_tool && !first_tool_is_primed && enable_first_prime) {
        tool_change_prime();
        first_tool_is_primed = true;
        TERN_(TOOLCHANGE_FS_INIT_BEFORE_SWAP, toolchange_extruder_ready[old_tool] = true); // Primed and initialized//启动和初始化
      }
    #endif

    if (new_tool != old_tool || TERN0(PARKING_EXTRUDER, extruder_parked)) { // PARKING_EXTRUDER may need to attach old_tool when homing//重新归位时，停车挤出机可能需要连接旧的工具
      destination = current_position;

      #if BOTH(TOOLCHANGE_FILAMENT_SWAP, HAS_FAN) && TOOLCHANGE_FS_FAN >= 0
        // Store and stop fan. Restored on any exit.//储存并停止风扇。在任何出口恢复。
        REMEMBER(fan, thermalManager.fan_speed[TOOLCHANGE_FS_FAN], 0);
      #endif

      // Z raise before retraction//Z缩回前升起
      #if ENABLED(TOOLCHANGE_ZRAISE_BEFORE_RETRACT) && DISABLED(SWITCHING_NOZZLE)
        if (can_move_away && TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park)) {
          // Do a small lift to avoid the workpiece in the move back (below)//进行小幅度提升，以避免工件向后移动（下图）
          current_position.z += toolchange_settings.z_raise;
          #if HAS_SOFTWARE_ENDSTOPS
            NOMORE(current_position.z, soft_endstop.max.z);
          #endif
          fast_line_to_current(Z_AXIS);
          planner.synchronize();
        }
      #endif

      // Unload / Retract//卸载/收回
      #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
        const bool should_swap = can_move_away && toolchange_settings.swap_length,
                   too_cold = TERN0(PREVENT_COLD_EXTRUSION,
                     !DEBUGGING(DRYRUN) && (thermalManager.targetTooColdToExtrude(old_tool) || thermalManager.targetTooColdToExtrude(new_tool))
                   );
        if (should_swap) {
          if (too_cold) {
            SERIAL_ECHO_MSG(STR_ERR_HOTEND_TOO_COLD);
            if (ENABLED(SINGLENOZZLE)) { active_extruder = new_tool; return; }
          }
          else {
            // For first new tool, change without unloading the old. 'Just prime/init the new'//对于第一个新刀具，在不卸载旧刀具的情况下进行更换。”只需初始化/初始化新的'
            if (TERN1(TOOLCHANGE_FS_PRIME_FIRST_USED, first_tool_is_primed))
              unscaled_e_move(-toolchange_settings.swap_length, MMM_TO_MMS(toolchange_settings.retract_speed));
            TERN_(TOOLCHANGE_FS_PRIME_FIRST_USED, first_tool_is_primed = true); // The first new tool will be primed by toolchanging//第一个新刀具将通过更换刀具来准备
          }
        }
      #endif

      TERN_(SWITCHING_NOZZLE_TWO_SERVOS, raise_nozzle(old_tool));

      REMEMBER(fr, feedrate_mm_s, XY_PROBE_FEEDRATE_MM_S);

      #if HAS_SOFTWARE_ENDSTOPS
        #if HAS_HOTEND_OFFSET
          #define _EXT_ARGS , old_tool, new_tool
        #else
          #define _EXT_ARGS
        #endif
        update_software_endstops(X_AXIS _EXT_ARGS);
        #if DISABLED(DUAL_X_CARRIAGE)
          update_software_endstops(Y_AXIS _EXT_ARGS);
          update_software_endstops(Z_AXIS _EXT_ARGS);
        #endif
      #endif

      #if DISABLED(TOOLCHANGE_ZRAISE_BEFORE_RETRACT) && DISABLED(SWITCHING_NOZZLE)
        if (can_move_away && TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park)) {
          // Do a small lift to avoid the workpiece in the move back (below)//进行小幅度提升，以避免工件向后移动（下图）
          current_position.z += toolchange_settings.z_raise;
          #if HAS_SOFTWARE_ENDSTOPS
            NOMORE(current_position.z, soft_endstop.max.z);
          #endif
          fast_line_to_current(Z_AXIS);
        }
      #endif

      // Toolchange park//工具更换公园
      #if ENABLED(TOOLCHANGE_PARK) && DISABLED(SWITCHING_NOZZLE)
        if (can_move_away && toolchange_settings.enable_park) {
          IF_DISABLED(TOOLCHANGE_PARK_Y_ONLY, current_position.x = toolchange_settings.change_point.x);
          IF_DISABLED(TOOLCHANGE_PARK_X_ONLY, current_position.y = toolchange_settings.change_point.y);
          planner.buffer_line(current_position, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE), old_tool);
          planner.synchronize();
        }
      #endif

      #if HAS_HOTEND_OFFSET
        xyz_pos_t diff = hotend_offset[new_tool] - hotend_offset[old_tool];
        TERN_(DUAL_X_CARRIAGE, diff.x = 0);
      #else
        constexpr xyz_pos_t diff{0};
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        dualx_tool_change(new_tool, no_move);
      #elif ENABLED(PARKING_EXTRUDER)                                   // Dual Parking extruder//双停车挤出机
        parking_extruder_tool_change(new_tool, no_move);
      #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)                          // Magnetic Parking extruder//磁力停车挤出机
        magnetic_parking_extruder_tool_change(new_tool);
      #elif ENABLED(SWITCHING_TOOLHEAD)                                 // Switching Toolhead//开关刀头
        switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)                        // Magnetic Switching Toolhead//磁开关刀头
        magnetic_switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)                 // Magnetic Switching ToolChanger//磁开关换刀器
        em_switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(SWITCHING_NOZZLE) && !SWITCHING_NOZZLE_TWO_SERVOS   // Switching Nozzle (single servo)//切换喷嘴（单伺服）
        // Raise by a configured distance to avoid workpiece, except with//升高一个配置的距离以避开工件，除非使用
        // SWITCHING_NOZZLE_TWO_SERVOS, as both nozzles will lift instead.//切换两个喷嘴，因为两个喷嘴都将提升。
        if (!no_move) {
          const float newz = current_position.z + _MAX(-diff.z, 0.0);

          // Check if Z has space to compensate at least z_offset, and if not, just abort now//检查Z是否有空间至少补偿Z_偏移，如果没有，现在就中止
          const float maxz = _MIN(TERN(HAS_SOFTWARE_ENDSTOPS, soft_endstop.max.z, Z_MAX_POS), Z_MAX_POS);
          if (newz > maxz) return;

          current_position.z = _MIN(newz + toolchange_settings.z_raise, maxz);
          fast_line_to_current(Z_AXIS);
        }
        move_nozzle_servo(new_tool);
      #endif

      IF_DISABLED(DUAL_X_CARRIAGE, active_extruder = new_tool); // Set the new active extruder//设置新的主动挤出机

      TERN_(TOOL_SENSOR, tool_sensor_disabled = false);

      (void)check_tool_sensor_stats(active_extruder, true);

      // The newly-selected extruder XYZ is actually at...//新选择的挤出机XYZ实际上位于。。。
      DEBUG_ECHOLNPAIR("Offset Tool XYZ by { ", diff.x, ", ", diff.y, ", ", diff.z, " }");
      current_position += diff;

      // Tell the planner the new "current position"//告诉计划员新的“当前位置”
      sync_plan_position();

      #if ENABLED(DELTA)
        //LOOP_LINEAR_AXES(i) update_software_endstops(i); // or modify the constrain function//循环(线性)轴(i)更新(i)软件(i)终止(i)//或修改约束函数
        const bool safe_to_move = current_position.z < delta_clip_start_height - 1;
      #else
        constexpr bool safe_to_move = true;
      #endif

      // Return to position and lower again//返回位置并再次降下
      const bool should_move = safe_to_move && !no_move && IsRunning();
      if (should_move) {

        #if EITHER(SINGLENOZZLE_STANDBY_TEMP, SINGLENOZZLE_STANDBY_FAN)
          thermalManager.singlenozzle_change(old_tool, new_tool);
        #endif

        #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
          if (should_swap && !too_cold) {

            float fr = toolchange_settings.unretract_speed;

            #if ENABLED(TOOLCHANGE_FS_INIT_BEFORE_SWAP)
              if (!toolchange_extruder_ready[new_tool]) {
                toolchange_extruder_ready[new_tool] = true;
                fr = toolchange_settings.prime_speed;       // Next move is a prime//下一步是一个主要步骤
                unscaled_e_move(0, MMM_TO_MMS(fr));         // Init planner with 0 length move//长度为0的初始计划器移动
              }
            #endif

            // Unretract (or Prime)//未收缩（或基本）
            unscaled_e_move(toolchange_settings.swap_length, MMM_TO_MMS(fr));

            // Extra Prime//额外素数
            unscaled_e_move(toolchange_settings.extra_prime, MMM_TO_MMS(toolchange_settings.prime_speed));

            // Cutting retraction//切削收缩
            #if TOOLCHANGE_FS_WIPE_RETRACT
              unscaled_e_move(-(TOOLCHANGE_FS_WIPE_RETRACT), MMM_TO_MMS(toolchange_settings.retract_speed));
            #endif

            // Cool down with fan//用风扇冷却
            #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
              thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = toolchange_settings.fan_speed;
              gcode.dwell(SEC_TO_MS(toolchange_settings.fan_time));
              thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = 0;
            #endif
          }
        #endif

        // Prevent a move outside physical bounds//阻止超出物理边界的移动
        #if ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
          // If the original position is within tool store area, go to X origin at once//如果原始位置在刀具存储区域内，请立即转到X原点
          if (destination.y < SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR) {
            current_position.x = X_MIN_POS;
            planner.buffer_line(current_position, planner.settings.max_feedrate_mm_s[X_AXIS], new_tool);
            planner.synchronize();
          }
        #else
          apply_motion_limits(destination);
        #endif

        // Should the nozzle move back to the old position?//喷嘴是否应移回原来的位置？
        if (can_move_away) {
          #if ENABLED(TOOLCHANGE_NO_RETURN)
            // Just move back down//往后退
            DEBUG_ECHOLNPGM("Move back Z only");

            if (TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park))
              do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);

          #else
            // Move back to the original (or adjusted) position//移回原始（或调整）位置
            DEBUG_POS("Move back", destination);

            #if ENABLED(TOOLCHANGE_PARK)
              if (toolchange_settings.enable_park) do_blocking_move_to_xy_z(destination, destination.z, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE));
            #else
              do_blocking_move_to_xy(destination, planner.settings.max_feedrate_mm_s[X_AXIS]);
              do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
            #endif

          #endif
        }

        else DEBUG_ECHOLNPGM("Move back skipped");

        #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
          if (should_swap && !too_cold) {
            // Cutting recover//切割回收
            unscaled_e_move(toolchange_settings.extra_resume + TOOLCHANGE_FS_WIPE_RETRACT, MMM_TO_MMS(toolchange_settings.unretract_speed));
            current_position.e = 0;
            sync_plan_position_e(); // New extruder primed and set to 0//新挤出机已涂底漆并设置为0

            // Restart Fan//重新启动风扇
            #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
              RESTORE(fan);
            #endif
          }
        #endif

        TERN_(DUAL_X_CARRIAGE, idex_set_parked(false));
      }

      #if ENABLED(SWITCHING_NOZZLE)
        // Move back down. (Including when the new tool is higher.)//向下移动。（包括新工具较高时。）
        if (!should_move)
          do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
      #endif

      TERN_(SWITCHING_NOZZLE_TWO_SERVOS, lower_nozzle(new_tool));

    } // (new_tool != old_tool)//（新工具！=旧工具）

    planner.synchronize();

    #if ENABLED(EXT_SOLENOID) && DISABLED(PARKING_EXTRUDER)
      disable_all_solenoids();
      enable_solenoid_on_active_extruder();
    #endif

    #if HAS_PRUSA_MMU1
      if (new_tool >= E_STEPPERS) return invalid_extruder_error(new_tool);
      select_multiplexed_stepper(new_tool);
    #endif

    #if DO_SWITCH_EXTRUDER
      planner.synchronize();
      move_extruder_servo(active_extruder);
    #endif

    TERN_(HAS_FANMUX, fanmux_switch(active_extruder));

    if (!no_move) {
      #ifdef EVENT_GCODE_TOOLCHANGE_T0
        if (new_tool == 0)
          gcode.process_subcommands_now_P(PSTR(EVENT_GCODE_TOOLCHANGE_T0));
      #endif

      #ifdef EVENT_GCODE_TOOLCHANGE_T1
        if (new_tool == 1)
          gcode.process_subcommands_now_P(PSTR(EVENT_GCODE_TOOLCHANGE_T1));
      #endif

      #ifdef EVENT_GCODE_AFTER_TOOLCHANGE
        if (TERN1(DUAL_X_CARRIAGE, dual_x_carriage_mode == DXC_AUTO_PARK_MODE))
          gcode.process_subcommands_now_P(PSTR(EVENT_GCODE_AFTER_TOOLCHANGE));
      #endif
    }

    SERIAL_ECHO_MSG(STR_ACTIVE_EXTRUDER, active_extruder);

  #endif // HAS_MULTI_EXTRUDER//HAS_多_挤出机
}

#if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)

  #define DEBUG_OUT ENABLED(DEBUG_TOOLCHANGE_MIGRATION_FEATURE)
  #include "../core/debug_out.h"

  bool extruder_migration() {

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      if (thermalManager.targetTooColdToExtrude(active_extruder)) {
        DEBUG_ECHOLNPGM("Migration Source Too Cold");
        return false;
      }
    #endif

    // No auto-migration or specified target?//没有自动迁移或指定的目标？
    if (!migration.target && active_extruder >= migration.last) {
      DEBUG_ECHO_MSG("No Migration Target");
      DEBUG_ECHO_MSG("Target: ", migration.target, " Last: ", migration.last, " Active: ", active_extruder);
      migration.automode = false;
      return false;
    }

    // Migrate to a target or the next extruder//迁移到目标或下一台挤出机

    uint8_t migration_extruder = active_extruder;

    if (migration.target) {
      DEBUG_ECHOLNPGM("Migration using fixed target");
      // Specified target ok?//指定的目标可以吗？
      const int16_t t = migration.target - 1;
      if (t != active_extruder) migration_extruder = t;
    }
    else if (migration.automode && migration_extruder < migration.last && migration_extruder < EXTRUDERS - 1)
      migration_extruder++;

    if (migration_extruder == active_extruder) {
      DEBUG_ECHOLNPGM("Migration source matches active");
      return false;
    }

    // Migration begins//迁移开始了
    DEBUG_ECHOLNPGM("Beginning migration");

    migration.in_progress = true; // Prevent runout script//防止输出脚本
    planner.synchronize();

    // Remember position before migration//迁移前记住位置
    const float resume_current_e = current_position.e;

    // Migrate the flow//迁移流
    planner.set_flow(migration_extruder, planner.flow_percentage[active_extruder]);

    // Migrate the retracted state//迁移收回状态
    #if ENABLED(FWRETRACT)
      fwretract.retracted[migration_extruder] = fwretract.retracted[active_extruder];
    #endif

    // Migrate the temperature to the new hotend//将温度迁移到新的热端
    #if HAS_MULTI_HOTEND
      thermalManager.setTargetHotend(thermalManager.degTargetHotend(active_extruder), migration_extruder);
      TERN_(AUTOTEMP, planner.autotemp_update());
      thermalManager.set_heating_message(0);
      thermalManager.wait_for_hotend(active_extruder);
    #endif

    // Migrate Linear Advance K factor to the new extruder//将线性推进K系数迁移到新挤出机
    TERN_(LIN_ADVANCE, planner.extruder_advance_K[active_extruder] = planner.extruder_advance_K[migration_extruder]);

    // Perform the tool change//执行刀具更换
    tool_change(migration_extruder);

    // Retract if previously retracted//如果先前已收回，则收回
    #if ENABLED(FWRETRACT)
      if (fwretract.retracted[active_extruder])
        unscaled_e_move(-fwretract.settings.retract_length, fwretract.settings.retract_feedrate_mm_s);
    #endif

    // If no available extruder//如果没有可用的挤出机
    if (EXTRUDERS < 2 || active_extruder >= EXTRUDERS - 2 || active_extruder == migration.last)
      migration.automode = false;

    migration.in_progress = false;

    current_position.e = resume_current_e;

    planner.synchronize();
    planner.set_e_position_mm(current_position.e); // New extruder primed and ready//新挤出机已涂底漆并准备就绪
    DEBUG_ECHOLNPGM("Migration Complete");
    return true;
  }

#endif // TOOLCHANGE_MIGRATION_FEATURE//工具更改\迁移\功能
