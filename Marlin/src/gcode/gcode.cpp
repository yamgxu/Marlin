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
 * gcode.cpp - Temporary container for all gcode handlers
 *             Most will migrate to classes, by feature.
 */

#include "gcode.h"
GcodeSuite gcode;

#if ENABLED(WIFI_CUSTOM_COMMAND)
  extern bool wifi_custom_command(char * const command_ptr);
#endif

#include "parser.h"
#include "queue.h"
#include "../module/motion.h"

#if ENABLED(PRINTCOUNTER)
  #include "../module/printcounter.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../feature/host_actions.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../sd/cardreader.h"
  #include "../feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "../feature/cancel_object.h"
#endif

#if ENABLED(LASER_MOVE_POWER)
  #include "../feature/spindle_laser.h"
#endif

#if ENABLED(FLOWMETER_SAFETY)
  #include "../feature/cooler.h"
#endif

#if ENABLED(PASSWORD_FEATURE)
  #include "../feature/password/password.h"
#endif

#include "../MarlinCore.h" // for idle, kill//无所事事，杀人

// Inactivity shutdown//非活动关机
millis_t GcodeSuite::previous_move_ms = 0,
         GcodeSuite::max_inactive_time = 0,
         GcodeSuite::stepper_inactive_time = SEC_TO_MS(DEFAULT_STEPPER_DEACTIVE_TIME);

// Relative motion mode for each logical axis//每个逻辑轴的相对运动模式
static constexpr xyze_bool_t ar_init = AXIS_RELATIVE_MODES;
uint8_t GcodeSuite::axis_relative = 0 LOGICAL_AXIS_GANG(
  | (ar_init.e << REL_E),
  | (ar_init.x << REL_X),
  | (ar_init.y << REL_Y),
  | (ar_init.z << REL_Z),
  | (ar_init.i << REL_I),
  | (ar_init.j << REL_J),
  | (ar_init.k << REL_K)
);

#if EITHER(HAS_AUTO_REPORTING, HOST_KEEPALIVE_FEATURE)
  bool GcodeSuite::autoreport_paused; // = false//=错误
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  GcodeSuite::MarlinBusyState GcodeSuite::busy_state = NOT_BUSY;
  uint8_t GcodeSuite::host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  GcodeSuite::WorkspacePlane GcodeSuite::workspace_plane = PLANE_XY;
#endif

#if ENABLED(CNC_COORDINATE_SYSTEMS)
  int8_t GcodeSuite::active_coordinate_system = -1; // machine space//机器空间
  xyz_pos_t GcodeSuite::coordinate_system[MAX_COORDINATE_SYSTEMS];
#endif

/**
 * Get the target extruder from the T parameter or the active_extruder
 * Return -1 if the T parameter is out of range
 */
int8_t GcodeSuite::get_target_extruder_from_command() {
  if (parser.seenval('T')) {
    const int8_t e = parser.value_byte();
    if (e < EXTRUDERS) return e;
    SERIAL_ECHO_START();
    SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
    SERIAL_ECHOLNPAIR(" " STR_INVALID_EXTRUDER " ", e);
    return -1;
  }
  return active_extruder;
}

/**
 * Get the target e stepper from the T parameter
 * Return -1 if the T parameter is out of range or unspecified
 */
int8_t GcodeSuite::get_target_e_stepper_from_command() {
  const int8_t e = parser.intval('T', -1);
  if (WITHIN(e, 0, E_STEPPERS - 1)) return e;

  SERIAL_ECHO_START();
  SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
  if (e == -1)
    SERIAL_ECHOLNPGM(" " STR_E_STEPPER_NOT_SPECIFIED);
  else
    SERIAL_ECHOLNPAIR(" " STR_INVALID_E_STEPPER " ", e);
  return -1;
}

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void GcodeSuite::get_destination_from_command() {
  xyze_bool_t seen{false};

  #if ENABLED(CANCEL_OBJECTS)
    const bool &skip_move = cancelable.skipping;
  #else
    constexpr bool skip_move = false;
  #endif

  // Get new XYZ position, whether absolute or relative//获取新的XYZ位置，无论是绝对位置还是相对位置
  LOOP_LINEAR_AXES(i) {
    if ( (seen[i] = parser.seenval(AXIS_CHAR(i))) ) {
      const float v = parser.value_axis_units((AxisEnum)i);
      if (skip_move)
        destination[i] = current_position[i];
      else
        destination[i] = axis_is_relative(AxisEnum(i)) ? current_position[i] + v : LOGICAL_TO_NATIVE(v, i);
    }
    else
      destination[i] = current_position[i];
  }

  #if HAS_EXTRUDERS
    // Get new E position, whether absolute or relative//获取新的E位置，无论是绝对位置还是相对位置
    if ( (seen.e = parser.seenval('E')) ) {
      const float v = parser.value_axis_units(E_AXIS);
      destination.e = axis_is_relative(E_AXIS) ? current_position.e + v : v;
    }
    else
      destination.e = current_position.e;
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY) && !PIN_EXISTS(POWER_LOSS)
    // Only update power loss recovery on moves with E//仅在使用E的移动中更新电源损耗恢复
    if (recovery.enabled && IS_SD_PRINTING() && seen.e && (seen.x || seen.y))
      recovery.save();
  #endif

  if (parser.linearval('F') > 0)
    feedrate_mm_s = parser.value_feedrate();

  #if ENABLED(PRINTCOUNTER)
    if (!DEBUGGING(DRYRUN) && !skip_move)
      print_job_timer.incFilamentUsed(destination.e - current_position.e);
  #endif

  // Get ABCDHI mixing factors//获得ABCDHI混合因子
  #if BOTH(MIXING_EXTRUDER, DIRECT_MIXING_IN_G1)
    M165();
  #endif

  #if ENABLED(LASER_MOVE_POWER)
    // Set the laser power in the planner to configure this move//在planner中设置激光功率以配置此移动
    if (parser.seen('S')) {
      const float spwr = parser.value_float();
      cutter.inline_power(TERN(SPINDLE_LASER_PWM, cutter.power_to_range(cutter_power_t(round(spwr))), spwr > 0 ? 255 : 0));
    }
    else if (ENABLED(LASER_MOVE_G0_OFF) && parser.codenum == 0) // G0//G0
      cutter.set_inline_enabled(false);
  #endif
}

/**
 * Dwell waits immediately. It does not synchronize. Use M400 instead of G4
 */
void GcodeSuite::dwell(millis_t time) {
  time += millis();
  while (PENDING(millis(), time)) idle();
}

/**
 * When G29_RETRY_AND_RECOVER is enabled, call G29() in
 * a loop with recovery and retry handling.
 */
#if ENABLED(G29_RETRY_AND_RECOVER)

  void GcodeSuite::event_probe_recover() {
    TERN_(HOST_PROMPT_SUPPORT, host_prompt_do(PROMPT_INFO, PSTR("G29 Retrying"), DISMISS_STR));
    #ifdef ACTION_ON_G29_RECOVER
      host_action(PSTR(ACTION_ON_G29_RECOVER));
    #endif
    #ifdef G29_RECOVER_COMMANDS
      process_subcommands_now_P(PSTR(G29_RECOVER_COMMANDS));
    #endif
  }

  #if ENABLED(G29_HALT_ON_FAILURE)
    #include "../lcd/marlinui.h"
  #endif

  void GcodeSuite::event_probe_failure() {
    #ifdef ACTION_ON_G29_FAILURE
      host_action(PSTR(ACTION_ON_G29_FAILURE));
    #endif
    #ifdef G29_FAILURE_COMMANDS
      process_subcommands_now_P(PSTR(G29_FAILURE_COMMANDS));
    #endif
    #if ENABLED(G29_HALT_ON_FAILURE)
      #ifdef ACTION_ON_CANCEL
        host_action_cancel();
      #endif
      kill(GET_TEXT(MSG_LCD_PROBING_FAILED));
    #endif
  }

  #ifndef G29_MAX_RETRIES
    #define G29_MAX_RETRIES 0
  #endif

  void GcodeSuite::G29_with_retry() {
    uint8_t retries = G29_MAX_RETRIES;
    while (G29()) { // G29 should return true for failed probes ONLY//G29应仅针对故障探头返回true
      if (retries) {
        event_probe_recover();
        --retries;
      }
      else {
        event_probe_failure();
        return;
      }
    }

    TERN_(HOST_PROMPT_SUPPORT, host_action_prompt_end());

    #ifdef G29_SUCCESS_COMMANDS
      process_subcommands_now_P(PSTR(G29_SUCCESS_COMMANDS));
    #endif
  }

#endif // G29_RETRY_AND_RECOVER//G29\u重试\u和\u恢复

/**
 * Process the parsed command and dispatch it to its handler
 */
void GcodeSuite::process_parsed_command(const bool no_ok/*=false*/) {
  KEEPALIVE_STATE(IN_HANDLER);
    SERIAL_ECHO_MSG("process_parsed_command");

 /**
  * Block all Gcodes except M511 Unlock Printer, if printer is locked
  * Will still block Gcodes if M511 is disabled, in which case the printer should be unlocked via LCD Menu
  */
  #if ENABLED(PASSWORD_FEATURE)
    if (password.is_locked && !parser.is_command('M', 511)) {
      SERIAL_ECHO_MSG(STR_PRINTER_LOCKED);
      if (!no_ok) queue.ok_to_send();
      return;
    }
  #endif

  #if ENABLED(FLOWMETER_SAFETY)
    if (cooler.fault) {
      SERIAL_ECHO_MSG(STR_FLOWMETER_FAULT);
      return;
    }
  #endif

  // Handle a known command or reply "unknown command"//处理已知命令或回复“未知命令”

  switch (parser.command_letter) {

    case 'G': switch (parser.codenum) {

      case 0: case 1:                                             // G0: Fast Move, G1: Linear Move//G0：快速移动，G1：线性移动
        G0_G1(TERN_(HAS_FAST_MOVES, parser.codenum == 0)); break;

      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: case 3: G2_G3(parser.codenum == 2); break;        // G2: CW ARC, G3: CCW ARC//G2:CW电弧，G3:CCW电弧
      #endif

      case 4: G4(); break;                                        // G4: Dwell//G4：驻留

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 5: G5(); break;                                      // G5: Cubic B_spline//G5：三次B_样条
      #endif

      #if ENABLED(DIRECT_STEPPING)
        case 6: G6(); break;                                      // G6: Direct Stepper Move//G6：直接步进移动
      #endif

      #if ENABLED(FWRETRACT)
        case 10: G10(); break;                                    // G10: Retract / Swap Retract//G10：收回/交换收回
        case 11: G11(); break;                                    // G11: Recover / Swap Recover//G11：恢复/交换恢复
      #endif

      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12: G12(); break;                                    // G12: Nozzle Clean//G12：喷嘴清洁
      #endif

      #if ENABLED(CNC_WORKSPACE_PLANES)
        case 17: G17(); break;                                    // G17: Select Plane XY//G17：选择平面XY
        case 18: G18(); break;                                    // G18: Select Plane ZX//G18：选择平面ZX
        case 19: G19(); break;                                    // G19: Select Plane YZ//G19：选择平面YZ
      #endif

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: G20(); break;                                    // G20: Inch Mode//G20：英寸模式
        case 21: G21(); break;                                    // G21: MM Mode//G21:MM模式
      #else
        case 21: NOOP; break;                                     // No error on unknown G21//未知G21上没有错误
      #endif

      #if ENABLED(G26_MESH_VALIDATION)
        case 26: G26(); break;                                    // G26: Mesh Validation Pattern generation//G26：网格验证模式生成
      #endif

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: G27(); break;                                    // G27: Nozzle Park//G27：喷嘴停车场
      #endif

      case 28: G28(); break;                                      // G28: Home one or more axes//G28：一个或多个轴的原点

      #if HAS_LEVELING
        case 29:                                                  // G29: Bed leveling calibration//G29：河床平整校准
          TERN(G29_RETRY_AND_RECOVER, G29_with_retry, G29)();
          break;
      #endif

      #if HAS_BED_PROBE
        case 30: G30(); break;                                    // G30: Single Z probe//G30：单Z探头
        #if ENABLED(Z_PROBE_SLED)
          case 31: G31(); break;                                  // G31: dock the sled//G31：停靠雪橇
          case 32: G32(); break;                                  // G32: undock the sled//G32：卸下雪橇
        #endif
      #endif

      #if ENABLED(DELTA_AUTO_CALIBRATION)
        case 33: G33(); break;                                    // G33: Delta Auto-Calibration//G33：增量自动校准
      #endif

      #if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN, MECHANICAL_GANTRY_CALIBRATION)
        case 34: G34(); break;                                    // G34: Z Stepper automatic alignment using probe//G34:Z步进机使用探针自动对准
      #endif

      #if ENABLED(ASSISTED_TRAMMING)
        case 35: G35(); break;                                    // G35: Read four bed corners to help adjust bed screws//G35：阅读床的四个角以帮助调整床螺钉
      #endif

      #if ENABLED(G38_PROBE_TARGET)
        case 38:                                                  // G38.2, G38.3: Probe towards target//G38.2、G38.3：探头朝向目标
          if (WITHIN(parser.subcode, 2, TERN(G38_PROBE_AWAY, 5, 3)))
            G38(parser.subcode);                                  // G38.4, G38.5: Probe away from target//G38.4、G38.5：探头远离目标
          break;
      #endif

      #if HAS_MESH
        case 42: G42(); break;                                    // G42: Coordinated move to a mesh point//G42：协调移动到网格点
      #endif

      #if ENABLED(CNC_COORDINATE_SYSTEMS)
        case 53: G53(); break;                                    // G53: (prefix) Apply native workspace//G53：（前缀）应用本机工作区
        case 54: G54(); break;                                    // G54: Switch to Workspace 1//G54：切换到工作区1
        case 55: G55(); break;                                    // G55: Switch to Workspace 2//G55：切换到工作区2
        case 56: G56(); break;                                    // G56: Switch to Workspace 3//G56：切换到工作区3
        case 57: G57(); break;                                    // G57: Switch to Workspace 4//G57：切换到工作区4
        case 58: G58(); break;                                    // G58: Switch to Workspace 5//G58：切换到工作区5
        case 59: G59(); break;                                    // G59.0 - G59.3: Switch to Workspace 6-9//G59.0-G59.3：切换到工作空间6-9
      #endif

      #if SAVED_POSITIONS
        case 60: G60(); break;                                    // G60:  save current position//G60：保存当前位置
        case 61: G61(); break;                                    // G61:  Apply/restore saved coordinates.//G61：应用/恢复保存的坐标。
      #endif

      #if ENABLED(PROBE_TEMP_COMPENSATION)
        case 76: G76(); break;                                    // G76: Calibrate first layer compensation values//G76：校准第一层补偿值
      #endif

      #if ENABLED(GCODE_MOTION_MODES)
        case 80: G80(); break;                                    // G80: Reset the current motion mode//G80：重置当前运动模式
      #endif

      case 90: set_relative_mode(false); break;                   // G90: Absolute Mode//G90：绝对模式
      case 91: set_relative_mode(true);  break;                   // G91: Relative Mode//G91：相对模式

      case 92: G92(); break;                                      // G92: Set current axis position(s)//G92：设置当前轴位置

      #if ENABLED(CALIBRATION_GCODE)
        case 425: G425(); break;                                  // G425: Perform calibration with calibration cube//G425：使用校准立方体执行校准
      #endif

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: parser.debug(); break;                          // G800: GCode Parser Test for G//G800:G的GCode解析器测试
      #endif

      default: parser.unknown_command_warning(); break;
    }
    break;

    case 'M': switch (parser.codenum) {

      #if HAS_RESUME_CONTINUE
        case 0:                                                   // M0: Unconditional stop - Wait for user button press on LCD//M0：无条件停止-等待LCD上的用户按钮按下
        case 1: M0_M1(); break;                                   // M1: Conditional stop - Wait for user button press on LCD//M1：条件停止-等待LCD上的用户按钮按下
      #endif

      #if HAS_CUTTER
        case 3: M3_M4(false); break;                              // M3: Turn ON Laser | Spindle (clockwise), set Power | Speed//M3：打开激光器|主轴（顺时针），设置功率|速度
        case 4: M3_M4(true ); break;                              // M4: Turn ON Laser | Spindle (counter-clockwise), set Power | Speed//M4：打开激光器|主轴（逆时针），设置功率|速度
        case 5: M5(); break;                                      // M5: Turn OFF Laser | Spindle//M5：关闭激光主轴
      #endif

      #if ENABLED(COOLANT_MIST)
        case 7: M7(); break;                                      // M7: Coolant Mist ON//M7：冷却液雾打开
      #endif

      #if EITHER(AIR_ASSIST, COOLANT_FLOOD)
        case 8: M8(); break;                                      // M8: Air Assist / Coolant Flood ON//M8：空气辅助/冷却液溢流开启
      #endif

      #if EITHER(AIR_ASSIST, COOLANT_CONTROL)
        case 9: M9(); break;                                      // M9: Air Assist / Coolant OFF//M9：空气辅助/冷却液关闭
      #endif

      #if ENABLED(AIR_EVACUATION)
        case 10: M10(); break;                                    // M10: Vacuum or Blower motor ON//M10：真空或鼓风机电机打开
        case 11: M11(); break;                                    // M11: Vacuum or Blower motor OFF//M11：真空或鼓风机电机关闭
      #endif

      #if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
        case 12: M12(); break;                                    // M12: Synchronize and optionally force a CLC set//M12：同步和强制CLC集（可选）
      #endif

      #if ENABLED(EXPECTED_PRINTER_CHECK)
        case 16: M16(); break;                                    // M16: Expected printer check//M16：预期的打印机检查
      #endif

      case 17: M17(); break;                                      // M17: Enable all stepper motors//M17：启用所有步进电机

      #if ENABLED(SDSUPPORT)
        case 20: M20(); break;                                    // M20: List SD card//M20：列出SD卡
        case 21: M21(); break;                                    // M21: Init SD card//M21：初始化SD卡
        case 22: M22(); break;                                    // M22: Release SD card//M22：释放SD卡
        case 23: M23(); break;                                    // M23: Select file//M23：选择文件
        case 24: M24(); break;                                    // M24: Start SD print//M24：开始SD打印
        case 25: M25(); break;                                    // M25: Pause SD print//M25：暂停SD打印
        case 26: M26(); break;                                    // M26: Set SD index//M26：设置SD索引
        case 27: M27(); break;                                    // M27: Get SD status//M27：获取SD状态
        case 28: M28(); break;                                    // M28: Start SD write//M28：启动SD写入
        case 29: M29(); break;                                    // M29: Stop SD write//M29：停止SD写入
        case 30: M30(); break;                                    // M30 <filename> Delete File//M30<filename>删除文件

        #if HAS_MEDIA_SUBCALLS
          case 32: M32(); break;                                  // M32: Select file and start SD print//M32：选择文件并开始SD打印
        #endif

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: M33(); break;                                  // M33: Get the long full path to a file or folder//M33：获取文件或文件夹的完整长路径
        #endif

        #if BOTH(SDCARD_SORT_ALPHA, SDSORT_GCODE)
          case 34: M34(); break;                                  // M34: Set SD card sorting options//M34：设置SD卡排序选项
        #endif

        case 928: M928(); break;                                  // M928: Start SD write//M928：启动SD写入
      #endif // SDSUPPORT//SDSUPPORT

      case 31: M31(); break;                                      // M31: Report time since the start of SD print or last M109//M31：自SD打印开始或上次M109以来的报告时间

      #if ENABLED(DIRECT_PIN_CONTROL)
        case 42: M42(); break;                                    // M42: Change pin state//M42：更改引脚状态
      #endif

      #if ENABLED(PINS_DEBUGGING)
        case 43: M43(); break;                                    // M43: Read pin state//M43：读取引脚状态
      #endif

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: M48(); break;                                    // M48: Z probe repeatability test//M48:Z探头重复性试验
      #endif

      #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
        case 73: M73(); break;                                    // M73: Set progress percentage (for display on LCD)//M73：设置进度百分比（用于LCD上的显示）
      #endif

      case 75: M75(); break;                                      // M75: Start print timer//M75：启动打印计时器
      case 76: M76(); break;                                      // M76: Pause print timer//M76：暂停打印计时器
      case 77: M77(); break;                                      // M77: Stop print timer//M77：停止打印计时器

      #if ENABLED(PRINTCOUNTER)
        case 78: M78(); break;                                    // M78: Show print statistics//M78：显示打印统计信息
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: M100(); break;                                  // M100: Free Memory Report//M100：可用内存报告
      #endif

      #if HAS_EXTRUDERS
        case 104: M104(); break;                                  // M104: Set hot end temperature//M104：设置热端温度
        case 109: M109(); break;                                  // M109: Wait for hotend temperature to reach target//M109：等待热端温度达到目标
      #endif

      case 105: M105(); return;                                   // M105: Report Temperatures (and say "ok")//M105：报告温度（并说“ok”）

      #if HAS_FAN
        case 106: M106(); break;                                  // M106: Fan On//M106：风扇打开
        case 107: M107(); break;                                  // M107: Fan Off//M107：扇形关闭
      #endif

      case 110: M110(); break;                                    // M110: Set Current Line Number//M110：设置当前行号
      case 111: M111(); break;                                    // M111: Set debug level//M111：设置调试级别

      #if DISABLED(EMERGENCY_PARSER)
        case 108: M108(); break;                                  // M108: Cancel Waiting//M108：取消等待
        case 112: M112(); break;                                  // M112: Full Shutdown//M112：完全关闭
        case 410: M410(); break;                                  // M410: Quickstop - Abort all the planned moves.//M410:快速停止-中止所有计划的移动。
        TERN_(HOST_PROMPT_SUPPORT, case 876:)                     // M876: Handle Host prompt responses//M876:处理主机提示响应
      #else
        case 108: case 112: case 410:
        TERN_(HOST_PROMPT_SUPPORT, case 876:)
        break;
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: M113(); break;                                  // M113: Set Host Keepalive interval//M113：设置主机保持间隔
      #endif

      #if HAS_HEATED_BED
        case 140: M140(); break;                                  // M140: Set bed temperature//M140：设定床温
        case 190: M190(); break;                                  // M190: Wait for bed temperature to reach target//M190：等待床温达到目标值
      #endif

      #if HAS_HEATED_CHAMBER
        case 141: M141(); break;                                  // M141: Set chamber temperature//M141：设置腔室温度
        case 191: M191(); break;                                  // M191: Wait for chamber temperature to reach target//M191：等待腔室温度达到目标
      #endif

      #if HAS_COOLER
        case 143: M143(); break;                                  // M143: Set cooler temperature//M143：设置冷却器温度
        case 193: M193(); break;                                  // M193: Wait for cooler temperature to reach target//M193：等待冷却器温度达到目标
      #endif

      #if ENABLED(AUTO_REPORT_POSITION)
        case 154: M154(); break;                                  // M154: Set position auto-report interval//M154：设置位置自动报告间隔
      #endif

      #if BOTH(AUTO_REPORT_TEMPERATURES, HAS_TEMP_SENSOR)
        case 155: M155(); break;                                  // M155: Set temperature auto-report interval//M155：设置温度自动报告间隔
      #endif

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: M125(); break;                                  // M125: Store current position and move to filament change position//M125：存储当前位置并移动到灯丝更换位置
      #endif

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN//加热器1针的PWM
        #if HAS_HEATER_1
          case 126: M126(); break;                                // M126: valve open//M126：阀打开
          case 127: M127(); break;                                // M127: valve closed//M127：阀关闭
        #endif

        // PWM for HEATER_2_PIN//加热器2针的PWM
        #if HAS_HEATER_2
          case 128: M128(); break;                                // M128: valve open//M128：阀打开
          case 129: M129(); break;                                // M129: valve closed//M129：阀关闭
        #endif
      #endif // BARICUDA//巴里库达

      #if ENABLED(PSU_CONTROL)
        case 80: M80(); break;                                    // M80: Turn on Power Supply//M80：打开电源
      #endif
      case 81: M81(); break;                                      // M81: Turn off Power, including Power Supply, if possible//M81：如果可能，关闭电源，包括电源

      #if HAS_EXTRUDERS
        case 82: M82(); break;                                    // M82: Set E axis normal mode (same as other axes)//M82：设置E轴正常模式（与其他轴相同）
        case 83: M83(); break;                                    // M83: Set E axis relative mode//M83：设置E轴相对模式
      #endif
      case 18: case 84: M18_M84(); break;                         // M18/M84: Disable Steppers / Set Timeout//M18/M84：禁用步进器/设置超时
      case 85: M85(); break;                                      // M85: Set inactivity stepper shutdown timeout//M85：设置不活动步进器关闭超时
      case 92: M92(); break;                                      // M92: Set the steps-per-unit for one or more axes//M92：设置一个或多个轴的每单位步数
      case 114: M114(); break;                                    // M114: Report current position//M114：报告当前位置
      case 115: M115(); break;                                    // M115: Report capabilities//M115：报告功能

      case 117: TERN_(HAS_STATUS_MESSAGE, M117()); break;         // M117: Set LCD message text, if possible//M117：如果可能，设置LCD消息文本

      case 118: M118(); break;                                    // M118: Display a message in the host console//M118：在主机控制台中显示消息
      case 119: M119(); break;                                    // M119: Report endstop states//M119：报告结束停止状态
      case 120: M120(); break;                                    // M120: Enable endstops//M120：启用端点止动块
      case 121: M121(); break;                                    // M121: Disable endstops//M121：禁用止动块

      #if PREHEAT_COUNT
        case 145: M145(); break;                                  // M145: Set material heatup parameters//M145：设置物料加热参数
      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: M149(); break;                                  // M149: Set temperature units//M149：设定温度单位
      #endif

      #if HAS_COLOR_LEDS
        case 150: M150(); break;                                  // M150: Set Status LED Color//M150：设置状态LED颜色
      #endif

      #if ENABLED(MIXING_EXTRUDER)
        case 163: M163(); break;                                  // M163: Set a component weight for mixing extruder//M163：设置混合挤出机的部件重量
        case 164: M164(); break;                                  // M164: Save current mix as a virtual extruder//M164：将当前混合保存为虚拟挤出机
        #if ENABLED(DIRECT_MIXING_IN_G1)
          case 165: M165(); break;                                // M165: Set multiple mix weights//M165：设置多个混合权重
        #endif
        #if ENABLED(GRADIENT_MIX)
          case 166: M166(); break;                                // M166: Set Gradient Mix//M166：设置渐变混合
        #endif
      #endif

      #if DISABLED(NO_VOLUMETRICS)
        case 200: M200(); break;                                  // M200: Set filament diameter, E to cubic units//M200：将灯丝直径E设置为立方单位
      #endif

      case 201: M201(); break;                                    // M201: Set max acceleration for print moves (units/s^2)//M201：设置打印移动的最大加速度（单位/s^2）

      #if 0
        case 202: M202(); break;                                  // M202: Not used for Sprinter/grbl gen6//M202：不用于Sprinter/grbl gen6
      #endif

      case 203: M203(); break;                                    // M203: Set max feedrate (units/sec)//M203：设置最大进给速度（单位/秒）
      case 204: M204(); break;                                    // M204: Set acceleration//M204：设置加速度
      case 205: M205(); break;                                    // M205: Set advanced settings//M205：设置高级设置

      #if HAS_M206_COMMAND
        case 206: M206(); break;                                  // M206: Set home offsets//M206：设置原点偏移
      #endif

      #if ENABLED(FWRETRACT)
        case 207: M207(); break;                                  // M207: Set Retract Length, Feedrate, and Z lift//M207：设置回缩长度、进给速度和Z提升
        case 208: M208(); break;                                  // M208: Set Recover (unretract) Additional Length and Feedrate//M208：设置恢复（未恢复）附加长度和进给速度
        #if ENABLED(FWRETRACT_AUTORETRACT)
          case 209:
            if (MIN_AUTORETRACT <= MAX_AUTORETRACT) M209();       // M209: Turn Automatic Retract Detection on/off//M209：打开/关闭自动缩回检测
            break;
        #endif
      #endif

      #if HAS_SOFTWARE_ENDSTOPS
        case 211: M211(); break;                                  // M211: Enable, Disable, and/or Report software endstops//M211：启用、禁用和/或报告软件终止
      #endif

      #if HAS_MULTI_EXTRUDER
        case 217: M217(); break;                                  // M217: Set filament swap parameters//M217：设置灯丝交换参数
      #endif

      #if HAS_HOTEND_OFFSET
        case 218: M218(); break;                                  // M218: Set a tool offset//M218：设置刀具偏移
      #endif

      case 220: M220(); break;                                    // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)//M220：设置进给速度百分比：S<percent>（“LCD上的FR”）

      #if HAS_EXTRUDERS
        case 221: M221(); break;                                  // M221: Set Flow Percentage//M221：设置流量百分比
      #endif

      #if ENABLED(DIRECT_PIN_CONTROL)
        case 226: M226(); break;                                  // M226: Wait until a pin reaches a state//M226：等待引脚达到某个状态
      #endif

      #if HAS_SERVOS
        case 280: M280(); break;                                  // M280: Set servo position absolute//M280：设置绝对伺服位置
        #if ENABLED(EDITABLE_SERVO_ANGLES)
          case 281: M281(); break;                                // M281: Set servo angles//M281：设置伺服角度
        #endif
      #endif

      #if ENABLED(BABYSTEPPING)
        case 290: M290(); break;                                  // M290: Babystepping//M290:Babystepping
      #endif

      #if HAS_BUZZER
        case 300: M300(); break;                                  // M300: Play beep tone//M300：播放蜂鸣音
      #endif

      #if ENABLED(PIDTEMP)
        case 301: M301(); break;                                  // M301: Set hotend PID parameters//M301：设置热端PID参数
      #endif

      #if ENABLED(PIDTEMPBED)
        case 304: M304(); break;                                  // M304: Set bed PID parameters//M304：设置床身PID参数
      #endif

      #if ENABLED(PIDTEMPCHAMBER)
        case 309: M309(); break;                                  // M309: Set chamber PID parameters//M309：设置燃烧室PID参数
      #endif

      #if ENABLED(PHOTO_GCODE)
        case 240: M240(); break;                                  // M240: Trigger a camera//M240：触发相机
      #endif

      #if HAS_LCD_CONTRAST
        case 250: M250(); break;                                  // M250: Set LCD contrast//M250：设置LCD对比度
      #endif

      #if ENABLED(EXPERIMENTAL_I2CBUS)
        case 260: M260(); break;                                  // M260: Send data to an i2c slave//M260：向i2c从机发送数据
        case 261: M261(); break;                                  // M261: Request data from an i2c slave//M261：从i2c从机请求数据
      #endif

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: M302(); break;                                  // M302: Allow cold extrudes (set the minimum extrude temperature)//M302：允许冷挤压（设置最小挤压温度）
      #endif

      #if HAS_PID_HEATING
        case 303: M303(); break;                                  // M303: PID autotune//M303:PID自动调谐
      #endif

      #if HAS_USER_THERMISTORS
        case 305: M305(); break;                                  // M305: Set user thermistor parameters//M305：设置用户热敏电阻参数
      #endif

      #if ENABLED(REPETIER_GCODE_M360)
        case 360: M360(); break;                                  // M360: Firmware settings//M360：固件设置
      #endif

      #if ENABLED(MORGAN_SCARA)
        case 360: if (M360()) return; break;                      // M360: SCARA Theta pos1//M360:SCARAθ位置1
        case 361: if (M361()) return; break;                      // M361: SCARA Theta pos2//M361:SCARAθ位置2
        case 362: if (M362()) return; break;                      // M362: SCARA Psi pos1//M362:SCARA Psi位置1
        case 363: if (M363()) return; break;                      // M363: SCARA Psi pos2//M363:SCARA Psi位置2
        case 364: if (M364()) return; break;                      // M364: SCARA Psi pos3 (90 deg to Theta)//M364:SCARA Psi位置3（90度至θ）
      #endif

      #if EITHER(EXT_SOLENOID, MANUAL_SOLENOID_CONTROL)
        case 380: M380(); break;                                  // M380: Activate solenoid on active (or specified) extruder//M380：激活激活（或指定）挤出机上的电磁阀
        case 381: M381(); break;                                  // M381: Disable all solenoids or, if MANUAL_SOLENOID_CONTROL, active (or specified) solenoid//M381：禁用所有电磁阀，或者如果手动电磁阀控制，则禁用激活（或指定）电磁阀
      #endif

      case 400: M400(); break;                                    // M400: Finish all moves//M400：完成所有动作

      #if HAS_BED_PROBE
        case 401: M401(); break;                                  // M401: Deploy probe//M401：部署探测器
        case 402: M402(); break;                                  // M402: Stow probe//M402：收起探头
      #endif

      #if HAS_PRUSA_MMU2
        case 403: M403(); break;
      #endif

      #if ENABLED(FILAMENT_WIDTH_SENSOR)
        case 404: M404(); break;                                  // M404: Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width//M404：输入标称灯丝宽度（3mm，1.75mm）N<3.0>或显示标称灯丝宽度
        case 405: M405(); break;                                  // M405: Turn on filament sensor for control//M405：打开控制灯丝传感器
        case 406: M406(); break;                                  // M406: Turn off filament sensor for control//M406：关闭用于控制的灯丝传感器
        case 407: M407(); break;                                  // M407: Display measured filament diameter//M407：显示测量的灯丝直径
      #endif

      #if HAS_FILAMENT_SENSOR
        case 412: M412(); break;                                  // M412: Enable/Disable filament runout detection//M412：启用/禁用灯丝偏移检测
      #endif

      #if HAS_MULTI_LANGUAGE
        case 414: M414(); break;                                  // M414: Select multi language menu//M414：选择多语言菜单
      #endif

      #if HAS_LEVELING
        case 420: M420(); break;                                  // M420: Enable/Disable Bed Leveling//M420：启用/禁用床层调平
      #endif

      #if HAS_MESH
        case 421: M421(); break;                                  // M421: Set a Mesh Bed Leveling Z coordinate//M421：设置网格床水平Z坐标
      #endif

      #if ENABLED(BACKLASH_GCODE)
        case 425: M425(); break;                                  // M425: Tune backlash compensation//M425：调整齿隙补偿
      #endif

      #if HAS_M206_COMMAND
        case 428: M428(); break;                                  // M428: Apply current_position to home_offset//M428：将当前位置应用于主位置偏移
      #endif

      #if HAS_POWER_MONITOR
        case 430: M430(); break;                                  // M430: Read the system current (A), voltage (V), and power (W)//M430：读取系统电流（A）、电压（V）和功率（W）
      #endif

      #if ENABLED(CANCEL_OBJECTS)
        case 486: M486(); break;                                  // M486: Identify and cancel objects//M486：识别和取消对象
      #endif

      case 500: M500(); break;                                    // M500: Store settings in EEPROM//M500：将设置存储在EEPROM中
      case 501: M501(); break;                                    // M501: Read settings from EEPROM//M501：从EEPROM读取设置
      case 502: M502(); break;                                    // M502: Revert to default settings//M502：恢复到默认设置
      #if DISABLED(DISABLE_M503)
        case 503: M503(); break;                                  // M503: print settings currently in memory//M503：当前内存中的打印设置
      #endif
      #if ENABLED(EEPROM_SETTINGS)
        case 504: M504(); break;                                  // M504: Validate EEPROM contents//M504：验证EEPROM内容
      #endif

      #if ENABLED(PASSWORD_FEATURE)
        case 510: M510(); break;                                  // M510: Lock Printer//M510：锁定打印机
        #if ENABLED(PASSWORD_UNLOCK_GCODE)
          case 511: M511(); break;                                // M511: Unlock Printer//M511：解锁打印机
        #endif
        #if ENABLED(PASSWORD_CHANGE_GCODE)
          case 512: M512(); break;                                // M512: Set/Change/Remove Password//M512：设置/更改/删除密码
        #endif
      #endif

      #if ENABLED(SDSUPPORT)
        case 524: M524(); break;                                  // M524: Abort the current SD print job//M524:中止当前SD打印作业
      #endif

      #if ENABLED(SD_ABORT_ON_ENDSTOP_HIT)
        case 540: M540(); break;                                  // M540: Set abort on endstop hit for SD printing//M540:在SD打印的结束停止命中时设置中止
      #endif

      #if HAS_ETHERNET
        case 552: M552(); break;                                  // M552: Set IP address//M552：设置IP地址
        case 553: M553(); break;                                  // M553: Set gateway//M553：设置网关
        case 554: M554(); break;                                  // M554: Set netmask//M554：设置网络掩码
      #endif

      #if ENABLED(BAUD_RATE_GCODE)
        case 575: M575(); break;                                  // M575: Set serial baudrate//M575：设置串行波特率
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: M600(); break;                                  // M600: Pause for Filament Change//M600：暂停更换灯丝
        case 603: M603(); break;                                  // M603: Configure Filament Change//M603：配置灯丝更换
      #endif

      #if HAS_DUPLICATION_MODE
        case 605: M605(); break;                                  // M605: Set Dual X Carriage movement mode//M605：设置双X机架移动模式
      #endif

      #if ENABLED(DELTA)
        case 665: M665(); break;                                  // M665: Set delta configurations//M665：设置增量配置
      #endif

      #if ENABLED(DELTA) || HAS_EXTRA_ENDSTOPS
        case 666: M666(); break;                                  // M666: Set delta or multiple endstop adjustment//M666：设置三角形或多端止动块调整
      #endif

      #if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
        case 672: M672(); break;                                  // M672: Set/clear Duet Smart Effector sensitivity//M672：设置/清除Duet智能效应器灵敏度
      #endif

      #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
        case 701: M701(); break;                                  // M701: Load Filament//M701：负载灯丝
        case 702: M702(); break;                                  // M702: Unload Filament//M702：卸载灯丝
      #endif

      #if ENABLED(CONTROLLER_FAN_EDITABLE)
        case 710: M710(); break;                                  // M710: Set Controller Fan settings//M710：设置控制器风扇设置
      #endif

      #if ENABLED(GCODE_MACROS)
        case 810: case 811: case 812: case 813: case 814:
        case 815: case 816: case 817: case 818: case 819:
        M810_819(); break;                                        // M810-M819: Define/execute G-code macro//M810-M819：定义/执行G代码宏
      #endif

      #if HAS_BED_PROBE
        case 851: M851(); break;                                  // M851: Set Z Probe Z Offset//M851：设置Z探头Z偏移
      #endif

      #if ENABLED(SKEW_CORRECTION_GCODE)
        case 852: M852(); break;                                  // M852: Set Skew factors//M852：设置倾斜因子
      #endif

      #if ENABLED(PROBE_TEMP_COMPENSATION)
        case 192: M192(); break;                                  // M192: Wait for probe temp//M192：等待探针温度
        case 871: M871(); break;                                  // M871: Print/reset/clear first layer temperature offset values//M871：打印/重置/清除第一层温度偏移值
      #endif

      #if ENABLED(LIN_ADVANCE)
        case 900: M900(); break;                                  // M900: Set advance K factor.//M900：设置前进K系数。
      #endif

      #if ANY(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_PWM, HAS_MOTOR_CURRENT_I2C, HAS_MOTOR_CURRENT_DAC)
        case 907: M907(); break;                                  // M907: Set digital trimpot motor current using axis codes.//M907：使用轴代码设置数字微调器电机电流。
        #if EITHER(HAS_MOTOR_CURRENT_SPI, HAS_MOTOR_CURRENT_DAC)
          case 908: M908(); break;                                // M908: Control digital trimpot directly.//M908：直接控制数字微调器。
          #if ENABLED(HAS_MOTOR_CURRENT_DAC)
            case 909: M909(); break;                              // M909: Print digipot/DAC current value//M909：打印数字输出/DAC当前值
            case 910: M910(); break;                              // M910: Commit digipot/DAC value to external EEPROM//M910：将digipot/DAC值提交到外部EEPROM
          #endif
        #endif
      #endif

      #if HAS_TRINAMIC_CONFIG
        case 122: M122(); break;                                  // M122: Report driver configuration and status//M12 2：报告驱动程序配置和状态
        case 906: M906(); break;                                  // M906: Set motor current in milliamps using axis codes X, Y, Z, E//M906：使用轴代码X、Y、Z、E以毫安为单位设置电机电流
        #if HAS_STEALTHCHOP
          case 569: M569(); break;                                // M569: Enable stealthChop on an axis.//M569：在轴上启用隐形斩波。
        #endif
        #if ENABLED(MONITOR_DRIVER_STATUS)
          case 911: M911(); break;                                // M911: Report TMC2130 prewarn triggered flags//M911：报告TMC2130预警触发标志
          case 912: M912(); break;                                // M912: Clear TMC2130 prewarn triggered flags//M912：清除TMC2130预警触发标志
        #endif
        #if ENABLED(HYBRID_THRESHOLD)
          case 913: M913(); break;                                // M913: Set HYBRID_THRESHOLD speed.//M913：设置混合动力车的临界速度。
        #endif
        #if USE_SENSORLESS
          case 914: M914(); break;                                // M914: Set StallGuard sensitivity.//M914：设置失速保护灵敏度。
        #endif
      #endif

      #if HAS_L64XX
        case 122: M122(); break;                                   // M122: Report status//M12 2：报告状态
        case 906: M906(); break;                                   // M906: Set or get motor drive level//M906：设置或获取电机驱动电平
        case 916: M916(); break;                                   // M916: L6470 tuning: Increase drive level until thermal warning//M916:L6470调谐：提高驱动器电平，直到出现热警告
        case 917: M917(); break;                                   // M917: L6470 tuning: Find minimum current thresholds//M917:L6470调谐：查找最小电流阈值
        case 918: M918(); break;                                   // M918: L6470 tuning: Increase speed until max or error//M918:L6470调谐：提高速度直到达到最大值或出现错误
      #endif

      #if HAS_MICROSTEPS
        case 350: M350(); break;                                  // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.//M350：设置微步模式。警告：每单位步数保持不变。S代码为所有驱动程序设置步进模式。
        case 351: M351(); break;                                  // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.//M351：直接切换MS1 MS2引脚，S#确定MS1或MS2，X#设置引脚高/低。
      #endif

      #if ENABLED(CASE_LIGHT_ENABLE)
        case 355: M355(); break;                                  // M355: Set case light brightness//M355：设置箱灯亮度
      #endif

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: parser.debug(); break;                          // M800: GCode Parser Test for M//M800:M的GCode解析器测试
      #endif

      #if ENABLED(GCODE_REPEAT_MARKERS)
        case 808: M808(); break;                                  // M808: Set / Goto repeat markers//M808：设置/转到重复标记
      #endif

      #if ENABLED(I2C_POSITION_ENCODERS)
        case 860: M860(); break;                                  // M860: Report encoder module position//M860：报告编码器模块位置
        case 861: M861(); break;                                  // M861: Report encoder module status//M861：报告编码器模块状态
        case 862: M862(); break;                                  // M862: Perform axis test//M862：执行轴测试
        case 863: M863(); break;                                  // M863: Calibrate steps/mm//M863：校准步数/毫米
        case 864: M864(); break;                                  // M864: Change module address//M864：更改模块地址
        case 865: M865(); break;                                  // M865: Check module firmware version//M865：检查模块固件版本
        case 866: M866(); break;                                  // M866: Report axis error count//M866：报告轴错误计数
        case 867: M867(); break;                                  // M867: Toggle error correction//M867：切换错误校正
        case 868: M868(); break;                                  // M868: Set error correction threshold//M868：设置纠错阈值
        case 869: M869(); break;                                  // M869: Report axis error//M869：报告轴错误
      #endif

      #if ENABLED(MAGNETIC_PARKING_EXTRUDER)
        case 951: M951(); break;                                  // M951: Set Magnetic Parking Extruder parameters//M951：设置磁性停车挤出机参数
      #endif

      #if ENABLED(Z_STEPPER_AUTO_ALIGN)
        case 422: M422(); break;                                  // M422: Set Z Stepper automatic alignment position using probe//M422：使用探针设置Z步进机自动对准位置
      #endif

      #if ALL(HAS_SPI_FLASH, SDSUPPORT, MARLIN_DEV_MODE)
        case 993: M993(); break;                                  // M993: Backup SPI Flash to SD//M993：将SPI闪存备份到SD
        case 994: M994(); break;                                  // M994: Load a Backup from SD to SPI Flash//M994:将备份从SD加载到SPI闪存
      #endif

      #if ENABLED(TOUCH_SCREEN_CALIBRATION)
        case 995: M995(); break;                                  // M995: Touch screen calibration for TFT display//M995：TFT显示屏的触摸屏校准
      #endif

      #if ENABLED(PLATFORM_M997_SUPPORT)
        case 997: M997(); break;                                  // M997: Perform in-application firmware update//M997:执行应用程序内固件更新
      #endif

      case 999: M999(); break;                                    // M999: Restart after being Stopped//M999：停止后重新启动

      #if ENABLED(POWER_LOSS_RECOVERY)
        case 413: M413(); break;                                  // M413: Enable/disable/query Power-Loss Recovery//M413：启用/禁用/查询断电恢复
        case 1000: M1000(); break;                                // M1000: [INTERNAL] Resume from power-loss//M1000:[内部]从断电恢复
      #endif

      #if ENABLED(SDSUPPORT)
        case 1001: M1001(); break;                                // M1001: [INTERNAL] Handle SD completion//M1001:[内部]句柄SD完成
      #endif

      #if ENABLED(DGUS_LCD_UI_MKS)
        case 1002: M1002(); break;                                // M1002: [INTERNAL] Tool-change and Relative E Move//M1002:[内部]换刀和相对E移动
      #endif

      #if ENABLED(UBL_MESH_WIZARD)
        case 1004: M1004(); break;                                // M1004: UBL Mesh Wizard//M1004:UBL网格向导
      #endif

      #if ENABLED(MAX7219_GCODE)
        case 7219: M7219(); break;                                // M7219: Set LEDs, columns, and rows//M7219：设置LED、列和行
      #endif

      default: parser.unknown_command_warning(); break;
    }
    break;

    case 'T': T(parser.codenum); break;                           // Tn: Tool Change//Tn：换刀

    #if ENABLED(MARLIN_DEV_MODE)
      case 'D': D(parser.codenum); break;                         // Dn: Debug codes//Dn：调试代码
    #endif

    #if ENABLED(REALTIME_REPORTING_COMMANDS)
      case 'S': case 'P': case 'R': break;                        // Invalid S, P, R commands already filtered//已筛选出无效的S、P、R命令
    #endif

    default:
      #if ENABLED(WIFI_CUSTOM_COMMAND)
        if (wifi_custom_command(parser.command_ptr)) break;
      #endif
      parser.unknown_command_warning();
  }

  if (!no_ok) queue.ok_to_send();

  SERIAL_OUT(msgDone); // Call the msgDone serial hook to signal command processing done//调用msgDone串行钩子，以信号命令处理完成
}

#if ENABLED(M100_FREE_MEMORY_DUMPER)
  void M100_dump_routine(PGM_P const title, const char * const start, const uintptr_t size);
#endif

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void GcodeSuite::process_next_command() {
  GCodeQueue::CommandLine &command = queue.ring_buffer.peek_next_command();

  PORT_REDIRECT(SERIAL_PORTMASK(command.port));

  TERN_(POWER_LOSS_RECOVERY, recovery.queue_index_r = queue.ring_buffer.index_r);

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(command.buffer);
    #if ENABLED(M100_FREE_MEMORY_DUMPER)
      SERIAL_ECHOPAIR("slot:", queue.ring_buffer.index_r);
      M100_dump_routine(PSTR("   Command Queue:"), (const char*)&queue.ring_buffer, sizeof(queue.ring_buffer));
    #endif
  }

  // Parse the next command in the queue//解析队列中的下一个命令
  parser.parse(command.buffer);
  process_parsed_command();
}

/**
 * Run a series of commands, bypassing the command queue to allow
 * G-code "macros" to be called from within other G-code handlers.
 */

void GcodeSuite::process_subcommands_now_P(PGM_P pgcode) {
  char * const saved_cmd = parser.command_ptr;        // Save the parser state//保存解析器状态
  for (;;) {
    PGM_P const delim = strchr_P(pgcode, '\n');       // Get address of next newline//获取下一个换行符的地址
    const size_t len = delim ? delim - pgcode : strlen_P(pgcode); // Get the command length//获取命令长度
    char cmd[len + 1];                                // Allocate a stack buffer//分配堆栈缓冲区
    strncpy_P(cmd, pgcode, len);                      // Copy the command to the stack//将命令复制到堆栈中
    cmd[len] = '\0';                                  // End with a nul//以nul结尾
    parser.parse(cmd);                                // Parse the command//解析命令
    process_parsed_command(true);                     // Process it (no "ok")//处理它（没有“确定”）
    if (!delim) break;                                // Last command?//最后的命令？
    pgcode = delim + 1;                               // Get the next command//获取下一个命令
  }
  parser.parse(saved_cmd);                            // Restore the parser state//恢复解析器状态
}

void GcodeSuite::process_subcommands_now(char * gcode) {
  char * const saved_cmd = parser.command_ptr;        // Save the parser state//保存解析器状态
  for (;;) {
    char * const delim = strchr(gcode, '\n');         // Get address of next newline//获取下一个换行符的地址
    if (delim) *delim = '\0';                         // Replace with nul//替换为nul
    parser.parse(gcode);                              // Parse the current command//解析当前命令
    process_parsed_command(true);                     // Process it (no "ok")//处理它（没有“确定”）
    if (!delim) break;                                // Last command?//最后的命令？
    *delim = '\n';                                    // Put back the newline//放回新线
    gcode = delim + 1;                                // Get the next command//获取下一个命令
  }
  parser.parse(saved_cmd);                            // Restore the parser state//恢复解析器状态
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void GcodeSuite::host_keepalive() {
    const millis_t ms = millis();
    static millis_t next_busy_signal_ms = 0;
    if (!autoreport_paused && host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      PORT_REDIRECT(SerialMask::All);
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_MSG(STR_BUSY_PROCESSING);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_position_moving());
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_USER);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_HOLD));
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_INPUT);
          TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_HOLD));
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + SEC_TO_MS(host_keepalive_interval);
  }

#endif // HOST_KEEPALIVE_FEATURE//主机保存功能
