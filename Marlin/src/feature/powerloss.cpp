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
 * feature/powerloss.cpp - Resume an SD print after power-loss
 */

#include "../inc/MarlinConfigPre.h"

#if ENABLED(POWER_LOSS_RECOVERY)

#include "powerloss.h"
#include "../core/macros.h"

bool PrintJobRecovery::enabled; // Initialized by settings.load()//由settings.load（）初始化

SdFile PrintJobRecovery::file;
job_recovery_info_t PrintJobRecovery::info;
const char PrintJobRecovery::filename[5] = "/PLR";
uint8_t PrintJobRecovery::queue_index_r;
uint32_t PrintJobRecovery::cmd_sdpos, // = 0// = 0
         PrintJobRecovery::sdpos[BUFSIZE];

#if ENABLED(DWIN_CREALITY_LCD)
  bool PrintJobRecovery::dwin_flag; // = false//=错误
#endif

#include "../sd/cardreader.h"
#include "../lcd/marlinui.h"
#include "../gcode/queue.h"
#include "../gcode/gcode.h"
#include "../module/motion.h"
#include "../module/planner.h"
#include "../module/printcounter.h"
#include "../module/temperature.h"
#include "../core/serial.h"

#if ENABLED(FWRETRACT)
  #include "fwretract.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_POWER_LOSS_RECOVERY)
#include "../core/debug_out.h"

PrintJobRecovery recovery;

#ifndef POWER_LOSS_PURGE_LEN
  #define POWER_LOSS_PURGE_LEN 0
#endif

#if DISABLED(BACKUP_POWER_SUPPLY)
  #undef POWER_LOSS_RETRACT_LEN   // No retract at outage without backup power//在没有备用电源的情况下，在大修时不得收回
#endif
#ifndef POWER_LOSS_RETRACT_LEN
  #define POWER_LOSS_RETRACT_LEN 0
#endif

/**
 * Clear the recovery info
 */
void PrintJobRecovery::init() { memset(&info, 0, sizeof(info)); }

/**
 * Enable or disable then call changed()
 */
void PrintJobRecovery::enable(const bool onoff) {
  enabled = onoff;
  changed();
}

/**
 * The enabled state was changed:
 *  - Enabled: Purge the job recovery file
 *  - Disabled: Write the job recovery file
 */
void PrintJobRecovery::changed() {
  if (!enabled)
    purge();
  else if (IS_SD_PRINTING())
    save(true);
}

/**
 * Check for Print Job Recovery during setup()
 *
 * If a saved state exists send 'M1000 S' to initiate job recovery.
 */
void PrintJobRecovery::check() {
  //if (!card.isMounted()) card.mount();//如果（！card.ismount（））card.mount（）；
  if (card.isMounted()) {
    load();
    if (!valid()) return cancel();
    queue.inject_P(PSTR("M1000S"));
  }
}

/**
 * Delete the recovery file and clear the recovery data
 */
void PrintJobRecovery::purge() {
  init();
  card.removeJobRecoveryFile();
}

/**
 * Load the recovery data, if it exists
 */
void PrintJobRecovery::load() {
  if (exists()) {
    open(true);
    (void)file.read(&info, sizeof(info));
    close();
  }
  debug(PSTR("Load"));
}

/**
 * Set info fields that won't change
 */
void PrintJobRecovery::prepare() {
  card.getAbsFilenameInCWD(info.sd_filename);  // SD filename//SD文件名
  cmd_sdpos = 0;
}

/**
 * Save the current machine state to the power-loss recovery file
 */
void PrintJobRecovery::save(const bool force/*=false*/, const float zraise/*=POWER_LOSS_ZRAISE*/, const bool raised/*=false*/) {

  // We don't check IS_SD_PRINTING here so a save may occur during a pause//我们不检查此处是否打印，因此在暂停期间可能会进行保存

  #if SAVE_INFO_INTERVAL_MS > 0
    static millis_t next_save_ms; // = 0// = 0
    millis_t ms = millis();
  #endif

  #ifndef POWER_LOSS_MIN_Z_CHANGE
    #define POWER_LOSS_MIN_Z_CHANGE 0.05  // Vase-mode-friendly out of the box//花瓶模式友好的开箱即用
  #endif

  // Did Z change since the last call?//自从上次打电话以来Z有变化吗？
  if (force
    #if DISABLED(SAVE_EACH_CMD_MODE)      // Always save state when enabled//启用时始终保存状态
      #if SAVE_INFO_INTERVAL_MS > 0       // Save if interval is elapsed//如果间隔已过，则保存
        || ELAPSED(ms, next_save_ms)
      #endif
      // Save if Z is above the last-saved position by some minimum height//如果Z比上次保存的位置高出某个最小高度，则保存
      || current_position.z > info.current_position.z + POWER_LOSS_MIN_Z_CHANGE
    #endif
  ) {

    #if SAVE_INFO_INTERVAL_MS > 0
      next_save_ms = ms + SAVE_INFO_INTERVAL_MS;
    #endif

    // Set Head and Foot to matching non-zero values//将头和脚设置为匹配的非零值
    if (!++info.valid_head) ++info.valid_head; // non-zero in sequence//顺序非零
    //if (!IS_SD_PRINTING()) info.valid_head = 0;//如果（！IS_SD_PRINTING（））info.valid_head=0；
    info.valid_foot = info.valid_head;

    // Machine state//机器状态
    info.current_position = current_position;
    info.feedrate = uint16_t(MMS_TO_MMM(feedrate_mm_s));
    info.zraise = zraise;
    info.flag.raised = raised;                      // Was Z raised before power-off?//断电前Z是否升高？

    TERN_(GCODE_REPEAT_MARKERS, info.stored_repeat = repeat);
    TERN_(HAS_HOME_OFFSET, info.home_offset = home_offset);
    TERN_(HAS_POSITION_SHIFT, info.position_shift = position_shift);
    TERN_(HAS_MULTI_EXTRUDER, info.active_extruder = active_extruder);

    #if DISABLED(NO_VOLUMETRICS)
      info.flag.volumetric_enabled = parser.volumetric_enabled;
      #if HAS_MULTI_EXTRUDER
        for (int8_t e = 0; e < EXTRUDERS; e++) info.filament_size[e] = planner.filament_size[e];
      #else
        if (parser.volumetric_enabled) info.filament_size[0] = planner.filament_size[active_extruder];
      #endif
    #endif

    #if HAS_EXTRUDERS
      HOTEND_LOOP() info.target_temperature[e] = thermalManager.degTargetHotend(e);
    #endif

    TERN_(HAS_HEATED_BED, info.target_temperature_bed = thermalManager.degTargetBed());

    #if HAS_FAN
      COPY(info.fan_speed, thermalManager.fan_speed);
    #endif

    #if HAS_LEVELING
      info.flag.leveling = planner.leveling_active;
      info.fade = TERN0(ENABLE_LEVELING_FADE_HEIGHT, planner.z_fade_height);
    #endif

    TERN_(GRADIENT_MIX, memcpy(&info.gradient, &mixer.gradient, sizeof(info.gradient)));

    #if ENABLED(FWRETRACT)
      COPY(info.retract, fwretract.current_retract);
      info.retract_hop = fwretract.current_hop;
    #endif

    // Elapsed print job time//已用打印作业时间
    info.print_job_elapsed = print_job_timer.duration();

    // Relative axis modes//相对轴模式
    info.axis_relative = gcode.axis_relative;

    // Misc. Marlin flags//杂项。马林旗
    info.flag.dryrun = !!(marlin_debug_flags & MARLIN_DEBUG_DRYRUN);
    info.flag.allow_cold_extrusion = TERN0(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude);

    write();
  }
}

#if PIN_EXISTS(POWER_LOSS)

  #if ENABLED(BACKUP_POWER_SUPPLY)

    void PrintJobRecovery::retract_and_lift(const_float_t zraise) {
      #if POWER_LOSS_RETRACT_LEN || POWER_LOSS_ZRAISE

        gcode.set_relative_mode(true);  // Use relative coordinates//使用相对坐标

        #if POWER_LOSS_RETRACT_LEN
          // Retract filament now//现在收回灯丝
          gcode.process_subcommands_now_P(PSTR("G1 F3000 E-" STRINGIFY(POWER_LOSS_RETRACT_LEN)));
        #endif

        #if POWER_LOSS_ZRAISE
          // Raise the Z axis now//现在升高Z轴
          if (zraise) {
            char cmd[20], str_1[16];
            sprintf_P(cmd, PSTR("G0Z%s"), dtostrf(zraise, 1, 3, str_1));
            gcode.process_subcommands_now(cmd);
          }
        #else
          UNUSED(zraise);
        #endif

        //gcode.axis_relative = info.axis_relative;//gcode.axis\u relative=info.axis\u relative；
        planner.synchronize();
      #endif
    }

  #endif

  /**
   * An outage was detected by a sensor pin.
   *  - If not SD printing, let the machine turn off on its own with no "KILL" screen
   *  - Disable all heaters first to save energy
   *  - Save the recovery data for the current instant
   *  - If backup power is available Retract E and Raise Z
   *  - Go to the KILL screen
   */
  void PrintJobRecovery::_outage() {
    #if ENABLED(BACKUP_POWER_SUPPLY)
      static bool lock = false;
      if (lock) return; // No re-entrance from idle() during retract_and_lift()//在缩回和提升过程中，不得从怠速（）重新进入
      lock = true;
    #endif

    #if POWER_LOSS_ZRAISE
      // Get the limited Z-raise to do now or on resume//现在或在简历中获得有限的Z-Rise
      const float zraise = _MAX(0, _MIN(current_position.z + POWER_LOSS_ZRAISE, Z_MAX_POS - 1) - current_position.z);
    #else
      constexpr float zraise = 0;
    #endif

    // Save the current position, distance that Z was (or should be) raised,//保存当前位置，Z升高（或应升高）的距离，
    // and a flag whether the raise was already done here.//以及是否已经在这里加薪的标志。
    if (IS_SD_PRINTING()) save(true, zraise, ENABLED(BACKUP_POWER_SUPPLY));

    // Disable all heaters to reduce power loss//禁用所有加热器以减少功率损耗
    thermalManager.disable_all_heaters();

    #if ENABLED(BACKUP_POWER_SUPPLY)
      // Do a hard-stop of the steppers (with possibly a loud thud)//用力停止步进器（可能发出一声巨响）
      quickstop_stepper();
      // With backup power a retract and raise can be done now//有了备用电源，现在可以进行缩回和升起
      retract_and_lift(zraise);
    #endif

    kill(GET_TEXT(MSG_OUTAGE_RECOVERY));
  }

#endif

/**
 * Save the recovery info the recovery file
 */
void PrintJobRecovery::write() {

  debug(PSTR("Write"));

  open(false);
  file.seekSet(0);
  const int16_t ret = file.write(&info, sizeof(info));
  if (ret == -1) DEBUG_ECHOLNPGM("Power-loss file write failed.");
  if (!file.close()) DEBUG_ECHOLNPGM("Power-loss file close failed.");
}

/**
 * Resume the saved print job
 */
void PrintJobRecovery::resume() {

  char cmd[MAX_CMD_SIZE+16], str_1[16], str_2[16];

  const uint32_t resume_sdpos = info.sdpos; // Get here before the stepper ISR overwrites it//在步进机ISR覆盖它之前到达这里

  // Apply the dry-run flag if enabled//如果启用，则应用干运行标志
  if (info.flag.dryrun) marlin_debug_flags |= MARLIN_DEBUG_DRYRUN;

  // Restore cold extrusion permission//恢复冷挤压权限
  TERN_(PREVENT_COLD_EXTRUSION, thermalManager.allow_cold_extrude = info.flag.allow_cold_extrusion);

  #if HAS_LEVELING
    // Make sure leveling is off before any G92 and G28//在进行任何G92和G28之前，确保已关闭调平
    gcode.process_subcommands_now_P(PSTR("M420 S0 Z0"));
  #endif

  #if HAS_HEATED_BED
    const celsius_t bt = info.target_temperature_bed;
    if (bt) {
      // Restore the bed temperature//恢复床温
      sprintf_P(cmd, PSTR("M190S%i"), bt);
      gcode.process_subcommands_now(cmd);
    }
  #endif

  // Heat hotend enough to soften material//将热端加热到足以软化材料的程度
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = _MAX(info.target_temperature[e], 180);
      if (et) {
        #if HAS_MULTI_HOTEND
          sprintf_P(cmd, PSTR("T%iS"), e);
          gcode.process_subcommands_now(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109S%i"), et);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Interpret the saved Z according to flags//根据标志解释保存的Z
  const float z_print = info.current_position.z,
              z_raised = z_print + info.zraise;

  ////
  // Home the axes that can safely be homed, and//将可安全归位的轴归位，以及
  // establish the current position as best we can.//尽我们所能确定当前位置。
  ////

  gcode.process_subcommands_now_P(PSTR("G92.9E0")); // Reset E to 0//将E重置为0

  #if Z_HOME_TO_MAX

    float z_now = z_raised;

    // If Z homing goes to max then just move back to the "raised" position//如果Z归位变为最大值，则只需移回“升起”位置
    sprintf_P(cmd, PSTR(
            "G28R0\n"     // Home all axes (no raise)//原点所有轴（无提升）
            "G1Z%sF1200"  // Move Z down to (raised) height//将Z向下移动到（升高）高度
          ), dtostrf(z_now, 1, 3, str_1));
    gcode.process_subcommands_now(cmd);

  #else

    #if ENABLED(POWER_LOSS_RECOVER_ZHOME) && defined(POWER_LOSS_ZHOME_POS)
      #define HOMING_Z_DOWN 1
    #else
      #define HOME_XY_ONLY 1
    #endif

    float z_now = info.flag.raised ? z_raised : z_print;

    // Reset E to 0 and set Z to the real position//将E重置为0，并将Z设置为实际位置
    #if HOME_XY_ONLY
      sprintf_P(cmd, PSTR("G92.9Z%s"), dtostrf(z_now, 1, 3, str_1));
      gcode.process_subcommands_now(cmd);
    #endif

    // Does Z need to be raised now? It should be raised before homing XY.//现在需要调高Z吗？它应该在XY归位之前升起。
    if (z_raised > z_now) {
      z_now = z_raised;
      sprintf_P(cmd, PSTR("G1Z%sF600"), dtostrf(z_now, 1, 3, str_1));
      gcode.process_subcommands_now(cmd);
    }

    // Home XY with no Z raise, and also home Z here if Z isn't homing down below.//原点XY没有Z上升，如果Z没有向下归位，这里也有原点Z。
    gcode.process_subcommands_now_P(PSTR("G28R0" TERN_(HOME_XY_ONLY, "XY"))); // No raise during G28//G28峰会期间没有加薪

  #endif

  #if HOMING_Z_DOWN
    // Move to a safe XY position and home Z while avoiding the print.//移动到安全XY位置和原点Z，同时避免打印。
    constexpr xy_pos_t p = POWER_LOSS_ZHOME_POS;
    sprintf_P(cmd, PSTR("G1X%sY%sF1000\nG28Z"), dtostrf(p.x, 1, 3, str_1), dtostrf(p.y, 1, 3, str_2));
    gcode.process_subcommands_now(cmd);
  #endif

  // Mark all axes as having been homed (no effect on current_position)//将所有轴标记为已原点（对当前位置无影响）
  set_all_homed();

  #if HAS_LEVELING
    // Restore Z fade and possibly re-enable bed leveling compensation.//恢复Z衰减，并可能重新启用河床平整补偿。
    // Leveling may already be enabled due to the ENABLE_LEVELING_AFTER_G28 option.//由于启用\u在\u G28之后进行找平选项，可能已启用找平。
    // TODO: Add a G28 parameter to leave leveling disabled.//TODO:添加G28参数以使水平设置处于禁用状态。
    sprintf_P(cmd, PSTR("M420S%cZ%s"), '0' + (char)info.flag.leveling, dtostrf(info.fade, 1, 1, str_1));
    gcode.process_subcommands_now(cmd);

    #if HOME_XY_ONLY
      // The physical Z was adjusted at power-off so undo the M420S1 correction to Z with G92.9.//物理Z在断电时进行了调整，因此使用G92.9将M420S1校正撤销为Z。
      sprintf_P(cmd, PSTR("G92.9Z%s"), dtostrf(z_now, 1, 1, str_1));
      gcode.process_subcommands_now(cmd);
    #endif
  #endif

  #if ENABLED(POWER_LOSS_RECOVER_ZHOME)
    // Z was homed down to the bed, so move up to the raised height.//Z回到床上，所以向上移动到升高的高度。
    z_now = z_raised;
    sprintf_P(cmd, PSTR("G1Z%sF600"), dtostrf(z_now, 1, 3, str_1));
    gcode.process_subcommands_now(cmd);
  #endif

  // Recover volumetric extrusion state//恢复体积挤压状态
  #if DISABLED(NO_VOLUMETRICS)
    #if HAS_MULTI_EXTRUDER
      for (int8_t e = 0; e < EXTRUDERS; e++) {
        sprintf_P(cmd, PSTR("M200T%iD%s"), e, dtostrf(info.filament_size[e], 1, 3, str_1));
        gcode.process_subcommands_now(cmd);
      }
      if (!info.flag.volumetric_enabled) {
        sprintf_P(cmd, PSTR("M200T%iD0"), info.active_extruder);
        gcode.process_subcommands_now(cmd);
      }
    #else
      if (info.flag.volumetric_enabled) {
        sprintf_P(cmd, PSTR("M200D%s"), dtostrf(info.filament_size[0], 1, 3, str_1));
        gcode.process_subcommands_now(cmd);
      }
    #endif
  #endif

  // Restore all hotend temperatures//恢复所有热端温度
  #if HAS_HOTEND
    HOTEND_LOOP() {
      const celsius_t et = info.target_temperature[e];
      if (et) {
        #if HAS_MULTI_HOTEND
          sprintf_P(cmd, PSTR("T%iS"), e);
          gcode.process_subcommands_now(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109S%i"), et);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Restore the previously active tool (with no_move)//恢复以前激活的刀具（不移动）
  #if HAS_MULTI_EXTRUDER || HAS_MULTI_HOTEND
    sprintf_P(cmd, PSTR("T%i S"), info.active_extruder);
    gcode.process_subcommands_now(cmd);
  #endif

  // Restore print cooling fan speeds//恢复打印冷却风扇速度
  #if HAS_FAN
    FANS_LOOP(i) {
      const int f = info.fan_speed[i];
      if (f) {
        sprintf_P(cmd, PSTR("M106P%iS%i"), i, f);
        gcode.process_subcommands_now(cmd);
      }
    }
  #endif

  // Restore retract and hop state from an active `G10` command//从激活的“G10”命令恢复缩回和跳跃状态
  #if ENABLED(FWRETRACT)
    LOOP_L_N(e, EXTRUDERS) {
      if (info.retract[e] != 0.0) {
        fwretract.current_retract[e] = info.retract[e];
        fwretract.retracted[e] = true;
      }
    }
    fwretract.current_hop = info.retract_hop;
  #endif

  #if ENABLED(GRADIENT_MIX)
    memcpy(&mixer.gradient, &info.gradient, sizeof(info.gradient));
  #endif

  // Un-retract if there was a retract at outage//如果大修时出现收回，则取消收回
  #if ENABLED(BACKUP_POWER_SUPPLY) && POWER_LOSS_RETRACT_LEN > 0
    gcode.process_subcommands_now_P(PSTR("G1E" STRINGIFY(POWER_LOSS_RETRACT_LEN) "F3000"));
  #endif

  // Additional purge on resume if configured//如果已配置，则在恢复时进行额外清除
  #if POWER_LOSS_PURGE_LEN
    sprintf_P(cmd, PSTR("G1 E%d F3000"), (POWER_LOSS_PURGE_LEN) + (POWER_LOSS_RETRACT_LEN));
    gcode.process_subcommands_now(cmd);
  #endif

  #if ENABLED(NOZZLE_CLEAN_FEATURE)
    gcode.process_subcommands_now_P(PSTR("G12"));
  #endif

  // Move back over to the saved XY//移回保存的XY
  sprintf_P(cmd, PSTR("G1X%sY%sF3000"),
    dtostrf(info.current_position.x, 1, 3, str_1),
    dtostrf(info.current_position.y, 1, 3, str_2)
  );
  gcode.process_subcommands_now(cmd);

  // Move back down to the saved Z for printing//向下移动到保存的Z以进行打印
  sprintf_P(cmd, PSTR("G1Z%sF600"), dtostrf(z_print, 1, 3, str_1));
  gcode.process_subcommands_now(cmd);

  // Restore the feedrate//恢复进给速度
  sprintf_P(cmd, PSTR("G1F%d"), info.feedrate);
  gcode.process_subcommands_now(cmd);

  // Restore E position with G92.9//用G92.9恢复E位置
  sprintf_P(cmd, PSTR("G92.9E%s"), dtostrf(info.current_position.e, 1, 3, str_1));
  gcode.process_subcommands_now(cmd);

  TERN_(GCODE_REPEAT_MARKERS, repeat = info.stored_repeat);
  TERN_(HAS_HOME_OFFSET, home_offset = info.home_offset);
  TERN_(HAS_POSITION_SHIFT, position_shift = info.position_shift);
  #if HAS_HOME_OFFSET || HAS_POSITION_SHIFT
    LOOP_LINEAR_AXES(i) update_workspace_offset((AxisEnum)i);
  #endif

  // Relative axis modes//相对轴模式
  gcode.axis_relative = info.axis_relative;

  #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    const uint8_t old_flags = marlin_debug_flags;
    marlin_debug_flags |= MARLIN_DEBUG_ECHO;
  #endif

  // Continue to apply PLR when a file is resumed!//当文件恢复时，继续应用PLR！
  enable(true);

  // Resume the SD file from the last position//从最后一个位置恢复SD文件
  char *fn = info.sd_filename;
  sprintf_P(cmd, M23_STR, fn);
  gcode.process_subcommands_now(cmd);
  sprintf_P(cmd, PSTR("M24S%ldT%ld"), resume_sdpos, info.print_job_elapsed);
  gcode.process_subcommands_now(cmd);

  TERN_(DEBUG_POWER_LOSS_RECOVERY, marlin_debug_flags = old_flags);
}

#if ENABLED(DEBUG_POWER_LOSS_RECOVERY)

  void PrintJobRecovery::debug(PGM_P const prefix) {
    DEBUG_ECHOPGM_P(prefix);
    DEBUG_ECHOLNPAIR(" Job Recovery Info...\nvalid_head:", info.valid_head, " valid_foot:", info.valid_foot);
    if (info.valid_head) {
      if (info.valid_head == info.valid_foot) {
        DEBUG_ECHOPGM("current_position: ");
        LOOP_LOGICAL_AXES(i) {
          if (i) DEBUG_CHAR(',');
          DEBUG_DECIMAL(info.current_position[i]);
        }
        DEBUG_EOL();

        DEBUG_ECHOLNPAIR("feedrate: ", info.feedrate);

        DEBUG_ECHOLNPAIR("zraise: ", info.zraise, " ", info.flag.raised ? "(before)" : "");

        #if ENABLED(GCODE_REPEAT_MARKERS)
          DEBUG_ECHOLNPAIR("repeat index: ", info.stored_repeat.index);
          LOOP_L_N(i, info.stored_repeat.index)
            DEBUG_ECHOLNPAIR("..... sdpos: ", info.stored_repeat.marker.sdpos, " count: ", info.stored_repeat.marker.counter);
        #endif

        #if HAS_HOME_OFFSET
          DEBUG_ECHOPGM("home_offset: ");
          LOOP_LINEAR_AXES(i) {
            if (i) DEBUG_CHAR(',');
            DEBUG_DECIMAL(info.home_offset[i]);
          }
          DEBUG_EOL();
        #endif

        #if HAS_POSITION_SHIFT
          DEBUG_ECHOPGM("position_shift: ");
          LOOP_LINEAR_AXES(i) {
            if (i) DEBUG_CHAR(',');
            DEBUG_DECIMAL(info.position_shift[i]);
          }
          DEBUG_EOL();
        #endif

        #if HAS_MULTI_EXTRUDER
          DEBUG_ECHOLNPAIR("active_extruder: ", info.active_extruder);
        #endif

        #if DISABLED(NO_VOLUMETRICS)
          DEBUG_ECHOPGM("filament_size:");
          LOOP_L_N(i, EXTRUDERS) DEBUG_ECHOLNPAIR(" ", info.filament_size[i]);
          DEBUG_EOL();
        #endif

        #if HAS_HOTEND
          DEBUG_ECHOPGM("target_temperature: ");
          HOTEND_LOOP() {
            DEBUG_ECHO(info.target_temperature[e]);
            if (e < HOTENDS - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
        #endif

        #if HAS_HEATED_BED
          DEBUG_ECHOLNPAIR("target_temperature_bed: ", info.target_temperature_bed);
        #endif

        #if HAS_FAN
          DEBUG_ECHOPGM("fan_speed: ");
          FANS_LOOP(i) {
            DEBUG_ECHO(info.fan_speed[i]);
            if (i < FAN_COUNT - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
        #endif

        #if HAS_LEVELING
          DEBUG_ECHOLNPAIR("leveling: ", info.flag.leveling ? "ON" : "OFF", "  fade: ", info.fade);
        #endif

        #if ENABLED(FWRETRACT)
          DEBUG_ECHOPGM("retract: ");
          for (int8_t e = 0; e < EXTRUDERS; e++) {
            DEBUG_ECHO(info.retract[e]);
            if (e < EXTRUDERS - 1) DEBUG_CHAR(',');
          }
          DEBUG_EOL();
          DEBUG_ECHOLNPAIR("retract_hop: ", info.retract_hop);
        #endif

        // Mixing extruder and gradient//混合挤出机与梯度
        #if BOTH(MIXING_EXTRUDER, GRADIENT_MIX)
          DEBUG_ECHOLNPAIR("gradient: ", info.gradient.enabled ? "ON" : "OFF");
        #endif

        DEBUG_ECHOLNPAIR("sd_filename: ", info.sd_filename);
        DEBUG_ECHOLNPAIR("sdpos: ", info.sdpos);
        DEBUG_ECHOLNPAIR("print_job_elapsed: ", info.print_job_elapsed);

        DEBUG_ECHOPGM("axis_relative:");
        if (TEST(info.axis_relative, REL_X)) DEBUG_ECHOPGM(" REL_X");
        if (TEST(info.axis_relative, REL_Y)) DEBUG_ECHOPGM(" REL_Y");
        if (TEST(info.axis_relative, REL_Z)) DEBUG_ECHOPGM(" REL_Z");
        if (TEST(info.axis_relative, REL_E)) DEBUG_ECHOPGM(" REL_E");
        if (TEST(info.axis_relative, E_MODE_ABS)) DEBUG_ECHOPGM(" E_MODE_ABS");
        if (TEST(info.axis_relative, E_MODE_REL)) DEBUG_ECHOPGM(" E_MODE_REL");
        DEBUG_EOL();

        DEBUG_ECHOLNPAIR("flag.dryrun: ", AS_DIGIT(info.flag.dryrun));
        DEBUG_ECHOLNPAIR("flag.allow_cold_extrusion: ", AS_DIGIT(info.flag.allow_cold_extrusion));
        DEBUG_ECHOLNPAIR("flag.volumetric_enabled: ", AS_DIGIT(info.flag.volumetric_enabled));
      }
      else
        DEBUG_ECHOLNPGM("INVALID DATA");
    }
    DEBUG_ECHOLNPGM("---");
  }

#endif // DEBUG_POWER_LOSS_RECOVERY//调试\u电源\u损耗\u恢复

#endif // POWER_LOSS_RECOVERY//功率损失恢复
