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

#include "../inc/MarlinConfig.h"

#if ENABLED(BLTOUCH)

#include "bltouch.h"

BLTouch bltouch;

bool BLTouch::last_written_mode; // Initialized by settings.load, 0 = Open Drain; 1 = 5V Drain//由settings.load初始化，0=排水明渠；1=5V漏极

#include "../module/servo.h"
#include "../module/probe.h"

void stop();

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

bool BLTouch::command(const BLTCommand cmd, const millis_t &ms) {
  if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("BLTouch Command :", cmd);
  MOVE_SERVO(Z_PROBE_SERVO_NR, cmd);
  safe_delay(_MAX(ms, (uint32_t)BLTOUCH_DELAY)); // BLTOUCH_DELAY is also the *minimum* delay//BLTOUCH_延迟也是*最小*延迟
  return triggered();
}

// Init the class and device. Call from setup().//初始化类和设备。从安装程序（）调用。
void BLTouch::init(const bool set_voltage/*=false*/) {
  // Voltage Setting (if enabled). At every Marlin initialization://电压设置（如果启用）。在每次Marlin初始化时：
  // BLTOUCH < V3.0 and clones: This will be ignored by the probe//BLTOUCH<V3.0和克隆：探测器将忽略此选项
  // BLTOUCH V3.0: SET_5V_MODE or SET_OD_MODE (if enabled).//BLTOUCH V3.0：设置5V模式或设置OD模式（如果启用）。
  //               OD_MODE is the default on power on, but setting it does not hurt//OD_模式是开机时的默认模式，但设置它不会有任何影响
  //               This mode will stay active until manual SET_OD_MODE or power cycle//此模式将保持激活状态，直到手动设置模式或电源循环
  // BLTOUCH V3.1: SET_5V_MODE or SET_OD_MODE (if enabled).//BLTOUCH V3.1：设置5V模式或设置OD模式（如果启用）。
  //               At power on, the probe will default to the eeprom settings configured by the user//通电时，探头将默认为用户配置的eeprom设置
  _reset();
  _stow();

  #if ENABLED(BLTOUCH_FORCE_MODE_SET)

    constexpr bool should_set = true;

  #else

    if (DEBUGGING(LEVELING)) {
      DEBUG_ECHOLNPAIR("last_written_mode - ", last_written_mode);
      DEBUG_ECHOLNPGM("config mode - "
        #if ENABLED(BLTOUCH_SET_5V_MODE)
          "BLTOUCH_SET_5V_MODE"
        #else
          "OD"
        #endif
      );
    }

    const bool should_set = last_written_mode != ENABLED(BLTOUCH_SET_5V_MODE);

  #endif

  if (should_set && set_voltage)
    mode_conv_proc(ENABLED(BLTOUCH_SET_5V_MODE));
}

void BLTouch::clear() {
  _reset();    // RESET or RESET_SW will clear an alarm condition but...//RESET或RESET_SW将清除报警条件，但。。。
               // ...it will not clear a triggered condition in SW mode when the pin is currently up//…当引脚当前处于上升状态时，不会清除SW模式下的触发条件
               // ANTClabs <-- CODE ERROR//Antclab<--代码错误
  _stow();     // STOW will pull up the pin and clear any triggered condition unless it fails, don't care//STOW将拔出销并清除任何触发条件，除非失败，不要在意
  _deploy();   // DEPLOY to test the probe. Could fail, don't care//部署以测试探测器。可能会失败，不在乎
  _stow();     // STOW to be ready for meaningful work. Could fail, don't care//收起，为有意义的工作做好准备。可能会失败，不在乎
}

bool BLTouch::triggered() { return PROBE_TRIGGERED(); }

bool BLTouch::deploy_proc() {
  // Do a DEPLOY//部署
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch DEPLOY requested");

  // Attempt to DEPLOY, wait for DEPLOY_DELAY or ALARM//尝试部署，等待部署延迟或警报
  if (_deploy_query_alarm()) {
    // The deploy might have failed or the probe is already triggered (nozzle too low?)//展开可能已失败或探头已触发（喷嘴过低？）
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch ALARM or TRIGGER after DEPLOY, recovering");

    clear();                               // Get the probe into start condition//使探头进入启动状态

    // Last attempt to DEPLOY//最后一次部署尝试
    if (_deploy_query_alarm()) {
      // The deploy might have failed or the probe is actually triggered (nozzle too low?) again//展开可能失败，或者探头实际再次触发（喷嘴太低？）
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch Recovery Failed");

      SERIAL_ERROR_MSG(STR_STOP_BLTOUCH);  // Tell the user something is wrong, needs action//告诉用户出了问题，需要采取行动
      stop();                              // but it's not too bad, no need to kill, allow restart//但也不算太糟，不需要杀戮，允许重启

      return true;                         // Tell our caller we goofed in case he cares to know//告诉打电话的人我们搞错了，以防他想知道
    }
  }

  // One of the recommended ANTClabs ways to probe, using SW MODE//推荐的Antclab探测方法之一，使用SW模式
  TERN_(BLTOUCH_FORCE_SW_MODE, _set_SW_mode());

  // Now the probe is ready to issue a 10ms pulse when the pin goes up.//现在，探针准备在引脚上升时发出10毫秒脉冲。
  // The trigger STOW (see motion.cpp for example) will pull up the probes pin as soon as the pulse//触发器STOW（例如，请参见motion.cpp）将在脉冲结束后立即拉起探针销
  // is registered.//是注册的。

  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("bltouch.deploy_proc() end");

  return false; // report success to caller//向来电者报告成功
}

bool BLTouch::stow_proc() {
  // Do a STOW//积载
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch STOW requested");

  // A STOW will clear a triggered condition in the probe (10ms pulse).//STOW将清除探头中的触发状态（10毫秒脉冲）。
  // At the moment that we come in here, we might (pulse) or will (SW mode) see the trigger on the pin.//在我们来到这里的那一刻，我们可能（脉冲）或将（SW模式）看到引脚上的触发器。
  // So even though we know a STOW will be ignored if an ALARM condition is active, we will STOW.//因此，即使我们知道如果报警条件处于激活状态，积载将被忽略，我们也将积载。
  // Note: If the probe is deployed AND in an ALARM condition, this STOW will not pull up the pin//注意：如果探头展开且处于报警状态，此收起装置将不会拉起销
  // and the ALARM condition will still be there. --> ANTClabs should change this behavior maybe//警报条件仍将存在。-->Antclab可能会改变这种行为

  // Attempt to STOW, wait for STOW_DELAY or ALARM//尝试收起，等待收起延迟或警报
  if (_stow_query_alarm()) {
    // The stow might have failed//装载可能失败了
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch ALARM or TRIGGER after STOW, recovering");

    _reset();                              // This RESET will then also pull up the pin. If it doesn't//然后，此重置也将拉起销。如果没有
                                           // work and the pin is still down, there will no longer be//工作时，销钉仍在向下，将不再有销钉
                                           // an ALARM condition though.//不过，这是一个警报条件。
                                           // But one more STOW will catch that//但再多加一个积木就够了
    // Last attempt to STOW//最后一次积载
    if (_stow_query_alarm()) {             // so if there is now STILL an ALARM condition://因此，如果现在仍然存在报警条件：

      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch Recovery Failed");

      SERIAL_ERROR_MSG(STR_STOP_BLTOUCH);  // Tell the user something is wrong, needs action//告诉用户出了问题，需要采取行动
      stop();                              // but it's not too bad, no need to kill, allow restart//但也不算太糟，不需要杀戮，允许重启

      return true;                         // Tell our caller we goofed in case he cares to know//告诉打电话的人我们搞错了，以防他想知道
    }
  }

  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("bltouch.stow_proc() end");

  return false; // report success to caller//向来电者报告成功
}

bool BLTouch::status_proc() {
  /**
   * Return a TRUE for "YES, it is DEPLOYED"
   * This function will ensure switch state is reset after execution
   */

  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("BLTouch STATUS requested");

  _set_SW_mode();              // Incidentally, _set_SW_mode() will also RESET any active alarm//顺便说一下，_set_SW_mode（）还将重置任何活动报警
  const bool tr = triggered(); // If triggered in SW mode, the pin is up, it is STOWED//如果在SW模式下触发，则针脚向上，它被收起

  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("BLTouch is ", tr);

  if (tr) _stow(); else _deploy();  // Turn off SW mode, reset any trigger, honor pin state//关闭SW模式，重置任何触发器，尊重引脚状态
  return !tr;
}

void BLTouch::mode_conv_proc(const bool M5V) {
  /**
   * BLTOUCH pre V3.0 and clones: No reaction at all to this sequence apart from a DEPLOY -> STOW
   * BLTOUCH V3.0: This will set the mode (twice) and sadly, a STOW is needed at the end, because of the deploy
   * BLTOUCH V3.1: This will set the mode and store it in the eeprom. The STOW is not needed but does not hurt
   */
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPAIR("BLTouch Set Mode - ", M5V);
  _deploy();
  if (M5V) _set_5V_mode(); else _set_OD_mode();
  _mode_store();
  if (M5V) _set_5V_mode(); else _set_OD_mode();
  _stow();
  last_written_mode = M5V;
}

#endif // BLTOUCH//BLTOUCH
