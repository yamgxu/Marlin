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

#if ENABLED(USE_CONTROLLER_FAN)

#include "controllerfan.h"
#include "../module/stepper/indirection.h"
#include "../module/temperature.h"

ControllerFan controllerFan;

uint8_t ControllerFan::speed;

#if ENABLED(CONTROLLER_FAN_EDITABLE)
  controllerFan_settings_t ControllerFan::settings; // {0}// {0}
 #else
   const controllerFan_settings_t &ControllerFan::settings = controllerFan_defaults;
#endif

void ControllerFan::setup() {
  SET_OUTPUT(CONTROLLER_FAN_PIN);
  init();
}

void ControllerFan::set_fan_speed(const uint8_t s) {
  speed = s < (CONTROLLERFAN_SPEED_MIN) ? 0 : s; // Fan OFF below minimum//扇形关闭低于最小值
}

void ControllerFan::update() {
  static millis_t lastMotorOn = 0,    // Last time a motor was turned on//上次打开马达的时候
                  nextMotorCheck = 0; // Last time the state was checked//上次检查状态时
  const millis_t ms = millis();
  if (ELAPSED(ms, nextMotorCheck)) {
    nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s//不是时间关键型功能，因此仅每2.5秒检查一次

    #define MOTOR_IS_ON(A,B) (A##_ENABLE_READ() == bool(B##_ENABLE_ON))
    #define _OR_ENABLED_E(N) || MOTOR_IS_ON(E##N,E)

    const bool motor_on = (
      ( DISABLED(CONTROLLER_FAN_IGNORE_Z) &&
        (    MOTOR_IS_ON(Z,Z)
          || TERN0(HAS_Z2_ENABLE, MOTOR_IS_ON(Z2,Z))
          || TERN0(HAS_Z3_ENABLE, MOTOR_IS_ON(Z3,Z))
          || TERN0(HAS_Z4_ENABLE, MOTOR_IS_ON(Z4,Z))
        )
      ) || (
        DISABLED(CONTROLLER_FAN_USE_Z_ONLY) &&
        (    MOTOR_IS_ON(X,X) || MOTOR_IS_ON(Y,Y)
          || TERN0(HAS_X2_ENABLE, MOTOR_IS_ON(X2,X))
          || TERN0(HAS_Y2_ENABLE, MOTOR_IS_ON(Y2,Y))
          #if E_STEPPERS
            REPEAT(E_STEPPERS, _OR_ENABLED_E)
          #endif
        )
      )
    );

    // If any of the drivers or the heated bed are enabled...//如果任何驱动器或加热床已启用。。。
    if (motor_on || TERN0(HAS_HEATED_BED, thermalManager.temp_bed.soft_pwm_amount > 0))
      lastMotorOn = ms; //... set time to NOW so the fan will turn on//... 将时间设置为现在，以便风扇将打开

    // Fan Settings. Set fan > 0://风扇设置。将风扇设置为>0：
    //  - If AutoMode is on and steppers have been enabled for CONTROLLERFAN_IDLE_TIME seconds.//-如果自动模式开启且步进器已启用控制器空闲时间秒。
    //  - If System is on idle and idle fan speed settings is activated.//-如果系统处于怠速且怠速风扇转速设置激活。
    set_fan_speed(
      settings.auto_mode && lastMotorOn && PENDING(ms, lastMotorOn + SEC_TO_MS(settings.duration))
      ? settings.active_speed : settings.idle_speed
    );

    // Allow digital or PWM fan output (see M42 handling)//允许数字或PWM风扇输出（参见M42处理）
    WRITE(CONTROLLER_FAN_PIN, speed);
    analogWrite(pin_t(CONTROLLER_FAN_PIN), speed);
  }
}

#endif // USE_CONTROLLER_FAN//使用\u控制器\u风扇
