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

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(PASSWORD_FEATURE)

#include "../../../feature/password/password.h"
#include "../../../core/serial.h"
#include "../../gcode.h"

////
// M510: Lock Printer//M510：锁定打印机
////
void GcodeSuite::M510() {
  password.lock_machine();
}

////
// M511: Unlock Printer//M511：解锁打印机
////
#if ENABLED(PASSWORD_UNLOCK_GCODE)

  void GcodeSuite::M511() {
    if (password.is_locked) {
      password.value_entry = parser.ulongval('P');
      password.authentication_check();
    }
  }

#endif // PASSWORD_UNLOCK_GCODE//密码\解锁\密码

////
// M512: Set/Change/Remove Password//M512：设置/更改/删除密码
////
#if ENABLED(PASSWORD_CHANGE_GCODE)

  void GcodeSuite::M512() {
    if (password.is_set && parser.ulongval('P') != password.value) {
      SERIAL_ECHOLNPGM(STR_WRONG_PASSWORD);
      return;
    }

    if (parser.seenval('S')) {
      password.value_entry = parser.ulongval('S');

      if (password.value_entry < CAT(1e, PASSWORD_LENGTH)) {
        password.is_set = true;
        password.value = password.value_entry;
        SERIAL_ECHOLNPAIR(STR_PASSWORD_SET, password.value); // TODO: Update password.string//TODO:更新密码.string
      }
      else
        SERIAL_ECHOLNPGM(STR_PASSWORD_TOO_LONG);
    }
    else {
      password.is_set = false;
      SERIAL_ECHOLNPGM(STR_PASSWORD_REMOVED);
    }
    SERIAL_ECHOLNPGM(STR_REMINDER_SAVE_SETTINGS);
  }

#endif // PASSWORD_CHANGE_GCODE//密码更改密码

#endif // PASSWORD_FEATURE//密码加密功能
