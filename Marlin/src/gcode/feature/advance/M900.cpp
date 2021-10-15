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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(LIN_ADVANCE)

#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/stepper.h"

#if ENABLED(EXTRA_LIN_ADVANCE_K)
  float other_extruder_advance_K[EXTRUDERS];
  uint8_t lin_adv_slot = 0;
#endif

/**
 * M900: Get or Set Linear Advance K-factor
 *  T<tool>     Which tool to address
 *  K<factor>   Set current advance K factor (Slot 0).
 *  L<factor>   Set secondary advance K factor (Slot 1). Requires EXTRA_LIN_ADVANCE_K.
 *  S<0/1>      Activate slot 0 or 1. Requires EXTRA_LIN_ADVANCE_K.
 */
void GcodeSuite::M900() {

  auto echo_value_oor = [](const char ltr, const bool ten=true) {
    SERIAL_CHAR('?', ltr);
    SERIAL_ECHOPGM(" value out of range");
    if (ten) SERIAL_ECHOPGM(" (0-10)");
    SERIAL_ECHOLNPGM(".");
  };

  #if EXTRUDERS < 2
    constexpr uint8_t tool_index = 0;
  #else
    const uint8_t tool_index = parser.intval('T', active_extruder);
    if (tool_index >= EXTRUDERS) {
      echo_value_oor('T', false);
      return;
    }
  #endif

  float &kref = planner.extruder_advance_K[tool_index], newK = kref;
  const float oldK = newK;

  #if ENABLED(EXTRA_LIN_ADVANCE_K)

    float &lref = other_extruder_advance_K[tool_index];

    const bool old_slot = TEST(lin_adv_slot, tool_index), // The tool's current slot (0 or 1)//工具的当前插槽（0或1）
               new_slot = parser.boolval('S', old_slot);  // The passed slot (default = current)//传递的插槽（默认值=当前）

    // If a new slot is being selected swap the current and//如果选择了新插槽，则交换当前插槽和
    // saved K values. Do here so K/L will apply correctly.//保存的K值。在此处执行此操作，以便正确应用K/L。
    if (new_slot != old_slot) {                       // Not the same slot?//不一样？
      SET_BIT_TO(lin_adv_slot, tool_index, new_slot); // Update the slot for the tool//更新该工具的插槽
      newK = lref;                                    // Get new K value from backup//从备份中获取新的K值
      lref = oldK;                                    // Save K to backup//将K保存到备份
    }

    // Set the main K value. Apply if the main slot is active.//设置主K值。如果主插槽处于活动状态，则应用。
    if (parser.seenval('K')) {
      const float K = parser.value_float();
      if (!WITHIN(K, 0, 10)) echo_value_oor('K');
      else if (new_slot)        lref = K;             // S1 Knn//S1 Knn
      else                      newK = K;             // S0 Knn//S0 Knn
    }

    // Set the extra K value. Apply if the extra slot is active.//设置额外的K值。如果额外插槽处于活动状态，则应用。
    if (parser.seenval('L')) {
      const float L = parser.value_float();
      if (!WITHIN(L, 0, 10)) echo_value_oor('L');
      else if (!new_slot)       lref = L;             // S0 Lnn//S0 Lnn
      else                      newK = L;             // S1 Lnn//S1 Lnn
    }

  #else

    if (parser.seenval('K')) {
      const float K = parser.value_float();
      if (WITHIN(K, 0, 10))
        newK = K;
      else
        echo_value_oor('K');
    }

  #endif

  if (newK != oldK) {
    planner.synchronize();
    kref = newK;
  }

  if (!parser.seen_any()) {

    #if ENABLED(EXTRA_LIN_ADVANCE_K)

      #if EXTRUDERS < 2
        SERIAL_ECHOLNPAIR("Advance S", new_slot, " K", kref, "(S", !new_slot, " K", lref, ")");
      #else
        LOOP_L_N(i, EXTRUDERS) {
          const bool slot = TEST(lin_adv_slot, i);
          SERIAL_ECHOLNPAIR("Advance T", i, " S", slot, " K", planner.extruder_advance_K[i],
                            "(S", !slot, " K", other_extruder_advance_K[i], ")");
          SERIAL_EOL();
        }
      #endif

    #else

      SERIAL_ECHO_START();
      #if EXTRUDERS < 2
        SERIAL_ECHOLNPAIR("Advance K=", planner.extruder_advance_K[0]);
      #else
        SERIAL_ECHOPGM("Advance K");
        LOOP_L_N(i, EXTRUDERS) {
          SERIAL_CHAR(' ', '0' + i, ':');
          SERIAL_DECIMAL(planner.extruder_advance_K[i]);
        }
        SERIAL_EOL();
      #endif

    #endif
  }

}

#endif // LIN_ADVANCE//林乌前进
