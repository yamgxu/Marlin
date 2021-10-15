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

#include "../../inc/MarlinConfig.h"

#if HAS_MEDIA_SUBCALLS

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../module/planner.h" // for synchronize()//用于同步（）

#include "../../MarlinCore.h" // for startOrResumeJob//对于startOrResumeJob

/**
 * M32: Select file and start SD Print
 *
 * Examples:
 *
 *    M32 !PATH/TO/FILE.GCO#      ; Start FILE.GCO
 *    M32 P !PATH/TO/FILE.GCO#    ; Start FILE.GCO as a procedure
 *    M32 S60 !PATH/TO/FILE.GCO#  ; Start FILE.GCO at byte 60
 */
void GcodeSuite::M32() {
  if (IS_SD_PRINTING()) planner.synchronize();

  if (card.isMounted()) {
    const uint8_t call_procedure = parser.boolval('P');

    card.openFileRead(parser.string_arg, call_procedure);

    if (parser.seenval('S')) card.setIndex(parser.value_long());

    card.startOrResumeFilePrinting();

    // Procedure calls count as normal print time.//过程调用计数为正常打印时间。
    if (!call_procedure) startOrResumeJob();
  }
}

#endif // HAS_MEDIA_SUBCALLS//有\u媒体\u子载波
