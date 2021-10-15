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
#pragma once

/**
 * lcd/extui/dgus/DGUSScreenHandler.h
 */

#include "../../../inc/MarlinConfigPre.h"

#include "../ui_api.h"

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)

  typedef struct  {
    ExtUI::extruder_t extruder; // which extruder to operate//操作哪台挤出机
    uint8_t action; // load or unload//装卸
    bool heated; // heating done ?//暖气好了吗？
    float purge_length; // the length to extrude before unload, prevent filament jam//卸料前挤出的长度，防止纤维堵塞
  } filament_data_t;

  extern filament_data_t filament_data;

#endif

// endianness swap//endianness交换
inline uint16_t swap16(const uint16_t value) { return (value & 0xFFU) << 8U | (value >> 8U); }

#if ENABLED(DGUS_LCD_UI_ORIGIN)
  #include "origin/DGUSScreenHandler.h"
#elif ENABLED(DGUS_LCD_UI_MKS)
  #include "mks/DGUSScreenHandler.h"
#elif ENABLED(DGUS_LCD_UI_FYSETC)
  #include "fysetc/DGUSScreenHandler.h"
#elif ENABLED(DGUS_LCD_UI_HIPRECY)
  #include "hiprecy/DGUSScreenHandler.h"
#endif

extern DGUSScreenHandler ScreenHandler;

// Helper to define a DGUS_VP_Variable for common use-cases.//帮助器为常见用例定义DGUS_VP_变量。
#define VPHELPER(VPADR, VPADRVAR, RXFPTR, TXFPTR) { \
  .VP = VPADR, \
  .memadr = VPADRVAR, \
  .size = sizeof(VPADRVAR), \
  .set_by_display_handler = RXFPTR, \
  .send_to_display_handler = TXFPTR \
}

// Helper to define a DGUS_VP_Variable when the size of the var cannot be determined automatically (e.g., a string)//当变量的大小无法自动确定时（例如字符串），帮助定义DGUS_VP_变量
#define VPHELPER_STR(VPADR, VPADRVAR, STRLEN, RXFPTR, TXFPTR) { \
  .VP = VPADR, \
  .memadr = VPADRVAR, \
  .size = STRLEN, \
  .set_by_display_handler = RXFPTR, \
  .send_to_display_handler = TXFPTR \
}
