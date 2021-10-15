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

#include "../../core/macros.h"

#if BOTH(SDSUPPORT, HAS_MARLINUI_U8GLIB) && (LCD_PINS_D4 == SD_SCK_PIN || LCD_PINS_ENABLE == SD_MOSI_PIN || DOGLCD_SCK == SD_SCK_PIN || DOGLCD_MOSI == SD_MOSI_PIN)
  #define LPC_SOFTWARE_SPI  // If the SD card and LCD adapter share the same SPI pins, then software SPI is currently//如果SD卡和LCD适配器共享相同的SPI引脚，则当前正在使用软件SPI
                            // needed due to the speed and mode required for communicating with each device being different.//由于与每个设备通信所需的速度和模式不同，因此需要。
                            // This requirement can be removed if the SPI access to these devices is updated to use//如果对这些设备的SPI访问更新为使用，则可以删除此要求
                            // spiBeginTransaction.//spibeginsaction。
#endif

/** onboard SD card */
//#define SD_SCK_PIN        P0_07//#定义SD_SCK_引脚P0_07
//#define SD_MISO_PIN       P0_08//#定义SD_MISO_引脚P0_08
//#define SD_MOSI_PIN       P0_09//#定义SD_MOSI_引脚P0_09
//#define SD_SS_PIN         P0_06//#定义SD_不锈钢_引脚P0_06
/** external */
#ifndef SD_SCK_PIN
  #define SD_SCK_PIN        P0_15
#endif
#ifndef SD_MISO_PIN
  #define SD_MISO_PIN       P0_17
#endif
#ifndef SD_MOSI_PIN
  #define SD_MOSI_PIN       P0_18
#endif
#ifndef SD_SS_PIN
  #define SD_SS_PIN         P1_23
#endif
#if !defined(SDSS) || SDSS == P_NC // gets defaulted in pins.h//在pins.h中获取默认值
  #undef SDSS
  #define SDSS          SD_SS_PIN
#endif
