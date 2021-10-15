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
 * lcd/dogm/marlinui_DOGM.h
 */

#include "../../inc/MarlinConfigPre.h"

#include <U8glib.h>
#include "HAL_LCD_class_defines.h"

//#define ALTERNATIVE_LCD//#定义可供选择的液晶显示器

#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  // RepRapWorld Graphical LCD//RepRapWorld图形液晶显示器

  #define U8G_CLASS U8GLIB_ST7920_128X64_4X
  #if DISABLED(SDSUPPORT) && (LCD_PINS_D4 == SD_SCK_PIN) && (LCD_PINS_ENABLE == SD_MOSI_PIN)
    #define U8G_PARAM LCD_PINS_RS
  #else
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS
  #endif

#elif ENABLED(U8GLIB_ST7920)

  // RepRap Discount Full Graphics Smart Controller//全图形智能控制器
  // and other variant LCDs using ST7920//以及其他使用ST7920的变型LCD

  #if DISABLED(SDSUPPORT) && (LCD_PINS_D4 == SD_SCK_PIN) && (LCD_PINS_ENABLE == SD_MOSI_PIN)
    #define U8G_CLASS U8GLIB_ST7920_128X64_4X_HAL               // 2 stripes, HW SPI (Shared with SD card. Non-standard LCD adapter on AVR.)//2条带，硬件SPI（与SD卡共享。AVR上的非标准LCD适配器）
    #define U8G_PARAM LCD_PINS_RS
  #else
    #if ENABLED(ALTERNATIVE_LCD)
      #define U8G_CLASS U8GLIB_ST7920_128X64_4X                 // 2 stripes, SW SPI (Original u8glib device)//2条条纹，SW SPI（原装u8glib设备）
    #else
      #define U8G_CLASS U8GLIB_ST7920_128X64_RRD                // Adjust stripes with PAGE_HEIGHT in ultralcd_st7920_u8glib_rrd.h//在ultralcd st7920 u8glib rrd.h中使用页面高度调整条纹
    #endif
    #define U8G_PARAM LCD_PINS_D4, LCD_PINS_ENABLE, LCD_PINS_RS // AVR version ignores these pin settings//AVR版本忽略这些引脚设置
                                                                // HAL version uses these pin settings//HAL版本使用这些pin设置
  #endif

#elif ENABLED(CARTESIO_UI)

  // CartesioUI LCD//CartesioUI液晶显示器

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes//4条纹
    #define FORCE_SOFT_SPI                                      // SW-SPI//SW-SPI
  #else
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes (HW-SPI)//4条条纹（HW-SPI）
  #endif

#elif ENABLED(U8GLIB_LM6059_AF)

  // Based on the Adafruit ST7565 (https://www.adafruit.com/products/250)//基于Adafruit ST7565(https://www.adafruit.com/products/250)

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_LM6059                             // 8 stripes (HW-SPI)//8条条纹（HW-SPI）
  #else
    #define U8G_CLASS U8GLIB_LM6059_2X                          // 4 stripes (HW-SPI)//4条条纹（HW-SPI）
  #endif

#elif ENABLED(U8GLIB_ST7565_64128N)

  // MaKrPanel, Mini Viki, Viki 2.0, AZSMZ 12864 ST7565 controller//MaKrPanel，迷你Viki，Viki 2.0，AZSMZ 12864 ST7565控制器

  #define SMART_RAMPS MB(RAMPS_SMART_EFB, RAMPS_SMART_EEB, RAMPS_SMART_EFF, RAMPS_SMART_EEF, RAMPS_SMART_SF)
  #define U8G_CLASS U8GLIB_64128N_2X_HAL                        // 4 stripes (HW-SPI)//4条条纹（HW-SPI）

  #if (SMART_RAMPS && defined(__SAM3X8E__)) || DOGLCD_SCK != SD_SCK_PIN || DOGLCD_MOSI != SD_MOSI_PIN
    #define FORCE_SOFT_SPI                                      // SW-SPI//SW-SPI
  #endif

#elif ANY(FYSETC_MINI_12864, MKS_MINI_12864, ENDER2_STOCKDISPLAY)

  // The FYSETC Mini 12864 display                              // "4 stripes"//FYSETC Mini 12864显示器//“4条条纹”

  // The MKS_MINI_12864 V1/V2 aren't exact copies of the MiniPanel.//MKS_MINI_12864 V1/V2并非迷你面板的精确副本。
  // Panel management is in u8g_dev_uc1701_mini12864_HAL.cpp with//面板管理在u8g_dev_uc1701_mini12864_HAL.cpp中，带有
  // extra delays added to remove glitches seen with fast MCUs.//增加额外延迟以消除fast MCU出现的故障。

  #define U8G_CLASS U8GLIB_MINI12864_2X_HAL                     // 8 stripes (HW-SPI)//8条条纹（HW-SPI）

#elif ENABLED(MINIPANEL)

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_MINI12864
  #else
    #define U8G_CLASS U8GLIB_MINI12864_2X                       // 8 stripes (HW-SPI)//8条条纹（HW-SPI）
  #endif

#elif ENABLED(MKS_12864OLED_SSD1306)

  // MKS 128x64 (SSD1306) OLED I2C LCD//MKS 128x64（SSD1306）OLED I2C液晶显示器

  #define FORCE_SOFT_SPI                                        // SW-SPI//SW-SPI

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SSD1306_128X64                     // 8 stripes//8条纹
  #endif

#elif EITHER(FYSETC_242_OLED_12864, K3D_242_OLED_CONTROLLER)

  // FYSETC OLED 2.42" 128 × 64 Full Graphics Controller//FYSETC OLED 2.42英寸128×64全图形控制器
  // or K3D OLED 2.42" 128 × 64 Full Graphics Controller//或K3D OLED 2.42英寸128×64全图形控制器

  #define FORCE_SOFT_SPI                                        // SW-SPI//SW-SPI

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SSD1309_128X64_HAL
  #endif

#elif ENABLED(ZONESTAR_12864OLED_SSD1306)

  // Zonestar SSD1306 OLED SPI LCD//Zonestar SSD1306 OLED SPI液晶显示器

  #define FORCE_SOFT_SPI                                        // SW-SPI//SW-SPI
  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1306_128X64_2X                   // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SH1306_128X64                      // 8 stripes//8条纹
  #endif

#elif EITHER(MKS_12864OLED, ZONESTAR_12864OLED)

  // MKS 128x64 (SH1106) OLED I2C LCD//MKS 128x64（SH1106）OLED I2C液晶显示器
  // - or -//-或-
  // Zonestar SH1106 OLED SPI LCD//Zonestar SH1106 OLED SPI液晶显示器

  #define FORCE_SOFT_SPI                                        // SW-SPI//SW-SPI
  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X                   // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SH1106_128X64                      // 8 stripes//8条纹
  #endif

#elif ENABLED(U8GLIB_SH1106_EINSTART)

  // Connected via motherboard header//通过主板头连接

  #define U8G_CLASS U8GLIB_SH1106_128X64
  #define U8G_PARAM DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, LCD_PINS_DC, LCD_PINS_RS

#elif ENABLED(U8GLIB_SH1106)

  // Generic SH1106 OLED I2C LCD//通用SH1106 OLED I2C液晶显示器

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X_I2C_2_WIRE        // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SH1106_128X64_2X                   // 4 stripes//4条纹
  #endif
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)       // I2C//I2C

#elif ENABLED(U8GLIB_SSD1309)

  // Generic support for SSD1309 OLED I2C LCDs//对SSD1309 OLED I2C LCD的通用支持

  #define U8G_CLASS U8GLIB_SSD1309_128X64
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)       // I2C//I2C

#elif ENABLED(U8GLIB_SSD1306)

  // Generic SSD1306 OLED I2C LCD//通用SSD1306 OLED I2C液晶显示器

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X_I2C_2_WIRE       // 4 stripes//4条纹
  #else
    #define U8G_CLASS U8GLIB_SSD1306_128X64_2X                  // 4 stripes//4条纹
  #endif
  #define U8G_PARAM (U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST)

#elif TFT_SCALED_DOGLCD

  // Unspecified 320x240 TFT pre-initialized by built-in bootloader//未指定的320x240 TFT由内置引导加载程序预初始化

  #define U8G_CLASS U8GLIB_TFT_320X240_UPSCALE_FROM_128X64
  #if HAS_FSMC_GRAPHICAL_TFT
    #define U8G_PARAM FSMC_CS_PIN, FSMC_RS_PIN
  #else
    #define U8G_PARAM -1, -1
  #endif

#else

  #if ENABLED(ALTERNATIVE_LCD)
    #define U8G_CLASS U8GLIB_DOGM128                            // 8 stripes (HW-SPI)//8条条纹（HW-SPI）
  #else
    #define U8G_CLASS U8GLIB_DOGM128_2X                         // 4 stripes (HW-SPI)//4条条纹（HW-SPI）
  #endif

#endif

// Use HW-SPI if no other option is specified//如果未指定其他选项，则使用HW-SPI
#ifndef U8G_PARAM
  #if ENABLED(FORCE_SOFT_SPI)
    #define U8G_PARAM DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, DOGLCD_A0 // SW-SPI//SW-SPI
  #else
    #define U8G_PARAM DOGLCD_CS, DOGLCD_A0                      // HW-SPI//HW-SPI
  #endif
#endif

// For selective rendering within a Y range//用于Y范围内的选择性渲染
#define PAGE_OVER(ya)         ((ya) <= u8g.getU8g()->current_page.y1) // Does the current page follow a region top?//当前页面是否跟随区域顶部？
#define PAGE_UNDER(yb)        ((yb) >= u8g.getU8g()->current_page.y0) // Does the current page precede a region bottom?//当前页面是否位于区域底部之前？
#define PAGE_CONTAINS(ya, yb) ((yb) >= u8g.getU8g()->current_page.y0 && (ya) <= u8g.getU8g()->current_page.y1) // Do two vertical regions overlap?//两个垂直区域重叠吗？

extern U8G_CLASS u8g;
