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

////////////////////////////////////////////////////////
// VENDOR VERSION EXAMPLE ////供应商版本示例//
////////////////////////////////////////////////////////

/**
 * Marlin release version identifier
 */
//#define SHORT_BUILD_VERSION "bugfix-2.0.x"//#定义短版本“bugfix-2.0.x”

/**
 * Verbose version identifier which should contain a reference to the location
 * from where the binary was downloaded or the source code was compiled.
 */
//#define DETAILED_BUILD_VERSION SHORT_BUILD_VERSION//#定义详细的构建版本简短的构建版本

/**
 * The STRING_DISTRIBUTION_DATE represents when the binary file was built,
 * here we define this default string as the date where the latest release
 * version was tagged.
 */
//#define STRING_DISTRIBUTION_DATE "2019-07-10"//#定义字符串分配日期“2019-07-10”

/**
 * Defines a generic printer name to be output to the LCD after booting Marlin.
 */
//#define MACHINE_NAME "3D Printer"//#定义机器名称“3D打印机”

/**
 * The SOURCE_CODE_URL is the location where users will find the Marlin Source
 * Code which is installed on the device. In most cases —unless the manufacturer
 * has a distinct Github fork— the Source Code URL should just be the main
 * Marlin repository.
 */
//#define SOURCE_CODE_URL "github.com/MarlinFirmware/Marlin"//#定义源代码URL“github.com/MarlinFirmware/Marlin”

/**
 * Default generic printer UUID.
 */
//#define DEFAULT_MACHINE_UUID "cede2a2f-41a2-4748-9b12-c55c62f367ff"//#定义默认机器ID“cede2a2f-41a2-4748-9b12-c55c62f367ff”

/**
 * The WEBSITE_URL is the location where users can get more information such as
 * documentation about a specific Marlin release.
 */
//#define WEBSITE_URL "marlinfw.org"//#定义网站URL“marlinfw.org”

/**
 * Set the vendor info the serial USB interface, if changable
 * Currently only supported by DUE platform
 */
//#define USB_DEVICE_VENDOR_ID           0x0000//#定义USB\u设备\u供应商\u ID 0x0000
//#define USB_DEVICE_PRODUCT_ID          0x0000//#定义USB_设备_产品_ID 0x0000
//#define USB_DEVICE_MANUFACTURE_NAME    WEBSITE_URL//#定义USB\u设备\u制造商\u名称网站\u URL
