/** translatione by yx */
/**
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */
#pragma once

/* USB functions */
#define _usb_h_

#include "../../../inc/MarlinConfigPre.h"

// WARNING: Do not change the order of includes, or stuff will break!//警告：请勿更改包含的顺序，否则内容将被破坏！
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>

// None of these should ever be included by a driver, or a user's sketch.//驱动程序或用户草图不应包含这些内容。
#include "settings.h"
#include "printhex.h"
#include "message.h"

#include "hexdump.h"
//#include "sink_parser.h"//#包括“sink_parser.h”
#include "max3421e.h"
#include "address.h"
//#include "avrpins.h"//#包括“avrpins.h”
#include "usb_ch9.h"
#include "usbhost.h"
#include "UsbCore.h"
#include "parsetools.h"
#include "confdescparser.h"

#undef _usb_h_
