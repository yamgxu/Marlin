/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
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

#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfigPre.h"

#if BOTH(WIFISUPPORT, OTASUPPORT)

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <driver/timer.h>

void OTA_init() {
  ArduinoOTA
    .onError([](ota_error_t error) {
      char *str;
      switch (error) {
        case OTA_AUTH_ERROR:    str = "Auth Failed";    break;
        case OTA_BEGIN_ERROR:   str = "Begin Failed";   break;
        case OTA_CONNECT_ERROR: str = "Connect Failed"; break;
        case OTA_RECEIVE_ERROR: str = "Receive Failed"; break;
        case OTA_END_ERROR:     str = "End Failed";     break;
      }
    });

  ArduinoOTA.begin();
}

void OTA_handle() {
  ArduinoOTA.handle();
}

#endif // WIFISUPPORT && OTASUPPORT
#endif // ARDUINO_ARCH_ESP32
