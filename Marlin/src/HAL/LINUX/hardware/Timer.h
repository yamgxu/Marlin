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

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>

#include "Clock.h"

class Timer {
public:
  Timer();
  virtual ~Timer();

  typedef void (callback_fn)();

  void init(uint32_t sig_id, uint32_t sim_freq, callback_fn* fn);
  void start(uint32_t frequency);
  void enable();
  bool enabled() {return active;}
  void disable();
  void setCompare(uint32_t compare);
  uint32_t getCount();
  uint32_t getCompare() {return compare;}
  uint32_t getOverruns() {return overruns;}
  uint32_t getAvgError() {return avg_error;}

  intptr_t getID() {
    return (*(intptr_t*)timerid);
  }

  static void handler(int sig, siginfo_t *si, void *uc){
    Timer* _this = (Timer*)si->si_value.sival_ptr;
    _this->avg_error += (Clock::nanos() - _this->start_time) - _this->period; //high_resolution_clock is also limited in precision, but best we have//高分辨率时钟的精度也有限，但我们拥有最好的
    _this->avg_error /= 2; //very crude precision analysis (actually within +-500ns usually)//非常粗略的精度分析（通常在+-500ns范围内）
    _this->start_time = Clock::nanos(); // wrap//包裹
    _this->cbfn();
    _this->overruns += timer_getoverrun(_this->timerid); // even at 50Khz this doesn't stay zero, again demonstrating the limitations//即使在50Khz，这也不会保持为零，再次证明了其局限性
                                                         // using a realtime linux kernel would help somewhat//使用实时linux内核会有所帮助
  }

private:
  bool active;
  uint32_t compare;
  uint32_t frequency;
  uint32_t overruns;
  timer_t timerid;
  sigset_t mask;
  callback_fn* cbfn;
  uint64_t period;
  uint64_t avg_error;
  uint64_t start_time;
};
