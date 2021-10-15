/** translatione by yx */
/******************
 * event_loop.cpp *
 ******************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "ftdi_extended.h"

#if ENABLED(FTDI_EXTENDED)
using namespace FTDI;

enum {
  UNPRESSED       = 0x00
};

tiny_timer_t touch_timer;
UIData::flags_t UIData::flags;
uint8_t pressed_tag  = UNPRESSED;

uint8_t UIData::get_persistent_data_mask() {
  // A bit mask for flags that should be stored to the EEPROM.//应存储到EEPROM的标志位掩码。
  // Others are considered temporarily values that need not be//其他被认为是暂时的值，不需要
  // saved.//拯救。
  constexpr flags_t persistent_flags = {
    bits: {
      touch_start_sound:  true,
      touch_end_sound:    true,
      touch_repeat_sound: true,
      show_animations:    true
    }
  };
  return persistent_flags.value;
}

void UIData::reset_persistent_data() {
  // Default values for persistent data//持久数据的默认值
  constexpr flags_t default_flags = {
    bits: {
      touch_start_sound:  true,
      touch_end_sound:    true,
      touch_repeat_sound: true,
      show_animations:    true,
      touch_debouncing:   false,
      ignore_unpress:     false
    }
  };
  flags.value = default_flags.value;
}

uint8_t UIData::get_persistent_data() {
  return flags.value & get_persistent_data_mask();
}

void UIData::set_persistent_data(uint8_t value) {
  flags.value = value & get_persistent_data_mask();
}


void UIData::enable_touch_sounds(bool enabled) {
  UIData::flags.bits.touch_start_sound  = enabled;
  UIData::flags.bits.touch_end_sound    = enabled;
  UIData::flags.bits.touch_repeat_sound = enabled;
}

bool UIData::touch_sounds_enabled() {
  return UIData::flags.bits.touch_start_sound || UIData::flags.bits.touch_end_sound || UIData::flags.bits.touch_repeat_sound;
}

void UIData::enable_animations(bool enabled) {
    UIData::flags.bits.show_animations = enabled;
}

bool UIData::animations_enabled() {
  return UIData::flags.bits.show_animations;
}

namespace FTDI {
  uint8_t EventLoop::get_pressed_tag() {
    return pressed_tag;
  }

  bool EventLoop::is_touch_held() {
    return pressed_tag != 0;
  }

  /**
   * process_events(): Process events from the touch panel.
   *
   * This function consists of a state machine that accomplishes the following:
   *
   *  - Reads the tag register from the touch panel
   *  - Dispatches onTouchStart and onTouchEnd events to the active screen.
   *  - Handles auto-repetition by sending onTouchHeld to the active screen periodically.
   *  - Plays touch feedback "click" sounds when appropriate.
   *  - Performs debouncing to supress spurious touch events.
   */
  void EventLoop::process_events() {
    // If the LCD is processing commands, don't check//如果LCD正在处理命令，不要检查
    // for tags since they may be changing and could//用于标记，因为它们可能正在更改并且可能
    // cause spurious events.//造成虚假事件。
    if (!touch_timer.elapsed(TOUCH_UPDATE_INTERVAL) || CLCD::CommandFifo::is_processing()) {
      return;
    }

    const uint8_t tag = CLCD::get_tag();

    switch (pressed_tag) {
      case UNPRESSED:
        if (tag != 0) {
          #if ENABLED(TOUCH_UI_DEBUG)
            SERIAL_ECHO_MSG("Touch start: ", tag);
          #endif

          pressed_tag = tag;
          current_screen.onRefresh();

          // When the user taps on a button, activate the onTouchStart handler//当用户点击按钮时，激活onTouchStart处理程序
          const uint8_t lastScreen = current_screen.getScreen();

          if (current_screen.onTouchStart(tag)) {
            touch_timer.start();
            if (UIData::flags.bits.touch_start_sound) sound.play(press_sound);
          }

          // In the case in which a touch event triggered a new screen to be//在触摸事件触发新屏幕显示的情况下
          // drawn, we don't issue a touchEnd since it would be sent to the//抽签后，我们不会发布touchEnd，因为它将被发送到
          // wrong screen.//错误的屏幕。
          UIData::flags.bits.ignore_unpress = (lastScreen != current_screen.getScreen());
        }
        else {
          touch_timer.start();
        }
        break;
      default: // PRESSED//压制
        if (!UIData::flags.bits.touch_debouncing) {
          if (tag == pressed_tag) {
            // The user is holding down a button.//用户正在按下一个按钮。
            if (touch_timer.elapsed(1000 / TOUCH_REPEATS_PER_SECOND)) {
              if (current_screen.onTouchHeld(tag)) {
                current_screen.onRefresh();
                if (UIData::flags.bits.touch_repeat_sound) sound.play(repeat_sound);
              }
              touch_timer.start();
            }
          }
          else if (tag == 0) {
            touch_timer.start();
            UIData::flags.bits.touch_debouncing = true;
          }
        }

        else {
          // Debouncing...//去Bouncing。。。

          if (tag == pressed_tag) {
            // If while debouncing, we detect a press, then cancel debouncing.//如果在去抖动时，我们检测到按下，则取消去抖动。
            UIData::flags.bits.touch_debouncing = false;
          }

          else if (touch_timer.elapsed(DEBOUNCE_PERIOD)) {
            UIData::flags.bits.touch_debouncing = false;

            if (UIData::flags.bits.ignore_unpress) {
              UIData::flags.bits.ignore_unpress = false;
              pressed_tag = UNPRESSED;
              break;
            }

            if (UIData::flags.bits.touch_end_sound) sound.play(unpress_sound);

            #if ENABLED(TOUCH_UI_DEBUG)
              SERIAL_ECHO_MSG("Touch end: ", pressed_tag);
            #endif

            const uint8_t saved_pressed_tag = pressed_tag;
            pressed_tag = UNPRESSED;
            current_screen.onTouchEnd(saved_pressed_tag);
            current_screen.onRefresh();
          }
        }
        break;
    } // switch (pressed_tag)//开关（按下标签）

  } // processEvents()//processEvents（）

  void EventLoop::setup() {
    CLCD::init();
    DLCache::init();
    UIData::reset_persistent_data();
    current_screen.start();
  }

  void EventLoop::loop() {
    sound.onIdle();

    /**
      * Guard against re-entry of UI methods, which can
      * crash. Re-entry can happen because some functions
      * (e.g. planner.synchronize) call idle().
      */
    if (!UIData::flags.bits.prevent_reentry) {
      UIData::flags.bits.prevent_reentry = true;
      current_screen.onIdle();
      process_events();
      UIData::flags.bits.prevent_reentry = false;
    }
  }
} // namespace FTDI//名称空间FTDI

#endif // FTDI_EXTENDED//FTDI_扩展
