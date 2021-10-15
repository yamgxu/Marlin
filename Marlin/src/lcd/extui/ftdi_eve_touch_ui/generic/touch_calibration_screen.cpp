/** translatione by yx */
/********************************
 * touch_calibration_screen.cpp *
 ********************************/

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

#include "../config.h"
#include "../screens.h"

#ifdef FTDI_TOUCH_CALIBRATION_SCREEN

using namespace FTDI;
using namespace Theme;

#define GRID_COLS 4
#define GRID_ROWS 16

#define TEXT_POS BTN_POS(1,1), BTN_SIZE(4,12)

void TouchCalibrationScreen::onEntry() {
  CommandProcessor cmd;

  BaseScreen::onEntry();

  if (CLCD::is_touching()) {
    // Ask the user to release the touch before starting,//请用户在启动前释放触摸，
    // as otherwise the first calibration point could//否则，第一个校准点可能会损坏
    // be misinterpreted.//被误解了。
    cmd.cmd(CMD_DLSTART)
       .cmd(CLEAR_COLOR_RGB(bg_color))
       .cmd(CLEAR(true,true,true))
       .cmd(COLOR_RGB(bg_text_enabled));
    draw_text_box(cmd, TEXT_POS, GET_TEXT_F(MSG_TOUCH_CALIBRATION_START), OPT_CENTER, font_large);
    cmd.cmd(DL::DL_DISPLAY)
       .cmd(CMD_SWAP)
       .execute();

    while (CLCD::is_touching()) {
      #if ENABLED(TOUCH_UI_DEBUG)
        SERIAL_ECHO_MSG("Waiting for touch release");
      #endif
    }
  }

  // Force a refresh//强迫刷新
  cmd.cmd(CMD_DLSTART);
  onRedraw(FOREGROUND);
  cmd.cmd(DL::DL_DISPLAY);
  cmd.execute();
}

void TouchCalibrationScreen::onRefresh() {
  // Don't do the regular refresh, as this would//不要进行常规刷新，因为这样会
  // cause the calibration be restarted on every//使校准在每一天重新开始
  // touch.//触摸。
}

void TouchCalibrationScreen::onRedraw(draw_mode_t) {
  CommandProcessor cmd;
  cmd.cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true))
     .cmd(COLOR_RGB(bg_text_enabled));

  draw_text_box(cmd, TEXT_POS, GET_TEXT_F(MSG_TOUCH_CALIBRATION_PROMPT), OPT_CENTER, font_large);
  cmd.cmd(CMD_CALIBRATE);
}

void TouchCalibrationScreen::onIdle() {
  if (!CLCD::is_touching() && !CommandProcessor::is_processing()) {
    GOTO_PREVIOUS();
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("Calibration routine finished");
    #endif
  }
}

#endif // FTDI_TOUCH_CALIBRATION_SCREEN//FTDI触摸屏校准屏
