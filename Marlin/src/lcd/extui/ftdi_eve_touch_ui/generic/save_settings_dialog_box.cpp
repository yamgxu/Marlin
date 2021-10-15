/** translatione by yx */
/********************************
 * save_settings_dialog_box.cpp *
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

#ifdef FTDI_SAVE_SETTINGS_DIALOG_BOX

using namespace ExtUI;

bool SaveSettingsDialogBox::needs_save = false;

void SaveSettingsDialogBox::onRedraw(draw_mode_t) {
  drawMessage(GET_TEXT_F(MSG_EEPROM_SAVE_PROMPT));
  drawYesNoButtons();
}

bool SaveSettingsDialogBox::onTouchEnd(uint8_t tag) {
  needs_save = false;
  switch (tag) {
    case 1:
      injectCommands_P(PSTR("M500"));
      AlertDialogBox::show(GET_TEXT_F(MSG_EEPROM_SAVED));
      // Remove SaveSettingsDialogBox from the stack//从堆栈中删除SaveSettingsDialogBox
      // so the alert box doesn't return to me.//所以警报框不会返回给我。
      current_screen.forget();
      return true;
    default:
      return DialogBoxBaseClass::onTouchEnd(tag);
  }
}

void SaveSettingsDialogBox::promptToSaveSettings() {
   if (needs_save) {
     // Remove current screen from the stack//从堆栈中删除当前屏幕
     // so SaveSettingsDialogBox doesn't return here.//所以SaveSettingsDialogBox不会返回这里。
     GOTO_SCREEN(SaveSettingsDialogBox);
     current_screen.forget();
   }
   else
     GOTO_PREVIOUS(); // No save needed.//不需要保存。
}

#endif // FTDI_SAVE_SETTINGS_DIALOG_BOX//FTDI保存设置对话框
