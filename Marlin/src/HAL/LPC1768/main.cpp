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
#ifdef TARGET_LPC1768

#include <usb/usb.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbcore.h>
#include <usb/cdc.h>
#include <usb/cdcuser.h>
#include <usb/mscuser.h>
#include <CDCSerial.h>
#include <usb/mscuser.h>

#include "../../inc/MarlinConfig.h"
#include "../../core/millis_t.h"

#include "../../sd/cardreader.h"

extern uint32_t MSC_SD_Init(uint8_t pdrv);

extern "C" {
  #include <debug_frmwrk.h>
  extern "C" int isLPC1769();
  extern "C" void disk_timerproc();
}

void SysTick_Callback() { disk_timerproc(); }

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

void HAL_init() {

  // Init LEDs//初始化发光二极管
  #if PIN_EXISTS(LED)
    SET_DIR_OUTPUT(LED_PIN);
    WRITE_PIN_CLR(LED_PIN);
    #if PIN_EXISTS(LED2)
      SET_DIR_OUTPUT(LED2_PIN);
      WRITE_PIN_CLR(LED2_PIN);
      #if PIN_EXISTS(LED3)
        SET_DIR_OUTPUT(LED3_PIN);
        WRITE_PIN_CLR(LED3_PIN);
        #if PIN_EXISTS(LED4)
          SET_DIR_OUTPUT(LED4_PIN);
          WRITE_PIN_CLR(LED4_PIN);
        #endif
      #endif
    #endif

    // Flash status LED 3 times to indicate Marlin has started booting//闪烁状态LED 3次，表示Marlin已开始引导
    LOOP_L_N(i, 6) {
      TOGGLE(LED_PIN);
      delay(100);
    }
  #endif

  // Init Servo Pins//初始伺服销
  #define INIT_SERVO(N) OUT_WRITE(SERVO##N##_PIN, LOW)
  #if HAS_SERVO_0
    INIT_SERVO(0);
  #endif
  #if HAS_SERVO_1
    INIT_SERVO(1);
  #endif
  #if HAS_SERVO_2
    INIT_SERVO(2);
  #endif
  #if HAS_SERVO_3
    INIT_SERVO(3);
  #endif

  //debug_frmwrk_init();//debug_frmwrk_init（）；
  //_DBG("\n\nDebug running\n");//_DBG（“\n\n正在运行的错误\n”）；
  // Initialize the SD card chip select pins as soon as possible//尽快初始化SD卡芯片选择引脚
  #if PIN_EXISTS(SD_SS)
    OUT_WRITE(SD_SS_PIN, HIGH);
  #endif

  #if PIN_EXISTS(ONBOARD_SD_CS) && ONBOARD_SD_CS_PIN != SD_SS_PIN
    OUT_WRITE(ONBOARD_SD_CS_PIN, HIGH);
  #endif

  #ifdef LPC1768_ENABLE_CLKOUT_12M
   /**
    * CLKOUTCFG register
    * bit 8 (CLKOUT_EN) = enables CLKOUT signal. Disabled for now to prevent glitch when enabling GPIO.
    * bits 7:4 (CLKOUTDIV) = set to 0 for divider setting of /1
    * bits 3:0 (CLKOUTSEL) = set to 1 to select main crystal oscillator as CLKOUT source
    */
    LPC_SC->CLKOUTCFG = (0<<8)|(0<<4)|(1<<0);
    // set P1.27 pin to function 01 (CLKOUT)//将P1.27引脚设置为功能01（CLKOUT）
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 27;
    PinCfg.Funcnum = 1;    // function 01 (CLKOUT)//功能01（CLKOUT）
    PinCfg.OpenDrain = 0;  // not open drain//不排水
    PinCfg.Pinmode = 2;    // no pull-up/pull-down//禁止上拉/下拉
    PINSEL_ConfigPin(&PinCfg);
    // now set CLKOUT_EN bit//现在设置CLKOUT_EN位
    SBI(LPC_SC->CLKOUTCFG, 8);
  #endif

  USB_Init();                               // USB Initialization//USB初始化
  USB_Connect(false);                       // USB clear connection//USB清除连接
  delay(1000);                              // Give OS time to notice//给操作系统时间通知
  USB_Connect(true);

  TERN_(HAS_SD_HOST_DRIVE, MSC_SD_Init(0)); // Enable USB SD card access//启用USB SD卡访问

  const millis_t usb_timeout = millis() + 2000;
  while (!USB_Configuration && PENDING(millis(), usb_timeout)) {
    delay(50);
    HAL_idletask();
    #if PIN_EXISTS(LED)
      TOGGLE(LED_PIN);     // Flash quickly during USB initialization//在USB初始化期间快速闪存
    #endif
  }

  HAL_timer_init();

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial()); // Install the min serial handler//安装最小串行处理程序
}

// HAL idle task//HAL空闲任务
void HAL_idletask() {
  #if HAS_SHARED_MEDIA
    // If Marlin is using the SD card we need to lock it to prevent access from//如果Marlin正在使用SD卡，我们需要将其锁定，以防止
    // a PC via USB.//通过USB连接的个人电脑。
    // Other HALs use IS_SD_PRINTING() and IS_SD_FILE_OPEN() to check for access but//HALs的另一个用途是打印（）和打开（）以检查访问，但
    // this will not reliably detect delete operations. To be safe we will lock//这将无法可靠地检测删除操作。为了安全起见，我们将把门锁上
    // the disk if Marlin has it mounted. Unfortunately there is currently no way//如果Marlin安装了磁盘，请将其删除。不幸的是，目前没有办法
    // to unmount the disk from the LCD menu.//从LCD菜单中卸载磁盘。
    // if (IS_SD_PRINTING() || IS_SD_FILE_OPEN())//if（IS_SD_PRINTING（）| | IS_SD_FILE_OPEN（））
    if (card.isMounted())
      MSC_Aquire_Lock();
    else
      MSC_Release_Lock();
  #endif
  // Perform USB stack housekeeping//执行USB堆栈管理
  MSC_RunDeferredCommands();
}

#endif // TARGET_LPC1768//目标为LPC1768
