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

#include "../../inc/MarlinConfig.h"

#include <L6470.h>
#if !(L6470_LIBRARY_VERSION >= 0x000800)
  #error 'L6470_LIBRARY_VERSION 0x000800 or later required'
#endif

#define L6470_GETPARAM(P,Q) stepper##Q.GetParam(P)

#define dSPIN_STEP_CLOCK      0x58
#define dSPIN_STEP_CLOCK_FWD dSPIN_STEP_CLOCK
#define dSPIN_STEP_CLOCK_REV dSPIN_STEP_CLOCK+1
#define HAS_L64XX_EXTRUDER (AXIS_IS_L64XX(E0) || AXIS_IS_L64XX(E1) || AXIS_IS_L64XX(E2) || AXIS_IS_L64XX(E3) || AXIS_IS_L64XX(E4) || AXIS_IS_L64XX(E5) || AXIS_IS_L64XX(E6) || AXIS_IS_L64XX(E7))

#define _EN_ITEM(N) , E##N
enum L64XX_axis_t : uint8_t { LINEAR_AXIS_LIST(X, Y, Z, I, J, K), X2, Y2, Z2, Z3, Z4 REPEAT(E_STEPPERS, _EN_ITEM), MAX_L64XX };
#undef _EN_ITEM

class L64XX_Marlin : public L64XXHelper {
public:
  static PGM_P const index_to_axis[MAX_L64XX];

  static const uint8_t index_to_dir[MAX_L64XX];

  static uint8_t dir_commands[MAX_L64XX];

  // Flags to guarantee graceful switch if stepper interrupts L6470 SPI transfer//当步进器中断L6470 SPI传输时，确保正常切换的标志
  static volatile uint8_t spi_abort;
  static uint8_t spi_active;

  L64XX_Marlin() {}

  static void init();
  static void init_to_defaults();

  static uint16_t get_stepper_status(L64XX &st);

  static uint16_t get_status(const L64XX_axis_t axis);

  static uint32_t get_param(const L64XX_axis_t axis, const uint8_t param);

  static void set_param(const L64XX_axis_t axis, const uint8_t param, const uint32_t value);

  //static void send_command(const L64XX_axis_t axis, uint8_t command);//静态无效发送命令（常量L64XX\u轴\u t轴，uint8\u t命令）；

  static uint8_t get_user_input(uint8_t &driver_count, L64XX_axis_t axis_index[3], char axis_mon[3][3],
                            float &position_max, float &position_min, float &final_feedrate, uint8_t &kval_hold,
                            uint8_t over_current_flag, uint8_t &OCD_TH_val, uint8_t &STALL_TH_val, uint16_t &over_current_threshold);

  static void transfer(uint8_t L6470_buf[], const uint8_t length);

  static void say_axis(const L64XX_axis_t axis, const uint8_t label=true);
  #if ENABLED(L6470_CHITCHAT)
    static void error_status_decode(
      const uint16_t status, const L64XX_axis_t axis,
      const uint16_t _status_axis_th_sd, const uint16_t _status_axis_th_wrn,
      const uint16_t _status_axis_step_loss_a, const uint16_t _status_axis_step_loss_b,
      const uint16_t _status_axis_ocd, const uint8_t _status_axis_layout
    );
  #else
    FORCE_INLINE static void error_status_decode(
      const uint16_t, const L64XX_axis_t,
      const uint16_t, const uint16_t,
      const uint16_t, const uint16_t,
      const uint16_t, const uint8_t
    ){}
  #endif

  // ~40 bytes SRAM to simplify status decode routines//~40字节SRAM，简化状态解码例程
  typedef struct {
    uint8_t STATUS_AXIS_LAYOUT;              // Copy of L6470_status_layout//L6470_状态_布局图的副本
    uint8_t AXIS_OCD_TH_MAX;              // Size of OCD_TH field//OCD_TH字段的大小
    uint8_t AXIS_STALL_TH_MAX;            // Size of STALL_TH field//失速区大小
    float AXIS_OCD_CURRENT_CONSTANT_INV;   // mA per count//每计数毫安
    float AXIS_STALL_CURRENT_CONSTANT_INV; // mA per count//每计数毫安
    uint8_t L6470_AXIS_CONFIG,            // Address of the CONFIG register//配置寄存器的地址
            L6470_AXIS_STATUS;            // Address of the STATUS register//身份登记册地址
    uint16_t L6470_ERROR_MASK,            // STATUS_UVLO | STATUS_TH_WRN | STATUS_TH_SD  | STATUS_OCD | STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B//状态|状态|工作状态|状态| SD |状态|强迫症|状态|步骤|损失| A |状态|步骤|损失|B
             L6474_ERROR_MASK,            // STATUS_UVLO | STATUS_TH_WRN | STATUS_TH_SD  | STATUS_OCD//状态|状态|工作状态|状态| SD |状态|强迫症
             STATUS_AXIS_RAW,             // Copy of status register contents//状态寄存器内容的副本
             STATUS_AXIS,                 // Copy of status register contents but with all error bits active low//状态寄存器内容的副本，但所有错误位均为低电平
             STATUS_AXIS_OCD,             // Overcurrent detected bit position//过电流检测位位置
             STATUS_AXIS_SCK_MOD,         // Step clock mode is active bit position//步进时钟模式为激活位位置
             STATUS_AXIS_STEP_LOSS_A,     // Stall detected on A bridge bit position//在桥接器位位置检测到失速
             STATUS_AXIS_STEP_LOSS_B,     // Stall detected on B bridge bit position//在B桥位位置检测到失速
             STATUS_AXIS_TH_SD,           // Thermal shutdown bit position//热停堆钻头位置
             STATUS_AXIS_TH_WRN,          // Thermal warning bit position//热警告位位置
             STATUS_AXIS_UVLO,            // Undervoltage lockout is active bit position//欠压锁定处于激活位位置
             STATUS_AXIS_WRONG_CMD,       // Last command not valid bit position//最后一个命令位位置无效
             STATUS_AXIS_CMD_ERR,         // Command error bit position//命令错误位位置
             STATUS_AXIS_NOTPERF_CMD;     // Last command not performed bit position//位位置未执行最后一个命令
  } L64XX_shadow_t;

  static L64XX_shadow_t shadow;

  #if ENABLED(MONITOR_L6470_DRIVER_STATUS)
    static bool monitor_paused;
    static inline void pause_monitor(const bool p) { monitor_paused = p; }
    static void monitor_update(L64XX_axis_t stepper_index);
    static void monitor_driver();
  #else
    static inline void pause_monitor(const bool) {}
  #endif

//protected://受保护：
  // L64XXHelper methods//L64XXHelper方法
  static void spi_init();
  static uint8_t transfer_single(uint8_t data, int16_t ss_pin);
  static uint8_t transfer_chain(uint8_t data, int16_t ss_pin, uint8_t chain_position);

private:
  static void append_stepper_err(char* &p, const uint8_t stepper_index, const char * const err=nullptr);

};

void echo_yes_no(const bool yes);

extern L64XX_Marlin L64xxManager;
