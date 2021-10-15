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

/**
 * The monitor_driver routines are a close copy of the TMC code
 */

#include "../../inc/MarlinConfig.h"

#if HAS_L64XX

#include "L64XX_Marlin.h"

L64XX_Marlin L64xxManager;

#include "../../module/stepper/indirection.h"
#include "../../gcode/gcode.h"
#include "../../module/planner.h"
#include "../../HAL/shared/Delay.h"

static const char LINEAR_AXIS_LIST(
                   str_X[] PROGMEM = "X ",          str_Y[] PROGMEM = "Y ",          str_Z[] PROGMEM = "Z ",
                   str_I[] PROGMEM = AXIS4_STR " ", str_J[] PROGMEM = AXIS5_STR " ", str_K[] PROGMEM = AXIS6_STR " "
                 ),
                 str_X2[] PROGMEM = "X2", str_Y2[] PROGMEM = "Y2",
                 str_Z2[] PROGMEM = "Z2", str_Z3[] PROGMEM = "Z3", str_Z4[] PROGMEM = "Z4",
                 LIST_N(EXTRUDERS,
                   str_E0[] PROGMEM = "E0", str_E1[] PROGMEM = "E1",
                   str_E2[] PROGMEM = "E2", str_E3[] PROGMEM = "E3",
                   str_E4[] PROGMEM = "E4", str_E5[] PROGMEM = "E5",
                   str_E6[] PROGMEM = "E6", str_E7[] PROGMEM = "E7"
                 )
                 ;

#define _EN_ITEM(N) , str_E##N
PGM_P const L64XX_Marlin::index_to_axis[] PROGMEM = {
  LINEAR_AXIS_LIST(str_X, str_Y, str_Z, str_I, str_J, str_K),
  str_X2, str_Y2, str_Z2, str_Z3, str_Z4
  REPEAT(E_STEPPERS, _EN_ITEM)
};
#undef _EN_ITEM

#define DEBUG_OUT ENABLED(L6470_CHITCHAT)
#include "../../core/debug_out.h"

void echo_yes_no(const bool yes) { DEBUG_ECHOPGM_P(yes ? PSTR(" YES") : PSTR(" NO ")); UNUSED(yes); }

uint8_t L64XX_Marlin::dir_commands[MAX_L64XX];  // array to hold direction command for each driver//为每个驱动器保留方向命令的数组

#define _EN_ITEM(N) , INVERT_E##N##_DIR
const uint8_t L64XX_Marlin::index_to_dir[MAX_L64XX] = {
    LINEAR_AXIS_LIST(INVERT_X_DIR, INVERT_Y_DIR, INVERT_Z_DIR, INVERT_I_DIR, INVERT_J_DIR, INVERT_K_DIR)
  , (INVERT_X_DIR) ^ BOTH(X_DUAL_STEPPER_DRIVERS, INVERT_X2_VS_X_DIR) // X2//X2
  , (INVERT_Y_DIR) ^ BOTH(Y_DUAL_STEPPER_DRIVERS, INVERT_Y2_VS_Y_DIR) // Y2//Y2
  , (INVERT_Z_DIR) ^ ENABLED(INVERT_Z2_VS_Z_DIR) // Z2//Z2
  , (INVERT_Z_DIR) ^ ENABLED(INVERT_Z3_VS_Z_DIR) // Z3//Z3
  , (INVERT_Z_DIR) ^ ENABLED(INVERT_Z4_VS_Z_DIR) // Z4//Z4
    REPEAT(E_STEPPERS, _EN_ITEM)
};
#undef _EN_ITEM

volatile uint8_t L64XX_Marlin::spi_abort = false;
uint8_t L64XX_Marlin::spi_active = false;

L64XX_Marlin::L64XX_shadow_t L64XX_Marlin::shadow;

//uint32_t UVLO_ADC = 0x0400; // ADC undervoltage event//uint32\u t UVLO\u ADC=0x0400；//ADC欠压事件

void L6470_populate_chain_array() {

  #define _L6470_INIT_SPI(Q)  do{ stepper##Q.set_chain_info(Q, Q##_CHAIN_POS); }while(0)

  #if AXIS_IS_L64XX(X)
    _L6470_INIT_SPI(X);
  #endif
  #if AXIS_IS_L64XX(X2)
    _L6470_INIT_SPI(X2);
  #endif
  #if AXIS_IS_L64XX(Y)
    _L6470_INIT_SPI(Y);
  #endif
  #if AXIS_IS_L64XX(Y2)
    _L6470_INIT_SPI(Y2);
  #endif
  #if AXIS_IS_L64XX(Z)
    _L6470_INIT_SPI(Z);
  #endif
  #if AXIS_IS_L64XX(Z2)
    _L6470_INIT_SPI(Z2);
  #endif
  #if AXIS_IS_L64XX(Z3)
    _L6470_INIT_SPI(Z3);
  #endif
  #if AXIS_IS_L64XX(Z4)
    _L6470_INIT_SPI(Z4);
  #endif
  #if AXIS_IS_L64XX(E0)
    _L6470_INIT_SPI(E0);
  #endif
  #if AXIS_IS_L64XX(E1)
    _L6470_INIT_SPI(E1);
  #endif
  #if AXIS_IS_L64XX(E2)
    _L6470_INIT_SPI(E2);
  #endif
  #if AXIS_IS_L64XX(E3)
    _L6470_INIT_SPI(E3);
  #endif
  #if AXIS_IS_L64XX(E4)
    _L6470_INIT_SPI(E4);
  #endif
  #if AXIS_IS_L64XX(E5)
    _L6470_INIT_SPI(E5);
  #endif
  #if AXIS_IS_L64XX(E6)
    _L6470_INIT_SPI(E6);
  #endif
  #if AXIS_IS_L64XX(E7)
    _L6470_INIT_SPI(E7);
  #endif
}


/**
 * Some status bit positions & definitions differ per driver.
 * Copy info to known locations to simplfy check/display logic.
 *   1. Copy stepper status
 *   2. Copy status bit definitions
 *   3. Copy status layout
 *   4. Make all error bits active low (as needed)
 */
uint16_t L64XX_Marlin::get_stepper_status(L64XX &st) {
  shadow.STATUS_AXIS_RAW           = st.getStatus();
  shadow.STATUS_AXIS               = shadow.STATUS_AXIS_RAW;
  shadow.STATUS_AXIS_LAYOUT        = st.L6470_status_layout;
  shadow.AXIS_OCD_TH_MAX           = st.OCD_TH_MAX;
  shadow.AXIS_STALL_TH_MAX         = st.STALL_TH_MAX;
  shadow.AXIS_OCD_CURRENT_CONSTANT_INV   = st.OCD_CURRENT_CONSTANT_INV;
  shadow.AXIS_STALL_CURRENT_CONSTANT_INV = st.STALL_CURRENT_CONSTANT_INV;
  shadow.L6470_AXIS_CONFIG         = st.L64XX_CONFIG;
  shadow.L6470_AXIS_STATUS         = st.L64XX_STATUS;
  shadow.STATUS_AXIS_OCD           = st.STATUS_OCD;
  shadow.STATUS_AXIS_SCK_MOD       = st.STATUS_SCK_MOD;
  shadow.STATUS_AXIS_STEP_LOSS_A   = st.STATUS_STEP_LOSS_A;
  shadow.STATUS_AXIS_STEP_LOSS_B   = st.STATUS_STEP_LOSS_B;
  shadow.STATUS_AXIS_TH_SD         = st.STATUS_TH_SD;
  shadow.STATUS_AXIS_TH_WRN        = st.STATUS_TH_WRN;
  shadow.STATUS_AXIS_UVLO          = st.STATUS_UVLO;
  shadow.STATUS_AXIS_WRONG_CMD     = st.STATUS_WRONG_CMD;
  shadow.STATUS_AXIS_CMD_ERR       = st.STATUS_CMD_ERR;
  shadow.STATUS_AXIS_NOTPERF_CMD   = st.STATUS_NOTPERF_CMD;

  switch (shadow.STATUS_AXIS_LAYOUT) {
    case L6470_STATUS_LAYOUT: {   // L6470//L6470
      shadow.L6470_ERROR_MASK      = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD | shadow.STATUS_AXIS_STEP_LOSS_A | shadow.STATUS_AXIS_STEP_LOSS_B;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_WRONG_CMD | shadow.STATUS_AXIS_NOTPERF_CMD);  // invert just error bits that are active high//仅反转高电平活动的错误位
      break;
    }
    case L6474_STATUS_LAYOUT: {   // L6474//L6474
      shadow.L6470_ERROR_MASK      = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD ;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_WRONG_CMD | shadow.STATUS_AXIS_NOTPERF_CMD);  // invert just error bits that are active high//仅反转高电平活动的错误位
      break;
    }
    case L6480_STATUS_LAYOUT: {   // L6480 & powerSTEP01//L6480和功率步骤01
      shadow.L6470_ERROR_MASK      = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD | shadow.STATUS_AXIS_STEP_LOSS_A | shadow.STATUS_AXIS_STEP_LOSS_B;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_CMD_ERR | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD);  // invert just error bits that are active high//仅反转高电平活动的错误位
      break;
    }
  }
  return shadow.STATUS_AXIS;
}


void L64XX_Marlin::init() {               // Set up SPI and then init chips//设置SPI，然后设置初始化芯片
  ENABLE_RESET_L64XX_CHIPS(LOW);          // hardware reset of drivers//驱动程序的硬件重置
  DELAY_US(100);
  ENABLE_RESET_L64XX_CHIPS(HIGH);
  DELAY_US(1000);                         // need about 650µs for the chip(s) to fully start up//芯片需要大约650µs才能完全启动
  L6470_populate_chain_array();           // Set up array to control where in the SPI transfer sequence a particular stepper's data goes//设置阵列以控制特定步进器数据在SPI传输序列中的位置

  spi_init();                             // Since L64XX SPI pins are unset we must init SPI here//由于L64XX SPI引脚未设置，我们必须在此处初始化SPI

  init_to_defaults();                     // init the chips//启动芯片
}

uint16_t L64XX_Marlin::get_status(const L64XX_axis_t axis) {

  #define STATUS_L6470(Q) get_stepper_status(stepper##Q)

  switch (axis) {
    default: break;
    #if AXIS_IS_L64XX(X)
      case X : return STATUS_L6470(X);
    #endif
    #if AXIS_IS_L64XX(Y)
      case Y : return STATUS_L6470(Y);
    #endif
    #if AXIS_IS_L64XX(Z)
      case Z : return STATUS_L6470(Z);
    #endif
    #if AXIS_IS_L64XX(X2)
      case X2: return STATUS_L6470(X2);
    #endif
    #if AXIS_IS_L64XX(Y2)
      case Y2: return STATUS_L6470(Y2);
    #endif
    #if AXIS_IS_L64XX(Z2)
      case Z2: return STATUS_L6470(Z2);
    #endif
    #if AXIS_IS_L64XX(Z3)
      case Z3: return STATUS_L6470(Z3);
    #endif
    #if AXIS_IS_L64XX(Z4)
      case Z4: return STATUS_L6470(Z4);
    #endif
    #if AXIS_IS_L64XX(E0)
      case E0: return STATUS_L6470(E0);
    #endif
    #if AXIS_IS_L64XX(E1)
      case E1: return STATUS_L6470(E1);
    #endif
    #if AXIS_IS_L64XX(E2)
      case E2: return STATUS_L6470(E2);
    #endif
    #if AXIS_IS_L64XX(E3)
      case E3: return STATUS_L6470(E3);
    #endif
    #if AXIS_IS_L64XX(E4)
      case E4: return STATUS_L6470(E4);
    #endif
    #if AXIS_IS_L64XX(E5)
      case E5: return STATUS_L6470(E5);
    #endif
    #if AXIS_IS_L64XX(E6)
      case E6: return STATUS_L6470(E6);
    #endif
    #if AXIS_IS_L64XX(E7)
      case E7: return STATUS_L6470(E7);
    #endif
  }

  return 0; // Not needed but kills a compiler warning//不需要，但会终止编译器警告
}

uint32_t L64XX_Marlin::get_param(const L64XX_axis_t axis, const uint8_t param) {

  #define GET_L6470_PARAM(Q) L6470_GETPARAM(param, Q)

  switch (axis) {
    default: break;
    #if AXIS_IS_L64XX(X)
      case X : return GET_L6470_PARAM(X);
    #endif
    #if AXIS_IS_L64XX(Y)
      case Y : return GET_L6470_PARAM(Y);
    #endif
    #if AXIS_IS_L64XX(Z)
      case Z : return GET_L6470_PARAM(Z);
    #endif
    #if AXIS_IS_L64XX(X2)
      case X2: return GET_L6470_PARAM(X2);
    #endif
    #if AXIS_IS_L64XX(Y2)
      case Y2: return GET_L6470_PARAM(Y2);
    #endif
    #if AXIS_IS_L64XX(Z2)
      case Z2: return GET_L6470_PARAM(Z2);
    #endif
    #if AXIS_IS_L64XX(Z3)
      case Z3: return GET_L6470_PARAM(Z3);
    #endif
    #if AXIS_IS_L64XX(Z4)
      case Z4: return GET_L6470_PARAM(Z4);
    #endif
    #if AXIS_IS_L64XX(E0)
      case E0: return GET_L6470_PARAM(E0);
    #endif
    #if AXIS_IS_L64XX(E1)
      case E1: return GET_L6470_PARAM(E1);
    #endif
    #if AXIS_IS_L64XX(E2)
      case E2: return GET_L6470_PARAM(E2);
    #endif
    #if AXIS_IS_L64XX(E3)
      case E3: return GET_L6470_PARAM(E3);
    #endif
    #if AXIS_IS_L64XX(E4)
      case E4: return GET_L6470_PARAM(E4);
    #endif
    #if AXIS_IS_L64XX(E5)
      case E5: return GET_L6470_PARAM(E5);
    #endif
    #if AXIS_IS_L64XX(E6)
      case E6: return GET_L6470_PARAM(E6);
    #endif
    #if AXIS_IS_L64XX(E7)
      case E7: return GET_L6470_PARAM(E7);
    #endif
  }

  return 0; // not needed but kills a compiler warning//不需要，但会终止编译器警告
}

void L64XX_Marlin::set_param(const L64XX_axis_t axis, const uint8_t param, const uint32_t value) {

  #define SET_L6470_PARAM(Q) stepper##Q.SetParam(param, value)

  switch (axis) {
    default: break;
    #if AXIS_IS_L64XX(X)
      case X : SET_L6470_PARAM(X); break;
    #endif
    #if AXIS_IS_L64XX(Y)
      case Y : SET_L6470_PARAM(Y); break;
    #endif
    #if AXIS_IS_L64XX(Z)
      case Z : SET_L6470_PARAM(Z); break;
    #endif
    #if AXIS_IS_L64XX(I)
      case I : SET_L6470_PARAM(I); break;
    #endif
    #if AXIS_IS_L64XX(J)
      case J : SET_L6470_PARAM(J); break;
    #endif
    #if AXIS_IS_L64XX(K)
      case K : SET_L6470_PARAM(K); break;
    #endif
    #if AXIS_IS_L64XX(X2)
      case X2: SET_L6470_PARAM(X2); break;
    #endif
    #if AXIS_IS_L64XX(Y2)
      case Y2: SET_L6470_PARAM(Y2); break;
    #endif
    #if AXIS_IS_L64XX(Z2)
      case Z2: SET_L6470_PARAM(Z2); break;
    #endif
    #if AXIS_IS_L64XX(Z3)
      case Z3: SET_L6470_PARAM(Z3); break;
    #endif
    #if AXIS_IS_L64XX(Z4)
      case Z4: SET_L6470_PARAM(Z4); break;
    #endif
    #if AXIS_IS_L64XX(E0)
      case E0: SET_L6470_PARAM(E0); break;
    #endif
    #if AXIS_IS_L64XX(E1)
      case E1: SET_L6470_PARAM(E1); break;
    #endif
    #if AXIS_IS_L64XX(E2)
      case E2: SET_L6470_PARAM(E2); break;
    #endif
    #if AXIS_IS_L64XX(E3)
      case E3: SET_L6470_PARAM(E3); break;
    #endif
    #if AXIS_IS_L64XX(E4)
      case E4: SET_L6470_PARAM(E4); break;
    #endif
    #if AXIS_IS_L64XX(E5)
      case E5: SET_L6470_PARAM(E5); break;
    #endif
    #if AXIS_IS_L64XX(E6)
      case E6: SET_L6470_PARAM(E6); break;
    #endif
    #if AXIS_IS_L64XX(E7)
      case E7: SET_L6470_PARAM(E7); break;
    #endif
  }
}

inline void echo_min_max(const char a, const_float_t min, const_float_t max) {
  DEBUG_CHAR(' '); DEBUG_CHAR(a);
  DEBUG_ECHOLNPAIR(" min = ", min, "  max = ", max);
}
inline void echo_oct_used(const_float_t oct, const uint8_t stall) {
  DEBUG_ECHOPAIR("over_current_threshold used     : ", oct);
  DEBUG_ECHOPGM_P(stall ? PSTR("  (Stall") : PSTR("  (OCD"));
  DEBUG_ECHOLNPGM(" threshold)");
}
inline void err_out_of_bounds() { DEBUG_ECHOLNPGM("Test aborted - motion out of bounds"); }

uint8_t L64XX_Marlin::get_user_input(uint8_t &driver_count, L64XX_axis_t axis_index[3],  char axis_mon[3][3],
                                 float &position_max, float &position_min, float &final_feedrate, uint8_t &kval_hold,
                                 uint8_t over_current_flag, uint8_t &OCD_TH_val, uint8_t &STALL_TH_val, uint16_t &over_current_threshold
) {
  // Return TRUE if the calling routine needs to abort/kill//如果调用例程需要中止/终止，则返回TRUE

  uint16_t displacement = 0;  // " = 0" to eliminate compiler warning//“=0”以消除编译器警告
  uint8_t j;   // general purpose counter//通用计数器

  if (!all_axes_homed()) {
    DEBUG_ECHOLNPGM("Test aborted - home all before running this command");
    return true;
  }

  uint8_t found_displacement = false;
  LOOP_LOGICAL_AXES(i) if (uint16_t _displacement = parser.intval(axis_codes[i])) {
    found_displacement = true;
    displacement = _displacement;
    uint8_t axis_offset = parser.byteval('J');
    axis_mon[0][0] = axis_codes[i];         // Axis first character, one of XYZE//轴第一个字符，XYZE中的一个
    const bool single_or_e = axis_offset >= 2 || axis_mon[0][0] == 'E',
               one_or_more = !single_or_e && axis_offset == 0;
    uint8_t driver_count_local = 0;         // Can't use "driver_count" directly as a subscript because it's passed by reference//无法将“driver\u count”直接用作下标，因为它是通过引用传递的
    if (single_or_e)                        // Single axis, E0, or E1//单轴、E0或E1
      axis_mon[0][1] = axis_offset + '0';   // Index given by 'J' parameter//由“J”参数给出的索引

    if (single_or_e || one_or_more) {
      for (j = 0; j < MAX_L64XX; j++) {     // Count up the drivers on this axis//计算此轴上的驱动程序数
        PGM_P str = (PGM_P)pgm_read_ptr(&index_to_axis[j]); // Get a PGM_P from progmem//从progmem获得PGM\u P
        const char c = pgm_read_byte(str);                  // Get a char from progmem//从progmem获取一个字符
        if (axis_mon[0][0] == c) {          // For each stepper on this axis...//对于此轴上的每个步进电机。。。
          char *mon = axis_mon[driver_count_local];
          *mon++ = c;                        // Copy the 3 letter axis name//复制3个字母的轴名称
          *mon++ = pgm_read_byte(&str[1]);   //  to the axis_mon array//到axis_mon数组
          *mon   = pgm_read_byte(&str[2]);
          axis_index[driver_count_local] = (L64XX_axis_t)j; // And store the L64XX axis index//并存储L64XX轴索引
          driver_count_local++;
        }
      }
      if (one_or_more) driver_count = driver_count_local;
    }
    break; // only take first axis found//只取找到的第一个轴
  }

  if (!found_displacement) {
    DEBUG_ECHOLNPGM("Test aborted - AXIS with displacement is required");
    return true;
  }

  ////
  // Position calcs & checks//定位计算和检查
  ////

  const float LOGICAL_AXIS_LIST(
                E_center = current_position.e,
                X_center = LOGICAL_X_POSITION(current_position.x),
                Y_center = LOGICAL_Y_POSITION(current_position.y),
                Z_center = LOGICAL_Z_POSITION(current_position.z),
                I_center = LOGICAL_I_POSITION(current_position.i),
                J_center = LOGICAL_J_POSITION(current_position.j),
                K_center = LOGICAL_K_POSITION(current_position.k)
              );

  switch (axis_mon[0][0]) {
    default: position_max = position_min = 0; break;

    case 'X': {
      position_min = X_center - displacement;
      position_max = X_center + displacement;
      echo_min_max('X', position_min, position_max);
      if (TERN0(HAS_ENDSTOPS, position_min < (X_MIN_POS) || position_max > (X_MAX_POS))) {
        err_out_of_bounds();
        return true;
      }
    } break;

    #if HAS_Y_AXIS
      case 'Y': {
        position_min = Y_center - displacement;
        position_max = Y_center + displacement;
        echo_min_max('Y', position_min, position_max);
        if (TERN0(HAS_ENDSTOPS, position_min < (Y_MIN_POS) || position_max > (Y_MAX_POS))) {
          err_out_of_bounds();
          return true;
        }
      } break;
    #endif

    #if HAS_Z_AXIS
      case 'Z': {
        position_min = Z_center - displacement;
        position_max = Z_center + displacement;
        echo_min_max('Z', position_min, position_max);
        if (TERN0(HAS_ENDSTOPS, position_min < (Z_MIN_POS) || position_max > (Z_MAX_POS))) {
          err_out_of_bounds();
          return true;
        }
      } break;
    #endif

    #if LINEAR_AXES >= 4
      case AXIS4_NAME: {
        position_min = I_center - displacement;
        position_max = I_center + displacement;
        echo_min_max(AXIS4_NAME, position_min, position_max);
        if (TERN0(HAS_ENDSTOPS, position_min < (I_MIN_POS) || position_max > (I_MAX_POS))) {
          err_out_of_bounds();
          return true;
        }
      } break;
    #endif

    #if LINEAR_AXES >= 5
      case AXIS5_NAME: {
        position_min = J_center - displacement;
        position_max = J_center + displacement;
        echo_min_max(AXIS5_NAME, position_min, position_max);
        if (TERN1(HAS_ENDSTOPS, position_min < (J_MIN_POS) || position_max > (J_MAX_POS))) {
          err_out_of_bounds();
          return true;
        }
      } break;
    #endif

    #if LINEAR_AXES >= 6
      case AXIS6_NAME: {
        position_min = K_center - displacement;
        position_max = K_center + displacement;
        echo_min_max(AXIS6_NAME, position_min, position_max);
        if (TERN2(HAS_ENDSTOPS, position_min < (K_MIN_POS) || position_max > (K_MAX_POS))) {
          err_out_of_bounds();
          return true;
        }
      } break;
    #endif

    #if HAS_EXTRUDERS
      case 'E': {
        position_min = E_center - displacement;
        position_max = E_center + displacement;
        echo_min_max('E', position_min, position_max);
      } break;
    #endif
  }

  ////
  // Work on the drivers//对司机的工作
  ////

  LOOP_L_N(k, driver_count) {
    uint8_t not_found = true;
    for (j = 1; j <= L64XX::chain[0]; j++) {
      PGM_P const str = (PGM_P)pgm_read_ptr(&index_to_axis[L64XX::chain[j]]);
      if (pgm_read_byte(&str[0]) == axis_mon[k][0] && pgm_read_byte(&str[1]) == axis_mon[k][1]) { // See if a L6470 driver//查看是否有L6470驱动程序
        not_found = false;
        break;
      }
    }
    if (not_found) {
      driver_count = k;
      axis_mon[k][0] = ' ';  // mark this entry invalid//将此条目标记为无效
      break;
    }
  }

  if (driver_count == 0) {
    DEBUG_ECHOLNPGM("Test aborted - not a L6470 axis");
    return true;
  }

  DEBUG_ECHOPGM("Monitoring:");
  for (j = 0; j < driver_count; j++) DEBUG_ECHOPAIR("  ", axis_mon[j]);
  DEBUG_EOL();

  // now have a list of driver(s) to monitor//现在有一个要监视的驱动程序列表

  ////
  // TVAL & kVAL_HOLD checks & settings//TVAL和kVAL_保持检查和设置
  ////
  const L64XX_shadow_t &sh = shadow;
  get_status(axis_index[0]); // populate shadow array//填充阴影阵列

  if (sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT) {  // L6474 - use TVAL//L6474-使用TVAL
    uint16_t TVAL_current = parser.ushortval('T');
    if (TVAL_current) {
      uint8_t TVAL_count = (TVAL_current /  sh.AXIS_STALL_CURRENT_CONSTANT_INV) - 1;
      LIMIT(TVAL_count, 0, sh.AXIS_STALL_TH_MAX);
      for (j = 0; j < driver_count; j++)
        set_param(axis_index[j], L6474_TVAL, TVAL_count);
    }
    // only print the tval from one of the drivers//仅打印其中一个驱动程序的tval
    kval_hold = get_param(axis_index[0], L6474_TVAL);
    DEBUG_ECHOLNPAIR("TVAL current (mA) = ", (kval_hold + 1) * sh.AXIS_STALL_CURRENT_CONSTANT_INV);
  }
  else {
    kval_hold = parser.byteval('K');
    if (kval_hold) {
      DEBUG_ECHOLNPAIR("kval_hold = ", kval_hold);
      for (j = 0; j < driver_count; j++)
        set_param(axis_index[j], L6470_KVAL_HOLD, kval_hold);
    }
    else {
      // only print the KVAL_HOLD from one of the drivers//仅打印其中一个驱动程序的KVAL_保持
      kval_hold = get_param(axis_index[0], L6470_KVAL_HOLD);
      DEBUG_ECHOLNPAIR("KVAL_HOLD = ", kval_hold);
    }
  }

  ////
  // Overcurrent checks & settings//过电流检查和设置
  ////

  if (over_current_flag) {

    uint8_t OCD_TH_val_local = 0,       // compiler thinks OCD_TH_val is unused if use it directly//如果直接使用OCD，编译器认为它是未使用的
            STALL_TH_val_local = 0;     // just in case ...//以防万一。。。

    over_current_threshold = parser.intval('I');

    if (over_current_threshold) {

      OCD_TH_val_local = over_current_threshold/375;
      LIMIT(OCD_TH_val_local, 0, 15);
      STALL_TH_val_local = over_current_threshold/31.25;
      LIMIT(STALL_TH_val_local, 0, 127);
      uint16_t OCD_TH_actual = (OCD_TH_val_local + 1) * 375,
               STALL_TH_actual = (STALL_TH_val_local + 1) * 31.25;
      if (OCD_TH_actual < STALL_TH_actual) {
        OCD_TH_val_local++;
        OCD_TH_actual = (OCD_TH_val_local + 1) * 375;
      }

      DEBUG_ECHOLNPAIR("over_current_threshold specified: ", over_current_threshold);
      if (!(sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT)) echo_oct_used((STALL_TH_val_local + 1) * 31.25, true);
      echo_oct_used((OCD_TH_val_local + 1) * 375, false);

      #define SET_OVER_CURRENT(Q) do { stepper##Q.SetParam(L6470_STALL_TH, STALL_TH_val_local); stepper##Q.SetParam(L6470_OCD_TH, OCD_TH_val_local);} while (0)

      for (j = 0; j < driver_count; j++) {
        set_param(axis_index[j], L6470_STALL_TH, STALL_TH_val_local);
        set_param(axis_index[j], L6470_OCD_TH, OCD_TH_val_local);
      }
    }
    else {
      // only get & print the OVER_CURRENT values from one of the drivers//仅从其中一个驱动程序获取并打印当前值
      STALL_TH_val_local = get_param(axis_index[0], L6470_STALL_TH);
      OCD_TH_val_local = get_param(axis_index[0], L6470_OCD_TH);

      if (!(sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT)) echo_oct_used((STALL_TH_val_local + 1) * 31.25, true);
      echo_oct_used((OCD_TH_val_local + 1) * 375, false);
    } // over_current_threshold//过电流阈值

    for (j = 0; j < driver_count; j++) {                 // set all drivers on axis the same//将轴上的所有驱动器设置为相同
      set_param(axis_index[j], L6470_STALL_TH, STALL_TH_val_local);
      set_param(axis_index[j], L6470_OCD_TH, OCD_TH_val_local);
    }

    OCD_TH_val = OCD_TH_val_local;        // force compiler to update the main routine's copy//强制编译器更新主例程的副本
    STALL_TH_val = STALL_TH_val_local;    // force compiler to update the main routine's copy//强制编译器更新主例程的副本
  } // end of overcurrent//过流结束

  ////
  // Feedrate//进给速度
  ////

  final_feedrate = parser.floatval('F');
  if (final_feedrate == 0) {
    static constexpr float default_max_feedrate[] = DEFAULT_MAX_FEEDRATE;
    const uint8_t num_feedrates = COUNT(default_max_feedrate);
    for (j = 0; j < num_feedrates; j++) {
      if (axis_codes[j] == axis_mon[0][0]) {
        final_feedrate = default_max_feedrate[j];
        break;
      }
    }
    if (j == 3 && num_feedrates > 4) {   // have more than one extruder feedrate//有一个以上的挤出机进料速度
      uint8_t extruder_num = axis_mon[0][1] - '0';
      if (j <= num_feedrates - extruder_num)     // have a feedrate specifically for this extruder//有专门针对该挤出机的进给速度
        final_feedrate = default_max_feedrate[j + extruder_num];
      else
        final_feedrate = default_max_feedrate[3];  // use E0 feedrate for this extruder//此挤出机使用E0进料速率
    }
    final_feedrate *= 60;  // convert to mm/minute//换算为毫米/分钟
  } // end of feedrate//喂料结束率

  return false;   // FALSE indicates no user input problems//FALSE表示没有用户输入问题
}

void L64XX_Marlin::say_axis(const L64XX_axis_t axis, const uint8_t label/*=true*/) {
  if (label) SERIAL_ECHOPGM("AXIS:");
  const char * const str = L64xxManager.index_to_axis[axis];
  SERIAL_CHAR(' ', str[0], str[1], ' ');
}

#if ENABLED(L6470_CHITCHAT)

  // Assumes status bits have been inverted//假设状态位已反转
  void L64XX_Marlin::error_status_decode(const uint16_t status, const L64XX_axis_t axis,
        const uint16_t _status_axis_th_sd, const uint16_t _status_axis_th_wrn,
        const uint16_t _status_axis_step_loss_a, const uint16_t _status_axis_step_loss_b,
        const uint16_t _status_axis_ocd, const uint8_t _status_axis_layout
  ) {
    say_axis(axis);
    DEBUG_ECHOPGM("  THERMAL: ");
    DEBUG_ECHOPGM_P((status & _status_axis_th_sd) ? PSTR("SHUTDOWN") : (status & _status_axis_th_wrn) ? PSTR("WARNING ") : PSTR("OK      "));
    DEBUG_ECHOPGM("   OVERCURRENT: ");
    echo_yes_no((status & _status_axis_ocd) != 0);
    if (!(_status_axis_layout == L6474_STATUS_LAYOUT)) {  // L6474 doesn't have these bits//L6474没有这些位
      DEBUG_ECHOPGM("   STALL: ");
      echo_yes_no((status & (_status_axis_step_loss_a | _status_axis_step_loss_b)) != 0);
    }
    DEBUG_EOL();
  }

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////
////  MONITOR_L6470_DRIVER_STATUS routines////监控L6470驱动程序状态例行程序
////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if ENABLED(MONITOR_L6470_DRIVER_STATUS)

  bool L64XX_Marlin::monitor_paused = false; // Flag to skip monitor during M122, M906, M916, M917, M918, etc.//在M122、M906、M916、M917、M918等期间跳过监视器的标志。

  struct L6470_driver_data {
    L64XX_axis_t driver_index;
    uint32_t driver_status;
    uint8_t is_otw;
    uint8_t otw_counter;
    uint8_t is_ot;
    uint8_t is_hi_Z;
    uint8_t com_counter;
  };

  L6470_driver_data driver_L6470_data[] = {
    #if AXIS_IS_L64XX(X)
      {  X, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Y)
      {  Y, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Z)
      {  Z, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(I)
      {  I, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(J)
      {  J, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(K)
      {  K, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(X2)
      { X2, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Y2)
      { Y2, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Z2)
      { Z2, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Z3)
      { Z3, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(Z4)
      { Z4, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E0)
      { E0, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E1)
      { E1, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E2)
      { E2, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E3)
      { E3, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E4)
      { E4, 0, 0, 0, 0, 0, 0 },
    #endif
    #if AXIS_IS_L64XX(E5)
      { E5, 0, 0, 0, 0, 0, 0 }
    #endif
    #if AXIS_IS_L64XX(E6)
      { E6, 0, 0, 0, 0, 0, 0 }
    #endif
    #if AXIS_IS_L64XX(E7)
      { E7, 0, 0, 0, 0, 0, 0 }
    #endif
  };

  void L64XX_Marlin::append_stepper_err(char* &p, const uint8_t stepper_index, const char * const err/*=nullptr*/) {
    PGM_P const str = (PGM_P)pgm_read_ptr(&index_to_axis[stepper_index]);
    p += sprintf_P(p, PSTR("Stepper %c%c "), pgm_read_byte(&str[0]), pgm_read_byte(&str[1]));
    if (err) p += sprintf_P(p, err);
  }

  void L64XX_Marlin::monitor_update(L64XX_axis_t stepper_index) {
    if (spi_abort) return;  // don't do anything if set_directions() has occurred//如果发生set_directions（），请不要执行任何操作
    const L64XX_shadow_t &sh = shadow;
    get_status(stepper_index); // get stepper status and details//获取步进器状态和详细信息
    uint16_t status = sh.STATUS_AXIS;
    uint8_t kval_hold, tval;
    char temp_buf[120], *p = temp_buf;
    uint8_t j;
    for (j = 0; j < L64XX::chain[0]; j++) // find the table for this stepper//找到这个步进机的表
      if (driver_L6470_data[j].driver_index == stepper_index) break;

    driver_L6470_data[j].driver_status = status;
    uint16_t _status = ~status;     // all error bits are active low//所有错误位均为低电平

    if (status == 0 || status == 0xFFFF) {              // com problem//com问题
      if (driver_L6470_data[j].com_counter == 0) {      // warn user when it first happens//第一次发生时警告用户
        driver_L6470_data[j].com_counter++;
        append_stepper_err(p, stepper_index, PSTR(" - communications lost\n"));
        DEBUG_ECHO(temp_buf);
      }
      else {
        driver_L6470_data[j].com_counter++;
        if (driver_L6470_data[j].com_counter > 240) {  // remind of com problem about every 2 minutes//每2分钟提醒一次com问题
          driver_L6470_data[j].com_counter = 1;
          append_stepper_err(p, stepper_index, PSTR(" - still no communications\n"));
          DEBUG_ECHO(temp_buf);
        }
      }
    }
    else {
      if (driver_L6470_data[j].com_counter) {   // comms re-established//重建通讯系统
        driver_L6470_data[j].com_counter = 0;
        append_stepper_err(p, stepper_index, PSTR(" - communications re-established\n.. setting all drivers to default values\n"));
        DEBUG_ECHO(temp_buf);
        init_to_defaults();
      }
      else {
        // no com problems - do the usual checks//没有com问题-执行常规检查
        if (_status & sh.L6470_ERROR_MASK) {
          append_stepper_err(p, stepper_index);

          if (status & STATUS_HIZ) {                         // The driver has shut down. HiZ is active high//司机停车了。HiZ活性高
            driver_L6470_data[j].is_hi_Z = true;
            p += sprintf_P(p, PSTR("%cIS SHUT DOWN"), ' ');
            //if (_status & sh.STATUS_AXIS_TH_SD) { // strange - TH_SD never seems to go active, must be implied by the HiZ and TH_WRN//如果（_status&sh.status_AXIS_TH_SD）{//奇怪-TH_SD似乎从未激活，则必须由HiZ和TH_WRN暗示
            if (_status & sh.STATUS_AXIS_TH_WRN) {                    // over current shutdown//过电流关机
              p += sprintf_P(p, PSTR("%cdue to over temperature"), ' ');
              driver_L6470_data[j].is_ot = true;
              if (sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT) {  // L6474//L6474
                tval = get_param(stepper_index, L6474_TVAL) - 2 * KVAL_HOLD_STEP_DOWN;
              set_param(stepper_index, L6474_TVAL, tval);     // reduce TVAL//减少TVAL
                p += sprintf_P(p, PSTR(" - TVAL reduced by %d to %d mA"), uint16_t (2 * KVAL_HOLD_STEP_DOWN * sh.AXIS_STALL_CURRENT_CONSTANT_INV), uint16_t ((tval + 1) * sh.AXIS_STALL_CURRENT_CONSTANT_INV));   // let user know//让用户知道
              }
              else {
                kval_hold = get_param(stepper_index, L6470_KVAL_HOLD) - 2 * KVAL_HOLD_STEP_DOWN;
                set_param(stepper_index, L6470_KVAL_HOLD, kval_hold);     // reduce KVAL_HOLD//减少KVAL_保持
                p += sprintf_P(p, PSTR(" - KVAL_HOLD reduced by %d to %d"), 2 * KVAL_HOLD_STEP_DOWN, kval_hold);   // let user know//让用户知道
              }
            }
            else
              driver_L6470_data[j].is_ot = false;
          }
          else {
            driver_L6470_data[j].is_hi_Z = false;

            if (_status & sh.STATUS_AXIS_TH_WRN) {     // have an over temperature warning//有超温警告
              driver_L6470_data[j].is_otw = true;
              driver_L6470_data[j].otw_counter++;
              kval_hold = get_param(stepper_index, L6470_KVAL_HOLD);
              if (driver_L6470_data[j].otw_counter > 4) {  // otw present for 2 - 2.5 seconds, reduce KVAL_HOLD//otw存在2-2.5秒，减少KVAL_保持
                driver_L6470_data[j].otw_counter = 0;
                driver_L6470_data[j].is_otw = true;
                if (sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT) {  // L6474//L6474
                  tval = get_param(stepper_index, L6474_TVAL) - KVAL_HOLD_STEP_DOWN;
                  set_param(stepper_index, L6474_TVAL, tval);     // reduce TVAL//减少TVAL
                  p += sprintf_P(p, PSTR(" - TVAL reduced by %d to %d mA"), uint16_t (KVAL_HOLD_STEP_DOWN * sh.AXIS_STALL_CURRENT_CONSTANT_INV), uint16_t ((tval + 1) * sh.AXIS_STALL_CURRENT_CONSTANT_INV));   // let user know//让用户知道
                }
                else {
                  kval_hold = get_param(stepper_index, L6470_KVAL_HOLD) - KVAL_HOLD_STEP_DOWN;
                  set_param(stepper_index, L6470_KVAL_HOLD, kval_hold);     // reduce KVAL_HOLD//减少KVAL_保持
                  p += sprintf_P(p, PSTR(" - KVAL_HOLD reduced by %d to %d"), KVAL_HOLD_STEP_DOWN, kval_hold);   // let user know//让用户知道
                }
              }
              else if (driver_L6470_data[j].otw_counter)
                p += sprintf_P(p, PSTR("%c- thermal warning"), ' ');   // warn user//警告用户
            }
          }

          #if ENABLED(L6470_STOP_ON_ERROR)
            if (_status & (sh.STATUS_AXIS_UVLO | sh.STATUS_AXIS_TH_WRN | sh.STATUS_AXIS_TH_SD))
              kill(temp_buf);
          #endif

          #if ENABLED(L6470_CHITCHAT)
            if (_status & sh.STATUS_AXIS_OCD)
              p += sprintf_P(p, PSTR("%c  over current"), ' ');

            if (_status & (sh.STATUS_AXIS_STEP_LOSS_A | sh.STATUS_AXIS_STEP_LOSS_B))
              p += sprintf_P(p, PSTR("%c  stall"), ' ');

            if (_status & sh.STATUS_AXIS_UVLO)
              p += sprintf_P(p, PSTR("%c  under voltage lock out"), ' ');

            p += sprintf_P(p, PSTR("%c\n"), ' ');
          #endif

          DEBUG_ECHOLN(temp_buf);  // print the error message//打印错误消息
        }
        else {
          driver_L6470_data[j].is_ot = false;
          driver_L6470_data[j].otw_counter = 0;   //clear out warning indicators//清除警告指示灯
          driver_L6470_data[j].is_otw = false;
        } // end usual checks//结束日常检查

      } // comms established but have errors//已建立通信，但有错误
    } // comms re-established//重建通讯系统
  } // end monitor_update()//结束监视器_更新（）


  void L64XX_Marlin::monitor_driver() {
    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 500;

      if (!monitor_paused) {  // Skip during M122, M906, M916, M917 or M918 (could steal status result from test)//在M122、M906、M916、M917或M918期间跳过（可能会从测试中窃取状态结果）

        spi_active = true;    // Tell set_directions() a series of SPI transfers is underway//告诉set_directions（）一系列SPI传输正在进行中

        #if AXIS_IS_L64XX(X)
          monitor_update(X);
        #endif
        #if AXIS_IS_L64XX(Y)
          monitor_update(Y);
        #endif
        #if AXIS_IS_L64XX(Z)
          monitor_update(Z);
        #endif
        #if AXIS_IS_L64XX(I)
          monitor_update(I);
        #endif
        #if AXIS_IS_L64XX(J)
          monitor_update(J);
        #endif
        #if AXIS_IS_L64XX(K)
          monitor_update(K);
        #endif
        #if AXIS_IS_L64XX(X2)
          monitor_update(X2);
        #endif
        #if AXIS_IS_L64XX(Y2)
          monitor_update(Y2);
        #endif
        #if AXIS_IS_L64XX(Z2)
          monitor_update(Z2);
        #endif
        #if AXIS_IS_L64XX(Z3)
          monitor_update(Z3);
        #endif
        #if AXIS_IS_L64XX(Z4)
          monitor_update(Z4);
        #endif
        #if AXIS_IS_L64XX(E0)
          monitor_update(E0);
        #endif
        #if AXIS_IS_L64XX(E1)
          monitor_update(E1);
        #endif
        #if AXIS_IS_L64XX(E2)
          monitor_update(E2);
        #endif
        #if AXIS_IS_L64XX(E3)
          monitor_update(E3);
        #endif
        #if AXIS_IS_L64XX(E4)
          monitor_update(E4);
        #endif
        #if AXIS_IS_L64XX(E5)
          monitor_update(E5);
        #endif
        #if AXIS_IS_L64XX(E6)
          monitor_update(E6);
        #endif
        #if AXIS_IS_L64XX(E7)
          monitor_update(E7);
        #endif

        if (TERN0(L6470_DEBUG, report_L6470_status)) DEBUG_EOL();

        spi_active = false;   // done with all SPI transfers - clear handshake flags//完成所有SPI传输-清除握手标志
        spi_abort = false;
      }
    }
  }

#endif // MONITOR_L6470_DRIVER_STATUS//监控L6470驱动程序的状态

#endif // HAS_L64XX//有"L64XX"吗?
