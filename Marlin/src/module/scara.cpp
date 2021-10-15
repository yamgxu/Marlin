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
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

#if ENABLED(AXEL_TPARA)
  #include "endstops.h"
  #include "../MarlinCore.h"
#endif

float segments_per_second = TERN(AXEL_TPARA, TPARA_SEGMENTS_PER_SECOND, SCARA_SEGMENTS_PER_SECOND);

#if EITHER(MORGAN_SCARA, MP_SCARA)

  static constexpr xy_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y };

  /**
   * Morgan SCARA Forward Kinematics. Results in 'cartes'.
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics(const_float_t a, const_float_t b) {
    const float a_sin = sin(RADIANS(a)) * L1,
                a_cos = cos(RADIANS(a)) * L1,
                b_sin = sin(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2,
                b_cos = cos(RADIANS(SUM_TERN(MP_SCARA, b, a))) * L2;

    cartes.x = a_cos + b_cos + scara_offset.x;  // theta//西塔
    cartes.y = a_sin + b_sin + scara_offset.y;  // phi//phi

    /*
      DEBUG_ECHOLNPAIR(
        "SCARA FK Angle a=", a,
        " b=", b,
        " a_sin=", a_sin,
        " a_cos=", a_cos,
        " b_sin=", b_sin,
        " b_cos=", b_cos
      );
      DEBUG_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
    //*///*/
  }

#endif

#if ENABLED(MORGAN_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MORGAN_SCARA uses a Cartesian XY home position//MORGAN_SCARA使用笛卡尔XY原点
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPAIR_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y);//调试回声对（PSTR（“homeposition X”）、homeposition.X、SP_Y_LBL、homeposition.Y）；

      delta = homeposition;
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPAIR_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);//调试回声对（PSTR（“笛卡尔X”）、当前位置.X、SP\U Y\U LBL、当前位置.Y）；
      update_software_endstops(axis);
    }
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results are stored in 'delta'.
   *
   * See https://reprap.org/forum/read.php?185,283327
   *
   * Maths and first version by QHARLEY.
   * Integrated into Marlin and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const xyz_pos_t &raw) {
    float C2, S2, SK1, SK2, THETA, PSI;

    // Translate SCARA to standard XY with scaling factor//使用比例因子将SCARA转换为标准XY
    const xy_pos_t spos = raw - scara_offset;

    const float H2 = HYPOT2(spos.x, spos.y);
    if (L1 == L2)
      C2 = H2 / L1_2_2 - 1;
    else
      C2 = (H2 - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    LIMIT(C2, -1, 1);

    S2 = SQRT(1.0f - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End//未旋转的臂1加上旋转的臂2给出了从中心到末端的距离
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2//旋转的臂2给出从臂1到臂2的距离
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow//臂1的角度是中心到末端的角度和中心到肘部的角度之差
    THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);

    // Angle of Arm2//臂角2
    PSI = ATAN2(S2, C2);

    delta.set(DEGREES(THETA), DEGREES(SUM_TERN(MORGAN_SCARA, PSI, THETA)), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      DEBUG_ECHOLNPAIR("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Psi=", PSI);
    //*///*/
  }

#elif ENABLED(MP_SCARA)

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      // MP_SCARA uses arm angles for AB home position//MP_SCARA使用手臂角度作为AB起始位置
      #ifndef SCARA_OFFSET_THETA1
        #define SCARA_OFFSET_THETA1  12 // degrees//学位
      #endif
      #ifndef SCARA_OFFSET_THETA2
        #define SCARA_OFFSET_THETA2 131 // degrees//学位
      #endif
      ab_float_t homeposition = { SCARA_OFFSET_THETA1, SCARA_OFFSET_THETA2 };
      //DEBUG_ECHOLNPAIR("homeposition A:", homeposition.a, " B:", homeposition.b);//调试回声对（“homeposition A:，homeposition.A，“B:，homeposition.B”）；

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPAIR_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);//调试回声对（PSTR（“笛卡尔X”）、当前位置.X、SP\U Y\U LBL、当前位置.Y）；
      update_software_endstops(axis);
    }
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const float x = raw.x, y = raw.y, c = HYPOT(x, y),
                THETA3 = ATAN2(y, x),
                THETA1 = THETA3 + ACOS((sq(c) + sq(L1) - sq(L2)) / (2.0f * c * L1)),
                THETA2 = THETA3 - ACOS((sq(c) + sq(L2) - sq(L1)) / (2.0f * c * L2));

    delta.set(DEGREES(THETA1), DEGREES(THETA2), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPAIR("  SCARA (x,y) ", x, ",", y," Theta1=", THETA1, " Theta2=", THETA2);
    //*///*/
  }

#elif ENABLED(AXEL_TPARA)

  static constexpr xyz_pos_t robot_offset = { TPARA_OFFSET_X, TPARA_OFFSET_Y, TPARA_OFFSET_Z };

  void scara_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPAIR_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y, SP_Z_LBL, homeposition.z);//调试对（PSTR（“homeposition X”）、homeposition.X、SP_Y_LBL、homeposition.Y、SP_Z_LBL、homeposition.Z）；

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPAIR_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);//调试回声对（PSTR（“笛卡尔X”）、当前位置.X、SP\U Y\U LBL、当前位置.Y）；
      update_software_endstops(axis);
    }
  }

  // Convert ABC inputs in degrees to XYZ outputs in mm//将以度为单位的ABC输入转换为以毫米为单位的XYZ输出
  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {
    const float w = c - b,
                r = L1 * cos(RADIANS(b)) + L2 * sin(RADIANS(w - (90 - b))),
                x = r  * cos(RADIANS(a)),
                y = r  * sin(RADIANS(a)),
                rho2 = L1_2 + L2_2 - 2.0f * L1 * L2 * cos(RADIANS(w));

    cartes = robot_offset + xyz_pos_t({ x, y, SQRT(rho2 - sq(x) - sq(y)) });
  }

  // Home YZ together, then X (or all at once). Based on quick_home_xy & home_delta//先将YZ放在一起，然后是X（或一次全部放在一起）。基于quick_home_xy和home_delta
  void home_TPARA() {
    // Init the current position of all carriages to 0,0,0//将所有车厢的当前位置初始化为0,0,0
    current_position.reset();
    destination.reset();
    sync_plan_position();

    // Disable stealthChop if used. Enable diag1 pin on driver.//禁用隐形斩波（如果使用）。启用驱动器上的diag1引脚。
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
      TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
      TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
    #endif

    //const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);//const int x_axis_home_dir=工具x_home_dir（主动挤出机）；

    //const xy_pos_t pos { max_length(X_AXIS) , max_length(Y_AXIS) };//常数xy_pos_t pos{max_length（X_轴），max_length（Y_轴）}；
    //const float mlz = max_length(X_AXIS),//常量浮点mlz=最大长度（X轴），

    // Move all carriages together linearly until an endstop is hit.//将所有车厢一起线性移动，直到碰到终点止动块。
    //do_blocking_move_to_xy_z(pos, mlz, homing_feedrate(Z_AXIS));//是否将阻塞移动到xy（位置、mlz、归位进给率（z轴））；

    current_position.x = 0 ;
    current_position.y = 0 ;
    current_position.z = max_length(Z_AXIS) ;
    line_to_current_position(homing_feedrate(Z_AXIS));
    planner.synchronize();

    // Re-enable stealthChop if used. Disable diag1 pin on driver.//重新启用隐身斩波（如果使用）。禁用驱动器上的diag1引脚。
    #if ENABLED(SENSORLESS_HOMING)
      TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
      TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
      TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
    #endif

    endstops.validate_homing_move();

    // At least one motor has reached its endstop.//至少有一个电机已达到其末端止动位置。
    // Now re-home each motor separately.//现在，请分别将每个电机重新安装回原位。
    homeaxis(A_AXIS);
    homeaxis(C_AXIS);
    homeaxis(B_AXIS);

    // Set all carriages to their home positions//将所有车厢设置到其原始位置
    // Do this here all at once for Delta, because//为Delta一次完成这一切，因为
    // XYZ isn't ABC. Applying this per-tower would//XYZ不是ABC。在每座塔上应用这一点
    // give the impression that they are the same.//给人的印象是他们是一样的。
    LOOP_LINEAR_AXES(i) set_axis_is_at_home((AxisEnum)i);

    sync_plan_position();
  }

  void inverse_kinematics(const xyz_pos_t &raw) {
    const xyz_pos_t spos = raw - robot_offset;

    const float RXY = SQRT(HYPOT2(spos.x, spos.y)),
                RHO2 = NORMSQ(spos.x, spos.y, spos.z),
                //RHO = SQRT(RHO2),//RHO=SQRT（RHO2），
                LSS = L1_2 + L2_2,
                LM = 2.0f * L1 * L2,

                CG = (LSS - RHO2) / LM,
                SG = SQRT(1 - POW(CG, 2)), // Method 2//方法2
                K1 = L1 - L2 * CG,
                K2 = L2 * SG,

                // Angle of Body Joint//身体关节角
                THETA = ATAN2(spos.y, spos.x),

                // Angle of Elbow Joint//肘关节角
                //GAMMA = ACOS(CG),//伽马=ACOS（CG），
                GAMMA = ATAN2(SG, CG), // Method 2//方法2

                // Angle of Shoulder Joint, elevation angle measured from horizontal (r+)//肩关节角度，从水平面测量的仰角（r+）
                //PHI = asin(spos.z/RHO) + asin(L2 * sin(GAMMA) / RHO),//φ=asin（spos.z/RHO）+asin（L2*sin（GAMMA）/RHO），
                PHI = ATAN2(spos.z, RXY) + ATAN2(K2, K1),   // Method 2//方法2

                // Elbow motor angle measured from horizontal, same frame as shoulder  (r+)//从水平方向测量的肘关节电机角度，与肩部相同（r+）
                PSI = PHI + GAMMA;

    delta.set(DEGREES(THETA), DEGREES(PHI), DEGREES(PSI));

    //SERIAL_ECHOLNPAIR(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);//序列回波对（“SCARA（x，y，z）”，spos.x，，，spos.y，，，，spos.z，“Rho=”，Rho，“Rho2=”，Rho2，“Theta=”，Theta，“Phi=”，Phi，“Psi=”，Psi，“Gamma=”，Gamma）；
  }

#endif

void scara_report_positions() {
  SERIAL_ECHOLNPAIR("SCARA Theta:", planner.get_axis_position_degrees(A_AXIS)
    #if ENABLED(AXEL_TPARA)
      , "  Phi:", planner.get_axis_position_degrees(B_AXIS)
      , "  Psi:", planner.get_axis_position_degrees(C_AXIS)
    #else
      , "  Psi" TERN_(MORGAN_SCARA, "+Theta") ":", planner.get_axis_position_degrees(B_AXIS)
    #endif
  );
  SERIAL_EOL();
}

#endif // IS_SCARA//你是斯卡拉吗
