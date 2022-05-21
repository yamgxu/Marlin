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

/**
 * Configuration.h
 *
 * Basic settings such as:
 *
 * - Type of electronics
 * - Type of temperature sensor
 * - Printer geometry
 * - Endstop configuration
 * - LCD controller
 * - Extra features
 *
 * Advanced settings can be found in Configuration_adv.h
 */
#define CONFIGURATION_H_VERSION 02000901

//===========================================================================//===========================================================================
//============================= Getting Started =============================//=========================================================开始使用=============================
//===========================================================================//===========================================================================

/**
 * Here are some useful links to help get your machine configured and calibrated:
 *
 * Example Configs:     https://github.com/MarlinFirmware/Configurations/branches/all
 *
 * Průša Calculator:    https://blog.prusaprinters.org/calculator_3416/
 *
 * Calibration Guides:  https://reprap.org/wiki/Calibration
 *                      https://reprap.org/wiki/Triffid_Hunter%27s_Calibration_Guide
 *                      https://sites.google.com/site/repraplogphase/calibration-of-your-reprap
 *                      https://youtu.be/wAL9d7FgInk
 *
 * Calibration Objects: https://www.thingiverse.com/thing:5573
 *                      https://www.thingiverse.com/thing:1278865
 */

//===========================================================================//===========================================================================
//========================== DELTA / SCARA / TPARA ==========================//========================================三角洲/圣甲虫/塔帕拉==========================
//===========================================================================//===========================================================================
////
// Download configurations from the link above and customize for your machine.//从上面的链接下载配置并为您的机器进行自定义。
// Examples are located in config/examples/delta, .../SCARA, and .../TPARA.//示例位于config/Examples/delta、…/SCARA和…/TPARA中。
////
//===========================================================================//===========================================================================

// @section info//@section info

// Author info of this build printed to the host during boot and M115//此版本的作者信息在引导和M115期间打印到主机
#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.//谁做的改变。
//#define CUSTOM_VERSION_FILE Version.h // Path from the root directory (no quotes)//#从根目录定义自定义\u版本\u文件VERSION.h//路径（无引号）

/**
 * *** VENDORS PLEASE READ ***
 *
 * Marlin allows you to add a custom boot image for Graphical LCDs.
 * With this option Marlin will first show your custom screen followed
 * by the standard Marlin logo with version number and web URL.
 *
 * We encourage you to take advantage of this new feature and we also
 * respectfully request that you retain the unmodified Marlin boot screen.
 */

// Show the Marlin bootscreen on startup. ** ENABLE FOR PRODUCTION **//启动时显示Marlin引导屏幕。**使能生产**
#define SHOW_BOOTSCREEN

// Show the bitmap in Marlin/_Bootscreen.h on startup.//启动时在Marlin/_Bootscreen.h中显示位图。
//#define SHOW_CUSTOM_BOOTSCREEN//#定义显示\自定义\引导屏幕

// Show the bitmap in Marlin/_Statusscreen.h on the status screen.//在状态屏幕上以Marlin/_Statusscreen.h显示位图。
//#define CUSTOM_STATUS_SCREEN_IMAGE//#定义自定义\u状态\u屏幕\u图像

// @section machine//型材机

/**
 * Select the serial port on the board to use for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 * Serial port -1 is the USB emulated serial port, if available.
 * Note: The first serial port (-1 or 0) will always be used by the Arduino bootloader.
 *
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
#define SERIAL_PORT 0

/**
 * Serial Port Baud Rate
 * This is the default communication speed for all serial ports.
 * Set the baud rate defaults for additional serial ports below.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 * You may try up to 1000000 to speed up SD file transfer.
 *
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 115200
//#define BAUD_RATE_GCODE     // Enable G-code M575 to set the baud rate//#定义波特率\u GCODE//启用G代码M575以设置波特率

/**
 * Select a secondary serial port on the board to use for communication with the host.
 * Currently Ethernet (-2) is only supported on Teensy 4.1 boards.
 * :[-2, -1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
//#define SERIAL_PORT_2 -1//#定义串行端口2-1
//#define BAUDRATE_2 250000   // Enable to override BAUDRATE//#定义波特率\u 2 250000//启用以覆盖波特率

/**
 * Select a third serial port on the board to use for communication with the host.
 * Currently only supported for AVR, DUE, LPC1768/9 and STM32/STM32F1
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7]
 */
//#define SERIAL_PORT_3 1//#定义串行_端口_3 1
//#define BAUDRATE_3 250000   // Enable to override BAUDRATE//#定义波特率\u 3 250000//启用以覆盖波特率

// Enable the Bluetooth serial interface on AT90USB devices//在AT90USB设备上启用蓝牙串行接口
//#define BLUETOOTH//#定义蓝牙

// Choose the name from boards.h that matches your setup//从boards.h中选择与您的设置匹配的名称
#define MOTHERBOARD BOARD_ESP32CONTROLLERR3
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMPS_14_EFB
#endif

// Name displayed in the LCD "Ready" message and Info menu//LCD“就绪”信息和信息菜单中显示的名称
//#define CUSTOM_MACHINE_NAME "3D Printer"//#定义自定义机器名称“3D打印机”

// Printer's unique ID, used by some programs to differentiate between machines.//打印机的唯一ID，某些程序用来区分机器。
// Choose your own or use a service like https://www.uuidgenerator.net/version4//选择您自己的或使用类似的服务https://www.uuidgenerator.net/version4
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"//#定义机器ID“00000000-0000-0000-0000-000000000000”

/**
 * Define the number of coordinated linear axes.
 * See https://github.com/DerAndere1/Marlin/wiki
 * Each linear axis gets its own stepper control and endstop:
 *
 *   Steppers: *_STEP_PIN, *_ENABLE_PIN, *_DIR_PIN, *_ENABLE_ON
 *   Endstops: *_STOP_PIN, USE_*MIN_PLUG, USE_*MAX_PLUG
 *       Axes: *_MIN_POS, *_MAX_POS, INVERT_*_DIR
 *    Planner: DEFAULT_AXIS_STEPS_PER_UNIT, DEFAULT_MAX_FEEDRATE
 *             DEFAULT_MAX_ACCELERATION, AXIS_RELATIVE_MODES,
 *             MICROSTEP_MODES, MANUAL_FEEDRATE
 *
 * :[3, 4, 5, 6]
 */
//#define LINEAR_AXES 3//#定义线性_轴3

/**
 * Axis codes for additional axes:
 * This defines the axis code that is used in G-code commands to
 * reference a specific axis.
 * 'A' for rotational axis parallel to X
 * 'B' for rotational axis parallel to Y
 * 'C' for rotational axis parallel to Z
 * 'U' for secondary linear axis parallel to X
 * 'V' for secondary linear axis parallel to Y
 * 'W' for secondary linear axis parallel to Z
 * Regardless of the settings, firmware-internal axis IDs are
 * I (AXIS4), J (AXIS5), K (AXIS6).
 */
#if LINEAR_AXES >= 4
#define AXIS4_NAME 'A' // :['A', 'B', 'C', 'U', 'V', 'W']//：['A'，'B'，'C'，'U'，'V'，'W']
#endif
#if LINEAR_AXES >= 5
#define AXIS5_NAME 'B' // :['A', 'B', 'C', 'U', 'V', 'W']//：['A'，'B'，'C'，'U'，'V'，'W']
#endif
#if LINEAR_AXES >= 6
#define AXIS6_NAME 'C' // :['A', 'B', 'C', 'U', 'V', 'W']//：['A'，'B'，'C'，'U'，'V'，'W']
#endif

// @section extruder//型材挤出机

// This defines the number of extruders//这定义了挤出机的数量
// :[0, 1, 2, 3, 4, 5, 6, 7, 8]// :[0, 1, 2, 3, 4, 5, 6, 7, 8]
#define EXTRUDERS 1

// Generally expected filament diameter (1.75, 2.85, 3.0, ...). Used for Volumetric, Filament Width Sensor, etc.//一般预期灯丝直径（1.75、2.85、3.0等）。用于体积、灯丝宽度传感器等。
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75

// For Cyclops or any "multi-extruder" that shares a single nozzle.//用于独眼巨人或共用一个喷嘴的任何“多挤出机”。
//#define SINGLENOZZLE//#定义单喷嘴

// Save and restore temperature and fan speed on tool-change.//更换刀具时保存并恢复温度和风扇转速。
// Set standby for the unselected tool with M104/106/109 T...//使用M104/106/109 T为未选择的刀具设置备用。。。
#if ENABLED(SINGLENOZZLE)
//#define SINGLENOZZLE_STANDBY_TEMP//#定义单喷嘴\u备用\u温度
  //#define SINGLENOZZLE_STANDBY_FAN//#定义单喷嘴\u备用\u风扇
#endif

/**
 * Multi-Material Unit
 * Set to one of these predefined models:
 *
 *   PRUSA_MMU1           : Průša MMU1 (The "multiplexer" version)
 *   PRUSA_MMU2           : Průša MMU2
 *   PRUSA_MMU2S          : Průša MMU2S (Requires MK3S extruder with motion sensor, EXTRUDERS = 5)
 *   EXTENDABLE_EMU_MMU2  : MMU with configurable number of filaments (ERCF, SMuFF or similar with Průša MMU2 compatible firmware)
 *   EXTENDABLE_EMU_MMU2S : MMUS with configurable number of filaments (ERCF, SMuFF or similar with Průša MMU2 compatible firmware)
 *
 * Requires NOZZLE_PARK_FEATURE to park print head in case MMU unit fails.
 * See additional options in Configuration_adv.h.
 */
//#define MMU_MODEL PRUSA_MMU2//#定义MMU_模型PRUSA_MMU2

// A dual extruder that uses a single stepper motor//使用单个步进电机的双挤出机
//#define SWITCHING_EXTRUDER//#挤出机开关的定义
#if ENABLED(SWITCHING_EXTRUDER)
#define SWITCHING_EXTRUDER_SERVO_NR 0
  #define SWITCHING_EXTRUDER_SERVO_ANGLES { 0, 90 } // Angles for E0, E1[, E2, E3]//E0、E1[、E2、E3]的角度
  #if EXTRUDERS > 3
    #define SWITCHING_EXTRUDER_E23_SERVO_NR 1
  #endif
#endif

// A dual-nozzle that uses a servomotor to raise/lower one (or both) of the nozzles//使用伺服电机升高/降低一个（或两个）喷嘴的双喷嘴
//#define SWITCHING_NOZZLE//#定义切换喷嘴
#if ENABLED(SWITCHING_NOZZLE)
#define SWITCHING_NOZZLE_SERVO_NR 0
  //#define SWITCHING_NOZZLE_E1_SERVO_NR 1          // If two servos are used, the index of the second//#定义切换喷嘴E1伺服1//如果使用两个伺服，则第二个伺服的索引
  #define SWITCHING_NOZZLE_SERVO_ANGLES { 0, 90 }   // Angles for E0, E1 (single servo) or lowered/raised (dual servo)//E0、E1（单伺服）或降低/升高（双伺服）的角度
#endif

/**
 * Two separate X-carriages with extruders that connect to a moving part
 * via a solenoid docking mechanism. Requires SOL1_PIN and SOL2_PIN.
 */
//#define PARKING_EXTRUDER//#挤出机的定义

/**
 * Two separate X-carriages with extruders that connect to a moving part
 * via a magnetic docking mechanism using movements and no solenoid
 *
 * project   : https://www.thingiverse.com/thing:3080893
 * movements : https://youtu.be/0xCEiG9VS3k
 *             https://youtu.be/Bqbcs0CU2FE
 */
//#define MAGNETIC_PARKING_EXTRUDER//#定义磁性挤出机

#if EITHER(PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER)

#define PARKING_EXTRUDER_PARKING_X { -78, 184 }     // X positions for parking the extruders//用于停放挤出机的X个位置
  #define PARKING_EXTRUDER_GRAB_DISTANCE 1            // (mm) Distance to move beyond the parking point to grab the extruder//（mm）超出停车点抓取挤出机的距离
  //#define MANUAL_SOLENOID_CONTROL                   // Manual control of docking solenoids with M380 S / M381//#定义手动电磁阀控制//使用M380 S/M381手动控制对接电磁阀

  #if ENABLED(PARKING_EXTRUDER)

    #define PARKING_EXTRUDER_SOLENOIDS_INVERT           // If enabled, the solenoid is NOT magnetized with applied voltage//如果启用，电磁阀不会被施加的电压磁化
    #define PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE LOW  // LOW or HIGH pin signal energizes the coil//低或高引脚信号使线圈通电
    #define PARKING_EXTRUDER_SOLENOIDS_DELAY 250        // (ms) Delay for magnetic field. No delay if 0 or not defined.//（ms）磁场延迟。如果0或未定义，则无延迟。
    //#define MANUAL_SOLENOID_CONTROL                   // Manual control of docking solenoids with M380 S / M381//#定义手动电磁阀控制//使用M380 S/M381手动控制对接电磁阀

  #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)

    #define MPE_FAST_SPEED      9000      // (mm/min) Speed for travel before last distance point//最后一个距离点之前的行驶速度（mm/min）
    #define MPE_SLOW_SPEED      4500      // (mm/min) Speed for last distance travel to park and couple//（mm/min）最后一次行驶至停车场和停车场的速度
    #define MPE_TRAVEL_DISTANCE   10      // (mm) Last distance point//（mm）最后一个距离点
    #define MPE_COMPENSATION       0      // Offset Compensation -1 , 0 , 1 (multiplier) only for coupling//偏移补偿-仅用于耦合的1,0,1（倍增器）

  #endif

#endif

/**
 * Switching Toolhead
 *
 * Support for swappable and dockable toolheads, such as
 * the E3D Tool Changer. Toolheads are locked with a servo.
 */
//#define SWITCHING_TOOLHEAD//#定义切换工具头

/**
 * Magnetic Switching Toolhead
 *
 * Support swappable and dockable toolheads with a magnetic
 * docking mechanism using movement and no servo.
 */
//#define MAGNETIC_SWITCHING_TOOLHEAD//#定义磁性开关工具头

/**
 * Electromagnetic Switching Toolhead
 *
 * Parking for CoreXY / HBot kinematics.
 * Toolheads are parked at one edge and held with an electromagnet.
 * Supports more than 2 Toolheads. See https://youtu.be/JolbsAKTKf4
 */
//#define ELECTROMAGNETIC_SWITCHING_TOOLHEAD//#定义电磁开关工具头

#if ANY(SWITCHING_TOOLHEAD, MAGNETIC_SWITCHING_TOOLHEAD, ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
#define SWITCHING_TOOLHEAD_Y_POS          235         // (mm) Y position of the toolhead dock//（mm）工具头底座的Y位置
  #define SWITCHING_TOOLHEAD_Y_SECURITY      10         // (mm) Security distance Y axis//（mm）安全距离Y轴
  #define SWITCHING_TOOLHEAD_Y_CLEAR         60         // (mm) Minimum distance from dock for unobstructed X axis//（mm）无障碍X轴与码头的最小距离
  #define SWITCHING_TOOLHEAD_X_POS          { 215, 0 }  // (mm) X positions for parking the extruders//（mm）用于停放挤出机的X个位置
  #if ENABLED(SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_SERVO_NR       2         // Index of the servo connector//伺服连接器的索引
    #define SWITCHING_TOOLHEAD_SERVO_ANGLES { 0, 180 }  // (degrees) Angles for Lock, Unlock//锁定、解锁的角度（度）
  #elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Y_RELEASE      5         // (mm) Security distance Y axis//（mm）安全距离Y轴
    #define SWITCHING_TOOLHEAD_X_SECURITY   { 90, 150 } // (mm) Security distance X axis (T0,T1)//（mm）安全距离X轴（T0，T1）
    //#define PRIME_BEFORE_REMOVE                       // Prime the nozzle before release from the dock//#在卸下/卸下喷嘴前，定义注油器。//从船坞释放前，给喷嘴注油
    #if ENABLED(PRIME_BEFORE_REMOVE)
      #define SWITCHING_TOOLHEAD_PRIME_MM           20  // (mm)   Extruder prime length//（mm）挤出机基本长度
      #define SWITCHING_TOOLHEAD_RETRACT_MM         10  // (mm)   Retract after priming length//（mm）打底长度后缩回
      #define SWITCHING_TOOLHEAD_PRIME_FEEDRATE    300  // (mm/min) Extruder prime feedrate//（mm/min）挤出机基本进给速度
      #define SWITCHING_TOOLHEAD_RETRACT_FEEDRATE 2400  // (mm/min) Extruder retract feedrate//（mm/min）挤出机回缩进给速度
    #endif
  #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Z_HOP          2         // (mm) Z raise for switching//（mm）开关的Z升高
  #endif
#endif

/**
 * "Mixing Extruder"
 *   - Adds G-codes M163 and M164 to set and "commit" the current mix factors.
 *   - Extends the stepping routines to move multiple steppers in proportion to the mix.
 *   - Optional support for Repetier Firmware's 'M164 S<index>' supporting virtual tools.
 *   - This implementation supports up to two mixing extruders.
 *   - Enable DIRECT_MIXING_IN_G1 for M165 and mixing in G1 (from Pia Taubert's reference implementation).
 */
//#define MIXING_EXTRUDER//#定义混炼挤出机
#if ENABLED(MIXING_EXTRUDER)
#define MIXING_STEPPERS 2        // Number of steppers in your mixing extruder//混合挤出机中的步进机数量
  #define MIXING_VIRTUAL_TOOLS 16  // Use the Virtual Tool method with M163 and M164//将虚拟工具方法用于M163和M164
  //#define DIRECT_MIXING_IN_G1    // Allow ABCDHI mix factors in G1 movement commands//#在G1中定义直接混合//在G1移动命令中允许ABCDHI混合因子
  //#define GRADIENT_MIX           // Support for gradient mixing with M166 and LCD//#定义渐变_MIX//支持M166和LCD的渐变混合
  //#define MIXING_PRESETS         // Assign 8 default V-tool presets for 2 or 3 MIXING_STEPPERS//#定义混合预设//为2或3个混合步进机指定8个默认V形刀具预设
  #if ENABLED(GRADIENT_MIX)
    //#define GRADIENT_VTOOL       // Add M166 T to use a V-tool index as a Gradient alias//#定义渐变_VTOOL//Add M166 T以使用V形工具索引作为渐变别名
  #endif
#endif

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).//挤出机的偏移量（如果使用多个挤出机并在更换时依靠固件定位，则取消注释）。
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).//挤出机0热端（默认挤出机）的偏移量必须为X=0，Y=0。
// For the other hotends it is their distance from the extruder 0 hotend.//对于其他热端，其与挤出机的距离为0热端。
//#define HOTEND_OFFSET_X { 0.0, 20.00 } // (mm) relative X-offset for each nozzle//#为每个喷嘴定义热端偏移量（X{0.0,20.00}//（mm）相对X偏移量
//#define HOTEND_OFFSET_Y { 0.0, 5.00 }  // (mm) relative Y-offset for each nozzle//#定义每个喷嘴的热端偏移量Y{0.0,5.00}//（mm）相对Y偏移量
//#define HOTEND_OFFSET_Z { 0.0, 0.00 }  // (mm) relative Z-offset for each nozzle//#为每个喷嘴定义热端偏移量_Z{0.0,0.00}//（mm）相对Z偏移量

// @section machine//型材机

/**
 * Power Supply Control
 *
 * Enable and connect the power supply to the PS_ON_PIN.
 * Specify whether the power supply is active HIGH or active LOW.
 */
//#define PSU_CONTROL//#定义PSU\U控制
//#define PSU_NAME "Power Supply"//#定义PSU_名称“电源”

#if ENABLED(PSU_CONTROL)
//#define MKS_PWC                 // Using the MKS PWC add-on//#使用MKS PWC附加组件定义MKS_PWC//
  //#define PS_OFF_CONFIRM          // Confirm dialog when power off//#定义PS_OFF_CONFIRM//关机时确认对话框
  //#define PS_OFF_SOUND            // Beep 1s when power off//#定义PS_OFF_声音//关机时发出嘟嘟声1s
  #define PSU_ACTIVE_STATE LOW      // Set 'LOW' for ATX, 'HIGH' for X-Box//ATX设置为“低”，X-Box设置为“高”

  //#define PSU_DEFAULT_OFF         // Keep power off until enabled directly with M80//#定义PSU_DEFAULT_OFF//关闭电源，直到直接使用M80启用
  //#define PSU_POWERUP_DELAY 250   // (ms) Delay for the PSU to warm up to full power//#定义PSU\u通电\u延迟250//（ms）延迟，以使PSU预热至满功率

  //#define PSU_POWERUP_GCODE  "M355 S1"  // G-code to run after power-on (e.g., case light on)//#定义PSU\U通电\U GCODE“M355 S1”//G代码，以便在通电后运行（例如，机箱指示灯亮起）
  //#define PSU_POWEROFF_GCODE "M355 S0"  // G-code to run before power-off (e.g., case light off)//#定义PSU_POWEROFF_GCODE“M355 S0”//G代码，以便在断电前运行（例如，机箱指示灯关闭）

  //#define AUTO_POWER_CONTROL      // Enable automatic control of the PS_ON pin//#定义自动电源控制//启用引脚上PS\U的自动控制
  #if ENABLED(AUTO_POWER_CONTROL)
    #define AUTO_POWER_FANS         // Turn on PSU if fans need power//如果风扇需要电源，请打开PSU
    #define AUTO_POWER_E_FANS
    #define AUTO_POWER_CONTROLLERFAN
    #define AUTO_POWER_CHAMBER_FAN
    #define AUTO_POWER_COOLER_FAN
    //#define AUTO_POWER_E_TEMP        50 // (°C) Turn on PSU if any extruder is over this temperature//#如果任何挤出机的温度超过此温度，则定义PSU上的自动电源温度为50/（°C）
    //#define AUTO_POWER_CHAMBER_TEMP  30 // (°C) Turn on PSU if the chamber is over this temperature//#如果腔室温度超过此温度，定义自动电源腔室温度30/（°C）打开PSU
    //#define AUTO_POWER_COOLER_TEMP   26 // (°C) Turn on PSU if the cooler is over this temperature//#如果冷却器温度超过此温度，定义自动电源冷却器温度26/（°C）开启PSU
    #define POWER_TIMEOUT              30 // (s) Turn off power if the machine is idle for this duration//（s）如果机器在此期间处于空闲状态，则关闭电源
    //#define POWER_OFF_DELAY          60 // (s) Delay of poweroff after M81 command. Useful to let fans run for extra time.//#定义M81命令后的断电延时60/（s）。有助于让球迷多跑一段时间。
  #endif
#endif

//===========================================================================//===========================================================================
//============================= Thermal Settings ============================//=================================================热设置============================
//===========================================================================//===========================================================================
// @section temperature//@截面温度

/**
 * --NORMAL IS 4.7kΩ PULLUP!-- 1kΩ pullup can be used on hotend sensor, using correct resistor and table
 *
 * Temperature sensors available:
 *
 *  SPI RTD/Thermocouple Boards - IMPORTANT: Read the NOTE below!
 *  -------
 *    -5 : MAX31865 with Pt100/Pt1000, 2, 3, or 4-wire  (only for sensors 0-1)
 *                  NOTE: You must uncomment/set the MAX31865_*_OHMS_n defines below.
 *    -3 : MAX31855 with Thermocouple, -200°C to +700°C (only for sensors 0-1)
 *    -2 : MAX6675  with Thermocouple, 0°C to +700°C    (only for sensors 0-1)
 *
 *  NOTE: Ensure TEMP_n_CS_PIN is set in your pins file for each TEMP_SENSOR_n using an SPI Thermocouple. By default,
 *        Hardware SPI on the default serial bus is used. If you have also set TEMP_n_SCK_PIN and TEMP_n_MISO_PIN,
 *        Software SPI will be used on those ports instead. You can force Hardware SPI on the default bus in the
 *        Configuration_adv.h file. At this time, separate Hardware SPI buses for sensors are not supported.
 *
 *  Analog Themocouple Boards
 *  -------
 *    -4 : AD8495 with Thermocouple
 *    -1 : AD595  with Thermocouple
 *
 *  Analog Thermistors - 4.7kΩ pullup - Normal
 *  -------
 *     1 : 100kΩ  EPCOS - Best choice for EPCOS thermistors
 *   331 : 100kΩ  Same as #1, but 3.3V scaled for MEGA
 *   332 : 100kΩ  Same as #1, but 3.3V scaled for DUE
 *     2 : 200kΩ  ATC Semitec 204GT-2
 *   202 : 200kΩ  Copymaster 3D
 *     3 : ???Ω   Mendel-parts thermistor
 *     4 : 10kΩ   Generic Thermistor !! DO NOT use for a hotend - it gives bad resolution at high temp. !!
 *     5 : 100kΩ  ATC Semitec 104GT-2/104NT-4-R025H42G - Used in ParCan, J-Head, and E3D, SliceEngineering 300°C
 *   501 : 100kΩ  Zonestar - Tronxy X3A
 *   502 : 100kΩ  Zonestar - used by hot bed in Zonestar Průša P802M
 *   512 : 100kΩ  RPW-Ultra hotend
 *     6 : 100kΩ  EPCOS - Not as accurate as table #1 (created using a fluke thermocouple)
 *     7 : 100kΩ  Honeywell 135-104LAG-J01
 *    71 : 100kΩ  Honeywell 135-104LAF-J01
 *     8 : 100kΩ  Vishay 0603 SMD NTCS0603E3104FXT
 *     9 : 100kΩ  GE Sensing AL03006-58.2K-97-G1
 *    10 : 100kΩ  RS PRO 198-961
 *    11 : 100kΩ  Keenovo AC silicone mats, most Wanhao i3 machines - beta 3950, 1%
 *    12 : 100kΩ  Vishay 0603 SMD NTCS0603E3104FXT (#8) - calibrated for Makibox hot bed
 *    13 : 100kΩ  Hisens up to 300°C - for "Simple ONE" & "All In ONE" hotend - beta 3950, 1%
 *    15 : 100kΩ  Calibrated for JGAurora A5 hotend
 *    18 : 200kΩ  ATC Semitec 204GT-2 Dagoma.Fr - MKS_Base_DKU001327
 *    22 : 100kΩ  GTM32 Pro vB - hotend - 4.7kΩ pullup to 3.3V and 220Ω to analog input
 *    23 : 100kΩ  GTM32 Pro vB - bed - 4.7kΩ pullup to 3.3v and 220Ω to analog input
 *    30 : 100kΩ  Kis3d Silicone heating mat 200W/300W with 6mm precision cast plate (EN AW 5083) NTC100K - beta 3950
 *    60 : 100kΩ  Maker's Tool Works Kapton Bed Thermistor - beta 3950
 *    61 : 100kΩ  Formbot/Vivedino 350°C Thermistor - beta 3950
 *    66 : 4.7MΩ  Dyze Design High Temperature Thermistor
 *    67 : 500kΩ  SliceEngineering 450°C Thermistor
 *    70 : 100kΩ  bq Hephestos 2
 *    75 : 100kΩ  Generic Silicon Heat Pad with NTC100K MGB18-104F39050L32
 *  2000 : 100kΩ  Ultimachine Rambo TDK NTCG104LH104KT1 NTC100K motherboard Thermistor
 *
 *  Analog Thermistors - 1kΩ pullup - Atypical, and requires changing out the 4.7kΩ pullup for 1kΩ.
 *  -------                           (but gives greater accuracy and more stable PID)
 *    51 : 100kΩ  EPCOS (1kΩ pullup)
 *    52 : 200kΩ  ATC Semitec 204GT-2 (1kΩ pullup)
 *    55 : 100kΩ  ATC Semitec 104GT-2 - Used in ParCan & J-Head (1kΩ pullup)
 *
 *  Analog Thermistors - 10kΩ pullup - Atypical
 *  -------
 *    99 : 100kΩ  Found on some Wanhao i3 machines with a 10kΩ pull-up resistor
 *
 *  Analog RTDs (Pt100/Pt1000)
 *  -------
 *   110 : Pt100  with 1kΩ pullup (atypical)
 *   147 : Pt100  with 4.7kΩ pullup
 *  1010 : Pt1000 with 1kΩ pullup (atypical)
 *  1047 : Pt1000 with 4.7kΩ pullup (E3D)
 *    20 : Pt100  with circuit in the Ultimainboard V2.x with mainboard ADC reference voltage = INA826 amplifier-board supply voltage.
 *                NOTE: (1) Must use an ADC input with no pullup. (2) Some INA826 amplifiers are unreliable at 3.3V so consider using sensor 147, 110, or 21.
 *    21 : Pt100  with circuit in the Ultimainboard V2.x with 3.3v ADC reference voltage (STM32, LPC176x....) and 5V INA826 amplifier board supply.
 *                NOTE: ADC pins are not 5V tolerant. Not recommended because it's possible to damage the CPU by going over 500°C.
 *   201 : Pt100  with circuit in Overlord, similar to Ultimainboard V2.x
 *
 *  Custom/Dummy/Other Thermal Sensors
 *  ------
 *     0 : not used
 *  1000 : Custom - Specify parameters in Configuration_adv.h
 *
 *   !!! Use these for Testing or Development purposes. NEVER for production machine. !!!
 *   998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
 *   999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.
 *
 */
#define TEMP_SENSOR_0 331
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_5 0
#define TEMP_SENSOR_6 0
#define TEMP_SENSOR_7 0
#define TEMP_SENSOR_BED 331
#define TEMP_SENSOR_PROBE 0
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_BOARD 0
#define TEMP_SENSOR_REDUNDANT 0

// Dummy thermistor constant temperature readings, for use with 998 and 999//模拟热敏电阻恒温读数，用于998和999
#define DUMMY_THERMISTOR_998_VALUE  25
#define DUMMY_THERMISTOR_999_VALUE 100

// Resistor values when using MAX31865 sensors (-5) on TEMP_SENSOR_0 / 1//在温度传感器0/1上使用MAX31865传感器（-5）时的电阻值
//#define MAX31865_SENSOR_OHMS_0      100   // (Ω) Typically 100 or 1000 (PT100 or PT1000)//#定义MAX31865_传感器_欧姆_0 100/（Ω）通常为100或1000（PT100或PT1000）
//#define MAX31865_CALIBRATION_OHMS_0 430   // (Ω) Typically 430 for Adafruit PT100; 4300 for Adafruit PT1000//#定义MAX31865_校准_欧姆_0 430/（Ω），Adafruit PT100通常为430；Adafruit PT1000为4300
//#define MAX31865_SENSOR_OHMS_1      100//#定义MAX31865_传感器_欧姆_1 100
//#define MAX31865_CALIBRATION_OHMS_1 430//#定义MAX31865_校准_欧姆_1430

#define TEMP_RESIDENCY_TIME         10  // (seconds) Time to wait for hotend to "settle" in M109//（秒）在M109中等待热端“稳定”的时间
#define TEMP_WINDOW                  1  // (°C) Temperature proximity for the "temperature reached" timer//（°C）“达到温度”计时器的温度接近度
#define TEMP_HYSTERESIS              3  // (°C) Temperature proximity considered "close enough" to the target//（°C）温度接近被认为与目标“足够接近”

#define TEMP_BED_RESIDENCY_TIME     10  // (seconds) Time to wait for bed to "settle" in M190//（秒）等待床在M190中“稳定”的时间
#define TEMP_BED_WINDOW              1  // (°C) Temperature proximity for the "temperature reached" timer//（°C）“达到温度”计时器的温度接近度
#define TEMP_BED_HYSTERESIS          3  // (°C) Temperature proximity considered "close enough" to the target//（°C）温度接近被认为与目标“足够接近”

#define TEMP_CHAMBER_RESIDENCY_TIME 10  // (seconds) Time to wait for chamber to "settle" in M191//（秒）在M191中等待腔室“稳定”的时间
#define TEMP_CHAMBER_WINDOW          1  // (°C) Temperature proximity for the "temperature reached" timer//（°C）“达到温度”计时器的温度接近度
#define TEMP_CHAMBER_HYSTERESIS      3  // (°C) Temperature proximity considered "close enough" to the target//（°C）温度接近被认为与目标“足够接近”

/**
 * Redundant Temperature Sensor (TEMP_SENSOR_REDUNDANT)
 *
 * Use a temp sensor as a redundant sensor for another reading. Select an unused temperature sensor, and another
 * sensor you'd like it to be redundant for. If the two thermistors differ by TEMP_SENSOR_REDUNDANT_MAX_DIFF (°C),
 * the print will be aborted. Whichever sensor is selected will have its normal functions disabled; i.e. selecting
 * the Bed sensor (-1) will disable bed heating/monitoring.
 *
 * For selecting source/target use: COOLER, PROBE, BOARD, CHAMBER, BED, E0, E1, E2, E3, E4, E5, E6, E7
 */
#if TEMP_SENSOR_REDUNDANT
#define TEMP_SENSOR_REDUNDANT_SOURCE    E1  // The sensor that will provide the redundant reading.//提供冗余读数的传感器。
  #define TEMP_SENSOR_REDUNDANT_TARGET    E0  // The sensor that we are providing a redundant reading for.//我们为其提供冗余读数的传感器。
  #define TEMP_SENSOR_REDUNDANT_MAX_DIFF  10  // (°C) Temperature difference that will trigger a print abort.//（°C）会触发打印中止的温差。
#endif

// Below this temperature the heater will be switched off//低于此温度时，加热器将关闭
// because it probably indicates a broken thermistor wire.//因为它可能表示热敏电阻线断了。
#define HEATER_0_MINTEMP   5
#define HEATER_1_MINTEMP   5
#define HEATER_2_MINTEMP   5
#define HEATER_3_MINTEMP   5
#define HEATER_4_MINTEMP   5
#define HEATER_5_MINTEMP   5
#define HEATER_6_MINTEMP   5
#define HEATER_7_MINTEMP   5
#define BED_MINTEMP        5
#define CHAMBER_MINTEMP    5

// Above this temperature the heater will be switched off.//高于此温度时，加热器将关闭。
// This can protect components from overheating, but NOT from shorts and failures.//这可以防止组件过热，但不能防止短路和故障。
// (Use MINTEMP for thermistor short/failure protection.)//（使用MINTEMP进行热敏电阻短路/故障保护。）
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP      150
#define CHAMBER_MAXTEMP  60

/**
 * Thermal Overshoot
 * During heatup (and printing) the temperature can often "overshoot" the target by many degrees
 * (especially before PID tuning). Setting the target temperature too close to MAXTEMP guarantees
 * a MAXTEMP shutdown! Use these values to forbid temperatures being set too close to MAXTEMP.
 */
#define HOTEND_OVERSHOOT 15   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT//（°C）禁止温度超过最大温度-超调
#define BED_OVERSHOOT    10   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT//（°C）禁止温度超过最大温度-超调
#define COOLER_OVERSHOOT  2   // (°C) Forbid temperatures closer than OVERSHOOT//（°C）禁止温度接近超调

//===========================================================================//===========================================================================
//============================= PID Settings ================================//==============================================PID设置================================
//===========================================================================//===========================================================================
// PID Tuning Guide here: https://reprap.org/wiki/PID_Tuning//PID调整指南如下：https://reprap.org/wiki/PID_Tuning

// Comment the following line to disable PID and enable bang-bang.//注释以下行以禁用PID并启用bang bang。
#define PIDTEMP
#define BANG_MAX 255     // Limits current to nozzle while in bang-bang mode; 255=full current//在“砰砰”模式下限制喷嘴的电流；255=全电流
#define PID_MAX BANG_MAX // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current//PID激活时限制喷嘴电流（见下面的PID功能范围）；255=全电流
#define PID_K1 0.95      // Smoothing factor within any PID loop//任何PID回路中的平滑因子

#if ENABLED(PIDTEMP)
//#define PID_EDIT_MENU         // Add PID editing to the "Advanced Settings" menu. (~700 bytes of PROGMEM)//#定义PID编辑菜单//将PID编辑添加到“高级设置”菜单。（~700字节的程序）
  //#define PID_AUTOTUNE_MENU     // Add PID auto-tuning to the "Advanced Settings" menu. (~250 bytes of PROGMEM)//#定义PID自动调谐菜单//将PID自动调谐添加到“高级设置”菜单。（~250字节的程序）
  //#define PID_PARAMS_PER_HOTEND // Uses separate PID parameters for each extruder (useful for mismatched extruders)//#定义PID_参数每_热端//为每个挤出机使用单独的PID参数（适用于不匹配的挤出机）
                                  // Set/get with gcode: M301 E[extruder number, 0-2]//设置/获取gcode:M301 E[挤出机编号，0-2]

  #if ENABLED(PID_PARAMS_PER_HOTEND)
    // Specify up to one value per hotend here, according to your setup.//根据您的设置，此处每个热端最多指定一个值。
    // If there are fewer values, the last one applies to the remaining hotends.//如果值较少，则最后一个值将应用于剩余的热端。
    #define DEFAULT_Kp_LIST {  22.20,  22.20 }
    #define DEFAULT_Ki_LIST {   1.08,   1.08 }
    #define DEFAULT_Kd_LIST { 114.00, 114.00 }
  #else
    #define DEFAULT_Kp 23.94
    #define DEFAULT_Ki 1.53
    #define DEFAULT_Kd 93.88
  #endif
#endif // PIDTEMP//皮特姆

//===========================================================================//===========================================================================
//====================== PID > Bed Temperature Control ======================//============================PID>床温控制======================
//===========================================================================//===========================================================================

/**
 * PID Bed Heating
 *
 * If this option is enabled set PID constants below.
 * If this option is disabled, bang-bang will be used and BED_LIMIT_SWITCHING will enable hysteresis.
 *
 * The PID frequency will be the same as the extruder PWM.
 * If PID_dT is the default, and correct for the hardware/configuration, that means 7.689Hz,
 * which is fine for driving a square wave into a resistive load and does not significantly
 * impact FET heating. This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W
 * heater. If your configuration is significantly different than this and you don't understand
 * the issues involved, don't use bed PID until someone else verifies that your hardware works.
 */
//#define PIDTEMPBED//#定义临时床

//#define BED_LIMIT_SWITCHING//#定义床位限制开关

/**
 * Max Bed Power
 * Applies to all forms of bed control (PID, bang-bang, and bang-bang with hysteresis).
 * When set to any value below 255, enables a form of PWM to the bed that acts like a divider
 * so don't use it unless you are OK with PWM on your bed. (See the comment on enabling PIDTEMPBED)
 */
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current//将工作循环限制在床上；255=全电流

#if ENABLED(PIDTEMPBED)
//#define MIN_BED_POWER 0//#定义最小床功率0
  //#define PID_BED_DEBUG // Sends debug data to the serial port.//#定义PID_BED_DEBUG//将调试数据发送到串行端口。

  // 120V 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)//120V 250W硅加热器，插入4mm硼硅酸盐（MendelMax 1.5+）
  // from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)//根据FOPDT模型-kp=0.39 Tp=405 Tdead=66，Tc设置为79.2，侵蚀因子为0.15（vs.1,1,10）
  #define DEFAULT_bedKp 10.00
  #define DEFAULT_bedKi .023
  #define DEFAULT_bedKd 305.4

  // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.//找到您自己的：“M303 E-1 C8 S90”在床上以90度ESC运行自动调谐8个周期。
#endif // PIDTEMPBED//皮坦普德

//===========================================================================//===========================================================================
//==================== PID > Chamber Temperature Control ====================//========================PID>腔室温度控制====================
//===========================================================================//===========================================================================

/**
 * PID Chamber Heating
 *
 * If this option is enabled set PID constants below.
 * If this option is disabled, bang-bang will be used and CHAMBER_LIMIT_SWITCHING will enable
 * hysteresis.
 *
 * The PID frequency will be the same as the extruder PWM.
 * If PID_dT is the default, and correct for the hardware/configuration, that means 7.689Hz,
 * which is fine for driving a square wave into a resistive load and does not significantly
 * impact FET heating. This also works fine on a Fotek SSR-10DA Solid State Relay into a 200W
 * heater. If your configuration is significantly different than this and you don't understand
 * the issues involved, don't use chamber PID until someone else verifies that your hardware works.
 */
//#define PIDTEMPCHAMBER//#定义临时会议室
//#define CHAMBER_LIMIT_SWITCHING//#定义腔室限制开关

/**
 * Max Chamber Power
 * Applies to all forms of chamber control (PID, bang-bang, and bang-bang with hysteresis).
 * When set to any value below 255, enables a form of PWM to the chamber heater that acts like a divider
 * so don't use it unless you are OK with PWM on your heater. (See the comment on enabling PIDTEMPCHAMBER)
 */
#define MAX_CHAMBER_POWER 255 // limits duty cycle to chamber heater; 255=full current//限制腔室加热器的占空比；255=全电流

#if ENABLED(PIDTEMPCHAMBER)
#define MIN_CHAMBER_POWER 0
  //#define PID_CHAMBER_DEBUG // Sends debug data to the serial port.//#define PID_CHAMBER_DEBUG//将调试数据发送到串行端口。

  // Lasko "MyHeat Personal Heater" (200w) modified with a Fotek SSR-10DA to control only the heating element//Lasko“MyHeat个人加热器”（200w）采用Fotek SSR-10DA进行改进，仅控制加热元件
  // and placed inside the small Creality printer enclosure tent.//并放置在小的Creal打印机外壳帐篷内。
  ////
  #define DEFAULT_chamberKp 37.04
  #define DEFAULT_chamberKi 1.40
  #define DEFAULT_chamberKd 655.17
  // M309 P37.04 I1.04 D655.17//M309 P37.04 I1.04 D655.17

  // FIND YOUR OWN: "M303 E-2 C8 S50" to run autotune on the chamber at 50 degreesC for 8 cycles.//找到您自己的：“M303 E-2 C8 S50”，在50摄氏度的温度下在腔室上运行自动调谐8个周期。
#endif // PIDTEMPCHAMBER//皮坦普尔酒店

#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
//#define PID_DEBUG             // Sends debug data to the serial port. Use 'M303 D' to toggle activation.//#定义PID_DEBUG//将调试数据发送到串行端口。使用“M303 D”切换激活。
  //#define PID_OPENLOOP          // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX//#定义PID_开环//将PID置于开环中。M104/M140将输出功率从0设置为PID_最大值
  //#define SLOW_PWM_HEATERS      // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay//#定义慢_PWM_加热器//PWM频率非常低（约0.125Hz=8s），最短状态时间约1s，对继电器驱动的加热器有用
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature//如果目标温度和实际温度之间的温差
                                  // is more than PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.//超过PID_功能_范围，则PID将关闭，加热器将设置为最小/最大。
#endif

// @section extruder//型材挤出机

/**
 * Prevent extrusion if the temperature is below EXTRUDE_MINTEMP.
 * Add M302 to set the minimum extrusion temperature and/or turn
 * cold extrusion prevention on and off.
 *
 * *** IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED! ***
 */
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

/**
 * Prevent a single extrusion longer than EXTRUDE_MAXLENGTH.
 * Note: For Bowden Extruders make this large enough to allow load/unload.
 */
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

//===========================================================================//===========================================================================
//======================== Thermal Runaway Protection =======================//=====================================热失控保护=======================
//===========================================================================//===========================================================================

/**
 * Thermal Protection provides additional protection to your printer from damage
 * and fire. Marlin always includes safe min and max temperature ranges which
 * protect against a broken or disconnected thermistor wire.
 *
 * The issue: If a thermistor falls out, it will report the much lower
 * temperature of the air in the room, and the the firmware will keep
 * the heater on.
 *
 * If you get "Thermal Runaway" or "Heating failed" errors the
 * details can be tuned in Configuration_adv.h
 */

#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders//为所有挤出机启用热保护
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed//启用加热床的热保护
#define THERMAL_PROTECTION_CHAMBER // Enable thermal protection for the heated chamber//启用加热室的热保护
#define THERMAL_PROTECTION_COOLER  // Enable thermal protection for the laser cooling//为激光器冷却启用热保护

//===========================================================================//===========================================================================
//============================= Mechanical Settings =========================//==============================================机械设置=========================
//===========================================================================//===========================================================================

// @section machine//型材机

// Enable one of the options below for CoreXY, CoreXZ, or CoreYZ kinematics,//为CoreXY、CoreXZ或CoreYZ运动学启用以下选项之一，
// either in the usual order or reversed//要么按通常的顺序，要么按相反的顺序
#define COREXY//#定义COREXY
//#define COREXZ//#定义COREXZ
//#define COREYZ//#定义COREYZ
//#define COREYX//#定义COREYX
//#define COREZX//#定义COREZX
//#define COREZY//#定义COREZY
//#define MARKFORGED_XY  // MarkForged. See https://reprap.org/forum/read.php?152,504042//#定义MARKFORGED\u XY//MARKFORGED。请参阅https://reprap.org/forum/read.php?152,504042

// Enable for a belt style printer with endless "Z" motion//启用具有无休止“Z”运动的皮带式打印机
//#define BELTPRINTER//#定义带式输送机

//===========================================================================//===========================================================================
//============================== Endstop Settings ===========================//====================================================结束停止设置===========================
//===========================================================================//===========================================================================

// @section homing//@段归位

// Specify here all the endstop connectors that are connected to any endstop or probe.//在此处指定连接到任何endstop或probe的所有endstop连接器。
// Almost all printers will be using one per axis. Probes will use one or more of the//几乎所有的打印机都将使用一个轴。探测器将使用一个或多个
// extra connectors. Leave undefined any used for non-endstop and non-probe purposes.//额外的连接器。保留未定义的任何用于非终止和非探测目的。
#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG
//#define USE_IMIN_PLUG//#定义使用IMIN插头
//#define USE_JMIN_PLUG//#定义使用插件
//#define USE_KMIN_PLUG//#定义使用插件
//#define USE_XMAX_PLUG//#定义USE_XMAX_插头
//#define USE_YMAX_PLUG//#定义使用_YMAX_插头
//#define USE_ZMAX_PLUG//#定义使用_ZMAX_插头
//#define USE_IMAX_PLUG//#定义使用IMAX插头
//#define USE_JMAX_PLUG//#定义使用_JMAX_插头
//#define USE_KMAX_PLUG//#定义使用\u KMAX\u插头

// Enable pullup for all endstops to prevent a floating state//为所有止动器启用上拉，以防止浮动状态
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
// Disable ENDSTOPPULLUPS to set pullups individually//禁用ENDSTOPPULLUPS以单独设置上拉
  //#define ENDSTOPPULLUP_XMAX//#定义ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX//#定义ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX//#定义EndStoppullupzmax
  //#define ENDSTOPPULLUP_IMAX//#定义ENDSTOPPULLUP_IMAX
  //#define ENDSTOPPULLUP_JMAX//#定义ENDSTOPPULLUP_JMAX
  //#define ENDSTOPPULLUP_KMAX//#定义ENDSTOPPULLUP_KMAX
  //#define ENDSTOPPULLUP_XMIN//#定义ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN//#定义ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN//#定义EndStoppullupzmin
  //#define ENDSTOPPULLUP_IMIN//#定义ENDSTOPPULLUP\u IMIN
  //#define ENDSTOPPULLUP_JMIN//#定义EndStoppullupjmin
  //#define ENDSTOPPULLUP_KMIN//#定义ENDSTOPPULLUP_KMIN
  //#define ENDSTOPPULLUP_ZMIN_PROBE//#定义ENDSTOPPULLUP_ZMIN_探针
#endif

// Enable pulldown for all endstops to prevent a floating state//为所有端点停止启用下拉，以防止浮动状态
//#define ENDSTOPPULLDOWNS//#定义ENDSTOPPULLDOWNS
#if DISABLED(ENDSTOPPULLDOWNS)
// Disable ENDSTOPPULLDOWNS to set pulldowns individually//禁用ENDSTOPPULLDOWNS以单独设置下拉列表
  //#define ENDSTOPPULLDOWN_XMAX//#定义ENDSTOPPULLDOWN_XMAX
  //#define ENDSTOPPULLDOWN_YMAX//#定义ENDSTOPPULLDOWN_YMAX
  //#define ENDSTOPPULLDOWN_ZMAX//#定义ENDSTOPPULLDOWN_ZMAX
  //#define ENDSTOPPULLDOWN_IMAX//#定义ENDSTOPPULLDOWN\u IMAX
  //#define ENDSTOPPULLDOWN_JMAX//#定义ENDSTOPPULLDOWN_JMAX
  //#define ENDSTOPPULLDOWN_KMAX//#定义ENDSTOPPULLDOWN_KMAX
  //#define ENDSTOPPULLDOWN_XMIN//#定义ENDSTOPPULLDOWN_XMIN
  //#define ENDSTOPPULLDOWN_YMIN//#定义ENDSTOPPULLDOWN_YMIN
  //#define ENDSTOPPULLDOWN_ZMIN//#定义EndStoppullDownzmin
  //#define ENDSTOPPULLDOWN_IMIN//#定义ENDSTOPPULLDOWN\u IMIN
  //#define ENDSTOPPULLDOWN_JMIN//#定义ENDSTOPPULLDOWN_JMIN
  //#define ENDSTOPPULLDOWN_KMIN//#定义ENDSTOPPULLDOWN\u KMIN
  //#define ENDSTOPPULLDOWN_ZMIN_PROBE//#定义ENDSTOPPULLDOWN\u ZMIN\u探针
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).//COM接地和NC接信号的机械止动器在此处使用“假”（最常见的设置）。
#define X_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define Y_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define Z_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define I_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define J_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define K_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define X_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define Y_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define Z_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define I_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define J_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define K_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.//设置为true可反转endstop的逻辑。
#define Z_MIN_PROBE_ENDSTOP_INVERTING false // Set to true to invert the logic of the probe.//设置为true可反转探头的逻辑。

/**
 * Stepper Drivers
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced options for
 * stepper drivers that support them. You may also override timing options in Configuration_adv.h.
 *
 * A4988 is assumed for unspecified drivers.
 *
 * Use TMC2208/TMC2208_STANDALONE for TMC2225 drivers and TMC2209/TMC2209_STANDALONE for TMC2226 drivers.
 *
 * Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
 *          TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'L6470', 'L6474', 'POWERSTEP01', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160', 'TMC5160_STANDALONE']
 */
#define X_DRIVER_TYPE  A4988
#define Y_DRIVER_TYPE  A4988
#define Z_DRIVER_TYPE  A4988
//#define X2_DRIVER_TYPE A4988//#定义X2_驱动器类型A4988
//#define Y2_DRIVER_TYPE A4988//#定义Y2_驱动器类型A4988
//#define Z2_DRIVER_TYPE A4988//#定义Z2_驱动器类型A4988
//#define Z3_DRIVER_TYPE A4988//#定义Z3_驱动器类型A4988
//#define Z4_DRIVER_TYPE A4988//#定义A4988型Z4_驱动器
//#define I_DRIVER_TYPE  A4988//#定义I_驱动器类型A4988
//#define J_DRIVER_TYPE  A4988//#定义J_驱动器类型A4988
//#define K_DRIVER_TYPE  A4988//#定义K_驱动器类型A4988
//#define E0_DRIVER_TYPE A4988//#定义E0_驱动器类型A4988
//#define E1_DRIVER_TYPE A4988//#定义E1_驱动器类型A4988
//#define E2_DRIVER_TYPE A4988//#定义E2驱动程序类型A4988
//#define E3_DRIVER_TYPE A4988//#定义E3驱动程序类型A4988
//#define E4_DRIVER_TYPE A4988//#定义E4_驱动程序类型A4988
//#define E5_DRIVER_TYPE A4988//#定义E5_驱动器类型A4988
//#define E6_DRIVER_TYPE A4988//#定义E6_驱动器类型A4988
//#define E7_DRIVER_TYPE A4988//#定义E7驱动程序类型A4988

// Enable this feature if all enabled endstop pins are interrupt-capable.//如果所有启用的endstop引脚都具有中断功能，则启用此功能。
// This will remove the need to poll the interrupt pins, saving many CPU cycles.//这将消除轮询中断引脚的需要，节省许多CPU周期。
//#define ENDSTOP_INTERRUPTS_FEATURE//#定义终止中断功能

/**
 * Endstop Noise Threshold
 *
 * Enable if your probe or endstops falsely trigger due to noise.
 *
 * - Higher values may affect repeatability or accuracy of some bed probes.
 * - To fix noise install a 100nF ceramic capacitor in parallel with the switch.
 * - This feature is not required for common micro-switches mounted on PCBs
 *   based on the Makerbot design, which already have the 100nF capacitor.
 *
 * :[2,3,4,5,6,7]
 */
//#define ENDSTOP_NOISE_THRESHOLD 2//#定义结束停止\u噪波\u阈值2

// Check for stuck or disconnected endstops during homing moves.//在归位移动过程中，检查是否有卡滞或断开的止动块。
//#define DETECT_BROKEN_ENDSTOP//#定义检测\断开\结束停止

//=============================================================================//=============================================================================
//============================== Movement Settings ============================//==============================================移动设置============================
//=============================================================================//=============================================================================
// @section motion//@节动议

/**
 * Default Settings
 *
 * These settings can be reset by M502
 *
 * Note that if EEPROM is enabled, saved values will override these.
 */

/**
 * With this option each E stepper can have its own factors for the
 * following movement settings. If fewer factors are given than the
 * total number of extruders, the last value applies to the rest.
 */
//#define DISTINCT_E_FACTORS//#定义不同的因素

/**
 * Default Axis Steps Per Unit (steps/mm)
 * Override with M92
 *                                      X, Y, Z [, I [, J [, K]]], E0 [, E1[, E2...]]//
 */
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 8*5, 8*5, 16*100,  199.333}
//16*200*3/(8*3.1415926535)
/**
 * Default Max Feed Rate (mm/s)
 * Override with M203
 *                                      X, Y, Z [, I [, J [, K]]], E0 [, E1[, E2...]]
 */
#define DEFAULT_MAX_FEEDRATE          { 300, 300, 5, 25 }

//#define LIMITED_MAX_FR_EDITING        // Limit edit via M203 or LCD to DEFAULT_MAX_FEEDRATE * 2//#定义有限的最大进给量编辑//通过M203或LCD将编辑限制为默认的最大进给量*2
#if ENABLED(LIMITED_MAX_FR_EDITING)
#define MAX_FEEDRATE_EDIT_VALUES    { 600, 600, 10, 50 } // ...or, set your own edit limits//…或者，设置自己的编辑限制
#endif

/**
 * Default Max Acceleration (change/s) change = mm/s
 * (Maximum start speed for accelerated moves)
 * Override with M201
 *                                      X, Y, Z [, I [, J [, K]]], E0 [, E1[, E2...]]
 */
#define DEFAULT_MAX_ACCELERATION      { 3000, 3000, 100, 10000 }

//#define LIMITED_MAX_ACCEL_EDITING     // Limit edit via M201 or LCD to DEFAULT_MAX_ACCELERATION * 2//#定义有限的最大加速度编辑//通过M201或LCD将编辑限制为默认的最大加速度*2
#if ENABLED(LIMITED_MAX_ACCEL_EDITING)
#define MAX_ACCEL_EDIT_VALUES       { 6000, 6000, 200, 20000 } // ...or, set your own edit limits//…或者，设置自己的编辑限制
#endif

/**
 * Default Acceleration (change/s) change = mm/s
 * Override with M204
 *
 *   M204 P    Acceleration
 *   M204 R    Retract Acceleration
 *   M204 T    Travel Acceleration
 */
#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E acceleration for printing moves//用于打印移动的X、Y、Z和E加速度
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration for retracts//E收缩加速度
#define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration for travel (non printing) moves//移动（非打印）时的X、Y、Z加速度

/**
 * Default Jerk limits (mm/s)
 * Override with M205 X Y Z E
 *
 * "Jerk" specifies the minimum speed change that requires acceleration.
 * When changing speed and direction, if the difference is less than the
 * value set here, it may happen instantaneously.
 */
//#define CLASSIC_JERK//#定义经典的“急动”
#if ENABLED(CLASSIC_JERK)
#define DEFAULT_XJERK 10.0
  #define DEFAULT_YJERK 10.0
  #define DEFAULT_ZJERK  0.3
  //#define DEFAULT_IJERK  0.3//#定义默认值_ijerk0.3
  //#define DEFAULT_JJERK  0.3//#定义默认值_JJERK 0.3
  //#define DEFAULT_KJERK  0.3//#定义默认值_KJERK 0.3

  //#define TRAVEL_EXTRA_XYJERK 0.0     // Additional jerk allowance for all travel moves//#定义行程额外脉动0.0//所有行程移动的额外脉动余量

  //#define LIMITED_JERK_EDITING        // Limit edit via M205 or LCD to DEFAULT_aJERK * 2//#定义有限的震动编辑//通过M205或LCD将编辑限制为默认值*2
  #if ENABLED(LIMITED_JERK_EDITING)
    #define MAX_JERK_EDIT_VALUES { 20, 20, 0.6, 10 } // ...or, set your own edit limits//…或者，设置自己的编辑限制
  #endif
#endif

#define DEFAULT_EJERK    5.0  // May be used by Linear Advance//可由线性推进使用

/**
 * Junction Deviation Factor
 *
 * See:
 *   https://reprap.org/forum/read.php?1,739819
 *   https://blog.kyneticcnc.com/2018/10/computing-junction-deviation-for-marlin.html
 */
#if DISABLED(CLASSIC_JERK)
#define JUNCTION_DEVIATION_MM 0.013 // (mm) Distance from real junction edge//（mm）与实际连接边缘的距离
  #define JD_HANDLE_SMALL_SEGMENTS    // Use curvature estimation instead of just the junction angle//使用曲率估计，而不仅仅是连接角度
                                      // for small segments (< 1mm) with large junction angles (> 135°).//对于具有大连接角（>135°）的小段（<1mm）。
#endif

/**
 * S-Curve Acceleration
 *
 * This option eliminates vibration during printing by fitting a Bézier
 * curve to move acceleration, producing much smoother direction changes.
 *
 * See https://github.com/synthetos/TinyG/wiki/Jerk-Controlled-Motion-Explained
 */
//#define S_CURVE_ACCELERATION//#定义S_曲线_加速度

//===========================================================================//===========================================================================
//============================= Z Probe Options =============================//=================================================Z探头选项=============================
//===========================================================================//===========================================================================
// @section probes//@剖面探测

////
// See https://marlinfw.org/docs/configuration/probes.html//看https://marlinfw.org/docs/configuration/probes.html
////

/**
 * Enable this option for a probe connected to the Z-MIN pin.
 * The probe replaces the Z-MIN endstop and is used for Z homing.
 * (Automatically enables USE_PROBE_FOR_Z_HOMING.)
 */
#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN

// Force the use of the probe for Z-axis homing//强制使用探针进行Z轴归位
//#define USE_PROBE_FOR_Z_HOMING//#定义使用探头进行自导

/**
 * Z_MIN_PROBE_PIN
 *
 * Define this pin if the probe is not connected to Z_MIN_PIN.
 * If not defined the default pin for the selected MOTHERBOARD
 * will be used. Most of the time the default is what you want.
 *
 *  - The simplest option is to use a free endstop connector.
 *  - Use 5V for powered (usually inductive) sensors.
 *
 *  - RAMPS 1.3/1.4 boards may use the 5V, GND, and Aux4->D32 pin:
 *    - For simple switches connect...
 *      - normally-closed switches to GND and D32.
 *      - normally-open switches to 5V and D32.
 */
//#define Z_MIN_PROBE_PIN 32 // Pin 32 is the RAMPS default//#定义Z_MIN_PROBE_引脚32//引脚32是斜坡默认值

/**
 * Probe Type
 *
 * Allen Key Probes, Servo Probes, Z-Sled Probes, FIX_MOUNTED_PROBE, etc.
 * Activate one of these to use Auto Bed Leveling below.
 */

/**
 * The "Manual Probe" provides a means to do "Auto" Bed Leveling without a probe.
 * Use G29 repeatedly, adjusting the Z height at each point with movement commands
 * or (with LCD_BED_LEVELING) the LCD controller.
 */
//#define PROBE_MANUALLY//#手动定义探测单元

/**
 * A Fix-Mounted Probe either doesn't deploy or needs manual deployment.
 *   (e.g., an inductive probe or a nozzle-based probe-switch.)
 */
#define FIX_MOUNTED_PROBE//#定义固定式探头

/**
 * Use the nozzle as the probe, as with a conductive
 * nozzle system or a piezo-electric smart effector.
 */
//#define NOZZLE_AS_PROBE//#将喷嘴_定义为_探头

/**
 * Z Servo Probe, such as an endstop switch on a rotating arm.
 */
//#define Z_PROBE_SERVO_NR 0       // Defaults to SERVO 0 connector.//#定义Z_探针_伺服_nR0//默认为伺服0连接器。
//#define Z_SERVO_ANGLES { 70, 0 } // Z Servo Deploy and Stow angles//#定义Z_伺服角度{70，0}//Z伺服展开和收起角度

/**
 * The BLTouch probe uses a Hall effect sensor and emulates a servo.
 */
//#define BLTOUCH//#定义BLTOUCH

/**
 * Touch-MI Probe by hotends.fr
 *
 * This probe is deployed and activated by moving the X-axis to a magnet at the edge of the bed.
 * By default, the magnet is assumed to be on the left and activated by a home. If the magnet is
 * on the right, enable and set TOUCH_MI_DEPLOY_XPOS to the deploy position.
 *
 * Also requires: BABYSTEPPING, BABYSTEP_ZPROBE_OFFSET, Z_SAFE_HOMING,
 *                and a minimum Z_HOMING_HEIGHT of 10.
 */
//#define TOUCH_MI_PROBE//#定义触摸式探针
#if ENABLED(TOUCH_MI_PROBE)
#define TOUCH_MI_RETRACT_Z 0.5                  // Height at which the probe retracts//探头缩回的高度
  //#define TOUCH_MI_DEPLOY_XPOS (X_MAX_BED + 2)  // For a magnet on the right side of the bed//#为床右侧的磁铁定义TOUCH\u MI\u DEPLOY\u XPOS（X\u MAX\u BED+2）//按钮
  //#define TOUCH_MI_MANUAL_DEPLOY                // For manual deploy (LCD menu)//#为手动部署（LCD菜单）定义触摸式手动部署
#endif

// A probe that is deployed and stowed with a solenoid pin (SOL1_PIN)//使用电磁阀销（SOL1_销）展开和收起的探头
//#define SOLENOID_PROBE//#定义螺线管探头

// A sled-mounted probe like those designed by Charles Bell.//类似查尔斯·贝尔设计的雪橇式探头。
//#define Z_PROBE_SLED//#定义Z_探测器底座
//#define SLED_DOCKING_OFFSET 5  // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.//#定义底座对接偏移5//X轴必须移动以拾取底座的额外距离。0应该可以，但如果愿意，可以进一步推动它。

// A probe deployed by moving the x-axis, such as the Wilson II's rack-and-pinion probe designed by Marty Rice.//通过移动x轴部署的探针，如由Marty Rice设计的Wilson II的齿条和齿轮探针。
//#define RACK_AND_PINION_PROBE//#定义齿条和小齿轮探针
#if ENABLED(RACK_AND_PINION_PROBE)
#define Z_PROBE_DEPLOY_X  X_MIN_POS
  #define Z_PROBE_RETRACT_X X_MAX_POS
#endif

// Duet Smart Effector (for delta printers) - https://bit.ly/2ul5U7J//Duet智能效应器（用于delta打印机）-https://bit.ly/2ul5U7J
// When the pin is defined you can use M672 to set/reset the probe sensitivity.//定义引脚后，可以使用M672设置/重置探头灵敏度。
//#define DUET_SMART_EFFECTOR//#定义DUET_智能效应器
#if ENABLED(DUET_SMART_EFFECTOR)
#define SMART_EFFECTOR_MOD_PIN  -1  // Connect a GPIO pin to the Smart Effector MOD pin//将GPIO引脚连接到智能效应器模块引脚
#endif

/**
 * Use StallGuard2 to probe the bed with the nozzle.
 * Requires stallGuard-capable Trinamic stepper drivers.
 * CAUTION: This can damage machines with Z lead screws.
 *          Take extreme care when setting up this feature.
 */
//#define SENSORLESS_PROBING//#定义无传感器探测

////
// For Z_PROBE_ALLEN_KEY see the Delta example configurations.//有关Z_PROBE_ALLEN_键，请参见Delta示例配置。
////

/**
 * Nozzle-to-Probe offsets { X, Y, Z }
 *
 * X and Y offset
 *   Use a caliper or ruler to measure the distance from the tip of
 *   the Nozzle to the center-point of the Probe in the X and Y axes.
 *
 * Z offset
 * - For the Z offset use your best known value and adjust at runtime.
 * - Common probes trigger below the nozzle and have negative values for Z offset.
 * - Probes triggering above the nozzle height are uncommon but do exist. When using
 *   probes such as this, carefully set Z_CLEARANCE_DEPLOY_PROBE and Z_CLEARANCE_BETWEEN_PROBES
 *   to avoid collisions during probing.
 *
 * Tune and Adjust
 * -  Probe Offsets can be tuned at runtime with 'M851', LCD menus, babystepping, etc.
 * -  PROBE_OFFSET_WIZARD (configuration_adv.h) can be used for setting the Z offset.
 *
 * Assuming the typical work area orientation:
 *  - Probe to RIGHT of the Nozzle has a Positive X offset
 *  - Probe to LEFT  of the Nozzle has a Negative X offset
 *  - Probe in BACK  of the Nozzle has a Positive Y offset
 *  - Probe in FRONT of the Nozzle has a Negative Y offset
 *
 * Some examples:
 *   #define NOZZLE_TO_PROBE_OFFSET { 10, 10, -1 }   // Example "1"
 *   #define NOZZLE_TO_PROBE_OFFSET {-10,  5, -1 }   // Example "2"
 *   #define NOZZLE_TO_PROBE_OFFSET {  5, -5, -1 }   // Example "3"
 *   #define NOZZLE_TO_PROBE_OFFSET {-15,-10, -1 }   // Example "4"
 *
 *     +-- BACK ---+
 *     |    [+]    |
 *   L |        1  | R <-- Example "1" (right+,  back+)
 *   E |  2        | I <-- Example "2" ( left-,  back+)
 *   F |[-]  N  [+]| G <-- Nozzle
 *   T |       3   | H <-- Example "3" (right+, front-)
 *     | 4         | T <-- Example "4" ( left-, front-)
 *     |    [-]    |
 *     O-- FRONT --+
 */
#define NOZZLE_TO_PROBE_OFFSET { 0, -25, 0 }

// Most probes should stay away from the edges of the bed, but//大多数探头应远离床的边缘，但
// with NOZZLE_AS_PROBE this can be negative for a wider probing area.//使用喷嘴作为探头时，对于更宽的探测区域，这可能是负面的。
#define PROBING_MARGIN 10

// X and Y axis travel speed (mm/min) between probes//探头之间的X轴和Y轴移动速度（mm/min）
#define XY_PROBE_FEEDRATE (133*60)

// Feedrate (mm/min) for the first approach when double-probing (MULTIPLE_PROBING == 2)//双探测时第一次进近的进给速度（mm/min）（多个探测==2）
#define Z_PROBE_FEEDRATE_FAST (4*60)

// Feedrate (mm/min) for the "accurate" probe of each point//各点“精确”探头的进给速度（mm/min）
#define Z_PROBE_FEEDRATE_SLOW (Z_PROBE_FEEDRATE_FAST / 2)

/**
 * Probe Activation Switch
 * A switch indicating proper deployment, or an optical
 * switch triggered when the carriage is near the bed.
 */
//#define PROBE_ACTIVATION_SWITCH//#定义探测激活开关
#if ENABLED(PROBE_ACTIVATION_SWITCH)
#define PROBE_ACTIVATION_SWITCH_STATE LOW // State indicating probe is active//状态指示探针处于活动状态
  //#define PROBE_ACTIVATION_SWITCH_PIN PC6 // Override default pin//#定义探针激活开关插脚PC6//覆盖默认插脚
#endif

/**
 * Tare Probe (determine zero-point) prior to each probe.
 * Useful for a strain gauge or piezo sensor that needs to factor out
 * elements such as cables pulling on the carriage.
 */
//#define PROBE_TARE//#定义探头重量
#if ENABLED(PROBE_TARE)
#define PROBE_TARE_TIME  200    // (ms) Time to hold tare pin//（ms）保持皮重销的时间
  #define PROBE_TARE_DELAY 200    // (ms) Delay after tare before//（ms）皮重之后的延迟
  #define PROBE_TARE_STATE HIGH   // State to write pin for tare//为皮重写入pin的状态
  //#define PROBE_TARE_PIN PA5    // Override default pin//#定义探针端号PA5//覆盖默认端号
  #if ENABLED(PROBE_ACTIVATION_SWITCH)
    //#define PROBE_TARE_ONLY_WHILE_INACTIVE  // Fail to tare/probe if PROBE_ACTIVATION_SWITCH is active//#当探测器处于非活动状态时，定义探测器仅去皮//如果探测器激活开关处于活动状态，则无法去皮/探测
  #endif
#endif

/**
 * Multiple Probing
 *
 * You may get improved results by probing 2 or more times.
 * With EXTRA_PROBING the more atypical reading(s) will be disregarded.
 *
 * A total of 2 does fast/slow probes with a weighted average.
 * A total of 3 or more adds more slow probes, taking the average.
 */
//#define MULTIPLE_PROBING 2//#定义多个_探测2
//#define EXTRA_PROBING    1//#定义额外的\u探测1

/**
 * Z probes require clearance when deploying, stowing, and moving between
 * probe points to avoid hitting the bed and other hardware.
 * Servo-mounted probes require extra space for the arm to rotate.
 * Inductive probes need space to keep from triggering early.
 *
 * Use these settings to specify the distance (mm) to raise the probe (or
 * lower the bed). The values set here apply over and above any (negative)
 * probe Z Offset set with NOZZLE_TO_PROBE_OFFSET, M851, or the LCD.
 * Only integer values >= 1 are valid here.
 *
 * Example: `M851 Z-5` with a CLEARANCE of 4  =>  9mm from bed to nozzle.
 *     But: `M851 Z+1` with a CLEARANCE of 2  =>  2mm from bed to nozzle.
 */
#define Z_CLEARANCE_DEPLOY_PROBE   10 // Z Clearance for Deploy/Stow//Z展开/收起间隙
#define Z_CLEARANCE_BETWEEN_PROBES  5 // Z Clearance between probe points//Z探头点之间的间隙
#define Z_CLEARANCE_MULTI_PROBE     5 // Z Clearance between multiple probes//多探头之间的Z间隙
//#define Z_AFTER_PROBING           5 // Z position after probing is done//#探测后定义Z_探测完成后的5//Z位置

#define Z_PROBE_LOW_POINT          -2 // Farthest distance below the trigger-point to go before stopping//停车前低于触发点的最远距离

// For M851 give a range for adjusting the Z probe offset//对于M851，给出调整Z探头偏移的范围
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20

// Enable the M48 repeatability test to test probe accuracy//启用M48重复性测试以测试探针精度
#define Z_MIN_PROBE_REPEATABILITY_TEST//#定义Z_MIN_探头_重复性_测试

// Before deploy/stow pause for user confirmation//展开/收起前暂停以供用户确认
//#define PAUSE_BEFORE_DEPLOY_STOW//#在部署前定义暂停
#if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)
//#define PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED // For Manual Deploy Allenkey Probe//#为手动部署链接探测定义触发时的暂停探测部署//
#endif

/**
 * Enable one or more of the following if probing seems unreliable.
 * Heaters and/or fans can be disabled during probing to minimize electrical
 * noise. A delay can also be added to allow noise and vibration to settle.
 * These options are most useful for the BLTouch probe, but may also improve
 * readings with inductive probes and piezo sensors.
 */
//#define PROBING_HEATERS_OFF       // Turn heaters off when probing//#定义探测加热器关闭//探测时关闭加热器
#if ENABLED(PROBING_HEATERS_OFF)
//#define WAIT_FOR_BED_HEATER     // Wait for bed to heat back up between probes (to improve accuracy)//#定义等待床加热器//等待床在探头之间加热（以提高精度）
  //#define WAIT_FOR_HOTEND         // Wait for hotend to heat back up between probes (to improve accuracy & prevent cold extrude)//#定义WAIT_FOR_HOTEND//等待HOTEND在探头之间加热（以提高精度并防止冷挤压）
#endif
//#define PROBING_FANS_OFF          // Turn fans off when probing//#定义探测\u风扇\u关闭//探测时关闭风扇
//#define PROBING_ESTEPPERS_OFF     // Turn all extruder steppers off when probing//#定义探测装置关闭//探测时关闭所有挤出机步进装置
//#define PROBING_STEPPERS_OFF      // Turn all steppers off (unless needed to hold position) when probing (including extruders)//#定义探测\步进器\关闭//探测时关闭所有步进器（除非需要保持位置）（包括挤出机）
//#define DELAY_BEFORE_PROBING 200  // (ms) To prevent vibrations from triggering piezo sensors//#在探测200/（ms）之前定义延迟，以防止振动触发压电传感器

// Require minimum nozzle and/or bed temperature for probing//需要最低喷嘴和/或床温进行探测
//#define PREHEAT_BEFORE_PROBING//#在探测前定义预热
#if ENABLED(PREHEAT_BEFORE_PROBING)
#define PROBING_NOZZLE_TEMP 120   // (°C) Only applies to E0 at this time//（°C）此时仅适用于E0
  #define PROBING_BED_TEMP     50
#endif

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1//对于反相步进电机启用引脚（低电平有效）使用0，非反相（高电平有效）使用1
// :{ 0:'Low', 1:'High' }//：{0:'Low'，1:'High'}
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders//适用于所有挤出机
//#define I_ENABLE_ON 0//#在0上定义I\u启用
//#define J_ENABLE_ON 0//#在0上定义J_ENABLE_
//#define K_ENABLE_ON 0//#在0上定义K_启用_

// Disable axis steppers immediately when they're not being stepped.//当轴步进器未被步进时，立即禁用轴步进器。
// WARNING: When motors turn off there is a chance of losing position accuracy!//警告：当电机关闭时，有可能失去位置精度！
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
//#define DISABLE_I false//#定义DISABLE_I false
//#define DISABLE_J false//#定义DISABLE_J false
//#define DISABLE_K false//#定义DISABLE_kfalse

// Turn off the display blinking that warns about possible accuracy reduction//关闭警告可能降低精度的显示屏闪烁
//#define DISABLE_REDUCED_ACCURACY_WARNING//#定义禁用\u精度降低\u警告

// @section extruder//型材挤出机

#define DISABLE_E false             // Disable the extruder when not stepping//不步进时禁用挤出机
#define DISABLE_INACTIVE_EXTRUDER   // Keep only the active extruder enabled//仅启用活动挤出机

// @section machine//型材机

// Invert the stepper direction. Change (or reverse the motor connector) if an axis goes the wrong way.//反转步进电机的方向。如果轴走错方向，则更换（或反转电机接头）。
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR true
//#define INVERT_I_DIR false//#定义反转方向false
//#define INVERT_J_DIR false//#定义反转方向false
//#define INVERT_K_DIR false//#定义反转方向false

// @section extruder//型材挤出机

// For direct drive extruder v9 set to true, for geared extruder set to false.//对于直接驱动挤出机v9，设置为真；对于齿轮传动挤出机，设置为假。
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define INVERT_E6_DIR false
#define INVERT_E7_DIR false

// @section homing//@段归位

#define NO_MOTION_BEFORE_HOMING // Inhibit movement until all axes have been homed. Also enable HOME_AFTER_DEACTIVATE for extra safety.//#在归位前定义无运动//禁止运动，直到所有轴都归位。也可以在禁用后启用HOME_，以提高安全性。
//#define HOME_AFTER_DEACTIVATE   // Require rehoming after steppers are deactivated. Also enable NO_MOTION_BEFORE_HOMING for extra safety.//#取消激活后定义主页//取消激活步进器后需要重新命名。还可以在归位前启用无运动，以获得额外的安全性。

/**
 * Set Z_IDLE_HEIGHT if the Z-Axis moves on its own when steppers are disabled.
 *  - Use a low value (i.e., Z_MIN_POS) if the nozzle falls down to the bed.
 *  - Use a large value (i.e., Z_MAX_POS) if the bed falls down, away from the nozzle.
 */
//#define Z_IDLE_HEIGHT Z_HOME_POS//#定义Z_空闲高度Z_主位置

//#define Z_HOMING_HEIGHT  4      // (mm) Minimal Z height before homing (G28) for Z clearance above the bed, clamps, ...//#定义Z_归位高度4/（mm）归位前的最小Z高度（G28），用于床身、夹具等上方的Z间隙。。。
// Be sure to have this much clearance over your Z_MAX_POS to prevent grinding.//确保在Z_MAX_位置上有这么大的间隙，以防止研磨。

//#define Z_AFTER_HOMING  10      // (mm) Height to move to after homing Z//#定义Z_归位后的Z_10//（mm）高度，以便在归位Z后移动

// Direction of endstops when homing; 1=MAX, -1=MIN//归位时的终止方向；1=最大值，-1=最小值
// :[-1,1]// :[-1,1]
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
//#define I_HOME_DIR -1//#定义I_HOME_DIR-1
//#define J_HOME_DIR -1//#定义J_HOME_DIR-1
//#define K_HOME_DIR -1//#定义K_HOME_DIR-1

// @section machine//型材机

// The size of the printable area//可打印区域的大小
#define X_BED_SIZE 210
#define Y_BED_SIZE 180

// Travel limits (mm) after homing, corresponding to endstop positions.//归位后的行程限制（mm），对应于末端停止位置。
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 300
//#define I_MIN_POS 0//#定义I_MIN_位置0
//#define I_MAX_POS 50//#定义I_MAX_位置50
//#define J_MIN_POS 0//#定义J_MIN_位置0
//#define J_MAX_POS 50//#定义J_MAX_位置50
//#define K_MIN_POS 0//#定义K_MIN_位置0
//#define K_MAX_POS 50//#定义K_MAX_位置50

/**
 * Software Endstops
 *
 * - Prevent moves outside the set machine bounds.
 * - Individual axes can be disabled, if desired.
 * - X and Y only apply to Cartesian robots.
 * - Use 'M211' to set software endstops on/off or report current state
 */

// Min software endstops constrain movement within minimum coordinate bounds//最小软件端点限制在最小坐标边界内的移动
#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
#define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
  #define MIN_SOFTWARE_ENDSTOP_I
  #define MIN_SOFTWARE_ENDSTOP_J
  #define MIN_SOFTWARE_ENDSTOP_K
#endif

// Max software endstops constrain movement within maximum coordinate bounds//最大软件终点限制在最大坐标范围内的移动
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
#define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
  #define MAX_SOFTWARE_ENDSTOP_I
  #define MAX_SOFTWARE_ENDSTOP_J
  #define MAX_SOFTWARE_ENDSTOP_K
#endif

#if EITHER(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
//#define SOFT_ENDSTOPS_MENU_ITEM  // Enable/Disable software endstops from the LCD//#定义软停止\菜单\项//从LCD启用/禁用软件停止
#endif

/**
 * Filament Runout Sensors
 * Mechanical or opto endstops are used to check for the presence of filament.
 *
 * IMPORTANT: Runout will only trigger if Marlin is aware that a print job is running.
 * Marlin knows a print job is running when:
 *  1. Running a print job from media started with M24.
 *  2. The Print Job Timer has been started with M75.
 *  3. The heaters were turned on and PRINTJOB_TIMER_AUTOSTART is enabled.
 *
 * RAMPS-based boards use SERVO3_PIN for the first runout sensor.
 * For other boards you may need to define FIL_RUNOUT_PIN, FIL_RUNOUT2_PIN, etc.
 */
//#define FILAMENT_RUNOUT_SENSOR//#定义灯丝跳动传感器
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
#define FIL_RUNOUT_ENABLED_DEFAULT true // Enable the sensor on startup. Override with M412 followed by M500.//启动时启用传感器。使用M412和M500进行超越。
  #define NUM_RUNOUT_SENSORS   1          // Number of sensors, up to one per extruder. Define a FIL_RUNOUT#_PIN for each.//传感器数量，每个挤出机最多一个。为每个管脚定义一个FIL_跳动。

  #define FIL_RUNOUT_STATE     LOW        // Pin state indicating that filament is NOT present.//指示灯丝不存在的Pin状态。
  #define FIL_RUNOUT_PULLUP               // Use internal pullup for filament runout pins.//对灯丝偏转销使用内部上拉。
  //#define FIL_RUNOUT_PULLDOWN           // Use internal pulldown for filament runout pins.//#定义FIL_RUNOUT_PULLDOWN//使用灯丝偏移销的内部下拉。
  //#define WATCH_ALL_RUNOUT_SENSORS      // Execute runout script on any triggering sensor, not only for the active extruder.//#定义监视所有跳动传感器//在任何触发传感器上执行跳动脚本，而不仅仅是在活动挤出机上。
                                          // This is automatically enabled for MIXING_EXTRUDERs.//这是自动启用的混合_挤出机。

  // Override individually if the runout sensors vary//如果跳动传感器变化，则单独超越
  //#define FIL_RUNOUT1_STATE LOW//#定义FIL\U RUNOUT1\U状态低
  //#define FIL_RUNOUT1_PULLUP//#定义FIL\u RUNOUT1\u上拉
  //#define FIL_RUNOUT1_PULLDOWN//#定义FIL\U RUNOUT1\U下拉列表

  //#define FIL_RUNOUT2_STATE LOW//#定义FIL\U RUNOUT2\U状态低
  //#define FIL_RUNOUT2_PULLUP//#定义FIL\u RUNOUT2\u上拉
  //#define FIL_RUNOUT2_PULLDOWN//#定义FIL\u RUNOUT2\u下拉列表

  //#define FIL_RUNOUT3_STATE LOW//#定义FIL_RUNOUT3_状态低
  //#define FIL_RUNOUT3_PULLUP//#定义FIL\u RUNOUT3\u上拉
  //#define FIL_RUNOUT3_PULLDOWN//#定义FIL\U RUNOUT3\U下拉列表

  //#define FIL_RUNOUT4_STATE LOW//#定义FIL_RUNOUT4_状态低
  //#define FIL_RUNOUT4_PULLUP//#定义FIL\u runout 4\u上拉
  //#define FIL_RUNOUT4_PULLDOWN//#定义FIL\u RUNOUT4\u下拉列表

  //#define FIL_RUNOUT5_STATE LOW//#定义FIL_RUNOUT5_状态低
  //#define FIL_RUNOUT5_PULLUP//#定义FIL\u runout 5\u上拉
  //#define FIL_RUNOUT5_PULLDOWN//#定义FIL\u runout 5\u下拉列表

  //#define FIL_RUNOUT6_STATE LOW//#定义FIL_RUNOUT6_状态低
  //#define FIL_RUNOUT6_PULLUP//#定义FIL_RUNOUT6_上拉
  //#define FIL_RUNOUT6_PULLDOWN//#定义FIL\U RUNOUT6\U下拉列表

  //#define FIL_RUNOUT7_STATE LOW//#定义FIL_RUNOUT7_状态低
  //#define FIL_RUNOUT7_PULLUP//#定义FIL\u runout 7\u上拉
  //#define FIL_RUNOUT7_PULLDOWN//#定义FIL\u runout 7\u下拉列表

  //#define FIL_RUNOUT8_STATE LOW//#定义FIL_输出8_状态低
  //#define FIL_RUNOUT8_PULLUP//#定义FIL\u RUNOUT8\u上拉
  //#define FIL_RUNOUT8_PULLDOWN//#定义FIL\U RUNOUT8\U下拉列表

  // Commands to execute on filament runout.//灯丝跳动时执行的命令。
  // With multiple runout sensors use the %c placeholder for the current tool in commands (e.g., "M600 T%c")//对于多个跳动传感器，在命令中为当前刀具使用%c占位符（例如，“M600 T%c”）
  // NOTE: After 'M412 H1' the host handles filament runout and this script does not apply.//注意：“M412 H1”之后，主机将处理灯丝跳动，此脚本不适用。
  #define FILAMENT_RUNOUT_SCRIPT "M600"

  // After a runout is detected, continue printing this length of filament//检测到跳动后，继续打印此长度的灯丝
  // before executing the runout script. Useful for a sensor at the end of//在执行runout脚本之前。适用于测试结束时的传感器
  // a feed tube. Requires 4 bytes SRAM per sensor, plus 4 bytes overhead.//进料管。每个传感器需要4字节SRAM，外加4字节开销。
  //#define FILAMENT_RUNOUT_DISTANCE_MM 25//#定义灯丝偏移距离25毫米

  #ifdef FILAMENT_RUNOUT_DISTANCE_MM
    // Enable this option to use an encoder disc that toggles the runout pin//启用此选项可使用可切换跳动销的编码器盘
    // as the filament moves. (Be sure to set FILAMENT_RUNOUT_DISTANCE_MM//随着灯丝的移动。（确保设置灯丝\u跳动\u距离\u MM
    // large enough to avoid false positives.)//足够大以避免误报。）
    //#define FILAMENT_MOTION_SENSOR//#定义灯丝运动传感器
  #endif
#endif

//===========================================================================//===========================================================================
//=============================== Bed Leveling ==============================//=================================================河床平整==============================
//===========================================================================//===========================================================================
// @section calibrate//@段校准

/**
 * Choose one of the options below to enable G29 Bed Leveling. The parameters
 * and behavior of G29 will change depending on your selection.
 *
 *  If using a Probe for Z Homing, enable Z_SAFE_HOMING also!
 *
 * - AUTO_BED_LEVELING_3POINT
 *   Probe 3 arbitrary points on the bed (that aren't collinear)
 *   You specify the XY coordinates of all 3 points.
 *   The result is a single tilted plane. Best for a flat bed.
 *
 * - AUTO_BED_LEVELING_LINEAR
 *   Probe several points in a grid.
 *   You specify the rectangle and the density of sample points.
 *   The result is a single tilted plane. Best for a flat bed.
 *
 * - AUTO_BED_LEVELING_BILINEAR
 *   Probe several points in a grid.
 *   You specify the rectangle and the density of sample points.
 *   The result is a mesh, best for large or uneven beds.
 *
 * - AUTO_BED_LEVELING_UBL (Unified Bed Leveling)
 *   A comprehensive bed leveling system combining the features and benefits
 *   of other systems. UBL also includes integrated Mesh Generation, Mesh
 *   Validation and Mesh Editing systems.
 *
 * - MESH_BED_LEVELING
 *   Probe a grid manually
 *   The result is a mesh, suitable for large or uneven beds. (See BILINEAR.)
 *   For machines without a probe, Mesh Bed Leveling provides a method to perform
 *   leveling in steps so you can manually adjust the Z height at each grid-point.
 *   With an LCD controller the process is guided step-by-step.
 */
//#define AUTO_BED_LEVELING_3POINT//#定义自动平层点
//#define AUTO_BED_LEVELING_LINEAR//#定义自动平层线性
#define AUTO_BED_LEVELING_BILINEAR//#定义自动调平双线性
//#define AUTO_BED_LEVELING_UBL//#定义自动调平床
//#define MESH_BED_LEVELING//#定义网格\u床\u找平

/**
 * Normally G28 leaves leveling disabled on completion. Enable one of
 * these options to restore the prior leveling state or to always enable
 * leveling immediately after G28.
 */
//#define RESTORE_LEVELING_AFTER_G28//#定义恢复\u升级\u在\u G28之后
//#define ENABLE_LEVELING_AFTER_G28//#定义启用\u找平\u在\u G28之后

/**
 * Auto-leveling needs preheating
 */
//#define PREHEAT_BEFORE_LEVELING//#在调平前定义预热
#if ENABLED(PREHEAT_BEFORE_LEVELING)
#define LEVELING_NOZZLE_TEMP 120   // (°C) Only applies to E0 at this time//（°C）此时仅适用于E0
  #define LEVELING_BED_TEMP     50
#endif

/**
 * Enable detailed logging of G28, G29, M48, etc.
 * Turn on with the command 'M111 S32'.
 * NOTE: Requires a lot of PROGMEM!
 */
#define DEBUG_LEVELING_FEATURE//#定义调试功能

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL, PROBE_MANUALLY)
// Set a height for the start of manual adjustment//设置开始手动调整的高度
  #define MANUAL_PROBE_START_Z 0.2  // (mm) Comment out to use the last-measured height//（mm）注释以使用最后测量的高度
#endif

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL)
// Gradually reduce leveling correction until a set height is reached,//逐渐减少水平校正，直到达到设定高度，
  // at which point movement will be level to the machine's XY plane.//此时，移动将与机器的XY平面保持水平。
  // The height can be set with M420 Z<height>//可使用M420 Z<height>
  #define ENABLE_LEVELING_FADE_HEIGHT
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    #define DEFAULT_LEVELING_FADE_HEIGHT 10.0 // (mm) Default fade height.//（mm）默认淡入度高度。
  #endif

  // For Cartesian machines, instead of dividing moves on mesh boundaries,//对于笛卡尔机器，不是在网格边界上分割移动，
  // split up moves into short segments like a Delta. This follows the//“拆分”会移动到类似三角形的短段中。这是根据
  // contours of the bed more closely than edge-to-edge straight moves.//床的轮廓比边到边的直线运动更紧密。
  #define SEGMENT_LEVELED_MOVES
  #define LEVELED_SEGMENT_LENGTH 5.0 // (mm) Length of all segments (except the last one)//（mm）所有段的长度（最后一段除外）

  /**
   * Enable the G26 Mesh Validation Pattern tool.
   */
  #define G26_MESH_VALIDATION//#定义G26网格验证
  #if ENABLED(G26_MESH_VALIDATION)
    #define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.//（mm）主喷嘴的直径。
    #define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for G26.//（mm）G26的默认层高度。
    #define MESH_TEST_HOTEND_TEMP  205    // (°C) Default nozzle temperature for G26.//G26的默认喷嘴温度（°C）。
    #define MESH_TEST_BED_TEMP      60    // (°C) Default bed temperature for G26.//G26的默认床温（°C）。
    #define G26_XY_FEEDRATE         20    // (mm/s) Feedrate for G26 XY moves.//G26 XY移动的进给速度（mm/s）。
    #define G26_XY_FEEDRATE_TRAVEL 100    // (mm/s) Feedrate for G26 XY travel moves.//G26 XY移动的进给速度（mm/s）。
    #define G26_RETRACT_MULTIPLIER   1.0  // G26 Q (retraction) used by default between mesh test elements.//网格测试元素之间默认使用G26 Q（收缩）。
  #endif

#endif

#if EITHER(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)

// Set the number of grid points per dimension.//设置每个标注的栅格点数。
  #define GRID_MAX_POINTS_X 3
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  // Probe along the Y axis, advancing X after each column//沿Y轴进行探测，在每列之后推进X
  //#define PROBE_Y_FIRST//#首先定义PROBE_Y_

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    // Beyond the probed grid, continue the implied tilt?//在探测网格之外，继续隐含的倾斜？
    // Default is to maintain the height of the nearest edge.//默认设置是保持最近边的高度。
    //#define EXTRAPOLATE_BEYOND_GRID//#定义超出网格的外推网格

    ////
    // Experimental Subdivision of the grid by Catmull-Rom method.//用Catmull-Rom方法对网格进行实验细分。
    // Synthesizes intermediate points to produce a more detailed mesh.//合成中间点以生成更详细的网格。
    ////
    //#define ABL_BILINEAR_SUBDIVISION//#定义ABL_双线性_细分
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      // Number of subdivisions between probe points//探测点之间的细分数
      #define BILINEAR_SUBDIVISIONS 3
    #endif

  #endif

#elif ENABLED(AUTO_BED_LEVELING_UBL)

//===========================================================================//===========================================================================
  //========================= Unified Bed Leveling ============================//===========================================统一河床平整============================
  //===========================================================================//===========================================================================

  //#define MESH_EDIT_GFX_OVERLAY   // Display a graphics overlay while editing the mesh//#定义网格\u编辑\u GFX\u覆盖//编辑网格时显示图形覆盖

  #define MESH_INSET 1              // Set Mesh bounds as an inset region of the bed//将网格边界设置为床的插入区域
  #define GRID_MAX_POINTS_X 10      // Don't use more than 15 points per axis, implementation limited.//每个轴使用的点不得超过15个，实施受到限制。
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  //#define UBL_HILBERT_CURVE       // Use Hilbert distribution for less travel when probing multiple points//#定义UBL_HILBERT_曲线//探测多个点时，使用HILBERT分布减少行程

  #define UBL_MESH_EDIT_MOVES_Z     // Sophisticated users prefer no movement of nozzle//老练的用户不喜欢移动喷嘴
  #define UBL_SAVE_ACTIVE_ON_M500   // Save the currently active mesh in the current slot on M500//将当前活动网格保存在M500上的当前插槽中

  //#define UBL_Z_RAISE_WHEN_OFF_MESH 2.5 // When the nozzle is off the mesh, this value is used//#定义UBL_Z_RAISE_WHEN_OFF_MESH 2.5//喷嘴离开网格时，使用此值
                                          // as the Z-Height correction value.//作为Z高度校正值。

  //#define UBL_MESH_WIZARD         // Run several commands in a row to get a complete mesh//#定义UBL_网格向导//在行中运行多个命令以获得完整的网格

#elif ENABLED(MESH_BED_LEVELING)

//===========================================================================//===========================================================================
  //=================================== Mesh ==================================//====================================================网格==================================
  //===========================================================================//===========================================================================

  #define MESH_INSET 10          // Set Mesh bounds as an inset region of the bed//将网格边界设置为床的插入区域
  #define GRID_MAX_POINTS_X 3    // Don't use more than 7 points per axis, implementation limited.//每个轴使用的点不得超过7个，实施受到限制。
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

  //#define MESH_G28_REST_ORIGIN // After homing all axes ('G28' or 'G28 XYZ') rest Z at Z_MIN_POS//#在Z_MIN_位置重置所有轴（“G28”或“G28 XYZ”）静止Z后，定义网格_G28_静止_原点//

#endif // BED_LEVELING//河床平整

/**
 * Add a bed leveling sub-menu for ABL or MBL.
 * Include a guided procedure if manual probing is enabled.
 */
//#define LCD_BED_LEVELING//#定义LCD_床_调平

#if ENABLED(LCD_BED_LEVELING)
#define MESH_EDIT_Z_STEP  0.025 // (mm) Step size while manually probing Z axis.//（mm）手动探测Z轴时的步长。
  #define LCD_PROBE_Z_RANGE 4     // (mm) Z Range centered on Z_MIN_POS for LCD Z adjustment//（mm）Z范围以Z_MIN_POS为中心，用于LCD Z调整
  //#define MESH_EDIT_MENU        // Add a menu to edit mesh points//#定义网格\编辑\菜单//添加菜单以编辑网格点
#endif

// Add a menu item to move between bed corners for manual bed adjustment//添加一个菜单项以在床角之间移动，以便手动调整床
//#define LEVEL_BED_CORNERS//#定义标高\u床\u角

#if ENABLED(LEVEL_BED_CORNERS)
#define LEVEL_CORNERS_INSET_LFRB { 30, 30, 30, 30 } // (mm) Left, Front, Right, Back insets//（mm）左、前、右、后插图
  #define LEVEL_CORNERS_HEIGHT      0.0   // (mm) Z height of nozzle at leveling points//（mm）找平点处喷嘴的Z高度
  #define LEVEL_CORNERS_Z_HOP       4.0   // (mm) Z height of nozzle between leveling points//（mm）调平点之间喷嘴的Z高度
  //#define LEVEL_CENTER_TOO              // Move to the center after the last corner//#定义标高\中心\也//移动到最后一个角点后的中心
  //#define LEVEL_CORNERS_USE_PROBE//#定义标高\u角点\u使用\u探头
  #if ENABLED(LEVEL_CORNERS_USE_PROBE)
    #define LEVEL_CORNERS_PROBE_TOLERANCE 0.1
    #define LEVEL_CORNERS_VERIFY_RAISED   // After adjustment triggers the probe, re-probe to verify//调整触发探针后，重新进行探针验证
    //#define LEVEL_CORNERS_AUDIO_FEEDBACK//#定义级别\u角落\u音频\u反馈
  #endif

  /**
   * Corner Leveling Order
   *
   * Set 2 or 4 points. When 2 points are given, the 3rd is the center of the opposite edge.
   *
   *  LF  Left-Front    RF  Right-Front
   *  LB  Left-Back     RB  Right-Back
   *
   * Examples:
   *
   *      Default        {LF,RB,LB,RF}         {LF,RF}           {LB,LF}
   *  LB --------- RB   LB --------- RB    LB --------- RB   LB --------- RB
   *  |  4       3  |   | 3         2 |    |     <3>     |   | 1           |
   *  |             |   |             |    |             |   |          <3>|
   *  |  1       2  |   | 1         4 |    | 1         2 |   | 2           |
   *  LF --------- RF   LF --------- RF    LF --------- RF   LF --------- RF
   */
  #define LEVEL_CORNERS_LEVELING_ORDER { LF, RF, RB, LB }
#endif

/**
 * Commands to execute at the end of G29 probing.
 * Useful to retract or move the Z probe out of the way.
 */
//#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10"//#定义Z_探测结束脚本“G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10”

// @section homing//@段归位

// The center of the bed is at (X=0, Y=0)//床的中心位于（X=0，Y=0）
//#define BED_CENTER_AT_0_0//#在0处定义床面中心

// Manually set the home position. Leave these undefined for automatic settings.//手动设置起始位置。保留这些未定义的自动设置。
// For DELTA this is the top-center of the Cartesian print volume.//对于DELTA，这是笛卡尔打印卷的顶部中心。
#define MANUAL_X_HOME_POS -20//#定义手动\u X\u主页\u位置0
#define MANUAL_Y_HOME_POS -40//#定义手动_Y_HOME_位置0
#define MANUAL_Z_HOME_POS 0//#定义手动_Z_HOME_位置0
//#define MANUAL_I_HOME_POS 0//#定义手动\u I\u主页\u位置0
//#define MANUAL_J_HOME_POS 0//#定义手动回家位置0
//#define MANUAL_K_HOME_POS 0//#定义手动\u K\u主页\u位置0

/**
 * Use "Z Safe Homing" to avoid homing with a Z probe outside the bed area.
 *
 * - Moves the Z probe (or nozzle) to a defined XY point before Z homing.
 * - Allows Z homing only when XY positions are known and trusted.
 * - If stepper drivers sleep, XY homing may be required again before Z homing.
 */
#define Z_SAFE_HOMING//#定义Z_安全归位
#define Z_SAFE_HOMING_X_POINT 110  // X point for Z homing//Z原点的X点
#define Z_SAFE_HOMING_Y_POINT 110  // Y point for Z homing//Z原点的Y点
/*#if ENABLED(Z_SAFE_HOMING)

#endif*/

// Homing speeds (mm/min)//归位速度（毫米/分钟）
#define HOMING_FEEDRATE_MM_M { (50*60), (50*60), (4*60) }

// Validate that endstops are triggered on homing moves//验证归位移动时是否触发结束停止
#define VALIDATE_HOMING_ENDSTOPS

// @section calibrate//@段校准

/**
 * Bed Skew Compensation
 *
 * This feature corrects for misalignment in the XYZ axes.
 *
 * Take the following steps to get the bed skew in the XY plane:
 *  1. Print a test square (e.g., https://www.thingiverse.com/thing:2563185)
 *  2. For XY_DIAG_AC measure the diagonal A to C
 *  3. For XY_DIAG_BD measure the diagonal B to D
 *  4. For XY_SIDE_AD measure the edge A to D
 *
 * Marlin automatically computes skew factors from these measurements.
 * Skew factors may also be computed and set manually:
 *
 *  - Compute AB     : SQRT(2*AC*AC+2*BD*BD-4*AD*AD)/2
 *  - XY_SKEW_FACTOR : TAN(PI/2-ACOS((AC*AC-AB*AB-AD*AD)/(2*AB*AD)))
 *
 * If desired, follow the same procedure for XZ and YZ.
 * Use these diagrams for reference:
 *
 *    Y                     Z                     Z
 *    ^     B-------C       ^     B-------C       ^     B-------C
 *    |    /       /        |    /       /        |    /       /
 *    |   /       /         |   /       /         |   /       /
 *    |  A-------D          |  A-------D          |  A-------D
 *    +-------------->X     +-------------->X     +-------------->Y
 *     XY_SKEW_FACTOR        XZ_SKEW_FACTOR        YZ_SKEW_FACTOR
 */
//#define SKEW_CORRECTION//#定义倾斜校正

#if ENABLED(SKEW_CORRECTION)
// Input all length measurements here://在此处输入所有长度测量值：
  #define XY_DIAG_AC 282.8427124746
  #define XY_DIAG_BD 282.8427124746
  #define XY_SIDE_AD 200

  // Or, set the default skew factors directly here//或者，直接在此处设置默认的倾斜因子
  // to override the above measurements://要覆盖上述测量值，请执行以下操作：
  #define XY_SKEW_FACTOR 0.0

  //#define SKEW_CORRECTION_FOR_Z//#为Z定义倾斜校正
  #if ENABLED(SKEW_CORRECTION_FOR_Z)
    #define XZ_DIAG_AC 282.8427124746
    #define XZ_DIAG_BD 282.8427124746
    #define YZ_DIAG_AC 282.8427124746
    #define YZ_DIAG_BD 282.8427124746
    #define YZ_SIDE_AD 200
    #define XZ_SKEW_FACTOR 0.0
    #define YZ_SKEW_FACTOR 0.0
  #endif

  // Enable this option for M852 to set skew at runtime//为M852启用此选项以在运行时设置倾斜
  //#define SKEW_CORRECTION_GCODE//#定义倾斜校正码
#endif

//=============================================================================//=============================================================================
//============================= Additional Features ===========================//==============================================其他功能===========================
//=============================================================================//=============================================================================

// @section extras//@额外部分

/**
 * EEPROM
 *
 * Persistent storage to preserve configurable settings across reboots.
 *
 *   M500 - Store settings to EEPROM.
 *   M501 - Read settings from EEPROM. (i.e., Throw away unsaved changes)
 *   M502 - Revert settings to "factory" defaults. (Follow with M500 to init the EEPROM.)
 */
#define EEPROM_SETTINGS     // Persistent storage with M500 and M501//#定义EEPROM_设置//使用M500和M501进行持久存储
//#define DISABLE_M503        // Saves ~2700 bytes of PROGMEM. Disable for release!//#define DISABLE_M503//保存约2700字节的程序。禁用发布！
#define EEPROM_CHITCHAT       // Give feedback on EEPROM commands. Disable to save PROGMEM.//对EEPROM命令进行反馈。禁用以保存程序。
#define EEPROM_BOOT_SILENT    // Keep M503 quiet and only give errors during first load//保持M503安静，仅在首次加载时给出错误
#if ENABLED(EEPROM_SETTINGS)
//#define EEPROM_AUTO_INIT  // Init EEPROM automatically on any errors.//#定义EEPROM\u AUTO\u INIT//INIT EEPROM在出现任何错误时自动启动。
#endif

////
// Host Keepalive//主机保持
////
// When enabled Marlin will send a busy status message to the host//启用后，Marlin将向主机发送忙碌状态消息
// every couple of seconds when it can't accept commands.//当它不能接受命令时每隔几秒钟。
////
#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages//如果您的主机不喜欢保留邮件，请禁用此选项
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.//“忙”消息之间的秒数。设置为M113。
#define BUSY_WHILE_HEATING            // Some hosts require "busy" messages even during heating//某些主机甚至在加热期间也需要“忙”消息

////
// G20/G21 Inch mode support//G20/G21英寸模式支持
////
//#define INCH_MODE_SUPPORT//#定义英寸模式支持

////
// M149 Set temperature units support//M149设置温度装置支架
////
//#define TEMPERATURE_UNITS_SUPPORT//#定义温度\u单位\u支持

// @section temperature//@截面温度

////
// Preheat Constants - Up to 5 are supported without changes//预热常数-最多支持5个，无需更改
////
#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 180
#define PREHEAT_1_TEMP_BED     70
#define PREHEAT_1_TEMP_CHAMBER 35
#define PREHEAT_1_FAN_SPEED     0 // Value from 0 to 255//从0到255的值

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    110
#define PREHEAT_2_TEMP_CHAMBER 35
#define PREHEAT_2_FAN_SPEED     0 // Value from 0 to 255//从0到255的值

/**
 * Nozzle Park
 *
 * Park the nozzle at the given XYZ position on idle or G27.
 *
 * The "P" parameter controls the action applied to the Z axis:
 *
 *    P0  (Default) If Z is below park Z raise the nozzle.
 *    P1  Raise the nozzle always to Z-park height.
 *    P2  Raise the nozzle by Z-park amount, limited to Z_MAX_POS.
 */
//#define NOZZLE_PARK_FEATURE//#定义喷嘴\u驻车\u功能

#if ENABLED(NOZZLE_PARK_FEATURE)
// Specify a park position as { X, Y, Z_raise }//将驻车位置指定为{X，Y，Z_}
  #define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
  //#define NOZZLE_PARK_X_ONLY          // X move only is required to park//#定义喷嘴\仅停驻\仅停驻//停驻仅需移动X
  //#define NOZZLE_PARK_Y_ONLY          // Y move only is required to park//#定义喷嘴\u驻车\u仅Y\u//驻车仅需要Y移动
  #define NOZZLE_PARK_Z_RAISE_MIN   2   // (mm) Always raise Z by at least this distance//（mm）始终将Z至少升高此距离
  #define NOZZLE_PARK_XY_FEEDRATE 100   // (mm/s) X and Y axes feedrate (also used for delta Z axis)//（mm/s）X和Y轴进给速度（也用于δZ轴）
  #define NOZZLE_PARK_Z_FEEDRATE    5   // (mm/s) Z axis feedrate (not used for delta printers)//（mm/s）Z轴进给速度（不用于delta打印机）
#endif

/**
 * Clean Nozzle Feature -- EXPERIMENTAL
 *
 * Adds the G12 command to perform a nozzle cleaning process.
 *
 * Parameters:
 *   P  Pattern
 *   S  Strokes / Repetitions
 *   T  Triangles (P1 only)
 *
 * Patterns:
 *   P0  Straight line (default). This process requires a sponge type material
 *       at a fixed bed location. "S" specifies strokes (i.e. back-forth motions)
 *       between the start / end points.
 *
 *   P1  Zig-zag pattern between (X0, Y0) and (X1, Y1), "T" specifies the
 *       number of zig-zag triangles to do. "S" defines the number of strokes.
 *       Zig-zags are done in whichever is the narrower dimension.
 *       For example, "G12 P1 S1 T3" will execute:
 *
 *          --
 *         |  (X0, Y1) |     /\        /\        /\     | (X1, Y1)
 *         |           |    /  \      /  \      /  \    |
 *       A |           |   /    \    /    \    /    \   |
 *         |           |  /      \  /      \  /      \  |
 *         |  (X0, Y0) | /        \/        \/        \ | (X1, Y0)
 *          --         +--------------------------------+
 *                       |________|_________|_________|
 *                           T1        T2        T3
 *
 *   P2  Circular pattern with middle at NOZZLE_CLEAN_CIRCLE_MIDDLE.
 *       "R" specifies the radius. "S" specifies the stroke count.
 *       Before starting, the nozzle moves to NOZZLE_CLEAN_START_POINT.
 *
 *   Caveats: The ending Z should be the same as starting Z.
 * Attention: EXPERIMENTAL. G-code arguments may change.
 */
//#define NOZZLE_CLEAN_FEATURE//#定义喷嘴清洁功能

#if ENABLED(NOZZLE_CLEAN_FEATURE)
// Default number of pattern repetitions//默认模式重复次数
  #define NOZZLE_CLEAN_STROKES  12

  // Default number of triangles//默认三角形数
  #define NOZZLE_CLEAN_TRIANGLES  3

  // Specify positions for each tool as { { X, Y, Z }, { X, Y, Z } }//将每个工具的位置指定为{X，Y，Z}，{X，Y，Z}
  // Dual hotend system may use { {  -20, (Y_BED_SIZE / 2), (Z_MIN_POS + 1) },  {  420, (Y_BED_SIZE / 2), (Z_MIN_POS + 1) }}//双热端系统可使用{-20，（Y_床_尺寸/2），（Z_MIN_位置+1）}，{420，（Y_床_尺寸/2），（Z_MIN_位置+1）}
  #define NOZZLE_CLEAN_START_POINT { {  30, 30, (Z_MIN_POS + 1) } }
  #define NOZZLE_CLEAN_END_POINT   { { 100, 60, (Z_MIN_POS + 1) } }

  // Circular pattern radius//圆形图案半径
  #define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
  // Circular pattern circle fragments number//圆形图案圆形碎片数
  #define NOZZLE_CLEAN_CIRCLE_FN 10
  // Middle point of circle//圆的中点
  #define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT

  // Move the nozzle to the initial position after cleaning//清洁后，将喷嘴移到初始位置
  #define NOZZLE_CLEAN_GOBACK

  // For a purge/clean station that's always at the gantry height (thus no Z move)//对于始终处于机架高度的净化/清洁站（因此无Z移动）
  //#define NOZZLE_CLEAN_NO_Z//#定义喷嘴的清洁度

  // For a purge/clean station mounted on the X axis//对于安装在X轴上的净化/清洁站
  //#define NOZZLE_CLEAN_NO_Y//#定义喷嘴是否清洁

  // Require a minimum hotend temperature for cleaning//清洁时需要最低热端温度
  #define NOZZLE_CLEAN_MIN_TEMP 170
  //#define NOZZLE_CLEAN_HEATUP       // Heat up the nozzle instead of skipping wipe//#定义喷嘴\u清洁\u加热//加热喷嘴，而不是跳过擦拭

  // Explicit wipe G-code script applies to a G12 with no arguments.//显式擦除G代码脚本适用于无参数的G12。
  //#define WIPE_SEQUENCE_COMMANDS "G1 X-17 Y25 Z10 F4000\nG1 Z1\nM114\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 Z15\nM400\nG0 X-10.0 Y-9.0"//#定义擦除序列命令“G1 X-17 Y25 Z10 F4000\nG1 Z1\nM114\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 X-17 Y95\nG1 X-17 Y25\nG1 X-17 Y95\nG1 Z15\nM400\nG0 X-10.0 Y-9.0”

#endif

/**
 * Print Job Timer
 *
 * Automatically start and stop the print job timer on M104/M109/M140/M190/M141/M191.
 * The print job timer will only be stopped if the bed/chamber target temp is
 * below BED_MINTEMP/CHAMBER_MINTEMP.
 *
 *   M104 (hotend, no wait)  - high temp = none,        low temp = stop timer
 *   M109 (hotend, wait)     - high temp = start timer, low temp = stop timer
 *   M140 (bed, no wait)     - high temp = none,        low temp = stop timer
 *   M190 (bed, wait)        - high temp = start timer, low temp = none
 *   M141 (chamber, no wait) - high temp = none,        low temp = stop timer
 *   M191 (chamber, wait)    - high temp = start timer, low temp = none
 *
 * For M104/M109, high temp is anything over EXTRUDE_MINTEMP / 2.
 * For M140/M190, high temp is anything over BED_MINTEMP.
 * For M141/M191, high temp is anything over CHAMBER_MINTEMP.
 *
 * The timer can also be controlled with the following commands:
 *
 *   M75 - Start the print job timer
 *   M76 - Pause the print job timer
 *   M77 - Stop the print job timer
 */
#define PRINTJOB_TIMER_AUTOSTART

/**
 * Print Counter
 *
 * Track statistical data such as:
 *
 *  - Total print jobs
 *  - Total successful print jobs
 *  - Total failed print jobs
 *  - Total time printing
 *
 * View the current statistics with M78.
 */
//#define PRINTCOUNTER//#定义打印计数器
#if ENABLED(PRINTCOUNTER)
#define PRINTCOUNTER_SAVE_INTERVAL 60 // (minutes) EEPROM save interval during print//（分钟）打印期间的EEPROM保存间隔
#endif

/**
 * Password
 *
 * Set a numerical password for the printer which can be requested:
 *
 *  - When the printer boots up
 *  - Upon opening the 'Print from Media' Menu
 *  - When SD printing is completed or aborted
 *
 * The following G-codes can be used:
 *
 *  M510 - Lock Printer. Blocks all commands except M511.
 *  M511 - Unlock Printer.
 *  M512 - Set, Change and Remove Password.
 *
 * If you forget the password and get locked out you'll need to re-flash
 * the firmware with the feature disabled, reset EEPROM, and (optionally)
 * re-flash the firmware again with this feature enabled.
 */
//#define PASSWORD_FEATURE//#定义密码功能
#if ENABLED(PASSWORD_FEATURE)
#define PASSWORD_LENGTH 4                 // (#) Number of digits (1-9). 3 or 4 is recommended//（#）位数（1-9）。建议使用3或4
  #define PASSWORD_ON_STARTUP
  #define PASSWORD_UNLOCK_GCODE             // Unlock with the M511 P<password> command. Disable to prevent brute-force attack.//使用M511 P<password>命令解锁。禁用以防止暴力攻击。
  #define PASSWORD_CHANGE_GCODE             // Change the password with M512 P<old> S<new>.//使用M512 P<old>S<new>更改密码。
  //#define PASSWORD_ON_SD_PRINT_MENU       // This does not prevent gcodes from running//#在\u SD\u PRINT\u菜单上定义密码\u//这不会阻止gcodes运行
  //#define PASSWORD_AFTER_SD_PRINT_END//#在打印结束后定义密码
  //#define PASSWORD_AFTER_SD_PRINT_ABORT//#在\u SD\u打印\u中止后定义密码\u
  //#include "Configuration_Secure.h"       // External file with PASSWORD_DEFAULT_VALUE//#包括“Configuration\u Secure.h”//带有密码\u默认值的外部文件
#endif

//=============================================================================//=============================================================================
//============================= LCD and SD support ============================//=======================================================LCD和SD支持============================
//=============================================================================//=============================================================================

// @section lcd//@section液晶显示器

/**
 * LCD LANGUAGE
 *
 * Select the language to display on the LCD. These languages are available:
 *
 *   en, an, bg, ca, cz, da, de, el, el_gr, es, eu, fi, fr, gl, hr, hu, it,
 *   jp_kana, ko_KR, nl, pl, pt, pt_br, ro, ru, sk, sv, tr, uk, vi, zh_CN, zh_TW
 *
 * :{ 'en':'English', 'an':'Aragonese', 'bg':'Bulgarian', 'ca':'Catalan', 'cz':'Czech', 'da':'Danish', 'de':'German', 'el':'Greek', 'el_gr':'Greek (Greece)', 'es':'Spanish', 'eu':'Basque-Euskera', 'fi':'Finnish', 'fr':'French', 'gl':'Galician', 'hr':'Croatian', 'hu':'Hungarian', 'it':'Italian', 'jp_kana':'Japanese', 'ko_KR':'Korean (South Korea)', 'nl':'Dutch', 'pl':'Polish', 'pt':'Portuguese', 'pt_br':'Portuguese (Brazilian)', 'ro':'Romanian', 'ru':'Russian', 'sk':'Slovak', 'sv':'Swedish', 'tr':'Turkish', 'uk':'Ukrainian', 'vi':'Vietnamese', 'zh_CN':'Chinese (Simplified)', 'zh_TW':'Chinese (Traditional)' }
 */
#define LCD_LANGUAGE en

/**
 * LCD Character Set
 *
 * Note: This option is NOT applicable to Graphical Displays.
 *
 * All character-based LCDs provide ASCII plus one of these
 * language extensions:
 *
 *  - JAPANESE ... the most common
 *  - WESTERN  ... with more accented characters
 *  - CYRILLIC ... for the Russian language
 *
 * To determine the language extension installed on your controller:
 *
 *  - Compile and upload with LCD_LANGUAGE set to 'test'
 *  - Click the controller to view the LCD menu
 *  - The LCD will display Japanese, Western, or Cyrillic text
 *
 * See https://marlinfw.org/docs/development/lcd_language.html
 *
 * :['JAPANESE', 'WESTERN', 'CYRILLIC']
 */
#define DISPLAY_CHARSET_HD44780 JAPANESE

/**
 * Info Screen Style (0:Classic, 1:Průša)
 *
 * :[0:'Classic', 1:'Průša']
 */
#define LCD_INFO_SCREEN_STYLE 0

/**
 * SD CARD
 *
 * SD Card support is disabled by default. If your controller has an SD slot,
 * you must uncomment the following option or it won't work.
 */
//#define SDSUPPORT//#定义SDS支持

/**
 * SD CARD: ENABLE CRC
 *
 * Use CRC checks and retries on the SD communication.
 */
//#define SD_CHECK_AND_RETRY//#定义SD\u检查\u和\u重试

/**
 * LCD Menu Items
 *
 * Disable all menus and only display the Status Screen, or
 * just remove some extraneous menu items to recover space.
 */
//#define NO_LCD_MENUS//#定义无LCD菜单
//#define SLIM_LCD_MENUS//#定义SLIM_LCD_菜单

////
// ENCODER SETTINGS//编码器设置
////
// This option overrides the default number of encoder pulses needed to//此选项将覆盖所需的编码器脉冲的默认数量
// produce one step. Should be increased for high-resolution encoders.//产生一个步骤。对于高分辨率编码器，应增加。
////
//#define ENCODER_PULSES_PER_STEP 4//#按照步骤4定义编码器脉冲

////
// Use this option to override the number of step signals required to//使用此选项可覆盖执行此操作所需的步进信号数
// move between next/prev menu items.//在下一个/上一个菜单项之间移动。
////
//#define ENCODER_STEPS_PER_MENU_ITEM 1//#根据菜单项1定义编码器步骤

/**
 * Encoder Direction Options
 *
 * Test your encoder's behavior first with both options disabled.
 *
 *  Reversed Value Edit and Menu Nav? Enable REVERSE_ENCODER_DIRECTION.
 *  Reversed Menu Navigation only?    Enable REVERSE_MENU_DIRECTION.
 *  Reversed Value Editing only?      Enable BOTH options.
 */

////
// This option reverses the encoder direction everywhere.//此选项可在任何地方反转编码器方向。
////
//  Set this option if CLOCKWISE causes values to DECREASE//如果顺时针方向导致值减小，则设置此选项
////
//#define REVERSE_ENCODER_DIRECTION//#定义反向编码器方向

////
// This option reverses the encoder direction for navigating LCD menus.//此选项可反转用于导航LCD菜单的编码器方向。
////
//  If CLOCKWISE normally moves DOWN this makes it go UP.//如果顺时针方向通常向下移动，则会使其向上移动。
//  If CLOCKWISE normally moves UP this makes it go DOWN.//如果顺时针方向正常向上移动，则使其向下移动。
////
//#define REVERSE_MENU_DIRECTION//#定义反向菜单方向

////
// This option reverses the encoder direction for Select Screen.//此选项反转选择屏幕的编码器方向。
////
//  If CLOCKWISE normally moves LEFT this makes it go RIGHT.//如果顺时针方向通常向左移动，则使其向右移动。
//  If CLOCKWISE normally moves RIGHT this makes it go LEFT.//如果顺时针方向通常向右移动，则使其向左移动。
////
//#define REVERSE_SELECT_DIRECTION//#定义反向选择方向

////
// Individual Axis Homing//单轴归位
////
// Add individual axis homing items (Home X, Home Y, and Home Z) to the LCD menu.//将各个轴原点项目（原点X、原点Y和原点Z）添加到LCD菜单。
////
//#define INDIVIDUAL_AXIS_HOMING_MENU//#定义单个_轴_归位_菜单
//#define INDIVIDUAL_AXIS_HOMING_SUBMENU//#定义单个_轴_归位_子菜单

////
// SPEAKER/BUZZER//扬声器/蜂鸣器
////
// If you have a speaker that can produce tones, enable it here.//如果您有一个可以发出音调的扬声器，请在此处启用它。
// By default Marlin assumes you have a buzzer with a fixed frequency.//默认情况下，Marlin假设您有一个固定频率的蜂鸣器。
////
//#define SPEAKER//#定义说话人

////
// The duration and frequency for the UI feedback sound.//UI反馈声音的持续时间和频率。
// Set these to 0 to disable audio feedback in the LCD menus.//将这些设置为0可禁用LCD菜单中的音频反馈。
////
// Note: Test audio output with the G-Code://注：使用G代码测试音频输出：
//  M300 S<frequency Hz> P<duration ms>//M300秒<频率Hz>P<持续时间ms>
////
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2//#定义LCD\u反馈\u频率\u持续时间\u毫秒2
//#define LCD_FEEDBACK_FREQUENCY_HZ 5000//#定义LCD_反馈_频率_HZ 5000

//=============================================================================//=============================================================================
//======================== LCD / Controller Selection =========================//========================================LCD/控制器选择=========================
//========================   (Character-based LCDs)   =========================//===============================（基于字符的LCD）=========================
//=============================================================================//=============================================================================

////
// RepRapDiscount Smart Controller.//重新计数智能控制器。
// https://reprap.org/wiki/RepRapDiscount_Smart_Controller// https://reprap.org/wiki/RepRapDiscount_Smart_Controller
////
// Note: Usually sold with a white PCB.//注：通常与白色PCB一起出售。
////
//#define REPRAP_DISCOUNT_SMART_CONTROLLER//#定义REPRAP\u折扣\u智能\u控制器

////
// GT2560 (YHCB2004) LCD Display//GT2560（YHCB2004）液晶显示器
////
// Requires Testato, Koepel softwarewire library and//需要Testato、Koepel softwarewire库和
// Andriy Golovnya's LiquidCrystal_AIP31068 library.//安德烈·戈洛夫尼亚的液态水晶AIP31068图书馆。
////
//#define YHCB2004//#定义YHCB2004

////
// Original RADDS LCD Display+Encoder+SDCardReader//原装RADDS液晶显示器+编码器+SDCardReader
// http://doku.radds.org/dokumentation/lcd-display/// http://doku.radds.org/dokumentation/lcd-display/
////
//#define RADDS_DISPLAY//#定义RADDS\u显示

////
// ULTIMAKER Controller.//ULTIMAKER控制器。
////
//#define ULTIMAKERCONTROLLER//#定义ULTIMAKERCONTROLLER

////
// ULTIPANEL as seen on Thingiverse.//如Thingiverse上所示。
////
//#define ULTIPANEL//#定义最终面板

////
// PanelOne from T3P3 (via RAMPS 1.4 AUX2/AUX3)//来自T3P3的面板一（通过坡道1.4 AUX2/AUX3）
// https://reprap.org/wiki/PanelOne// https://reprap.org/wiki/PanelOne
////
//#define PANEL_ONE//#定义面板1

////
// GADGETS3D G3D LCD/SD Controller//GADGETS3D G3D LCD/SD控制器
// https://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel// https://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
////
// Note: Usually sold with a blue PCB.//注：通常与蓝色PCB一起出售。
////
//#define G3D_PANEL//#定义G3D_面板

////
// RigidBot Panel V1.0//RigidBot面板V1.0
// http://www.inventapart.com/// http://www.inventapart.com/
////
//#define RIGIDBOT_PANEL//#“定义刚体”面板

////
// Makeboard 3D Printer Parts 3D Printer Mini Display 1602 Mini Controller//Makeboard 3D打印机部件3D打印机迷你显示器1602迷你控制器
// https://www.aliexpress.com/item/32765887917.html// https://www.aliexpress.com/item/32765887917.html
////
//#define MAKEBOARD_MINI_2_LINE_DISPLAY_1602//#定义MAKEBOARD_MINI_2_LINE_DISPLAY_1602

////
// ANET and Tronxy 20x4 Controller//ANET和Tronxy 20x4控制器
////
//#define ZONESTAR_LCD            // Requires ADC_KEYPAD_PIN to be assigned to an analog pin.//#定义ZONESTAR_LCD//要求将ADC_键盘_引脚分配给模拟引脚。
// This LCD is known to be susceptible to electrical interference//已知该LCD易受电气干扰影响
// which scrambles the display.  Pressing any button clears it up.//这会扰乱显示。按下任何按钮都可以清除它。
// This is a LCD2004 display with 5 analog buttons.//这是一个带有5个模拟按钮的LCD2004显示器。

////
// Generic 16x2, 16x4, 20x2, or 20x4 character-based LCD.//基于字符的通用16x2、16x4、20x2或20x4液晶显示器。
////
//#define ULTRA_LCD//#定义ULTRA_LCD

//=============================================================================//=============================================================================
//======================== LCD / Controller Selection =========================//========================================LCD/控制器选择=========================
//=====================   (I2C and Shift-Register LCDs)   =====================//=========================（I2C和移位寄存器LCD）=====================
//=============================================================================//=============================================================================

////
// CONTROLLER TYPE: I2C//控制器类型：I2C
////
// Note: These controllers require the installation of Arduino's LiquidCrystal_I2C//注：这些控制器需要安装Arduino的Liquidcystal_I2C
// library. For more info: https://github.com/kiyoshigawa/LiquidCrystal_I2C//图书馆。有关更多信息：https://github.com/kiyoshigawa/LiquidCrystal_I2C
////

////
// Elefu RA Board Control Panel//Elefu RA板控制面板
// http://www.elefu.com/index.php?route=product/product&product_id=53// http://www.elefu.com/index.php?route=product/product&product_id=53
////
//#define RA_CONTROL_PANEL//#定义RA_控制面板

////
// Sainsmart (YwRobot) LCD Displays//赛因斯马特（YwRobot）液晶显示器
////
// These require F.Malpartida's LiquidCrystal_I2C library//这些需要F.Malpartida的Liquidcystal_I2C库
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
////
//#define LCD_SAINSMART_I2C_1602//#定义LCD_SAINSMART_I2C_1602
//#define LCD_SAINSMART_I2C_2004//#定义LCD_SAINSMART_I2C_2004

////
// Generic LCM1602 LCD adapter//通用LCM1602 LCD适配器
////
//#define LCM1602//#定义LCM1602

////
// PANELOLU2 LCD with status LEDs,//带状态LED的PANELOLU2 LCD，
// separate encoder and click inputs.//分离编码器并单击输入。
////
// Note: This controller requires Arduino's LiquidTWI2 library v1.2.3 or later.//注意：此控制器需要Arduino的LiquidTWI2库v1.2.3或更高版本。
// For more info: https://github.com/lincomatic/LiquidTWI2//有关更多信息：https://github.com/lincomatic/LiquidTWI2
////
// Note: The PANELOLU2 encoder click input can either be directly connected to//注：PANELOLU2编码器点击输入可以直接连接到
// a pin (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).//引脚（如果BTN_ENC定义为！=-1）或通过I2C读取（当BTN_ENC==-1时）。
////
//#define LCD_I2C_PANELOLU2//#定义LCD_I2C_面板2

////
// Panucatt VIKI LCD with status LEDs,//带状态LED的Panucatt VIKI LCD，
// integrated click & L/R/U/D buttons, separate encoder inputs.//集成的点击和L/R/U/D按钮，单独的编码器输入。
////
//#define LCD_I2C_VIKI//#定义LCD_I2C_VIKI

////
// CONTROLLER TYPE: Shift register panels//控制器类型：移位寄存器面板
////

////
// 2-wire Non-latching LCD SR from https://goo.gl/aJJ4sH//2线非闭锁式液晶显示器SR从https://goo.gl/aJJ4sH
// LCD configuration: https://reprap.org/wiki/SAV_3D_LCD//LCD配置：https://reprap.org/wiki/SAV_3D_LCD
////
//#define SAV_3DLCD//#定义SAV_3DLCD

////
// 3-wire SR LCD with strobe using 74HC4094//使用74HC4094的带选通的3线SR LCD
// https://github.com/mikeshub/SailfishLCD// https://github.com/mikeshub/SailfishLCD
// Uses the code directly from Sailfish//直接使用来自Sailfish的代码
////
//#define FF_INTERFACEBOARD//#定义FF_接口板

////
// TFT GLCD Panel with Marlin UI//带Marlin UI的TFT GLCD面板
// Panel connected to main board by SPI or I2C interface.//面板通过SPI或I2C接口连接至主板。
// See https://github.com/Serhiy-K/TFTGLCDAdapter//看https://github.com/Serhiy-K/TFTGLCDAdapter
////
//#define TFTGLCD_PANEL_SPI//#定义TFTGLCD_面板_SPI
//#define TFTGLCD_PANEL_I2C//#定义TFTGLCD_面板_I2C

//=============================================================================//=============================================================================
//=======================   LCD / Controller Selection  =======================//=====================================LCD/控制器选择=======================
//=========================      (Graphical LCDs)      ========================//=====================================（图形LCD）========================
//=============================================================================//=============================================================================

////
// CONTROLLER TYPE: Graphical 128x64 (DOGM)//控制器类型：图形128x64（DOGM）
////
// IMPORTANT: The U8glib library is required for Graphical Display!//重要提示：图形显示需要U8glib库！
//            https://github.com/olikraus/U8glib_Arduino//            https://github.com/olikraus/U8glib_Arduino
////
// NOTE: If the LCD is unresponsive you may need to reverse the plugs.//注意：如果LCD没有响应，您可能需要反转插头。
////

////
// RepRapDiscount FULL GRAPHIC Smart Controller//RepRapDiscount全图形智能控制器
// https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller// https://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
////
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER//#定义REPRAP\u折扣\u完整\u图形\u智能\u控制器

////
// K.3D Full Graphic Smart Controller//K.3D全图形智能控制器
////
//#define K3D_FULL_GRAPHIC_SMART_CONTROLLER//#定义K3D\u完整\u图形\u智能\u控制器

////
// ReprapWorld Graphical LCD//ReprapWorld图形液晶显示器
// https://reprapworld.com/?products_details&products_id/1218// https://reprapworld.com/?products_details&products_id/1218
////
//#define REPRAPWORLD_GRAPHICAL_LCD//#定义REPRAPWORLD_图形_LCD

////
// Activate one of these if you have a Panucatt Devices//如果您有Panucatt设备，请激活其中一个
// Viki 2.0 or mini Viki with Graphic LCD//Viki 2.0或带图形液晶显示器的迷你Viki
// https://www.panucatt.com// https://www.panucatt.com
////
//#define VIKI2//#定义VIKI2
//#define miniVIKI//#定义miniVIKI

////
// MakerLab Mini Panel with graphic//MakerLab迷你面板，带有图形
// controller and SD support - https://reprap.org/wiki/Mini_panel//控制器和SD支持-https://reprap.org/wiki/Mini_panel
////
//#define MINIPANEL//#定义迷你面板

////
// MaKr3d Makr-Panel with graphic controller and SD support.//带图形控制器和SD支持的MaKr3d Makr面板。
// https://reprap.org/wiki/MaKr3d_MaKrPanel// https://reprap.org/wiki/MaKr3d_MaKrPanel
////
//#define MAKRPANEL//#定义MAKRPANEL

////
// Adafruit ST7565 Full Graphic Controller.//Adafruit ST7565全图形控制器。
// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/// https://github.com/eboston/Adafruit-ST7565-Full-Graphic-Controller/
////
//#define ELB_FULL_GRAPHIC_CONTROLLER//#定义ELB\U FULL\U图形\U控制器

////
// BQ LCD Smart Controller shipped by//BQ LCD智能控制器由制造商提供
// default with the BQ Hephestos 2 and Witbox 2.//默认为BQ Hephestos 2和Witbox 2。
////
//#define BQ_LCD_SMART_CONTROLLER//#定义BQ\U LCD\U智能\U控制器

////
// Cartesio UI//卡特西奥
// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface// http://mauk.cc/webshop/cartesio-shop/electronics/user-interface
////
//#define CARTESIO_UI//#定义用户界面

////
// LCD for Melzi Card with Graphical LCD//带图形LCD的Melzi卡LCD
////
//#define LCD_FOR_MELZI//#为MELZI定义LCD_

////
// Original Ulticontroller from Ultimaker 2 printer with SSD1309 I2C display and encoder//Ultimaker 2打印机的原装Ultimaker控制器，带有SSD1309 I2C显示器和编码器
// https://github.com/Ultimaker/Ultimaker2/tree/master/1249_Ulticontroller_Board_(x1)// https://github.com/Ultimaker/Ultimaker2/tree/master/1249_Ulticontroller_Board_（x1）
////
//#define ULTI_CONTROLLER//#定义ULTI_控制器

////
// MKS MINI12864 with graphic controller and SD support//带有图形控制器和SD支持的MKS MINI12864
// https://reprap.org/wiki/MKS_MINI_12864// https://reprap.org/wiki/MKS_MINI_12864
////
//#define MKS_MINI_12864//#定义MKS_MINI_12864

////
// MKS MINI12864 V3 is an alias for FYSETC_MINI_12864_2_1. Type A/B. NeoPixel RGB Backlight.//MKS MINI12864 V3是FYSETC_MINI_12864_2_1的别名。A/B型。Neopix RGB背光。
////
//#define MKS_MINI_12864_V3//#定义MKS_MINI_12864_V3

////
// MKS LCD12864A/B with graphic controller and SD support. Follows MKS_MINI_12864 pinout.//带有图形控制器和SD支持的MKS LCD12864A/B。遵循MKS_MINI_12864引脚。
// https://www.aliexpress.com/item/33018110072.html// https://www.aliexpress.com/item/33018110072.html
////
//#define MKS_LCD12864A//#定义MKS_LCD12864A
//#define MKS_LCD12864B//#定义MKS_LCD12864B

////
// FYSETC variant of the MINI12864 graphic controller with SD support//支持SD的MINI12864图形控制器的FYSETC变体
// https://wiki.fysetc.com/Mini12864_Panel/// https://wiki.fysetc.com/Mini12864_Panel/
////
//#define FYSETC_MINI_12864_X_X    // Type C/D/E/F. No tunable RGB Backlight by default//#定义FYSETC_MINI_12864_X_X//类型C/D/E/F。默认情况下无可调RGB背光
//#define FYSETC_MINI_12864_1_2    // Type C/D/E/F. Simple RGB Backlight (always on)//#定义FYSETC_MINI_12864_1_2//类型C/D/E/F。简单RGB背光（始终打开）
//#define FYSETC_MINI_12864_2_0    // Type A/B. Discreet RGB Backlight//#定义FYSETC_MINI_12864_2_0//类型A/B。谨慎的RGB背光
//#define FYSETC_MINI_12864_2_1    // Type A/B. NeoPixel RGB Backlight//#定义FYSETC_MINI_12864_2_1//类型A/B。Neopix RGB背光
//#define FYSETC_GENERIC_12864_1_1 // Larger display with basic ON/OFF backlight.//#定义FYSETC_GENERIC_12864_1_1//带基本开/关背光的更大显示屏。

////
// Factory display for Creality CR-10//Creality CR-10的工厂显示器
// https://www.aliexpress.com/item/32833148327.html// https://www.aliexpress.com/item/32833148327.html
////
// This is RAMPS-compatible using a single 10-pin connector.//这是使用单个10针连接器的坡道兼容。
// (For CR-10 owners who want to replace the Melzi Creality board but retain the display)//（适用于希望更换Melzi Creality董事会但保留显示器的CR-10所有者）
////
//#define CR10_STOCKDISPLAY//#定义CR10_库存显示

////
// Ender-2 OEM display, a variant of the MKS_MINI_12864//Ender-2 OEM显示器，MKS_MINI_12864的一种变体
////
//#define ENDER2_STOCKDISPLAY//#定义ENDER2\u STOCKDISPLAY

////
// ANET and Tronxy Graphical Controller//ANET和Tronxy图形控制器
////
// Anet 128x64 full graphics lcd with rotary encoder as used on Anet A6//Anet A6上使用的带旋转编码器的Anet 128x64全图形lcd
// A clone of the RepRapDiscount full graphics display but with//RepraDiscount完整图形显示的克隆，但带有
// different pins/wiring (see pins_ANET_10.h). Enable one of these.//不同的引脚/接线（参见引脚10.h）。启用其中一个。
////
//#define ANET_FULL_GRAPHICS_LCD//#定义一个完整的图形液晶显示器
//#define ANET_FULL_GRAPHICS_LCD_ALT_WIRING//#定义网络、完整图形、LCD、ALT接线

////
// AZSMZ 12864 LCD with SD//带SD的AZSMZ 12864液晶显示器
// https://www.aliexpress.com/item/32837222770.html// https://www.aliexpress.com/item/32837222770.html
////
//#define AZSMZ_12864//#定义AZSMZ_12864

////
// Silvergate GLCD controller//Silvergate GLCD控制器
// https://github.com/android444/Silvergate// https://github.com/android444/Silvergate
////
//#define SILVER_GATE_GLCD_CONTROLLER//#定义银色门GLCD控制器

//=============================================================================//=============================================================================
//==============================  OLED Displays  ==============================//===================================================================OLED显示器==============================
//=============================================================================//=============================================================================

////
// SSD1306 OLED full graphics generic display//SSD1306 OLED全图形通用显示器
////
//#define U8GLIB_SSD1306//#定义U8GLIB_SSD1306

////
// SAV OLEd LCD module support using either SSD1306 or SH1106 based LCD modules//使用基于SSD1306或SH1106的LCD模块支持SAV OLEd LCD模块
////
//#define SAV_3DGLCD//#定义SAV_3DGLCD
#if ENABLED(SAV_3DGLCD)
#define U8GLIB_SSD1306
  //#define U8GLIB_SH1106//#定义U8GLIB_SH1106
#endif

////
// TinyBoy2 128x64 OLED / Encoder Panel//TinyBoy2 128x64 OLED/编码器面板
////
//#define OLED_PANEL_TINYBOY2//#定义OLED_面板_TINYBOY2

////
// MKS OLED 1.3" 128×64 Full Graphics Controller//MKS OLED 1.3英寸128×64全图形控制器
// https://reprap.org/wiki/MKS_12864OLED// https://reprap.org/wiki/MKS_12864OLED
////
// Tiny, but very sharp OLED display//小巧但非常锐利的OLED显示屏
////
//#define MKS_12864OLED          // Uses the SH1106 controller (default)//#定义MKS_12864OLED//使用SH1106控制器（默认）
//#define MKS_12864OLED_SSD1306  // Uses the SSD1306 controller//#定义MKS_12864OLED_SSD1306//使用SSD1306控制器

////
// Zonestar OLED 128×64 Full Graphics Controller//Zonestar OLED 128×64全图形控制器
////
//#define ZONESTAR_12864LCD           // Graphical (DOGM) with ST7920 controller//#使用ST7920控制器定义ZONESTAR_12864LCD//图形（DOGM）
//#define ZONESTAR_12864OLED          // 1.3" OLED with SH1106 controller (default)//#使用SH1106控制器定义ZONESTAR_12864; OLED//1.3“OLED（默认）
//#define ZONESTAR_12864OLED_SSD1306  // 0.96" OLED with SSD1306 controller//#使用SSD1306控制器定义ZONESTAR_12864OLED_SSD1306//0.96“OLED

////
// Einstart S OLED SSD1306//Einstart S OLED SSD1306
////
//#define U8GLIB_SH1106_EINSTART//#定义U8GLIB_SH1106_EINSTART

////
// Overlord OLED display/controller with i2c buzzer and LEDs//带i2c蜂鸣器和LED的霸王OLED显示器/控制器
////
//#define OVERLORD_OLED//#定义霸王OLED

////
// FYSETC OLED 2.42" 128×64 Full Graphics Controller with WS2812 RGB//FYSETC OLED 2.42英寸128×64全图形控制器，带WS2812 RGB
// Where to find : https://www.aliexpress.com/item/4000345255731.html//在哪里可以找到：https://www.aliexpress.com/item/4000345255731.html
//#define FYSETC_242_OLED_12864   // Uses the SSD1309 controller//#定义FYSETC_242_OLED_12864//使用SSD1309控制器

////
// K.3D SSD1309 OLED 2.42" 128×64 Full Graphics Controller//K.3D SSD1309 OLED 2.42英寸128×64全图形控制器
////
//#define K3D_242_OLED_CONTROLLER   // Software SPI//#定义K3D_242_OLED_控制器//软件SPI

//=============================================================================//=============================================================================
//========================== Extensible UI Displays ===========================//===========================================可扩展UI显示===========================
//=============================================================================//=============================================================================

////
// DGUS Touch Display with DWIN OS. (Choose one.)//带有DWIN操作系统的DGUS触摸屏。（选择一个。）
// ORIGIN : https://www.aliexpress.com/item/32993409517.html//来源：https://www.aliexpress.com/item/32993409517.html
// FYSETC : https://www.aliexpress.com/item/32961471929.html//FYSETC：https://www.aliexpress.com/item/32961471929.html
// MKS    : https://www.aliexpress.com/item/1005002008179262.html//MKS：https://www.aliexpress.com/item/1005002008179262.html
////
// Flash display with DGUS Displays for Marlin://用于Marlin的带DGUS显示器的闪光显示器：
//  - Format the SD card to FAT32 with an allocation size of 4kb.//-将SD卡格式化为FAT32，分配大小为4kb。
//  - Download files as specified for your type of display.//-下载为您的显示器类型指定的文件。
//  - Plug the microSD card into the back of the display.//-将microSD卡插入显示器背面。
//  - Boot the display and wait for the update to complete.//-启动显示器并等待更新完成。
////
// ORIGIN (Marlin DWIN_SET)//原点（马林德温集）
//  - Download https://github.com/coldtobi/Marlin_DGUS_Resources//-下载https://github.com/coldtobi/Marlin_DGUS_Resources
//  - Copy the downloaded DWIN_SET folder to the SD card.//-将下载的DWIN_SET文件夹复制到SD卡。
////
// FYSETC (Supplier default)//FYSETC（供应商默认）
//  - Download https://github.com/FYSETC/FYSTLCD-2.0//-下载https://github.com/FYSETC/FYSTLCD-2.0
//  - Copy the downloaded SCREEN folder to the SD card.//-将下载的屏幕文件夹复制到SD卡。
////
// HIPRECY (Supplier default)//HIPRECY（供应商默认）
//  - Download https://github.com/HiPrecy/Touch-Lcd-LEO//-下载https://github.com/HiPrecy/Touch-Lcd-LEO
//  - Copy the downloaded DWIN_SET folder to the SD card.//-将下载的DWIN_SET文件夹复制到SD卡。
////
// MKS (MKS-H43) (Supplier default)//MKS（MKS-H43）（供应商默认）
//  - Download https://github.com/makerbase-mks/MKS-H43//-下载https://github.com/makerbase-mks/MKS-H43
//  - Copy the downloaded DWIN_SET folder to the SD card.//-将下载的DWIN_SET文件夹复制到SD卡。
////
// RELOADED (T5UID1)//重新加载（T5UID1）
//  - Download https://github.com/Desuuuu/DGUS-reloaded/releases//-下载https://github.com/Desuuuu/DGUS-reloaded/releases
//  - Copy the downloaded DWIN_SET folder to the SD card.//-将下载的DWIN_SET文件夹复制到SD卡。
////
//#define DGUS_LCD_UI_ORIGIN//#定义DGUS_LCD_UI_原点
//#define DGUS_LCD_UI_FYSETC//#定义DGUS_LCD_UI_FYSETC
//#define DGUS_LCD_UI_HIPRECY//#定义DGU_LCD_UI_HIPRECY
//#define DGUS_LCD_UI_MKS//#定义DGUS_LCD_UI_MKS
//#define DGUS_LCD_UI_RELOADED//#定义DGUS_LCD_UI_重新加载
#if ENABLED(DGUS_LCD_UI_MKS)
#define USE_MKS_GREEN_UI
#endif

////
// Touch-screen LCD for Malyan M200/M300 printers//用于Malyan M200/M300打印机的触摸屏LCD
////
//#define MALYAN_LCD//#定义MALYAN_LCD
#if ENABLED(MALYAN_LCD)
#define LCD_SERIAL_PORT 1  // Default is 1 for Malyan M200//Malyan M200的默认值为1
#endif

////
// Touch UI for FTDI EVE (FT800/FT810) displays//FTDI EVE（FT800/FT810）显示的触摸式用户界面
// See Configuration_adv.h for all configuration options.//有关所有配置选项，请参阅Configuration_adv.h。
////
//#define TOUCH_UI_FTDI_EVE//#定义触摸屏和FTDI EVE

////
// Touch-screen LCD for Anycubic printers//触摸屏LCD，适用于任意立方打印机
////
//#define ANYCUBIC_LCD_I3MEGA//#定义任意立方\u LCD\u I3MEGA
//#define ANYCUBIC_LCD_CHIRON//#定义任意立方\u液晶\u凯龙
#if EITHER(ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON)
#define LCD_SERIAL_PORT 3  // Default is 3 for Anycubic//Anycubic的默认值为3
  //#define ANYCUBIC_LCD_DEBUG//#定义任意立方体\u LCD\u调试
#endif

////
// 320x240 Nextion 2.8" serial TFT Resistive Touch Screen NX3224T028//320x240 Nextion 2.8英寸串行TFT电阻式触摸屏NX3224T028
////
//#define NEXTION_TFT//#定义NEXTION\u TFT
#if ENABLED(NEXTION_TFT)
#define LCD_SERIAL_PORT 1  // Default is 1 for Nextion//Nextion的默认值为1
#endif

////
// Third-party or vendor-customized controller interfaces.//第三方或供应商定制的控制器接口。
// Sources should be installed in 'src/lcd/extui'.//源应安装在“src/lcd/extui”中。
////
//#define EXTENSIBLE_UI//#定义可扩展的用户界面

#if ENABLED(EXTENSIBLE_UI)
//#define EXTUI_LOCAL_BEEPER // Enables use of local Beeper pin with external display//#定义EXTUI_LOCAL_蜂鸣器//启用带外部显示的本地蜂鸣器pin
#endif

//=============================================================================//=============================================================================
//=============================== Graphical TFTs ==============================//=======================================================图形TFT==============================
//=============================================================================//=============================================================================

/**
 * Specific TFT Model Presets. Enable one of the following options
 * or enable TFT_GENERIC and set sub-options.
 */

////
// 480x320, 3.5", SPI Display From MKS//来自MKS的480x320，3.5英寸SPI显示屏
// Normally used in MKS Robin Nano V2//通常用于MKS Robin Nano V2
////
//#define MKS_TS35_V2_0//#定义MKS_TS35_V2_0

////
// 320x240, 2.4", FSMC Display From MKS//MKS的320x240，2.4英寸FSMC显示屏
// Normally used in MKS Robin Nano V1.2//通常用于MKS Robin Nano V1.2
////
//#define MKS_ROBIN_TFT24//#定义MKS_ROBIN_TFT24

////
// 320x240, 2.8", FSMC Display From MKS//MKS的320x240，2.8英寸FSMC显示屏
// Normally used in MKS Robin Nano V1.2//通常用于MKS Robin Nano V1.2
////
//#define MKS_ROBIN_TFT28//#定义MKS_ROBIN_TFT28

////
// 320x240, 3.2", FSMC Display From MKS//MKS的320x240，3.2英寸FSMC显示屏
// Normally used in MKS Robin Nano V1.2//通常用于MKS Robin Nano V1.2
////
//#define MKS_ROBIN_TFT32//#定义MKS_ROBIN_TFT32

////
// 480x320, 3.5", FSMC Display From MKS//来自MKS的480x320，3.5英寸FSMC显示屏
// Normally used in MKS Robin Nano V1.2//通常用于MKS Robin Nano V1.2
////
//#define MKS_ROBIN_TFT35//#定义MKS_ROBIN_TFT35

////
// 480x272, 4.3", FSMC Display From MKS//来自MKS的480x272，4.3英寸FSMC显示屏
////
//#define MKS_ROBIN_TFT43//#定义MKS_ROBIN_TFT43

////
// 320x240, 3.2", FSMC Display From MKS//MKS的320x240，3.2英寸FSMC显示屏
// Normally used in MKS Robin//通常用于MKS Robin
////
//#define MKS_ROBIN_TFT_V1_1R//#定义MKS_ROBIN_TFT_V1_1R

////
// 480x320, 3.5", FSMC Stock Display from TronxXY//来自TronxXY的480x320，3.5英寸FSMC库存显示器
////
//#define TFT_TRONXY_X5SA//#定义TFT_TRONXY_X5SA

////
// 480x320, 3.5", FSMC Stock Display from AnyCubic//来自AnyCubic的480x320，3.5英寸FSMC库存显示器
////
//#define ANYCUBIC_TFT35//#定义任意三次方

////
// 320x240, 2.8", FSMC Stock Display from Longer/Alfawise//320x240，2.8英寸，FSMC库存显示，从长/短
////
//#define LONGER_LK_TFT28//#定义更长的\u LK\u TFT28

////
// 320x240, 2.8", FSMC Stock Display from ET4//ET4的320x240，2.8英寸FSMC库存显示器
////
//#define ANET_ET4_TFT28//#定义ANET_ET4_TFT28

////
// 480x320, 3.5", FSMC Stock Display from ET5//ET5的480x320，3.5英寸FSMC库存显示器
////
//#define ANET_ET5_TFT35//#定义ANET_ET5_TFT35

////
// 1024x600, 7", RGB Stock Display from BIQU-BX//BIQU-BX的1024x600，7英寸RGB库存显示器
////
//#define BIQU_BX_TFT70//#定义BIQU_BX_TFT70

////
// Generic TFT with detailed options//具有详细选项的通用TFT
////
//#define TFT_GENERIC//#定义TFT_泛型
#if ENABLED(TFT_GENERIC)
// :[ 'AUTO', 'ST7735', 'ST7789', 'ST7796', 'R61505', 'ILI9328', 'ILI9341', 'ILI9488' ]//：[‘自动’、‘ST7735’、‘ST7789’、‘ST7796’、‘R61505’、‘ILI9328’、‘ILI9341’、‘ILI9488’]
  #define TFT_DRIVER AUTO

  // Interface. Enable one of the following options://接口。启用以下选项之一：
  //#define TFT_INTERFACE_FSMC//#定义TFT\U接口\U FSMC
  //#define TFT_INTERFACE_SPI//#定义TFT_接口_SPI

  // TFT Resolution. Enable one of the following options://TFT分辨率。启用以下选项之一：
  //#define TFT_RES_320x240//#定义TFT_RES_320x240
  //#define TFT_RES_480x272//#定义TFT_RES_480x272
  //#define TFT_RES_480x320//#定义TFT_RES_480x320
  //#define TFT_RES_1024x600//#定义TFT_RES_1024x600
#endif

/**
 * TFT UI - User Interface Selection. Enable one of the following options:
 *
 *   TFT_CLASSIC_UI - Emulated DOGM - 128x64 Upscaled
 *   TFT_COLOR_UI   - Marlin Default Menus, Touch Friendly, using full TFT capabilities
 *   TFT_LVGL_UI    - A Modern UI using LVGL
 *
 *   For LVGL_UI also copy the 'assets' folder from the build directory to the
 *   root of your SD card, together with the compiled firmware.
 */
//#define TFT_CLASSIC_UI//#定义TFT_经典_用户界面
//#define TFT_COLOR_UI//#定义TFT_颜色_用户界面
//#define TFT_LVGL_UI//#定义TFT\U LVGL\U用户界面

#if ENABLED(TFT_LVGL_UI)
//#define MKS_WIFI_MODULE  // MKS WiFi module//#定义MKS_WIFI_模块//MKS WIFI模块
#endif

/**
 * TFT Rotation. Set to one of the following values:
 *
 *   TFT_ROTATE_90,  TFT_ROTATE_90_MIRROR_X,  TFT_ROTATE_90_MIRROR_Y,
 *   TFT_ROTATE_180, TFT_ROTATE_180_MIRROR_X, TFT_ROTATE_180_MIRROR_Y,
 *   TFT_ROTATE_270, TFT_ROTATE_270_MIRROR_X, TFT_ROTATE_270_MIRROR_Y,
 *   TFT_MIRROR_X, TFT_MIRROR_Y, TFT_NO_ROTATION
 */
//#define TFT_ROTATION TFT_NO_ROTATION//#定义TFT\U旋转TFT\U无旋转

//=============================================================================//=============================================================================
//============================  Other Controllers  ============================//===========================================其他控制器============================
//=============================================================================//=============================================================================

////
// Ender-3 v2 OEM display. A DWIN display with Rotary Encoder.//Ender-3 v2 OEM显示器。带旋转编码器的DWN显示器。
////
//#define DWIN_CREALITY_LCD//#定义DWIN_CREALITY_LCD

////
// Ender-3 v2 OEM display, enhanced.//Ender-3 v2 OEM显示屏，增强型。
////
//#define DWIN_CREALITY_LCD_ENHANCED//#定义DWIN\u CREALITY\u LCD\u增强

////
// Ender-3 v2 OEM display with enhancements by Jacob Myers//Ender-3 v2 OEM显示屏，由Jacob Myers提供增强功能
////
//#define DWIN_CREALITY_LCD_JYERSUI//#定义DWIN\U CREALITY\U LCD\U JYERSUI

////
// MarlinUI for Creality's DWIN display (and others)//用于Crealice的DWN显示的MarlinUI（和其他）
////
//#define DWIN_MARLINUI_PORTRAIT//#定义DWIN_MARLINUI_肖像
//#define DWIN_MARLINUI_LANDSCAPE//#定义德温马尔利尼景观

////
// Touch Screen Settings//触摸屏设置
////
//#define TOUCH_SCREEN//#定义触摸屏
#if ENABLED(TOUCH_SCREEN)
#define BUTTON_DELAY_EDIT  50 // (ms) Button repeat delay for edit screens//（ms）编辑屏幕的按钮重复延迟
  #define BUTTON_DELAY_MENU 250 // (ms) Button repeat delay for menus//（ms）菜单的按钮重复延迟

  //#define TOUCH_IDLE_SLEEP 300 // (secs) Turn off the TFT backlight if set (5mn)//#定义触摸\空闲\睡眠300/（秒）关闭TFT背光（如果设置）（5mn）

  #define TOUCH_SCREEN_CALIBRATION

  //#define TOUCH_CALIBRATION_X 12316//#定义触摸校准×12316
  //#define TOUCH_CALIBRATION_Y -8981//#定义触摸校准Y-8981
  //#define TOUCH_OFFSET_X        -43//#定义触摸偏移量\u X-43
  //#define TOUCH_OFFSET_Y        257//#定义触摸偏移量257
  //#define TOUCH_ORIENTATION TOUCH_LANDSCAPE//#定义触摸方向触摸景观

  #if BOTH(TOUCH_SCREEN_CALIBRATION, EEPROM_SETTINGS)
    #define TOUCH_CALIBRATION_AUTO_SAVE // Auto save successful calibration values to EEPROM//自动将成功校准值保存到EEPROM
  #endif

  #if ENABLED(TFT_COLOR_UI)
    //#define SINGLE_TOUCH_NAVIGATION//#定义单触式导航
  #endif
#endif

////
// RepRapWorld REPRAPWORLD_KEYPAD v1.1//RepRapWorld RepRapWorld_键盘v1.1
// https://reprapworld.com/products/electronics/ramps/keypad_v1_0_fully_assembled/// https://reprapworld.com/products/electronics/ramps/keypad_v1_0_fully_assembled/
////
//#define REPRAPWORLD_KEYPAD//#定义REPRAPWORLD_键盘
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // (mm) Distance to move per key-press//#定义REPRAPWORLD_键盘_移动_步骤10.0//（mm）每次按键移动的距离

//=============================================================================//=============================================================================
//=============================== Extra Features ==============================//=======================================================附加功能==============================
//=============================================================================//=============================================================================

// @section extras//@额外部分

// Set number of user-controlled fans. Disable to use all board-defined fans.//设置用户控制风扇的数量。禁用以使用所有板定义的风扇。
// :[1,2,3,4,5,6,7,8]// :[1,2,3,4,5,6,7,8]
//#define NUM_M106_FANS 1//#定义数量M106风扇1

// Increase the FAN PWM frequency. Removes the PWM noise but increases heating in the FET/Arduino//增加风扇PWM频率。消除PWM噪声，但增加FET/Arduino中的加热
//#define FAST_PWM_FAN//#定义快速PWM风扇

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency//使用软件PWM驱动风扇，如加热器。这使用非常低的频率
// which is not as annoying as with the hardware PWM. On the other hand, if this frequency//这并不像硬件PWM那样烦人。另一方面，如果这个频率
// is too low, you should also increment SOFT_PWM_SCALE.//如果太低，还应增加软_PWM_比例。
//#define FAN_SOFT_PWM//#定义风扇\u软\u PWM

// Incrementing this by 1 will double the software PWM frequency,//将其增加1将使软件PWM频率加倍，
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.//如果启用风扇软PWM，则影响加热器和风扇。
// However, control resolution will be halved for each increment;//但是，每增加一次，控制分辨率将减半；
// at zero value, there are 128 effective control positions.//零值时，有128个有效控制位置。
// :[0,1,2,3,4,5,6,7]// :[0,1,2,3,4,5,6,7]
#define SOFT_PWM_SCALE 0

// If SOFT_PWM_SCALE is set to a value higher than 0, dithering can//如果SOFT_PWM_SCALE设置为大于0的值，则可能会出现抖动
// be used to mitigate the associated resolution loss. If enabled,//用于减轻相关的分辨率损失。如果启用，
// some of the PWM cycles are stretched so on average the desired//一些PWM周期被拉伸，因此平均达到所需的
// duty cycle is attained.//达到了占空比。
//#define SOFT_PWM_DITHER//#定义软脉冲宽度调制抖动

// Temperature status LEDs that display the hotend and bed temperature.//显示热端和床温的温度状态LED。
// If all hotends, bed temperature, and target temperature are under 54C//如果所有热端、床温和目标温度低于54℃
// then the BLUE led is on. Otherwise the RED led is on. (1C hysteresis)//然后蓝色led亮起。否则，红色指示灯亮起。（1C滞后）
//#define TEMP_STAT_LEDS//#定义临时状态指示灯

// Support for the BariCUDA Paste Extruder//BariCUDA糊料挤出机的支架
//#define BARICUDA//#定义巴里库达

// Support for BlinkM/CyzRgb//支持BlinkM/CyzRgb
//#define BLINKM//#定义BLINKM

// Support for PCA9632 PWM LED driver//支持PCA9632 PWM LED驱动器
//#define PCA9632//#定义PCA9632

// Support for PCA9533 PWM LED driver//支持PCA9533 PWM LED驱动器
//#define PCA9533//#定义PCA9533

/**
 * RGB LED / LED Strip Control
 *
 * Enable support for an RGB LED connected to 5V digital pins, or
 * an RGB Strip connected to MOSFETs controlled by digital pins.
 *
 * Adds the M150 command to set the LED (or LED strip) color.
 * If pins are PWM capable (e.g., 4, 5, 6, 11) then a range of
 * luminance values can be set from 0 to 255.
 * For NeoPixel LED an overall brightness parameter is also available.
 *
 * *** CAUTION ***
 *  LED Strips require a MOSFET Chip between PWM lines and LEDs,
 *  as the Arduino cannot handle the current the LEDs will require.
 *  Failure to follow this precaution can destroy your Arduino!
 *  NOTE: A separate 5V power supply is required! The NeoPixel LED needs
 *  more current than the Arduino 5V linear regulator can produce.
 * *** CAUTION ***
 *
 * LED Type. Enable only one of the following two options.
 */
//#define RGB_LED//#定义RGB_发光二极管
//#define RGBW_LED//#定义RGBW_发光二极管

#if EITHER(RGB_LED, RGBW_LED)
//#define RGB_LED_R_PIN 34//#定义RGB_LED_R_引脚34
  //#define RGB_LED_G_PIN 43//#定义RGB_LED_G_引脚43
  //#define RGB_LED_B_PIN 35//#定义RGB_LED__引脚35
  //#define RGB_LED_W_PIN -1//#定义RGB_LED_W_引脚-1
#endif

// Support for Adafruit NeoPixel LED driver//支持Adafruit Neopix LED驱动程序
//#define NEOPIXEL_LED//#定义NEOPIXEL_LED
#if ENABLED(NEOPIXEL_LED)
#define NEOPIXEL_TYPE   NEO_GRBW // NEO_GRBW / NEO_GRB - four/three channel driver type (defined in Adafruit_NeoPixel.h)//NEO_GRBW/NEO_GRB-四通道/三通道驱动器类型（在Adafruit_Neopix.h中定义）
  //#define NEOPIXEL_PIN     4     // LED driving pin//#定义Neopix_引脚4//LED驱动引脚
  //#define NEOPIXEL2_TYPE NEOPIXEL_TYPE//#定义NEOPIXEL2\u类型NEOPIXEL\u类型
  //#define NEOPIXEL2_PIN    5//#定义NEOPIXEL2_引脚5
  #define NEOPIXEL_PIXELS 30       // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is disabled.)//条带中的LED数量。（禁用NEOPIXEL2_分离时的最长条带。）
  #define NEOPIXEL_IS_SEQUENTIAL   // Sequential display for temperature change - LED by LED. Disable to change all LEDs at once.//温度变化顺序显示-LED逐LED显示。禁用一次更改所有指示灯。
  #define NEOPIXEL_BRIGHTNESS 127  // Initial brightness (0-255)//初始亮度（0-255）
  //#define NEOPIXEL_STARTUP_TEST  // Cycle through colors at startup//#定义NEOPIXEL_STARTUP_TEST//启动时循环颜色

  // Support for second Adafruit NeoPixel LED driver controlled with M150 S1 ...//支持第二个Adafruit NeoPixel LED驱动器，由M150 S1控制。。。
  //#define NEOPIXEL2_SEPARATE//#定义NEOPIXEL2_单独
  #if ENABLED(NEOPIXEL2_SEPARATE)
    #define NEOPIXEL2_PIXELS      15  // Number of LEDs in the second strip//第二条带中的LED数量
    #define NEOPIXEL2_BRIGHTNESS 127  // Initial brightness (0-255)//初始亮度（0-255）
    #define NEOPIXEL2_STARTUP_TEST    // Cycle through colors at startup//启动时循环使用颜色
  #else
    //#define NEOPIXEL2_INSERIES      // Default behavior is NeoPixel 2 in parallel//#定义NEOPIXEL2\u插入序列//默认行为是NEOPIXEL2并行
  #endif

  // Use some of the NeoPixel LEDs for static (background) lighting//使用一些NeoPixel LED进行静态（背景）照明
  //#define NEOPIXEL_BKGD_INDEX_FIRST  0              // Index of the first background LED//#定义NEOPIXEL_BKGD_索引_第一个0//第一个背景LED的索引
  //#define NEOPIXEL_BKGD_INDEX_LAST   5              // Index of the last background LED//#定义NEOPIXEL_BKGD_索引_最后5//最后一个背景LED的索引
  //#define NEOPIXEL_BKGD_COLOR { 255, 255, 255, 0 }  // R, G, B, W//#定义新像素颜色{255，255，255，0}//R，G，B，W
  //#define NEOPIXEL_BKGD_ALWAYS_ON                   // Keep the backlight on when other NeoPixels are off//#定义NEOPIXEL_BKGD_ALWAYS_ON//在其他NEOPIXEL关闭时保持背光打开
#endif

/**
 * Printer Event LEDs
 *
 * During printing, the LEDs will reflect the printer status:
 *
 *  - Gradually change from blue to violet as the heated bed gets to target temp
 *  - Gradually change from violet to red as the hotend gets to temperature
 *  - Change to white to illuminate work surface
 *  - Change to green once print has finished
 *  - Turn off after the print has finished and the user has pushed a button
 */
#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
#define PRINTER_EVENT_LEDS
#endif

/**
 * Number of servos
 *
 * For some servo-related options NUM_SERVOS will be set automatically.
 * Set this manually if there are extra servos needing manual control.
 * Set to 0 to turn off servo support.
 */
//#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command//#定义NUM_SERVOS 3//对于M280命令，伺服索引从0开始

// (ms) Delay  before the next move will start, to give the servo time to reach its target angle.//（ms）下一步开始前的延迟，以给伺服时间达到其目标角度。
// 300ms is a good value but you can try less delay.//300ms是一个很好的值，但您可以尝试减少延迟。
// If the servo can't reach the requested position, increase it.//如果伺服无法达到要求的位置，则增加伺服。
#define SERVO_DELAY { 300 }

// Only power servos during movement, otherwise leave off to prevent jitter//在移动过程中，仅电源伺服，否则关闭以防止抖动
//#define DEACTIVATE_SERVOS_AFTER_MOVE//#定义移动后停用伺服

// Edit servo angles with M281 and save to EEPROM with M500//使用M281编辑伺服角度，并使用M500保存到EEPROM
//#define EDITABLE_SERVO_ANGLES//#定义可编辑的伺服角度
