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
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 */
#define CONFIGURATION_ADV_H_VERSION 02000901

//===========================================================================//===========================================================================
//============================= Thermal Settings ============================//=================================================热设置============================
//===========================================================================//===========================================================================
// @section temperature//@截面温度

/**
 * Thermocouple sensors are quite sensitive to noise.  Any noise induced in
 * the sensor wires, such as by stepper motor wires run in parallel to them,
 * may result in the thermocouple sensor reporting spurious errors.  This
 * value is the number of errors which can occur in a row before the error
 * is reported.  This allows us to ignore intermittent error conditions while
 * still detecting an actual failure, which should result in a continuous
 * stream of errors from the sensor.
 *
 * Set this value to 0 to fail on the first error to occur.
 */
#define THERMOCOUPLE_MAX_ERRORS 15

////
// Custom Thermistor 1000 parameters//自定义热敏电阻1000参数
////
#if TEMP_SENSOR_0 == 1000
#define HOTEND0_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND0_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND0_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_1 == 1000
#define HOTEND1_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND1_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND1_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_2 == 1000
#define HOTEND2_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND2_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND2_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_3 == 1000
#define HOTEND3_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND3_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND3_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_4 == 1000
#define HOTEND4_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND4_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND4_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_5 == 1000
#define HOTEND5_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND5_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND5_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_6 == 1000
#define HOTEND6_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND6_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND6_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_7 == 1000
#define HOTEND7_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define HOTEND7_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define HOTEND7_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_BED == 1000
#define BED_PULLUP_RESISTOR_OHMS     4700    // Pullup resistor//上拉电阻器
  #define BED_RESISTANCE_25C_OHMS      100000  // Resistance at 25C//25℃时的电阻
  #define BED_BETA                     3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_CHAMBER == 1000
#define CHAMBER_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define CHAMBER_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define CHAMBER_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_COOLER == 1000
#define COOLER_PULLUP_RESISTOR_OHMS 4700    // Pullup resistor//上拉电阻器
  #define COOLER_RESISTANCE_25C_OHMS  100000  // Resistance at 25C//25℃时的电阻
  #define COOLER_BETA                 3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_PROBE == 1000
#define PROBE_PULLUP_RESISTOR_OHMS   4700    // Pullup resistor//上拉电阻器
  #define PROBE_RESISTANCE_25C_OHMS    100000  // Resistance at 25C//25℃时的电阻
  #define PROBE_BETA                   3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_BOARD == 1000
#define BOARD_PULLUP_RESISTOR_OHMS   4700    // Pullup resistor//上拉电阻器
  #define BOARD_RESISTANCE_25C_OHMS    100000  // Resistance at 25C//25℃时的电阻
  #define BOARD_BETA                   3950    // Beta value//贝塔值
#endif

#if TEMP_SENSOR_REDUNDANT == 1000
#define REDUNDANT_PULLUP_RESISTOR_OHMS   4700    // Pullup resistor//上拉电阻器
  #define REDUNDANT_RESISTANCE_25C_OHMS    100000  // Resistance at 25C//25℃时的电阻
  #define REDUNDANT_BETA                   3950    // Beta value//贝塔值
#endif

/**
 * Configuration options for MAX Thermocouples (-2, -3, -5).
 *   FORCE_HW_SPI:   Ignore SCK/MOSI/MISO pins and just use the CS pin & default SPI bus.
 *   MAX31865_WIRES: Set the number of wires for the probe connected to a MAX31865 board, 2-4. Default: 2
 *   MAX31865_50HZ:  Enable 50Hz filter instead of the default 60Hz.
 */
//#define TEMP_SENSOR_FORCE_HW_SPI//#定义温度传感器、力传感器和SPI
//#define MAX31865_SENSOR_WIRES_0 2//#定义MAX31865_传感器_导线_0 2
//#define MAX31865_SENSOR_WIRES_1 2//#定义MAX31865_传感器_导线_1 2
//#define MAX31865_50HZ_FILTER//#定义MAX31865_50HZ_滤波器

/**
 * Hephestos 2 24V heated bed upgrade kit.
 * https://store.bq.com/en/heated-bed-kit-hephestos2
 */
//#define HEPHESTOS2_HEATED_BED_KIT//#定义HEPHESTOS2加热床套件
#if ENABLED(HEPHESTOS2_HEATED_BED_KIT)
#undef TEMP_SENSOR_BED
  #define TEMP_SENSOR_BED 70
  #define HEATER_BED_INVERTING true
#endif

////
// Heated Bed Bang-Bang options//加热床砰砰的一声
////
#if DISABLED(PIDTEMPBED)
#define BED_CHECK_INTERVAL 5000   // (ms) Interval between checks in bang-bang control//（ms）爆炸控制中的检查间隔
  #if ENABLED(BED_LIMIT_SWITCHING)
    #define BED_HYSTERESIS 2        // (°C) Only set the relevant heater state when ABS(T-target) > BED_HYSTERESIS//（°C）仅当ABS（T-目标）>床滞后时设置相关加热器状态
  #endif
#endif

////
// Heated Chamber options//加热室选项
////
#if DISABLED(PIDTEMPCHAMBER)
#define CHAMBER_CHECK_INTERVAL 5000   // (ms) Interval between checks in bang-bang control//（ms）爆炸控制中的检查间隔
  #if ENABLED(CHAMBER_LIMIT_SWITCHING)
    #define CHAMBER_HYSTERESIS 2        // (°C) Only set the relevant heater state when ABS(T-target) > CHAMBER_HYSTERESIS//（°C）仅在ABS（T目标）>腔室滞后时设置相关加热器状态
  #endif
#endif

#if TEMP_SENSOR_CHAMBER
//#define HEATER_CHAMBER_PIN      P2_04   // Required heater on/off pin (example: SKR 1.4 Turbo HE1 plug)//#定义加热器\腔室\针脚P2 \ U 04//所需的加热器开/关针脚（例如：SKR 1.4 Turbo HE1插头）
  //#define HEATER_CHAMBER_INVERTING false//#定义加热器\u腔室\u反转错误
  //#define FAN1_PIN                   -1   // Remove the fan signal on pin P2_04 (example: SKR 1.4 Turbo HE1 plug)//#定义风扇1_引脚-1//移除引脚P2_04上的风扇信号（例如：SKR 1.4 Turbo HE1插头）

  //#define CHAMBER_FAN               // Enable a fan on the chamber//#定义腔室\风扇//在腔室上启用风扇
  #if ENABLED(CHAMBER_FAN)
    #define CHAMBER_FAN_MODE 2        // Fan control mode: 0=Static; 1=Linear increase when temp is higher than target; 2=V-shaped curve; 3=similar to 1 but fan is always on.//风机控制模式：0=静态；1=温度高于目标时线性增加；2=V形曲线；3=类似于1，但风扇始终打开。
    #if CHAMBER_FAN_MODE == 0
      #define CHAMBER_FAN_BASE  255   // Chamber fan PWM (0-255)//腔室风扇PWM（0-255）
    #elif CHAMBER_FAN_MODE == 1
      #define CHAMBER_FAN_BASE  128   // Base chamber fan PWM (0-255); turns on when chamber temperature is above the target//基室风扇PWM（0-255）；当腔室温度高于目标温度时打开
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C above target//高于目标温度每°C PWM增加
    #elif CHAMBER_FAN_MODE == 2
      #define CHAMBER_FAN_BASE  128   // Minimum chamber fan PWM (0-255)//最小腔室风扇PWM（0-255）
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C difference from target//每与目标相差一°C，PWM增加
    #elif CHAMBER_FAN_MODE == 3
      #define CHAMBER_FAN_BASE  128   // Base chamber fan PWM (0-255)//基本腔室风扇PWM（0-255）
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C above target//高于目标温度每°C PWM增加
    #endif
  #endif

  //#define CHAMBER_VENT              // Enable a servo-controlled vent on the chamber//#定义腔室通风//在腔室上启用伺服控制通风
  #if ENABLED(CHAMBER_VENT)
    #define CHAMBER_VENT_SERVO_NR  1  // Index of the vent servo//排气伺服阀的索引
    #define HIGH_EXCESS_HEAT_LIMIT 5  // How much above target temp to consider there is excess heat in the chamber//以上目标温度要考虑多少，在腔室中有多余的热量。
    #define LOW_EXCESS_HEAT_LIMIT  3
    #define MIN_COOLING_SLOPE_TIME_CHAMBER_VENT 20
    #define MIN_COOLING_SLOPE_DEG_CHAMBER_VENT 1.5
  #endif
#endif

////
// Laser Cooler options//激光冷却器选项
////
#if TEMP_SENSOR_COOLER
#define COOLER_MINTEMP           8  // (°C)//（°C）
  #define COOLER_MAXTEMP          26  // (°C)//（°C）
  #define COOLER_DEFAULT_TEMP     16  // (°C)//（°C）
  #define TEMP_COOLER_HYSTERESIS   1  // (°C) Temperature proximity considered "close enough" to the target//（°C）温度接近被认为与目标“足够接近”
  #define COOLER_PIN               8  // Laser cooler on/off pin used to control power to the cooling element (e.g., TEC, External chiller via relay)//用于控制冷却元件电源的激光冷却器开/关引脚（例如TEC、通过继电器的外部冷却器）
  #define COOLER_INVERTING     false
  #define TEMP_COOLER_PIN         15  // Laser/Cooler temperature sensor pin. ADC is required.//激光器/冷却器温度传感器引脚。ADC是必需的。
  #define COOLER_FAN                  // Enable a fan on the cooler, Fan# 0,1,2,3 etc.//启用冷却器上的风扇、风扇#0、1、2、3等。
  #define COOLER_FAN_INDEX         0  // FAN number 0, 1, 2 etc. e.g.//风扇编号0、1、2等，例如。
  #if ENABLED(COOLER_FAN)
    #define COOLER_FAN_BASE      100  // Base Cooler fan PWM (0-255); turns on when Cooler temperature is above the target//基础冷却器风扇PWM（0-255）；当冷却器温度高于目标温度时打开
    #define COOLER_FAN_FACTOR     25  // PWM increase per °C above target//高于目标温度每°C PWM增加
  #endif
#endif

////
// Motherboard Sensor options//主板传感器选项
////
#if TEMP_SENSOR_BOARD
#define THERMAL_PROTECTION_BOARD   // Halt the printer if the board sensor leaves the temp range below.//如果板传感器离开以下温度范围，请停止打印机。
  #define BOARD_MINTEMP           8  // (°C)//（°C）
  #define BOARD_MAXTEMP          70  // (°C)//（°C）
  #ifndef TEMP_BOARD_PIN
    //#define TEMP_BOARD_PIN -1      // Board temp sensor pin, if not set in pins file.//#如果未在管脚文件中设置，则定义温度板管脚-1//板温度传感器管脚。
  #endif
#endif

////
// Laser Coolant Flow Meter//激光冷却剂流量计
////
//#define LASER_COOLANT_FLOW_METER//#定义激光冷却液流量计
#if ENABLED(LASER_COOLANT_FLOW_METER)
#define FLOWMETER_PIN         20  // Requires an external interrupt-enabled pin (e.g., RAMPS 2,3,18,19,20,21)//需要外部中断启用引脚（例如斜坡2,3,18,19,20,21）
  #define FLOWMETER_PPL       5880  // (pulses/liter) Flow meter pulses-per-liter on the input pin//（脉冲/升）输入引脚上的流量计脉冲/升
  #define FLOWMETER_INTERVAL  1000  // (ms) Flow rate calculation interval in milliseconds//（ms）流量计算间隔（毫秒）
  #define FLOWMETER_SAFETY          // Prevent running the laser without the minimum flow rate set below//在没有以下最小流速设置的情况下，防止运行激光器
  #if ENABLED(FLOWMETER_SAFETY)
    #define FLOWMETER_MIN_LITERS_PER_MINUTE 1.5 // (liters/min) Minimum flow required when enabled//启用时所需的最小流量（升/分钟）
  #endif
#endif

/**
 * Thermal Protection provides additional protection to your printer from damage
 * and fire. Marlin always includes safe min and max temperature ranges which
 * protect against a broken or disconnected thermistor wire.
 *
 * The issue: If a thermistor falls out, it will report the much lower
 * temperature of the air in the room, and the the firmware will keep
 * the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too
 * long (period), the firmware will halt the machine as a safety precaution.
 *
 * If you get false positives for "Thermal Runaway", increase
 * THERMAL_PROTECTION_HYSTERESIS and/or THERMAL_PROTECTION_PERIOD
 */
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
#define THERMAL_PROTECTION_PERIOD 40        // Seconds//秒
  #define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius//摄氏度

  //#define ADAPTIVE_FAN_SLOWING              // Slow part cooling fan if temperature drops//#定义自适应_风扇_减速//如果温度下降，则减速部分冷却风扇
  #if BOTH(ADAPTIVE_FAN_SLOWING, PIDTEMP)
    //#define NO_FAN_SLOWING_IN_PID_TUNING    // Don't slow fan speed during M303//#在PID调整中定义无风扇减速//M303期间不要减慢风扇转速
  #endif

  /**
   * Whenever an M104, M109, or M303 increases the target temperature, the
   * firmware will wait for the WATCH_TEMP_PERIOD to expire. If the temperature
   * hasn't increased by WATCH_TEMP_INCREASE degrees, the machine is halted and
   * requires a hard reset. This test restarts with any M104/M109/M303, but only
   * if the current temperature is far enough below the target for a reliable
   * test.
   *
   * If you get false positives for "Heating failed", increase WATCH_TEMP_PERIOD
   * and/or decrease WATCH_TEMP_INCREASE. WATCH_TEMP_INCREASE should not be set
   * below 2.
   */
  #define WATCH_TEMP_PERIOD  20               // Seconds//秒
  #define WATCH_TEMP_INCREASE 2               // Degrees Celsius//摄氏度
#endif

/**
 * Thermal Protection parameters for the bed are just as above for hotends.
 */
#if ENABLED(THERMAL_PROTECTION_BED)
#define THERMAL_PROTECTION_BED_PERIOD        20 // Seconds//秒
  #define THERMAL_PROTECTION_BED_HYSTERESIS     2 // Degrees Celsius//摄氏度

  /**
   * As described above, except for the bed (M140/M190/M303).
   */
  #define WATCH_BED_TEMP_PERIOD                60 // Seconds//秒
  #define WATCH_BED_TEMP_INCREASE               2 // Degrees Celsius//摄氏度
#endif

/**
 * Thermal Protection parameters for the heated chamber.
 */
#if ENABLED(THERMAL_PROTECTION_CHAMBER)
#define THERMAL_PROTECTION_CHAMBER_PERIOD    20 // Seconds//秒
  #define THERMAL_PROTECTION_CHAMBER_HYSTERESIS 2 // Degrees Celsius//摄氏度

  /**
   * Heated chamber watch settings (M141/M191).
   */
  #define WATCH_CHAMBER_TEMP_PERIOD            60 // Seconds//秒
  #define WATCH_CHAMBER_TEMP_INCREASE           2 // Degrees Celsius//摄氏度
#endif

/**
 * Thermal Protection parameters for the laser cooler.
 */
#if ENABLED(THERMAL_PROTECTION_COOLER)
#define THERMAL_PROTECTION_COOLER_PERIOD    10 // Seconds//秒
  #define THERMAL_PROTECTION_COOLER_HYSTERESIS 3 // Degrees Celsius//摄氏度

  /**
   * Laser cooling watch settings (M143/M193).
   */
  #define WATCH_COOLER_TEMP_PERIOD            60 // Seconds//秒
  #define WATCH_COOLER_TEMP_INCREASE           3 // Degrees Celsius//摄氏度
#endif

#if ENABLED(PIDTEMP)
// Add an experimental additional term to the heater power, proportional to the extrusion speed.//在加热器功率中添加一个与挤出速度成比例的实验附加项。
  // A well-chosen Kc value should add just enough power to melt the increased material volume.//选择适当的Kc值应增加足够的功率，以熔化增加的材料体积。
  //#define PID_EXTRUSION_SCALING//#定义PID\u拉伸\u缩放
  #if ENABLED(PID_EXTRUSION_SCALING)
    #define DEFAULT_Kc (100) // heating power = Kc * e_speed//加热功率=Kc*e_速度
    #define LPQ_MAX_LEN 50
  #endif

  /**
   * Add an experimental additional term to the heater power, proportional to the fan speed.
   * A well-chosen Kf value should add just enough power to compensate for power-loss from the cooling fan.
   * You can either just add a constant compensation with the DEFAULT_Kf value
   * or follow the instruction below to get speed-dependent compensation.
   *
   * Constant compensation (use only with fanspeeds of 0% and 100%)
   * ---------------------------------------------------------------------
   * A good starting point for the Kf-value comes from the calculation:
   *   kf = (power_fan * eff_fan) / power_heater * 255
   * where eff_fan is between 0.0 and 1.0, based on fan-efficiency and airflow to the nozzle / heater.
   *
   * Example:
   *   Heater: 40W, Fan: 0.1A * 24V = 2.4W, eff_fan = 0.8
   *   Kf = (2.4W * 0.8) / 40W * 255 = 12.24
   *
   * Fan-speed dependent compensation
   * --------------------------------
   * 1. To find a good Kf value, set the hotend temperature, wait for it to settle, and enable the fan (100%).
   *    Make sure PID_FAN_SCALING_LIN_FACTOR is 0 and PID_FAN_SCALING_ALTERNATIVE_DEFINITION is not enabled.
   *    If you see the temperature drop repeat the test, increasing the Kf value slowly, until the temperature
   *    drop goes away. If the temperature overshoots after enabling the fan, the Kf value is too big.
   * 2. Note the Kf-value for fan-speed at 100%
   * 3. Determine a good value for PID_FAN_SCALING_MIN_SPEED, which is around the speed, where the fan starts moving.
   * 4. Repeat step 1. and 2. for this fan speed.
   * 5. Enable PID_FAN_SCALING_ALTERNATIVE_DEFINITION and enter the two identified Kf-values in
   *    PID_FAN_SCALING_AT_FULL_SPEED and PID_FAN_SCALING_AT_MIN_SPEED. Enter the minimum speed in PID_FAN_SCALING_MIN_SPEED
   */
  //#define PID_FAN_SCALING//#定义PID\u风扇\u缩放
  #if ENABLED(PID_FAN_SCALING)
    //#define PID_FAN_SCALING_ALTERNATIVE_DEFINITION//#定义PID\u风扇\u缩放\u备选\u定义
    #if ENABLED(PID_FAN_SCALING_ALTERNATIVE_DEFINITION)
      // The alternative definition is used for an easier configuration.//替代定义用于更简单的配置。
      // Just figure out Kf at fullspeed (255) and PID_FAN_SCALING_MIN_SPEED.//只需计算全速（255）下的Kf和PID_风扇_缩放_最小_速度。
      // DEFAULT_Kf and PID_FAN_SCALING_LIN_FACTOR are calculated accordingly.//相应地计算默认的系数和PID系数。

      #define PID_FAN_SCALING_AT_FULL_SPEED 13.0        //=PID_FAN_SCALING_LIN_FACTOR*255+DEFAULT_Kf//=PID\u风机\u缩放\u风机\u系数*255+默认值\u Kf
      #define PID_FAN_SCALING_AT_MIN_SPEED   6.0        //=PID_FAN_SCALING_LIN_FACTOR*PID_FAN_SCALING_MIN_SPEED+DEFAULT_Kf//=PID\u风扇\u缩放系数*PID\u风扇\u缩放系数\u最小速度+默认值\u Kf
      #define PID_FAN_SCALING_MIN_SPEED     10.0        // Minimum fan speed at which to enable PID_FAN_SCALING//启用PID_风扇_缩放的最小风扇转速

      #define DEFAULT_Kf (255.0*PID_FAN_SCALING_AT_MIN_SPEED-PID_FAN_SCALING_AT_FULL_SPEED*PID_FAN_SCALING_MIN_SPEED)/(255.0-PID_FAN_SCALING_MIN_SPEED)
      #define PID_FAN_SCALING_LIN_FACTOR (PID_FAN_SCALING_AT_FULL_SPEED-DEFAULT_Kf)/255.0

    #else
      #define PID_FAN_SCALING_LIN_FACTOR (0)             // Power loss due to cooling = Kf * (fan_speed)//冷却导致的功率损失=Kf*（风扇转速）
      #define DEFAULT_Kf 10                              // A constant value added to the PID-tuner//添加到PID调谐器的常量值
      #define PID_FAN_SCALING_MIN_SPEED 10               // Minimum fan speed at which to enable PID_FAN_SCALING//启用PID_风扇_缩放的最小风扇转速
    #endif
  #endif
#endif

/**
 * Automatic Temperature Mode
 *
 * Dynamically adjust the hotend target temperature based on planned E moves.
 *
 * (Contrast with PID_EXTRUSION_SCALING, which tracks E movement and adjusts PID
 *  behavior using an additional kC value.)
 *
 * Autotemp is calculated by (mintemp + factor * mm_per_sec), capped to maxtemp.
 *
 * Enable Autotemp Mode with M104/M109 F<factor> S<mintemp> B<maxtemp>.
 * Disable by sending M104/M109 with no F parameter (or F0 with AUTOTEMP_PROPORTIONAL).
 */
#define AUTOTEMP
#if ENABLED(AUTOTEMP)
#define AUTOTEMP_OLDWEIGHT    0.98
  // Turn on AUTOTEMP on M104/M109 by default using proportions set here//默认情况下，使用此处设置的比例在M104/M109上启用自动temp
  //#define AUTOTEMP_PROPORTIONAL//#定义自动temp_比例
  #if ENABLED(AUTOTEMP_PROPORTIONAL)
    #define AUTOTEMP_MIN_P      0 // (°C) Added to the target temperature//（°C）添加到目标温度
    #define AUTOTEMP_MAX_P      5 // (°C) Added to the target temperature//（°C）添加到目标温度
    #define AUTOTEMP_FACTOR_P   1 // Apply this F parameter by default (overridden by M104/M109 F)//默认情况下应用此F参数（由M104/M109 F覆盖）
  #endif
#endif

// Show Temperature ADC value//显示温度ADC值
// Enable for M105 to include ADC values read from temperature sensors.//启用M105以包括从温度传感器读取的ADC值。
//#define SHOW_TEMP_ADC_VALUES//#定义显示温度ADC值

/**
 * High Temperature Thermistor Support
 *
 * Thermistors able to support high temperature tend to have a hard time getting
 * good readings at room and lower temperatures. This means TEMP_SENSOR_X_RAW_LO_TEMP
 * will probably be caught when the heating element first turns on during the
 * preheating process, which will trigger a min_temp_error as a safety measure
 * and force stop everything.
 * To circumvent this limitation, we allow for a preheat time (during which,
 * min_temp_error won't be triggered) and add a min_temp buffer to handle
 * aberrant readings.
 *
 * If you want to enable this feature for your hotend thermistor(s)
 * uncomment and set values > 0 in the constants below
 */

// The number of consecutive low temperature errors that can occur//可能发生的连续低温错误数
// before a min_temp_error is triggered. (Shouldn't be more than 10.)//在触发最低温度错误之前。（不应超过10。）
//#define MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED 0//#定义允许的最大\u连续\u低温\u错误\u 0

// The number of milliseconds a hotend will preheat before starting to check//热端在开始检查前预热的毫秒数
// the temperature. This value should NOT be set to the time it takes the//温度。此值不应设置为
// hot end to reach the target temperature, but the time it takes to reach//热端达到目标温度，但达到目标温度所需的时间
// the minimum temperature your thermistor can read. The lower the better/safer.//热敏电阻可以读取的最低温度。越低越好/越安全。
// This shouldn't need to be more than 30 seconds (30000)//这不需要超过30秒（30000）
//#define MILLISECONDS_PREHEAT_TIME 0//#定义毫秒\u预热\u时间0

// @section extruder//型材挤出机

// Extruder runout prevention.//挤出机跑偏预防。
// If the machine is idle and the temperature over MINTEMP//如果机器怠速且温度超过MINTEMP
// then extrude some filament every couple of SECONDS.//然后每隔几秒钟挤出一些细丝。
//#define EXTRUDER_RUNOUT_PREVENT//#定义挤出机\u跳动\u防止
#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
#define EXTRUDER_RUNOUT_MINTEMP 190
  #define EXTRUDER_RUNOUT_SECONDS 30
  #define EXTRUDER_RUNOUT_SPEED 1500  // (mm/min)//（毫米/分钟）
  #define EXTRUDER_RUNOUT_EXTRUDE 5   // (mm)//（毫米）
#endif

/**
 * Hotend Idle Timeout
 * Prevent filament in the nozzle from charring and causing a critical jam.
 */
//#define HOTEND_IDLE_TIMEOUT//#定义热端空闲超时
#if ENABLED(HOTEND_IDLE_TIMEOUT)
#define HOTEND_IDLE_TIMEOUT_SEC (5*60)    // (seconds) Time without extruder movement to trigger protection//（秒）挤出机不移动触发保护的时间
  #define HOTEND_IDLE_MIN_TRIGGER   180     // (°C) Minimum temperature to enable hotend protection//（°C）启用热端保护的最低温度
  #define HOTEND_IDLE_NOZZLE_TARGET   0     // (°C) Safe temperature for the nozzle after timeout//超时后喷嘴的安全温度（°C）
  #define HOTEND_IDLE_BED_TARGET      0     // (°C) Safe temperature for the bed after timeout//超时后床的安全温度（°C）
#endif

// @section temperature//@截面温度

// Calibration for AD595 / AD8495 sensor to adjust temperature measurements.//校准AD595/AD8495传感器以调整温度测量值。
// The final temperature is calculated as (measuredTemp * GAIN) + OFFSET.//最终温度计算为（测量温度*增益）+偏移量。
#define TEMP_SENSOR_AD595_OFFSET  0.0
#define TEMP_SENSOR_AD595_GAIN    1.0
#define TEMP_SENSOR_AD8495_OFFSET 0.0
#define TEMP_SENSOR_AD8495_GAIN   1.0

/**
 * Controller Fan
 * To cool down the stepper drivers and MOSFETs.
 *
 * The fan turns on automatically whenever any driver is enabled and turns
 * off (or reduces to idle speed) shortly after drivers are turned off.
 */
//#define USE_CONTROLLER_FAN//#定义使用\u控制器\u风扇
#if ENABLED(USE_CONTROLLER_FAN)
//#define CONTROLLER_FAN_PIN -1           // Set a custom pin for the controller fan//#定义控制器\u风扇\u引脚-1//为控制器风扇设置自定义引脚
  //#define CONTROLLER_FAN_USE_Z_ONLY       // With this option only the Z axis is considered//#定义控制器\u FAN\u仅使用\u Z\u//使用此选项仅考虑Z轴
  //#define CONTROLLER_FAN_IGNORE_Z         // Ignore Z stepper. Useful when stepper timeout is disabled.//#定义控制器\u FAN\u IGNORE\u Z//IGNORE Z步进器。禁用步进超时时有用。
  #define CONTROLLERFAN_SPEED_MIN         0 // (0-255) Minimum speed. (If set below this value the fan is turned off.)//（0-255）最低速度。（如果设置低于此值，风扇将关闭。）
  #define CONTROLLERFAN_SPEED_ACTIVE    255 // (0-255) Active speed, used when any motor is enabled//（0-255）激活速度，在任何电机启用时使用
  #define CONTROLLERFAN_SPEED_IDLE        0 // (0-255) Idle speed, used when motors are disabled//（0-255）怠速，在禁用电机时使用
  #define CONTROLLERFAN_IDLE_TIME        60 // (seconds) Extra time to keep the fan running after disabling motors//（秒）禁用电机后保持风扇运行的额外时间

  // Use TEMP_SENSOR_BOARD as a trigger for enabling the controller fan//使用温度传感器板作为启动控制器风扇的触发器
  //#define CONTROLLER_FAN_MIN_BOARD_TEMP 40  // (°C) Turn on the fan if the board reaches this temperature//#定义控制器风扇最小电路板温度40/（°C），如果电路板达到此温度，则打开风扇

  //#define CONTROLLER_FAN_EDITABLE         // Enable M710 configurable settings//#定义控制器\u风扇\u可编辑//启用M710可配置设置
  #if ENABLED(CONTROLLER_FAN_EDITABLE)
    #define CONTROLLER_FAN_MENU             // Enable the Controller Fan submenu//启用控制器风扇子菜单
  #endif
#endif

// When first starting the main fan, run it at full speed for the//首次启动主风扇时，以全速运行一段时间
// given number of milliseconds.  This gets the fan spinning reliably//给定的毫秒数。这使风扇可靠地旋转
// before setting a PWM value. (Does not work with software PWM for fan on Sanguinololu)//在设置PWM值之前。（不适用于Sanguinololu上风扇的软件PWM）
//#define FAN_KICKSTART_TIME 100//#定义风扇启动时间100

// Some coolers may require a non-zero "off" state.//一些冷却器可能需要非零“关闭”状态。
//#define FAN_OFF_PWM  1//#定义风扇关闭PWM 1

/**
 * PWM Fan Scaling
 *
 * Define the min/max speeds for PWM fans (as set with M106).
 *
 * With these options the M106 0-255 value range is scaled to a subset
 * to ensure that the fan has enough power to spin, or to run lower
 * current fans with higher current. (e.g., 5V/12V fans with 12V/24V)
 * Value 0 always turns off the fan.
 *
 * Define one or both of these to override the default 0-255 range.
 */
//#define FAN_MIN_PWM 50//#定义风扇最小脉宽调制50
//#define FAN_MAX_PWM 128//#定义风扇\u最大\u PWM 128

/**
 * FAST PWM FAN Settings
 *
 * Use to change the FAST FAN PWM frequency (if enabled in Configuration.h)
 * Combinations of PWM Modes, prescale values and TOP resolutions are used internally to produce a
 * frequency as close as possible to the desired frequency.
 *
 * FAST_PWM_FAN_FREQUENCY [undefined by default]
 *   Set this to your desired frequency.
 *   If left undefined this defaults to F = F_CPU/(2*255*1)
 *   i.e., F = 31.4kHz on 16MHz microcontrollers or F = 39.2kHz on 20MHz microcontrollers.
 *   These defaults are the same as with the old FAST_PWM_FAN implementation - no migration is required
 *   NOTE: Setting very low frequencies (< 10 Hz) may result in unexpected timer behavior.
 *
 * USE_OCR2A_AS_TOP [undefined by default]
 *   Boards that use TIMER2 for PWM have limitations resulting in only a few possible frequencies on TIMER2:
 *   16MHz MCUs: [62.5KHz, 31.4KHz (default), 7.8KHz, 3.92KHz, 1.95KHz, 977Hz, 488Hz, 244Hz, 60Hz, 122Hz, 30Hz]
 *   20MHz MCUs: [78.1KHz, 39.2KHz (default), 9.77KHz, 4.9KHz, 2.44KHz, 1.22KHz, 610Hz, 305Hz, 153Hz, 76Hz, 38Hz]
 *   A greater range can be achieved by enabling USE_OCR2A_AS_TOP. But note that this option blocks the use of
 *   PWM on pin OC2A. Only use this option if you don't need PWM on 0C2A. (Check your schematic.)
 *   USE_OCR2A_AS_TOP sacrifices duty cycle control resolution to achieve this broader range of frequencies.
 */
#if ENABLED(FAST_PWM_FAN)
//#define FAST_PWM_FAN_FREQUENCY 31400//#定义快速PWM风扇频率31400
  //#define USE_OCR2A_AS_TOP//#将“使用”定义为“顶部”
#endif

/**
 * Use one of the PWM fans as a redundant part-cooling fan
 */
//#define REDUNDANT_PART_COOLING_FAN 2  // Index of the fan to sync with FAN 0.//#定义要与风扇0同步的风扇的冗余\u零件\u冷却\u风扇2//索引。

// @section extruder//型材挤出机

/**
 * Extruder cooling fans
 *
 * Extruder auto fans automatically turn on when their extruders'
 * temperatures go above EXTRUDER_AUTO_FAN_TEMPERATURE.
 *
 * Your board's pins file specifies the recommended pins. Override those here
 * or set to -1 to disable completely.
 *
 * Multiple extruders can be assigned to the same pin in which case
 * the fan will turn on when any selected extruder is above the threshold.
 */
#define E0_AUTO_FAN_PIN -1
#define E1_AUTO_FAN_PIN -1
#define E2_AUTO_FAN_PIN -1
#define E3_AUTO_FAN_PIN -1
#define E4_AUTO_FAN_PIN -1
#define E5_AUTO_FAN_PIN -1
#define E6_AUTO_FAN_PIN -1
#define E7_AUTO_FAN_PIN -1
#define CHAMBER_AUTO_FAN_PIN -1
#define COOLER_AUTO_FAN_PIN -1
#define COOLER_FAN_PIN -1

#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED 255   // 255 == full speed//255==全速
#define CHAMBER_AUTO_FAN_TEMPERATURE 30
#define CHAMBER_AUTO_FAN_SPEED 255
#define COOLER_AUTO_FAN_TEMPERATURE 18
#define COOLER_AUTO_FAN_SPEED 255

/**
 * Part-Cooling Fan Multiplexer
 *
 * This feature allows you to digitally multiplex the fan output.
 * The multiplexer is automatically switched at tool-change.
 * Set FANMUX[012]_PINs below for up to 2, 4, or 8 multiplexed fans.
 */
#define FANMUX0_PIN -1
#define FANMUX1_PIN -1
#define FANMUX2_PIN -1

/**
 * M355 Case Light on-off / brightness
 */
//#define CASE_LIGHT_ENABLE//#定义案例\灯光\启用
#if ENABLED(CASE_LIGHT_ENABLE)
//#define CASE_LIGHT_PIN 4                  // Override the default pin if needed//#定义案例\灯\插脚4//如果需要，覆盖默认插脚
  #define INVERT_CASE_LIGHT false             // Set true if Case Light is ON when pin is LOW//引脚低时，如果外壳指示灯点亮，则设置为真
  #define CASE_LIGHT_DEFAULT_ON true          // Set default power-up state on//将默认开机状态设置为on
  #define CASE_LIGHT_DEFAULT_BRIGHTNESS 105   // Set default power-up brightness (0-255, requires PWM pin)//设置默认通电亮度（0-255，需要PWM引脚）
  //#define CASE_LIGHT_NO_BRIGHTNESS          // Disable brightness control. Enable for non-PWM lighting.//#定义大小写灯光亮度//禁用亮度控制。启用非PWM照明。
  //#define CASE_LIGHT_MAX_PWM 128            // Limit PWM duty cycle (0-255)//#定义情况灯最大脉宽调制128//限制脉宽调制占空比（0-255）
  //#define CASE_LIGHT_MENU                   // Add Case Light options to the LCD menu//#定义箱灯菜单//将箱灯选项添加到LCD菜单
  #if ENABLED(NEOPIXEL_LED)
    //#define CASE_LIGHT_USE_NEOPIXEL         // Use NeoPixel LED as case light//#定义案例灯使用案例灯//使用案例灯LED
  #endif
  #if EITHER(RGB_LED, RGBW_LED)
    //#define CASE_LIGHT_USE_RGB_LED          // Use RGB / RGBW LED as case light//#定义案例灯使用RGB LED//使用RGB/RGBW LED作为案例灯
  #endif
  #if EITHER(CASE_LIGHT_USE_NEOPIXEL, CASE_LIGHT_USE_RGB_LED)
    #define CASE_LIGHT_DEFAULT_COLOR { 255, 255, 255, 255 } // { Red, Green, Blue, White }//{红、绿、蓝、白}
  #endif
#endif

// @section homing//@段归位

// If you want endstops to stay on (by default) even when not homing//如果希望端点停止（默认情况下）即使在不归位时也保持启用状态
// enable this option. Override at any time with M120, M121.//启用此选项。随时用M120、M121替代。
#define ENDSTOPS_ALWAYS_ON_DEFAULT//#默认情况下，定义终止点\u始终\u

// @section extras//@额外部分

//#define Z_LATE_ENABLE // Enable Z the last moment. Needed if your Z driver overheats.//#定义Z_LATE_ENABLE//在最后一刻启用Z。如果Z驱动程序过热，则需要。

// Employ an external closed loop controller. Override pins here if needed.//采用外部闭环控制器。如果需要，在此处覆盖销。
//#define EXTERNAL_CLOSED_LOOP_CONTROLLER//#定义外部闭环控制器
#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
//#define CLOSED_LOOP_ENABLE_PIN        -1//#定义闭环\u启用\u引脚-1
  //#define CLOSED_LOOP_MOVE_COMPLETE_PIN -1//#定义闭合环\u移动\u完成\u引脚-1
#endif

/**
 * Dual Steppers / Dual Endstops
 *
 * This section will allow you to use extra E drivers to drive a second motor for X, Y, or Z axes.
 *
 * For example, set X_DUAL_STEPPER_DRIVERS setting to use a second motor. If the motors need to
 * spin in opposite directions set INVERT_X2_VS_X_DIR. If the second motor needs its own endstop
 * set X_DUAL_ENDSTOPS. This can adjust for "racking." Use X2_USE_ENDSTOP to set the endstop plug
 * that should be used for the second endstop. Extra endstops will appear in the output of 'M119'.
 *
 * Use X_DUAL_ENDSTOP_ADJUSTMENT to adjust for mechanical imperfection. After homing both motors
 * this offset is applied to the X2 motor. To find the offset home the X axis, and measure the error
 * in X2. Dual endstop offsets can be set at runtime with 'M666 X<offset> Y<offset> Z<offset>'.
 */

//#define X_DUAL_STEPPER_DRIVERS//#定义X_双_步进驱动程序
#if ENABLED(X_DUAL_STEPPER_DRIVERS)
//#define INVERT_X2_VS_X_DIR    // Enable if X2 direction signal is opposite to X//#定义反转_X2_VS_X_DIR//如果X2方向信号与X相反，则启用
  //#define X_DUAL_ENDSTOPS//#定义X_双_止动块
  #if ENABLED(X_DUAL_ENDSTOPS)
    #define X2_USE_ENDSTOP _XMAX_
    #define X2_ENDSTOP_ADJUSTMENT  0
  #endif
#endif

//#define Y_DUAL_STEPPER_DRIVERS//#定义Y_双_步进驱动程序
#if ENABLED(Y_DUAL_STEPPER_DRIVERS)
//#define INVERT_Y2_VS_Y_DIR   // Enable if Y2 direction signal is opposite to Y//#定义反转Y 2与Y方向//如果Y 2方向信号与Y方向相反，则启用
  //#define Y_DUAL_ENDSTOPS//#定义Y_双止动块
  #if ENABLED(Y_DUAL_ENDSTOPS)
    #define Y2_USE_ENDSTOP _YMAX_
    #define Y2_ENDSTOP_ADJUSTMENT  0
  #endif
#endif

////
// For Z set the number of stepper drivers//对于Z，设置步进驱动程序的数量
////
#define NUM_Z_STEPPER_DRIVERS 1   // (1-4) Z options change based on how many//（1-4）Z选项根据数量变化

#if NUM_Z_STEPPER_DRIVERS > 1
// Enable if Z motor direction signals are the opposite of Z1//如果Z电机方向信号与Z1相反，则启用
  //#define INVERT_Z2_VS_Z_DIR//#定义反转Z2与反转方向
  //#define INVERT_Z3_VS_Z_DIR//#定义反向_Z3_VS_Z_DIR
  //#define INVERT_Z4_VS_Z_DIR//#定义反转方向

  //#define Z_MULTI_ENDSTOPS//#定义Z_多个端点
  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z2_USE_ENDSTOP          _XMAX_
    #define Z2_ENDSTOP_ADJUSTMENT   0
    #if NUM_Z_STEPPER_DRIVERS >= 3
      #define Z3_USE_ENDSTOP        _YMAX_
      #define Z3_ENDSTOP_ADJUSTMENT 0
    #endif
    #if NUM_Z_STEPPER_DRIVERS >= 4
      #define Z4_USE_ENDSTOP        _ZMAX_
      #define Z4_ENDSTOP_ADJUSTMENT 0
    #endif
  #endif
#endif

// Drive the E axis with two synchronized steppers//使用两个同步步进器驱动E轴
//#define E_DUAL_STEPPER_DRIVERS//#定义双步进电机驱动程序
#if ENABLED(E_DUAL_STEPPER_DRIVERS)
//#define INVERT_E1_VS_E0_DIR   // Enable if the E motors need opposite DIR states//#定义反转_E1_VS_E0_DIR//如果电机需要相反的DIR状态，则启用
#endif

/**
 * Dual X Carriage
 *
 * This setup has two X carriages that can move independently, each with its own hotend.
 * The carriages can be used to print an object with two colors or materials, or in
 * "duplication mode" it can print two identical or X-mirrored objects simultaneously.
 * The inactive carriage is parked automatically to prevent oozing.
 * X1 is the left carriage, X2 the right. They park and home at opposite ends of the X axis.
 * By default the X2 stepper is assigned to the first unused E plug on the board.
 *
 * The following Dual X Carriage modes can be selected with M605 S<mode>:
 *
 *   0 : (FULL_CONTROL) The slicer has full control over both X-carriages and can achieve optimal travel
 *       results as long as it supports dual X-carriages. (M605 S0)
 *
 *   1 : (AUTO_PARK) The firmware automatically parks and unparks the X-carriages on tool-change so
 *       that additional slicer support is not required. (M605 S1)
 *
 *   2 : (DUPLICATION) The firmware moves the second X-carriage and extruder in synchronization with
 *       the first X-carriage and extruder, to print 2 copies of the same object at the same time.
 *       Set the constant X-offset and temperature differential with M605 S2 X[offs] R[deg] and
 *       follow with M605 S2 to initiate duplicated movement.
 *
 *   3 : (MIRRORED) Formbot/Vivedino-inspired mirrored mode in which the second extruder duplicates
 *       the movement of the first except the second extruder is reversed in the X axis.
 *       Set the initial X offset and temperature differential with M605 S2 X[offs] R[deg] and
 *       follow with M605 S3 to initiate mirrored movement.
 */
//#define DUAL_X_CARRIAGE//#定义双_X_车厢
#if ENABLED(DUAL_X_CARRIAGE)
#define X1_MIN_POS X_MIN_POS   // Set to X_MIN_POS//设置为X_最小位置
  #define X1_MAX_POS X_BED_SIZE  // Set a maximum so the first X-carriage can't hit the parked second X-carriage//设置一个最大值，使第一个X-车厢不会撞到停放的第二个X-车厢
  #define X2_MIN_POS    80       // Set a minimum to ensure the  second X-carriage can't hit the parked first X-carriage//设置最小值，以确保第二个X-车厢不会撞到停放的第一个X-车厢
  #define X2_MAX_POS   353       // Set this to the distance between toolheads when both heads are homed//将此设置为两个刀头均已原点时刀头之间的距离
  #define X2_HOME_DIR    1       // Set to 1. The second X-carriage always homes to the maximum endstop position//设置为1。第二个X-托架始终位于最大止动位置
  #define X2_HOME_POS X2_MAX_POS // Default X2 home position. Set to X2_MAX_POS.//默认X2原点位置。设置为X2_最大位置。
                      // However: In this mode the HOTEND_OFFSET_X value for the second extruder provides a software//但是：在此模式下，第二台挤出机的热端偏移值提供了一个软件
                      // override for X2_HOME_POS. This also allow recalibration of the distance between the two endstops//X2_HOME_位置的超控。这也允许重新校准两个端止点之间的距离
                      // without modifying the firmware (through the "M218 T1 X???" command).//无需修改固件（通过“M218 T1 X？？？”命令）。
                      // Remember: you should set the second extruder x-offset to 0 in your slicer.//请记住：应在切片机中将第二台挤出机的x偏移设置为0。

  // This is the default power-up mode which can be later using M605.//这是默认的通电模式，以后可以使用M605。
  #define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_AUTO_PARK_MODE

  // Default x offset in duplication mode (typically set to half print bed width)//复制模式下的默认x偏移（通常设置为打印床宽度的一半）
  #define DEFAULT_DUPLICATION_X_OFFSET 100

  // Default action to execute following M605 mode change commands. Typically G28X to apply new mode.//执行以下M605模式更改命令的默认操作。通常使用G28X来应用新模式。
  //#define EVENT_GCODE_IDEX_AFTER_MODECHANGE "G28X"//#在模式更改“G28X”后定义事件代码
#endif

// Activate a solenoid on the active extruder with M380. Disable all with M381.//使用M380启动主动挤出机上的电磁阀。使用M381禁用所有。
// Define SOL0_PIN, SOL1_PIN, etc., for each extruder that has a solenoid.//为每个具有电磁阀的挤出机定义SOL0_引脚、SOL1_引脚等。
//#define EXT_SOLENOID//#定义外部电磁阀

// @section homing//@段归位

/**
 * Homing Procedure
 * Homing (G28) does an indefinite move towards the endstops to establish
 * the position of the toolhead relative to the workspace.
 */

#define SENSORLESS_BACKOFF_MM  { 2, 2, 0 }  // (mm) Backoff from endstops before sensorless homing//#在无传感器归位之前，定义无传感器回退{2,2,0}//（MM）从端止点回退

#define HOMING_BUMP_MM      { 5, 5, 2 }       // (mm) Backoff from endstops after first bump//（mm）第一次碰撞后从止动块后退
#define HOMING_BUMP_DIVISOR { 2, 2, 4 }       // Re-Bump Speed Divisor (Divides the Homing Feedrate)//重碰撞速度分割器（分割回零进给速度）

//#define HOMING_BACKOFF_POST_MM { 2, 2, 2 }  // (mm) Backoff from endstops after homing//#定义归位后的归位回退(POST)MM{2,2,2}//(MM)从归位后的结束停止回退

#define QUICK_HOME                          // If G28 contains XY do a diagonal move first//#定义QUICK_HOME//如果G28包含XY，则首先进行对角线移动
//#define HOME_Y_BEFORE_X                     // If G28 contains XY home Y before X//#在X之前定义HOME\u Y\u//如果G28在X之前包含XY HOME Y
#define HOME_Z_FIRST                        // Home Z first. Requires a Z-MIN endstop (not a probe).//#定义HOME_Z_FIRST//HOME Z FIRST。需要Z-MIN止动块（不是探针）。
//#define CODEPENDENT_XY_HOMING               // If X/Y can't home without homing Y/X first//#如果X/Y在没有先返回Y/X的情况下无法返回，则定义相互依赖的返回

// @section bltouch//@bltouch节

#if ENABLED(BLTOUCH)
/**
   * Either: Use the defaults (recommended) or: For special purposes, use the following DEFINES
   * Do not activate settings that the probe might not understand. Clones might misunderstand
   * advanced commands.
   *
   * Note: If the probe is not deploying, do a "Reset" and "Self-Test" and then check the
   *       wiring of the BROWN, RED and ORANGE wires.
   *
   * Note: If the trigger signal of your probe is not being recognized, it has been very often
   *       because the BLACK and WHITE wires needed to be swapped. They are not "interchangeable"
   *       like they would be with a real switch. So please check the wiring first.
   *
   * Settings for all BLTouch and clone probes:
   */

  // Safety: The probe needs time to recognize the command.//安全性：探测器需要时间来识别命令。
  //         Minimum command delay (ms). Enable and increase if needed.//最小命令延迟（ms）。启用并在需要时增加。
  //#define BLTOUCH_DELAY 500//#定义BLTOUCH_延迟500

  /**
   * Settings for BLTOUCH Classic 1.2, 1.3 or BLTouch Smart 1.0, 2.0, 2.2, 3.0, 3.1, and most clones:
   */

  // Feature: Switch into SW mode after a deploy. It makes the output pulse longer. Can be useful//功能：部署后切换到SW模式。它使输出脉冲更长。可能有用
  //          in special cases, like noisy or filtered input configurations.//在特殊情况下，如噪声或过滤输入配置。
  //#define BLTOUCH_FORCE_SW_MODE//#定义BLTOUCH\U FORCE\U SW\U模式

  /**
   * Settings for BLTouch Smart 3.0 and 3.1
   * Summary:
   *   - Voltage modes: 5V and OD (open drain - "logic voltage free") output modes
   *   - High-Speed mode
   *   - Disable LCD voltage options
   */

  /**
   * Danger: Don't activate 5V mode unless attached to a 5V-tolerant controller!
   * V3.0 or 3.1: Set default mode to 5V mode at Marlin startup.
   * If disabled, OD mode is the hard-coded default on 3.0
   * On startup, Marlin will compare its eeprom to this value. If the selected mode
   * differs, a mode set eeprom write will be completed at initialization.
   * Use the option below to force an eeprom write to a V3.1 probe regardless.
   */
  //#define BLTOUCH_SET_5V_MODE//#定义BLTOUCH_设置_5V_模式

  /**
   * Safety: Activate if connecting a probe with an unknown voltage mode.
   * V3.0: Set a probe into mode selected above at Marlin startup. Required for 5V mode on 3.0
   * V3.1: Force a probe with unknown mode into selected mode at Marlin startup ( = Probe EEPROM write )
   * To preserve the life of the probe, use this once then turn it off and re-flash.
   */
  //#define BLTOUCH_FORCE_MODE_SET//#定义BLTOUCH\u力\u模式\u集

  /**
   * Use "HIGH SPEED" mode for probing.
   * Danger: Disable if your probe sometimes fails. Only suitable for stable well-adjusted systems.
   * This feature was designed for Deltabots with very fast Z moves; however, higher speed Cartesians
   * might be able to use it. If the machine can't raise Z fast enough the BLTouch may go into ALARM.
   */
  //#define BLTOUCH_HS_MODE//#定义BLTOUCH\u HS\u模式

  // Safety: Enable voltage mode settings in the LCD menu.//安全：在LCD菜单中启用电压模式设置。
  //#define BLTOUCH_LCD_VOLTAGE_MENU//#定义BLTOUCH\u LCD\u电压\u菜单

#endif // BLTOUCH//BLTOUCH

// @section extras//@额外部分

/**
 * Z Steppers Auto-Alignment
 * Add the G34 command to align multiple Z steppers using a bed probe.
 */
//#define Z_STEPPER_AUTO_ALIGN//#定义Z_步进器自动对齐
#if ENABLED(Z_STEPPER_AUTO_ALIGN)
// Define probe X and Y positions for Z1, Z2 [, Z3 [, Z4]]//定义Z1、Z2[、Z3[、Z4]的探针X和Y位置
  // If not defined, probe limits will be used.//如果未定义，将使用探针极限。
  // Override with 'M422 S<index> X<pos> Y<pos>'//用“M422 S<index>X<pos>Y<pos>覆盖”
  //#define Z_STEPPER_ALIGN_XY { {  10, 190 }, { 100,  10 }, { 190, 190 } }//#定义Z_步进器_对齐_XY{{10，190}，{100，10}，{190，190}

  /**
   * Orientation for the automatically-calculated probe positions.
   * Override Z stepper align points with 'M422 S<index> X<pos> Y<pos>'
   *
   * 2 Steppers:  (0)     (1)
   *               |       |   2   |
   *               | 1   2 |       |
   *               |       |   1   |
   *
   * 3 Steppers:  (0)     (1)     (2)     (3)
   *               |   3   | 1     | 2   1 |     2 |
   *               |       |     3 |       | 3     |
   *               | 1   2 | 2     |   3   |     1 |
   *
   * 4 Steppers:  (0)     (1)     (2)     (3)
   *               | 4   3 | 1   4 | 2   1 | 3   2 |
   *               |       |       |       |       |
   *               | 1   2 | 2   3 | 3   4 | 4   1 |
   */
  #ifndef Z_STEPPER_ALIGN_XY
    //#define Z_STEPPERS_ORIENTATION 0//#定义Z_步进器_方向0
  #endif

  // Provide Z stepper positions for more rapid convergence in bed alignment.//提供Z步进器位置，以便在床对齐时更快收敛。
  // Requires triple stepper drivers (i.e., set NUM_Z_STEPPER_DRIVERS to 3)//需要三个步进驱动程序（即，将NUM_Z_步进驱动程序设置为3）
  //#define Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS//#定义Z_步进器_对齐_已知_步进器_位置
  #if ENABLED(Z_STEPPER_ALIGN_KNOWN_STEPPER_POSITIONS)
    // Define Stepper XY positions for Z1, Z2, Z3 corresponding to//定义Z1、Z2、Z3的步进XY位置，对应于
    // the Z screw positions in the bed carriage.//Z螺钉位于床托架中。
    // Define one position per Z stepper in stepper driver order.//按步进驱动器顺序，每Z步进定义一个位置。
    #define Z_STEPPER_ALIGN_STEPPER_XY { { 210.7, 102.5 }, { 152.6, 220.0 }, { 94.5, 102.5 } }
  #else
    // Amplification factor. Used to scale the correction step up or down in case//放大系数。用于放大或缩小校正步长，以防
    // the stepper (spindle) position is farther out than the test point.//步进电机（主轴）位置比测试点更远。
    #define Z_STEPPER_ALIGN_AMP 1.0       // Use a value > 1.0 NOTE: This may cause instability!//使用大于1.0的值注意：这可能会导致不稳定！
  #endif

  // On a 300mm bed a 5% grade would give a misalignment of ~1.5cm//在300毫米的河床上，5%的坡度将产生约1.5厘米的偏差
  #define G34_MAX_GRADE              5    // (%) Maximum incline that G34 will handle//（%）G34将处理的最大坡度
  #define Z_STEPPER_ALIGN_ITERATIONS 5    // Number of iterations to apply during alignment//对齐期间要应用的迭代次数
  #define Z_STEPPER_ALIGN_ACC        0.02 // Stop iterating early if the accuracy is better than this//如果精度高于此值，请尽早停止迭代
  #define RESTORE_LEVELING_AFTER_G34      // Restore leveling after G34 is done?//G34完成后恢复水平？
  // After G34, re-home Z (G28 Z) or just calculate it from the last probe heights?//G34之后，重新设置Z（G28 Z）或仅从最后一个探头高度计算？
  // Re-homing might be more precise in reproducing the actual 'G28 Z' homing height, especially on an uneven bed.//在再现实际的“G28 Z”归位高度时，重新归位可能更精确，尤其是在不平的河床上。
  #define HOME_AFTER_G34
#endif

////
// Add the G35 command to read bed corners to help adjust screws. Requires a bed probe.//添加G35命令以读取床角，以帮助调整螺钉。需要一个床探头。
////
//#define ASSISTED_TRAMMING//#定义辅助运输
#if ENABLED(ASSISTED_TRAMMING)

// Define positions for probe points.//定义探测点的位置。
  #define TRAMMING_POINT_XY { {  20, 20 }, { 180,  20 }, { 180, 180 }, { 20, 180 } }

  // Define position names for probe points.//定义探测点的位置名称。
  #define TRAMMING_POINT_NAME_1 "Front-Left"
  #define TRAMMING_POINT_NAME_2 "Front-Right"
  #define TRAMMING_POINT_NAME_3 "Back-Right"
  #define TRAMMING_POINT_NAME_4 "Back-Left"

  #define RESTORE_LEVELING_AFTER_G35    // Enable to restore leveling setup after operation//启用以在操作后恢复调平设置
  //#define REPORT_TRAMMING_MM          // Report Z deviation (mm) for each point relative to the first//#定义报告_TRAMMING_MM//报告相对于第一个点的每个点的Z偏差（MM）

  //#define ASSISTED_TRAMMING_WIZARD    // Add a Tramming Wizard to the LCD menu//#定义辅助跟踪向导//将跟踪向导添加到LCD菜单

  //#define ASSISTED_TRAMMING_WAIT_POSITION { X_CENTER, Y_CENTER, 30 } // Move the nozzle out of the way for adjustment//#定义辅助运输等待位置{X_中心，Y_中心，30}//将喷嘴移开进行调整

  /**
   * Screw thread:
   *   M3: 30 = Clockwise, 31 = Counter-Clockwise
   *   M4: 40 = Clockwise, 41 = Counter-Clockwise
   *   M5: 50 = Clockwise, 51 = Counter-Clockwise
   */
  #define TRAMMING_SCREW_THREAD 30

#endif

// @section motion//@节动议

#define AXIS_RELATIVE_MODES { false, false, false, false }

// Add a Duplicate option for well-separated conjoined nozzles//为分离良好的连体接管添加重复选项
//#define MULTI_NOZZLE_DUPLICATION//#定义多喷嘴复制

// By default pololu step drivers require an active high signal. However, some high power drivers require an active low signal as step.//默认情况下，步进驱动器需要激活的高信号。然而，一些高功率驱动器需要一个有效的低电平信号作为步骤。
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_I_STEP_PIN false
#define INVERT_J_STEP_PIN false
#define INVERT_K_STEP_PIN false
#define INVERT_E_STEP_PIN false

/**
 * Idle Stepper Shutdown
 * Set DISABLE_INACTIVE_? 'true' to shut down axis steppers after an idle period.
 * The Deactive Time can be overridden with M18 and M84. Set to 0 for No Timeout.
 */
#define DEFAULT_STEPPER_DEACTIVE_TIME 120
#define DISABLE_INACTIVE_X true
#define DISABLE_INACTIVE_Y true
#define DISABLE_INACTIVE_Z true  // Set 'false' if the nozzle could fall onto your printed part!//如果喷嘴可能掉落到打印部件上，请设置“false”！
#define DISABLE_INACTIVE_I true
#define DISABLE_INACTIVE_J true
#define DISABLE_INACTIVE_K true
#define DISABLE_INACTIVE_E true

// Default Minimum Feedrates for printing and travel moves//打印和移动的默认最小进给速率
#define DEFAULT_MINIMUMFEEDRATE       0.0     // (mm/s) Minimum feedrate. Set with M205 S.//（mm/s）最小进给速度。设置为M205 S。
#define DEFAULT_MINTRAVELFEEDRATE     0.0     // (mm/s) Minimum travel feedrate. Set with M205 T.//（mm/s）最小行程进给速度。设置为M205 T。

// Minimum time that a segment needs to take as the buffer gets emptied//缓冲区清空时段需要的最短时间
#define DEFAULT_MINSEGMENTTIME        20000   // (µs) Set with M205 B.//（µs）设置为M205 B。

// Slow down the machine if the lookahead buffer is (by default) half full.//如果前瞻缓冲区（默认情况下）已满一半，请减速机器。
// Increase the slowdown divisor for larger buffer sizes.//对于较大的缓冲区大小，增加减速因子。
#define SLOWDOWN
#if ENABLED(SLOWDOWN)
#define SLOWDOWN_DIVISOR 2
#endif

/**
 * XY Frequency limit
 * Reduce resonance by limiting the frequency of small zigzag infill moves.
 * See https://hydraraptor.blogspot.com/2010/12/frequency-limit.html
 * Use M201 F<freq> G<min%> to change limits at runtime.
 */
//#define XY_FREQUENCY_LIMIT      10 // (Hz) Maximum frequency of small zigzag infill moves. Set with M201 F<hertz>.//#定义XY_频率_限制10//（Hz）小Z字形填充移动的最大频率。设置为M201 F<赫兹>。
#ifdef XY_FREQUENCY_LIMIT
#define XY_FREQUENCY_MIN_PERCENT 5 // (percent) Minimum FR percentage to apply. Set with M201 G<min%>.//（百分比）适用的最低FR百分比。设置M201 G<min%>。
#endif

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end//最小规划器连接速度。设置计划器在结束时计划的默认最小速度
// of the buffer and all stops. This should not be much greater than zero and should only be changed//缓冲区和所有停止。该值不应远大于零，并且只应进行更改
// if unwanted behavior is observed on a user's machine when running at very slow speeds.//如果在用户的机器上以非常慢的速度运行时观察到不必要的行为。
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/s)//（毫米/秒）

////
// Backlash Compensation//齿隙补偿
// Adds extra movement to axes on direction-changes to account for backlash.//在方向改变时向轴添加额外的移动，以考虑齿隙。
////
//#define BACKLASH_COMPENSATION//#定义齿隙补偿
#if ENABLED(BACKLASH_COMPENSATION)
// Define values for backlash distance and correction.//定义齿隙距离和修正值。
  // If BACKLASH_GCODE is enabled these values are the defaults.//如果启用了齿隙代码，则这些值为默认值。
  #define BACKLASH_DISTANCE_MM { 0, 0, 0 } // (mm) One value for each linear axis//（mm）每个线性轴一个值
  #define BACKLASH_CORRECTION    0.0       // 0.0 = no correction; 1.0 = full correction//0.0=无校正；1.0=完全校正

  // Add steps for motor direction changes on CORE kinematics//添加在核心运动学上更改电机方向的步骤
  //#define CORE_BACKLASH//#定义型芯齿隙

  // Set BACKLASH_SMOOTHING_MM to spread backlash correction over multiple segments//设置齿隙_平滑_MM以将齿隙校正扩展到多个齿段
  // to reduce print artifacts. (Enabling this is costly in memory and computation!)//减少打印瑕疵。（启用此功能会耗费大量内存和计算！）
  //#define BACKLASH_SMOOTHING_MM 3 // (mm)//#定义齿隙_平滑_MM 3/（MM）

  // Add runtime configuration and tuning of backlash values (M425)//添加运行时配置和齿隙值调整（M425）
  //#define BACKLASH_GCODE//#定义齿隙代码

  #if ENABLED(BACKLASH_GCODE)
    // Measure the Z backlash when probing (G29) and set with "M425 Z"//探测（G29）时测量Z齿隙并设置为“M425 Z”
    #define MEASURE_BACKLASH_WHEN_PROBING

    #if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
      // When measuring, the probe will move up to BACKLASH_MEASUREMENT_LIMIT//测量时，探头将向上移动至齿隙测量极限
      // mm away from point of contact in BACKLASH_MEASUREMENT_RESOLUTION//在齿隙测量分辨率中，距离接触点mm
      // increments while checking for the contact to be broken.//在检查触点是否断开时增加。
      #define BACKLASH_MEASUREMENT_LIMIT       0.5   // (mm)//（毫米）
      #define BACKLASH_MEASUREMENT_RESOLUTION  0.005 // (mm)//（毫米）
      #define BACKLASH_MEASUREMENT_FEEDRATE    Z_PROBE_FEEDRATE_SLOW // (mm/min)//（毫米/分钟）
    #endif
  #endif
#endif

/**
 * Automatic backlash, position and hotend offset calibration
 *
 * Enable G425 to run automatic calibration using an electrically-
 * conductive cube, bolt, or washer mounted on the bed.
 *
 * G425 uses the probe to touch the top and sides of the calibration object
 * on the bed and measures and/or correct positional offsets, axis backlash
 * and hotend offsets.
 *
 * Note: HOTEND_OFFSET and CALIBRATION_OBJECT_CENTER must be set to within
 *       ±5mm of true values for G425 to succeed.
 */
//#define CALIBRATION_GCODE//#定义校准代码
#if ENABLED(CALIBRATION_GCODE)

//#define CALIBRATION_SCRIPT_PRE  "M117 Starting Auto-Calibration\nT0\nG28\nG12\nM117 Calibrating..."//#定义校准脚本“M117开始自动校准\nT0\nG28\nG12\nM117校准…”
  //#define CALIBRATION_SCRIPT_POST "M500\nM117 Calibration data saved"//#定义校准\u脚本\u POST“M500\nM117已保存校准数据”

  #define CALIBRATION_MEASUREMENT_RESOLUTION     0.01 // mm//嗯

  #define CALIBRATION_FEEDRATE_SLOW             60    // mm/min//毫米/分钟
  #define CALIBRATION_FEEDRATE_FAST           1200    // mm/min//毫米/分钟
  #define CALIBRATION_FEEDRATE_TRAVEL         3000    // mm/min//毫米/分钟

  // The following parameters refer to the conical section of the nozzle tip.//以下参数适用于喷嘴尖端的锥形截面。
  #define CALIBRATION_NOZZLE_TIP_HEIGHT          1.0  // mm//嗯
  #define CALIBRATION_NOZZLE_OUTER_DIAMETER      2.0  // mm//嗯

  // Uncomment to enable reporting (required for "G425 V", but consumes PROGMEM).//取消注释以启用报告（对于“G425 V”是必需的，但会消耗PROGMEM）。
  //#define CALIBRATION_REPORTING//#定义校准报告

  // The true location and dimension the cube/bolt/washer on the bed.//底座上立方体/螺栓/垫圈的真实位置和尺寸。
  #define CALIBRATION_OBJECT_CENTER     { 264.0, -22.0,  -2.0 } // mm//嗯
  #define CALIBRATION_OBJECT_DIMENSIONS {  10.0,  10.0,  10.0 } // mm//嗯

  // Comment out any sides which are unreachable by the probe. For best//注释掉探头无法触及的任何面。最好
  // auto-calibration results, all sides must be reachable.//自动校准结果，所有侧面都必须可到达。
  #define CALIBRATION_MEASURE_RIGHT
  #define CALIBRATION_MEASURE_FRONT
  #define CALIBRATION_MEASURE_LEFT
  #define CALIBRATION_MEASURE_BACK

  //#define CALIBRATION_MEASURE_IMIN//#定义校准和测量
  //#define CALIBRATION_MEASURE_IMAX//#定义校准\测量\ IMAX
  //#define CALIBRATION_MEASURE_JMIN//#定义校准和测量
  //#define CALIBRATION_MEASURE_JMAX//#定义校准_测量_JMAX
  //#define CALIBRATION_MEASURE_KMIN//#定义校准\u测量\u KMIN
  //#define CALIBRATION_MEASURE_KMAX//#定义校准\u测量\u KMAX

  // Probing at the exact top center only works if the center is flat. If//只有在中心是平的情况下，才能在准确的顶部中心进行探测。如果
  // probing on a screwhead or hollow washer, probe near the edges.//在螺丝头或空心垫圈上探测，在边缘附近探测。
  //#define CALIBRATION_MEASURE_AT_TOP_EDGES//#在顶部边缘定义校准和测量

  // Define the pin to read during calibration//定义校准期间要读取的管脚
  #ifndef CALIBRATION_PIN
    //#define CALIBRATION_PIN -1            // Define here to override the default pin//#定义校准引脚-1//在此处定义以覆盖默认引脚
    #define CALIBRATION_PIN_INVERTING false // Set to true to invert the custom pin//设置为true可反转自定义接点
    //#define CALIBRATION_PIN_PULLDOWN//#定义校准引脚下拉列表
    #define CALIBRATION_PIN_PULLUP
  #endif
#endif

/**
 * Adaptive Step Smoothing increases the resolution of multi-axis moves, particularly at step frequencies
 * below 1kHz (for AVR) or 10kHz (for ARM), where aliasing between axes in multi-axis moves causes audible
 * vibration and surface artifacts. The algorithm adapts to provide the best possible step smoothing at the
 * lowest stepping frequencies.
 */
//#define ADAPTIVE_STEP_SMOOTHING//#定义自适应步长平滑

/**
 * Custom Microstepping
 * Override as-needed for your setup. Up to 3 MS pins are supported.
 */
//#define MICROSTEP1 LOW,LOW,LOW//#定义微步1低、低、低
//#define MICROSTEP2 HIGH,LOW,LOW//#定义微步2高、低、低
//#define MICROSTEP4 LOW,HIGH,LOW//#定义微步4低、高、低
//#define MICROSTEP8 HIGH,HIGH,LOW//#定义微步8高、高、低
//#define MICROSTEP16 LOW,LOW,HIGH//#定义微步16低、低、高
//#define MICROSTEP32 HIGH,LOW,HIGH//#定义微步32高、低、高

// Microstep settings (Requires a board with pins named X_MS1, X_MS2, etc.)//微步设置（需要一块带有引脚名为X_MS1、X_MS2等的板）
#define MICROSTEP_MODES { 16, 16, 16, 16, 16, 16 } // [1,2,4,8,16]// [1,2,4,8,16]

/**
 *  @section  stepper motor current
 *
 *  Some boards have a means of setting the stepper motor current via firmware.
 *
 *  The power on motor currents are set by:
 *    PWM_MOTOR_CURRENT - used by MINIRAMBO & ULTIMAIN_2
 *                         known compatible chips: A4982
 *    DIGIPOT_MOTOR_CURRENT - used by BQ_ZUM_MEGA_3D, RAMBO & SCOOVO_X9H
 *                         known compatible chips: AD5206
 *    DAC_MOTOR_CURRENT_DEFAULT - used by PRINTRBOARD_REVF & RIGIDBOARD_V2
 *                         known compatible chips: MCP4728
 *    DIGIPOT_I2C_MOTOR_CURRENTS - used by 5DPRINT, AZTEEG_X3_PRO, AZTEEG_X5_MINI_WIFI, MIGHTYBOARD_REVE
 *                         known compatible chips: MCP4451, MCP4018
 *
 *  Motor currents can also be set by M907 - M910 and by the LCD.
 *    M907 - applies to all.
 *    M908 - BQ_ZUM_MEGA_3D, RAMBO, PRINTRBOARD_REVF, RIGIDBOARD_V2 & SCOOVO_X9H
 *    M909, M910 & LCD - only PRINTRBOARD_REVF & RIGIDBOARD_V2
 */
//#define PWM_MOTOR_CURRENT { 1300, 1300, 1250 }          // Values in milliamps//#定义PWM_电机_电流{1300、1300、1250}//以毫安为单位的值
//#define DIGIPOT_MOTOR_CURRENT { 135,135,135,135,135 }   // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)//#定义数字电机电流{135135}//值0-255（兰博135=~0.75A，185=~1A）
//#define DAC_MOTOR_CURRENT_DEFAULT { 70, 80, 90, 80 }    // Default drive percent - X, Y, Z, E axis//#定义DAC_电机_电流_默认值{70,80,90,80}//默认驱动百分比-X、Y、Z、E轴

/**
 * I2C-based DIGIPOTs (e.g., Azteeg X3 Pro)
 */
//#define DIGIPOT_MCP4018             // Requires https://github.com/felias-fogg/SlowSoftI2CMaster//#定义DIGIPOT_MCP4018//需要https://github.com/felias-fogg/SlowSoftI2CMaster
//#define DIGIPOT_MCP4451//#定义DIGIPOT_MCP4451
#if EITHER(DIGIPOT_MCP4018, DIGIPOT_MCP4451)
#define DIGIPOT_I2C_NUM_CHANNELS 8  // 5DPRINT:4   AZTEEG_X3_PRO:8   MKS_SBASE:5   MIGHTYBOARD_REVE:5//5点：4个AZTEEG_X3_专业版：8个MKS_SBASE:5个可能的版本：5个

  // Actual motor currents in Amps. The number of entries must match DIGIPOT_I2C_NUM_CHANNELS.//以安培为单位的实际电机电流。条目的数量必须与DIGIPOT_I2C_NUM_通道匹配。
  // These correspond to the physical drivers, so be mindful if the order is changed.//这些与物理驱动程序相对应，因此如果顺序发生变化，请注意。
  #define DIGIPOT_I2C_MOTOR_CURRENTS { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 } // AZTEEG_X3_PRO//阿兹提格职业选手

  //#define DIGIPOT_USE_RAW_VALUES    // Use DIGIPOT_MOTOR_CURRENT raw wiper values (instead of A4988 motor currents)//#定义DIGIPOT_使用原始值//使用DIGIPOT_电机当前原始雨刮器值（而不是A4988电机电流）

  /**
   * Common slave addresses:
   *
   *                        A   (A shifted)   B   (B shifted)  IC
   * Smoothie              0x2C (0x58)       0x2D (0x5A)       MCP4451
   * AZTEEG_X3_PRO         0x2C (0x58)       0x2E (0x5C)       MCP4451
   * AZTEEG_X5_MINI        0x2C (0x58)       0x2E (0x5C)       MCP4451
   * AZTEEG_X5_MINI_WIFI         0x58              0x5C        MCP4451
   * MIGHTYBOARD_REVE      0x2F (0x5E)                         MCP4018
   */
  //#define DIGIPOT_I2C_ADDRESS_A 0x2C  // Unshifted slave address for first DIGIPOT//#为第一个DIGIPOT定义DIGIPOT_I2C_地址\u 0x2C//未移位的从属地址
  //#define DIGIPOT_I2C_ADDRESS_B 0x2D  // Unshifted slave address for second DIGIPOT//#定义DIGIPOT_I2C_地址_B 0x2D//第二个DIGIPOT的未移位从属地址
#endif

//===========================================================================//===========================================================================
//=============================Additional Features===========================//==============================================其他功能===========================
//===========================================================================//===========================================================================

// @section lcd//@section液晶显示器

#if ANY(HAS_LCD_MENU, EXTENSIBLE_UI, HAS_DWIN_E3V2)
#define MANUAL_FEEDRATE { 50*60, 50*60, 4*60, 2*60 } // (mm/min) Feedrates for manual moves along X, Y, Z, E from panel//（mm/min）从面板沿X、Y、Z、E手动移动的进给速度
  #define FINE_MANUAL_MOVE 0.025    // (mm) Smallest manual move (< 0.1mm) applying to Z on most machines//（mm）适用于大多数机器Z的最小手动移动（<0.1mm）
  #if IS_ULTIPANEL
    #define MANUAL_E_MOVES_RELATIVE // Display extruder move distance rather than "position"//显示挤出机移动距离而不是“位置”
    #define ULTIPANEL_FEEDMULTIPLY  // Encoder sets the feedrate multiplier on the Status Screen//编码器在状态屏幕上设置进给速度倍增器
  #endif
#endif

// Change values more rapidly when the encoder is rotated faster//当编码器旋转得更快时，更改值的速度更快
#define ENCODER_RATE_MULTIPLIER
#if ENABLED(ENCODER_RATE_MULTIPLIER)
#define ENCODER_10X_STEPS_PER_SEC   30  // (steps/s) Encoder rate for 10x speed//（步/秒）编码器速度为10倍
  #define ENCODER_100X_STEPS_PER_SEC  80  // (steps/s) Encoder rate for 100x speed//速度为100倍的编码器速率（步/秒）
#endif

// Play a beep when the feedrate is changed from the Status Screen//从状态屏幕更改进给速度时，播放蜂鸣音
//#define BEEP_ON_FEEDRATE_CHANGE//#在进给速度变化时定义蜂鸣音
#if ENABLED(BEEP_ON_FEEDRATE_CHANGE)
#define FEEDRATE_CHANGE_BEEP_DURATION   10
  #define FEEDRATE_CHANGE_BEEP_FREQUENCY 440
#endif

#if HAS_LCD_MENU

// Add Probe Z Offset calibration to the Z Probe Offsets menu//将探头Z偏移校准添加到Z探头偏移菜单
  #if HAS_BED_PROBE
    //#define PROBE_OFFSET_WIZARD//#定义探测偏移向导
    #if ENABLED(PROBE_OFFSET_WIZARD)
      ////
      // Enable to init the Probe Z-Offset when starting the Wizard.//启动向导时启用以初始化探测器Z偏移。
      // Use a height slightly above the estimated nozzle-to-probe Z offset.//使用略高于估计喷嘴的高度探测Z偏移。
      // For example, with an offset of -5, consider a starting height of -4.例如，在偏移量为-5时，考虑起始高度为-4。
      ////
      //#define PROBE_OFFSET_WIZARD_START_Z -4.0//#定义探测器\u偏移量\u向导\u开始\u Z-4.0

      // Set a convenient position to do the calibration (probing point and nozzle/bed-distance)//设置一个方便的位置进行校准（探测点和喷嘴/底座距离）
      //#define PROBE_OFFSET_WIZARD_XY_POS { X_CENTER, Y_CENTER }//#定义探测器偏移向导\u XY\u位置{X\u中心，Y\u中心}
    #endif
  #endif

  // Include a page of printer information in the LCD Main Menu//在LCD主菜单中包括一页打印机信息
  //#define LCD_INFO_MENU//#定义LCD\u信息\u菜单
  #if ENABLED(LCD_INFO_MENU)
    //#define LCD_PRINTER_INFO_IS_BOOTSCREEN // Show bootscreen(s) instead of Printer Info pages//#定义LCD_打印机_信息_是_引导屏幕//显示引导屏幕而不是打印机信息页面
  #endif

  // BACK menu items keep the highlight at the top//“后退”菜单项使突出显示保持在顶部
  //#define TURBO_BACK_MENU_ITEM//#定义“返回”菜单项

  // Add a mute option to the LCD menu//在LCD菜单中添加静音选项
  //#define SOUND_MENU_ITEM//#定义声音菜单项

  /**
   * LED Control Menu
   * Add LED Control to the LCD menu
   */
  //#define LED_CONTROL_MENU//#定义LED_控制_菜单
  #if ENABLED(LED_CONTROL_MENU)
    #define LED_COLOR_PRESETS                 // Enable the Preset Color menu option//启用预设颜色菜单选项
    //#define NEO2_COLOR_PRESETS              // Enable a second NeoPixel Preset Color menu option//#定义NEO2_颜色_预设//启用第二个NeoPixel预设颜色菜单选项
    #if ENABLED(LED_COLOR_PRESETS)
      #define LED_USER_PRESET_RED        255  // User defined RED value//用户定义的红色值
      #define LED_USER_PRESET_GREEN      128  // User defined GREEN value//用户定义的绿色值
      #define LED_USER_PRESET_BLUE         0  // User defined BLUE value//用户定义的蓝色值
      #define LED_USER_PRESET_WHITE      255  // User defined WHITE value//用户定义的白色值
      #define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity//用户定义的强度
      //#define LED_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup//#定义LED_用户_预设_启动//让打印机在启动时显示用户预设颜色
    #endif
    #if ENABLED(NEO2_COLOR_PRESETS)
      #define NEO2_USER_PRESET_RED        255  // User defined RED value//用户定义的红色值
      #define NEO2_USER_PRESET_GREEN      128  // User defined GREEN value//用户定义的绿色值
      #define NEO2_USER_PRESET_BLUE         0  // User defined BLUE value//用户定义的蓝色值
      #define NEO2_USER_PRESET_WHITE      255  // User defined WHITE value//用户定义的白色值
      #define NEO2_USER_PRESET_BRIGHTNESS 255  // User defined intensity//用户定义的强度
      //#define NEO2_USER_PRESET_STARTUP       // Have the printer display the user preset color on startup for the second strip//#定义NEO2\u USER\u PRESET\u STARTUP//让打印机在启动时显示第二条色带的用户预设颜色
    #endif
  #endif

  // Insert a menu for preheating at the top level to allow for quick access//在顶层插入预热菜单，以便快速访问
  //#define PREHEAT_SHORTCUT_MENU_ITEM//#定义预热快捷菜单项

#endif // HAS_LCD_MENU//有LCD菜单吗

#if HAS_DISPLAY
// The timeout (in ms) to return to the status screen from sub-menus//从子菜单返回状态屏幕的超时（毫秒）
  //#define LCD_TIMEOUT_TO_STATUS 15000//#将LCD_超时_定义为_状态15000

  #if ENABLED(SHOW_BOOTSCREEN)
    #define BOOTSCREEN_TIMEOUT 4000      // (ms) Total Duration to display the boot screen(s)//（ms）显示启动屏幕的总持续时间
    #if EITHER(HAS_MARLINUI_U8GLIB, TFT_COLOR_UI)
      #define BOOT_MARLIN_LOGO_SMALL     // Show a smaller Marlin logo on the Boot Screen (saving lots of flash)//在开机屏幕上显示较小的马林鱼徽标（节省大量闪光灯）
    #endif
  #endif

  // Scroll a longer status message into view//将较长的状态消息滚动到视图中
  //#define STATUS_MESSAGE_SCROLLING//#定义状态\消息\滚动

  // On the Info Screen, display XY with one decimal place when possible//在信息屏幕上，尽可能显示带一位小数的XY
  //#define LCD_DECIMAL_SMALL_XY//#定义LCD_十进制_小_XY

  // Add an 'M73' G-code to set the current percentage//添加“M73”G代码以设置当前百分比
  //#define LCD_SET_PROGRESS_MANUALLY//#手动定义LCD\u设置\u进度\u

  // Show the E position (filament used) during printing//显示打印期间的E位置（使用的灯丝）
  //#define LCD_SHOW_E_TOTAL//#定义LCD\u显示\u E\u总计
#endif

// LCD Print Progress options//LCD打印进度选项
#if EITHER(SDSUPPORT, LCD_SET_PROGRESS_MANUALLY)
#if ANY(HAS_MARLINUI_U8GLIB, EXTENSIBLE_UI, HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL, IS_DWIN_MARLINUI)
    //#define SHOW_REMAINING_TIME         // Display estimated time to completion//#定义显示剩余时间//显示预计完成时间
    #if ENABLED(SHOW_REMAINING_TIME)
      //#define USE_M73_REMAINING_TIME    // Use remaining time from M73 command instead of estimation//#定义使用\u M73\u剩余\u时间//使用来自M73命令的剩余时间，而不是估计
      //#define ROTATE_PROGRESS_DISPLAY   // Display (P)rogress, (E)lapsed, and (R)emaining time//#定义旋转\进度\显示//显示（P）进度、（E）延时和（R）显示时间
    #endif
  #endif

  #if EITHER(HAS_MARLINUI_U8GLIB, EXTENSIBLE_UI)
    //#define PRINT_PROGRESS_SHOW_DECIMALS // Show progress with decimal digits//#定义打印\进度\显示\小数//用十进制数字显示进度
  #endif

  #if EITHER(HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL)
    //#define LCD_PROGRESS_BAR            // Show a progress bar on HD44780 LCDs for SD printing//#定义LCD\进度\栏//在HD44780 LCD上显示进度栏以进行SD打印
    #if ENABLED(LCD_PROGRESS_BAR)
      #define PROGRESS_BAR_BAR_TIME 2000  // (ms) Amount of time to show the bar//（ms）显示条形图的时间量
      #define PROGRESS_BAR_MSG_TIME 3000  // (ms) Amount of time to show the status message//（ms）显示状态消息的时间量
      #define PROGRESS_MSG_EXPIRE   0     // (ms) Amount of time to retain the status message (0=forever)//（ms）保留状态消息的时间量（0=永久）
      //#define PROGRESS_MSG_ONCE         // Show the message for MSG_TIME then clear it//#定义一次进度消息//显示消息时间消息，然后清除它
      //#define LCD_PROGRESS_BAR_TEST     // Add a menu item to test the progress bar//#定义LCD\u进度条\u测试//添加一个菜单项以测试进度条
    #endif
  #endif
#endif

#if ENABLED(SDSUPPORT)
/**
   * SD Card SPI Speed
   * May be required to resolve "volume init" errors.
   *
   * Enable and set to SPI_HALF_SPEED, SPI_QUARTER_SPEED, or SPI_EIGHTH_SPEED
   *  otherwise full speed will be applied.
   *
   * :['SPI_HALF_SPEED', 'SPI_QUARTER_SPEED', 'SPI_EIGHTH_SPEED']
   */
  //#define SD_SPI_SPEED SPI_HALF_SPEED//#定义SD_SPI_速度SPI_半速

  // The standard SD detect circuit reads LOW when media is inserted and HIGH when empty.//插入介质时，标准SD检测电路的读数为低，空时为高。
  // Enable this option and set to HIGH if your SD cards are incorrectly detected.//如果未正确检测到SD卡，请启用此选项并设置为高。
  //#define SD_DETECT_STATE HIGH//#定义SD_检测_状态高

  //#define SD_IGNORE_AT_STARTUP            // Don't mount the SD card when starting up//#在启动时定义SD\U忽略//启动时不要挂载SD卡
  //#define SDCARD_READONLY                 // Read-only SD card (to save over 2K of flash)//#定义SD卡\ U只读//只读SD卡（节省超过2K的闪存）

  //#define GCODE_REPEAT_MARKERS            // Enable G-code M808 to set repeat markers and do looping//#定义GCODE_REPEAT_标记//启用G代码M808以设置重复标记并执行循环

  #define SD_PROCEDURE_DEPTH 1              // Increase if you need more nested M32 calls//如果需要更多嵌套M32调用，请增加

  #define SD_FINISHED_STEPPERRELEASE true   // Disable steppers when SD Print is finished//SD打印完成后禁用步进器
  #define SD_FINISHED_RELEASECOMMAND "M84"  // Use "M84XYE" to keep Z enabled so your bed stays in place//使用“M84XYE”使Z处于启用状态，以便您的床保持在原位

  // Reverse SD sort to show "more recent" files first, according to the card's FAT.//根据卡片的FAT，反向SD排序首先显示“最近”的文件。
  // Since the FAT gets out of order with usage, SDCARD_SORT_ALPHA is recommended.//由于FAT在使用过程中出现问题，建议使用SDCARD_SORT_ALPHA。
  #define SDCARD_RATHERRECENTFIRST

  #define SD_MENU_CONFIRM_START             // Confirm the selected SD file before printing//打印前确认选定的SD文件

  //#define NO_SD_AUTOSTART                 // Remove auto#.g file support completely to save some Flash, SRAM//#定义NO_SD_AUTOSTART//完全删除auto#.g文件支持以保存一些闪存、SRAM
  //#define MENU_ADDAUTOSTART               // Add a menu option to run auto#.g files//#定义菜单添加自动启动//添加一个菜单选项以运行auto#.g文件

  //#define BROWSE_MEDIA_ON_INSERT          // Open the file browser when media is inserted//#在插入时定义浏览介质//插入介质时打开文件浏览器

  //#define MEDIA_MENU_AT_TOP               // Force the media menu to be listed on the top of the main menu//#在顶部定义媒体菜单//强制将媒体菜单列在主菜单顶部

  #define EVENT_GCODE_SD_ABORT "G28XY"      // G-code to run on SD Abort Print (e.g., "G28XY" or "G27")//在SD中止打印上运行的G代码（例如，“G28XY”或“G27”）

  #if ENABLED(PRINTER_EVENT_LEDS)
    #define PE_LEDS_COMPLETED_TIME  (30*60) // (seconds) Time to keep the LED "done" color before restoring normal illumination//（秒）恢复正常照明前保持LED“完成”颜色的时间
  #endif

  /**
   * Continue after Power-Loss (Creality3D)
   *
   * Store the current state to the SD Card at the start of each layer
   * during SD printing. If the recovery file is found at boot time, present
   * an option on the LCD screen to continue the print from the last-known
   * point in the file.
   */
  //#define POWER_LOSS_RECOVERY//#定义功率损耗和恢复
  #if ENABLED(POWER_LOSS_RECOVERY)
    #define PLR_ENABLED_DEFAULT   false // Power Loss Recovery enabled by default. (Set with 'M413 Sn' & M500)//默认情况下启用电源损耗恢复。（设置为“M413序列号”和M500）
    //#define BACKUP_POWER_SUPPLY       // Backup power / UPS to move the steppers on power loss//#定义备用电源//备用电源/UPS，以便在断电时移动步进电机
    //#define POWER_LOSS_ZRAISE       2 // (mm) Z axis raise on resume (on power loss with UPS)//#定义功率损耗，恢复时Z轴上升2//（mm）（UPS功率损耗时）
    //#define POWER_LOSS_PIN         44 // Pin to detect power loss. Set to -1 to disable default pin on boards without module.//#定义功耗引脚44//引脚以检测功耗。设置为-1可禁用无模块板上的默认引脚。
    //#define POWER_LOSS_STATE     HIGH // State of pin indicating power loss//#定义功率损耗状态高//指示功率损耗的引脚状态
    //#define POWER_LOSS_PULLUP         // Set pullup / pulldown as appropriate for your sensor//#定义功率损耗上拉//根据传感器的需要设置上拉/下拉
    //#define POWER_LOSS_PULLDOWN//#定义功率损耗下拉列表
    //#define POWER_LOSS_PURGE_LEN   20 // (mm) Length of filament to purge on resume//#定义恢复时要吹扫的灯丝的功率损失吹扫长度为20/（mm）
    //#define POWER_LOSS_RETRACT_LEN 10 // (mm) Length of filament to retract on fail. Requires backup power.//#定义失效时缩回灯丝的功率损耗缩回长度为10//（mm）。需要备用电源。

    // Without a POWER_LOSS_PIN the following option helps reduce wear on the SD card,//在没有电源损耗引脚的情况下，以下选项有助于减少SD卡的磨损，
    // especially with "vase mode" printing. Set too high and vases cannot be continued.//特别是“花瓶模式”印刷。设置过高，花瓶无法继续。
    #define POWER_LOSS_MIN_Z_CHANGE 0.05 // (mm) Minimum Z change before saving power-loss data//（mm）保存功率损耗数据前的最小Z变化

    // Enable if Z homing is needed for proper recovery. 99.9% of the time this should be disabled!//如果正确恢复需要Z原点，则启用。99.9%的时间应禁用此功能！
    //#define POWER_LOSS_RECOVER_ZHOME//#定义电源损耗和恢复
    #if ENABLED(POWER_LOSS_RECOVER_ZHOME)
      //#define POWER_LOSS_ZHOME_POS { 0, 0 } // Safe XY position to home Z while avoiding objects on the bed//#在避免床上物体的同时，定义电源损耗home位置{0，0}//安全XY位置到home Z
    #endif
  #endif

  /**
   * Sort SD file listings in alphabetical order.
   *
   * With this option enabled, items on SD cards will be sorted
   * by name for easier navigation.
   *
   * By default...
   *
   *  - Use the slowest -but safest- method for sorting.
   *  - Folders are sorted to the top.
   *  - The sort key is statically allocated.
   *  - No added G-code (M34) support.
   *  - 40 item sorting limit. (Items after the first 40 are unsorted.)
   *
   * SD sorting uses static allocation (as set by SDSORT_LIMIT), allowing the
   * compiler to calculate the worst-case usage and throw an error if the SRAM
   * limit is exceeded.
   *
   *  - SDSORT_USES_RAM provides faster sorting via a static directory buffer.
   *  - SDSORT_USES_STACK does the same, but uses a local stack-based buffer.
   *  - SDSORT_CACHE_NAMES will retain the sorted file listing in RAM. (Expensive!)
   *  - SDSORT_DYNAMIC_RAM only uses RAM when the SD menu is visible. (Use with caution!)
   */
  //#define SDCARD_SORT_ALPHA//#定义SDCARD_SORT_ALPHA

  // SD Card Sorting options//SD卡排序选项
  #if ENABLED(SDCARD_SORT_ALPHA)
    #define SDSORT_LIMIT       40     // Maximum number of sorted items (10-256). Costs 27 bytes each.//已排序项目的最大数量（10-256）。每个成本为27字节。
    #define FOLDER_SORTING     -1     // -1=above  0=none  1=below//-1=高于0=无1=低于
    #define SDSORT_GCODE       false  // Allow turning sorting on/off with LCD and M34 G-code.//允许使用LCD和M34 G代码打开/关闭分拣。
    #define SDSORT_USES_RAM    false  // Pre-allocate a static array for faster pre-sorting.//预分配静态数组以加快预排序。
    #define SDSORT_USES_STACK  false  // Prefer the stack for pre-sorting to give back some SRAM. (Negated by next 2 options.)//首选用于预排序的堆栈，以返回一些SRAM。（由下两个选项否定。）
    #define SDSORT_CACHE_NAMES false  // Keep sorted items in RAM longer for speedy performance. Most expensive option.//将已排序的项目在RAM中保留更长的时间，以便快速执行。最昂贵的选择。
    #define SDSORT_DYNAMIC_RAM false  // Use dynamic allocation (within SD menus). Least expensive option. Set SDSORT_LIMIT before use!//使用动态分配（在SD菜单中）。最便宜的选择。使用前设置SDSORT_限制！
    #define SDSORT_CACHE_VFATS 2      // Maximum number of 13-byte VFAT entries to use for sorting.//用于排序的最大13字节VFAT条目数。
                                      // Note: Only affects SCROLL_LONG_FILENAMES with SDSORT_CACHE_NAMES but not SDSORT_DYNAMIC_RAM.//注意：仅影响具有SDSORT\u缓存\u名称的SCROLL\u LONG\u文件名，而不影响SDSORT\u DYNAMIC\u RAM。
  #endif

  // Allow international symbols in long filenames. To display correctly, the//允许在长文件名中使用国际符号。要正确显示，请将
  // LCD's font must contain the characters. Check your selected LCD language.//LCD的字体必须包含字符。检查您选择的LCD语言。
  //#define UTF_FILENAME_SUPPORT//#定义UTF\u文件名\u支持

  // This allows hosts to request long names for files and folders with M33//这允许主机请求使用M33的文件和文件夹的长名称
  //#define LONG_FILENAME_HOST_SUPPORT//#定义长\u文件名\u主机\u支持

  // Enable this option to scroll long filenames in the SD card menu//启用此选项可在SD卡菜单中滚动长文件名
  //#define SCROLL_LONG_FILENAMES//#定义滚动\u长\u文件名

  // Leave the heaters on after Stop Print (not recommended!)//停止打印后保持加热器打开（不建议！）
  //#define SD_ABORT_NO_COOLDOWN//#定义SD\u中止\u否\u冷却时间

  /**
   * This option allows you to abort SD printing when any endstop is triggered.
   * This feature must be enabled with "M540 S1" or from the LCD menu.
   * To have any effect, endstops must be enabled during SD printing.
   */
  //#define SD_ABORT_ON_ENDSTOP_HIT//#在结束停止命中时定义SD\U ABORT\U

  /**
   * This option makes it easier to print the same SD Card file again.
   * On print completion the LCD Menu will open with the file selected.
   * You can just click to start the print, or navigate elsewhere.
   */
  //#define SD_REPRINT_LAST_SELECTED_FILE//#定义SD\u重新打印\u上次选择的\u文件

  /**
   * Auto-report SdCard status with M27 S<seconds>
   */
  //#define AUTO_REPORT_SD_STATUS//#定义自动报告状态

  /**
   * Support for USB thumb drives using an Arduino USB Host Shield or
   * equivalent MAX3421E breakout board. The USB thumb drive will appear
   * to Marlin as an SD card.
   *
   * The MAX3421E can be assigned the same pins as the SD card reader, with
   * the following pin mapping:
   *
   *    SCLK, MOSI, MISO --> SCLK, MOSI, MISO
   *    INT              --> SD_DETECT_PIN [1]
   *    SS               --> SDSS
   *
   * [1] On AVR an interrupt-capable pin is best for UHS3 compatibility.
   */
  //#define USB_FLASH_DRIVE_SUPPORT//#定义USB\U闪存\U驱动器\U支持
  #if ENABLED(USB_FLASH_DRIVE_SUPPORT)
    /**
     * USB Host Shield Library
     *
     * - UHS2 uses no interrupts and has been production-tested
     *   on a LulzBot TAZ Pro with a 32-bit Archim board.
     *
     * - UHS3 is newer code with better USB compatibility. But it
     *   is less tested and is known to interfere with Servos.
     *   [1] This requires USB_INTR_PIN to be interrupt-capable.
     */
    //#define USE_UHS2_USB//#定义使用\u UHS2\u USB
    //#define USE_UHS3_USB//#定义使用\u UHS3\u USB

    /**
     * Native USB Host supported by some boards (USB OTG)
     */
    //#define USE_OTG_USB_HOST//#定义使用\u OTG\u USB\u主机

    #if DISABLED(USE_OTG_USB_HOST)
      #define USB_CS_PIN    SDSS
      #define USB_INTR_PIN  SD_DETECT_PIN
    #endif
  #endif

  /**
   * When using a bootloader that supports SD-Firmware-Flashing,
   * add a menu item to activate SD-FW-Update on the next reboot.
   *
   * Requires ATMEGA2560 (Arduino Mega)
   *
   * Tested with this bootloader:
   *   https://github.com/FleetProbe/MicroBridge-Arduino-ATMega2560
   */
  //#define SD_FIRMWARE_UPDATE//#定义SD_固件_更新
  #if ENABLED(SD_FIRMWARE_UPDATE)
    #define SD_FIRMWARE_UPDATE_EEPROM_ADDR    0x1FF
    #define SD_FIRMWARE_UPDATE_ACTIVE_VALUE   0xF0
    #define SD_FIRMWARE_UPDATE_INACTIVE_VALUE 0xFF
  #endif

  // Add an optimized binary file transfer mode, initiated with 'M28 B1'//添加优化的二进制文件传输模式，由“M28 B1”启动
  //#define BINARY_FILE_TRANSFER//#定义二进制文件传输

  /**
   * Set this option to one of the following (or the board's defaults apply):
   *
   *           LCD - Use the SD drive in the external LCD controller.
   *       ONBOARD - Use the SD drive on the control board.
   *  CUSTOM_CABLE - Use a custom cable to access the SD (as defined in a pins file).
   *
   * :[ 'LCD', 'ONBOARD', 'CUSTOM_CABLE' ]
   */
  //#define SDCARD_CONNECTION LCD//#定义SDCARD_连接LCD

  // Enable if SD detect is rendered useless (e.g., by using an SD extender)//如果SD detect变得无用（例如，通过使用SD扩展器）则启用
  //#define NO_SD_DETECT//#定义无故障检测

  // Multiple volume support - EXPERIMENTAL.//多卷支持-实验性。
  //#define MULTI_VOLUME//#定义多卷
  #if ENABLED(MULTI_VOLUME)
    #define VOLUME_SD_ONBOARD
    #define VOLUME_USB_FLASH_DRIVE
    #define DEFAULT_VOLUME SV_SD_ONBOARD
    #define DEFAULT_SHARED_VOLUME SV_USB_FLASH_DRIVE
  #endif

#endif // SDSUPPORT//SDSUPPORT

/**
 * By default an onboard SD card reader may be shared as a USB mass-
 * storage device. This option hides the SD card from the host PC.
 */
//#define NO_SD_HOST_DRIVE   // Disable SD Card access over USB (for security).//#定义NO_SD_HOST_DRIVE//禁用通过USB访问SD卡（出于安全考虑）。

/**
 * Additional options for Graphical Displays
 *
 * Use the optimizations here to improve printing performance,
 * which can be adversely affected by graphical display drawing,
 * especially when doing several short moves, and when printing
 * on DELTA and SCARA machines.
 *
 * Some of these options may result in the display lagging behind
 * controller events, as there is a trade-off between reliable
 * printing performance versus fast display updates.
 */
#if HAS_MARLINUI_U8GLIB
// Save many cycles by drawing a hollow frame or no frame on the Info Screen//通过在信息屏幕上绘制空心框架或无框架来节省许多周期
  //#define XYZ_NO_FRAME//#定义XYZ_无_帧
  #define XYZ_HOLLOW_FRAME

  // A bigger font is available for edit items. Costs 3120 bytes of PROGMEM.//较大的字体可用于编辑项目。需要3120字节的程序。
  // Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.//只有西式的。不适用于西里尔语、假名、土耳其语、希腊语或汉语。
  //#define USE_BIG_EDIT_FONT//#定义使用大字体编辑字体

  // A smaller font may be used on the Info Screen. Costs 2434 bytes of PROGMEM.//信息屏幕上可以使用较小的字体。需要2434字节的程序。
  // Western only. Not available for Cyrillic, Kana, Turkish, Greek, or Chinese.//只有西式的。不适用于西里尔语、假名、土耳其语、希腊语或汉语。
  //#define USE_SMALL_INFOFONT//#定义使用\u小\u信息字体

  /**
   * ST7920-based LCDs can emulate a 16 x 4 character display using
   * the ST7920 character-generator for very fast screen updates.
   * Enable LIGHTWEIGHT_UI to use this special display mode.
   *
   * Since LIGHTWEIGHT_UI has limited space, the position and status
   * message occupy the same line. Set STATUS_EXPIRE_SECONDS to the
   * length of time to display the status message before clearing.
   *
   * Set STATUS_EXPIRE_SECONDS to zero to never clear the status.
   * This will prevent position updates from being displayed.
   */
  #if ENABLED(U8GLIB_ST7920)
    // Enable this option and reduce the value to optimize screen updates.//启用此选项并降低值以优化屏幕更新。
    // The normal delay is 10µs. Use the lowest value that still gives a reliable display.//正常延迟为10µs。使用仍能提供可靠显示的最低值。
    //#define DOGM_SPI_DELAY_US 5//#定义DOGM_SPI_DELAY_US 5

    //#define LIGHTWEIGHT_UI//#定义轻量级用户界面
    #if ENABLED(LIGHTWEIGHT_UI)
      #define STATUS_EXPIRE_SECONDS 20
    #endif
  #endif

  /**
   * Status (Info) Screen customizations
   * These options may affect code size and screen render time.
   * Custom status screens can forcibly override these settings.
   */
  //#define STATUS_COMBINE_HEATERS    // Use combined heater images instead of separate ones//#定义状态\组合\加热器//使用组合加热器图像而不是单独的图像
  //#define STATUS_HOTEND_NUMBERLESS  // Use plain hotend icons instead of numbered ones (with 2+ hotends)//#定义状态\热端\无编号//使用普通热端图标，而不是编号的图标（2+热端）
  #define STATUS_HOTEND_INVERTED      // Show solid nozzle bitmaps when heating (Requires STATUS_HOTEND_ANIM for numbered hotends)//加热时显示实心喷嘴位图（编号热端需要状态\u热端\u动画）
  #define STATUS_HOTEND_ANIM          // Use a second bitmap to indicate hotend heating//使用第二个位图指示热端加热
  #define STATUS_BED_ANIM             // Use a second bitmap to indicate bed heating//使用第二个位图指示床加热
  #define STATUS_CHAMBER_ANIM         // Use a second bitmap to indicate chamber heating//使用第二个位图指示腔室加热
  //#define STATUS_CUTTER_ANIM        // Use a second bitmap to indicate spindle / laser active//#定义状态\u刀具\u动画//使用第二个位图指示主轴/激光激活
  //#define STATUS_COOLER_ANIM        // Use a second bitmap to indicate laser cooling//#定义状态\u冷却器\u动画//使用第二个位图指示激光冷却
  //#define STATUS_FLOWMETER_ANIM     // Use multiple bitmaps to indicate coolant flow//#定义状态\流量计\动画//使用多个位图指示冷却液流量
  //#define STATUS_ALT_BED_BITMAP     // Use the alternative bed bitmap//#定义状态\可选\床\位图//使用备用床位图
  //#define STATUS_ALT_FAN_BITMAP     // Use the alternative fan bitmap//#定义状态\u ALT\u风扇\u位图//使用备用风扇位图
  //#define STATUS_FAN_FRAMES 3       // :[0,1,2,3,4] Number of fan animation frames//#定义状态_FAN_FRAMES 3/：[0,1,2,3,4]风扇动画帧数
  //#define STATUS_HEAT_PERCENT       // Show heating in a progress bar//#定义状态\u加热\u百分比//在进度条中显示加热
  //#define BOOT_MARLIN_LOGO_ANIMATED // Animated Marlin logo. Costs ~‭3260 (or ~940) bytes of PROGMEM.//#定义靴子\u马林鱼\u徽标\u动画//马林鱼动画徽标。费用~‭3260（或940）字节的程序。

  // Frivolous Game Options//无聊的游戏选择
  //#define MARLIN_BRICKOUT//#定义MARLIN_BRICKOUT
  //#define MARLIN_INVADERS//#定义马林鱼入侵者
  //#define MARLIN_SNAKE//#给马林鱼下定义
  //#define GAMES_EASTER_EGG          // Add extra blank lines above the "Games" sub-menu//#定义游戏\u复活节\u彩蛋//在“游戏”子菜单上方添加额外的空行

#endif // HAS_MARLINUI_U8GLIB//马林努伊能说会道吗

#if HAS_MARLINUI_U8GLIB || IS_DWIN_MARLINUI
// Show SD percentage next to the progress bar//在进度条旁边显示SD百分比
  //#define SHOW_SD_PERCENT//#定义显示百分比

  // Enable to save many cycles by drawing a hollow frame on Menu Screens//通过在菜单屏幕上绘制空心框架，可以节省许多周期
  #define MENU_HOLLOW_FRAME

  // Swap the CW/CCW indicators in the graphics overlay//交换图形叠加中的CW/CCW指示器
  //#define OVERLAY_GFX_REVERSE//#定义覆盖\u GFX\u反转
#endif

////
// Additional options for DGUS / DWIN displays//DGUS/DWN显示的其他选项
////
#if HAS_DGUS_LCD
#define LCD_SERIAL_PORT 3
  #define LCD_BAUDRATE 115200

  #define DGUS_RX_BUFFER_SIZE 128
  #define DGUS_TX_BUFFER_SIZE 48
  //#define SERIAL_STATS_RX_BUFFER_OVERRUNS  // Fix Rx overrun situation (Currently only for AVR)//#定义串行统计数据接收缓冲区溢出//修复接收溢出情况（目前仅适用于AVR）

  #define DGUS_UPDATE_INTERVAL_MS  500    // (ms) Interval between automatic screen updates//（ms）自动屏幕更新之间的间隔

  #if ANY(DGUS_LCD_UI_FYSETC, DGUS_LCD_UI_MKS, DGUS_LCD_UI_HIPRECY)
    #define DGUS_PRINT_FILENAME           // Display the filename during printing//打印时显示文件名
    #define DGUS_PREHEAT_UI               // Display a preheat screen during heatup//在加热过程中显示预热屏幕

    #if EITHER(DGUS_LCD_UI_FYSETC, DGUS_LCD_UI_MKS)
      //#define DGUS_UI_MOVE_DIS_OPTION   // Disabled by default for FYSETC and MKS//#定义DGUS_UI_MOVE_DIS_选项//FYSETC和MKS默认禁用
    #else
      #define DGUS_UI_MOVE_DIS_OPTION     // Enabled by default for UI_HIPRECY//默认情况下为UI\u HIPRECY启用
    #endif

    #define DGUS_FILAMENT_LOADUNLOAD
    #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
      #define DGUS_FILAMENT_PURGE_LENGTH 10
      #define DGUS_FILAMENT_LOAD_LENGTH_PER_TIME 0.5 // (mm) Adjust in proportion to DGUS_UPDATE_INTERVAL_MS//（mm）根据DGUS_更新_间隔_MS按比例调整
    #endif

    #define DGUS_UI_WAITING               // Show a "waiting" screen between some screens//在一些屏幕之间显示“等待”屏幕
    #if ENABLED(DGUS_UI_WAITING)
      #define DGUS_UI_WAITING_STATUS 10
      #define DGUS_UI_WAITING_STATUS_PERIOD 8 // Increase to slower waiting status looping//增加到较慢的等待状态循环
    #endif
  #endif
#endif // HAS_DGUS_LCD//有液晶显示器吗

////
// Additional options for AnyCubic Chiron TFT displays//任何立方型Chiron TFT显示器的附加选项
////
#if ENABLED(ANYCUBIC_LCD_CHIRON)
// By default the type of panel is automatically detected.//默认情况下，将自动检测面板的类型。
  // Enable one of these options if you know the panel type.//如果您知道配电盘类型，请启用其中一个选项。
  //#define CHIRON_TFT_STANDARD//#定义CHIRON_TFT_标准
  //#define CHIRON_TFT_NEW//#定义CHIRON_TFT_新

  // Enable the longer Anycubic powerup startup tune//启用更长的Anycubic加电启动调谐
  //#define AC_DEFAULT_STARTUP_TUNE//#定义AC\u默认值\u启动\u调整

  /**
   * Display Folders
   * By default the file browser lists all G-code files (including those in subfolders) in a flat list.
   * Enable this option to display a hierarchical file browser.
   *
   * NOTES:
   * - Without this option it helps to enable SDCARD_SORT_ALPHA so files are sorted before/after folders.
   * - When used with the "new" panel, folder names will also have '.gcode' appended to their names.
   *   This hack is currently required to force the panel to show folders.
   */
  #define AC_SD_FOLDER_VIEW
#endif

////
// Specify additional languages for the UI. Default specified by LCD_LANGUAGE.//为UI指定其他语言。由LCD\u语言指定的默认值。
////
#if ANY(DOGLCD, TFT_COLOR_UI, TOUCH_UI_FTDI_EVE, IS_DWIN_MARLINUI)
//#define LCD_LANGUAGE_2 fr//#定义LCD_语言_2 fr
  //#define LCD_LANGUAGE_3 de//#定义LCD_语言_3 de
  //#define LCD_LANGUAGE_4 es//#定义LCD_语言_4 es
  //#define LCD_LANGUAGE_5 it//#定义LCD_语言_5 it
  #ifdef LCD_LANGUAGE_2
    //#define LCD_LANGUAGE_AUTO_SAVE // Automatically save language to EEPROM on change//#定义LCD_语言_自动_保存//更改时自动将语言保存到EEPROM
  #endif
#endif

////
// Touch UI for the FTDI Embedded Video Engine (EVE)//FTDI嵌入式视频引擎（EVE）的触摸式用户界面
////
#if ENABLED(TOUCH_UI_FTDI_EVE)
// Display board used//使用的显示板
  //#define LCD_FTDI_VM800B35A        // FTDI 3.5" with FT800 (320x240)//#使用FT800（320x240）定义LCD_FTDI_VM800B35A//FTDI 3.5英寸
  //#define LCD_4DSYSTEMS_4DLCD_FT843 // 4D Systems 4.3" (480x272)//#定义LCD_4DSYSTEMS_4DLCD_FT843//4D Systems 4.3”（480x272）
  //#define LCD_HAOYU_FT800CB         // Haoyu with 4.3" or 5" (480x272)//#将LCD_HAOYU_FT800CB//HAOYU定义为4.3英寸或5英寸（480x272）
  //#define LCD_HAOYU_FT810CB         // Haoyu with 5" (800x480)//#用5英寸（800x480）定义LCD_HAOYU_FT810CB//HAOYU
  //#define LCD_LULZBOT_CLCD_UI       // LulzBot Color LCD UI//#定义LCD_LULZBOT_CLCD_UI//LULZBOT彩色LCD UI
  //#define LCD_FYSETC_TFT81050       // FYSETC with 5" (800x480)//#定义LCD_FYSETC_TFT81050//FYSETC和5英寸（800x480）
  //#define LCD_EVE3_50G              // Matrix Orbital 5.0", 800x480, BT815//#定义LCD_EVE3_50G//矩阵轨道5.0“，800x480，BT815
  //#define LCD_EVE2_50G              // Matrix Orbital 5.0", 800x480, FT813//#定义LCD_EVE2_50G//矩阵轨道5.0“，800x480，FT813

  // Correct the resolution if not using the stock TFT panel.//如果不使用库存TFT面板，请更正分辨率。
  //#define TOUCH_UI_320x240//#定义触摸屏界面320x240
  //#define TOUCH_UI_480x272//#定义触摸屏界面480x272
  //#define TOUCH_UI_800x480//#定义触摸屏界面800x480

  // Mappings for boards with a standard RepRapDiscount Display connector//带有标准RepraDiscount显示连接器的板的映射
  //#define AO_EXP1_PINMAP      // LulzBot CLCD UI EXP1 mapping//#定义AO_EXP1_PINMAP//LulzBot CLCD UI EXP1映射
  //#define AO_EXP2_PINMAP      // LulzBot CLCD UI EXP2 mapping//#定义AO_EXP2_PINMAP//LulzBot CLCD UI EXP2映射
  //#define CR10_TFT_PINMAP     // Rudolph Riedel's CR10 pin mapping//#定义CR10_TFT_引脚映射//鲁道夫·里德尔的CR10引脚映射
  //#define S6_TFT_PINMAP       // FYSETC S6 pin mapping//#定义S6_TFT_引脚映射//FYSETC S6引脚映射
  //#define F6_TFT_PINMAP       // FYSETC F6 pin mapping//#定义F6_TFT_引脚映射//FYSETC F6引脚映射

  //#define OTHER_PIN_LAYOUT  // Define pins manually below//#定义其他引脚布局//在下面手动定义引脚
  #if ENABLED(OTHER_PIN_LAYOUT)
    // Pins for CS and MOD_RESET (PD) must be chosen//必须选择CS和MOD_重置（PD）的引脚
    #define CLCD_MOD_RESET  9
    #define CLCD_SPI_CS    10

    // If using software SPI, specify pins for SCLK, MOSI, MISO//如果使用软件SPI，请指定SCLK、MOSI、MISO的引脚
    //#define CLCD_USE_SOFT_SPI//#定义CLCD\u使用\u软\u SPI
    #if ENABLED(CLCD_USE_SOFT_SPI)
      #define CLCD_SOFT_SPI_MOSI 11
      #define CLCD_SOFT_SPI_MISO 12
      #define CLCD_SOFT_SPI_SCLK 13
    #endif
  #endif

  // Display Orientation. An inverted (i.e. upside-down) display//显示方向。倒置（即倒置）显示器
  // is supported on the FT800. The FT810 and beyond also support//在FT800上受支持。FT810及更高版本还支持
  // portrait and mirrored orientations.//纵向和镜像方向。
  //#define TOUCH_UI_INVERTED//#定义触摸屏
  //#define TOUCH_UI_PORTRAIT//#定义触摸图
  //#define TOUCH_UI_MIRRORED//#定义触摸屏

  // UTF8 processing and rendering.//UTF8处理和渲染。
  // Unsupported characters are shown as '?'.//不支持的字符显示为“？”。
  //#define TOUCH_UI_USE_UTF8//#定义触摸屏\u用户界面\u使用\u UTF8
  #if ENABLED(TOUCH_UI_USE_UTF8)
    // Western accents support. These accented characters use//西方口音支持。这些重音字符使用
    // combined bitmaps and require relatively little storage.//组合位图并需要相对较少的存储空间。
    #define TOUCH_UI_UTF8_WESTERN_CHARSET
    #if ENABLED(TOUCH_UI_UTF8_WESTERN_CHARSET)
      // Additional character groups. These characters require//其他字符组。这些字符需要
      // full bitmaps and take up considerable storage://完整位图并占用大量存储空间：
      //#define TOUCH_UI_UTF8_SUPERSCRIPTS  // ¹ ² ³//#定义触摸式用户界面UTF8\U上标//imk_²³
      //#define TOUCH_UI_UTF8_COPYRIGHT     // © ®//#定义TOUCH\u UI\u UTF8\u版权//©®
      //#define TOUCH_UI_UTF8_GERMANIC      // ß//#定义触摸屏
      //#define TOUCH_UI_UTF8_SCANDINAVIAN  // Æ Ð Ø Þ æ ð ø þ//#定义触摸屏
      //#define TOUCH_UI_UTF8_PUNCTUATION   // « » ¿ ¡//#定义触摸屏\u UI\u UTF8\u标点符号//«»»
      //#define TOUCH_UI_UTF8_CURRENCY      // ¢ £ ¤ ¥//#定义触摸屏（UTF8）货币//，%
      //#define TOUCH_UI_UTF8_ORDINALS      // º ª//#定义触摸式界面UTF8顺序//ºª
      //#define TOUCH_UI_UTF8_MATHEMATICS   // ± × ÷//#定义触摸界面UTF8\U数学//±×÷
      //#define TOUCH_UI_UTF8_FRACTIONS     // ¼ ½ ¾//#定义触摸式界面UTF8分数//¼½¾
      //#define TOUCH_UI_UTF8_SYMBOLS       // µ ¶ ¦ § ¬//#定义触摸式用户界面UTF8符号//µ¨§¨
    #endif

    // Cyrillic character set, costs about 27KiB of flash//西里尔文字字符集，耗资约27KiB闪存
    //#define TOUCH_UI_UTF8_CYRILLIC_CHARSET//#定义TOUCH\u UI\u UTF8\u西里尔字母\u字符集
  #endif

  // Use a smaller font when labels don't fit buttons//当标签与按钮不匹配时，请使用较小的字体
  #define TOUCH_UI_FIT_TEXT

  // Use a numeric passcode for "Screen lock" keypad.//为“屏幕锁定”键盘使用数字密码。
  // (recommended for smaller displays)//（建议用于较小的显示器）
  //#define TOUCH_UI_PASSCODE//#定义触摸式用户界面密码

  // Output extra debug info for Touch UI events//输出触摸屏UI事件的额外调试信息
  //#define TOUCH_UI_DEBUG//#定义触摸界面调试

  // Developer menu (accessed by touching "About Printer" copyright text)//开发者菜单（通过触摸“关于打印机”版权文本访问）
  //#define TOUCH_UI_DEVELOPER_MENU//#定义触摸屏界面开发者菜单
#endif

////
// Classic UI Options//经典用户界面选项
////
#if TFT_SCALED_DOGLCD
//#define TFT_MARLINUI_COLOR 0xFFFF // White//#定义TFT\u MARLINUI\u颜色0xFFFF//白色
  //#define TFT_MARLINBG_COLOR 0x0000 // Black//#定义TFT_MARLINBG_颜色0x0000//黑色
  //#define TFT_DISABLED_COLOR 0x0003 // Almost black//#定义TFT_禁用_颜色0x0003//几乎为黑色
  //#define TFT_BTCANCEL_COLOR 0xF800 // Red//#定义TFT\u BTCANCEL\u颜色0xF800//红色
  //#define TFT_BTARROWS_COLOR 0xDEE6 // 11011 110111 00110 Yellow//#定义TFT箭头颜色0xDEE6//11011 110111 00110黄色
  //#define TFT_BTOKMENU_COLOR 0x145F // 00010 100010 11111 Cyan//#定义TFT_BTOKMENU_颜色0x145F//00010 100010 11111青色
#endif

////
// ADC Button Debounce//ADC按钮去抖动
////
#if HAS_ADC_BUTTONS
#define ADC_BUTTON_DEBOUNCE_DELAY 16  // Increase if buttons bounce or repeat too fast//如果按钮反弹或重复过快，则增加
#endif

// @section safety//@路段安全

/**
 * The watchdog hardware timer will do a reset and disable all outputs
 * if the firmware gets too overloaded to read the temperature sensors.
 *
 * If you find that watchdog reboot causes your AVR board to hang forever,
 * enable WATCHDOG_RESET_MANUAL to use a custom timer instead of WDTO.
 * NOTE: This method is less reliable as it can only catch hangups while
 * interrupts are enabled.
 */
#define USE_WATCHDOG
#if ENABLED(USE_WATCHDOG)
//#define WATCHDOG_RESET_MANUAL//#定义看门狗重置手册
#endif

// @section lcd//@section液晶显示器

/**
 * Babystepping enables movement of the axes by tiny increments without changing
 * the current position values. This feature is used primarily to adjust the Z
 * axis in the first layer of a print in real-time.
 *
 * Warning: Does not respect endstops!
 */
//#define BABYSTEPPING//#定义BABYSTEPPING
#if ENABLED(BABYSTEPPING)
//#define INTEGRATED_BABYSTEPPING         // EXPERIMENTAL integration of babystepping into the Stepper ISR//#定义集成_BABYSTEPPING//将BABYSTEPPING实验集成到步进机ISR中
  //#define BABYSTEP_WITHOUT_HOMING//#定义BABYSTEP_而不使用_归位
  //#define BABYSTEP_ALWAYS_AVAILABLE       // Allow babystepping at all times (not just during movement).//#定义BABYSTEP_ALWAYS_AVAILABLE//始终允许BABYSTEP（不仅仅在移动过程中）。
  //#define BABYSTEP_XY                     // Also enable X/Y Babystepping. Not supported on DELTA!//#定义BABYSTEP_XY//同时启用X/Y Babystepping。DELTA上不支持！
  #define BABYSTEP_INVERT_Z false           // Change if Z babysteps should go the other way//如果Z babysteps应该走另一条路，则进行更改
  //#define BABYSTEP_MILLIMETER_UNITS       // Specify BABYSTEP_MULTIPLICATOR_(XY|Z) in mm instead of micro-steps//#定义BABYSTEP_mm_单位//指定BABYSTEP_乘法器（XY | Z），单位为mm，而不是微步
  #define BABYSTEP_MULTIPLICATOR_Z  1       // (steps or mm) Steps or millimeter distance for each Z babystep//（步数或毫米）每个Z babystep的步数或毫米距离
  #define BABYSTEP_MULTIPLICATOR_XY 1       // (steps or mm) Steps or millimeter distance for each XY babystep//每个XY babystep的（步数或毫米）步数或毫米距离

  //#define DOUBLECLICK_FOR_Z_BABYSTEPPING  // Double-click on the Status Screen for Z Babystepping.//#定义双击以进行双击//双击状态屏幕以进行双击。
  #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING)
    #define DOUBLECLICK_MAX_INTERVAL 1250   // Maximum interval between clicks, in milliseconds.//单击之间的最大间隔，以毫秒为单位。
                                            // Note: Extra time may be added to mitigate controller latency.//注意：可能会增加额外的时间以减轻控制器延迟。
    //#define MOVE_Z_WHEN_IDLE              // Jump to the move Z menu on doubleclick when printer is idle.//#当打印机空闲时，定义MOVE_Z_//在双击时跳转到MOVE Z菜单。
    #if ENABLED(MOVE_Z_WHEN_IDLE)
      #define MOVE_Z_IDLE_MULTIPLICATOR 1   // Multiply 1mm by this factor for the move step size.//将1mm乘以该系数，即为移动步长。
    #endif
  #endif

  //#define BABYSTEP_DISPLAY_TOTAL          // Display total babysteps since last G28//#定义BABYSTEP\u DISPLAY\u TOTAL//DISPLAY TOTAL babysteps（自上次G28峰会以来）

  //#define BABYSTEP_ZPROBE_OFFSET          // Combine M851 Z and Babystepping//#定义BABYSTEP_ZPROBE_偏移//组合M851 Z和Babystepping
  #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
    //#define BABYSTEP_HOTEND_Z_OFFSET      // For multiple hotends, babystep relative Z offsets//#定义BABYSTEP_HOTEND_Z_偏移//对于多个hotends，BABYSTEP相对Z偏移
    //#define BABYSTEP_ZPROBE_GFX_OVERLAY   // Enable graphical overlay on Z-offset editor//#定义BABYSTEP_ZPROBE_GFX_覆盖//在Z偏移编辑器上启用图形覆盖
  #endif
#endif

// @section extruder//型材挤出机

/**
 * Linear Pressure Control v1.5
 *
 * Assumption: advance [steps] = k * (delta velocity [steps/s])
 * K=0 means advance disabled.
 *
 * NOTE: K values for LIN_ADVANCE 1.5 differ from earlier versions!
 *
 * Set K around 0.22 for 3mm PLA Direct Drive with ~6.5cm between the drive gear and heatbreak.
 * Larger K values will be needed for flexible filament and greater distances.
 * If this algorithm produces a higher speed offset than the extruder can handle (compared to E jerk)
 * print acceleration will be reduced during the affected moves to keep within the limit.
 *
 * See https://marlinfw.org/docs/features/lin_advance.html for full instructions.
 */
//#define LIN_ADVANCE//#定义LINU前进
#if ENABLED(LIN_ADVANCE)
//#define EXTRA_LIN_ADVANCE_K // Enable for second linear advance constants//#为第二个线性进阶常数定义额外的进阶
  #define LIN_ADVANCE_K 0.22    // Unit: mm compression per 1mm/s extruder speed//单位：每1mm/s挤出机速度压缩mm
  //#define LA_DEBUG            // If enabled, this will generate debug information output over USB.//#定义LA_DEBUG//如果启用，将通过USB生成调试信息输出。
  //#define EXPERIMENTAL_SCURVE // Enable this option to permit S-Curve Acceleration//#定义试验曲线//启用此选项以允许S曲线加速
#endif

// @section leveling//@剖面调平

/**
 * Points to probe for all 3-point Leveling procedures.
 * Override if the automatically selected points are inadequate.
 */
#if EITHER(AUTO_BED_LEVELING_3POINT, AUTO_BED_LEVELING_UBL)
//#define PROBE_PT_1_X 15//#定义探测点1×15
  //#define PROBE_PT_1_Y 180//#定义探测点1_Y 180
  //#define PROBE_PT_2_X 15//#定义探测点2×15
  //#define PROBE_PT_2_Y 20//#定义探测点2年20
  //#define PROBE_PT_3_X 170//#定义探头位置3×170
  //#define PROBE_PT_3_Y 20//#定义探测点3年20
#endif

/**
 * Probing Margins
 *
 * Override PROBING_MARGIN for each side of the build plate
 * Useful to get probe points to exact positions on targets or
 * to allow leveling to avoid plate clamps on only specific
 * sides of the bed. With NOZZLE_AS_PROBE negative values are
 * allowed, to permit probing outside the bed.
 *
 * If you are replacing the prior *_PROBE_BED_POSITION options,
 * LEFT and FRONT values in most cases will map directly over
 * RIGHT and REAR would be the inverse such as
 * (X/Y_BED_SIZE - RIGHT/BACK_PROBE_BED_POSITION)
 *
 * This will allow all positions to match at compilation, however
 * should the probe position be modified with M851XY then the
 * probe points will follow. This prevents any change from causing
 * the probe to be unable to reach any points.
 */
#if PROBE_SELECTED && !IS_KINEMATIC
//#define PROBING_MARGIN_LEFT PROBING_MARGIN//#定义探测\u边距\u左探测\u边距
  //#define PROBING_MARGIN_RIGHT PROBING_MARGIN//#定义探测\u边距\u右探测\u边距
  //#define PROBING_MARGIN_FRONT PROBING_MARGIN//#定义探测\u边距\u前探测\u边距
  //#define PROBING_MARGIN_BACK PROBING_MARGIN//#定义探测\u边距\u后探测\u边距
#endif

#if EITHER(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)
// Override the mesh area if the automatic (max) area is too large//如果“自动”（最大）区域太大，请替代网格区域
  //#define MESH_MIN_X MESH_INSET//#定义网格\u最小\u X网格\u插入
  //#define MESH_MIN_Y MESH_INSET//#定义网格\u最小\u Y网格\u插入
  //#define MESH_MAX_X X_BED_SIZE - (MESH_INSET)//#定义网格\最大\ X \床\尺寸-（网格\插图）
  //#define MESH_MAX_Y Y_BED_SIZE - (MESH_INSET)//#定义网格大小（网格插图）
#endif

#if BOTH(AUTO_BED_LEVELING_UBL, EEPROM_SETTINGS)
//#define OPTIMIZED_MESH_STORAGE  // Store mesh with less precision to save EEPROM space//#定义优化的网格存储//以较低的精度存储网格以节省EEPROM空间
#endif

/**
 * Repeatedly attempt G29 leveling until it succeeds.
 * Stop after G29_MAX_RETRIES attempts.
 */
//#define G29_RETRY_AND_RECOVER//#定义G29\u重试\u和\u恢复
#if ENABLED(G29_RETRY_AND_RECOVER)
#define G29_MAX_RETRIES 3
  #define G29_HALT_ON_FAILURE
  /**
   * Specify the GCODE commands that will be executed when leveling succeeds,
   * between attempts, and after the maximum number of retries have been tried.
   */
  #define G29_SUCCESS_COMMANDS "M117 Bed leveling done."
  #define G29_RECOVER_COMMANDS "M117 Probe failed. Rewiping.\nG28\nG12 P0 S12 T0"
  #define G29_FAILURE_COMMANDS "M117 Bed leveling failed.\nG0 Z10\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nG4 S1"

#endif

/**
 * Thermal Probe Compensation
 * Probe measurements are adjusted to compensate for temperature distortion.
 * Use G76 to calibrate this feature. Use M871 to set values manually.
 * For a more detailed explanation of the process see G76_M871.cpp.
 */
#if HAS_BED_PROBE && TEMP_SENSOR_PROBE && TEMP_SENSOR_BED
// Enable thermal first layer compensation using bed and probe temperatures//使用床层和探头温度启用第一层热补偿
  #define PROBE_TEMP_COMPENSATION

  // Add additional compensation depending on hotend temperature//根据热端温度添加额外补偿
  // Note: this values cannot be calibrated and have to be set manually//注：该值无法校准，必须手动设置
  #if ENABLED(PROBE_TEMP_COMPENSATION)
    // Park position to wait for probe cooldown//停驻位置等待探头冷却
    #define PTC_PARK_POS   { 0, 0, 100 }

    // Probe position to probe and wait for probe to reach target temperature//探头位置至探头，并等待探头达到目标温度
    #define PTC_PROBE_POS  { 90, 100 }

    // Enable additional compensation using hotend temperature//使用热端温度启用附加补偿
    // Note: this values cannot be calibrated automatically but have to be set manually//注：该值不能自动校准，但必须手动设置
    //#define USE_TEMP_EXT_COMPENSATION//#定义使用临时外部补偿

    // Probe temperature calibration generates a table of values starting at PTC_SAMPLE_START//探头温度校准从PTC_样品_开始生成一个值表
    // (e.g., 30), in steps of PTC_SAMPLE_RES (e.g., 5) with PTC_SAMPLE_COUNT (e.g., 10) samples.//（例如，30），在PTC_样品（例如，5）与PTC_样品计数（例如，10）样品的步骤中。

    //#define PTC_SAMPLE_START  30  // (°C)//#定义PTC样品起始温度30/（°C）
    //#define PTC_SAMPLE_RES     5  // (°C)//#定义PTC样品5/（°C）
    //#define PTC_SAMPLE_COUNT  10//#定义PTC样本计数10

    // Bed temperature calibration builds a similar table.//床温校准建立了一个类似的表。

    //#define BTC_SAMPLE_START  60  // (°C)//#定义BTC样品开始温度为60/（°C）
    //#define BTC_SAMPLE_RES     5  // (°C)//#定义BTC_样品5/（°C）
    //#define BTC_SAMPLE_COUNT  10//#定义BTC\u样本\u计数10

    // The temperature the probe should be at while taking measurements during bed temperature//在床温期间进行测量时，探头应处于的温度
    // calibration.//校准。
    //#define BTC_PROBE_TEMP    30  // (°C)//#定义BTC探头温度30/（°C）

    // Height above Z=0.0 to raise the nozzle. Lowering this can help the probe to heat faster.//高于Z的高度=0.0以升高喷嘴。降低此值有助于探头更快加热。
    // Note: the Z=0.0 offset is determined by the probe offset which can be set using M851.//注：Z=0.0偏移量由探头偏移量确定，可使用M851进行设置。
    //#define PTC_PROBE_HEATING_OFFSET 0.5//#定义PTC探头加热偏移量0.5

    // Height to raise the Z-probe between heating and taking the next measurement. Some probes//在加热和进行下一次测量之间升高Z型探头的高度。一些探针
    // may fail to untrigger if they have been triggered for a long time, which can be solved by//如果长时间触发，可能无法解除迁移，这可以通过以下方法解决：
    // increasing the height the probe is raised to.//增加探头升高到的高度。
    //#define PTC_PROBE_RAISE 15//#定义PTC探头上升15

    // If the probe is outside of the defined range, use linear extrapolation using the closest//如果探头超出定义范围，则使用最接近的
    // point and the PTC_LINEAR_EXTRAPOLATION'th next point. E.g. if set to 4 it will use data[0]//点和PTC线性外推的下一个点。例如，如果设置为4，则将使用数据[0]
    // and data[4] to perform linear extrapolation for values below PTC_SAMPLE_START.//和数据[4]，对低于PTC_SAMPLE_START的值进行线性外推。
    //#define PTC_LINEAR_EXTRAPOLATION 4//#定义PTC_线性_外推4
  #endif
#endif

// @section extras//@额外部分

////
// G60/G61 Position Save and Return//G60/G61位置保存和返回
////
//#define SAVED_POSITIONS 1         // Each saved position slot costs 12 bytes//#定义保存的位置1//每个保存的位置插槽花费12字节

////
// G2/G3 Arc Support//G2/G3电弧支架
////
#define ARC_SUPPORT                 // Disable this feature to save ~3226 bytes//禁用此功能可保存约3226个字节
#if ENABLED(ARC_SUPPORT)
  #define MM_PER_ARC_SEGMENT      1 // (mm) Length (or minimum length) of each arc segment//（mm）每个弧段的长度（或最小长度）
  //#define ARC_SEGMENTS_PER_R    1 // Max segment length, MM_PER = Min//#定义弧段/每段长度1//Max段长度，MM每段=最小
  #define MIN_ARC_SEGMENTS       24 // Minimum number of segments in a complete circle//完整圆中的最小分段数
  //#define ARC_SEGMENTS_PER_SEC 50 // Use feedrate to choose segment length (with MM_PER_ARC_SEGMENT as the minimum)//#定义弧段/弧段/秒50//使用进给速度选择段长度（最小为每弧段MM）
  #define N_ARC_CORRECTION       25 // Number of interpolated segments between corrections//校正之间的插值段数
  //#define ARC_P_CIRCLES           // Enable the 'P' parameter to specify complete circles//#定义圆弧P_圆//启用“P”参数以指定完整圆
  //#define CNC_WORKSPACE_PLANES    // Allow G2/G3 to operate in XY, ZX, or YZ planes//#定义CNC_工作空间_平面//允许G2/G3在XY、ZX或YZ平面中操作
  //#define SF_ARC_FIX              // Enable only if using SkeinForge with "Arc Point" fillet procedure//#定义SF_ARC_FIX//仅在将SKEInforme与“弧点”圆角过程一起使用时启用
#endif

// G5 Bézier Curve Support with XYZE destination and IJPQ offsets//带有XYZE目标和IJPQ偏移的G5贝塞尔曲线支持
//#define BEZIER_CURVE_SUPPORT        // Requires ~2666 bytes//#定义BEZIER_曲线_支持//需要约2666字节

#if EITHER(ARC_SUPPORT, BEZIER_CURVE_SUPPORT)
//#define CNC_WORKSPACE_PLANES      // Allow G2/G3/G5 to operate in XY, ZX, or YZ planes//#定义CNC_工作空间_平面//允许G2/G3/G5在XY、ZX或YZ平面中操作
#endif

/**
 * Direct Stepping
 *
 * Comparable to the method used by Klipper, G6 direct stepping significantly
 * reduces motion calculations, increases top printing speeds, and results in
 * less step aliasing by calculating all motions in advance.
 * Preparing your G-code: https://github.com/colinrgodsey/step-daemon
 */
//#define DIRECT_STEPPING//#定义直接步进

/**
 * G38 Probe Target
 *
 * This option adds G38.2 and G38.3 (probe towards target)
 * and optionally G38.4 and G38.5 (probe away from target).
 * Set MULTIPLE_PROBING for G38 to probe more than once.
 */
//#define G38_PROBE_TARGET//#定义G38_探头_目标
#if ENABLED(G38_PROBE_TARGET)
//#define G38_PROBE_AWAY        // Include G38.4 and G38.5 to probe away from target//#定义G38_探头_远离//包括G38.4和G38.5探头远离目标
  #define G38_MINIMUM_MOVE 0.0275 // (mm) Minimum distance that will produce a move.//（mm）将产生移动的最小距离。
#endif

// Moves (or segments) with fewer steps than this will be joined with the next move//步数少于此步数的移动（或分段）将与下一个移动合并
#define MIN_STEPS_PER_SEGMENT 6

/**
 * Minimum delay before and after setting the stepper DIR (in ns)
 *     0 : No delay (Expect at least 10µS since one Stepper ISR must transpire)
 *    20 : Minimum for TMC2xxx drivers
 *   200 : Minimum for A4988 drivers
 *   400 : Minimum for A5984 drivers
 *   500 : Minimum for LV8729 drivers (guess, no info in datasheet)
 *   650 : Minimum for DRV8825 drivers
 *  1500 : Minimum for TB6600 drivers (guess, no info in datasheet)
 * 15000 : Minimum for TB6560 drivers (guess, no info in datasheet)
 *
 * Override the default value based on the driver type set in Configuration.h.
 */
//#define MINIMUM_STEPPER_POST_DIR_DELAY 650//#定义最小步进器后方向延迟650
//#define MINIMUM_STEPPER_PRE_DIR_DELAY 650//#定义最小步进器预定向延迟650

/**
 * Minimum stepper driver pulse width (in µs)
 *   0 : Smallest possible width the MCU can produce, compatible with TMC2xxx drivers
 *   0 : Minimum 500ns for LV8729, adjusted in stepper.h
 *   1 : Minimum for A4988 and A5984 stepper drivers
 *   2 : Minimum for DRV8825 stepper drivers
 *   3 : Minimum for TB6600 stepper drivers
 *  30 : Minimum for TB6560 stepper drivers
 *
 * Override the default value based on the driver type set in Configuration.h.
 */
//#define MINIMUM_STEPPER_PULSE 2//#定义最小步进器脉冲2

/**
 * Maximum stepping rate (in Hz) the stepper driver allows
 *  If undefined, defaults to 1MHz / (2 * MINIMUM_STEPPER_PULSE)
 *  5000000 : Maximum for TMC2xxx stepper drivers
 *  1000000 : Maximum for LV8729 stepper driver
 *  500000  : Maximum for A4988 stepper driver
 *  250000  : Maximum for DRV8825 stepper driver
 *  150000  : Maximum for TB6600 stepper driver
 *   15000  : Maximum for TB6560 stepper driver
 *
 * Override the default value based on the driver type set in Configuration.h.
 */
//#define MAXIMUM_STEPPER_RATE 250000//#定义最大步进电机速率250000

// @section temperature//@截面温度

// Control heater 0 and heater 1 in parallel.//并联控制加热器0和加热器1。
//#define HEATERS_PARALLEL//#定义加热器，使其平行

//===========================================================================//===========================================================================
//================================= Buffers =================================//================================================缓冲区=================================
//===========================================================================//===========================================================================

// @section motion//@节动议

// The number of linear moves that can be in the planner at once.//一次可以在计划器中的线性移动数。
// The value of BLOCK_BUFFER_SIZE must be a power of 2 (e.g., 8, 16, 32)//块缓冲区大小的值必须是2的幂（例如，8、16、32）
#if BOTH(SDSUPPORT, DIRECT_STEPPING)
#define BLOCK_BUFFER_SIZE  8
#elif ENABLED(SDSUPPORT)
#define BLOCK_BUFFER_SIZE 16
#else
#define BLOCK_BUFFER_SIZE 16
#endif

// @section serial//@节序列号

// The ASCII buffer for serial input//用于串行输入的ASCII缓冲区
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

// Transmission to Host Buffer Size//传输到主机缓冲区大小
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.//要保存386字节的PROGMEM（以及TX\U BUFFER\U SIZE+3字节的RAM），请将设置为0。
// To buffer a simple "ok" you need 4 bytes.//要缓冲一个简单的“ok”，您需要4个字节。
// For ADVANCED_OK (M105) you need 32 bytes.//对于ADVANCED_OK（M105），您需要32个字节。
// For debug-echo: 128 bytes for the optimal speed.//对于调试回显：128字节为最佳速度。
// Other output doesn't need to be that speedy.//其他输出不需要那么快。
// :[0, 2, 4, 8, 16, 32, 64, 128, 256]// :[0, 2, 4, 8, 16, 32, 64, 128, 256]
#define TX_BUFFER_SIZE 32

// Host Receive Buffer Size//主机接收缓冲区大小
// Without XON/XOFF flow control (see SERIAL_XON_XOFF below) 32 bytes should be enough.//如果没有XON/XOFF流控制（参见下面的串行XON\XOFF），32字节应该足够了。
// To use flow control, set this buffer size to at least 1024 bytes.//要使用流控制，请将此缓冲区大小设置为至少1024字节。
// :[0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]// :[0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048]
//#define RX_BUFFER_SIZE 1024//#定义接收缓冲区大小1024

#if RX_BUFFER_SIZE >= 1024
// Enable to have the controller send XON/XOFF control characters to//允许控制器将XON/XOFF控制字符发送到
  // the host to signal the RX buffer is becoming full.//发送接收缓冲器信号的主机已满。
  //#define SERIAL_XON_XOFF//#定义串行XON XOFF
#endif

#if ENABLED(SDSUPPORT)
// Enable this option to collect and display the maximum//启用此选项以收集和显示最大值
  // RX queue usage after transferring a file to SD.//将文件传输到SD后的RX队列使用情况。
  //#define SERIAL_STATS_MAX_RX_QUEUED//#定义序列统计数据最大接收队列

  // Enable this option to collect and display the number//启用此选项以收集和显示号码
  // of dropped bytes after a file transfer to SD.//文件传输到SD后丢弃的字节数。
  //#define SERIAL_STATS_DROPPED_RX//#定义序列统计数据丢弃接收
#endif

// Monitor RX buffer usage//监视接收缓冲区使用情况
// Dump an error to the serial port if the serial receive buffer overflows.//如果串行接收缓冲区溢出，则将错误转储到串行端口。
// If you see these errors, increase the RX_BUFFER_SIZE value.//如果您看到这些错误，请增加RX\u BUFFER\u SIZE值。
// Not supported on all platforms.//并非所有平台都支持。
//#define RX_BUFFER_MONITOR//#定义接收缓冲区监视器

/**
 * Emergency Command Parser
 *
 * Add a low-level parser to intercept certain commands as they
 * enter the serial receive buffer, so they cannot be blocked.
 * Currently handles M108, M112, M410, M876
 * NOTE: Not yet implemented for all platforms.
 */
//#define EMERGENCY_PARSER//#定义紧急语法分析器

/**
 * Realtime Reporting (requires EMERGENCY_PARSER)
 *
 * - Report position and state of the machine (like Grbl).
 * - Auto-report position during long moves.
 * - Useful for CNC/LASER.
 *
 * Adds support for commands:
 *  S000 : Report State and Position while moving.
 *  P000 : Instant Pause / Hold while moving.
 *  R000 : Resume from Pause / Hold.
 *
 * - During Hold all Emergency Parser commands are available, as usual.
 * - Enable NANODLP_Z_SYNC and NANODLP_ALL_AXIS for move command end-state reports.
 */
//#define REALTIME_REPORTING_COMMANDS
#if ENABLED(REALTIME_REPORTING_COMMANDS)
#define FULL_REPORT_TO_HOST_FEATURE   // Auto-report the machine status like Grbl CNC//自动报告机器状态，如Grbl CNC
#endif

// Bad Serial-connections can miss a received command by sending an 'ok'//错误的串行连接可能会通过发送“ok”而错过接收到的命令
// Therefore some clients abort after 30 seconds in a timeout.//因此，一些客户端在超时30秒后中止。
// Some other clients start sending commands while receiving a 'wait'.//其他一些客户端在接收“等待”时开始发送命令。
// This "wait" is only sent when the buffer is empty. 1 second is a good value here.//此“等待”仅在缓冲区为空时发送。1秒在这里是一个很好的值。
//#define NO_TIMEOUTS 1000 // Milliseconds//#定义无超时1000//毫秒

// Some clients will have this feature soon. This could make the NO_TIMEOUTS unnecessary.//一些客户很快就会有这个功能。这可能会使禁止超时变得不必要。
#define ADVANCED_OK

// Printrun may have trouble receiving long strings all at once.//Printrun可能无法同时接收长字符串。
// This option inserts short delays between lines of serial output.//此选项在串行输出线之间插入短延迟。
#define SERIAL_OVERRUN_PROTECTION

// For serial echo, the number of digits after the decimal point//对于串行回显，小数点后的位数
//#define SERIAL_FLOAT_PRECISION 4//#定义串行浮点精度4

// @section extras//@额外部分

/**
 * Extra Fan Speed
 * Adds a secondary fan speed for each print-cooling fan.
 *   'M106 P<fan> T3-255' : Set a secondary speed for <fan>
 *   'M106 P<fan> T2'     : Use the set secondary speed
 *   'M106 P<fan> T1'     : Restore the previous fan speed
 */
//#define EXTRA_FAN_SPEED//#定义额外的风扇转速

/**
 * Firmware-based and LCD-controlled retract
 *
 * Add G10 / G11 commands for automatic firmware-based retract / recover.
 * Use M207 and M208 to define parameters for retract / recover.
 *
 * Use M209 to enable or disable auto-retract.
 * With auto-retract enabled, all G1 E moves within the set range
 * will be converted to firmware-based retract/recover moves.
 *
 * Be sure to turn off auto-retract during filament change.
 *
 * Note that M207 / M208 / M209 settings are saved to EEPROM.
 */
//#define FWRETRACT//#定义收缩
#if ENABLED(FWRETRACT)
#define FWRETRACT_AUTORETRACT             // Override slicer retractions//超控切片机缩回
  #if ENABLED(FWRETRACT_AUTORETRACT)
    #define MIN_AUTORETRACT             0.1 // (mm) Don't convert E moves under this length//（mm）不转换此长度下的E移动
    #define MAX_AUTORETRACT            10.0 // (mm) Don't convert E moves over this length//（mm）不转换此长度上的E移动
  #endif
  #define RETRACT_LENGTH                3   // (mm) Default retract length (positive value)//（mm）默认收缩长度（正值）
  #define RETRACT_LENGTH_SWAP          13   // (mm) Default swap retract length (positive value)//（mm）默认交换回缩长度（正值）
  #define RETRACT_FEEDRATE             45   // (mm/s) Default feedrate for retracting//（mm/s）收缩的默认进给速度
  #define RETRACT_ZRAISE                0   // (mm) Default retract Z-raise//（mm）默认缩回Z向上升
  #define RETRACT_RECOVER_LENGTH        0   // (mm) Default additional recover length (added to retract length on recover)//（mm）默认附加恢复长度（恢复时添加到收缩长度）
  #define RETRACT_RECOVER_LENGTH_SWAP   0   // (mm) Default additional swap recover length (added to retract length on recover from toolchange)//（mm）默认附加交换恢复长度（添加到从刀具更改恢复时的收缩长度）
  #define RETRACT_RECOVER_FEEDRATE      8   // (mm/s) Default feedrate for recovering from retraction//（mm/s）从收缩中恢复的默认进给速度
  #define RETRACT_RECOVER_FEEDRATE_SWAP 8   // (mm/s) Default feedrate for recovering from swap retraction//（mm/s）从交换收回中恢复的默认进给速度
  #if ENABLED(MIXING_EXTRUDER)
    //#define RETRACT_SYNC_MIXING           // Retract and restore all mixing steppers simultaneously//#定义缩回\同步\混合//同时缩回和恢复所有混合步进器
  #endif
#endif

/**
 * Universal tool change settings.
 * Applies to all types of extruders except where explicitly noted.
 */
#if HAS_MULTI_EXTRUDER
// Z raise distance for tool-change, as needed for some extruders//Z根据某些挤出机的需要，提高换刀距离
  #define TOOLCHANGE_ZRAISE                 2 // (mm)//（毫米）
  //#define TOOLCHANGE_ZRAISE_BEFORE_RETRACT  // Apply raise before swap retraction (if enabled)//#在收回前定义工具更改提升//在交换收回前应用提升（如果启用）
  //#define TOOLCHANGE_NO_RETURN              // Never return to previous position on tool-change//#定义刀具更改\不\返回//刀具更改时从不返回到上一个位置
  #if ENABLED(TOOLCHANGE_NO_RETURN)
    //#define EVENT_GCODE_AFTER_TOOLCHANGE "G12X"   // Extra G-code to run after tool-change//#定义工具更改后的事件代码“G12X”//工具更改后运行的额外G代码
  #endif

  /**
   * Extra G-code to run while executing tool-change commands. Can be used to use an additional
   * stepper motor (I axis, see option LINEAR_AXES in Configuration.h) to drive the tool-changer.
   */
  //#define EVENT_GCODE_TOOLCHANGE_T0 "G28 A\nG1 A0" // Extra G-code to run while executing tool-change command T0//#定义执行刀具更换命令T0时要运行的事件\u GCODE \u TOOLCHANGE \u T0“G28 A\nG1 A0”//额外G代码
  //#define EVENT_GCODE_TOOLCHANGE_T1 "G1 A10"       // Extra G-code to run while executing tool-change command T1//#定义执行刀具更改命令T1时要运行的事件\ GCODE \刀具更改\ T1“G1 A10”//额外G代码

  /**
   * Tool Sensors detect when tools have been picked up or dropped.
   * Requires the pins TOOL_SENSOR1_PIN, TOOL_SENSOR2_PIN, etc.
   */
  //#define TOOL_SENSOR//#定义刀具位置传感器

  /**
   * Retract and prime filament on tool-change to reduce
   * ooze and stringing and to get cleaner transitions.
   */
  //#define TOOLCHANGE_FILAMENT_SWAP//#定义工具更改\u灯丝\u交换
  #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
    // Load / Unload//装载/卸载
    #define TOOLCHANGE_FS_LENGTH              12  // (mm) Load / Unload length//（mm）装载/卸载长度
    #define TOOLCHANGE_FS_EXTRA_RESUME_LENGTH  0  // (mm) Extra length for better restart, fine tune by LCD/Gcode)//（mm）额外长度，以便更好地重新启动，通过LCD/Gcode进行微调）
    #define TOOLCHANGE_FS_RETRACT_SPEED   (50*60) // (mm/min) (Unloading)//（毫米/分钟）（卸载）
    #define TOOLCHANGE_FS_UNRETRACT_SPEED (25*60) // (mm/min) (On SINGLENOZZLE or Bowden loading must be slowed down)//（mm/min）（单喷嘴或波顿加载必须减速）

    // Longer prime to clean out a SINGLENOZZLE//清洗单喷嘴的时间更长
    #define TOOLCHANGE_FS_EXTRA_PRIME          0  // (mm) Extra priming length//（mm）额外底漆长度
    #define TOOLCHANGE_FS_PRIME_SPEED    (4.6*60) // (mm/min) Extra priming feedrate//（mm/min）额外起动进给速度
    #define TOOLCHANGE_FS_WIPE_RETRACT         0  // (mm/min) Retract before cooling for less stringing, better wipe, etc.//（mm/min）在冷却前收缩，以减少架线、更好地擦拭等。

    // Cool after prime to reduce stringing//涂底漆后冷却，以减少架线
    #define TOOLCHANGE_FS_FAN                 -1  // Fan index or -1 to skip//要跳过风扇索引或-1
    #define TOOLCHANGE_FS_FAN_SPEED          255  // 0-255// 0-255
    #define TOOLCHANGE_FS_FAN_TIME            10  // (seconds)//（秒）

    // Swap uninitialized extruder with TOOLCHANGE_FS_PRIME_SPEED for all lengths (recover + prime)//将未初始化挤出机更换为所有长度的换刀速度（恢复+充模）
    // (May break filament if not retracted beforehand.)//（如果事先未缩回，可能会损坏灯丝。）
    //#define TOOLCHANGE_FS_INIT_BEFORE_SWAP//#在交换之前定义工具更改\u FS\u INIT\u

    // Prime on the first T0 (If other, TOOLCHANGE_FS_INIT_BEFORE_SWAP applied)//在第一个T0上初始化（如果其他，则在应用交换之前，工具更改\u FS\u INIT\u）
    // Enable it (M217 V[0/1]) before printing, to avoid unwanted priming on host connect//打印前启用它（M217 V[0/1]），以避免主机连接上出现不必要的启动
    //#define TOOLCHANGE_FS_PRIME_FIRST_USED//#首先定义使用的工具更改\u FS\u PRIME\u

    /**
     * Tool Change Migration
     * This feature provides G-code and LCD options to switch tools mid-print.
     * All applicable tool properties are migrated so the print can continue.
     * Tools must be closely matching and other restrictions may apply.
     * Useful to:
     *   - Change filament color without interruption
     *   - Switch spools automatically on filament runout
     *   - Switch to a different nozzle on an extruder jam
     */
    #define TOOLCHANGE_MIGRATION_FEATURE

  #endif

  /**
   * Position to park head during tool change.
   * Doesn't apply to SWITCHING_TOOLHEAD, DUAL_X_CARRIAGE, or PARKING_EXTRUDER
   */
  //#define TOOLCHANGE_PARK//#定义刀具更换位置
  #if ENABLED(TOOLCHANGE_PARK)
    #define TOOLCHANGE_PARK_XY    { X_MIN_POS + 10, Y_MIN_POS + 10 }
    #define TOOLCHANGE_PARK_XY_FEEDRATE 6000  // (mm/min)//（毫米/分钟）
    //#define TOOLCHANGE_PARK_X_ONLY          // X axis only move//#定义刀具更改\u仅驻车\u X\u//X轴仅移动
    //#define TOOLCHANGE_PARK_Y_ONLY          // Y axis only move//#定义刀具更改\u仅驻车\u仅Y\u//仅Y轴移动
  #endif
#endif // HAS_MULTI_EXTRUDER//HAS_多_挤出机

/**
 * Advanced Pause for Filament Change
 *  - Adds the G-code M600 Filament Change to initiate a filament change.
 *  - This feature is required for the default FILAMENT_RUNOUT_SCRIPT.
 *
 * Requirements:
 *  - For Filament Change parking enable and configure NOZZLE_PARK_FEATURE.
 *  - For user interaction enable an LCD display, HOST_PROMPT_SUPPORT, or EMERGENCY_PARSER.
 *
 * Enable PARK_HEAD_ON_PAUSE to add the G-code M125 Pause and Park.
 */
//#define ADVANCED_PAUSE_FEATURE//#定义高级暂停功能
#if ENABLED(ADVANCED_PAUSE_FEATURE)
#define PAUSE_PARK_RETRACT_FEEDRATE         60  // (mm/s) Initial retract feedrate.//（mm/s）初始回缩进给速度。
  #define PAUSE_PARK_RETRACT_LENGTH            2  // (mm) Initial retract.//（mm）初始缩回。
                                                  // This short retract is done immediately, before parking the nozzle.//在停车喷嘴之前，立即进行此短缩回。
  #define FILAMENT_CHANGE_UNLOAD_FEEDRATE     10  // (mm/s) Unload filament feedrate. This can be pretty fast.//（mm/s）卸载灯丝进给速度。这可能很快。
  #define FILAMENT_CHANGE_UNLOAD_ACCEL        25  // (mm/s^2) Lower acceleration may allow a faster feedrate.//（mm/s^2）较低的加速度可能允许更快的进给速度。
  #define FILAMENT_CHANGE_UNLOAD_LENGTH      100  // (mm) The length of filament for a complete unload.//（mm）完全卸载时灯丝的长度。
                                                  //   For Bowden, the full length of the tube and nozzle.//对于波顿，指管和喷嘴的全长。
                                                  //   For direct drive, the full length of the nozzle.//对于直接驱动，喷嘴的全长。
                                                  //   Set to 0 for manual unloading.//手动卸载设置为0。
  #define FILAMENT_CHANGE_SLOW_LOAD_FEEDRATE   6  // (mm/s) Slow move when starting load.//（mm/s）启动负载时缓慢移动。
  #define FILAMENT_CHANGE_SLOW_LOAD_LENGTH     0  // (mm) Slow length, to allow time to insert material.//（mm）慢速长度，以便有时间插入材料。
                                                  // 0 to disable start loading and skip to fast load only//0以禁用开始加载并仅跳到快速加载
  #define FILAMENT_CHANGE_FAST_LOAD_FEEDRATE   6  // (mm/s) Load filament feedrate. This can be pretty fast.//（mm/s）负载灯丝进给速度。这可能很快。
  #define FILAMENT_CHANGE_FAST_LOAD_ACCEL     25  // (mm/s^2) Lower acceleration may allow a faster feedrate.//（mm/s^2）较低的加速度可能允许更快的进给速度。
  #define FILAMENT_CHANGE_FAST_LOAD_LENGTH     0  // (mm) Load length of filament, from extruder gear to nozzle.//（mm）从挤出机齿轮到喷嘴的灯丝负载长度。
                                                  //   For Bowden, the full length of the tube and nozzle.//对于波顿，指管和喷嘴的全长。
                                                  //   For direct drive, the full length of the nozzle.//对于直接驱动，喷嘴的全长。
  //#define ADVANCED_PAUSE_CONTINUOUS_PURGE       // Purge continuously up to the purge length until interrupted.//#定义高级\u暂停\u连续\u清除//持续清除，直到清除长度中断。
  #define ADVANCED_PAUSE_PURGE_FEEDRATE        3  // (mm/s) Extrude feedrate (after loading). Should be slower than load feedrate.//（mm/s）挤压进给速度（加载后）。应低于负载进给速度。
  #define ADVANCED_PAUSE_PURGE_LENGTH         50  // (mm) Length to extrude after loading.//（mm）加载后挤出的长度。
                                                  //   Set to 0 for manual extrusion.//手动拉伸设置为0。
                                                  //   Filament can be extruded repeatedly from the Filament Change menu//可以从“灯丝更改”菜单重复挤出灯丝
                                                  //   until extrusion is consistent, and to purge old filament.//直到挤出一致，并吹扫旧灯丝。
  #define ADVANCED_PAUSE_RESUME_PRIME          0  // (mm) Extra distance to prime nozzle after returning from park.//（mm）从停车场返回后到注油嘴的额外距离。
  //#define ADVANCED_PAUSE_FANS_PAUSE             // Turn off print-cooling fans while the machine is paused.//#定义高级\u暂停\u风扇\u暂停//在机器暂停时关闭打印冷却风扇。

                                                  // Filament Unload does a Retract, Delay, and Purge first://灯丝卸载首先进行缩回、延迟和吹扫：
  #define FILAMENT_UNLOAD_PURGE_RETRACT       13  // (mm) Unload initial retract length.//（mm）卸载初始回缩长度。
  #define FILAMENT_UNLOAD_PURGE_DELAY       5000  // (ms) Delay for the filament to cool after retract.//（ms）回缩后灯丝冷却的延迟时间。
  #define FILAMENT_UNLOAD_PURGE_LENGTH         8  // (mm) An unretract is done, then this length is purged.//（mm）完成未回收，然后清除该长度。
  #define FILAMENT_UNLOAD_PURGE_FEEDRATE      25  // (mm/s) feedrate to purge before unload//（mm/s）卸载前净化的进给速度

  #define PAUSE_PARK_NOZZLE_TIMEOUT           45  // (seconds) Time limit before the nozzle is turned off for safety.//（秒）为安全起见关闭喷嘴前的时间限制。
  #define FILAMENT_CHANGE_ALERT_BEEPS         10  // Number of alert beeps to play when a response is needed.//需要响应时播放的警报蜂鸣音数。
  #define PAUSE_PARK_NO_STEPPER_TIMEOUT           // Enable for XYZ steppers to stay powered on during filament change.//使XYZ步进电机在灯丝更换期间保持通电。

  //#define PARK_HEAD_ON_PAUSE                    // Park the nozzle during pause and filament change.//#在暂停和灯丝更换期间，在暂停/停驻喷嘴上定义停驻头。
  //#define HOME_BEFORE_FILAMENT_CHANGE           // If needed, home before parking for filament change//#在更换灯丝前定义原点//如果需要，在停车更换灯丝前定义原点

  //#define FILAMENT_LOAD_UNLOAD_GCODES           // Add M701/M702 Load/Unload G-codes, plus Load/Unload in the LCD Prepare menu.//#定义灯丝加载卸载代码//添加M701/M702加载/卸载G代码，以及LCD准备菜单中的加载/卸载。
  //#define FILAMENT_UNLOAD_ALL_EXTRUDERS         // Allow M702 to unload all extruders above a minimum target temp (as set by M302)//#定义灯丝卸载所有挤出机//允许M702卸载高于最低目标温度的所有挤出机（由M302设定）
#endif

// @section tmc//@节tmc

/**
 * TMC26X Stepper Driver options
 *
 * The TMC26XStepper library is required for this stepper driver.
 * https://github.com/trinamic/TMC26XStepper
 */
#if HAS_DRIVER(TMC26X)

#if AXIS_DRIVER_TYPE_X(TMC26X)
    #define X_MAX_CURRENT     1000  // (mA)//（马）
    #define X_SENSE_RESISTOR    91  // (mOhms)//（莫姆斯）
    #define X_MICROSTEPS        16  // Number of microsteps//微步数
  #endif

  #if AXIS_DRIVER_TYPE_X2(TMC26X)
    #define X2_MAX_CURRENT    1000
    #define X2_SENSE_RESISTOR   91
    #define X2_MICROSTEPS       X_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_Y(TMC26X)
    #define Y_MAX_CURRENT     1000
    #define Y_SENSE_RESISTOR    91
    #define Y_MICROSTEPS        16
  #endif

  #if AXIS_DRIVER_TYPE_Y2(TMC26X)
    #define Y2_MAX_CURRENT    1000
    #define Y2_SENSE_RESISTOR   91
    #define Y2_MICROSTEPS       Y_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_Z(TMC26X)
    #define Z_MAX_CURRENT     1000
    #define Z_SENSE_RESISTOR    91
    #define Z_MICROSTEPS        16
  #endif

  #if AXIS_DRIVER_TYPE_Z2(TMC26X)
    #define Z2_MAX_CURRENT    1000
    #define Z2_SENSE_RESISTOR   91
    #define Z2_MICROSTEPS       Z_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_Z3(TMC26X)
    #define Z3_MAX_CURRENT    1000
    #define Z3_SENSE_RESISTOR   91
    #define Z3_MICROSTEPS       Z_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_Z4(TMC26X)
    #define Z4_MAX_CURRENT    1000
    #define Z4_SENSE_RESISTOR   91
    #define Z4_MICROSTEPS       Z_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_I(TMC26X)
    #define I_MAX_CURRENT    1000
    #define I_SENSE_RESISTOR   91
    #define I_MICROSTEPS       16
  #endif

  #if AXIS_DRIVER_TYPE_J(TMC26X)
    #define J_MAX_CURRENT    1000
    #define J_SENSE_RESISTOR   91
    #define J_MICROSTEPS       16
  #endif

  #if AXIS_DRIVER_TYPE_K(TMC26X)
    #define K_MAX_CURRENT    1000
    #define K_SENSE_RESISTOR   91
    #define K_MICROSTEPS       16
  #endif

  #if AXIS_DRIVER_TYPE_E0(TMC26X)
    #define E0_MAX_CURRENT    1000
    #define E0_SENSE_RESISTOR   91
    #define E0_MICROSTEPS       16
  #endif

  #if AXIS_DRIVER_TYPE_E1(TMC26X)
    #define E1_MAX_CURRENT    1000
    #define E1_SENSE_RESISTOR   91
    #define E1_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E2(TMC26X)
    #define E2_MAX_CURRENT    1000
    #define E2_SENSE_RESISTOR   91
    #define E2_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E3(TMC26X)
    #define E3_MAX_CURRENT    1000
    #define E3_SENSE_RESISTOR   91
    #define E3_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E4(TMC26X)
    #define E4_MAX_CURRENT    1000
    #define E4_SENSE_RESISTOR   91
    #define E4_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E5(TMC26X)
    #define E5_MAX_CURRENT    1000
    #define E5_SENSE_RESISTOR   91
    #define E5_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E6(TMC26X)
    #define E6_MAX_CURRENT    1000
    #define E6_SENSE_RESISTOR   91
    #define E6_MICROSTEPS       E0_MICROSTEPS
  #endif

  #if AXIS_DRIVER_TYPE_E7(TMC26X)
    #define E7_MAX_CURRENT    1000
    #define E7_SENSE_RESISTOR   91
    #define E7_MICROSTEPS       E0_MICROSTEPS
  #endif

#endif // TMC26X//TMC26X

// @section tmc_smart//@section tmc_smart

/**
 * To use TMC2130, TMC2160, TMC2660, TMC5130, TMC5160 stepper drivers in SPI mode
 * connect your SPI pins to the hardware SPI interface on your board and define
 * the required CS pins in your `pins_MYBOARD.h` file. (e.g., RAMPS 1.4 uses AUX3
 * pins `X_CS_PIN 53`, `Y_CS_PIN 49`, etc.).
 * You may also use software SPI if you wish to use general purpose IO pins.
 *
 * To use TMC2208 stepper UART-configurable stepper drivers connect #_SERIAL_TX_PIN
 * to the driver side PDN_UART pin with a 1K resistor.
 * To use the reading capabilities, also connect #_SERIAL_RX_PIN to PDN_UART without
 * a resistor.
 * The drivers can also be used with hardware serial.
 *
 * TMCStepper library is required to use TMC stepper drivers.
 * https://github.com/teemuatlut/TMCStepper
 */
#if HAS_TRINAMIC_CONFIG

#define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current//从运行电流向下缩放保持电流

  /**
   * Interpolate microsteps to 256
   * Override for each driver with <driver>_INTERPOLATE settings below
   */
  #define INTERPOLATE      true

  #if AXIS_IS_TMC(X)
    #define X_CURRENT       800        // (mA) RMS current. Multiply by 1.414 for peak current.//（mA）均方根电流。峰值电流乘以1.414。
    #define X_CURRENT_HOME  X_CURRENT  // (mA) RMS current for sensorless homing//（mA）无传感器寻的均方根电流
    #define X_MICROSTEPS     16        // 0..256// 0..256
    #define X_RSENSE          0.11
    #define X_CHAIN_POS      -1        // -1..0: Not chained. 1: MCU MOSI connected. 2: Next in chain, ...//-1..0:未链接。1：MCU MOSI已连接。2：链中的下一个。。。
    //#define X_INTERPOLATE  true      // Enable to override 'INTERPOLATE' for the X axis//#定义X_INTERPOLATE true//启用以覆盖X轴的“INTERPOLATE”
  #endif

  #if AXIS_IS_TMC(X2)
    #define X2_CURRENT      800
    #define X2_CURRENT_HOME X2_CURRENT
    #define X2_MICROSTEPS    X_MICROSTEPS
    #define X2_RSENSE         0.11
    #define X2_CHAIN_POS     -1
    //#define X2_INTERPOLATE true//#定义X2_插值为真
  #endif

  #if AXIS_IS_TMC(Y)
    #define Y_CURRENT       800
    #define Y_CURRENT_HOME  Y_CURRENT
    #define Y_MICROSTEPS     16
    #define Y_RSENSE          0.11
    #define Y_CHAIN_POS      -1
    //#define Y_INTERPOLATE  true//#定义Y_插值为真
  #endif

  #if AXIS_IS_TMC(Y2)
    #define Y2_CURRENT      800
    #define Y2_CURRENT_HOME Y2_CURRENT
    #define Y2_MICROSTEPS    Y_MICROSTEPS
    #define Y2_RSENSE         0.11
    #define Y2_CHAIN_POS     -1
    //#define Y2_INTERPOLATE true//#定义Y2_插值为真
  #endif

  #if AXIS_IS_TMC(Z)
    #define Z_CURRENT       800
    #define Z_CURRENT_HOME  Z_CURRENT
    #define Z_MICROSTEPS     16
    #define Z_RSENSE          0.11
    #define Z_CHAIN_POS      -1
    //#define Z_INTERPOLATE  true//#定义Z_插值为真
  #endif

  #if AXIS_IS_TMC(Z2)
    #define Z2_CURRENT      800
    #define Z2_CURRENT_HOME Z2_CURRENT
    #define Z2_MICROSTEPS    Z_MICROSTEPS
    #define Z2_RSENSE         0.11
    #define Z2_CHAIN_POS     -1
    //#define Z2_INTERPOLATE true//#定义Z2_插值为真
  #endif

  #if AXIS_IS_TMC(Z3)
    #define Z3_CURRENT      800
    #define Z3_CURRENT_HOME Z3_CURRENT
    #define Z3_MICROSTEPS    Z_MICROSTEPS
    #define Z3_RSENSE         0.11
    #define Z3_CHAIN_POS     -1
    //#define Z3_INTERPOLATE true//#定义Z3_插值为真
  #endif

  #if AXIS_IS_TMC(Z4)
    #define Z4_CURRENT      800
    #define Z4_CURRENT_HOME Z4_CURRENT
    #define Z4_MICROSTEPS    Z_MICROSTEPS
    #define Z4_RSENSE         0.11
    #define Z4_CHAIN_POS     -1
    //#define Z4_INTERPOLATE true//#定义Z4_插值为真
  #endif

  #if AXIS_IS_TMC(I)
    #define I_CURRENT      800
    #define I_CURRENT_HOME I_CURRENT
    #define I_MICROSTEPS    16
    #define I_RSENSE         0.11
    #define I_CHAIN_POS     -1
    //#define I_INTERPOLATE  true//#定义I_插值为真
  #endif

  #if AXIS_IS_TMC(J)
    #define J_CURRENT      800
    #define J_CURRENT_HOME J_CURRENT
    #define J_MICROSTEPS    16
    #define J_RSENSE         0.11
    #define J_CHAIN_POS     -1
    //#define J_INTERPOLATE  true//#定义J_插值为真
  #endif

  #if AXIS_IS_TMC(K)
    #define K_CURRENT      800
    #define K_CURRENT_HOME K_CURRENT
    #define K_MICROSTEPS    16
    #define K_RSENSE         0.11
    #define K_CHAIN_POS     -1
    //#define K_INTERPOLATE  true//#定义K_插值为真
  #endif

  #if AXIS_IS_TMC(E0)
    #define E0_CURRENT      800
    #define E0_MICROSTEPS    16
    #define E0_RSENSE         0.11
    #define E0_CHAIN_POS     -1
    //#define E0_INTERPOLATE true//#定义E0_插值为真
  #endif

  #if AXIS_IS_TMC(E1)
    #define E1_CURRENT      800
    #define E1_MICROSTEPS   E0_MICROSTEPS
    #define E1_RSENSE         0.11
    #define E1_CHAIN_POS     -1
    //#define E1_INTERPOLATE true//#定义E1_插值为真
  #endif

  #if AXIS_IS_TMC(E2)
    #define E2_CURRENT      800
    #define E2_MICROSTEPS   E0_MICROSTEPS
    #define E2_RSENSE         0.11
    #define E2_CHAIN_POS     -1
    //#define E2_INTERPOLATE true//#定义E2_插值为真
  #endif

  #if AXIS_IS_TMC(E3)
    #define E3_CURRENT      800
    #define E3_MICROSTEPS   E0_MICROSTEPS
    #define E3_RSENSE         0.11
    #define E3_CHAIN_POS     -1
    //#define E3_INTERPOLATE true//#定义E3_插值为真
  #endif

  #if AXIS_IS_TMC(E4)
    #define E4_CURRENT      800
    #define E4_MICROSTEPS   E0_MICROSTEPS
    #define E4_RSENSE         0.11
    #define E4_CHAIN_POS     -1
    //#define E4_INTERPOLATE true//#定义E4_插值为真
  #endif

  #if AXIS_IS_TMC(E5)
    #define E5_CURRENT      800
    #define E5_MICROSTEPS   E0_MICROSTEPS
    #define E5_RSENSE         0.11
    #define E5_CHAIN_POS     -1
    //#define E5_INTERPOLATE true//#定义E5_插值为真
  #endif

  #if AXIS_IS_TMC(E6)
    #define E6_CURRENT      800
    #define E6_MICROSTEPS   E0_MICROSTEPS
    #define E6_RSENSE         0.11
    #define E6_CHAIN_POS     -1
    //#define E6_INTERPOLATE true//#定义E6_插值为真
  #endif

  #if AXIS_IS_TMC(E7)
    #define E7_CURRENT      800
    #define E7_MICROSTEPS   E0_MICROSTEPS
    #define E7_RSENSE         0.11
    #define E7_CHAIN_POS     -1
    //#define E7_INTERPOLATE true//#定义E7_插值为真
  #endif

  /**
   * Override default SPI pins for TMC2130, TMC2160, TMC2660, TMC5130 and TMC5160 drivers here.
   * The default pins can be found in your board's pins file.
   */
  //#define X_CS_PIN          -1//#定义X_CS_引脚-1
  //#define Y_CS_PIN          -1//#定义Y_CS_引脚-1
  //#define Z_CS_PIN          -1//#定义Z_CS_引脚-1
  //#define X2_CS_PIN         -1//#定义X2_CS_引脚-1
  //#define Y2_CS_PIN         -1//#定义Y2_CS_引脚-1
  //#define Z2_CS_PIN         -1//#定义Z2_CS_引脚-1
  //#define Z3_CS_PIN         -1//#定义Z3_CS_引脚-1
  //#define Z4_CS_PIN         -1//#定义Z4_CS_引脚-1
  //#define I_CS_PIN          -1//#定义I_CS_引脚-1
  //#define J_CS_PIN          -1//#定义J_CS_引脚-1
  //#define K_CS_PIN          -1//#定义K_CS_引脚-1
  //#define E0_CS_PIN         -1//#定义E0_CS_引脚-1
  //#define E1_CS_PIN         -1//#定义E1_CS_引脚-1
  //#define E2_CS_PIN         -1//#定义E2_CS_引脚-1
  //#define E3_CS_PIN         -1//#定义E3_CS_引脚-1
  //#define E4_CS_PIN         -1//#定义E4_CS_引脚-1
  //#define E5_CS_PIN         -1//#定义E5_CS_引脚-1
  //#define E6_CS_PIN         -1//#定义E6_CS_引脚-1
  //#define E7_CS_PIN         -1//#定义E7_CS_引脚-1

  /**
   * Software option for SPI driven drivers (TMC2130, TMC2160, TMC2660, TMC5130 and TMC5160).
   * The default SW SPI pins are defined the respective pins files,
   * but you can override or define them here.
   */
  //#define TMC_USE_SW_SPI//#定义TMC\u使用\u软件\u SPI
  //#define TMC_SW_MOSI       -1//#定义TMC_SW_MOSI-1
  //#define TMC_SW_MISO       -1//#定义TMC_SW_味噌-1
  //#define TMC_SW_SCK        -1//#定义TMC_SW_SCK-1

  /**
   * Four TMC2209 drivers can use the same HW/SW serial port with hardware configured addresses.
   * Set the address using jumpers on pins MS1 and MS2.
   * Address | MS1  | MS2
   *       0 | LOW  | LOW
   *       1 | HIGH | LOW
   *       2 | LOW  | HIGH
   *       3 | HIGH | HIGH
   *
   * Set *_SERIAL_TX_PIN and *_SERIAL_RX_PIN to match for all drivers
   * on the same serial port, either here or in your board's pins file.
   */
  //#define  X_SLAVE_ADDRESS 0//#定义X_从属地址0
  //#define  Y_SLAVE_ADDRESS 0//#定义Y_从属地址0
  //#define  Z_SLAVE_ADDRESS 0//#定义Z_从属地址0
  //#define X2_SLAVE_ADDRESS 0//#定义X2_从_地址0
  //#define Y2_SLAVE_ADDRESS 0//#定义Y2_从_地址0
  //#define Z2_SLAVE_ADDRESS 0//#定义Z2_从_地址0
  //#define Z3_SLAVE_ADDRESS 0//#定义Z3_从_地址0
  //#define Z4_SLAVE_ADDRESS 0//#定义Z4_从_地址0
  //#define  I_SLAVE_ADDRESS 0//#定义I_从属地址0
  //#define  J_SLAVE_ADDRESS 0//#定义J_从属地址0
  //#define  K_SLAVE_ADDRESS 0//#定义K_从_地址0
  //#define E0_SLAVE_ADDRESS 0//#定义E0_从_地址0
  //#define E1_SLAVE_ADDRESS 0//#定义E1_从_地址0
  //#define E2_SLAVE_ADDRESS 0//#定义E2_从_地址0
  //#define E3_SLAVE_ADDRESS 0//#定义E3_从_地址0
  //#define E4_SLAVE_ADDRESS 0//#定义E4\u从\u地址0
  //#define E5_SLAVE_ADDRESS 0//#定义E5_从_地址0
  //#define E6_SLAVE_ADDRESS 0//#定义E6_从_地址0
  //#define E7_SLAVE_ADDRESS 0//#定义E7_从_地址0

  /**
   * Software enable
   *
   * Use for drivers that do not use a dedicated enable pin, but rather handle the same
   * function through a communication line such as SPI or UART.
   */
  //#define SOFTWARE_DRIVER_ENABLE//#定义软件\u驱动程序\u启用

  /**
   * TMC2130, TMC2160, TMC2208, TMC2209, TMC5130 and TMC5160 only
   * Use Trinamic's ultra quiet stepping mode.
   * When disabled, Marlin will use spreadCycle stepping mode.
   */
  #define STEALTHCHOP_XY
  #define STEALTHCHOP_Z
  #define STEALTHCHOP_I
  #define STEALTHCHOP_J
  #define STEALTHCHOP_K
  #define STEALTHCHOP_E

  /**
   * Optimize spreadCycle chopper parameters by using predefined parameter sets
   * or with the help of an example included in the library.
   * Provided parameter sets are
   * CHOPPER_DEFAULT_12V
   * CHOPPER_DEFAULT_19V
   * CHOPPER_DEFAULT_24V
   * CHOPPER_DEFAULT_36V
   * CHOPPER_09STEP_24V   // 0.9 degree steppers (24V)
   * CHOPPER_PRUSAMK3_24V // Imported parameters from the official Průša firmware for MK3 (24V)
   * CHOPPER_MARLIN_119   // Old defaults from Marlin v1.1.9
   *
   * Define your own with:
   * { <off_time[1..15]>, <hysteresis_end[-3..12]>, hysteresis_start[1..8] }
   */
  #define CHOPPER_TIMING CHOPPER_DEFAULT_12V        // All axes (override below)//所有轴（下面的替代）
  //#define CHOPPER_TIMING_X  CHOPPER_TIMING        // For X Axes (override below)//#为X轴定义斩波器_正时_X斩波器_正时//如下（覆盖）
  //#define CHOPPER_TIMING_X2 CHOPPER_TIMING_X//#定义斩波器_正时_X2斩波器_正时_X
  //#define CHOPPER_TIMING_Y  CHOPPER_TIMING        // For Y Axes (override below)//#为Y轴定义斩波器\正时\ Y斩波器\正时//如下（覆盖）
  //#define CHOPPER_TIMING_Y2 CHOPPER_TIMING_Y//#定义斩波器\正时\ Y2斩波器\正时\ Y
  //#define CHOPPER_TIMING_Z  CHOPPER_TIMING        // For Z Axes (override below)//#为Z轴定义斩波器正时（覆盖如下）
  //#define CHOPPER_TIMING_Z2 CHOPPER_TIMING_Z//#定义斩波器正时2斩波器正时
  //#define CHOPPER_TIMING_Z3 CHOPPER_TIMING_Z//#定义斩波器正时3斩波器正时
  //#define CHOPPER_TIMING_Z4 CHOPPER_TIMING_Z//#定义斩波器正时4斩波器正时
  //#define CHOPPER_TIMING_E  CHOPPER_TIMING        // For Extruders (override below)//#为挤出机定义切碎器正时切碎器正时//如下所示
  //#define CHOPPER_TIMING_E1 CHOPPER_TIMING_E//#定义斩波器定时E1斩波器定时
  //#define CHOPPER_TIMING_E2 CHOPPER_TIMING_E//#定义斩波器正时E2斩波器正时E
  //#define CHOPPER_TIMING_E3 CHOPPER_TIMING_E//#定义斩波器定时E3斩波器定时
  //#define CHOPPER_TIMING_E4 CHOPPER_TIMING_E//#定义斩波器正时4斩波器正时
  //#define CHOPPER_TIMING_E5 CHOPPER_TIMING_E//#定义斩波器正时E5斩波器正时
  //#define CHOPPER_TIMING_E6 CHOPPER_TIMING_E//#定义斩波器正时6斩波器正时
  //#define CHOPPER_TIMING_E7 CHOPPER_TIMING_E//#定义斩波器正时7斩波器正时

  /**
   * Monitor Trinamic drivers
   * for error conditions like overtemperature and short to ground.
   * To manage over-temp Marlin can decrease the driver current until the error condition clears.
   * Other detected conditions can be used to stop the current print.
   * Relevant G-codes:
   * M906 - Set or get motor current in milliamps using axis codes X, Y, Z, E. Report values if no axis codes given.
   * M911 - Report stepper driver overtemperature pre-warn condition.
   * M912 - Clear stepper driver overtemperature pre-warn condition flag.
   * M122 - Report driver parameters (Requires TMC_DEBUG)
   */
  //#define MONITOR_DRIVER_STATUS//#定义监视器驱动程序状态

  #if ENABLED(MONITOR_DRIVER_STATUS)
    #define CURRENT_STEP_DOWN     50  // [mA]//[硕士]
    #define REPORT_CURRENT_CHANGE
    #define STOP_ON_ERROR
  #endif

  /**
   * TMC2130, TMC2160, TMC2208, TMC2209, TMC5130 and TMC5160 only
   * The driver will switch to spreadCycle when stepper speed is over HYBRID_THRESHOLD.
   * This mode allows for faster movements at the expense of higher noise levels.
   * STEALTHCHOP_(XY|Z|E) must be enabled to use HYBRID_THRESHOLD.
   * M913 X/Y/Z/E to live tune the setting
   */
  //#define HYBRID_THRESHOLD//#定义混合_阈值

  #define X_HYBRID_THRESHOLD     100  // [mm/s]//[毫米/秒]
  #define X2_HYBRID_THRESHOLD    100
  #define Y_HYBRID_THRESHOLD     100
  #define Y2_HYBRID_THRESHOLD    100
  #define Z_HYBRID_THRESHOLD       3
  #define Z2_HYBRID_THRESHOLD      3
  #define Z3_HYBRID_THRESHOLD      3
  #define Z4_HYBRID_THRESHOLD      3
  #define I_HYBRID_THRESHOLD       3
  #define J_HYBRID_THRESHOLD       3
  #define K_HYBRID_THRESHOLD       3
  #define E0_HYBRID_THRESHOLD     30
  #define E1_HYBRID_THRESHOLD     30
  #define E2_HYBRID_THRESHOLD     30
  #define E3_HYBRID_THRESHOLD     30
  #define E4_HYBRID_THRESHOLD     30
  #define E5_HYBRID_THRESHOLD     30
  #define E6_HYBRID_THRESHOLD     30
  #define E7_HYBRID_THRESHOLD     30

  /**
   * Use StallGuard to home / probe X, Y, Z.
   *
   * TMC2130, TMC2160, TMC2209, TMC2660, TMC5130, and TMC5160 only
   * Connect the stepper driver's DIAG1 pin to the X/Y endstop pin.
   * X, Y, and Z homing will always be done in spreadCycle mode.
   *
   * X/Y/Z_STALL_SENSITIVITY is the default stall threshold.
   * Use M914 X Y Z to set the stall threshold at runtime:
   *
   *  Sensitivity   TMC2209   Others
   *    HIGHEST       255      -64    (Too sensitive => False positive)
   *    LOWEST         0        63    (Too insensitive => No trigger)
   *
   * It is recommended to set HOMING_BUMP_MM to { 0, 0, 0 }.
   *
   * SPI_ENDSTOPS  *** Beta feature! *** TMC2130/TMC5160 Only ***
   * Poll the driver through SPI to determine load when homing.
   * Removes the need for a wire from DIAG1 to an endstop pin.
   *
   * IMPROVE_HOMING_RELIABILITY tunes acceleration and jerk when
   * homing and adds a guard period for endstop triggering.
   *
   * Comment *_STALL_SENSITIVITY to disable sensorless homing for that axis.
   */
  //#define SENSORLESS_HOMING // StallGuard capable drivers only//#定义无传感器_归位//仅限支持StallGuard的驱动程序

  #if EITHER(SENSORLESS_HOMING, SENSORLESS_PROBING)
    // TMC2209: 0...255. TMC2130: -64...63//TMC2209:0…255。TMC2130:-64…63
    #define X_STALL_SENSITIVITY  8
    #define X2_STALL_SENSITIVITY X_STALL_SENSITIVITY
    #define Y_STALL_SENSITIVITY  8
    #define Y2_STALL_SENSITIVITY Y_STALL_SENSITIVITY
    //#define Z_STALL_SENSITIVITY  8//#定义Z_失速灵敏度8
    //#define Z2_STALL_SENSITIVITY Z_STALL_SENSITIVITY//#定义Z2_失速灵敏度Z_失速灵敏度
    //#define Z3_STALL_SENSITIVITY Z_STALL_SENSITIVITY//#定义Z3_失速灵敏度Z_失速灵敏度
    //#define Z4_STALL_SENSITIVITY Z_STALL_SENSITIVITY//#定义Z4_失速灵敏度Z_失速灵敏度
    //#define I_STALL_SENSITIVITY  8//#定义I_失速灵敏度8
    //#define J_STALL_SENSITIVITY  8//#定义J_失速灵敏度8
    //#define K_STALL_SENSITIVITY  8//#定义K_失速灵敏度8
    //#define SPI_ENDSTOPS              // TMC2130 only//#仅定义SPI_端止点//TMC2130
    //#define IMPROVE_HOMING_RELIABILITY//#定义并提高归位可靠性
  #endif

  /**
   * TMC Homing stepper phase.
   *
   * Improve homing repeatability by homing to stepper coil's nearest absolute
   * phase position. Trinamic drivers use a stepper phase table with 1024 values
   * spanning 4 full steps with 256 positions each (ergo, 1024 positions).
   * Full step positions (128, 384, 640, 896) have the highest holding torque.
   *
   * Values from 0..1023, -1 to disable homing phase for that axis.
   */
   //#define TMC_HOME_PHASE { 896, 896, 896 }//#定义TMC_HOME_阶段{896896896}

  /**
   * Beta feature!
   * Create a 50/50 square wave step pulse optimal for stepper drivers.
   */
  //#define SQUARE_WAVE_STEPPING//#定义方波步进

  /**
   * Enable M122 debugging command for TMC stepper drivers.
   * M122 S0/1 will enable continuous reporting.
   */
  //#define TMC_DEBUG//#定义TMC_调试

  /**
   * You can set your own advanced settings by filling in predefined functions.
   * A list of available functions can be found on the library github page
   * https://github.com/teemuatlut/TMCStepper
   *
   * Example:
   * #define TMC_ADV() { \
   *   stepperX.diag0_otpw(1); \
   *   stepperY.intpol(0); \
   * }
   */
  #define TMC_ADV() {  }

#endif // HAS_TRINAMIC_CONFIG//有_TRINAMIC _CONFIG

// @section L64XX//@第L64XX节

/**
 * L64XX Stepper Driver options
 *
 * Arduino-L6470 library (0.8.0 or higher) is required.
 * https://github.com/ameyer/Arduino-L6470
 *
 * Requires the following to be defined in your pins_YOUR_BOARD file
 *     L6470_CHAIN_SCK_PIN
 *     L6470_CHAIN_MISO_PIN
 *     L6470_CHAIN_MOSI_PIN
 *     L6470_CHAIN_SS_PIN
 *     ENABLE_RESET_L64XX_CHIPS(Q)  where Q is 1 to enable and 0 to reset
 */

#if HAS_L64XX

//#define L6470_CHITCHAT        // Display additional status info//#定义L6470_聊天//显示其他状态信息

  #if AXIS_IS_L64XX(X)
    #define X_MICROSTEPS       128  // Number of microsteps (VALID: 1, 2, 4, 8, 16, 32, 128) - L6474 max is 16//微步数（有效值：1、2、4、8、16、32、128）-L6474最大值为16
    #define X_OVERCURRENT     2000  // (mA) Current where the driver detects an over current//（mA）驱动器检测到过电流时的电流
                                    //   L6470 & L6474 - VALID: 375 x (1 - 16) - 6A max - rounds down//L6470和L6474-有效：375 x（1-16）-6A最大值-向下舍入
                                    //   POWERSTEP01: VALID: 1000 x (1 - 32) - 32A max - rounds down//POWERSTEP01:有效：1000 x（1-32）-最大32A-向下舍入
    #define X_STALLCURRENT    1500  // (mA) Current where the driver detects a stall (VALID: 31.25 * (1-128) -  4A max - rounds down)//（mA）驾驶员检测到失速时的电流（有效值：31.25*（1-128）-4A最大值-向下舍入）
                                    //   L6470 & L6474 - VALID: 31.25 * (1-128) -  4A max - rounds down//L6470和L6474-有效期：31.25*（1-128）-4A最大值-向下舍入
                                    //   POWERSTEP01: VALID: 200 x (1 - 32) - 6.4A max - rounds down//POWERSTEP01:有效：200 x（1-32）-最大6.4A-向下舍入
                                    //   L6474 - STALLCURRENT setting is used to set the nominal (TVAL) current//L6474-失速电流设置用于设置标称（TVAL）电流
    #define X_MAX_VOLTAGE      127  // 0-255, Maximum effective voltage seen by stepper - not used by L6474//0-255，步进电机看到的最大有效电压-L6474未使用
    #define X_CHAIN_POS         -1  // Position in SPI chain, 0=Not in chain, 1=Nearest MOSI//SPI链中的位置，0=不在链中，1=最近的MOSI
    #define X_SLEW_RATE          1  // 0-3, Slew 0 is slowest, 3 is fastest//0-3，回转0最慢，3最快
  #endif

  #if AXIS_IS_L64XX(X2)
    #define X2_MICROSTEPS     X_MICROSTEPS
    #define X2_OVERCURRENT            2000
    #define X2_STALLCURRENT           1500
    #define X2_MAX_VOLTAGE             127
    #define X2_CHAIN_POS                -1
    #define X2_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(Y)
    #define Y_MICROSTEPS               128
    #define Y_OVERCURRENT             2000
    #define Y_STALLCURRENT            1500
    #define Y_MAX_VOLTAGE              127
    #define Y_CHAIN_POS                 -1
    #define Y_SLEW_RATE                  1
  #endif

  #if AXIS_IS_L64XX(Y2)
    #define Y2_MICROSTEPS     Y_MICROSTEPS
    #define Y2_OVERCURRENT            2000
    #define Y2_STALLCURRENT           1500
    #define Y2_MAX_VOLTAGE             127
    #define Y2_CHAIN_POS                -1
    #define Y2_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(Z)
    #define Z_MICROSTEPS               128
    #define Z_OVERCURRENT             2000
    #define Z_STALLCURRENT            1500
    #define Z_MAX_VOLTAGE              127
    #define Z_CHAIN_POS                 -1
    #define Z_SLEW_RATE                  1
  #endif

  #if AXIS_IS_L64XX(Z2)
    #define Z2_MICROSTEPS     Z_MICROSTEPS
    #define Z2_OVERCURRENT            2000
    #define Z2_STALLCURRENT           1500
    #define Z2_MAX_VOLTAGE             127
    #define Z2_CHAIN_POS                -1
    #define Z2_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(Z3)
    #define Z3_MICROSTEPS     Z_MICROSTEPS
    #define Z3_OVERCURRENT            2000
    #define Z3_STALLCURRENT           1500
    #define Z3_MAX_VOLTAGE             127
    #define Z3_CHAIN_POS                -1
    #define Z3_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(Z4)
    #define Z4_MICROSTEPS     Z_MICROSTEPS
    #define Z4_OVERCURRENT            2000
    #define Z4_STALLCURRENT           1500
    #define Z4_MAX_VOLTAGE             127
    #define Z4_CHAIN_POS                -1
    #define Z4_SLEW_RATE                 1
  #endif

  #if AXIS_DRIVER_TYPE_I(L6470)
    #define I_MICROSTEPS      128
    #define I_OVERCURRENT    2000
    #define I_STALLCURRENT   1500
    #define I_MAX_VOLTAGE     127
    #define I_CHAIN_POS        -1
    #define I_SLEW_RATE         1
  #endif

  #if AXIS_DRIVER_TYPE_J(L6470)
    #define J_MICROSTEPS      128
    #define J_OVERCURRENT    2000
    #define J_STALLCURRENT   1500
    #define J_MAX_VOLTAGE     127
    #define J_CHAIN_POS        -1
    #define J_SLEW_RATE         1
  #endif

  #if AXIS_DRIVER_TYPE_K(L6470)
    #define K_MICROSTEPS      128
    #define K_OVERCURRENT    2000
    #define K_STALLCURRENT   1500
    #define K_MAX_VOLTAGE     127
    #define K_CHAIN_POS        -1
    #define K_SLEW_RATE         1
  #endif

  #if AXIS_IS_L64XX(E0)
    #define E0_MICROSTEPS              128
    #define E0_OVERCURRENT            2000
    #define E0_STALLCURRENT           1500
    #define E0_MAX_VOLTAGE             127
    #define E0_CHAIN_POS                -1
    #define E0_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E1)
    #define E1_MICROSTEPS    E0_MICROSTEPS
    #define E1_OVERCURRENT            2000
    #define E1_STALLCURRENT           1500
    #define E1_MAX_VOLTAGE             127
    #define E1_CHAIN_POS                -1
    #define E1_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E2)
    #define E2_MICROSTEPS    E0_MICROSTEPS
    #define E2_OVERCURRENT            2000
    #define E2_STALLCURRENT           1500
    #define E2_MAX_VOLTAGE             127
    #define E2_CHAIN_POS                -1
    #define E2_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E3)
    #define E3_MICROSTEPS    E0_MICROSTEPS
    #define E3_OVERCURRENT            2000
    #define E3_STALLCURRENT           1500
    #define E3_MAX_VOLTAGE             127
    #define E3_CHAIN_POS                -1
    #define E3_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E4)
    #define E4_MICROSTEPS    E0_MICROSTEPS
    #define E4_OVERCURRENT            2000
    #define E4_STALLCURRENT           1500
    #define E4_MAX_VOLTAGE             127
    #define E4_CHAIN_POS                -1
    #define E4_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E5)
    #define E5_MICROSTEPS    E0_MICROSTEPS
    #define E5_OVERCURRENT            2000
    #define E5_STALLCURRENT           1500
    #define E5_MAX_VOLTAGE             127
    #define E5_CHAIN_POS                -1
    #define E5_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E6)
    #define E6_MICROSTEPS    E0_MICROSTEPS
    #define E6_OVERCURRENT            2000
    #define E6_STALLCURRENT           1500
    #define E6_MAX_VOLTAGE             127
    #define E6_CHAIN_POS                -1
    #define E6_SLEW_RATE                 1
  #endif

  #if AXIS_IS_L64XX(E7)
    #define E7_MICROSTEPS    E0_MICROSTEPS
    #define E7_OVERCURRENT            2000
    #define E7_STALLCURRENT           1500
    #define E7_MAX_VOLTAGE             127
    #define E7_CHAIN_POS                -1
    #define E7_SLEW_RATE                 1
  #endif

  /**
   * Monitor L6470 drivers for error conditions like over temperature and over current.
   * In the case of over temperature Marlin can decrease the drive until the error condition clears.
   * Other detected conditions can be used to stop the current print.
   * Relevant G-codes:
   * M906 - I1/2/3/4/5  Set or get motor drive level using axis codes X, Y, Z, E. Report values if no axis codes given.
   *         I not present or I0 or I1 - X, Y, Z or E0
   *         I2 - X2, Y2, Z2 or E1
   *         I3 - Z3 or E3
   *         I4 - Z4 or E4
   *         I5 - E5
   * M916 - Increase drive level until get thermal warning
   * M917 - Find minimum current thresholds
   * M918 - Increase speed until max or error
   * M122 S0/1 - Report driver parameters
   */
  //#define MONITOR_L6470_DRIVER_STATUS//#定义监视器\u L6470\u驱动器\u状态

  #if ENABLED(MONITOR_L6470_DRIVER_STATUS)
    #define KVAL_HOLD_STEP_DOWN     1
    //#define L6470_STOP_ON_ERROR//#定义L6470\u错误时停止\u
  #endif

#endif // HAS_L64XX//有"L64XX"吗?

// @section i2cbus//@第I2C节总线

////
// I2C Master ID for LPC176x LCD and Digital Current control//用于LPC176x LCD和数字电流控制的I2C主ID
// Does not apply to other peripherals based on the Wire library.//不适用于基于Wire库的其他外围设备。
////
//#define I2C_MASTER_ID  1  // Set a value from 0 to 2//#定义I2C_MASTER_ID 1//将值设置为0到2

/**
 * TWI/I2C BUS
 *
 * This feature is an EXPERIMENTAL feature so it shall not be used on production
 * machines. Enabling this will allow you to send and receive I2C data from slave
 * devices on the bus.
 *
 * ; Example #1
 * ; This macro send the string "Marlin" to the slave device with address 0x63 (99)
 * ; It uses multiple M260 commands with one B<base 10> arg
 * M260 A99  ; Target slave address
 * M260 B77  ; M
 * M260 B97  ; a
 * M260 B114 ; r
 * M260 B108 ; l
 * M260 B105 ; i
 * M260 B110 ; n
 * M260 S1   ; Send the current buffer
 *
 * ; Example #2
 * ; Request 6 bytes from slave device with address 0x63 (99)
 * M261 A99 B5
 *
 * ; Example #3
 * ; Example serial output of a M261 request
 * echo:i2c-reply: from:99 bytes:5 data:hello
 */

//#define EXPERIMENTAL_I2CBUS//#定义实验总线
#if ENABLED(EXPERIMENTAL_I2CBUS)
#define I2C_SLAVE_ADDRESS  0  // Set a value from 8 to 127 to act as a slave//将值设置为8到127，以用作从属
#endif

// @section extras//@额外部分

/**
 * Photo G-code
 * Add the M240 G-code to take a photo.
 * The photo can be triggered by a digital pin or a physical movement.
 */
//#define PHOTO_GCODE//#定义照片编码
#if ENABLED(PHOTO_GCODE)
// A position to move to (and raise Z) before taking the photo//拍照前移动（并升高Z）的位置
  //#define PHOTO_POSITION { X_MAX_POS - 5, Y_MAX_POS, 0 }  // { xpos, ypos, zraise } (M240 X Y Z)//#定义照片位置{X_MAX_POS-5，Y_MAX_POS，0}/{xpos，ypos，zraise}（M240 X Y Z）
  //#define PHOTO_DELAY_MS   100                            // (ms) Duration to pause before moving back (M240 P)//#定义后退前暂停的照片延迟时间（M240 P）
  //#define PHOTO_RETRACT_MM   6.5                          // (mm) E retract/recover for the photo move (M240 R S)//#为照片移动定义照片缩回毫米6.5/（毫米）E缩回/恢复（M240 R S）

  // Canon RC-1 or homebrew digital camera trigger//佳能RC-1或自制数码相机触发器
  // Data from: https://www.doc-diy.net/photo/rc-1_hacked///数据来源：https://www.doc-diy.net/photo/rc-1_hacked/
  //#define PHOTOGRAPH_PIN 23//#第23针

  // Canon Hack Development Kit//佳能黑客开发工具包
  // https://captain-slow.dk/2014/03/09/3d-printing-timelapses/// https://captain-slow.dk/2014/03/09/3d-printing-timelapses/
  //#define CHDK_PIN        4//#定义CHDK_引脚4

  // Optional second move with delay to trigger the camera shutter//可选第二次移动，延迟触发相机快门
  //#define PHOTO_SWITCH_POSITION { X_MAX_POS, Y_MAX_POS }  // { xpos, ypos } (M240 I J)//#定义光开关位置{X_MAX_POS，Y_MAX_POS}/{xpos，ypos}（M240 I J）

  // Duration to hold the switch or keep CHDK_PIN high//保持开关或保持CHDK_引脚高的持续时间
  //#define PHOTO_SWITCH_MS   50 // (ms) (M240 D)//#定义照片开关\u MS 50/（MS）（M240 D）

  /**
   * PHOTO_PULSES_US may need adjustment depending on board and camera model.
   * Pin must be running at 48.4kHz.
   * Be sure to use a PHOTOGRAPH_PIN which can rise and fall quick enough.
   * (e.g., MKS SBase temp sensor pin was too slow, so used P1.23 on J8.)
   *
   *  Example pulse data for Nikon: https://bit.ly/2FKD0Aq
   *                     IR Wiring: https://git.io/JvJf7
   */
  //#define PHOTO_PULSES_US { 2000, 27850, 400, 1580, 400, 3580, 400 }  // (µs) Durations for each 48.4kHz oscillation//#定义每个48.4kHz振荡的光脉冲持续时间
  #ifdef PHOTO_PULSES_US
    #define PHOTO_PULSE_DELAY_US 13 // (µs) Approximate duration of each HIGH and LOW pulse in the oscillation//（µs）振荡中每个高脉冲和低脉冲的近似持续时间
  #endif
#endif

/**
 * Spindle & Laser control
 *
 * Add the M3, M4, and M5 commands to turn the spindle/laser on and off, and
 * to set spindle speed, spindle direction, and laser power.
 *
 * SuperPid is a router/spindle speed controller used in the CNC milling community.
 * Marlin can be used to turn the spindle on and off. It can also be used to set
 * the spindle speed from 5,000 to 30,000 RPM.
 *
 * You'll need to select a pin for the ON/OFF function and optionally choose a 0-5V
 * hardware PWM pin for the speed control and a pin for the rotation direction.
 *
 * See https://marlinfw.org/docs/configuration/laser_spindle.html for more config details.
 */
//#define SPINDLE_FEATURE//#定义主轴回转特性
//#define LASER_FEATURE//#定义激光功能
#if EITHER(SPINDLE_FEATURE, LASER_FEATURE)
#define SPINDLE_LASER_ACTIVE_STATE    LOW    // Set to "HIGH" if SPINDLE_LASER_ENA_PIN is active HIGH//如果主轴激光器ENA引脚处于高激活状态，则设置为“高”

  #define SPINDLE_LASER_USE_PWM                // Enable if your controller supports setting the speed/power//如果控制器支持设置速度/功率，则启用
  #if ENABLED(SPINDLE_LASER_USE_PWM)
    #define SPINDLE_LASER_PWM_INVERT    false  // Set to "true" if the speed/power goes up when you want it to go slower//如果您希望速度/功率变慢时速度/功率升高，则将其设置为“真”
    #define SPINDLE_LASER_FREQUENCY     2500   // (Hz) Spindle/laser frequency (only on supported HALs: AVR and LPC)//（Hz）主轴/激光频率（仅支持HAL:AVR和LPC）
  #endif

  //#define AIR_EVACUATION                     // Cutter Vacuum / Laser Blower motor control with G-codes M10-M11//#定义排气//切割机真空/激光鼓风机电机控制，G代码为M10-M11
  #if ENABLED(AIR_EVACUATION)
    #define AIR_EVACUATION_ACTIVE       LOW    // Set to "HIGH" if the on/off function is active HIGH//如果接通/断开功能处于激活状态，则设置为“高”
    //#define AIR_EVACUATION_PIN        42     // Override the default Cutter Vacuum or Laser Blower pin//#定义排气管脚42//覆盖默认的切割机真空管脚或激光鼓风机管脚
  #endif

  //#define AIR_ASSIST                         // Air Assist control with G-codes M8-M9//#使用G代码M8-M9定义空气辅助//空气辅助控制
  #if ENABLED(AIR_ASSIST)
    #define AIR_ASSIST_ACTIVE           LOW    // Active state on air assist pin//空气辅助销处于激活状态
    //#define AIR_ASSIST_PIN            44     // Override the default Air Assist pin//#定义空气辅助引脚44//覆盖默认空气辅助引脚
  #endif

  //#define SPINDLE_SERVO                      // A servo converting an angle to spindle power//#定义主轴伺服//将角度转换为主轴功率的伺服
  #ifdef SPINDLE_SERVO
    #define SPINDLE_SERVO_NR   0               // Index of servo used for spindle control//用于主轴控制的伺服系统指标
    #define SPINDLE_SERVO_MIN 10               // Minimum angle for servo spindle//伺服主轴的最小角度
  #endif

  /**
   * Speed / Power can be set ('M3 S') and displayed in terms of:
   *  - PWM255  (S0 - S255)
   *  - PERCENT (S0 - S100)
   *  - RPM     (S0 - S50000)  Best for use with a spindle
   *  - SERVO   (S0 - S180)
   */
  #define CUTTER_POWER_UNIT PWM255

  /**
   * Relative Cutter Power
   * Normally, 'M3 O<power>' sets
   * OCR power is relative to the range SPEED_POWER_MIN...SPEED_POWER_MAX.
   * so input powers of 0...255 correspond to SPEED_POWER_MIN...SPEED_POWER_MAX
   * instead of normal range (0 to SPEED_POWER_MAX).
   * Best used with (e.g.) SuperPID router controller: S0 = 5,000 RPM and S255 = 30,000 RPM
   */
  //#define CUTTER_POWER_RELATIVE              // Set speed proportional to [SPEED_POWER_MIN...SPEED_POWER_MAX]//#定义刀具功率相对//设置与[速度功率最小值…速度功率最大值]成比例的速度

  #if ENABLED(SPINDLE_FEATURE)
    //#define SPINDLE_CHANGE_DIR               // Enable if your spindle controller can change spindle direction//#定义主轴\u更改\u方向//如果主轴控制器可以更改主轴方向，则启用
    #define SPINDLE_CHANGE_DIR_STOP            // Enable if the spindle should stop before changing spin direction//如果主轴应在改变旋转方向前停止，则启用
    #define SPINDLE_INVERT_DIR          false  // Set to "true" if the spin direction is reversed//如果旋转方向相反，则设置为“真”

    #define SPINDLE_LASER_POWERUP_DELAY   5000 // (ms) Delay to allow the spindle/laser to come up to speed/power//（ms）允许主轴/激光器达到速度/功率的延迟
    #define SPINDLE_LASER_POWERDOWN_DELAY 5000 // (ms) Delay to allow the spindle to stop//（ms）允许主轴停止的延迟

    /**
     * M3/M4 Power Equation
     *
     * Each tool uses different value ranges for speed / power control.
     * These parameters are used to convert between tool power units and PWM.
     *
     * Speed/Power = (PWMDC / 255 * 100 - SPEED_POWER_INTERCEPT) / SPEED_POWER_SLOPE
     * PWMDC = (spdpwr - SPEED_POWER_MIN) / (SPEED_POWER_MAX - SPEED_POWER_MIN) / SPEED_POWER_SLOPE
     */
    #if ENABLED(SPINDLE_LASER_USE_PWM)
      #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage//（%）0-100，即最小功率百分比
      #define SPEED_POWER_MIN          5000    // (RPM)//（转/分）
      #define SPEED_POWER_MAX         30000    // (RPM) SuperPID router controller 0 - 30,000 RPM//（RPM）超级PID路由器控制器0-30000 RPM
      #define SPEED_POWER_STARTUP     25000    // (RPM) M3/M4 speed/power default (with no arguments)//（RPM）M3/M4速度/功率默认值（无参数）
    #endif

  #else

    #if ENABLED(SPINDLE_LASER_USE_PWM)
      #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage//（%）0-100，即最小功率百分比
      #define SPEED_POWER_MIN             0    // (%) 0-100// (%) 0-100
      #define SPEED_POWER_MAX           100    // (%) 0-100// (%) 0-100
      #define SPEED_POWER_STARTUP        80    // (%) M3/M4 speed/power default (with no arguments)//（%）M3/M4速度/功率默认值（无参数）
    #endif

    // Define the minimum and maximum test pulse time values for a laser test fire function//定义激光测试点火功能的最小和最大测试脉冲时间值
    #define LASER_TEST_PULSE_MIN           1   // Used with Laser Control Menu//与激光控制菜单一起使用
    #define LASER_TEST_PULSE_MAX         999   // Caution: Menu may not show more than 3 characters//注意：菜单显示的字符不能超过3个

    /**
     * Enable inline laser power to be handled in the planner / stepper routines.
     * Inline power is specified by the I (inline) flag in an M3 command (e.g., M3 S20 I)
     * or by the 'S' parameter in G0/G1/G2/G3 moves (see LASER_MOVE_POWER).
     *
     * This allows the laser to keep in perfect sync with the planner and removes
     * the powerup/down delay since lasers require negligible time.
     */
    //#define LASER_POWER_INLINE//#定义激光功率

    #if ENABLED(LASER_POWER_INLINE)
      /**
       * Scale the laser's power in proportion to the movement rate.
       *
       * - Sets the entry power proportional to the entry speed over the nominal speed.
       * - Ramps the power up every N steps to approximate the speed trapezoid.
       * - Due to the limited power resolution this is only approximate.
       */
      #define LASER_POWER_INLINE_TRAPEZOID

      /**
       * Continuously calculate the current power (nominal_power * current_rate / nominal_rate).
       * Required for accurate power with non-trapezoidal acceleration (e.g., S_CURVE_ACCELERATION).
       * This is a costly calculation so this option is discouraged on 8-bit AVR boards.
       *
       * LASER_POWER_INLINE_TRAPEZOID_CONT_PER defines how many step cycles there are between power updates. If your
       * board isn't able to generate steps fast enough (and you are using LASER_POWER_INLINE_TRAPEZOID_CONT), increase this.
       * Note that when this is zero it means it occurs every cycle; 1 means a delay wait one cycle then run, etc.
       */
      //#define LASER_POWER_INLINE_TRAPEZOID_CONT//#定义激光功率内联梯形控制

      /**
       * Stepper iterations between power updates. Increase this value if the board
       * can't keep up with the processing demands of LASER_POWER_INLINE_TRAPEZOID_CONT.
       * Disable (or set to 0) to recalculate power on every stepper iteration.
       */
      //#define LASER_POWER_INLINE_TRAPEZOID_CONT_PER 10//#根据10定义激光功率内联梯形控制

      /**
       * Include laser power in G0/G1/G2/G3/G5 commands with the 'S' parameter
       */
      //#define LASER_MOVE_POWER//#定义激光移动功率

      #if ENABLED(LASER_MOVE_POWER)
        // Turn off the laser on G0 moves with no power parameter.//在无电源参数的G0移动时关闭激光器。
        // If a power parameter is provided, use that instead.//如果提供了电源参数，请改用该参数。
        //#define LASER_MOVE_G0_OFF//#定义激光器\u移动\u G0\u关闭

        // Turn off the laser on G28 homing.//关闭G28归零上的激光器。
        //#define LASER_MOVE_G28_OFF//#定义激光器\u移动\u G28\u关闭
      #endif

      /**
       * Inline flag inverted
       *
       * WARNING: M5 will NOT turn off the laser unless another move
       *          is done (so G-code files must end with 'M5 I').
       */
      //#define LASER_POWER_INLINE_INVERT//#定义激光功率内联反转

      /**
       * Continuously apply inline power. ('M3 S3' == 'G1 S3' == 'M3 S3 I')
       *
       * The laser might do some weird things, so only enable this
       * feature if you understand the implications.
       */
      //#define LASER_POWER_INLINE_CONTINUOUS//#定义激光功率在线连续

    #else

      #define SPINDLE_LASER_POWERUP_DELAY     50 // (ms) Delay to allow the spindle/laser to come up to speed/power//（ms）允许主轴/激光器达到速度/功率的延迟
      #define SPINDLE_LASER_POWERDOWN_DELAY   50 // (ms) Delay to allow the spindle to stop//（ms）允许主轴停止的延迟

    #endif

    ////
    // Laser I2C Ammeter (High precision INA226 low/high side module)//激光I2C安培计（高精度INA226低端/高端模块）
    ////
    //#define I2C_AMMETER//#定义I2C_安培计
    #if ENABLED(I2C_AMMETER)
      #define I2C_AMMETER_IMAX            0.1    // (Amps) Calibration value for the expected current range//预期电流范围的（安培）校准值
      #define I2C_AMMETER_SHUNT_RESISTOR  0.1    // (Ohms) Calibration shunt resistor value//（欧姆）校准分流电阻器值
    #endif

  #endif
#endif // SPINDLE_FEATURE || LASER_FEATURE//主轴| | |激光|功能

/**
 * Synchronous Laser Control with M106/M107
 *
 * Marlin normally applies M106/M107 fan speeds at a time "soon after" processing
 * a planner block. This is too inaccurate for a PWM/TTL laser attached to the fan
 * header (as with some add-on laser kits). Enable this option to set fan/laser
 * speeds with much more exact timing for improved print fidelity.
 *
 * NOTE: This option sacrifices some cooling fan speed options.
 */
//#define LASER_SYNCHRONOUS_M106_M107//#定义激光器\u同步\u M106\u M107

/**
 * Coolant Control
 *
 * Add the M7, M8, and M9 commands to turn mist or flood coolant on and off.
 *
 * Note: COOLANT_MIST_PIN and/or COOLANT_FLOOD_PIN must also be defined.
 */
//#define COOLANT_CONTROL//#定义冷却液温度控制
#if ENABLED(COOLANT_CONTROL)
#define COOLANT_MIST                // Enable if mist coolant is present//如果存在水雾冷却液，则启用
  #define COOLANT_FLOOD               // Enable if flood coolant is present//如果存在溢流冷却液，则启用
  #define COOLANT_MIST_INVERT  false  // Set "true" if the on/off function is reversed//如果接通/断开功能反转，则设置“真”
  #define COOLANT_FLOOD_INVERT false  // Set "true" if the on/off function is reversed//如果接通/断开功能反转，则设置“真”
#endif

/**
 * Filament Width Sensor
 *
 * Measures the filament width in real-time and adjusts
 * flow rate to compensate for any irregularities.
 *
 * Also allows the measured filament diameter to set the
 * extrusion rate, so the slicer only has to specify the
 * volume.
 *
 * Only a single extruder is supported at this time.
 *
 *  34 RAMPS_14    : Analog input 5 on the AUX2 connector
 *  81 PRINTRBOARD : Analog input 2 on the Exp1 connector (version B,C,D,E)
 * 301 RAMBO       : Analog input 3
 *
 * Note: May require analog pins to be defined for other boards.
 */
//#define FILAMENT_WIDTH_SENSOR//#定义灯丝宽度传感器

#if ENABLED(FILAMENT_WIDTH_SENSOR)
#define FILAMENT_SENSOR_EXTRUDER_NUM 0    // Index of the extruder that has the filament sensor. :[0,1,2,3,4]//具有灯丝传感器的挤出机的索引：[0,1,2,3,4]
  #define MEASUREMENT_DELAY_CM        14    // (cm) The distance from the filament sensor to the melting chamber//（cm）从灯丝传感器到熔化室的距离

  #define FILWIDTH_ERROR_MARGIN        1.0  // (mm) If a measurement differs too much from nominal width ignore it//（mm）如果测量值与标称宽度相差过大，则忽略它
  #define MAX_MEASUREMENT_DELAY       20    // (bytes) Buffer size for stored measurements (1 byte per cm). Must be larger than MEASUREMENT_DELAY_CM.//（字节）存储测量的缓冲区大小（每厘米1字节）。必须大于测量值\u延迟\u CM。

  #define DEFAULT_MEASURED_FILAMENT_DIA DEFAULT_NOMINAL_FILAMENT_DIA // Set measured to nominal initially//最初将测量值设置为标称值

  // Display filament width on the LCD status line. Status messages will expire after 5 seconds.//在LCD状态行上显示灯丝宽度。状态消息将在5秒后过期。
  //#define FILAMENT_LCD_DISPLAY//#定义灯丝\u LCD\u显示器
#endif

/**
 * Power Monitor
 * Monitor voltage (V) and/or current (A), and -when possible- power (W)
 *
 * Read and configure with M430
 *
 * The current sensor feeds DC voltage (relative to the measured current) to an analog pin
 * The voltage sensor feeds DC voltage (relative to the measured voltage) to an analog pin
 */
//#define POWER_MONITOR_CURRENT   // Monitor the system current//#定义电源\u监视器\u电流//监视系统电流
//#define POWER_MONITOR_VOLTAGE   // Monitor the system voltage//#定义电源\监视器\电压//监视系统电压

#if ENABLED(POWER_MONITOR_CURRENT)
#define POWER_MONITOR_VOLTS_PER_AMP    0.05000  // Input voltage to the MCU analog pin per amp  - DO NOT apply more than ADC_VREF!//每安培MCU模拟引脚的输入电压-施加的电压不得超过ADC_VREF！
  #define POWER_MONITOR_CURRENT_OFFSET   0        // Offset (in amps) applied to the calculated current//应用于计算电流的偏移量（单位：安培）
  #define POWER_MONITOR_FIXED_VOLTAGE   13.6      // Voltage for a current sensor with no voltage sensor (for power display)//无电压传感器的电流传感器的电压（用于电源显示）
#endif

#if ENABLED(POWER_MONITOR_VOLTAGE)
#define POWER_MONITOR_VOLTS_PER_VOLT  0.077933  // Input voltage to the MCU analog pin per volt - DO NOT apply more than ADC_VREF!//每伏MCU模拟引脚的输入电压-施加的电压不得超过ADC_VREF！
  #define POWER_MONITOR_VOLTAGE_OFFSET  0         // Offset (in volts) applied to the calculated voltage//应用于计算电压的偏移量（单位：伏特）
#endif

/**
 * Stepper Driver Anti-SNAFU Protection
 *
 * If the SAFE_POWER_PIN is defined for your board, Marlin will check
 * that stepper drivers are properly plugged in before applying power.
 * Disable protection if your stepper drivers don't support the feature.
 */
//#define DISABLE_DRIVER_SAFE_POWER_PROTECT//#定义禁用\u驱动器\u安全\u电源\u保护

/**
 * CNC Coordinate Systems
 *
 * Enables G53 and G54-G59.3 commands to select coordinate systems
 * and G92.1 to reset the workspace to native machine space.
 */
//#define CNC_COORDINATE_SYSTEMS//#定义CNC坐标系

/**
 * Auto-report temperatures with M155 S<seconds>
 */
#define AUTO_REPORT_TEMPERATURES

/**
 * Auto-report position with M154 S<seconds>
 */
//#define AUTO_REPORT_POSITION//#定义自动报告位置

/**
 * Include capabilities in M115 output
 */
#define EXTENDED_CAPABILITIES_REPORT
#if ENABLED(EXTENDED_CAPABILITIES_REPORT)
//#define M115_GEOMETRY_REPORT//#定义M115_几何体_报告
#endif

/**
 * Expected Printer Check
 * Add the M16 G-code to compare a string to the MACHINE_NAME.
 * M16 with a non-matching string causes the printer to halt.
 */
//#define EXPECTED_PRINTER_CHECK//#定义预期的\u打印机\u检查

/**
 * Disable all Volumetric extrusion options
 */
//#define NO_VOLUMETRICS//#定义无体积度量

#if DISABLED(NO_VOLUMETRICS)
/**
   * Volumetric extrusion default state
   * Activate to make volumetric extrusion the default method,
   * with DEFAULT_NOMINAL_FILAMENT_DIA as the default diameter.
   *
   * M200 D0 to disable, M200 Dn to set a new diameter (and enable volumetric).
   * M200 S0/S1 to disable/enable volumetric extrusion.
   */
  //#define VOLUMETRIC_DEFAULT_ON//#定义体积默认值

  //#define VOLUMETRIC_EXTRUDER_LIMIT//#定义挤出机的容积限制
  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
    /**
     * Default volumetric extrusion limit in cubic mm per second (mm^3/sec).
     * This factory setting applies to all extruders.
     * Use 'M200 [T<extruder>] L<limit>' to override and 'M502' to reset.
     * A non-zero value activates Volume-based Extrusion Limiting.
     */
    #define DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT 0.00      // (mm^3/sec)//（毫米^3/秒）
  #endif
#endif

/**
 * Enable this option for a leaner build of Marlin that removes all
 * workspace offsets, simplifying coordinate transformations, leveling, etc.
 *
 *  - M206 and M428 are disabled.
 *  - G92 will revert to its behavior from Marlin 1.0.
 */
//#define NO_WORKSPACE_OFFSETS//#定义无工作空间偏移

// Extra options for the M114 "Current Position" report//M114“当前位置”报告的额外选项
//#define M114_DETAIL         // Use 'M114` for details to check planner calculations//#定义M114_详细信息//使用“M114”查看详细信息，以检查计划者的计算
//#define M114_REALTIME       // Real current position based on forward kinematics//#基于正向运动学定义M114_REALTIME//Real current position
//#define M114_LEGACY         // M114 used to synchronize on every call. Enable if needed.//#定义M114_LEGACY//M114，用于在每次调用时进行同步。如果需要，启用。

//#define REPORT_FAN_CHANGE   // Report the new fan speed when changed by M106 (and others)//#定义报告风机改变//报告M106（和其他）改变时的新风机速度

/**
 * Set the number of proportional font spaces required to fill up a typical character space.
 * This can help to better align the output of commands like `G29 O` Mesh Output.
 *
 * For clients that use a fixed-width font (like OctoPrint), leave this set to 1.0.
 * Otherwise, adjust according to your client and font.
 */
#define PROPORTIONAL_FONT_RATIO 1.0

/**
 * Spend 28 bytes of SRAM to optimize the G-code parser
 */
#define FASTER_GCODE_PARSER

#if ENABLED(FASTER_GCODE_PARSER)
//#define GCODE_QUOTED_STRINGS  // Support for quoted string parameters//#定义GCODE_QUOTED_string//对QUOTED string参数的支持
#endif

// Support for MeatPack G-code compression (https://github.com/scottmudge/OctoPrint-MeatPack)//支持MeatPack G代码压缩(https://github.com/scottmudge/OctoPrint-MeatPack)
//#define MEATPACK_ON_SERIAL_PORT_1//#在串行端口1上定义MEATPACK
//#define MEATPACK_ON_SERIAL_PORT_2//#在串行端口2上定义MEATPACK

//#define GCODE_CASE_INSENSITIVE  // Accept G-code sent to the firmware in lowercase//#定义GCODE_不区分大小写//接受以小写形式发送到固件的G代码

//#define REPETIER_GCODE_M360     // Add commands originally from Repetier FW//#定义REPETIER_GCODE_M360//添加最初来自REPETIER FW的命令

/**
 * CNC G-code options
 * Support CNC-style G-code dialects used by laser cutters, drawing machine cams, etc.
 * Note that G0 feedrates should be used with care for 3D printing (if used at all).
 * High feedrates may cause ringing and harm print quality.
 */
//#define PAREN_COMMENTS      // Support for parentheses-delimited comments//#定义PAREN_注释//支持括号分隔的注释
//#define GCODE_MOTION_MODES  // Remember the motion mode (G0 G1 G2 G3 G5 G38.X) and apply for X Y Z E F, etc.//#定义GCODE_MOTION_MODES//记住运动模式（G0 G1 G2 G3 G5 G38.X）并申请X Y Z E F等。

// Enable and set a (default) feedrate for all G0 moves//为所有G0移动启用并设置（默认）进给速度
//#define G0_FEEDRATE 3000 // (mm/min)//#定义G0_进给速度3000/（mm/min）
#ifdef G0_FEEDRATE
//#define VARIABLE_G0_FEEDRATE // The G0 feedrate is set by F in G0 motion mode//#定义变量_G0_进给速度//G0进给速度由F在G0运动模式下设置
#endif

/**
 * Startup commands
 *
 * Execute certain G-code commands immediately after power-on.
 */
//#define STARTUP_COMMANDS "M17 Z"//#定义启动命令“M17 Z”

/**
 * G-code Macros
 *
 * Add G-codes M810-M819 to define and run G-code macros.
 * Macros are not saved to EEPROM.
 */
//#define GCODE_MACROS//#定义GCODE_宏
#if ENABLED(GCODE_MACROS)
#define GCODE_MACROS_SLOTS       5  // Up to 10 may be used//最多可使用10个
  #define GCODE_MACROS_SLOT_SIZE  50  // Maximum length of a single macro//单个宏的最大长度
#endif

/**
 * User-defined menu items to run custom G-code.
 * Up to 25 may be defined, but the actual number is LCD-dependent.
 */

// Custom Menu: Main Menu//自定义菜单：主菜单
//#define CUSTOM_MENU_MAIN//#定义自定义菜单主菜单
#if ENABLED(CUSTOM_MENU_MAIN)
//#define CUSTOM_MENU_MAIN_TITLE "Custom Commands"//#定义自定义菜单主标题“自定义命令”
  #define CUSTOM_MENU_MAIN_SCRIPT_DONE "M117 User Script Done"
  #define CUSTOM_MENU_MAIN_SCRIPT_AUDIBLE_FEEDBACK
  //#define CUSTOM_MENU_MAIN_SCRIPT_RETURN   // Return to status screen after a script//#定义自定义菜单主脚本返回//返回脚本后的状态屏幕
  #define CUSTOM_MENU_MAIN_ONLY_IDLE         // Only show custom menu when the machine is idle//仅在机器空闲时显示自定义菜单

  #define MAIN_MENU_ITEM_1_DESC "Home & UBL Info"
  #define MAIN_MENU_ITEM_1_GCODE "G28\nG29 W"
  //#define MAIN_MENU_ITEM_1_CONFIRM          // Show a confirmation dialog before this action//#定义主菜单项确认//在此操作之前显示确认对话框

  #define MAIN_MENU_ITEM_2_DESC "Preheat for " PREHEAT_1_LABEL
  #define MAIN_MENU_ITEM_2_GCODE "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
  //#define MAIN_MENU_ITEM_2_CONFIRM//#定义主菜单项2确认

  //#define MAIN_MENU_ITEM_3_DESC "Preheat for " PREHEAT_2_LABEL//#定义主菜单项描述“预热为”预热标签
  //#define MAIN_MENU_ITEM_3_GCODE "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)//#定义主菜单项代码“M140 S”字符串化（预热2临时床）“\nM104 S”字符串化（预热2临时热端）
  //#define MAIN_MENU_ITEM_3_CONFIRM//#定义主菜单项3确认

  //#define MAIN_MENU_ITEM_4_DESC "Heat Bed/Home/Level"//#定义主菜单项描述“热床/主/标高”
  //#define MAIN_MENU_ITEM_4_GCODE "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nG28\nG29"//#定义主菜单项代码“M140 S”字符串化（预热2临时床）“\nG28\nG29”
  //#define MAIN_MENU_ITEM_4_CONFIRM//#定义主菜单项4确认

  //#define MAIN_MENU_ITEM_5_DESC "Home & Info"//#定义主菜单项描述“主页和信息”
  //#define MAIN_MENU_ITEM_5_GCODE "G28\nM503"//#定义主菜单项代码“G28\nM503”
  //#define MAIN_MENU_ITEM_5_CONFIRM//#定义主菜单项确认
#endif

// Custom Menu: Configuration Menu//自定义菜单：配置菜单
//#define CUSTOM_MENU_CONFIG//#定义自定义菜单配置
#if ENABLED(CUSTOM_MENU_CONFIG)
//#define CUSTOM_MENU_CONFIG_TITLE "Custom Commands"//#定义自定义菜单配置标题“自定义命令”
  #define CUSTOM_MENU_CONFIG_SCRIPT_DONE "M117 Wireless Script Done"
  #define CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK
  //#define CUSTOM_MENU_CONFIG_SCRIPT_RETURN  // Return to status screen after a script//#定义自定义菜单配置脚本返回//返回脚本后的状态屏幕
  #define CUSTOM_MENU_CONFIG_ONLY_IDLE        // Only show custom menu when the machine is idle//仅在机器空闲时显示自定义菜单

  #define CONFIG_MENU_ITEM_1_DESC "Wifi ON"
  #define CONFIG_MENU_ITEM_1_GCODE "M118 [ESP110] WIFI-STA pwd=12345678"
  //#define CONFIG_MENU_ITEM_1_CONFIRM        // Show a confirmation dialog before this action//#定义配置菜单项确认//在此操作之前显示确认对话框

  #define CONFIG_MENU_ITEM_2_DESC "Bluetooth ON"
  #define CONFIG_MENU_ITEM_2_GCODE "M118 [ESP110] BT pwd=12345678"
  //#define CONFIG_MENU_ITEM_2_CONFIRM//#定义配置菜单项2确认

  //#define CONFIG_MENU_ITEM_3_DESC "Radio OFF"//#定义配置菜单项描述“收音机关闭”
  //#define CONFIG_MENU_ITEM_3_GCODE "M118 [ESP110] OFF pwd=12345678"//#定义配置菜单项代码“M118[ESP110]关闭pwd=12345678”
  //#define CONFIG_MENU_ITEM_3_CONFIRM//#定义配置菜单项3确认

  //#define CONFIG_MENU_ITEM_4_DESC "Wifi ????"//#定义配置菜单项描述“Wifi？？”
  //#define CONFIG_MENU_ITEM_4_GCODE "M118 ????"//#定义配置菜单项4代码“M118？？”
  //#define CONFIG_MENU_ITEM_4_CONFIRM//#定义配置菜单项4确认

  //#define CONFIG_MENU_ITEM_5_DESC "Wifi ????"//#定义配置菜单项描述“Wifi？？”
  //#define CONFIG_MENU_ITEM_5_GCODE "M118 ????"//#定义配置菜单项5代码“M118？？”
  //#define CONFIG_MENU_ITEM_5_CONFIRM//#定义配置菜单项确认
#endif

/**
 * User-defined buttons to run custom G-code.
 * Up to 25 may be defined.
 */
//#define CUSTOM_USER_BUTTONS//#定义自定义用户按钮
#if ENABLED(CUSTOM_USER_BUTTONS)
//#define BUTTON1_PIN -1//#定义按钮1\u引脚-1
  #if PIN_EXISTS(BUTTON1)
    #define BUTTON1_HIT_STATE     LOW       // State of the triggered button. NC=LOW. NO=HIGH.//已触发按钮的状态。NC=低。否=高。
    #define BUTTON1_WHEN_PRINTING false     // Button allowed to trigger during printing?//打印期间是否允许触发按钮？
    #define BUTTON1_GCODE         "G28"
    #define BUTTON1_DESC          "Homing"  // Optional string to set the LCD status//用于设置LCD状态的可选字符串
  #endif

  //#define BUTTON2_PIN -1//#定义按钮2\u引脚-1
  #if PIN_EXISTS(BUTTON2)
    #define BUTTON2_HIT_STATE     LOW
    #define BUTTON2_WHEN_PRINTING false
    #define BUTTON2_GCODE         "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
    #define BUTTON2_DESC          "Preheat for " PREHEAT_1_LABEL
  #endif

  //#define BUTTON3_PIN -1//#定义按钮3\u引脚-1
  #if PIN_EXISTS(BUTTON3)
    #define BUTTON3_HIT_STATE     LOW
    #define BUTTON3_WHEN_PRINTING false
    #define BUTTON3_GCODE         "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)
    #define BUTTON3_DESC          "Preheat for " PREHEAT_2_LABEL
  #endif
#endif

/**
 * Host Action Commands
 *
 * Define host streamer action commands in compliance with the standard.
 *
 * See https://reprap.org/wiki/G-code#Action_commands
 * Common commands ........ poweroff, pause, paused, resume, resumed, cancel
 * G29_RETRY_AND_RECOVER .. probe_rewipe, probe_failed
 *
 * Some features add reason codes to extend these commands.
 *
 * Host Prompt Support enables Marlin to use the host for user prompts so
 * filament runout and other processes can be managed from the host side.
 */
//#define HOST_ACTION_COMMANDS//#定义主机\u操作\u命令
#if ENABLED(HOST_ACTION_COMMANDS)
//#define HOST_PAUSE_M76//#定义主机\u暂停\u M76
  //#define HOST_PROMPT_SUPPORT//#定义主机提示支持
  //#define HOST_START_MENU_ITEM  // Add a menu item that tells the host to start//#定义主机启动菜单项//添加告诉主机启动的菜单项
#endif

/**
 * Cancel Objects
 *
 * Implement M486 to allow Marlin to skip objects
 */
//#define CANCEL_OBJECTS//#定义取消对象
#if ENABLED(CANCEL_OBJECTS)
#define CANCEL_OBJECTS_REPORTING // Emit the current object as a status message//将当前对象作为状态消息发出
#endif

/**
 * I2C position encoders for closed loop control.
 * Developed by Chris Barr at Aus3D.
 *
 * Wiki: https://wiki.aus3d.com.au/Magnetic_Encoder
 * Github: https://github.com/Aus3D/MagneticEncoder
 *
 * Supplier: https://aus3d.com.au/magnetic-encoder-module
 * Alternative Supplier: https://reliabuild3d.com/
 *
 * Reliabuild encoders have been modified to improve reliability.
 */

//#define I2C_POSITION_ENCODERS//#定义I2C_位置_编码器
#if ENABLED(I2C_POSITION_ENCODERS)

#define I2CPE_ENCODER_CNT         1                       // The number of encoders installed; max of 5//安装的编码器数量；最多5个
                                                            // encoders supported currently.//目前支持的编码器。

  #define I2CPE_ENC_1_ADDR          I2CPE_PRESET_ADDR_X     // I2C address of the encoder. 30-200.//编码器的I2C地址。30-200.
  #define I2CPE_ENC_1_AXIS          X_AXIS                  // Axis the encoder module is installed on.  <X|Y|Z|E>_AXIS.//编码器模块安装在轴上<X | Y | Z | E>_轴。
  #define I2CPE_ENC_1_TYPE          I2CPE_ENC_TYPE_LINEAR   // Type of encoder:  I2CPE_ENC_TYPE_LINEAR -or-//编码器类型：I2CPE\u ENC\u类型\u线性-或-
                                                            // I2CPE_ENC_TYPE_ROTARY.//I2CPE_ENC_类型_旋转式。
  #define I2CPE_ENC_1_TICKS_UNIT    2048                    // 1024 for magnetic strips with 2mm poles; 2048 for//磁极为2mm的磁条为1024；2048年
                                                            // 1mm poles. For linear encoders this is ticks / mm,//1毫米杆。对于线性编码器，这是滴答/mm，
                                                            // for rotary encoders this is ticks / revolution.//对于旋转编码器，这是滴答声/旋转。
  //#define I2CPE_ENC_1_TICKS_REV     (16 * 200)            // Only needed for rotary encoders; number of stepper//#定义I2CPE_ENC_1_TICKS_REV（16*200）//仅旋转编码器需要；步进电机数量
                                                            // steps per full revolution (motor steps/rev * microstepping)//每整转步数（电机步数/转*微步数）
  //#define I2CPE_ENC_1_INVERT                              // Invert the direction of axis travel.//#定义I2CPE_ENC_1_反转//反转轴移动方向。
  #define I2CPE_ENC_1_EC_METHOD     I2CPE_ECM_MICROSTEP     // Type of error error correction.//错误更正的类型。
  #define I2CPE_ENC_1_EC_THRESH     0.10                    // Threshold size for error (in mm) above which the//错误的阈值大小（单位：mm），超过该阈值
                                                            // printer will attempt to correct the error; errors//打印机将尝试更正错误；错误
                                                            // smaller than this are ignored to minimize effects of//小于此值的将被忽略，以最小化
                                                            // measurement noise / latency (filter).//测量噪声/延迟（滤波器）。

  #define I2CPE_ENC_2_ADDR          I2CPE_PRESET_ADDR_Y     // Same as above, but for encoder 2.//同上，但适用于编码器2。
  #define I2CPE_ENC_2_AXIS          Y_AXIS
  #define I2CPE_ENC_2_TYPE          I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_ENC_2_TICKS_UNIT    2048
  //#define I2CPE_ENC_2_TICKS_REV   (16 * 200)//#定义I2CPE_ENC_2_TICKS_REV（16*200）
  //#define I2CPE_ENC_2_INVERT//#定义I2CPE\U ENC\U 2\U反转
  #define I2CPE_ENC_2_EC_METHOD     I2CPE_ECM_MICROSTEP
  #define I2CPE_ENC_2_EC_THRESH     0.10

  #define I2CPE_ENC_3_ADDR          I2CPE_PRESET_ADDR_Z     // Encoder 3.  Add additional configuration options//编码器3。添加其他配置选项
  #define I2CPE_ENC_3_AXIS          Z_AXIS                  // as above, or use defaults below.//如上所述，或使用以下默认值。

  #define I2CPE_ENC_4_ADDR          I2CPE_PRESET_ADDR_E     // Encoder 4.//编码器4。
  #define I2CPE_ENC_4_AXIS          E_AXIS

  #define I2CPE_ENC_5_ADDR          34                      // Encoder 5.//编码器5。
  #define I2CPE_ENC_5_AXIS          E_AXIS

  // Default settings for encoders which are enabled, but without settings configured above.//启用但未配置上述设置的编码器的默认设置。
  #define I2CPE_DEF_TYPE            I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_DEF_ENC_TICKS_UNIT  2048
  #define I2CPE_DEF_TICKS_REV       (16 * 200)
  #define I2CPE_DEF_EC_METHOD       I2CPE_ECM_NONE
  #define I2CPE_DEF_EC_THRESH       0.1

  //#define I2CPE_ERR_THRESH_ABORT  100.0                   // Threshold size for error (in mm) error on any given//#定义I2CPE_ERR_THRESH_ABORT 100.0//Threshold size for error（以毫米为单位）在任何给定的
                                                            // axis after which the printer will abort. Comment out to//打印机将中止的轴。评出
                                                            // disable abort behavior.//禁用中止行为。

  #define I2CPE_TIME_TRUSTED        10000                   // After an encoder fault, there must be no further fault//编码器故障后，不得再出现其他故障
                                                            // for this amount of time (in ms) before the encoder//编码器之前的这段时间（毫秒）
                                                            // is trusted again.//再次得到信任。

  /**
   * Position is checked every time a new command is executed from the buffer but during long moves,
   * this setting determines the minimum update time between checks. A value of 100 works well with
   * error rolling average when attempting to correct only for skips and not for vibration.
   */
  #define I2CPE_MIN_UPD_TIME_MS     4                       // (ms) Minimum time between encoder checks.//（ms）编码器检查之间的最短时间。

  // Use a rolling average to identify persistent errors that indicate skips, as opposed to vibration and noise.//使用滚动平均值来识别指示跳跃的持续性错误，而不是振动和噪音。
  #define I2CPE_ERR_ROLLING_AVERAGE

#endif // I2C_POSITION_ENCODERS//I2C_位置_编码器

/**
 * Analog Joystick(s)
 */
//#define JOYSTICK//#定义操纵杆
#if ENABLED(JOYSTICK)
#define JOY_X_PIN    5  // RAMPS: Suggested pin A5  on AUX2//坡道：AUX2上的建议插脚A5
  #define JOY_Y_PIN   10  // RAMPS: Suggested pin A10 on AUX2//坡道：AUX2上的建议插脚A10
  #define JOY_Z_PIN   12  // RAMPS: Suggested pin A12 on AUX2//坡道：AUX2上的建议插脚A12
  #define JOY_EN_PIN  44  // RAMPS: Suggested pin D44 on AUX2//坡道：AUX2上的建议引脚D44

  //#define INVERT_JOY_X  // Enable if X direction is reversed//#定义“反转”\u JOY\u X//如果X方向反转，则启用
  //#define INVERT_JOY_Y  // Enable if Y direction is reversed//#定义反转方向//如果Y方向反转，则启用
  //#define INVERT_JOY_Z  // Enable if Z direction is reversed//#定义反转_JOY _Z//如果Z方向反转则启用

  // Use M119 with JOYSTICK_DEBUG to find reasonable values after connecting://连接后，使用M119和U DEBUG查找合理值：
  #define JOY_X_LIMITS { 5600, 8190-100, 8190+100, 10800 } // min, deadzone start, deadzone end, max//最小值，死区开始，死区结束，最大值
  #define JOY_Y_LIMITS { 5600, 8250-100, 8250+100, 11000 }
  #define JOY_Z_LIMITS { 4800, 8080-100, 8080+100, 11550 }
  //#define JOYSTICK_DEBUG//#定义调试
#endif

/**
 * Mechanical Gantry Calibration
 * Modern replacement for the Prusa TMC_Z_CALIBRATION.
 * Adds capability to work with any adjustable current drivers.
 * Implemented as G34 because M915 is deprecated.
 */
//#define MECHANICAL_GANTRY_CALIBRATION//#定义机械标定
#if ENABLED(MECHANICAL_GANTRY_CALIBRATION)
#define GANTRY_CALIBRATION_CURRENT          600     // Default calibration current in ma//默认校准电流（毫安）
  #define GANTRY_CALIBRATION_EXTRA_HEIGHT      15     // Extra distance in mm past Z_###_POS to move//要移动的Z#####U位置后的额外距离（mm）
  #define GANTRY_CALIBRATION_FEEDRATE         500     // Feedrate for correction move//修正移动的进给速度
  //#define GANTRY_CALIBRATION_TO_MIN                 // Enable to calibrate Z in the MIN direction//#定义机架校准到最小//启用在最小方向校准Z

  //#define GANTRY_CALIBRATION_SAFE_POSITION XY_CENTER // Safe position for nozzle//#定义机架校准安全位置XY中心//喷嘴安全位置
  //#define GANTRY_CALIBRATION_XY_PARK_FEEDRATE 3000  // XY Park Feedrate - MMM//#定义机架校准XY停车进给率3000//XY停车进给率-毫米
  //#define GANTRY_CALIBRATION_COMMANDS_PRE   ""//#定义机架校准命令
  #define GANTRY_CALIBRATION_COMMANDS_POST  "G28"     // G28 highly recommended to ensure an accurate position//G28强烈建议确保位置准确
#endif

/**
 * Instant freeze / unfreeze functionality
 * Specified pin has pullup and connecting to ground will instantly pause motion.
 * Potentially useful for emergency stop that allows being resumed.
 */
//#define FREEZE_FEATURE//#定义冻结功能
#if ENABLED(FREEZE_FEATURE)
//#define FREEZE_PIN 41   // Override the default (KILL) pin here//#定义冻结引脚41//在此处覆盖默认（终止）引脚
#endif

/**
 * MAX7219 Debug Matrix
 *
 * Add support for a low-cost 8x8 LED Matrix based on the Max7219 chip as a realtime status display.
 * Requires 3 signal wires. Some useful debug options are included to demonstrate its usage.
 */
//#define MAX7219_DEBUG//#定义MAX7219_调试
#if ENABLED(MAX7219_DEBUG)
#define MAX7219_CLK_PIN   64
  #define MAX7219_DIN_PIN   57
  #define MAX7219_LOAD_PIN  44

  //#define MAX7219_GCODE          // Add the M7219 G-code to control the LED matrix//#定义MAX7219_GCODE//添加M7219 G代码以控制LED矩阵
  #define MAX7219_INIT_TEST    2   // Test pattern at startup: 0=none, 1=sweep, 2=spiral//启动时的测试模式：0=无，1=扫描，2=螺旋
  #define MAX7219_NUMBER_UNITS 1   // Number of Max7219 units in chain.//链中的Max7219单元数。
  #define MAX7219_ROTATE       0   // Rotate the display clockwise (in multiples of +/- 90°)//顺时针旋转显示器（以+/-90°的倍数）
                                   // connector at:  right=0   bottom=-90  top=90  left=180//接头位置：右=0底部=-90顶部=90左侧=180
  //#define MAX7219_REVERSE_ORDER  // The individual LED matrix units may be in reversed order//#定义MAX7219\u反向顺序//单个LED矩阵单元可以反向顺序
  //#define MAX7219_SIDE_BY_SIDE   // Big chip+matrix boards can be chained side-by-side//#定义MAX7219_SIDE_BY_SIDE//大芯片+矩阵板可以并排链接

  /**
   * Sample debug features
   * If you add more debug displays, be careful to avoid conflicts!
   */
  #define MAX7219_DEBUG_PRINTER_ALIVE    // Blink corner LED of 8x8 matrix to show that the firmware is functioning//8x8矩阵的转角LED闪烁，表示固件正在运行
  #define MAX7219_DEBUG_PLANNER_HEAD  3  // Show the planner queue head position on this and the next LED matrix row//显示此和下一个LED矩阵行上的计划器队列头位置
  #define MAX7219_DEBUG_PLANNER_TAIL  5  // Show the planner queue tail position on this and the next LED matrix row//显示此和下一个LED矩阵行上的计划器队列尾部位置

  #define MAX7219_DEBUG_PLANNER_QUEUE 0  // Show the current planner queue depth on this and the next LED matrix row//显示此和下一个LED矩阵行上的当前计划器队列深度
                                         // If you experience stuttering, reboots, etc. this option can reveal how//如果您遇到口吃、重新启动等问题，此选项可以显示
                                         // tweaks made to the configuration are affecting the printer in real-time.//对配置所做的调整会实时影响打印机。
#endif

/**
 * NanoDLP Sync support
 *
 * Support for Synchronized Z moves when used with NanoDLP. G0/G1 axis moves will
 * output a "Z_move_comp" string to enable synchronization with DLP projector exposure.
 * This feature allows you to use [[WaitForDoneMessage]] instead of M400 commands.
 */
//#define NANODLP_Z_SYNC//#定义NANODLP_Z_同步
#if ENABLED(NANODLP_Z_SYNC)
//#define NANODLP_ALL_AXIS  // Send a "Z_move_comp" report for any axis move (not just Z).//#定义NANODLP_ALL_AXIS//为任何轴移动（不仅仅是Z轴）发送“Z_move_comp”报告。
#endif

/**
 * Ethernet. Use M552 to enable and set the IP address.
 */
#if HAS_ETHERNET
#define MAC_ADDRESS { 0xDE, 0xAD, 0xBE, 0xEF, 0xF0, 0x0D }  // A MAC address unique to your network//网络特有的MAC地址
#endif

/**
 * WiFi Support (Espressif ESP32 WiFi)
 */
//#define WIFISUPPORT         // Marlin embedded WiFi managenent//Marlin嵌入式WiFi管理器
//#define ESP3D_WIFISUPPORT   // ESP3D Library WiFi management (https://github.com/luc-github/ESP3DLib)//#定义ESP3D_WIFISUPPORT//ESP3D库WiFi管理(https://github.com/luc-github/ESP3DLib)

#if EITHER(WIFISUPPORT, ESP3D_WIFISUPPORT)
  #define WEBSUPPORT          // Start a webserver (which may include auto-discovery)//启动Web服务器（可能包括自动发现）
  // #define OTASUPPORT          // Support over-the-air firmware updates//#定义OTA支持//支持无线固件更新
  #define WIFI_CUSTOM_COMMAND // Accept feature config commands (e.g., WiFi ESP3D) from the host//接受主机发出的功能配置命令（如WiFi ESP3D）

  /**
   * To set a default WiFi SSID / Password, create a file called Configuration_Secure.h with
   * the following defines, customized for your network. This specific file is excluded via
   * .gitignore to prevent it from accidentally leaking to the public.
   *
   *   #define WIFI_SSID "WiFi SSID"
   *   #define WIFI_PWD  "WiFi Password"
   */
  #include "Configuration_Secure.h" // External file with WiFi SSID / Password//带有WiFi SSID/密码的外部文件
#endif

/**
 * Průša Multi-Material Unit (MMU)
 * Enable in Configuration.h
 *
 * These devices allow a single stepper driver on the board to drive
 * multi-material feeders with any number of stepper motors.
 */
#if HAS_PRUSA_MMU1
/**
   * This option only allows the multiplexer to switch on tool-change.
   * Additional options to configure custom E moves are pending.
   *
   * Override the default DIO selector pins here, if needed.
   * Some pins files may provide defaults for these pins.
   */
  //#define E_MUX0_PIN 40  // Always Required//#定义E_MUX0_引脚40//始终需要
  //#define E_MUX1_PIN 42  // Needed for 3 to 8 inputs//#定义3到8个输入所需的E_MUX1_引脚42//
  //#define E_MUX2_PIN 44  // Needed for 5 to 8 inputs//#定义5到8个输入所需的E_MUX2_引脚44//
#elif HAS_PRUSA_MMU2
// Serial port used for communication with MMU2.//用于与MMU2通信的串行端口。
  #define MMU2_SERIAL_PORT 2

  // Use hardware reset for MMU if a pin is defined for it//如果定义了MMU的引脚，则使用MMU的硬件重置
  //#define MMU2_RST_PIN 23//#定义MMU2_RST_引脚23

  // Enable if the MMU2 has 12V stepper motors (MMU2 Firmware 1.0.2 and up)//如果MMU2具有12V步进电机（MMU2固件1.0.2及更高版本），则启用
  //#define MMU2_MODE_12V//#定义MMU2_模式_12V

  // G-code to execute when MMU2 F.I.N.D.A. probe detects filament runout//MMU2 F.I.N.D.A.时执行的G代码。探头检测灯丝跳动
  #define MMU2_FILAMENT_RUNOUT_SCRIPT "M600"

  // Add an LCD menu for MMU2//为MMU2添加LCD菜单
  //#define MMU2_MENUS//#定义MMU2_菜单
  #if EITHER(MMU2_MENUS, HAS_PRUSA_MMU2S)
    // Settings for filament load / unload from the LCD menu.//从LCD菜单加载/卸载灯丝的设置。
    // This is for Průša MK3-style extruders. Customize for your hardware.//这适用于Průša MK3型挤出机。为您的硬件定制。
    #define MMU2_FILAMENTCHANGE_EJECT_FEED 80.0
    #define MMU2_LOAD_TO_NOZZLE_SEQUENCE \
      {  7.2, 1145 }, \
      { 14.4,  871 }, \
      { 36.0, 1393 }, \
      { 14.4,  871 }, \
      { 50.0,  198 }

    #define MMU2_RAMMING_SEQUENCE \
      {   1.0, 1000 }, \
      {   1.0, 1500 }, \
      {   2.0, 2000 }, \
      {   1.5, 3000 }, \
      {   2.5, 4000 }, \
      { -15.0, 5000 }, \
      { -14.0, 1200 }, \
      {  -6.0,  600 }, \
      {  10.0,  700 }, \
      { -10.0,  400 }, \
      { -50.0, 2000 }
  #endif

  /**
   * Using a sensor like the MMU2S
   * This mode requires a MK3S extruder with a sensor at the extruder idler, like the MMU2S.
   * See https://help.prusa3d.com/en/guide/3b-mk3s-mk2-5s-extruder-upgrade_41560, step 11
   */
  #if HAS_PRUSA_MMU2S
    #define MMU2_C0_RETRY   5             // Number of retries (total time = timeout*retries)//重试次数（总时间=超时*重试次数）

    #define MMU2_CAN_LOAD_FEEDRATE 800    // (mm/min)//（毫米/分钟）
    #define MMU2_CAN_LOAD_SEQUENCE \
      {  0.1, MMU2_CAN_LOAD_FEEDRATE }, \
      {  60.0, MMU2_CAN_LOAD_FEEDRATE }, \
      { -52.0, MMU2_CAN_LOAD_FEEDRATE }

    #define MMU2_CAN_LOAD_RETRACT   6.0   // (mm) Keep under the distance between Load Sequence values//（mm）保持在荷载顺序值之间的距离以下
    #define MMU2_CAN_LOAD_DEVIATION 0.8   // (mm) Acceptable deviation//（mm）可接受偏差

    #define MMU2_CAN_LOAD_INCREMENT 0.2   // (mm) To reuse within MMU2 module//（mm）在MMU2模块内重复使用
    #define MMU2_CAN_LOAD_INCREMENT_SEQUENCE \
      { -MMU2_CAN_LOAD_INCREMENT, MMU2_CAN_LOAD_FEEDRATE }

  #else

    /**
     * MMU1 Extruder Sensor
     *
     * Support for a Průša (or other) IR Sensor to detect filament near the extruder
     * and make loading more reliable. Suitable for an extruder equipped with a filament
     * sensor less than 38mm from the gears.
     *
     * During loading the extruder will stop when the sensor is triggered, then do a last
     * move up to the gears. If no filament is detected, the MMU2 can make some more attempts.
     * If all attempts fail, a filament runout will be triggered.
     */
    //#define MMU_EXTRUDER_SENSOR//#定义MMU_挤出机_传感器
    #if ENABLED(MMU_EXTRUDER_SENSOR)
      #define MMU_LOADING_ATTEMPTS_NR 5 // max. number of attempts to load filament if first load fail//如果第一次加载失败，尝试加载灯丝的最大次数
    #endif

  #endif

  //#define MMU2_DEBUG  // Write debug info to serial output//#定义MMU2_调试//将调试信息写入串行输出

#endif // HAS_PRUSA_MMU2//你有什么想法

/**
 * Advanced Print Counter settings
 */
#if ENABLED(PRINTCOUNTER)
#define SERVICE_WARNING_BUZZES  3
  // Activate up to 3 service interval watchdogs//激活最多3个维修间隔看门狗
  //#define SERVICE_NAME_1      "Service S"//#定义服务\u名称\u 1“服务”
  //#define SERVICE_INTERVAL_1  100 // print hours//#定义服务间隔\u 1 100//打印小时数
  //#define SERVICE_NAME_2      "Service L"//#定义服务\u名称\u 2“服务L”
  //#define SERVICE_INTERVAL_2  200 // print hours//#定义服务间隔200//打印小时数
  //#define SERVICE_NAME_3      "Service 3"//#定义服务\u名称\u 3“服务3”
  //#define SERVICE_INTERVAL_3    1 // print hours//#定义服务间隔\u 3 1//打印小时数
#endif

// @section develop//@节开发

////
// M100 Free Memory Watcher to debug memory usage//用于调试内存使用情况的M100空闲内存观察程序
////
//#define M100_FREE_MEMORY_WATCHER//#定义M100空闲内存观察程序

////
// M42 - Set pin states//M42-设置引脚状态
////
//#define DIRECT_PIN_CONTROL//#定义直接引脚控制

////
// M43 - display pin status, toggle pins, watch pins, watch endstops & toggle LED, test servo probe//M43-显示管脚状态、拨动管脚、手表管脚、手表止动块和拨动LED、测试伺服探针
////
//#define PINS_DEBUGGING//#在调试中定义引脚

// Enable Marlin dev mode which adds some special commands//启用添加一些特殊命令的Marlin dev模式
//#define MARLIN_DEV_MODE//#定义MARLIN_DEV_模式

#if ENABLED(MARLIN_DEV_MODE)
/**
   * D576 - Buffer Monitoring
   * To help diagnose print quality issues stemming from empty command buffers.
   */
  //#define BUFFER_MONITORING//#定义缓冲区监视
#endif

/**
 * Postmortem Debugging captures misbehavior and outputs the CPU status and backtrace to serial.
 * When running in the debugger it will break for debugging. This is useful to help understand
 * a crash from a remote location. Requires ~400 bytes of SRAM and 5Kb of flash.
 */
//#define POSTMORTEM_DEBUGGING//#定义事后调试

/**
 * Software Reset options
 */
//#define SOFT_RESET_VIA_SERIAL         // 'KILL' and '^X' commands will soft-reset the controller//#通过串行//“KILL”和“^X”命令定义软重置，软重置控制器
//#define SOFT_RESET_ON_KILL            // Use a digital button to soft-reset the controller after KILL//#在压井时定义软压井复位//压井后使用数字按钮软压井复位控制器
