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
 * parser.h - Parser for a GCode line, providing a parameter interface.
 *           Codes like M149 control the way the GCode parser behaves,
 *           so settings for these codes are located in this class.
 */

#include "../inc/MarlinConfig.h"

//#define DEBUG_GCODE_PARSER//#定义调试代码解析器
#if ENABLED(DEBUG_GCODE_PARSER)
  #include "../libs/hex_print.h"
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  typedef enum : uint8_t { TEMPUNIT_C, TEMPUNIT_K, TEMPUNIT_F } TempUnit;
#endif

#if ENABLED(INCH_MODE_SUPPORT)
  typedef enum : uint8_t { LINEARUNIT_MM, LINEARUNIT_INCH } LinearUnit;
#endif

/**
 * GCode parser
 *
 *  - Parse a single gcode line for its letter, code, subcode, and parameters
 *  - FASTER_GCODE_PARSER:
 *    - Flags existing params (1 bit each)
 *    - Stores value offsets (1 byte each)
 *  - Provide accessors for parameters:
 *    - Parameter exists
 *    - Parameter has value
 *    - Parameter value in different units and types
 */
class GCodeParser {

private:
  static char *value_ptr;           // Set by seen, used to fetch the value//由seen设置，用于获取值

  #if ENABLED(FASTER_GCODE_PARSER)
    static uint32_t codebits;       // Parameters pre-scanned//预扫描参数
    static uint8_t param[26];       // For A-Z, offsets into command args//对于A-Z，将偏移量转换为命令参数
  #else
    static char *command_args;      // Args start here, for slow scan//Args从这里开始，用于慢速扫描
  #endif

public:

  // Global states for GCode-level units features//GCode级别单位功能的全局状态

  static bool volumetric_enabled;

  #if ENABLED(INCH_MODE_SUPPORT)
    static float linear_unit_factor, volumetric_unit_factor;
  #endif

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
    static TempUnit input_temp_units;
  #endif

  // Command line state//命令行状态
  static char *command_ptr,               // The command, so it can be echoed//命令，以便可以响应
              *string_arg,                // string of command line//命令行字符串
              command_letter;             // G, M, or T//G、M或T
  static uint16_t codenum;                // 123// 123
  #if USE_GCODE_SUBCODES
    static uint8_t subcode;               // .1// .1
  #endif

  #if ENABLED(GCODE_MOTION_MODES)
    static int16_t motion_mode_codenum;
    #if USE_GCODE_SUBCODES
      static uint8_t motion_mode_subcode;
    #endif
    FORCE_INLINE static void cancel_motion_mode() { motion_mode_codenum = -1; }
  #endif

  #if ENABLED(DEBUG_GCODE_PARSER)
    static void debug();
  #endif

  // Reset is done before parsing//重置在解析之前完成
  static void reset();

  #define LETTER_BIT(N) ((N) - 'A')

  FORCE_INLINE static bool valid_signless(const char * const p) {
    return NUMERIC(p[0]) || (p[0] == '.' && NUMERIC(p[1])); // .?[0-9]// .?[0-9]
  }

  FORCE_INLINE static bool valid_float(const char * const p) {
    return valid_signless(p) || ((p[0] == '-' || p[0] == '+') && valid_signless(&p[1])); // [-+]?.?[0-9]// [-+]?.?[0-9]
  }

  FORCE_INLINE static bool valid_number(const char * const p) {
    // TODO: With MARLIN_DEV_MODE allow HEX values starting with "x"//TODO:在MARLIN_DEV_模式下，允许以“x”开头的十六进制值
    return valid_float(p);
  }

  #if ENABLED(FASTER_GCODE_PARSER)

    FORCE_INLINE static bool valid_int(const char * const p) {
      return NUMERIC(p[0]) || ((p[0] == '-' || p[0] == '+') && NUMERIC(p[1])); // [-+]?[0-9]// [-+]?[0-9]
    }

    // Set the flag and pointer for a parameter//设置参数的标志和指针
    static inline void set(const char c, char * const ptr) {
      const uint8_t ind = LETTER_BIT(c);
      if (ind >= COUNT(param)) return;           // Only A-Z//只有A-Z
      SBI32(codebits, ind);                      // parameter exists//参数存在
      param[ind] = ptr ? ptr - command_ptr : 0;  // parameter offset or 0//参数偏移量或0
      #if ENABLED(DEBUG_GCODE_PARSER)
        if (codenum == 800) {
          SERIAL_ECHOPAIR("Set bit ", ind, " of codebits (", hex_address((void*)(codebits >> 16)));
          print_hex_word((uint16_t)(codebits & 0xFFFF));
          SERIAL_ECHOLNPAIR(") | param = ", param[ind]);
        }
      #endif
    }

    // Code seen bit was set. If not found, value_ptr is unchanged.//代码位已设置。如果未找到，则值_ptr不变。
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.//这允许“if（seen（'A'）| | seen（'B'））”使用最后找到的值。
    static inline bool seen(const char c) {
      const uint8_t ind = LETTER_BIT(c);
      if (ind >= COUNT(param)) return false; // Only A-Z//只有A-Z
      const bool b = TEST32(codebits, ind);
      if (b) {
        if (param[ind]) {
          char * const ptr = command_ptr + param[ind];
          value_ptr = valid_number(ptr) ? ptr : nullptr;
        }
        else
          value_ptr = nullptr;
      }
      return b;
    }

    FORCE_INLINE static constexpr uint32_t letter_bits(const char * const str) {
      return  (str[0] ? _BV32(LETTER_BIT(str[0])) |
              (str[1] ? _BV32(LETTER_BIT(str[1])) |
              (str[2] ? _BV32(LETTER_BIT(str[2])) |
              (str[3] ? _BV32(LETTER_BIT(str[3])) |
              (str[4] ? _BV32(LETTER_BIT(str[4])) |
              (str[5] ? _BV32(LETTER_BIT(str[5])) |
              (str[6] ? _BV32(LETTER_BIT(str[6])) |
              (str[7] ? _BV32(LETTER_BIT(str[7])) |
              (str[8] ? _BV32(LETTER_BIT(str[8])) |
              (str[9] ? _BV32(LETTER_BIT(str[9]))
            : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0) : 0);
    }

    // At least one of a list of code letters was seen//至少看到了代码字母列表中的一个
    #ifdef CPU_32_BIT
      FORCE_INLINE static bool seen(const char * const str) { return !!(codebits & letter_bits(str)); }
    #else
      FORCE_INLINE static bool seen(const char * const str) {
        const uint32_t letrbits = letter_bits(str);
        const uint8_t * const cb = (uint8_t*)&codebits;
        const uint8_t * const lb = (uint8_t*)&letrbits;
        return (cb[0] & lb[0]) || (cb[1] & lb[1]) || (cb[2] & lb[2]) || (cb[3] & lb[3]);
      }
    #endif

    static inline bool seen_any() { return !!codebits; }

    FORCE_INLINE static bool seen_test(const char c) { return TEST32(codebits, LETTER_BIT(c)); }

  #else // !FASTER_GCODE_PARSER// !更快的代码解析器

    #if ENABLED(GCODE_CASE_INSENSITIVE)
      FORCE_INLINE static char* strgchr(char *p, char g) {
        auto uppercase = [](char c) {
          return c + (WITHIN(c, 'a', 'z') ? 'A' - 'a' : 0);
        };
        const char d = uppercase(g);
        for (char cc; (cc = uppercase(*p)); p++) if (cc == d) return p;
        return nullptr;
      }
    #else
      #define strgchr strchr
    #endif

    // Code is found in the string. If not found, value_ptr is unchanged.//在字符串中找到代码。如果未找到，则值_ptr不变。
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.//这允许“if（seen（'A'）| | seen（'B'））”使用最后找到的值。
    static inline bool seen(const char c) {
      char *p = strgchr(command_args, c);
      const bool b = !!p;
      if (b) value_ptr = valid_number(&p[1]) ? &p[1] : nullptr;
      return b;
    }

    static inline bool seen_any() { return *command_args == '\0'; }

    FORCE_INLINE static bool seen_test(const char c) { return (bool)strgchr(command_args, c); }

    // At least one of a list of code letters was seen//至少看到了代码字母列表中的一个
    static inline bool seen(const char * const str) {
      for (uint8_t i = 0; const char c = str[i]; i++)
        if (seen_test(c)) return true;
      return false;
    }

  #endif // !FASTER_GCODE_PARSER// !更快的代码解析器

  // Seen any axis parameter//看到任何轴参数了吗
  static inline bool seen_axis() {
    return seen(LOGICAL_AXIS_GANG("E", "X", "Y", "Z", AXIS4_STR, AXIS5_STR, AXIS6_STR));
  }

  #if ENABLED(GCODE_QUOTED_STRINGS)
    static char* unescape_string(char* &src);
  #else
    FORCE_INLINE static char* unescape_string(char* &src) { return src; }
  #endif

  // Populate all fields by parsing a single line of GCode//通过解析一行GCode填充所有字段
  // This uses 54 bytes of SRAM to speed up seen/value//这使用54字节的SRAM来加速SEED/value
  static void parse(char * p);

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    // Parse the next parameter as a new command//将下一个参数解析为新命令
    static bool chain();
  #endif

  // Test whether the parsed command matches the input//测试解析的命令是否与输入匹配
  static inline bool is_command(const char ltr, const uint16_t num) { return command_letter == ltr && codenum == num; }

  // The code value pointer was set//已设置代码值指针
  FORCE_INLINE static bool has_value() { return !!value_ptr; }

  // Seen a parameter with a value//查看带有值的参数
  static inline bool seenval(const char c) { return seen(c) && has_value(); }

  // The value as a string//将值设置为字符串
  static inline char* value_string() { return value_ptr; }

  // Float removes 'E' to prevent scientific notation interpretation//Float删除“E”以防止科学符号解释
  static inline float value_float() {
    if (value_ptr) {
      char *e = value_ptr;
      for (;;) {
        const char c = *e;
        if (c == '\0' || c == ' ') break;
        if (c == 'E' || c == 'e') {
          *e = '\0';
          const float ret = strtof(value_ptr, nullptr);
          *e = c;
          return ret;
        }
        ++e;
      }
      return strtof(value_ptr, nullptr);
    }
    return 0;
  }

  // Code value as a long or ulong//代码值为long或ulong
  static inline int32_t value_long() { return value_ptr ? strtol(value_ptr, nullptr, 10) : 0L; }
  static inline uint32_t value_ulong() { return value_ptr ? strtoul(value_ptr, nullptr, 10) : 0UL; }

  // Code value for use as time//用作时间的代码值
  static inline millis_t value_millis() { return value_ulong(); }
  static inline millis_t value_millis_from_seconds() { return (millis_t)SEC_TO_MS(value_float()); }

  // Reduce to fewer bits//减少到更少的位
  static inline int16_t value_int() { return (int16_t)value_long(); }
  static inline uint16_t value_ushort() { return (uint16_t)value_long(); }
  static inline uint8_t value_byte() { return (uint8_t)constrain(value_long(), 0, 255); }

  // Bool is true with no value or non-zero//Bool为true，无值或非零
  static inline bool value_bool() { return !has_value() || !!value_byte(); }

  // Units modes: Inches, Fahrenheit, Kelvin//单位模式：英寸、华氏度、开尔文

  #if ENABLED(INCH_MODE_SUPPORT)
    static inline float mm_to_linear_unit(const_float_t mm)     { return mm / linear_unit_factor; }
    static inline float mm_to_volumetric_unit(const_float_t mm) { return mm / (volumetric_enabled ? volumetric_unit_factor : linear_unit_factor); }

    // Init linear units by constructor//构造函数初始化线性单位
    GCodeParser() { set_input_linear_units(LINEARUNIT_MM); }

    static inline void set_input_linear_units(const LinearUnit units) {
      switch (units) {
        default:
        case LINEARUNIT_MM:   linear_unit_factor =  1.0f; break;
        case LINEARUNIT_INCH: linear_unit_factor = 25.4f; break;
      }
      volumetric_unit_factor = POW(linear_unit_factor, 3);
    }

    static inline float axis_unit_factor(const AxisEnum axis) {
      return (
        #if HAS_EXTRUDERS
          axis >= E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor
        #else
          linear_unit_factor
        #endif
      );
    }

    static inline float linear_value_to_mm(const_float_t v)                  { return v * linear_unit_factor; }
    static inline float axis_value_to_mm(const AxisEnum axis, const float v) { return v * axis_unit_factor(axis); }
    static inline float per_axis_value(const AxisEnum axis, const float v)   { return v / axis_unit_factor(axis); }

  #else

    static inline float mm_to_linear_unit(const_float_t mm)     { return mm; }
    static inline float mm_to_volumetric_unit(const_float_t mm) { return mm; }

    static inline float linear_value_to_mm(const_float_t v)             { return v; }
    static inline float axis_value_to_mm(const AxisEnum, const float v) { return v; }
    static inline float per_axis_value(const AxisEnum, const float v)   { return v; }

  #endif

  static inline bool using_inch_units() { return mm_to_linear_unit(1.0f) != 1.0f; }

  #define IN_TO_MM(I)        ((I) * 25.4f)
  #define MM_TO_IN(M)        ((M) / 25.4f)
  #define LINEAR_UNIT(V)     parser.mm_to_linear_unit(V)
  #define VOLUMETRIC_UNIT(V) parser.mm_to_volumetric_unit(V)

  static inline float value_linear_units()                      { return linear_value_to_mm(value_float()); }
  static inline float value_axis_units(const AxisEnum axis)     { return axis_value_to_mm(axis, value_float()); }
  static inline float value_per_axis_units(const AxisEnum axis) { return per_axis_value(axis, value_float()); }

  #if ENABLED(TEMPERATURE_UNITS_SUPPORT)

    static inline void set_input_temp_units(const TempUnit units) { input_temp_units = units; }

    #if HAS_LCD_MENU && DISABLED(DISABLE_M503)

      static inline char temp_units_code() {
        return input_temp_units == TEMPUNIT_K ? 'K' : input_temp_units == TEMPUNIT_F ? 'F' : 'C';
      }
      static inline PGM_P temp_units_name() {
        return input_temp_units == TEMPUNIT_K ? PSTR("Kelvin") : input_temp_units == TEMPUNIT_F ? PSTR("Fahrenheit") : PSTR("Celsius");
      }
      static inline float to_temp_units(celsius_t c) {
        switch (input_temp_units) {
          default:
          case TEMPUNIT_C: return c;
          case TEMPUNIT_K: return c + 273.15f;
          case TEMPUNIT_F: return c * 0.5555555556f + 32;
        }
      }

    #endif // HAS_LCD_MENU && !DISABLE_M503//有_LCD_菜单&！禁用_M503

    static inline celsius_t value_celsius() {
      float f = value_float();
      switch (input_temp_units) {
        default:
        case TEMPUNIT_C: break;
        case TEMPUNIT_K: f -= 273.15f;
        case TEMPUNIT_F: f = (f - 32) * 0.5555555556f;
      }
      return LROUND(f);
    }

    static inline celsius_t value_celsius_diff() {
      float f = value_float();
      switch (input_temp_units) {
        default:
        case TEMPUNIT_C:
        case TEMPUNIT_K: break;
        case TEMPUNIT_F: f *= 0.5555555556f;
      }
      return LROUND(f);
    }

  #else // !TEMPERATURE_UNITS_SUPPORT// !温度单元支持

    static inline float to_temp_units(int16_t c) { return (float)c; }

    static inline celsius_t value_celsius()      { return value_int(); }
    static inline celsius_t value_celsius_diff() { return value_int(); }

  #endif // !TEMPERATURE_UNITS_SUPPORT// !温度单元支持

  static inline feedRate_t value_feedrate() { return MMM_TO_MMS(value_linear_units()); }

  void unknown_command_warning();

  // Provide simple value accessors with default option//提供带有默认选项的简单值访问器
  static inline char*     stringval(const char c, char * const dval=nullptr) { return seenval(c) ? value_string()   : dval; }
  static inline float     floatval(const char c, const float dval=0.0)   { return seenval(c) ? value_float()        : dval; }
  static inline bool      boolval(const char c, const bool dval=false)   { return seenval(c) ? value_bool()         : (seen(c) ? true : dval); }
  static inline uint8_t   byteval(const char c, const uint8_t dval=0)    { return seenval(c) ? value_byte()         : dval; }
  static inline int16_t   intval(const char c, const int16_t dval=0)     { return seenval(c) ? value_int()          : dval; }
  static inline uint16_t  ushortval(const char c, const uint16_t dval=0) { return seenval(c) ? value_ushort()       : dval; }
  static inline int32_t   longval(const char c, const int32_t dval=0)    { return seenval(c) ? value_long()         : dval; }
  static inline uint32_t  ulongval(const char c, const uint32_t dval=0)  { return seenval(c) ? value_ulong()        : dval; }
  static inline float     linearval(const char c, const float dval=0)    { return seenval(c) ? value_linear_units() : dval; }
  static inline float     axisunitsval(const char c, const AxisEnum a, const float dval=0)
                                                                         { return seenval(c) ? value_axis_units(a)  : dval; }
  static inline celsius_t celsiusval(const char c, const float dval=0)   { return seenval(c) ? value_celsius()      : dval; }

  #if ENABLED(MARLIN_DEV_MODE)

    static inline uint8_t* hex_adr_val(const char c, uint8_t * const dval=nullptr) {
      if (!seen(c) || *value_ptr != 'x') return dval;
      uint8_t *out = nullptr;
      for (char *vp = value_ptr + 1; HEXCHR(*vp) >= 0; vp++)
        out = (uint8_t*)((uintptr_t(out) << 8) | HEXCHR(*vp));
      return out;
    }

    static inline uint16_t hex_val(const char c, uint16_t const dval=0) {
      if (!seen(c) || *value_ptr != 'x') return dval;
      uint16_t out = 0;
      for (char *vp = value_ptr + 1; HEXCHR(*vp) >= 0; vp++)
        out = ((out) << 8) | HEXCHR(*vp);
      return out;
    }

  #endif
};

extern GCodeParser parser;
