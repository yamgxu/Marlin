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

#include "../inc/MarlinConfigPre.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../feature/e_parser.h"
#endif

// Used in multiple places//用于多个地方
// You can build it but not manipulate it.//您可以构建它，但不能操纵它。
// There are only few places where it's required to access the underlying member: GCodeQueue, SerialMask and MultiSerial//只有很少几个地方需要它来访问底层成员：GCodeQueue、SerialMask和MultiSerial
struct serial_index_t {
  // A signed index, where -1 is a special case meaning no action (neither output or input)//有符号索引，其中-1是一个特例，表示没有操作（既没有输出也没有输入）
  int8_t  index;

  // Check if the index is within the range [a ... b]//检查索引是否在[a…b]范围内
  constexpr inline bool within(const int8_t a, const int8_t b) const { return WITHIN(index, a, b); }
  constexpr inline bool valid() const { return WITHIN(index, 0, 7); } // At most, 8 bits//最多8位

  // Construction is either from an index//构造是从一个索引开始的
  constexpr serial_index_t(const int8_t index) : index(index) {}

  // Default to "no index"//默认为“无索引”
  constexpr serial_index_t() : index(-1) {}
};

// In order to catch usage errors in code, we make the base to encode number explicit//为了捕获代码中的使用错误，我们使编码数字的基显式化
// If given a number (and not this enum), the compiler will reject the overload, falling back to the (double, digit) version//如果给定一个数字（而不是这个枚举），编译器将拒绝重载，返回到（双位数）版本
// We don't want hidden conversion of the first parameter to double, so it has to be as hard to do for the compiler as creating this enum//我们不希望将第一个参数隐藏转换为double，因此编译器必须像创建此枚举一样困难
enum class PrintBase {
  Dec = 10,
  Hex = 16,
  Oct = 8,
  Bin = 2
};

// A simple feature list enumeration//一个简单的特征列表枚举
enum class SerialFeature {
  None                = 0x00,
  MeatPack            = 0x01,   //!< Enabled when Meatpack is present//!< 当肉包存在时启用
  BinaryFileTransfer  = 0x02,   //!< Enabled for BinaryFile transfer support (in the future)//!< 已启用二进制文件传输支持（将来）
  Virtual             = 0x04,   //!< Enabled for virtual serial port (like Telnet / Websocket / ...)//!< 为虚拟串行端口启用（如Telnet/Websocket/…）
  Hookable            = 0x08,   //!< Enabled if the serial class supports a setHook method//!< 如果串行类支持setHook方法，则启用
};
ENUM_FLAGS(SerialFeature);

// flushTX is not implemented in all HAL, so use SFINAE to call the method where it is.//flushTX并不是在所有HAL中都实现的，所以使用SFINAE调用它所在的方法。
CALL_IF_EXISTS_IMPL(void, flushTX);
CALL_IF_EXISTS_IMPL(bool, connected, true);
CALL_IF_EXISTS_IMPL(SerialFeature, features, SerialFeature::None);

// A simple forward struct to prevent the compiler from selecting print(double, int) as a default overload//一个简单的正向结构，防止编译器选择print（double，int）作为默认重载
// for any type other than double/float. For double/float, a conversion exists so the call will be invisible.//适用于除双重/浮动以外的任何类型。对于double/float，存在转换，因此调用将不可见。
struct EnsureDouble {
  double a;
  FORCE_INLINE operator double() { return a; }
  // If the compiler breaks on ambiguity here, it's likely because print(X, base) is called with X not a double/float, and//如果编译器在这里出现歧义，很可能是因为print（X，base）是用X而不是double/float调用的，并且
  // a base that's not a PrintBase value. This code is made to detect the error. You MUST set a base explicitly like this://不是PrintBase值的基。此代码用于检测错误。您必须像下面这样显式地设置基：
  // SERIAL_PRINT(v, PrintBase::Hex)//串行打印（v，打印基：：十六进制）
  FORCE_INLINE EnsureDouble(double a) : a(a) {}
  FORCE_INLINE EnsureDouble(float a) : a(a) {}
};

// Using Curiously-Recurring Template Pattern here to avoid virtual table cost when compiling.//在这里使用奇怪的循环模板模式，以避免编译时的虚拟表成本。
// Since the real serial class is known at compile time, this results in the compiler writing//因为真正的串行类在编译时是已知的，这会导致编译器编写
// a completely efficient code.//一个完全有效的代码。
template <class Child>
struct SerialBase {
  #if ENABLED(EMERGENCY_PARSER)
    const bool ep_enabled;
    EmergencyParser::State emergency_state;
    inline bool emergency_parser_enabled() { return ep_enabled; }
    SerialBase(bool ep_capable) : ep_enabled(ep_capable), emergency_state(EmergencyParser::State::EP_RESET) {}
  #else
    SerialBase(const bool) {}
  #endif

  #define SerialChild static_cast<Child*>(this)

  // Static dispatch methods below://静态调度方法如下：
  // The most important method here is where it all ends to://这里最重要的方法是它的最终目的：
  void write(uint8_t c)             { SerialChild->write(c); }

  // Called when the parser finished processing an instruction, usually build to nothing//在解析器完成指令处理时调用，通常编译为空
  void msgDone() const              { SerialChild->msgDone(); }

  // Called on initialization//在初始化时调用
  void begin(const long baudRate)   { SerialChild->begin(baudRate); }

  // Called on destruction//呼吁销毁
  void end()                        { SerialChild->end(); }

  /** Check for available data from the port
      @param index  The port index, usually 0 */
  int available(serial_index_t index=0) const { return SerialChild->available(index); }

  /** Read a value from the port
      @param index  The port index, usually 0 */
  int read(serial_index_t index=0)        { return SerialChild->read(index); }

  /** Combine the features of this serial instance and return it
      @param index  The port index, usually 0 */
  SerialFeature features(serial_index_t index=0) const { return static_cast<const Child*>(this)->features(index);  }

  // Check if the serial port has a feature//检查串行端口是否具有功能
  bool has_feature(serial_index_t index, SerialFeature flag) const { return (features(index) & flag) != SerialFeature::None; }

  // Check if the serial port is connected (usually bypassed)//检查串行端口是否已连接（通常为旁路）
  bool connected() const            { return SerialChild->connected(); }

  // Redirect flush//重定向刷新
  void flush()                      { SerialChild->flush(); }

  // Not all implementation have a flushTX, so let's call them only if the child has the implementation//并不是所有的实现都有flushTX，所以只有当子实现有flushTX时，我们才调用它们
  void flushTX()                    { CALL_IF_EXISTS(void, SerialChild, flushTX); }

  // Glue code here//在这里粘贴代码
  FORCE_INLINE void write(const char *str)                    { while (*str) write(*str++); }
  FORCE_INLINE void write(const uint8_t *buffer, size_t size) { while (size--) write(*buffer++); }
  FORCE_INLINE void print(const char *str)                    { write(str); }
  // No default argument to avoid ambiguity//没有默认参数以避免歧义
  NO_INLINE void print(char c, PrintBase base)                { printNumber((signed long)c, (uint8_t)base); }
  NO_INLINE void print(unsigned char c, PrintBase base)       { printNumber((unsigned long)c, (uint8_t)base); }
  NO_INLINE void print(int c, PrintBase base)                 { printNumber((signed long)c, (uint8_t)base); }
  NO_INLINE void print(unsigned int c, PrintBase base)        { printNumber((unsigned long)c, (uint8_t)base); }
  void print(unsigned long c, PrintBase base)                 { printNumber((unsigned long)c, (uint8_t)base); }
  void print(long c, PrintBase base)                          { printNumber((signed long)c, (uint8_t)base); }
  void print(EnsureDouble c, int digits)                      { printFloat(c, digits); }

  // Forward the call to the former's method//将调用转发给前者的方法
  FORCE_INLINE void print(char c)                { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(unsigned char c)       { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(int c)                 { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(unsigned int c)        { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(unsigned long c)       { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(long c)                { print(c, PrintBase::Dec); }
  FORCE_INLINE void print(double c)              { print(c, 2); }

  FORCE_INLINE void println(const char s[])                  { print(s); println(); }
  FORCE_INLINE void println(char c, PrintBase base)          { print(c, base); println(); }
  FORCE_INLINE void println(unsigned char c, PrintBase base) { print(c, base); println(); }
  FORCE_INLINE void println(int c, PrintBase base)           { print(c, base); println(); }
  FORCE_INLINE void println(unsigned int c, PrintBase base)  { print(c, base); println(); }
  FORCE_INLINE void println(long c, PrintBase base)          { print(c, base); println(); }
  FORCE_INLINE void println(unsigned long c, PrintBase base) { print(c, base); println(); }
  FORCE_INLINE void println(double c, int digits)            { print(c, digits); println(); }
  FORCE_INLINE void println()                                { write('\r'); write('\n'); }

  // Forward the call to the former's method//将调用转发给前者的方法
  FORCE_INLINE void println(char c)                { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(unsigned char c)       { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(int c)                 { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(unsigned int c)        { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(unsigned long c)       { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(long c)                { println(c, PrintBase::Dec); }
  FORCE_INLINE void println(double c)              { println(c, 2); }

  // Print a number with the given base//用给定的基数打印一个数字
  NO_INLINE void printNumber(unsigned long n, const uint8_t base) {
    if (!base) return; // Hopefully, this should raise visible bug immediately//希望这会立即引起可见的bug

    if (n) {
      unsigned char buf[8 * sizeof(long)]; // Enough space for base 2//有足够的空间放置底座2
      int8_t i = 0;
      while (n) {
        buf[i++] = n % base;
        n /= base;
      }
      while (i--) write((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else write('0');
  }
  void printNumber(signed long n, const uint8_t base) {
    if (base == 10 && n < 0) {
      n = -n; // This works because all platforms Marlin's builds on are using 2-complement encoding for negative number//这是因为Marlin构建的所有平台都对负数使用2补码编码
              // On such CPU, changing the sign of a number is done by inverting the bits and adding one, so if n = 0x80000000 = -2147483648 then//在这样的CPU上，更改数字的符号是通过反转位并添加一来完成的，因此，如果n=0x8000000=-2147483648，则
              // -n = 0x7FFFFFFF + 1 => 0x80000000 = 2147483648 (if interpreted as unsigned) or -2147483648 if interpreted as signed.//-n=0x7FFFFFFF+1=>0x8000000=2147483648（如果解释为无符号）或-2147483648（如果解释为有符号）。
              // On non 2-complement CPU, there would be no possible representation for 2147483648.//在非2补码CPU上，2147483648没有可能的表示形式。
      write('-');
    }
    printNumber((unsigned long)n , base);
  }

  // Print a decimal number//打印十进制数
  NO_INLINE void printFloat(double number, uint8_t digits) {
    // Handle negative numbers//处理负数
    if (number < 0.0) {
      write('-');
      number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"//正确四舍五入，以便打印（1.999，2）打印为“2.00”
    double rounding = 0.5;
    LOOP_L_N(i, digits) rounding *= 0.1;
    number += rounding;

    // Extract the integer part of the number and print it//提取数字的整数部分并打印它
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    printNumber(int_part, 10);

    // Print the decimal point, but only if there are digits beyond//打印小数点，但仅当数字超过
    if (digits) {
      write('.');
      // Extract digits from the remainder one at a time//从余数中一次提取一个数字
      while (digits--) {
        remainder *= 10.0;
        unsigned long toPrint = (unsigned long)remainder;
        printNumber(toPrint, 10);
        remainder -= toPrint;
      }
    }
  }
};

// All serial instances will be built by chaining the features required//所有串行实例都将通过链接所需的功能来构建
// for the function in the form of a template type definition.//对于模板类型定义形式的函数。
