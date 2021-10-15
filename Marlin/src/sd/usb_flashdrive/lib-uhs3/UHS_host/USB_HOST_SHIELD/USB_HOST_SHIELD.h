/** translatione by yx */
/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com//www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#ifndef USB_HOST_SHIELD_H
#define USB_HOST_SHIELD_H

// uncomment to get 'printf' console debugging. NOT FOR UNO!//取消注释以获取“printf”控制台调试。不适合乌诺！
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD//#定义调试\打印\额外\巨大\ USB \主机\屏蔽

#ifdef LOAD_USB_HOST_SHIELD
#include "UHS_max3421e.h"
#include <SPI.h>


#ifndef SPI_HAS_TRANSACTION
#error "Your SPI library installation is too old."
#else
#ifndef SPI_ATOMIC_VERSION
#warning "Your SPI library installation lacks 'SPI_ATOMIC_VERSION'. Please complain to the maintainer."
#elif SPI_ATOMIC_VERSION < 1
#error "Your SPI library installation is too old."
#endif

#endif
#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef DEBUG_PRINTF_EXTRA_HUGE_USB_HOST_SHIELD
#define MAX_HOST_DEBUG(...) printf_P(__VA_ARGS__)
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif
#else
#define MAX_HOST_DEBUG(...) VOID0
#endif

#ifndef USB_HOST_SHIELD_USE_ISR
#ifdef USE_MULTIPLE_APP_API
#define USB_HOST_SHIELD_USE_ISR 0
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif
#else
#define USB_HOST_SHIELD_USE_ISR 1
#endif



#if !USB_HOST_SHIELD_USE_ISR
#error NOISR Polled mode _NOT SUPPORTED YET_

////
// Polled defaults//轮询默认值
////
#ifdef BOARD_BLACK_WIDDOW
#define UHS_MAX3421E_SS_ 6
#define UHS_MAX3421E_INT_ 3
#elif defined(CORE_TEENSY) && (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))
#if EXT_RAM
// Teensy++ 2.0 with XMEM2//Teensy++2.0与XMEM2
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_SS_ 9
#define UHS_MAX3421E_INT_ 8
#endif
#define UHS_MAX3421E_SPD
#elif defined(ARDUINO_AVR_ADK)
#define UHS_MAX3421E_SS_ 53
#define UHS_MAX3421E_INT_ 54
#elif defined(ARDUINO_AVR_BALANDUINO)
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 19
#else
#define UHS_MAX3421E_SS_ 10
#define UHS_MAX3421E_INT_ 9
#endif

#else
#ifdef ARDUINO_ARCH_PIC32
// PIC32 only allows edge interrupts, isn't that lovely? We'll emulate it...//PIC32只允许边缘中断，是不是很可爱？我们将模仿它。。。
#if CHANGE < 2
#error core too old.
#endif

#define IRQ_IS_EDGE 0
#ifndef digitalPinToInterrupt
// great, this isn't implemented.//太好了，这还没有实现。
#warning digitalPinToInterrupt is not defined, complain here https://github.com/chipKIT32/chipKIT-core/issues/114//github.com/chipKIT32/chipKIT-core/issues/114
#if defined(_BOARD_UNO_) || defined(_BOARD_UC32_)
#define digitalPinToInterrupt(p) ((p) == 2 ? 1 : ((p) == 7 ? 2 : ((p) == 8 ? 3 : ((p) == 35 ? 4 : ((p) == 38 ? 0 : NOT_AN_INTERRUPT)))))
#warning digitalPinToInterrupt is now defined until this is taken care of.
#else
#error digitalPinToInterrupt not defined for your board, complain here https://github.com/chipKIT32/chipKIT-core/issues/114//github.com/chipKIT32/chipKIT-core/issues/114
#endif
#endif
#else
#define IRQ_IS_EDGE 0
#endif

// More stupidity from our friends @ Sony...//来自我们的朋友@Sony的更多愚蠢。。。
#ifdef ARDUINO_spresense_ast
#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT -1
#endif
#endif

// SAMD uses an enum for this instead of a define. Isn't that just dandy?//SAMD为此使用枚举，而不是定义。那不正是花花公子吗？
#if !defined(NOT_AN_INTERRUPT) && !defined(ARDUINO_ARCH_SAMD)
#warning NOT_AN_INTERRUPT not defined, possible problems ahead.
#warning If NOT_AN_INTERRUPT is an enum or something else, complain to UHS30 developers on github.
#warning Otherwise complain to your board core developer/maintainer.
#define NOT_AN_INTERRUPT -1
#endif

////
// Interrupt defaults. Int0 or Int1//中断默认值。Int0或Int1
////
#ifdef BOARD_BLACK_WIDDOW
#error "HELP! Please send us an email, I don't know the values for Int0 and Int1 on the Black Widow board!"
#elif defined(ARDUINO_AVR_ADK)
#define UHS_MAX3421E_SS_ 53
#define UHS_MAX3421E_INT_ 54
#elif defined(ARDUINO_spresense_ast)
#define UHS_MAX3421E_SS_ 21
#define UHS_MAX3421E_INT_ 20
#define SPIclass SPI5
//#define UHS_MAX3421E_SPD 100000//#定义UHS_MAX3421E_SPD 100000
#elif defined(CORE_TEENSY) && (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))

// TO-DO!//待办事项！

#if EXT_RAM
// Teensy++ 2.0 with XMEM2//Teensy++2.0与XMEM2
#define UHS_MAX3421E_SS_ 20
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_SS_ 9
#define UHS_MAX3421E_INT_ 8
#endif

#elif defined(ARDUINO_AVR_BALANDUINO)
#error "ISR mode is currently not supported on the Balanduino. Please set USB_HOST_SHIELD_USE_ISR to 0."
#else
#define UHS_MAX3421E_SS_ 10
#ifdef __AVR__
#ifdef __AVR_ATmega32U4__
#define INT_FOR_PIN2 1
#define INT_FOR_PIN3 0
#else
// Everybody else???//其他人呢？？？
#define INT_FOR_PIN2 0
#define INT_FOR_PIN3 1
#endif
#define UHS_MAX3421E_INT_ 3
#else
// Non-avr//非avr
#ifdef ARDUINO_ARCH_PIC32
// UNO32 External Interrupts://UNO32外部中断：
// Pin 38 (INT0), Pin 2 (INT1), Pin 7 (INT2), Pin 8 (INT3), Pin 35 (INT4)//插脚38（INT0）、插脚2（INT1）、插脚7（INT2）、插脚8（INT3）、插脚35（INT4）
#define UHS_MAX3421E_INT_ 7
#else
#define UHS_MAX3421E_INT_ 9
#endif
#endif
#endif
#endif



#ifdef NO_AUTO_SPEED
// Ugly details section...//丑陋的细节部分。。。
// MAX3421E characteristics//MAX3421E特性
// SPI Serial - Clock Input. An external SPI master supplies SCLK with frequencies up to 26MHz. The//串行时钟输入。外部SPI主机为SCLK提供高达26MHz的频率。这个
// logic level is referenced to the voltage on VL. Data is clocked into the SPI slave inter face on the//逻辑电平参考VL上的电压。数据被计时到主机上的SPI从机接口
// rising edge of SCLK. Data is clocked out of the SPI slave interface on the falling edge of SCLK.//SCLK的上升沿。数据从SCLK下降沿上的SPI从接口发条。
// Serial Clock (SCLK) Period 38.4ns minimum. 17ns minimum pulse width. VL >2.5V//串行时钟（SCLK）周期最小为38.4ns。最小脉冲宽度为17ns。VL>2.5V
// SCLK Fall to MISO Propagation Delay 14.2ns//SCLK下降至MISO传播延迟14.2ns
// SCLK Fall to MOSI Propagation Delay 14.2ns//SCLK下降至MOSI传播延迟14.2ns
// SCLK Fall to MOSI Drive 3.5ns//SCLK下降至MOSI车道3.5ns
// Theoretical deadline for reply 17.7ns//答复的理论截止时间17.7ns
// 26MHz 38.4615ns period <-- MAX3421E theoretical maximum//26MHz 38.4615ns周期<--MAX3421E理论最大值

#ifndef UHS_MAX3421E_SPD
#ifdef ARDUINO_SAMD_ZERO
// Zero violates spec early, needs a long setup time, or doesn't like high latency.//Zero很早就违反了规范，需要很长的安装时间，或者不喜欢高延迟。
#define UHS_MAX3421E_SPD 10000000
#elif defined(ARDUINO_ARCH_PIC32)
// PIC MX 5/6/7 characteristics//PIC MX 5/6/7特性
// 25MHZ 40ns period <-- PIC MX  5/6/7 theoretical maximum//25MHZ 40ns周期<--PIC MX 5/6/7理论最大值
// pulse width minimum Tsclk/2ns//最小脉宽Tsclk/2ns
// Trise/fall 10ns maximum. 5ns is typical but not guaranteed.//三重/下降最大10纳秒。5ns是典型的，但不能保证。
// Tsetup minimum for MISO 10ns.//t味噌10ns的最低安装量。
// We are in violation by 7.7ns @ 25MHz due to latency alone.//仅由于延迟，我们在25MHz时就违反了7.7ns。
// Even reading at end of data cycle, we only have a 2.3ns window.//即使在数据周期结束时读取，我们也只有一个2.3ns窗口。
// This is too narrow to to compensate for capacitance, trace lengths, and noise.//这太窄，无法补偿电容、迹线长度和噪声。

// 17.7ns + 10ns = 27.7ns//17.7ns+10ns=27.7ns
// 18MHz fits and has enough slack time to compensate for capacitance, trace lengths, and noise.//18MHz适合并有足够的空闲时间来补偿电容、跟踪长度和噪声。
// For high speeds the SMP bit is recommended too, which samples at the end instead of the middle.//对于高速，也建议使用SMP位，它在末尾而不是中间采样。
// 20Mhz seems to work.//20Mhz似乎能工作。

#define UHS_MAX3421E_SPD 20000000
#else
#define UHS_MAX3421E_SPD 25000000
#endif
#endif
#else
// We start at 25MHz, and back down until hardware can take it.//我们从25MHz开始，然后后退，直到硬件能够接收。
// Of course, SPI library can adjust this for us too.//当然，SPI库也可以为我们调整。
// Why not 26MHz? Because I have not found any MCU board that//为什么不是26MHz？因为我没有发现任何MCU板
// can actually go that fast without problems.//可以毫无问题地跑那么快。
// Could be a shield limitation too.//也可能是一个屏蔽限制。
#ifndef UHS_MAX3421E_SPD
#define UHS_MAX3421E_SPD 25000000
#endif
#endif

#ifndef UHS_MAX3421E_INT
#define UHS_MAX3421E_INT UHS_MAX3421E_INT_
#endif

#ifndef UHS_MAX3421E_SS
#define UHS_MAX3421E_SS UHS_MAX3421E_SS_
#endif

// NOTE: On the max3421e the irq enable and irq bits are in the same position.//注意：在max3421e上，irq启用位和irq位位于同一位置。

// IRQs used if CPU polls//如果CPU轮询，则使用IRQ
#define   ENIBITSPOLLED (bmCONDETIE | bmBUSEVENTIE  | bmFRAMEIE)
// IRQs used if CPU is interrupted//如果CPU中断，则使用IRQ
#define      ENIBITSISR (bmCONDETIE | bmBUSEVENTIE | bmFRAMEIE /* | bmRCVDAVIRQ | bmSNDBAVIRQ | bmHXFRDNIRQ */ )

#if !USB_HOST_SHIELD_USE_ISR
#define IRQ_CHECK_MASK (ENIBITSPOLLED & ICLRALLBITS)
#define IRQ_IS_EDGE 0
#else
#define IRQ_CHECK_MASK (ENIBITSISR & ICLRALLBITS)
#endif

#if IRQ_IS_EDGE
// Note: UNO32 Interrupts can only be RISING, or FALLING.//注意：UNO32中断只能是上升或下降。
// This poses an interesting problem, since we want to use a LOW level.//这带来了一个有趣的问题，因为我们希望使用低级别。
// The MAX3421E provides for pulse width control for an IRQ.//MAX3421E为IRQ提供脉冲宽度控制。
// We do need to watch the timing on this, as a second IRQ could cause//我们确实需要关注这方面的时机，因为第二次IRQ可能会导致
// a missed IRQ, since we read the level of the line to check if the IRQ//缺少IRQ，因为我们读取行的级别以检查IRQ
// is actually for this chip. The only other alternative is to add a capacitor//实际上是为了这个芯片。唯一的其他选择是增加一个电容器
// and an NPN transistor, and use two lines. We can try this first, though.//和一个NPN晶体管，使用两条线。不过，我们可以先试试这个。
// Worse case, we can ignore reading the pin for verification on UNO32.//更糟糕的情况是，我们可以忽略读取UNO32上的pin进行验证。
// Too bad there is no minimum low width setting.//太糟糕了，没有最小低宽度设置。
////
//   Single    Clear     First  Second   Clear first      Clear last//单次清除第一次清除第二次清除第一次清除最后一次
//   IRQ       Single    IRQ    IRQ      Second active    pending IRQ//IRQ单个IRQ IRQ第二个激活待处理IRQ
//      |      |         |      |        |                |//      |      |         |      |        |                |
//      V      V         V      V        V                V//V V V V V
// _____        _________        _        _                _______// _____        _________        _        _                _______
//      |______|         |______| |______| |______________|//      |______|         |______| |______| |______________|
////
#define IRQ_SENSE FALLING
#ifdef ARDUINO_ARCH_PIC32
//#define bmPULSEWIDTH PUSLEWIDTH10_6//#定义bmPULSEWIDTH puslewidth 10_6
#define bmPULSEWIDTH 0
#define bmIRQ_SENSE 0
#else
#define bmPULSEWIDTH PUSLEWIDTH1_3
#define bmIRQ_SENSE 0
#endif
#else
#ifndef IRQ_SENSE
#define IRQ_SENSE LOW
#endif
#ifndef bmPULSEWIDTH
#define bmPULSEWIDTH 0
#endif
#ifndef bmIRQ_SENSE
#define bmIRQ_SENSE bmINTLEVEL
#endif
#endif

class MAX3421E_HOST :
public UHS_USB_HOST_BASE
#ifdef SWI_IRQ_NUM
, public dyn_SWI
#endif
{
        // TO-DO: move these into the parent class.//待办事项：将它们移动到父类中。
        volatile uint8_t vbusState;
        volatile uint16_t sof_countdown;

        // TO-DO: pack into a struct/union and use one byte//待办事项：打包到结构/联合中并使用一个字节
        volatile bool busevent;
        volatile bool sofevent;
        volatile bool counted;
        volatile bool condet;
        volatile bool doingreset;

        #ifdef USB_HOST_MANUAL_POLL
                volatile bool frame_irq_enabled = false;

                bool enable_frame_irq(bool enable) {
                        const bool prev_state = frame_irq_enabled;
                        if(prev_state != enable) {
                                if(enable)
                                        regWr(rHIEN, regRd(rHIEN) |  bmFRAMEIE);
                                else
                                        regWr(rHIEN, regRd(rHIEN) & ~bmFRAMEIE);
                                frame_irq_enabled = enable;
                        }
                        return prev_state;
                }
        #endif

public:
        SPISettings MAX3421E_SPI_Settings;
        uint8_t ss_pin;
        uint8_t irq_pin;
        // Will use the defaults UHS_MAX3421E_SS, UHS_MAX3421E_INT and speed//将使用默认值UHS_MAX3421E_SS、UHS_MAX3421E_INT和速度

        UHS_NI MAX3421E_HOST() {
                sof_countdown = 0;
                busevent = false;
                doingreset = false;
                sofevent = false;
                condet = false;
                ss_pin = UHS_MAX3421E_SS;
                irq_pin = UHS_MAX3421E_INT;
                MAX3421E_SPI_Settings = SPISettings(UHS_MAX3421E_SPD, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        // Will use user supplied pins, and UHS_MAX3421E_SPD//将使用用户提供的引脚和UHS_MAX3421E_SPD

        UHS_NI MAX3421E_HOST(uint8_t pss, uint8_t pirq) {
                sof_countdown = 0;
                busevent = false;
                doingreset = false;
                sofevent = false;
                condet = false;
                ss_pin = pss;
                irq_pin = pirq;
                MAX3421E_SPI_Settings = SPISettings(UHS_MAX3421E_SPD, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        // Will use user supplied pins, and speed//将使用用户提供的引脚和速度

        UHS_NI MAX3421E_HOST(uint8_t pss, uint8_t pirq, uint32_t pspd) {
                sof_countdown = 0;
                doingreset = false;
                busevent = false;
                sofevent = false;
                condet = false;
                ss_pin = pss;
                irq_pin = pirq;
                MAX3421E_SPI_Settings = SPISettings(pspd, MSBFIRST, SPI_MODE0);
                hub_present = 0;
        };

        virtual bool UHS_NI sof_delay(uint16_t x) {
#ifdef USB_HOST_MANUAL_POLL
                const bool saved_irq_state = enable_frame_irq(true);
#endif
                sof_countdown = x;
                while((sof_countdown != 0) && !condet) {
                        SYSTEM_OR_SPECIAL_YIELD();
#if !USB_HOST_SHIELD_USE_ISR
                        Task();
#endif
                }
#ifdef USB_HOST_MANUAL_POLL
                enable_frame_irq(saved_irq_state);
#endif
                //                Serial.println("...Wake");//Serial.println（“…唤醒”）；
                return (!condet);
        };

        virtual UHS_EpInfo *ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr);

        virtual void UHS_NI vbusPower(VBUS_t state) {
                regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE) | (uint8_t)(state));
        };

        void UHS_NI Task();

        virtual uint8_t SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit);
        virtual uint8_t OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data);
        virtual uint8_t InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data);
        virtual uint8_t ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr);
        virtual uint8_t ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr);
        virtual uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

        void UHS_NI ReleaseChildren() {
                for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
                        if(devConfig[i])
                                devConfig[i]->Release();
                hub_present = 0;
        };

        virtual bool IsHub(uint8_t klass) {
                if(klass == UHS_USB_CLASS_HUB) {
                        hub_present = bmHUBPRE;
                        return true;
                }
                return false;
        };

        virtual void VBUS_changed();

        virtual void UHS_NI doHostReset() {
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts//启用中断
                noInterrupts();
#endif
                doingreset = true;
                busevent = true;
                regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.//见数据表。
                regWr(rHCTL, bmBUSRST); //issue bus reset//发布总线重置
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts//启用中断
                interrupts();
#endif
                while(busevent) {
                        DDSB();
                        SYSTEM_OR_SPECIAL_YIELD();
                }
#endif
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts//启用中断
                noInterrupts();
#endif
                #ifdef USB_HOST_MANUAL_POLL
                        enable_frame_irq(true);
                #endif
                sofevent = true;
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts//启用中断
                interrupts();
#endif
                // Wait for SOF//等待SOF
                while(sofevent) {
                }
#if USB_HOST_SHIELD_USE_ISR
                // Enable interrupts//启用中断
                noInterrupts();
#endif
                doingreset = false;
#if USB_HOST_SHIELD_USE_ISR
                DDSB();
                // Enable interrupts//启用中断
                interrupts();
        };


        int16_t UHS_NI Init(int16_t mseconds);

        int16_t UHS_NI Init() {
                return Init(INT16_MIN);
        };

        void ISRTask();
        void ISRbottom();
        void busprobe();
        uint16_t reset();

        // MAX3421e specific//MAX3421e特定
        void regWr(uint8_t reg, uint8_t data);
        void gpioWr(uint8_t data);
        uint8_t regRd(uint8_t reg);
        uint8_t gpioRd();
        uint8_t* bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p);
        uint8_t* bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p);

        // ARM/NVIC specific, used to emulate reentrant ISR.//特定于ARM/NVIC，用于模拟可重入ISR。
#ifdef SWI_IRQ_NUM

        void dyn_SWISR() {
                ISRbottom();
        };
#endif

        virtual void UHS_NI suspend_host() {
                // Used on MCU that lack control of IRQ priority (AVR).//用于缺少IRQ优先级（AVR）控制的MCU。
                // Suspends ISRs, for critical code. IRQ will be serviced after it is resumed.//暂停关键代码的ISR。IRQ将在恢复后进行维修。
                // NOTE: you must track the state yourself!//注意：您必须自己跟踪状态！
#ifdef __AVR__
                noInterrupts();
                detachInterrupt(UHS_GET_DPI(irq_pin));
                interrupts();
#endif
        };

        virtual void UHS_NI resume_host();
};
#ifndef SPIclass
#define SPIclass SPI
#endif
#ifndef USB_HOST_SHIELD_LOADED
#include "USB_HOST_SHIELD_INLINE.h"
#endif
#else
#error "define LOAD_USB_HOST_SHIELD in your sketch, never include USB_HOST_SHIELD.h in a driver."
#endif
#endif /* USB_HOST_SHIELD_H */
