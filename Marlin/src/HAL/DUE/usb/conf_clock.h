/** translatione by yx */
/**
 * \file
 *
 * \brief SAM3X clock configuration.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.atmel.com/design-support/">Atmel Support</a>//www.atmel.com/design support/“>atmel支持</a>
 */

#ifndef CONF_CLOCK_H_INCLUDED
#define CONF_CLOCK_H_INCLUDED

// ===== System Clock (MCK) Source Options//====系统时钟（MCK）源选项
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_SLCK_RC//#定义配置\u系统时钟\u源系统时钟\u SRC\u SLCK\u RC
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_SLCK_XTAL//#定义配置\u系统时钟\u源系统时钟\u SRC\u SLCK\u XTAL
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_SLCK_BYPASS//#定义配置\系统时钟\源系统时钟\ SRC\ SLCK\旁路
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_MAINCK_4M_RC//#定义配置\u系统时钟\u源系统时钟\u SRC\u MAINCK\u 4M\u RC
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_MAINCK_8M_RC//#定义配置\u系统时钟\u源系统时钟\u SRC\u MAINCK\u 8M\u RC
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_MAINCK_12M_RC//#定义配置\u系统时钟\u源系统时钟\u SRC\u MAINCK\u 12M\u RC
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_MAINCK_XTAL//#定义配置\u系统时钟\u源系统时钟\u SRC\u MAINCK\u XTAL
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_MAINCK_BYPASS//#定义配置\u系统时钟\u源系统时钟\u SRC\u主时钟\u旁路
#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_PLLACK
//#define CONFIG_SYSCLK_SOURCE        SYSCLK_SRC_UPLLCK//#定义配置\系统时钟\源系统时钟\ SRC\ UPLLCK

// ===== System Clock (MCK) Prescaler Options   (Fmck = Fsys / (SYSCLK_PRES))//====系统时钟（MCK）预分频器选项（Fmck=Fsys/（系统时钟预分频器））
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_1//#定义配置\u系统时钟\u压力系统时钟\u压力\u 1
#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_2
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_4//#定义配置\u系统时钟\u压力系统时钟\u压力4
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_8//#定义配置\u系统时钟\u压力系统时钟\u压力8
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_16//#定义配置\u系统时钟\u压力系统时钟\u压力16
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_32//#定义配置\u系统时钟\u压力系统时钟\u压力\u 32
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_64//#定义配置\u系统时钟\u压力系统时钟\u压力64
//#define CONFIG_SYSCLK_PRES          SYSCLK_PRES_3//#定义配置\u系统时钟\u压力系统时钟\u压力\u 3

// ===== PLL0 (A) Options   (Fpll = (Fclk * PLL_mul) / PLL_div)//====PLL0（A）选项（Fpll=（Fclk*PLL\U mul）/PLL\U div）
// Use mul and div effective values here.//在此处使用mul和div有效值。
#define CONFIG_PLL0_SOURCE          PLL_SRC_MAINCK_XTAL
#define CONFIG_PLL0_MUL             14
#define CONFIG_PLL0_DIV             1

// ===== UPLL (UTMI) Hardware fixed at 480MHz.//======固定在480MHz的UPLL（UTMI）硬件。

// ===== USB Clock Source Options   (Fusb = FpllX / USB_div)//====USB时钟源选项（Fusb=FpllX/USB_div）
// Use div effective value here.//在这里使用div有效值。
//#define CONFIG_USBCLK_SOURCE        USBCLK_SRC_PLL0//#定义配置\u USBCLK\u源USBCLK\u SRC\u PLL0
#define CONFIG_USBCLK_SOURCE        USBCLK_SRC_UPLL
#define CONFIG_USBCLK_DIV           1

// ===== Target frequency (System clock)//====目标频率（系统时钟）
// - XTAL frequency: 12MHz//-XTAL频率：12MHz
// - System clock source: PLLA//-系统时钟源：PLLA
// - System clock prescaler: 2 (divided by 2)//-系统时钟预分频器：2（除以2）
// - PLLA source: XTAL//-PLLA来源：XTAL
// - PLLA output: XTAL * 14 / 1//-PLLA输出：XTAL*14/1
// - System clock is: 12 * 14 / 1 /2 = 84MHz//-系统时钟为：12*14/1/2=84MHz
// ===== Target frequency (USB Clock)//====目标频率（USB时钟）
// - USB clock source: UPLL//-USB时钟源：UPLL
// - USB clock divider: 1 (not divided)//-USB时钟分频器：1（未分频）
// - UPLL frequency: 480MHz//-UPLL频率：480MHz
// - USB clock: 480 / 1 = 480MHz//-USB时钟：480/1=480MHz


#endif /* CONF_CLOCK_H_INCLUDED */
