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

#include <stdint.h>

/**
 * This file contains the custom port/pin definitions for the PRINTRBOARD_G2
 * motherboard. This motherboard uses the SAM3X8C which is a subset of the
 * SAM3X8E used in the DUE board.  It uses port/pin pairs that are not
 * available using the DUE definitions.
 *
 * The first part is a copy of the pin descriptions in the
 * "variants\arduino_due_x\variant.cpp" file but with pins 34-41 replaced by
 * the G2 pins.
 *
 * The second part is the FASTIO port/pin definitions.
 *
 * THESE PINS CAN ONLY BE ACCESSED VIA FASTIO COMMANDS.
 */

/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

typedef struct _G2_PinDescription {
  Pio* pPort;
  uint32_t ulPin;
  uint32_t ulPeripheralId;
  EPioType ulPinType;
  uint32_t ulPinConfiguration;
  uint32_t ulPinAttribute;
  EAnalogChannel ulAnalogChannel; /* Analog pin in the Arduino context (label on the board) */
  EAnalogChannel ulADCChannelNumber; /* ADC Channel number in the SAM device */
  EPWMChannel ulPWMChannel;
  ETCChannel ulTCChannel;
} G2_PinDescription;

/**
 * This section is a copy of the pin descriptions in the "variants\arduino_due_x\variant.cpp" file
 * with pins 34-41 replaced by the G2 pins.
 */

/**
 * Pins descriptions
 */
const G2_PinDescription G2_g_APinDescription[] = {
  // 0 .. 53 - Digital pins// 0 .. 53-数字管脚
  // ----------------------// ----------------------
  // 0/1 - UART (Serial)//0/1-UART（串行）
  { PIOA, PIO_PA8A_URXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // URXD//URXD
  { PIOA, PIO_PA9A_UTXD,     ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT,  PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // UTXD//UTXD

  // 2// 2
  { PIOB, PIO_PB25B_TIOA0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHA0     }, // TIOA0//TIOA0
  { PIOC, PIO_PC28B_TIOA7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA7     }, // TIOA7//TIOA7
  { PIOC, PIO_PC26B_TIOB6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB6     }, // TIOB6//TIOB6

  // 5// 5
  { PIOC, PIO_PC25B_TIOA6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA6     }, // TIOA6//TIOA6
  { PIOC, PIO_PC24B_PWML7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH7,     NOT_ON_TIMER }, // PWML7//PWML7
  { PIOC, PIO_PC23B_PWML6,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH6,     NOT_ON_TIMER }, // PWML6//PWML6
  { PIOC, PIO_PC22B_PWML5,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH5,     NOT_ON_TIMER }, // PWML5//PWML5
  { PIOC, PIO_PC21B_PWML4,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH4,     NOT_ON_TIMER }, // PWML4//PWML4
  // 10// 10
  { PIOC, PIO_PC29B_TIOB7,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB7     }, // TIOB7//TIOB7
  { PIOD, PIO_PD7B_TIOA8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHA8     }, // TIOA8//TIOA8
  { PIOD, PIO_PD8B_TIOB8,    ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC2_CHB8     }, // TIOB8//TIOB8

  // 13 - AMBER LED//13-琥珀色发光二极管
  { PIOB, PIO_PB27B_TIOB0,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,  TC0_CHB0     }, // TIOB0//TIOB0

  // 14/15 - USART3 (Serial3)//14/15-USART3（连载3）
  { PIOD, PIO_PD4B_TXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD3//TXD3
  { PIOD, PIO_PD5B_RXD3,     ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD3//RXD3

  // 16/17 - USART1 (Serial2)//17月16日-USART1（连载2）
  { PIOA, PIO_PA13A_TXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD1//TXD1
  { PIOA, PIO_PA12A_RXD1,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD1//RXD1

  // 18/19 - USART0 (Serial1)//18/19-USART0（连载1）
  { PIOA, PIO_PA11A_TXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TXD0//TXD0
  { PIOA, PIO_PA10A_RXD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // RXD0//RXD0

  // 20/21 - TWI1//20/21-1
  { PIOB, PIO_PB12A_TWD1,    ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD1 - SDA0//TWD1-SDA0
  { PIOB, PIO_PB13A_TWCK1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK1 - SCL0//TWCK1-SCL0

  // 22// 22
  { PIOB, PIO_PB26,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 22//引脚22
  { PIOA, PIO_PA14,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 23//引脚23
  { PIOA, PIO_PA15,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 24//引脚24
  { PIOD, PIO_PD0,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 25//引脚25

  // 26// 26
  { PIOD, PIO_PD1,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 26//针脚26
  { PIOD, PIO_PD2,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 27//引脚27
  { PIOD, PIO_PD3,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 28//针脚28
  { PIOD, PIO_PD6,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 29//引脚29

  // 30// 30
  { PIOD, PIO_PD9,           ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 30//引脚30
  { PIOA, PIO_PA7,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 31//引脚31
  { PIOD, PIO_PD10,          ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 32//引脚32
  { PIOC, PIO_PC1,           ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 33//引脚33

  // 34// 34

  // start of custom pins//自定义PIN的开始
  { PIOA, PIO_PA29,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 34  Y_STEP_PIN//针脚34 Y_阶梯_针脚
  { PIOB, PIO_PB1,           ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 35  Y_DIR_PIN//引脚35 Y_方向引脚
  { PIOB, PIO_PB0,           ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 36  Y_ENABLE_PIN//引脚36 Y_启用_引脚
  { PIOB, PIO_PB22,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 37  E0_ENABLE_PIN//引脚37 E0_启用_引脚
  { PIOB, PIO_PB11,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 38  E0_MS1_PIN//引脚38 E0_MS1_引脚
  { PIOB, PIO_PB10,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 39  E0_MS3_PIN//引脚39 E0_MS3_引脚
  { PIOA, PIO_PA5,           ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 40  HEATER_0_PIN//引脚40加热器0引脚
  { PIOB, PIO_PB24,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 41  HEATER_BED_PIN//针脚41加热器\床\针脚
  // end of custom pins//自定义端号

  // 42// 42
  { PIOA, PIO_PA19,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 42//引脚42
  { PIOA, PIO_PA20,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 43//引脚43
  { PIOC, PIO_PC19,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 44//引脚44
  { PIOC, PIO_PC18,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 45//引脚45

  // 46// 46
  { PIOC, PIO_PC17,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 46//针脚46
  { PIOC, PIO_PC16,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 47//引脚47
  { PIOC, PIO_PC15,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 48//针脚48
  { PIOC, PIO_PC14,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 49//引脚49

  // 50// 50
  { PIOC, PIO_PC13,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 50//针脚50
  { PIOC, PIO_PC12,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 51//引脚51
  { PIOB, PIO_PB21,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 52//针脚52
  { PIOB, PIO_PB14,          ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 53//插脚53


  // 54 .. 65 - Analog pins// 54 .. 65-模拟引脚
  // ----------------------// ----------------------
  { PIOA, PIO_PA16X1_AD7,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0//AD0
  { PIOA, PIO_PA24X1_AD6,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD1//AD1
  { PIOA, PIO_PA23X1_AD5,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD2//AD2
  { PIOA, PIO_PA22X1_AD4,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD3//AD3
  // 58// 58
  { PIOA, PIO_PA6X1_AD3,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC3,   NOT_ON_PWM,  TC0_CHB2     }, // AD4//AD4
  { PIOA, PIO_PA4X1_AD2,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD5//AD5
  { PIOA, PIO_PA3X1_AD1,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC1,   NOT_ON_PWM,  TC0_CHB1     }, // AD6//AD6
  { PIOA, PIO_PA2X1_AD0,     ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC0,   NOT_ON_PWM,  TC0_CHA1     }, // AD7//公元7年
  // 62// 62
  { PIOB, PIO_PB17X1_AD10,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC8,   ADC10,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD8//公元8年
  { PIOB, PIO_PB18X1_AD11,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC9,   ADC11,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD9//AD9
  { PIOB, PIO_PB19X1_AD12,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC10,  ADC12,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD10//公元10年
  { PIOB, PIO_PB20X1_AD13,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC11,  ADC13,  NOT_ON_PWM,  NOT_ON_TIMER }, // AD11//公元11年

  // 66/67 - DAC0/DAC1//66/67-DAC0/DAC1
  { PIOB, PIO_PB15X1_DAC0,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC12,  DA0,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC0//DAC0
  { PIOB, PIO_PB16X1_DAC1,   ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC13,  DA1,    NOT_ON_PWM,  NOT_ON_TIMER }, // DAC1//DAC1

  // 68/69 - CANRX0/CANTX0//68/69-CANRX0/CANTX0
  { PIOA, PIO_PA1A_CANRX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC14,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX//CANRX
  { PIOA, PIO_PA0A_CANTX0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  ADC15,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX//坎茨

  // 70/71 - TWI0//70/71-0
  { PIOA, PIO_PA17A_TWD0,    ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA1//TWD0-SDA1
  { PIOA, PIO_PA18A_TWCK0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL1//TWCK0-SCL1

  // 72/73 - LEDs//72/73-发光二极管
  { PIOC, PIO_PC30,          ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED AMBER RXL//LED琥珀色RXL
  { PIOA, PIO_PA21,          ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED AMBER TXL//LED琥珀色TXL

  // 74/75/76 - SPI//74/75/76-SPI
  { PIOA, PIO_PA25A_SPI0_MISO,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MISO//味噌
  { PIOA, PIO_PA26A_SPI0_MOSI,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MOSI//摩西
  { PIOA, PIO_PA27A_SPI0_SPCK,ID_PIOA,PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPCK//斯普克

  // 77 - SPI CS0//77-SPI CS0
  { PIOA, PIO_PA28A_SPI0_NPCS0,ID_PIOA,PIO_PERIPH_A,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS0//NPCS0

  // 78 - SPI CS3 (unconnected)//78-SPI CS3（未连接）
  { PIOB, PIO_PB23B_SPI0_NPCS3,ID_PIOB,PIO_PERIPH_B,PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS3//NPCS3

  // 79 .. 84 - "All pins" masks// 79 .. 84-“所有针”遮罩

  // 79 - TWI0 all pins//79-TWI0所有引脚
  { PIOA, PIO_PA17A_TWD0|PIO_PA18A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 80 - TWI1 all pins//80-1所有引脚
  { PIOB, PIO_PB12A_TWD1|PIO_PB13A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 81 - UART (Serial) all pins//81-UART（串行）所有引脚
  { PIOA, PIO_PA8A_URXD|PIO_PA9A_UTXD, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 82 - USART0 (Serial1) all pins//82-USART0（串行1）所有引脚
  { PIOA, PIO_PA11A_TXD0|PIO_PA10A_RXD0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 83 - USART1 (Serial2) all pins//83-USART1（串行2）所有引脚
  { PIOA, PIO_PA13A_TXD1|PIO_PA12A_RXD1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 84 - USART3 (Serial3) all pins//84-USART3（串行3）所有引脚
  { PIOD, PIO_PD4B_TXD3|PIO_PD5B_RXD3, ID_PIOD, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // 85 - USB//85-USB
  { PIOB, PIO_PB11A_UOTGID|PIO_PB10A_UOTGVBOF, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ID - VBOF//ID-VBOF

  // 86 - SPI CS2//86-SPI CS2
  { PIOB, PIO_PB21B_SPI0_NPCS2, ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS2//NPCS2

  // 87 - SPI CS1//87-SPI CS1
  { PIOA, PIO_PA29A_SPI0_NPCS1, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS1//NPCS1

  // 88/89 - CANRX1/CANTX1 (same physical pin for 66/53)//88/89-CANRX1/CANTX1（66/53的物理引脚相同）
  { PIOB, PIO_PB15A_CANRX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANRX1//CANRX1
  { PIOB, PIO_PB14A_CANTX1,     ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // CANTX1//CANTX1

  // 90 .. 91 - "All CAN pins" masks// 90 .. 91-“所有CAN引脚”遮罩
  // 90 - CAN0 all pins//90-CAN0所有引脚
  { PIOA, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
  // 91 - CAN1 all pins//91-CAN1所有引脚
  { PIOB, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },

  // END//结束
  { nullptr, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
};

// This section replaces the FASTIO definitions of pins 34-41//本节取代针脚34-41的FASTIO定义

#define DIO34_PIN 29
#define DIO34_WPORT PIOA  // only available via FASTIO      //       34 PA29 - Y_STEP_PIN//仅可通过FASTIO//34 PA29-Y_步骤_引脚获得

#define DIO35_PIN 1
#define DIO35_WPORT PIOB  // only available via FASTIO      //       35 PAB1 - Y_DIR_PIN//仅可通过FASTIO//35 PAB1-Y_DIR_引脚获得

#define DIO36_PIN 0
#define DIO36_WPORT PIOB  // only available via FASTIO      //       36 PB0 - Y_ENABLE_PIN//仅通过FASTIO//36 PB0-Y_启用_引脚提供

#define DIO37_PIN 22
#define DIO37_WPORT PIOB  // only available via FASTIO      //       37 PB22 - E0_ENABLE_PIN//仅通过FASTIO//37 PB22-E0_启用_引脚提供

#define DIO38_PIN 11
#define DIO38_WPORT PIOB  // only available via FASTIO      //       38 PB11 - E0_MS1_PIN//仅通过FASTIO//38 PB11-E0_MS1_引脚提供

#define DIO39_PIN 10
#define DIO39_WPORT PIOB  // only available via FASTIO      //       39 PB10 - E0_MS3_PIN//仅通过FASTIO//39 PB10-E0_MS3_引脚提供

#define DIO40_PIN 5
#define DIO40_WPORT PIOA  // only available via FASTIO      //       40 PA5  - HEATER_0_PIN//仅通过FASTIO//40 PA5-加热器0针提供

#define DIO41_PIN 24
#define DIO41_WPORT PIOB  // only available via FASTIO      //       41 PB24 - HEATER_BED_PIN//仅可通过FASTIO//41 PB24-加热器\u床\u引脚提供
