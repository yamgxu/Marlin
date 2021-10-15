/** translatione by yx */
/**
 * \file
 *
 * \brief USB Communication Device Class (CDC) protocol definitions
 *
 * Copyright (c) 2009-2015 Atmel Corporation. All rights reserved.
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
#ifndef _USB_PROTOCOL_CDC_H_
#define _USB_PROTOCOL_CDC_H_

#include "compiler.h"

/**
 * \ingroup usb_protocol_group
 * \defgroup cdc_protocol_group Communication Device Class Definitions
 * @{
 */

/**
 * \name Possible values of class
 */
//@{//@{
#define  CDC_CLASS_DEVICE     0x02	//!< USB Communication Device Class//！<USB通信设备类
#define  CDC_CLASS_COMM       0x02	//!< CDC Communication Class Interface//！<CDC通信类接口
#define  CDC_CLASS_DATA       0x0A	//!< CDC Data Class Interface//！<CDC数据类接口
#define  CDC_CLASS_MULTI      0xEF      //!< CDC Multi-interface Function//！<CDC多接口功能

//@}//@}

//! \name USB CDC Subclass IDs//！\name USB CDC子类ID
//@{//@{
#define  CDC_SUBCLASS_DLCM    0x01	//!< Direct Line Control Model//！<直接线路控制模型
#define  CDC_SUBCLASS_ACM     0x02	//!< Abstract Control Model//抽象控制模型
#define  CDC_SUBCLASS_TCM     0x03	//!< Telephone Control Model//！<电话控制模型
#define  CDC_SUBCLASS_MCCM    0x04	//!< Multi-Channel Control Model//！<多通道控制模型
#define  CDC_SUBCLASS_CCM     0x05	//!< CAPI Control Model//！<CAPI控制模型
#define  CDC_SUBCLASS_ETH     0x06	//!< Ethernet Networking Control Model//！<以太网网络控制模型
#define  CDC_SUBCLASS_ATM     0x07	//!< ATM Networking Control Model//！<ATM网络控制模型
//@}//@}

//! \name USB CDC Communication Interface Protocol IDs//！\name USB CDC通信接口协议ID
//@{//@{
#define  CDC_PROTOCOL_V25TER  0x01	//!< Common AT commands//！<通用AT命令
//@}//@}

//! \name USB CDC Data Interface Protocol IDs//！\name USB CDC数据接口协议ID
//@{//@{
#define  CDC_PROTOCOL_I430    0x30	//!< ISDN BRI//！<ISDN BRI
#define  CDC_PROTOCOL_HDLC    0x31	//!< HDLC//！<HDLC
#define  CDC_PROTOCOL_TRANS   0x32	//!< Transparent//！<透明
#define  CDC_PROTOCOL_Q921M   0x50	//!< Q.921 management protocol//！<Q.921管理协议
#define  CDC_PROTOCOL_Q921    0x51	//!< Q.931 [sic] Data link protocol//！<Q.931[sic]数据链路协议
#define  CDC_PROTOCOL_Q921TM  0x52	//!< Q.921 TEI-multiplexor//！<Q.921 TEI多路复用器
#define  CDC_PROTOCOL_V42BIS  0x90	//!< Data compression procedures//！<数据压缩程序
#define  CDC_PROTOCOL_Q931    0x91	//!< Euro-ISDN protocol control//！<欧洲ISDN协议控制
#define  CDC_PROTOCOL_V120    0x92	//!< V.24 rate adaption to ISDN//！<V.24速率自适应到ISDN
#define  CDC_PROTOCOL_CAPI20  0x93	//!< CAPI Commands//！<CAPI命令
#define  CDC_PROTOCOL_HOST    0xFD	//!< Host based driver//！<基于主机的驱动程序
/**
 * \brief Describes the Protocol Unit Functional Descriptors [sic]
 * on Communication Class Interface
 */
#define  CDC_PROTOCOL_PUFD    0xFE
//@}//@}

//! \name USB CDC Functional Descriptor Types//！\name USB CDC功能描述符类型
//@{//@{
#define  CDC_CS_INTERFACE     0x24	//!< Interface Functional Descriptor//！<接口功能描述符
#define  CDC_CS_ENDPOINT      0x25	//!< Endpoint Functional Descriptor//端点函数描述符
//@}//@}

//! \name USB CDC Functional Descriptor Subtypes//！\name USB CDC功能描述符子类型
//@{//@{
#define  CDC_SCS_HEADER       0x00	//!< Header Functional Descriptor//头函数描述符
#define  CDC_SCS_CALL_MGMT    0x01	//!< Call Management//！<呼叫管理
#define  CDC_SCS_ACM          0x02	//!< Abstract Control Management//！<抽象控制管理
#define  CDC_SCS_UNION        0x06	//!< Union Functional Descriptor//联合函数描述符
//@}//@}

//! \name USB CDC Request IDs//！\name USB CDC请求ID
//@{//@{
#define  USB_REQ_CDC_SEND_ENCAPSULATED_COMMAND                   0x00
#define  USB_REQ_CDC_GET_ENCAPSULATED_RESPONSE                   0x01
#define  USB_REQ_CDC_SET_COMM_FEATURE                            0x02
#define  USB_REQ_CDC_GET_COMM_FEATURE                            0x03
#define  USB_REQ_CDC_CLEAR_COMM_FEATURE                          0x04
#define  USB_REQ_CDC_SET_AUX_LINE_STATE                          0x10
#define  USB_REQ_CDC_SET_HOOK_STATE                              0x11
#define  USB_REQ_CDC_PULSE_SETUP                                 0x12
#define  USB_REQ_CDC_SEND_PULSE                                  0x13
#define  USB_REQ_CDC_SET_PULSE_TIME                              0x14
#define  USB_REQ_CDC_RING_AUX_JACK                               0x15
#define  USB_REQ_CDC_SET_LINE_CODING                             0x20
#define  USB_REQ_CDC_GET_LINE_CODING                             0x21
#define  USB_REQ_CDC_SET_CONTROL_LINE_STATE                      0x22
#define  USB_REQ_CDC_SEND_BREAK                                  0x23
#define  USB_REQ_CDC_SET_RINGER_PARMS                            0x30
#define  USB_REQ_CDC_GET_RINGER_PARMS                            0x31
#define  USB_REQ_CDC_SET_OPERATION_PARMS                         0x32
#define  USB_REQ_CDC_GET_OPERATION_PARMS                         0x33
#define  USB_REQ_CDC_SET_LINE_PARMS                              0x34
#define  USB_REQ_CDC_GET_LINE_PARMS                              0x35
#define  USB_REQ_CDC_DIAL_DIGITS                                 0x36
#define  USB_REQ_CDC_SET_UNIT_PARAMETER                          0x37
#define  USB_REQ_CDC_GET_UNIT_PARAMETER                          0x38
#define  USB_REQ_CDC_CLEAR_UNIT_PARAMETER                        0x39
#define  USB_REQ_CDC_GET_PROFILE                                 0x3A
#define  USB_REQ_CDC_SET_ETHERNET_MULTICAST_FILTERS              0x40
#define  USB_REQ_CDC_SET_ETHERNET_POWER_MANAGEMENT_PATTERNFILTER 0x41
#define  USB_REQ_CDC_GET_ETHERNET_POWER_MANAGEMENT_PATTERNFILTER 0x42
#define  USB_REQ_CDC_SET_ETHERNET_PACKET_FILTER                  0x43
#define  USB_REQ_CDC_GET_ETHERNET_STATISTIC                      0x44
#define  USB_REQ_CDC_SET_ATM_DATA_FORMAT                         0x50
#define  USB_REQ_CDC_GET_ATM_DEVICE_STATISTICS                   0x51
#define  USB_REQ_CDC_SET_ATM_DEFAULT_VC                          0x52
#define  USB_REQ_CDC_GET_ATM_VC_STATISTICS                       0x53
// Added bNotification codes according cdc spec 1.1 chapter 6.3//根据cdc规范1.1第6.3章增加了通知代码
#define  USB_REQ_CDC_NOTIFY_RING_DETECT                          0x09
#define  USB_REQ_CDC_NOTIFY_SERIAL_STATE                         0x20
#define  USB_REQ_CDC_NOTIFY_CALL_STATE_CHANGE                    0x28
#define  USB_REQ_CDC_NOTIFY_LINE_STATE_CHANGE                    0x29
//@}//@}

/*
 * Need to pack structures tightly, or the compiler might insert padding
 * and violate the spec-mandated layout.
 */
COMPILER_PACK_SET(1)

//! \name USB CDC Descriptors//！\name USB CDC描述符
//@{//@{


//! CDC Header Functional Descriptor//！CDC头函数描述符
typedef struct {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	le16_t bcdCDC;
} usb_cdc_hdr_desc_t;

//! CDC Call Management Functional Descriptor//！CDC呼叫管理功能描述符
typedef struct {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmCapabilities;
	uint8_t bDataInterface;
} usb_cdc_call_mgmt_desc_t;

//! CDC ACM Functional Descriptor//！CDC ACM功能描述符
typedef struct {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmCapabilities;
} usb_cdc_acm_desc_t;

//! CDC Union Functional Descriptor//！CDC联合函数描述符
typedef struct {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bMasterInterface;
	uint8_t bSlaveInterface0;
} usb_cdc_union_desc_t;


//! \name USB CDC Call Management Capabilities//！\name USB CDC呼叫管理功能
//@{//@{
//! Device handles call management itself//！设备本身处理呼叫管理
#define  CDC_CALL_MGMT_SUPPORTED             (1 << 0)
//! Device can send/receive call management info over a Data Class interface//！设备可以通过数据类接口发送/接收呼叫管理信息
#define  CDC_CALL_MGMT_OVER_DCI              (1 << 1)
//@}//@}

//! \name USB CDC ACM Capabilities//！\name USB CDC ACM功能
//@{//@{
//! Device supports the request combination of//！设备支持以下请求组合：
//! Set_Comm_Feature, Clear_Comm_Feature, and Get_Comm_Feature.//！设置通信功能、清除通信功能和获取通信功能。
#define  CDC_ACM_SUPPORT_FEATURE_REQUESTS    (1 << 0)
//! Device supports the request combination of//！设备支持以下请求组合：
//! Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding,//！设置线路编码，设置控制线路状态，获取线路编码，
//! and the notification Serial_State.//！和通知序列状态。
#define  CDC_ACM_SUPPORT_LINE_REQUESTS       (1 << 1)
//! Device supports the request Send_Break//！设备支持请求发送\u中断
#define  CDC_ACM_SUPPORT_SENDBREAK_REQUESTS  (1 << 2)
//! Device supports the notification Network_Connection.//！设备支持通知网络\u连接。
#define  CDC_ACM_SUPPORT_NOTIFY_REQUESTS     (1 << 3)
//@}//@}
//@}//@}

//! \name USB CDC line control//！\name USB CDC线路控制
//@{//@{

//! \name USB CDC line coding//！\name USB CDC线路编码
//@{//@{
//! Line Coding structure//！行编码结构
typedef struct {
	le32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t bParityType;
	uint8_t bDataBits;
} usb_cdc_line_coding_t;
//! Possible values of bCharFormat//！bCharFormat的可能值
enum cdc_char_format {
	CDC_STOP_BITS_1 = 0,	//!< 1 stop bit//！<1停止位
	CDC_STOP_BITS_1_5 = 1,	//!< 1.5 stop bits//！<1.5停止位
	CDC_STOP_BITS_2 = 2,	//!< 2 stop bits//！<2个停止位
};
//! Possible values of bParityType//！bParityType的可能值
enum cdc_parity {
	CDC_PAR_NONE = 0,	//!< No parity//！<没有对等
	CDC_PAR_ODD = 1,	//!< Odd parity//！<奇偶校验
	CDC_PAR_EVEN = 2,	//!< Even parity//！<偶奇偶校验
	CDC_PAR_MARK = 3,	//!< Parity forced to 0 (space)//！<奇偶校验强制为0（空格）
	CDC_PAR_SPACE = 4,	//!< Parity forced to 1 (mark)//！<奇偶校验强制为1（标记）
};
//@}//@}

//! \name USB CDC control signals//! \名称USB CDC控制信号
//! spec 1.1 chapter 6.2.14//! 规范1.1第6.2.14章
//@{//@{

//! Control signal structure//！控制信号结构
typedef struct {
	uint16_t value;
} usb_cdc_control_signal_t;

//! \name Possible values in usb_cdc_control_signal_t//！\n说出usb\u cdc\u控制\u信号中可能的值\u t
//@{//@{
//! Carrier control for half duplex modems.//！半双工调制解调器的载波控制。
//! This signal corresponds to V.24 signal 105 and RS-232 signal RTS.//！此信号对应于V.24信号105和RS-232信号RTS。
//! The device ignores the value of this bit//！设备忽略此位的值
//! when operating in full duplex mode.//！在全双工模式下运行时。
#define  CDC_CTRL_SIGNAL_ACTIVATE_CARRIER    (1 << 1)
//! Indicates to DCE if DTE is present or not.//！向DCE指示DTE是否存在。
//! This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR.//！该信号对应于V.24信号108/2和RS-232信号DTR。
#define  CDC_CTRL_SIGNAL_DTE_PRESENT         (1 << 0)
//@}//@}
//@}//@}


//! \name USB CDC notification message//! \名称USB CDC通知消息
//@{//@{

typedef struct {
	uint8_t bmRequestType;
	uint8_t bNotification;
	le16_t wValue;
	le16_t wIndex;
	le16_t wLength;
} usb_cdc_notify_msg_t;

//! \name USB CDC serial state//！\name USB CDC串行状态
//@{*//@{*

//! Hardware handshake support (cdc spec 1.1 chapter 6.3.5)//！硬件握手支持（cdc规范1.1第6.3.5章）
typedef struct {
	usb_cdc_notify_msg_t header;
	le16_t value;
} usb_cdc_notify_serial_state_t;

//! \name Possible values in usb_cdc_notify_serial_state_t//！\n说出usb\u cdc\u notify\u serial\u state\t中可能的值
//@{//@{
#define  CDC_SERIAL_STATE_DCD       CPU_TO_LE16((1<<0))
#define  CDC_SERIAL_STATE_DSR       CPU_TO_LE16((1<<1))
#define  CDC_SERIAL_STATE_BREAK     CPU_TO_LE16((1<<2))
#define  CDC_SERIAL_STATE_RING      CPU_TO_LE16((1<<3))
#define  CDC_SERIAL_STATE_FRAMING   CPU_TO_LE16((1<<4))
#define  CDC_SERIAL_STATE_PARITY    CPU_TO_LE16((1<<5))
#define  CDC_SERIAL_STATE_OVERRUN   CPU_TO_LE16((1<<6))
//@}//@}
//! @}//! @}

//! @}//! @}

COMPILER_PACK_RESET()

//! @}//! @}

#endif // _USB_PROTOCOL_CDC_H_//_USB_协议_CDC_H_
