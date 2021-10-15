/** translatione by yx */
/**
 * \file
 *
 * \brief USB Mass Storage Class (MSC) protocol definitions.
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

#ifndef _USB_PROTOCOL_MSC_H_
#define _USB_PROTOCOL_MSC_H_


/**
 * \ingroup usb_protocol_group
 * \defgroup usb_msc_protocol USB Mass Storage Class (MSC) protocol definitions
 *
 * @{
 */

/**
 * \name Possible Class value
 */
//@{//@{
#define  MSC_CLASS                  0x08
//@}//@}

/**
 * \name Possible SubClass value
 * \note In practise, most devices should use
 * #MSC_SUBCLASS_TRANSPARENT and specify the actual command set in
 * the standard INQUIRY data block, even if the MSC spec indicates
 * otherwise. In particular, RBC is not supported by certain major
 * operating systems like Windows XP.
 */
//@{//@{
#define  MSC_SUBCLASS_RBC           0x01	//!< Reduced Block Commands//！<缩减块命令
#define  MSC_SUBCLASS_ATAPI         0x02	//!< CD/DVD devices//！<CD/DVD设备
#define  MSC_SUBCLASS_QIC_157       0x03	//!< Tape devices//！<磁带设备
#define  MSC_SUBCLASS_UFI           0x04	//!< Floppy disk drives//！<软盘驱动器
#define  MSC_SUBCLASS_SFF_8070I     0x05	//!< Floppy disk drives//！<软盘驱动器
#define  MSC_SUBCLASS_TRANSPARENT   0x06	//!< Determined by INQUIRY//！<通过查询确定
//@}//@}

/**
 * \name Possible protocol value
 * \note Only the BULK protocol should be used in new designs.
 */
//@{//@{
#define  MSC_PROTOCOL_CBI           0x00	//!< Command/Bulk/Interrupt//！<命令/批量/中断
#define  MSC_PROTOCOL_CBI_ALT       0x01	//!< W/o command completion//！<W/o命令完成
#define  MSC_PROTOCOL_BULK          0x50	//!< Bulk-only//！<仅限散装
//@}//@}


/**
 * \brief MSC USB requests (bRequest)
 */
enum usb_reqid_msc {
	USB_REQ_MSC_BULK_RESET = 0xFF,	//!< Mass Storage Reset//!< 大容量存储器复位
	USB_REQ_MSC_GET_MAX_LUN = 0xFE 	//!< Get Max LUN//!< 获取最大LUN
};


COMPILER_PACK_SET(1)

/**
 * \name A Command Block Wrapper (CBW).
 */
//@{//@{
struct usb_msc_cbw {
	le32_t dCBWSignature;	//!< Must contain 'USBC'//！<必须包含“USBC”
	le32_t dCBWTag;	//!< Unique command ID//！<唯一命令ID
	le32_t dCBWDataTransferLength;	//!< Number of bytes to transfer//！<要传输的字节数
	uint8_t bmCBWFlags;	//!< Direction in bit 7//！<位7中的方向
	uint8_t bCBWLUN;	//!< Logical Unit Number//！<逻辑单元号
	uint8_t bCBWCBLength;	//!< Number of valid CDB bytes//！<有效CDB字节数
	uint8_t CDB[16];	//!< SCSI Command Descriptor Block//！<SCSI命令描述符块
};

#define  USB_CBW_SIGNATURE          0x55534243	//!< dCBWSignature value//！<dcbw签名值
#define  USB_CBW_DIRECTION_IN       (1<<7)	//!< Data from device to host//！<从设备到主机的数据
#define  USB_CBW_DIRECTION_OUT      (0<<7)	//!< Data from host to device//！<从主机到设备的数据
#define  USB_CBW_LUN_MASK           0x0F	//!< Valid bits in bCBWLUN//！<bCBWLUN中的有效位
#define  USB_CBW_LEN_MASK           0x1F	//!< Valid bits in bCBWCBLength//！<BCBWCBENGTH中的有效位
//@}//@}


/**
 * \name A Command Status Wrapper (CSW).
 */
//@{//@{
struct usb_msc_csw {
	le32_t dCSWSignature;	//!< Must contain 'USBS'//！<必须包含“USB”
	le32_t dCSWTag;	//!< Same as dCBWTag//！<与dCBWTag相同
	le32_t dCSWDataResidue;	//!< Number of bytes not transfered//！<未传输的字节数
	uint8_t bCSWStatus;	//!< Status code//！<状态代码
};

#define  USB_CSW_SIGNATURE          0x55534253	//!< dCSWSignature value//！<dCSWSignature值
#define  USB_CSW_STATUS_PASS        0x00	//!< Command Passed//！<命令已通过
#define  USB_CSW_STATUS_FAIL        0x01	//!< Command Failed//！<命令失败
#define  USB_CSW_STATUS_PE          0x02	//!< Phase Error//相位误差
//@}//@}

COMPILER_PACK_RESET()

//@}//@}

#endif // _USB_PROTOCOL_MSC_H_//_USB_协议_MSC_H_
