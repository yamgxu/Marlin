/** translatione by yx */
/**
 * \file
 *
 * \brief SCSI Primary Commands
 *
 * This file contains definitions of some of the commands found in the
 * SPC-2 standard.
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
#ifndef _SPC_PROTOCOL_H_
#define _SPC_PROTOCOL_H_

/**
 * \ingroup usb_msc_protocol
 * \defgroup usb_spc_protocol SCSI Primary Commands protocol definitions
 *
 * @{
 */

//! \name SCSI commands defined by SPC-2//！\name由SPC-2定义的SCSI命令
//@{//@{
#define  SPC_TEST_UNIT_READY              0x00
#define  SPC_REQUEST_SENSE                0x03
#define  SPC_INQUIRY                      0x12
#define  SPC_MODE_SELECT6                 0x15
#define  SPC_MODE_SENSE6                  0x1A
#define  SPC_SEND_DIAGNOSTIC              0x1D
#define  SPC_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1E
#define  SPC_MODE_SENSE10                 0x5A
#define  SPC_REPORT_LUNS                  0xA0
//@}//@}

//! \brief May be set in byte 0 of the INQUIRY CDB//！\BRIENT可设置在查询CDB的字节0中
//@{//@{
//! Enable Vital Product Data//！启用重要的产品数据
#define  SCSI_INQ_REQ_EVPD    0x01
//! Command Support Data specified by the PAGE OR OPERATION CODE field//！页面或操作代码字段指定的命令支持数据
#define  SCSI_INQ_REQ_CMDT    0x02
//@}//@}

COMPILER_PACK_SET(1)

/**
 * \brief SCSI Standard Inquiry data structure
 */
struct scsi_inquiry_data {
	uint8_t pq_pdt; //!< Peripheral Qual / Peripheral Dev Type//！<外围设备质量/外围设备开发类型
#define  SCSI_INQ_PQ_CONNECTED   0x00   //!< Peripheral connected//！<外设已连接
#define  SCSI_INQ_PQ_NOT_CONN    0x20   //!< Peripheral not connected//！<外围设备未连接
#define  SCSI_INQ_PQ_NOT_SUPP    0x60   //!< Peripheral not supported//！<不支持外围设备
#define  SCSI_INQ_DT_DIR_ACCESS  0x00   //!< Direct Access (SBC)//！<直接访问（SBC）
#define  SCSI_INQ_DT_SEQ_ACCESS  0x01   //!< Sequential Access//！<顺序存取
#define  SCSI_INQ_DT_PRINTER     0x02   //!< Printer//！<打印机
#define  SCSI_INQ_DT_PROCESSOR   0x03   //!< Processor device//处理器设备
#define  SCSI_INQ_DT_WRITE_ONCE  0x04   //!< Write-once device//！<一次写入设备
#define  SCSI_INQ_DT_CD_DVD      0x05   //!< CD/DVD device//！<CD/DVD设备
#define  SCSI_INQ_DT_OPTICAL     0x07   //!< Optical Memory//！<光存储器
#define  SCSI_INQ_DT_MC          0x08   //!< Medium Changer//！<媒体转换器
#define  SCSI_INQ_DT_ARRAY       0x0C   //!< Storage Array Controller//！<存储阵列控制器
#define  SCSI_INQ_DT_ENCLOSURE   0x0D   //!< Enclosure Services//！<附件服务
#define  SCSI_INQ_DT_RBC         0x0E   //!< Simplified Direct Access//！<简化的直接访问
#define  SCSI_INQ_DT_OCRW        0x0F   //!< Optical card reader/writer//！<光卡读写器
#define  SCSI_INQ_DT_BCC         0x10   //!< Bridge Controller Commands//！<桥接器控制器命令
#define  SCSI_INQ_DT_OSD         0x11   //!< Object-based Storage//！<基于对象的存储
#define  SCSI_INQ_DT_NONE        0x1F   //!< No Peripheral//！<没有外设
	uint8_t flags1; //!< Flags (byte 1)//！<标志（字节1）
#define  SCSI_INQ_RMB            0x80   //!< Removable Medium//！<可移动介质
	uint8_t version; //!< Version//！<版本
#define  SCSI_INQ_VER_NONE       0x00   //!< No standards conformance//！<不符合标准
#define  SCSI_INQ_VER_SPC        0x03   //!< SCSI Primary Commands     (link to SBC)//！<SCSI主命令（链接到SBC）
#define  SCSI_INQ_VER_SPC2       0x04   //!< SCSI Primary Commands - 2 (link to SBC-2)//！<SCSI主命令-2（链接到SBC-2）
#define  SCSI_INQ_VER_SPC3       0x05   //!< SCSI Primary Commands - 3 (link to SBC-2)//！<SCSI主命令-3（链接到SBC-2）
#define  SCSI_INQ_VER_SPC4       0x06   //!< SCSI Primary Commands - 4 (link to SBC-3)//！<SCSI主命令-4（链接到SBC-3）
	uint8_t flags3; //!< Flags (byte 3)//！<标志（字节3）
#define  SCSI_INQ_NORMACA        0x20   //!< Normal ACA Supported//！<支持正常ACA
#define  SCSI_INQ_HISUP          0x10   //!< Hierarchal LUN addressing//！<分层LUN寻址
#define  SCSI_INQ_RSP_SPC2       0x02   //!< SPC-2 / SPC-3 response format//！<SPC-2/SPC-3响应格式
	uint8_t addl_len; //!< Additional Length (n-4)//！<附加长度（n-4）
#define  SCSI_INQ_ADDL_LEN(tot)  ((tot)-5) //!< Total length is \a tot//！<总长度为\a总长度
	uint8_t flags5; //!< Flags (byte 5)//！<标志（字节5）
#define  SCSI_INQ_SCCS           0x80
	uint8_t flags6; //!< Flags (byte 6)//！<标志（字节6）
#define  SCSI_INQ_BQUE           0x80
#define  SCSI_INQ_ENCSERV        0x40
#define  SCSI_INQ_MULTIP         0x10
#define  SCSI_INQ_MCHGR          0x08
#define  SCSI_INQ_ADDR16         0x01
	uint8_t flags7; //!< Flags (byte 7)//！<标志（字节7）
#define  SCSI_INQ_WBUS16         0x20
#define  SCSI_INQ_SYNC           0x10
#define  SCSI_INQ_LINKED         0x08
#define  SCSI_INQ_CMDQUE         0x02
	uint8_t vendor_id[8];   //!< T10 Vendor Identification//!< T10供应商标识
	uint8_t product_id[16]; //!< Product Identification//!< 产品标识
	uint8_t product_rev[4]; //!< Product Revision Level//!< 产品修订级别
};

/**
 * \brief SCSI Standard Request sense data structure
 */
struct scsi_request_sense_data {
	/* 1st byte: REQUEST SENSE response flags*/
	uint8_t valid_reponse_code;
#define  SCSI_SENSE_VALID              0x80 //!< Indicates the INFORMATION field contains valid information//!< 指示信息字段包含有效信息
#define  SCSI_SENSE_RESPONSE_CODE_MASK 0x7F
#define  SCSI_SENSE_CURRENT            0x70 //!< Response code 70h (current errors)//!< 响应代码70h（当前错误）
#define  SCSI_SENSE_DEFERRED           0x71

	/* 2nd byte */
	uint8_t obsolete;

	/* 3rd byte */
	uint8_t sense_flag_key;
#define  SCSI_SENSE_FILEMARK        0x80 //!< Indicates that the current command has read a filemark or setmark.//!< 指示当前命令已读取文件标记或设置标记。
#define  SCSI_SENSE_EOM             0x40 //!< Indicates that an end-of-medium condition exists.//!< 指示存在介质结束条件。
#define  SCSI_SENSE_ILI             0x20 //!< Indicates that the requested logical block length did not match the logical block length of the data on the medium.//!< 指示请求的逻辑块长度与介质上数据的逻辑块长度不匹配。
#define  SCSI_SENSE_RESERVED        0x10 //!< Reserved//!< 含蓄的
#define  SCSI_SENSE_KEY(x)          (x&0x0F) //!< Sense Key//!< 感知键

	/* 4th to 7th bytes - INFORMATION field */
	uint8_t information[4];

	/* 8th byte  - ADDITIONAL SENSE LENGTH field */
	uint8_t AddSenseLen;
#define  SCSI_SENSE_ADDL_LEN(total_len)   ((total_len) - 8)

	/* 9th to 12th byte  - COMMAND-SPECIFIC INFORMATION field */
	uint8_t CmdSpecINFO[4];

	/* 13th byte  - ADDITIONAL SENSE CODE field */
	uint8_t AddSenseCode;

	/* 14th byte  - ADDITIONAL SENSE CODE QUALIFIER field */
	uint8_t AddSnsCodeQlfr;

	/* 15th byte  - FIELD REPLACEABLE UNIT CODE field */
	uint8_t FldReplUnitCode;

	/* 16th byte */
	uint8_t SenseKeySpec[3];
#define  SCSI_SENSE_SKSV            0x80 //!< Indicates the SENSE-KEY SPECIFIC field contains valid information//！<表示传感键特定字段包含有效信息
};

COMPILER_PACK_RESET()

/* Vital Product Data page codes */
enum scsi_vpd_page_code {
	SCSI_VPD_SUPPORTED_PAGES = 0x00,
	SCSI_VPD_UNIT_SERIAL_NUMBER = 0x80,
	SCSI_VPD_DEVICE_IDENTIFICATION = 0x83,
};
#define  SCSI_VPD_HEADER_SIZE       4

/* Constants associated with the Device Identification VPD page */
#define  SCSI_VPD_ID_HEADER_SIZE    4

#define  SCSI_VPD_CODE_SET_BINARY   1
#define  SCSI_VPD_CODE_SET_ASCII    2
#define  SCSI_VPD_CODE_SET_UTF8     3

#define  SCSI_VPD_ID_TYPE_T10       1


/* Sense keys */
enum scsi_sense_key {
	SCSI_SK_NO_SENSE = 0x0,
	SCSI_SK_RECOVERED_ERROR = 0x1,
	SCSI_SK_NOT_READY = 0x2,
	SCSI_SK_MEDIUM_ERROR = 0x3,
	SCSI_SK_HARDWARE_ERROR = 0x4,
	SCSI_SK_ILLEGAL_REQUEST = 0x5,
	SCSI_SK_UNIT_ATTENTION = 0x6,
	SCSI_SK_DATA_PROTECT = 0x7,
	SCSI_SK_BLANK_CHECK = 0x8,
	SCSI_SK_VENDOR_SPECIFIC = 0x9,
	SCSI_SK_COPY_ABORTED = 0xA,
	SCSI_SK_ABORTED_COMMAND = 0xB,
	SCSI_SK_VOLUME_OVERFLOW = 0xD,
	SCSI_SK_MISCOMPARE = 0xE,
};

/* Additional Sense Code / Additional Sense Code Qualifier pairs */
enum scsi_asc_ascq {
	SCSI_ASC_NO_ADDITIONAL_SENSE_INFO = 0x0000,
	SCSI_ASC_LU_NOT_READY_REBUILD_IN_PROGRESS = 0x0405,
	SCSI_ASC_WRITE_ERROR = 0x0C00,
	SCSI_ASC_UNRECOVERED_READ_ERROR = 0x1100,
	SCSI_ASC_INVALID_COMMAND_OPERATION_CODE = 0x2000,
	SCSI_ASC_INVALID_FIELD_IN_CDB = 0x2400,
	SCSI_ASC_WRITE_PROTECTED = 0x2700,
	SCSI_ASC_NOT_READY_TO_READY_CHANGE = 0x2800,
	SCSI_ASC_MEDIUM_NOT_PRESENT = 0x3A00,
	SCSI_ASC_INTERNAL_TARGET_FAILURE = 0x4400,
};

/**
 * \brief SPC-2 Mode parameter
 * This subclause describes the block descriptors and the pages
 * used with MODE SELECT and MODE SENSE commands
 * that are applicable to all SCSI devices.
 */
enum scsi_spc_mode {
	SCSI_MS_MODE_VENDOR_SPEC = 0x00,
	SCSI_MS_MODE_INFEXP = 0x1C,    // Informational exceptions control page//信息性异常控制页
	SCSI_MS_MODE_ALL = 0x3F,
};

/**
 * \brief SPC-2 Informational exceptions control page
 * See chapter 8.3.8
 */
struct spc_control_page_info_execpt {
	uint8_t page_code;
	uint8_t page_length;
#define  SPC_MP_INFEXP_PAGE_LENGTH     0x0A
	uint8_t flags1;
#define  SPC_MP_INFEXP_PERF            (1<<7)   //!< Initiator Control//!< 启动器控制
#define  SPC_MP_INFEXP_EBF             (1<<5)   //!< Caching Analysis Permitted//!< 允许进行缓存分析
#define  SPC_MP_INFEXP_EWASC           (1<<4)   //!< Discontinuity//!< 间断
#define  SPC_MP_INFEXP_DEXCPT          (1<<3)   //!< Size enable//!< 大小启用
#define  SPC_MP_INFEXP_TEST            (1<<2)   //!< Writeback Cache Enable//!< 写回缓存启用
#define  SPC_MP_INFEXP_LOGERR          (1<<0)   //!< Log errors bit//!< 日志错误位
	uint8_t mrie;
#define  SPC_MP_INFEXP_MRIE_NO_REPORT           0x00
#define  SPC_MP_INFEXP_MRIE_ASYNC_EVENT         0x01
#define  SPC_MP_INFEXP_MRIE_GEN_UNIT            0x02
#define  SPC_MP_INFEXP_MRIE_COND_RECOV_ERROR    0x03
#define  SPC_MP_INFEXP_MRIE_UNCOND_RECOV_ERROR  0x04
#define  SPC_MP_INFEXP_MRIE_NO_SENSE            0x05
#define  SPC_MP_INFEXP_MRIE_ONLY_REPORT         0x06
	be32_t interval_timer;
	be32_t report_count;
};


enum scsi_spc_mode_sense_pc {
	SCSI_MS_SENSE_PC_CURRENT = 0,
	SCSI_MS_SENSE_PC_CHANGEABLE = 1,
	SCSI_MS_SENSE_PC_DEFAULT = 2,
	SCSI_MS_SENSE_PC_SAVED = 3,
};



static inline bool scsi_mode_sense_dbd_is_set(const uint8_t * cdb)
{
	return (cdb[1] >> 3) & 1;
}

static inline uint8_t scsi_mode_sense_get_page_code(const uint8_t * cdb)
{
	return cdb[2] & 0x3F;
}

static inline uint8_t scsi_mode_sense_get_pc(const uint8_t * cdb)
{
	return cdb[2] >> 6;
}

/**
 * \brief SCSI Mode Parameter Header used by MODE SELECT(6) and MODE
 * SENSE(6)
 */
struct scsi_mode_param_header6 {
	uint8_t mode_data_length;	//!< Number of bytes after this//！<在此之后的字节数
	uint8_t medium_type;	//!< Medium Type//！<中型
	uint8_t device_specific_parameter;	//!< Defined by command set//！<由命令集定义
	uint8_t block_descriptor_length;	//!< Length of block descriptors//！<块描述符的长度
};

/**
 * \brief SCSI Mode Parameter Header used by MODE SELECT(10) and MODE
 * SENSE(10)
 */
struct scsi_mode_param_header10 {
	be16_t mode_data_length;	//!< Number of bytes after this//！<在此之后的字节数
	uint8_t medium_type;	//!< Medium Type//！<中型
	uint8_t device_specific_parameter;	//!< Defined by command set//！<由命令集定义
	uint8_t flags4;	//!< LONGLBA in bit 0//！<位0中的LONGLBA
	uint8_t reserved;
	be16_t block_descriptor_length;	//!< Length of block descriptors//！<块描述符的长度
};

/**
 * \brief SCSI Page_0 Mode Page header (SPF not set)
 */
struct scsi_mode_page_0_header {
	uint8_t page_code;
#define  SCSI_PAGE_CODE_PS          (1 << 7)	//!< Parameters Saveable//！<可保存的参数
#define  SCSI_PAGE_CODE_SPF         (1 << 6)	//!< SubPage Format//！<子页面格式
	uint8_t page_length;	//!< Number of bytes after this//！<在此之后的字节数
#define  SCSI_MS_PAGE_LEN(total)   ((total) - 2)
};

//@}//@}

#endif // SPC_PROTOCOL_H_//SPC_协议_
