/** translatione by yx */
/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
 Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com//www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

#ifndef __UHS_BULK_STORAGE_H__
#define __UHS_BULK_STORAGE_H__


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define any of these options at the top of your sketch to override//在草图顶部定义要替代的任何选项
// the defaults contained herewith. Do NOT do modifications here.//本函所载默认值。不要在这里做修改。
// Macro                                 | Settings and notes    | Default//宏|设置和注释|默认值
// -----------------------------------------+-----------------------+-----------// -----------------------------------------+-----------------------+-----------
//                                          | 1 to 8                |//| 1至8|
//                                          | Each LUN needs        |//|每个LUN都需要|
// MASS_MAX_SUPPORTED_LUN                   | ~13 bytes to be able  | 8//支持的海量LUN |最多| 13个字节，可以| 8
//                                          | to track the state of |//|跟踪|
//                                          | each unit.            |//|每个单元|
// -----------------------------------------+-----------------------+-----------// -----------------------------------------+-----------------------+-----------
//                                          | Just define to use.   |//|只需定义使用即可|
// DEBUG_PRINTF_EXTRA_HUGE_UHS_BULK_STORAGE | works only if extra   |//DEBUG_PRINTF_EXTRA_Large_UHS_BULK|u STORAGE|仅在额外的情况下工作|
//                                          | huge debug is on too. |//|大型调试也在进行中|
// -----------------------------------------^-----------------------^-----------// -----------------------------------------^-----------------------^-----------

#ifndef MASS_MAX_SUPPORTED_LUN
#define MASS_MAX_SUPPORTED_LUN 8
#endif

#include "UHS_SCSI.h"

#define                UHS_BULK_bmREQ_OUT USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define                 UHS_BULK_bmREQ_IN USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE

// Request Codes//请求代码
#define                 UHS_BULK_REQ_ADSC 0x00U
#define                  UHS_BULK_REQ_GET 0xFCU
#define                  UHS_BULK_REQ_PUT 0xFDU
#define          UHS_BULK_REQ_GET_MAX_LUN 0xFEU
#define                UHS_BULK_REQ_BOMSR 0xFFU // Mass Storage Reset//大容量存储器复位

#define            UHS_BULK_CBW_SIGNATURE 0x43425355LU
#define            UHS_BULK_CSW_SIGNATURE 0x53425355LU

#define              UHS_BULK_CMD_DIR_OUT 0x00U
#define               UHS_BULK_CMD_DIR_IN 0x80U

/* Bulk error codes */
#define              UHS_BULK_ERR_SUCCESS UHS_HOST_ERROR_NONE
#define          UHS_BULK_ERR_PHASE_ERROR 0x22U
#define       UHS_BULK_ERR_UNIT_NOT_READY 0x23U
#define            UHS_BULK_ERR_UNIT_BUSY 0x24U
#define                UHS_BULK_ERR_STALL 0x25U
#define    UHS_BULK_ERR_CMD_NOT_SUPPORTED 0x26U
#define          UHS_BULK_ERR_INVALID_CSW 0x27U
#define             UHS_BULK_ERR_NO_MEDIA 0x28U
#define              UHS_BULK_ERR_BAD_LBA 0x29U
#define        UHS_BULK_ERR_MEDIA_CHANGED 0x2AU
#define  UHS_BULK_ERR_DEVICE_DISCONNECTED UHS_HOST_ERROR_UNPLUGGED
#define    UHS_BULK_ERR_UNABLE_TO_RECOVER 0x32U // Reset recovery error//重置恢复错误
#define          UHS_BULK_ERR_INVALID_LUN 0x33U
#define          UHS_BULK_ERR_WRITE_STALL 0x34U
#define            UHS_BULK_ERR_READ_NAKS 0x35U
#define           UHS_BULK_ERR_WRITE_NAKS 0x36U
#define      UHS_BULK_ERR_WRITE_PROTECTED 0x37U
#define      UHS_BULK_ERR_NOT_IMPLEMENTED 0xFDU
#define   UHS_BULK_ERR_GENERAL_SCSI_ERROR 0xF0U
#define    UHS_BULK_ERR_GENERAL_USB_ERROR 0xFFU
#define                 UHS_BULK_ERR_USER 0xA0U // For subclasses to define their own error codes//用于子类定义自己的错误代码

#define                MASS_MAX_ENDPOINTS 3

struct UHS_BULK_CommandBlockWrapperBase {
        volatile uint32_t dCBWSignature;
        volatile uint32_t dCBWTag;
        volatile uint32_t dCBWDataTransferLength;
        volatile uint8_t bmCBWFlags;
public:

        UHS_BULK_CommandBlockWrapperBase() {
        }

        UHS_BULK_CommandBlockWrapperBase(uint32_t tag, uint32_t xflen, uint8_t flgs) :
        dCBWSignature(UHS_BULK_CBW_SIGNATURE), dCBWTag(tag), dCBWDataTransferLength(xflen), bmCBWFlags(flgs) {
        }
} __attribute__((packed));

struct UHS_BULK_CommandBlockWrapper : public UHS_BULK_CommandBlockWrapperBase {

        struct {
                uint8_t bmCBWLUN : 4;
                uint8_t bmReserved1 : 4;
        };

        struct {
                uint8_t bmCBWCBLength : 4;
                uint8_t bmReserved2 : 4;
        };

        uint8_t CBWCB[16];

public:
        // All zeroed.//全部归零。

        UHS_BULK_CommandBlockWrapper() :
        UHS_BULK_CommandBlockWrapperBase(0, 0, 0), bmReserved1(0), bmReserved2(0) {
                for(int i = 0; i < 16; i++) CBWCB[i] = 0;
        }

        // Generic Wrap, CDB zeroed.//通用包装，CDB归零。

        UHS_BULK_CommandBlockWrapper(uint32_t tag, uint32_t xflen, uint8_t flgs, uint8_t lu, uint8_t cmdlen, uint8_t cmd) :
        UHS_BULK_CommandBlockWrapperBase(tag, xflen, flgs),
        bmCBWLUN(lu), bmReserved1(0), bmCBWCBLength(cmdlen), bmReserved2(0) {
                for(int i = 0; i < 16; i++) CBWCB[i] = 0;
                SCSI_CDB_BASE_t *x = reinterpret_cast<SCSI_CDB_BASE_t *>(CBWCB);
                x->LUN = cmd;
        }

        // Wrap for CDB of 6//国开行6号包裹

        UHS_BULK_CommandBlockWrapper(uint32_t tag, uint32_t xflen, SCSI_CDB6_t *cdb, uint8_t dir) :
        UHS_BULK_CommandBlockWrapperBase(tag, xflen, dir),
        bmCBWLUN(cdb->LUN), bmReserved1(0), bmCBWCBLength(6), bmReserved2(0) {
                memcpy(&CBWCB, cdb, 6);
        }
        // Wrap for CDB of 10//国开行10美元包装

        UHS_BULK_CommandBlockWrapper(uint32_t tag, uint32_t xflen, SCSI_CDB10_t *cdb, uint8_t dir) :
        UHS_BULK_CommandBlockWrapperBase(tag, xflen, dir),
        bmCBWLUN(cdb->LUN), bmReserved1(0), bmCBWCBLength(10), bmReserved2(0) {
                memcpy(&CBWCB, cdb, 10);
        }
} __attribute__((packed));

struct UHS_BULK_CommandStatusWrapper {
        uint32_t dCSWSignature;
        uint32_t dCSWTag;
        uint32_t dCSWDataResidue;
        uint8_t bCSWStatus;
} __attribute__((packed));

class UHS_Bulk_Storage : public UHS_USBInterface {
protected:
        static const uint8_t epDataInIndex = 1; // DataIn endpoint index//端点索引中的数据
        static const uint8_t epDataOutIndex = 2; // DataOUT endpoint index//数据输出端点索引
        static const uint8_t epInterruptInIndex = 3; // InterruptIN  endpoint index//中断端点索引

        uint8_t bMaxLUN; // Max LUN//最大LUN
        volatile uint32_t dCBWTag; // Tag//标签
        volatile uint8_t bTheLUN; // Active LUN//活动LUN
        volatile uint32_t CurrentCapacity[MASS_MAX_SUPPORTED_LUN]; // Total sectors//总扇区
        volatile uint16_t CurrentSectorSize[MASS_MAX_SUPPORTED_LUN]; // Sector size, clipped to 16 bits//扇区大小，剪裁为16位
        volatile bool LUNOk[MASS_MAX_SUPPORTED_LUN]; // use this to check for media changes.//使用此选项检查介质更改。
        volatile bool WriteOk[MASS_MAX_SUPPORTED_LUN];
        void PrintEndpointDescriptor(const USB_FD_ENDPOINT_DESCRIPTOR* ep_ptr);

public:
        UHS_Bulk_Storage(UHS_USB_HOST_BASE *p);

        volatile UHS_EpInfo epInfo[MASS_MAX_ENDPOINTS];

        uint8_t GetbMaxLUN() {
                return bMaxLUN; // Max LUN//最大LUN
        }

        uint8_t GetbTheLUN() {
                return bTheLUN; // Active LUN//活动LUN
        }

        bool WriteProtected(uint8_t lun);
        uint8_t MediaCTL(uint8_t lun, uint8_t ctl);
        uint8_t Read(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, uint8_t *buf);
        uint8_t Write(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, const uint8_t *buf);
        uint8_t LockMedia(uint8_t lun, uint8_t lock);

        bool LUNIsGood(uint8_t lun);
        uint32_t GetCapacity(uint8_t lun);
        uint16_t GetSectorSize(uint8_t lun);
        uint8_t SCSITransaction6(SCSI_CDB6_t *cdb, uint16_t buf_size, void *buf, uint8_t dir);
        uint8_t SCSITransaction10(SCSI_CDB10_t *cdb, uint16_t buf_size, void *buf, uint8_t dir);


        // Configure and internal methods, these should never be called by a user's sketch.//配置和内部方法，用户的草图不应调用这些方法。
        uint8_t Start();
        bool OKtoEnumerate(ENUMERATION_INFO *ei);
        uint8_t SetInterface(ENUMERATION_INFO *ei);

        uint8_t GetAddress() {
                return bAddress;
        };


        void Poll();

        void DriverDefaults();


private:
        void Reset();
        void CheckMedia();

        bool IsValidCBW(uint8_t size, uint8_t *pcbw);
        bool IsMeaningfulCBW(uint8_t size, uint8_t *pcbw);
        bool IsValidCSW(UHS_BULK_CommandStatusWrapper *pcsw, UHS_BULK_CommandBlockWrapperBase *pcbw);

        bool CheckLUN(uint8_t lun);

        uint8_t Inquiry(uint8_t lun, uint16_t size, uint8_t *buf);
        uint8_t TestUnitReady(uint8_t lun);
        uint8_t RequestSense(uint8_t lun, uint16_t size, uint8_t *buf);
        uint8_t ModeSense6(uint8_t lun, uint8_t pc, uint8_t page, uint8_t subpage, uint8_t len, uint8_t *buf);
        uint8_t GetMaxLUN(uint8_t *max_lun);
        uint8_t SetCurLUN(uint8_t lun);
        uint8_t ResetRecovery();
        uint8_t ReadCapacity10(uint8_t lun, uint8_t *buf);
        uint8_t Page3F(uint8_t lun);
        uint8_t ClearEpHalt(uint8_t index);
        uint8_t Transaction(UHS_BULK_CommandBlockWrapper *cbw, uint16_t bsize, void *buf);
        uint8_t HandleUsbError(uint8_t error, uint8_t index);
        uint8_t HandleSCSIError(uint8_t status);

};

#if defined(LOAD_UHS_BULK_STORAGE) && !defined(UHS_BULK_STORAGE_LOADED)
#include "UHS_BULK_STORAGE_INLINE.h"
#endif
#endif // __MASSTORAGE_H__//质量存储__
