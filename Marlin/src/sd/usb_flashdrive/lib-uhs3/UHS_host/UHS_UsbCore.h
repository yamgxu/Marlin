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

#if !defined(_UHS_host_h_) || defined(USBCORE_H)
#error "Never include UHS_UsbCore.h directly; include UHS_Host.h instead"
#else
#define USBCORE_H

#ifndef UHS_HOST_MAX_INTERFACE_DRIVERS
#define                UHS_HOST_MAX_INTERFACE_DRIVERS 0x10U // Default maximum number of USB interface drivers//默认最大USB接口驱动程序数
#endif

#ifndef SYSTEM_OR_SPECIAL_YIELD
#define SYSTEM_OR_SPECIAL_YIELD(...) VOID0
#endif

#ifndef SYSTEM_OR_SPECIAL_YIELD_FROM_ISR
#define SYSTEM_OR_SPECIAL_YIELD_FROM_ISR(...) SYSTEM_OR_SPECIAL_YIELD
#endif

// As we make extensions to a target interface add to UHS_HOST_MAX_INTERFACE_DRIVERS//当我们对目标接口进行扩展时，添加UHS_主机_MAX_接口_驱动程序
// This offset gets calculated for supporting wide subclasses, such as HID, BT, etc.//计算此偏移量是为了支持广泛的子类，如HID、BT等。
#define                                 UHS_HID_INDEX (UHS_HOST_MAX_INTERFACE_DRIVERS + 1)

/* Common setup data constant combinations  */
//get descriptor request type//获取描述符请求类型
#define                           UHS_bmREQ_GET_DESCR (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//set request type for all but 'set feature' and 'set interface'//设置除“设置功能”和“设置接口”之外的所有请求类型
#define                                 UHS_bmREQ_SET (USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE)

//get interface request type//获取接口请求类型
#define                         UHS_bmREQ_CL_GET_INTF (USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE)

// D7           data transfer direction (0 - host-to-device, 1 - device-to-host)//D7数据传输方向（0-主机到设备，1-设备到主机）
// D6-5         Type (0- standard, 1 - class, 2 - vendor, 3 - reserved)//D6-5型（0-标准，1-等级，2-供应商，3-保留）
// D4-0         Recipient (0 - device, 1 - interface, 2 - endpoint, 3 - other, 4..31 - reserved)//D4-0收件人（0-设备，1-接口，2-端点，3-其他，4..31-保留）


// TO-DO: Use the python script to generate these.//待办事项：使用python脚本生成这些。
// TO-DO: Add _all_ subclasses here.//待办事项：在此处添加u all_uu子类。
// USB Device Classes, Subclasses and Protocols//USB设备类、子类和协议
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Use Class Info in the Interface Descriptors//在接口描述符中使用类信息
#define                  UHS_USB_CLASS_USE_CLASS_INFO 0x00U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Audio//音频
#define                           UHS_USB_CLASS_AUDIO 0x01U
// Subclasses//子类
#define                 UHS_USB_SUBCLASS_AUDIOCONTROL 0x01U
#define               UHS_USB_SUBCLASS_AUDIOSTREAMING 0x02U
#define                UHS_USB_SUBCLASS_MIDISTREAMING 0x03U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Communications and CDC Control//通信和疾病控制中心控制
#define                UHS_USB_CLASS_COM_AND_CDC_CTRL 0x02U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HID//隐藏
#define                             UHS_USB_CLASS_HID 0x03U
// Subclasses//子类
#define                         UHS_HID_BOOT_SUBCLASS 0x01U
// Protocols//协议
#define             UHS_HID_PROTOCOL_HIDBOOT_KEYBOARD 0x01U
#define                UHS_HID_PROTOCOL_HIDBOOT_MOUSE 0x02U
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Physical//物理的
#define                        UHS_USB_CLASS_PHYSICAL 0x05U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Image//形象
#define                           UHS_USB_CLASS_IMAGE 0x06U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Printer//印刷机
#define                         UHS_USB_CLASS_PRINTER 0x07U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mass Storage//大容量存储
#define                    UHS_USB_CLASS_MASS_STORAGE 0x08
// Subclasses//子类
#define           UHS_BULK_SUBCLASS_SCSI_NOT_REPORTED 0x00U   // De facto use//实际使用
#define                         UHS_BULK_SUBCLASS_RBC 0x01U
#define                       UHS_BULK_SUBCLASS_ATAPI 0x02U   // MMC-5 (ATAPI)//MMC-5（ATAPI）
#define                   UHS_BULK_SUBCLASS_OBSOLETE1 0x03U   // Was QIC-157//是QIC-157吗
#define                         UHS_BULK_SUBCLASS_UFI 0x04U   // Specifies how to interface Floppy Disk Drives to USB//指定如何将软盘驱动器连接到USB
#define                   UHS_BULK_SUBCLASS_OBSOLETE2 0x05U   // Was SFF-8070i//是SFF-8070i吗
#define                        UHS_BULK_SUBCLASS_SCSI 0x06U   // SCSI Transparent Command Set//SCSI透明命令集
#define                       UHS_BULK_SUBCLASS_LSDFS 0x07U   // Specifies how host has to negotiate access before trying SCSI//指定主机在尝试SCSI之前协商访问的方式
#define                    UHS_BULK_SUBCLASS_IEEE1667 0x08U
// Protocols//协议
#define                            UHS_STOR_PROTO_CBI 0x00U   // CBI (with command completion interrupt)//CBI（带命令完成中断）
#define                     UHS_STOR_PROTO_CBI_NO_INT 0x01U   // CBI (without command completion interrupt)//CBI（无命令完成中断）
#define                       UHS_STOR_PROTO_OBSOLETE 0x02U
#define                            UHS_STOR_PROTO_BBB 0x50U   // Bulk Only Transport//散装运输
#define                            UHS_STOR_PROTO_UAS 0x62U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hub//枢纽
#define                             UHS_USB_CLASS_HUB 0x09U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CDC-Data//CDC数据
#define                        UHS_USB_CLASS_CDC_DATA 0x0AU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Smart-Card//智能卡
#define                      UHS_USB_CLASS_SMART_CARD 0x0BU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Content Security//内容安全
#define                UHS_USB_CLASS_CONTENT_SECURITY 0x0DU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Video//录像带
#define                           UHS_USB_CLASS_VIDEO 0x0EU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Personal Healthcare//个人保健
#define                 UHS_USB_CLASS_PERSONAL_HEALTH 0x0FU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Diagnostic Device//诊断装置
#define               UHS_USB_CLASS_DIAGNOSTIC_DEVICE 0xDCU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Wireless Controller//无线控制器
#define                   UHS_USB_CLASS_WIRELESS_CTRL 0xE0U

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Miscellaneous//杂
#define                            UHS_USB_CLASS_MISC 0xEFU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Application Specific//特定于应用程序
#define                    UHS_USB_CLASS_APP_SPECIFIC 0xFEU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Vendor Specific//特定于供应商
#define                 UHS_USB_CLASS_VENDOR_SPECIFIC 0xFFU

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* USB state machine states */
#define                       UHS_USB_HOST_STATE_MASK 0xF0U

// Configure states, MSN == 0 --------------------------V//配置状态，MSN==0------------------V
#define                   UHS_USB_HOST_STATE_DETACHED 0x00U
#define                   UHS_USB_HOST_STATE_DEBOUNCE 0x01U
#define      UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE 0x02U
#define         UHS_USB_HOST_STATE_RESET_NOT_COMPLETE 0x03U
#define                   UHS_USB_HOST_STATE_WAIT_SOF 0x04U
#define             UHS_USB_HOST_STATE_WAIT_BUS_READY 0x05U
#define               UHS_USB_HOST_STATE_RESET_DEVICE 0x0AU
#define                UHS_USB_HOST_STATE_CONFIGURING 0x0CU // Looks like "CO"nfig (backwards)//看起来像“CO”nfig（向后）
#define           UHS_USB_HOST_STATE_CONFIGURING_DONE 0x0DU // Looks like "DO"one (backwards)//看起来像“做”一个（向后）
#define                      UHS_USB_HOST_STATE_CHECK 0x0EU
#define                    UHS_USB_HOST_STATE_ILLEGAL 0x0FU // Foo//福

// Run states, MSN != 0 --------------------------------V//运行状态，MSN！=0------------------V
#define                    UHS_USB_HOST_STATE_RUNNING 0x60U // Looks like "GO"//看起来像“走”
#define                       UHS_USB_HOST_STATE_IDLE 0x1DU // Looks like "ID"le//看起来像是“身份证”
#define                      UHS_USB_HOST_STATE_ERROR 0xF0U // Looks like "FO"o//看起来像“FO”o
#define                 UHS_USB_HOST_STATE_INITIALIZE 0x10U // Looks like "I"nit//看起来像“我”尼特

// Host SE result codes.//主机SE结果代码。
// Common SE results are stored in the low nybble, all interface drivers understand these plus 0x1F.//通用SE结果存储在低nybble中，所有接口驱动程序都理解这些加上0x1F。
// Extended SE results are 0x10-0x1E. SE code only understands these internal to the hardware.//扩展SE结果为0x10-0x1E。SE代码只理解硬件内部的这些。
// Values > 0x1F are driver or other internal error conditions.//值>0x1F是驱动程序或其他内部错误条件。
// Return these result codes from your host controller driver to match the error condition//从主机控制器驱动程序返回这些结果代码，以匹配错误条件
// ALL Non-zero values are errors.//所有非零值都是错误。
// Values not listed in this table are not handled in the base class, or any host driver.//未在此表中列出的值不会在基类或任何主机驱动程序中处理。

#define                           UHS_HOST_ERROR_NONE 0x00U // No error//无误
#define                           UHS_HOST_ERROR_BUSY 0x01U // transfer pending//转移待定
#define                         UHS_HOST_ERROR_BADREQ 0x02U // Transfer Launch Request was bad//传输启动请求不正确
#define                            UHS_HOST_ERROR_DMA 0x03U // DMA was too short, or too long//DMA太短或太长
#define                            UHS_HOST_ERROR_NAK 0x04U // Peripheral returned NAK//外围返回NAK
#define                          UHS_HOST_ERROR_STALL 0x05U // Peripheral returned STALL//周边回流失速
#define                         UHS_HOST_ERROR_TOGERR 0x06U // Toggle error/ISO over-underrun//切换错误/ISO过欠载
#define                       UHS_HOST_ERROR_WRONGPID 0x07U // Received wrong Packet ID//收到错误的数据包ID
#define                          UHS_HOST_ERROR_BADBC 0x08U // Byte count is bad//字节计数不正确
#define                         UHS_HOST_ERROR_PIDERR 0x09U // Received Packet ID is corrupted//收到的数据包ID已损坏
#define                          UHS_HOST_ERROR_BADRQ 0x0AU // Packet error. Increase max packet.//数据包错误。增加最大数据包数。
#define                            UHS_HOST_ERROR_CRC 0x0BU // USB CRC was incorrect//USB CRC不正确
#define                           UHS_HOST_ERROR_KERR 0x0CU // K-state instead of response, usually indicates wrong speed//K状态而不是响应，通常指示错误的速度
#define                           UHS_HOST_ERROR_JERR 0x0DU // J-state instead of response, usually indicates wrong speed//J状态而不是响应，通常指示错误的速度
#define                        UHS_HOST_ERROR_TIMEOUT 0x0EU // Device did not respond in time//设备没有及时响应
#define                         UHS_HOST_ERROR_BABBLE 0x0FU // Line noise/unexpected data//线路噪声/意外数据
#define                        UHS_HOST_ERROR_MEM_LAT 0x10U // Error caused by memory latency.//内存延迟导致的错误。
#define                           UHS_HOST_ERROR_NYET 0x11U // OUT transfer accepted with NYET//NYET接受转出

// Addressing error codes//寻址错误代码
#define                      ADDR_ERROR_INVALID_INDEX 0xA0U
#define                    ADDR_ERROR_INVALID_ADDRESS 0xA1U

// Common Interface Driver error codes//通用接口驱动程序错误代码
#define           UHS_HOST_ERROR_DEVICE_NOT_SUPPORTED 0xD1U // Driver doesn't support the device or interfaces//驱动程序不支持设备或接口
#define         UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE 0xD2U // Init partially finished, but died.//Init部分完成，但已死亡。
#define     UHS_HOST_ERROR_CANT_REGISTER_DEVICE_CLASS 0xD3U // There was no driver for the interface requested.//请求的接口没有驱动程序。
#define              UHS_HOST_ERROR_ADDRESS_POOL_FULL 0xD4U // No addresses left in the address pool.//地址池中没有剩余地址。
#define           UHS_HOST_ERROR_HUB_ADDRESS_OVERFLOW 0xD5U // No hub addresses left. The maximum is 7.//没有中心地址了。最大值为7。
#define             UHS_HOST_ERROR_NO_ADDRESS_IN_POOL 0xD6U // Address was not allocated in the pool, thus not found.//未在池中分配地址，因此未找到。
#define                    UHS_HOST_ERROR_NULL_EPINFO 0xD7U // The supplied endpoint was NULL, indicates a bug or other problem.//提供的终结点为空，表示存在错误或其他问题。
#define                   UHS_HOST_ERROR_BAD_ARGUMENT 0xD8U // Indicates a range violation bug.//表示范围冲突错误。
#define             UHS_HOST_ERROR_DEVICE_DRIVER_BUSY 0xD9U // The interface driver is busy or out buffer is full, try again later.//接口驱动程序正忙或缓冲区已满，请稍后再试。
#define            UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE 0xDAU // The maximum packet size was exceeded. Try again with smaller size.//超过了最大数据包大小。请用较小的尺寸再试一次。
#define           UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE 0xDBU // The endpoint could not be found in the endpoint table.//在终结点表中找不到终结点。
#define                      UHS_HOST_ERROR_UNPLUGGED 0xDEU // Someone removed the USB device, or Vbus was turned off.//有人删除了USB设备，或Vbus已关闭。
#define                          UHS_HOST_ERROR_NOMEM 0xDFU // Out Of Memory.//内存不足。

// Control request stream errors//控制请求流错误
#define                UHS_HOST_ERROR_FailGetDevDescr 0xE1U
#define             UHS_HOST_ERROR_FailSetDevTblEntry 0xE2U
#define               UHS_HOST_ERROR_FailGetConfDescr 0xE3U
#define                  UHS_HOST_ERROR_END_OF_STREAM 0xEFU

// Host base class specific Error codes//主机基类特定错误代码
#define                UHS_HOST_ERROR_NOT_IMPLEMENTED 0xFEU
#define               UHS_HOST_ERROR_TRANSFER_TIMEOUT 0xFFU

// SEI interaction defaults//SEI交互默认值
#define                      UHS_HOST_TRANSFER_MAX_MS 10000 // USB transfer timeout in ms, per section 9.2.6.1 of USB 2.0 spec//USB传输超时（毫秒），根据USB 2.0规范第9.2.6.1节
#define               UHS_HOST_TRANSFER_RETRY_MAXIMUM 3     // 3 retry limit for a transfer//3传输的重试限制
#define                    UHS_HOST_DEBOUNCE_DELAY_MS 500   // settle delay in milliseconds//以毫秒为单位结算延迟
#define                        UHS_HUB_RESET_DELAY_MS 20    // hub port reset delay, 10ms recomended, but can be up to 20ms//集线器端口重置延迟，建议10ms，但最长可达20ms

////
// We only provide the minimum needed information for enumeration.//我们只提供枚举所需的最少信息。
// Interface drivers should be able to set up what is needed with nothing more.//接口驱动程序应该能够设置所需的内容，而无需更多。
// A driver needs to know the following information://驾驶员需要了解以下信息：
// 1: address on the USB network, parent and port (aka UsbDeviceAddress)//1：USB网络上的地址、父级和端口（也称为UsbDeviceAddress）
// 2: endpoints//2：端点
// 3: vid:pid, class, subclass, protocol//3:vid:pid，类，子类，协议
////

struct ENDPOINT_INFO {
        uint8_t bEndpointAddress;       // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).//端点地址。第7位表示方向（0=输出，1=输入）。
        uint8_t bmAttributes;           // Endpoint transfer type.//端点传输类型。
        uint16_t wMaxPacketSize;        // Maximum packet size.//最大数据包大小。
        uint8_t bInterval;              // Polling interval in frames.//以帧为单位的轮询间隔。
} __attribute__((packed));

struct INTERFACE_INFO {
        uint8_t bInterfaceNumber;
        uint8_t bAlternateSetting;
        uint8_t numep;
        uint8_t klass;
        uint8_t subklass;
        uint8_t protocol;
        ENDPOINT_INFO epInfo[16];
} __attribute__((packed));

struct ENUMERATION_INFO {
        uint16_t vid;
        uint16_t pid;
        uint16_t bcdDevice;
        uint8_t klass;
        uint8_t subklass;
        uint8_t protocol;
        uint8_t bMaxPacketSize0;
        uint8_t currentconfig;
        uint8_t parent;
        uint8_t port;
        uint8_t address;
        INTERFACE_INFO interface;
} __attribute__((packed));

/* USB Setup Packet Structure   */
typedef struct {
        // offset   description//偏移量说明
        //   0      Bit-map of request type//请求类型的0位映射
         union {
                uint8_t bmRequestType;

                struct {
                        uint8_t recipient : 5;  // Recipient of the request//请求接收人
                        uint8_t type : 2;       // Type of request//请求类型
                        uint8_t direction : 1;  // Direction of data transfer//数据传输方向
                } __attribute__((packed));
        } ReqType_u;

        //   1      Request//1请求
        uint8_t bRequest;

        //   2      Depends on bRequest//2取决于酿造
        union {
                uint16_t wValue;

                struct {
                        uint8_t wValueLo;
                        uint8_t wValueHi;
                } __attribute__((packed));
        } wVal_u;
        //   4      Depends on bRequest//4靠酿造
        uint16_t wIndex;
        //   6      Depends on bRequest//6靠酿造
        uint16_t wLength;
        // 8 bytes total//总共8字节
} __attribute__((packed)) SETUP_PKT, *PSETUP_PKT;


// little endian :-)                                                                             8                                8                          8                         8                          16                      16//小恩迪安：-）8 8 16 16
#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wValLo))<<16)|(((uint64_t)(wValHi))<<24)|(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))
#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType)))|(((uint64_t)(bRequest))<<8)|(((uint64_t)(wVal  ))<<16)                           |(((uint64_t)(wInd))<<32)|(((uint64_t)(total)<<48)))

// Big endian -- but we aren't able to use this :-///Big-endian--但我们无法使用它：-/
//#define mkSETUP_PKT8(bmReqType, bRequest, wValLo, wValHi, wInd, total) ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)|(((uint64_t)(wValLo))<<40)|(((uint64_t)(wValHi))<<32)|(((uint64_t)(wInd))<<16)|((uint64_t)(total)))//#定义mkSETUP_PKT8（需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型、需求类型
//#define mkSETUP_PKT16(bmReqType, bRequest, wVal, wInd, total)          ((uint64_t)(((uint64_t)(bmReqType))<<56)|(((uint64_t)(bRequest))<<48)                           |(((uint64_t)(wVal))<<32)  |(((uint64_t)(wInd))<<16)|((uint64_t)(total)))//#定义mkSETUP_PKT16（需求类型，需求类型，需求类型，wVal，风，总计）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（需求类型）（

#endif /* USBCORE_H */
