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

#if !defined(_UHS_host_h_) || defined(__ADDRESS_H__)
#error "Never include UHS_address.h directly; include UHS_Usb.h instead"
#else
#define __ADDRESS_H__



/* NAK powers. To save space in endpoint data structure, amount of retries before giving up and returning 0x4 is stored in */
/* bmNakPower as a power of 2. The actual nak_limit is then calculated as nak_limit = ( 2^bmNakPower - 1) */
#define UHS_USB_NAK_MAX_POWER               14      // NAK binary order maximum value//二元序最大值
#define UHS_USB_NAK_DEFAULT                 13      // default 16K-1 NAKs before giving up//放弃前默认16K-1 NAK
#define UHS_USB_NAK_NOWAIT                  1       // Single NAK stops transfer//单NAK停止传输
#define UHS_USB_NAK_NONAK                   0       // Do not count NAKs, stop retrying after USB Timeout. Try not to use this.//不计算NAK，在USB超时后停止重试。尽量不要用这个。

#define bmUSB_DEV_ADDR_PORT             0x07
#define bmUSB_DEV_ADDR_PARENT           0x78
#define bmUSB_DEV_ADDR_HUB              0x40

// TODO: embed parent?//TODO:嵌入父对象？
struct UHS_EpInfo {
        uint8_t epAddr; // Endpoint address//端点地址
        uint8_t bIface;
        uint16_t maxPktSize; // Maximum packet size//最大数据包大小

        union {
                uint8_t epAttribs;

                struct {
                        uint8_t bmSndToggle : 1; // Send toggle, when zero bmSNDTOG0, bmSNDTOG1 otherwise//发送切换，当bmSNDTOG0为零时，否则为bmSNDTOG1
                        uint8_t bmRcvToggle : 1; // Send toggle, when zero bmRCVTOG0, bmRCVTOG1 otherwise//发送切换，当bmRCVTOG0为零时，否则为bmRCVTOG1
                        uint8_t bmNeedPing : 1; // 1 == ping protocol needed for next out packet//1==下一个输出数据包所需的ping协议
                        uint8_t bmNakPower : 5; // Binary order for NAK_LIMIT value//NAK_极限值的二进制顺序
                } __attribute__((packed));
        };
} __attribute__((packed));

// TODO: embed parent address and port into epinfo struct,//TODO:将父地址和端口嵌入epinfo结构，
// and nuke this address stupidity.//还有核弹这个地址的愚蠢。
// This is a compact scheme. Should also support full spec.//这是一个紧凑的计划。还应支持完整规范。
// This produces a 7 hub limit, 49 devices + 7 hubs, 56 total.//这将产生7个集线器限制，49个设备+7个集线器，总共56个。
////
//    7   6   5   4   3   2   1   0//    7   6   5   4   3   2   1   0
//  ---------------------------------//  ---------------------------------
//  |   | H | P | P | P | A | A | A |//| | H | P | P | A | A | A|
//  ---------------------------------//  ---------------------------------
////
// H - if 1 the address is a hub address//H-如果为1，则地址为集线器地址
// P - parent hub number//P-父集线器编号
// A - port number of parent//A-父端口号
////

struct UHS_DeviceAddress {

        union {

                struct {
                        uint8_t bmAddress : 3; // port number//端口号
                        uint8_t bmParent : 3; // parent hub address//父集线器地址
                        uint8_t bmHub : 1; // hub flag//中心旗
                        uint8_t bmReserved : 1; // reserved, must be zero//保留，必须为零
                } __attribute__((packed));
                uint8_t devAddress;
        };
} __attribute__((packed));

struct UHS_Device {
        volatile UHS_EpInfo *epinfo[UHS_HOST_MAX_INTERFACE_DRIVERS]; // endpoint info pointer//端点信息指针
        UHS_DeviceAddress address;
        uint8_t epcount; // number of endpoints//端点数
        uint8_t speed; // indicates device speed//指示设备速度
} __attribute__((packed));

typedef void (*UsbDeviceHandleFunc)(UHS_Device *pdev);

class AddressPool {
        UHS_EpInfo dev0ep; //Endpoint data structure used during enumeration for uninitialized device//枚举未初始化设备期间使用的终结点数据结构

        // In order to avoid hub address duplication, this should use bits//为了避免集线器地址重复，应该使用位
        uint8_t hubCounter; // hub counter//集线器计数器

        UHS_Device thePool[UHS_HOST_MAX_INTERFACE_DRIVERS];

        // Initializes address pool entry//初始化地址池条目

        void UHS_NI InitEntry(uint8_t index) {
                thePool[index].address.devAddress = 0;
                thePool[index].epcount = 1;
                thePool[index].speed = 0;
                for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        thePool[index].epinfo[i] = &dev0ep;
                }
        };

        // Returns thePool index for a given address//返回给定地址的池索引

        uint8_t UHS_NI FindAddressIndex(uint8_t address = 0) {
                for(uint8_t i = 1; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        if(thePool[i].address.devAddress == address)
                                return i;
                }
                return 0;
        };

        // Returns thePool child index for a given parent//返回给定父级的工具子索引

        uint8_t UHS_NI FindChildIndex(UHS_DeviceAddress addr, uint8_t start = 1) {
                for(uint8_t i = (start < 1 || start >= UHS_HOST_MAX_INTERFACE_DRIVERS) ? 1 : start; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) {
                        if(thePool[i].address.bmParent == addr.bmAddress)
                                return i;
                }
                return 0;
        };

        // Frees address entry specified by index parameter//释放由索引参数指定的地址项

        void UHS_NI FreeAddressByIndex(uint8_t index) {
                // Zero field is reserved and should not be affected//零字段是保留的，不应受到影响
                if(index == 0)
                        return;

                UHS_DeviceAddress uda = thePool[index].address;
                // If a hub was switched off all port addresses should be freed//如果集线器已关闭，则应释放所有端口地址
                if(uda.bmHub == 1) {
                        for(uint8_t i = 1; (i = FindChildIndex(uda, i));)
                                FreeAddressByIndex(i);

                        // FIXME: use BIT MASKS//修复方法：使用位掩码
                        // If the hub had the last allocated address, hubCounter should be decremented//如果集线器具有最后分配的地址，则应递减集线器计数器
                        if(hubCounter == uda.bmAddress)
                                hubCounter--;
                }
                InitEntry(index);
        }

        void InitAllAddresses() {
                for(uint8_t i = 1; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++) InitEntry(i);
                hubCounter = 0;
        };
public:

        AddressPool() {
                hubCounter = 0;
                // Zero address is reserved//保留零地址
                InitEntry(0);

                thePool[0].epinfo[0] = &dev0ep;
                dev0ep.epAddr = 0;
#if UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE
                dev0ep.maxPktSize = 0x40; //starting at 0x40 and work down//从0x40开始向下工作
#else
                dev0ep.maxPktSize = 0x08;
#endif
                dev0ep.epAttribs = 0; //set DATA0/1 toggles to 0//将数据0/1设置为0
                dev0ep.bmNakPower = UHS_USB_NAK_MAX_POWER;
                InitAllAddresses();
        };

        // Returns a pointer to a specified address entry//返回指向指定地址项的指针

        UHS_Device* UHS_NI GetUsbDevicePtr(uint8_t addr) {
                if(!addr)
                        return thePool;

                uint8_t index = FindAddressIndex(addr);

                return (!index) ? NULL : &thePool[index];
        };


        // Allocates new address//分配新地址

        uint8_t UHS_NI AllocAddress(uint8_t parent, bool is_hub = false, uint8_t port = 1) {
                /* if (parent != 0 && port == 0)
                        USB_HOST_SERIAL.println("PRT:0"); */
                UHS_DeviceAddress _parent;
                _parent.devAddress = parent;
                if(_parent.bmReserved || port > 7)
                        //if(parent > 127 || port > 7)//如果（父端口>127 | |端口>7）
                        return 0;

                // FIXME: use BIT MASKS//修复方法：使用位掩码
                if(is_hub && hubCounter == 7)
                        return 0;

                // finds first empty address entry starting from one//查找从1开始的第一个空地址项
                uint8_t index = FindAddressIndex(0);

                if(!index) // if empty entry is not found//如果未找到空条目
                        return 0;

                UHS_DeviceAddress addr;
                addr.devAddress = port;
                addr.bmParent = _parent.bmAddress;

                // FIXME: use BIT MASKS//修复方法：使用位掩码
                if(is_hub) {
                        hubCounter++;
                        addr.bmHub = 1;
                        addr.bmAddress = hubCounter;
                }
                thePool[index].address = addr;
#if DEBUG_PRINTF_EXTRA_HUGE
#ifdef UHS_DEBUG_USB_ADDRESS
                printf("Address: %x (%x.%x.%x)\r\n", addr.devAddress, addr.bmHub, addr.bmParent, addr.bmAddress);
#endif
#endif
                return thePool[index].address.devAddress;
        };

        void UHS_NI FreeAddress(uint8_t addr) {
                // if the root hub is disconnected all the addresses should be initialized//如果根集线器断开连接，则应初始化所有地址
                if(addr == 0x41) {
                        InitAllAddresses();
                        return;
                }
                uint8_t index = FindAddressIndex(addr);
                FreeAddressByIndex(index);
        };

};

#endif // __ADDRESS_H__//地址__
