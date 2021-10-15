/** translatione by yx */
/**
 * Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Contact information
 * -------------------
 *
 * Circuits At Home, LTD
 * Web      :  https://www.circuitsathome.com
 * e-mail   :  support@circuitsathome.com
 */
#pragma once

#ifndef _usb_h_
  #error "Never include address.h directly; include Usb.h instead"
#endif

/* NAK powers. To save space in endpoint data structure, amount of retries before giving up and returning 0x4 is stored in */
/* bmNakPower as a power of 2. The actual nak_limit is then calculated as nak_limit = ( 2^bmNakPower - 1) */
#define USB_NAK_MAX_POWER               15              //NAK binary order maximum value//二元序最大值
#define USB_NAK_DEFAULT                 14              //default 32K-1 NAKs before giving up//放弃前默认32K-1 NAK
#define USB_NAK_NOWAIT                  1               //Single NAK stops transfer//单NAK停止传输
#define USB_NAK_NONAK                   0               //Do not count NAKs, stop retrying after USB Timeout//不计算NAK，在USB超时后停止重试

struct EpInfo {
  uint8_t epAddr; // Endpoint address//端点地址
  uint8_t maxPktSize; // Maximum packet size//最大数据包大小

  union {
    uint8_t epAttribs;

    struct {
      uint8_t bmSndToggle : 1; // Send toggle, when zero bmSNDTOG0, bmSNDTOG1 otherwise//发送切换，当bmSNDTOG0为零时，否则为bmSNDTOG1
      uint8_t bmRcvToggle : 1; // Send toggle, when zero bmRCVTOG0, bmRCVTOG1 otherwise//发送切换，当bmRCVTOG0为零时，否则为bmRCVTOG1
      uint8_t bmNakPower : 6; // Binary order for NAK_LIMIT value//NAK_极限值的二进制顺序
    } __attribute__((packed));
  };
} __attribute__((packed));

//        7   6   5   4   3   2   1   0//        7   6   5   4   3   2   1   0
//  ---------------------------------//  ---------------------------------
//  |   | H | P | P | P | A | A | A |//| | H | P | P | A | A | A|
//  ---------------------------------//  ---------------------------------
////
// H - if 1 the address is a hub address//H-如果为1，则地址为集线器地址
// P - parent hub address//P-父集线器地址
// A - device address / port number in case of hub//A-集线器情况下的设备地址/端口号
////

struct UsbDeviceAddress {
  union {
    struct {
      uint8_t bmAddress : 3; // device address/port number//设备地址/端口号
      uint8_t bmParent : 3; // parent hub address//父集线器地址
      uint8_t bmHub : 1; // hub flag//中心旗
      uint8_t bmReserved : 1; // reserved, must be zero//保留，必须为零
    } __attribute__((packed));
    uint8_t devAddress;
  };
} __attribute__((packed));

#define bmUSB_DEV_ADDR_ADDRESS          0x07
#define bmUSB_DEV_ADDR_PARENT           0x38
#define bmUSB_DEV_ADDR_HUB              0x40

struct UsbDevice {
  EpInfo *epinfo; // endpoint info pointer//端点信息指针
  UsbDeviceAddress address;
  uint8_t epcount; // number of endpoints//端点数
  bool lowspeed; // indicates if a device is the low speed one//指示设备是否为低速设备
  //      uint8_t devclass; // device class//uint8_t devclass；//设备类
} __attribute__((packed));

class AddressPool {
  public:
    virtual UsbDevice* GetUsbDevicePtr(uint8_t addr) = 0;
    virtual uint8_t AllocAddress(uint8_t parent, bool is_hub = false, uint8_t port = 0) = 0;
    virtual void FreeAddress(uint8_t addr) = 0;
};

typedef void (*UsbDeviceHandleFunc)(UsbDevice *pdev);

#define ADDR_ERROR_INVALID_INDEX                0xFF
#define ADDR_ERROR_INVALID_ADDRESS              0xFF

template <const uint8_t MAX_DEVICES_ALLOWED>
class AddressPoolImpl : public AddressPool {
  EpInfo dev0ep; //Endpoint data structure used during enumeration for uninitialized device//枚举未初始化设备期间使用的终结点数据结构

  uint8_t hubCounter; // hub counter is kept//集线器计数器被保留
  // in order to avoid hub address duplication//为了避免集线器地址重复

  UsbDevice thePool[MAX_DEVICES_ALLOWED];

  // Initialize address pool entry//初始化地址池条目

  void InitEntry(uint8_t index) {
    thePool[index].address.devAddress = 0;
    thePool[index].epcount = 1;
    thePool[index].lowspeed = 0;
    thePool[index].epinfo = &dev0ep;
  }

  // Return thePool index for a given address//返回给定地址的池索引

  uint8_t FindAddressIndex(uint8_t address = 0) {
    for (uint8_t i = 1; i < MAX_DEVICES_ALLOWED; i++)
      if (thePool[i].address.devAddress == address)
        return i;

    return 0;
  }

  // Return thePool child index for a given parent//返回给定父级的工具子索引

  uint8_t FindChildIndex(UsbDeviceAddress addr, uint8_t start = 1) {
    for (uint8_t i = (start < 1 || start >= MAX_DEVICES_ALLOWED) ? 1 : start; i < MAX_DEVICES_ALLOWED; i++) {
      if (thePool[i].address.bmParent == addr.bmAddress)
        return i;
    }
    return 0;
  }

  // Frees address entry specified by index parameter//释放由索引参数指定的地址项

  void FreeAddressByIndex(uint8_t index) {
    // Zero field is reserved and should not be affected//零字段是保留的，不应受到影响
    if (index == 0) return;

    UsbDeviceAddress uda = thePool[index].address;
    // If a hub was switched off all port addresses should be freed//如果集线器已关闭，则应释放所有端口地址
    if (uda.bmHub == 1) {
      for (uint8_t i = 1; (i = FindChildIndex(uda, i));)
        FreeAddressByIndex(i);

      // If the hub had the last allocated address, hubCounter should be decremented//如果集线器具有最后分配的地址，则应递减集线器计数器
      if (hubCounter == uda.bmAddress) hubCounter--;
    }
    InitEntry(index);
  }

  // Initialize the whole address pool at once//立即初始化整个地址池

  void InitAllAddresses() {
    for (uint8_t i = 1; i < MAX_DEVICES_ALLOWED; i++)
      InitEntry(i);

    hubCounter = 0;
  }

public:

  AddressPoolImpl() : hubCounter(0) {
    // Zero address is reserved//保留零地址
    InitEntry(0);

    thePool[0].address.devAddress = 0;
    thePool[0].epinfo = &dev0ep;
    dev0ep.epAddr = 0;
    dev0ep.maxPktSize = 8;
    dev0ep.bmSndToggle = 0; // Set DATA0/1 toggles to 0//将数据0/1设置为0
    dev0ep.bmRcvToggle = 0;
    dev0ep.bmNakPower = USB_NAK_MAX_POWER;

    InitAllAddresses();
  }

  // Return a pointer to a specified address entry//返回指向指定地址项的指针

  virtual UsbDevice* GetUsbDevicePtr(uint8_t addr) {
    if (!addr) return thePool;
    uint8_t index = FindAddressIndex(addr);
    return index ? thePool + index : nullptr;
  }

  // Perform an operation specified by pfunc for each addressed device//对每个寻址设备执行pfunc指定的操作

  void ForEachUsbDevice(UsbDeviceHandleFunc pfunc) {
    if (pfunc) {
      for (uint8_t i = 1; i < MAX_DEVICES_ALLOWED; i++)
        if (thePool[i].address.devAddress)
          pfunc(thePool + i);
    }
  }

  // Allocate new address//分配新地址

  virtual uint8_t AllocAddress(uint8_t parent, bool is_hub = false, uint8_t port = 0) {
    /* if (parent != 0 && port == 0)
      USB_HOST_SERIAL.println("PRT:0"); */
    UsbDeviceAddress _parent;
    _parent.devAddress = parent;
    if (_parent.bmReserved || port > 7)
      //if(parent > 127 || port > 7)//如果（父端口>127 | |端口>7）
      return 0;

    if (is_hub && hubCounter == 7) return 0;

    // finds first empty address entry starting from one//查找从1开始的第一个空地址项
    uint8_t index = FindAddressIndex(0);

    if (!index) return 0; // if empty entry is not found//如果未找到空条目

    if (_parent.devAddress == 0) {
      if (is_hub) {
        thePool[index].address.devAddress = 0x41;
        hubCounter++;
      }
      else
        thePool[index].address.devAddress = 1;

      return thePool[index].address.devAddress;
    }

    UsbDeviceAddress addr;
    addr.devAddress = 0; // Ensure all bits are zero//确保所有位均为零
    addr.bmParent = _parent.bmAddress;
    if (is_hub) {
      addr.bmHub = 1;
      addr.bmAddress = ++hubCounter;
    }
    else {
      addr.bmHub = 0;
      addr.bmAddress = port;
    }
    thePool[index].address = addr;
    /*
      USB_HOST_SERIAL.print("Addr:");
      USB_HOST_SERIAL.print(addr.bmHub, HEX);
      USB_HOST_SERIAL.print(".");
      USB_HOST_SERIAL.print(addr.bmParent, HEX);
      USB_HOST_SERIAL.print(".");
      USB_HOST_SERIAL.println(addr.bmAddress, HEX);
    */
    return thePool[index].address.devAddress;
  }

  // Empty the pool entry//清空池条目

  virtual void FreeAddress(uint8_t addr) {
    // if the root hub is disconnected all the addresses should be initialized//如果根集线器断开连接，则应初始化所有地址
    if (addr == 0x41) {
      InitAllAddresses();
      return;
    }
    FreeAddressByIndex(FindAddressIndex(addr));
  }

  // Return number of hubs attached//返回连接的集线器数
  // It can be helpful to find out if hubs are attached when getting the exact number of hubs.//在获取集线器的确切数量时，可以了解集线器是否已连接。
  //uint8_t GetNumHubs() { return hubCounter; }//uint8_t GetNumHubs（）{return hubCounter；}
  //uint8_t GetNumDevices() {//uint8_t GetNumDevices（）{
  //  uint8_t counter = 0;//uint8_t计数器=0；
  //  for (uint8_t i = 1; i < MAX_DEVICES_ALLOWED; i++)//对于（uint8_t i=1；i<允许的最大设备数；i++）
  //    if (thePool[i].address != 0); counter++;//if（池[i]。地址！=0）；计数器++；
  //  return counter;//返回计数器；
  //}//}
};
