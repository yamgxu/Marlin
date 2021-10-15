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

#ifndef _usb_h_
  #error "Never include UsbCore.h directly; include Usb.h instead"
#endif

#pragma once

// Not used anymore? If anyone uses this, please let us know so that this may be//不再使用了？如果有人使用，请让我们知道，以便
// moved to the proper place, settings.h.//移动到正确的位置，设置。
//#define USB_METHODS_INLINE//#定义USB\u方法\u内联

/* shield pins. First parameter - SS pin, second parameter - INT pin */

#ifdef __MARLIN_FIRMWARE__
typedef MAX3421e MAX3421E; // Marlin redefines this class in "../usb_host.h"//Marlin在“./usb\u host.h”中重新定义了此类
#elif defined(BOARD_BLACK_WIDDOW)
typedef MAX3421e<P6, P3> MAX3421E; // Black Widow//黑寡妇
#elif defined(CORE_TEENSY) && (defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__))
#if EXT_RAM
typedef MAX3421e<P20, P7> MAX3421E; // Teensy++ 2.0 with XMEM2//Teensy++2.0与XMEM2
#else
typedef MAX3421e<P9, P8> MAX3421E; // Teensy++ 1.0 and 2.0//Teensy++1.0和2.0
#endif
#elif defined(BOARD_MEGA_ADK)
typedef MAX3421e<P53, P54> MAX3421E; // Arduino Mega ADK//Arduino Mega ADK
#elif defined(ARDUINO_AVR_BALANDUINO)
typedef MAX3421e<P20, P19> MAX3421E; // Balanduino//巴兰迪诺
#elif defined(__ARDUINO_X86__) && PLATFORM_ID == 0x06
typedef MAX3421e<P3, P2> MAX3421E; // The Intel Galileo supports much faster read and write speed at pin 2 and 3//Intel Galileo在引脚2和引脚3上支持更快的读写速度
#elif defined(ESP8266)
typedef MAX3421e<P15, P5> MAX3421E; // ESP8266 boards//ESP8266板
#elif defined(ESP32)
typedef MAX3421e<P5, P17> MAX3421E; // ESP32 boards//ESP32板
#else
typedef MAX3421e<P10, P9> MAX3421E; // Official Arduinos (UNO, Duemilanove, Mega, 2560, Leonardo, Due etc.), Intel Edison, Intel Galileo 2 or Teensy 2.0 and 3.x//官方Arduinos（UNO、Duemilanove、Mega、2560、Leonardo、Due等）、Intel Edison、Intel Galileo 2或Teensy 2.0和3.x
#endif

/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //get descriptor request type//获取描述符请求类型
#define bmREQ_SET           USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //set request type for all but 'set feature' and 'set interface'//设置除“设置功能”和“设置接口”之外的所有请求类型
#define bmREQ_CL_GET_INTF   USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE     //get interface request type//获取接口请求类型

// D7           data transfer direction (0 - host-to-device, 1 - device-to-host)//D7数据传输方向（0-主机到设备，1-设备到主机）
// D6-5         Type (0- standard, 1 - class, 2 - vendor, 3 - reserved)//D6-5型（0-标准，1-等级，2-供应商，3-保留）
// D4-0         Recipient (0 - device, 1 - interface, 2 - endpoint, 3 - other, 4..31 - reserved)//D4-0收件人（0-设备，1-接口，2-端点，3-其他，4..31-保留）

// USB Device Classes//USB设备类
#define USB_CLASS_USE_CLASS_INFO        0x00    // Use Class Info in the Interface Descriptors//在接口描述符中使用类信息
#define USB_CLASS_AUDIO                 0x01    // Audio//音频
#define USB_CLASS_COM_AND_CDC_CTRL      0x02    // Communications and CDC Control//通信和疾病控制中心控制
#define USB_CLASS_HID                   0x03    // HID//隐藏
#define USB_CLASS_PHYSICAL              0x05    // Physical//物理的
#define USB_CLASS_IMAGE                 0x06    // Image//形象
#define USB_CLASS_PRINTER               0x07    // Printer//印刷机
#define USB_CLASS_MASS_STORAGE          0x08    // Mass Storage//大容量存储
#define USB_CLASS_HUB                   0x09    // Hub//枢纽
#define USB_CLASS_CDC_DATA              0x0A    // CDC-Data//CDC数据
#define USB_CLASS_SMART_CARD            0x0B    // Smart-Card//智能卡
#define USB_CLASS_CONTENT_SECURITY      0x0D    // Content Security//内容安全
#define USB_CLASS_VIDEO                 0x0E    // Video//录像带
#define USB_CLASS_PERSONAL_HEALTH       0x0F    // Personal Healthcare//个人保健
#define USB_CLASS_DIAGNOSTIC_DEVICE     0xDC    // Diagnostic Device//诊断装置
#define USB_CLASS_WIRELESS_CTRL         0xE0    // Wireless Controller//无线控制器
#define USB_CLASS_MISC                  0xEF    // Miscellaneous//杂
#define USB_CLASS_APP_SPECIFIC          0xFE    // Application Specific//特定于应用程序
#define USB_CLASS_VENDOR_SPECIFIC       0xFF    // Vendor Specific//特定于供应商

// Additional Error Codes//附加错误代码
#define USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED       0xD1
#define USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE     0xD2
#define USB_ERROR_UNABLE_TO_REGISTER_DEVICE_CLASS       0xD3
#define USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL          0xD4
#define USB_ERROR_HUB_ADDRESS_OVERFLOW                  0xD5
#define USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL             0xD6
#define USB_ERROR_EPINFO_IS_NULL                        0xD7
#define USB_ERROR_INVALID_ARGUMENT                      0xD8
#define USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE         0xD9
#define USB_ERROR_INVALID_MAX_PKT_SIZE                  0xDA
#define USB_ERROR_EP_NOT_FOUND_IN_TBL                   0xDB
#define USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET      0xE0
#define USB_ERROR_FailGetDevDescr                       0xE1
#define USB_ERROR_FailSetDevTblEntry                    0xE2
#define USB_ERROR_FailGetConfDescr                      0xE3
#define USB_ERROR_TRANSFER_TIMEOUT                      0xFF

#define USB_XFER_TIMEOUT        5000    // (5000) USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec//（5000）USB传输超时，以毫秒为单位，符合USB 2.0规范第9.2.6.1节
//#define USB_NAK_LIMIT         32000   // NAK limit for a transfer. 0 means NAKs are not counted//#为传输定义USB_NAK_LIMIT 32000//NAK LIMIT。0表示不计算NAK
#define USB_RETRY_LIMIT         3       // 3 retry limit for a transfer//3传输的重试限制
#define USB_SETTLE_DELAY        200     // settle delay in milliseconds//以毫秒为单位结算延迟

#define USB_NUMDEVICES          16      //number of USB devices//USB设备的数量
//#define HUB_MAX_HUBS          7       // maximum number of hubs that can be attached to the host controller//#定义集线器\u MAX\u集线器7//可以连接到主机控制器的最大集线器数
#define HUB_PORT_RESET_DELAY    20      // hub port reset delay 10 ms recomended, can be up to 20 ms//集线器端口重置延迟建议为10毫秒，最长可达20毫秒

/* USB state machine states */
#define USB_STATE_MASK                                      0xF0

#define USB_STATE_DETACHED                                  0x10
#define USB_DETACHED_SUBSTATE_INITIALIZE                    0x11
#define USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE               0x12
#define USB_DETACHED_SUBSTATE_ILLEGAL                       0x13
#define USB_ATTACHED_SUBSTATE_SETTLE                        0x20
#define USB_ATTACHED_SUBSTATE_RESET_DEVICE                  0x30
#define USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE           0x40
#define USB_ATTACHED_SUBSTATE_WAIT_SOF                      0x50
#define USB_ATTACHED_SUBSTATE_WAIT_RESET                    0x51
#define USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE    0x60
#define USB_STATE_ADDRESSING                                0x70
#define USB_STATE_CONFIGURING                               0x80
#define USB_STATE_RUNNING                                   0x90
#define USB_STATE_ERROR                                     0xA0

class USBDeviceConfig {
public:

  virtual uint8_t Init(uint8_t parent __attribute__((unused)), uint8_t port __attribute__((unused)), bool lowspeed __attribute__((unused))) {
    return 0;
  }

  virtual uint8_t ConfigureDevice(uint8_t parent __attribute__((unused)), uint8_t port __attribute__((unused)), bool lowspeed __attribute__((unused))) {
    return 0;
  }

  virtual uint8_t Release() {
    return 0;
  }

  virtual uint8_t Poll() {
    return 0;
  }

  virtual uint8_t GetAddress() {
    return 0;
  }

  virtual void ResetHubPort(uint8_t port __attribute__((unused))) {
    return;
  } // Note used for hubs only!//注意：仅用于集线器！

  virtual bool VIDPIDOK(uint16_t vid __attribute__((unused)), uint16_t pid __attribute__((unused))) {
    return false;
  }

  virtual bool DEVCLASSOK(uint8_t klass __attribute__((unused))) {
    return false;
  }

  virtual bool DEVSUBCLASSOK(uint8_t subklass __attribute__((unused))) {
    return true;
  }

};

/* USB Setup Packet Structure   */
typedef struct {

  union { // offset   description//偏移量说明
    uint8_t bmRequestType; //   0      Bit-map of request type//请求类型的0位映射

    struct {
      uint8_t recipient : 5; //          Recipient of the request//请求接收人
      uint8_t type : 2; //          Type of request//请求类型
      uint8_t direction : 1; //          Direction of data X-fer//数据传输方向
    } __attribute__((packed));
  } ReqType_u;
  uint8_t bRequest; //   1      Request//1请求

  union {
    uint16_t wValue; //   2      Depends on bRequest//2取决于酿造

    struct {
      uint8_t wValueLo;
      uint8_t wValueHi;
    } __attribute__((packed));
  } wVal_u;
  uint16_t wIndex; //   4      Depends on bRequest//4靠酿造
  uint16_t wLength; //   6      Depends on bRequest//6靠酿造
} __attribute__((packed)) SETUP_PKT, *PSETUP_PKT;



// Base class for incoming data parser//传入数据解析器的基类

class USBReadParser {
public:
  virtual void Parse(const uint16_t len, const uint8_t *pbuf, const uint16_t &offset) = 0;
};

class USB : public MAX3421E {
  AddressPoolImpl<USB_NUMDEVICES> addrPool;
  USBDeviceConfig* devConfig[USB_NUMDEVICES];
  uint8_t bmHubPre;

public:
  USB();

  void SetHubPreMask() {
    bmHubPre |= bmHUBPRE;
  };

  void ResetHubPreMask() {
    bmHubPre &= (~bmHUBPRE);
  };

  AddressPool& GetAddressPool() {
    return (AddressPool&)addrPool;
  };

  uint8_t RegisterDeviceClass(USBDeviceConfig *pdev) {
    for (uint8_t i = 0; i < USB_NUMDEVICES; i++) {
      if (!devConfig[i]) {
        devConfig[i] = pdev;
        return 0;
      }
    }
    return USB_ERROR_UNABLE_TO_REGISTER_DEVICE_CLASS;
  };

  void ForEachUsbDevice(UsbDeviceHandleFunc pfunc) {
    addrPool.ForEachUsbDevice(pfunc);
  };
  uint8_t getUsbTaskState();
  void setUsbTaskState(uint8_t state);

  EpInfo* getEpInfoEntry(uint8_t addr, uint8_t ep);
  uint8_t setEpInfoEntry(uint8_t addr, uint8_t epcount, EpInfo* eprecord_ptr);

  /* Control requests */
  uint8_t getDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *dataptr);
  uint8_t getConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, uint8_t *dataptr);

  uint8_t getConfDescr(uint8_t addr, uint8_t ep, uint8_t conf, USBReadParser *p);

  uint8_t getStrDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t index, uint16_t langid, uint8_t *dataptr);
  uint8_t setAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr);
  uint8_t setConf(uint8_t addr, uint8_t ep, uint8_t conf_value);
  /**/
  uint8_t ctrlData(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *dataptr, bool direction);
  uint8_t ctrlStatus(uint8_t ep, bool direction, uint16_t nak_limit);
  uint8_t inTransfer(uint8_t addr, uint8_t ep, uint16_t *nbytesptr, uint8_t *data, uint8_t bInterval = 0);
  uint8_t outTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *data);
  uint8_t dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit);

  void Task();

  uint8_t DefaultAddressing(uint8_t parent, uint8_t port, bool lowspeed);
  uint8_t Configuring(uint8_t parent, uint8_t port, bool lowspeed);
  uint8_t ReleaseDevice(uint8_t addr);

  uint8_t ctrlReq(uint8_t addr, uint8_t ep, uint8_t bmReqType, uint8_t bRequest, uint8_t wValLo, uint8_t wValHi,
    uint16_t wInd, uint16_t total, uint16_t nbytes, uint8_t *dataptr, USBReadParser *p);

private:
  void init();
  uint8_t SetAddress(uint8_t addr, uint8_t ep, EpInfo **ppep, uint16_t *nak_limit);
  uint8_t OutTransfer(EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data);
  uint8_t InTransfer(EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data, uint8_t bInterval = 0);
  uint8_t AttemptConfig(uint8_t driver, uint8_t parent, uint8_t port, bool lowspeed);
};

#if 0 //defined(USB_METHODS_INLINE)//已定义（USB_方法_内联）
//get device descriptor//获取设备描述符

inline uint8_t USB::getDevDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, nbytes, dataptr));
}
//get configuration descriptor//获取配置描述符

inline uint8_t USB::getConfDescr(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t conf, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, nbytes, dataptr));
}
//get string descriptor//获取字符串描述符

inline uint8_t USB::getStrDescr(uint8_t addr, uint8_t ep, uint16_t nuint8_ts, uint8_t index, uint16_t langid, uint8_t *dataptr) {
  return ( ctrlReq(addr, ep, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, langid, nuint8_ts, dataptr));
}
//set address//设定地址

inline uint8_t USB::setAddr(uint8_t oldaddr, uint8_t ep, uint8_t newaddr) {
  return ( ctrlReq(oldaddr, ep, bmREQ_SET, USB_REQUEST_SET_ADDRESS, newaddr, 0x00, 0x0000, 0x0000, nullptr));
}
//set configuration//设置配置

inline uint8_t USB::setConf(uint8_t addr, uint8_t ep, uint8_t conf_value) {
  return ( ctrlReq(addr, ep, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, conf_value, 0x00, 0x0000, 0x0000, nullptr));
}

#endif // defined(USB_METHODS_INLINE)//已定义（USB_方法_内联）
