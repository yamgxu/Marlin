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
  #error "Never include usb_ch9.h directly; include Usb.h instead"
#endif

/* USB chapter 9 structures */

/* Misc.USB constants */
#define DEV_DESCR_LEN   18      //device descriptor length//设备描述符长度
#define CONF_DESCR_LEN  9       //configuration descriptor length//配置描述符长度
#define INTR_DESCR_LEN  9       //interface descriptor length//接口描述符长度
#define EP_DESCR_LEN    7       //endpoint descriptor length//端点描述符长度

/* Standard Device Requests */

#define USB_REQUEST_GET_STATUS                  0       // Standard Device Request - GET STATUS//标准设备请求-获取状态
#define USB_REQUEST_CLEAR_FEATURE               1       // Standard Device Request - CLEAR FEATURE//标准设备请求-清除功能
#define USB_REQUEST_SET_FEATURE                 3       // Standard Device Request - SET FEATURE//标准设备请求集功能
#define USB_REQUEST_SET_ADDRESS                 5       // Standard Device Request - SET ADDRESS//标准设备请求-设置地址
#define USB_REQUEST_GET_DESCRIPTOR              6       // Standard Device Request - GET DESCRIPTOR//标准设备请求-获取描述符
#define USB_REQUEST_SET_DESCRIPTOR              7       // Standard Device Request - SET DESCRIPTOR//标准设备请求集描述符
#define USB_REQUEST_GET_CONFIGURATION           8       // Standard Device Request - GET CONFIGURATION//标准设备请求-获取配置
#define USB_REQUEST_SET_CONFIGURATION           9       // Standard Device Request - SET CONFIGURATION//标准设备请求-设置配置
#define USB_REQUEST_GET_INTERFACE               10      // Standard Device Request - GET INTERFACE//标准设备请求-获取接口
#define USB_REQUEST_SET_INTERFACE               11      // Standard Device Request - SET INTERFACE//标准设备请求集接口
#define USB_REQUEST_SYNCH_FRAME                 12      // Standard Device Request - SYNCH FRAME//标准设备请求-同步帧

#define USB_FEATURE_ENDPOINT_HALT               0       // CLEAR/SET FEATURE - Endpoint Halt//清除/设置功能-端点暂停
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP        1       // CLEAR/SET FEATURE - Device remote wake-up//清除/设置功能-设备远程唤醒
#define USB_FEATURE_TEST_MODE                   2       // CLEAR/SET FEATURE - Test mode//清除/设置功能-测试模式

/* Setup Data Constants */

#define USB_SETUP_HOST_TO_DEVICE                0x00    // Device Request bmRequestType transfer direction - host to device transfer//设备请求bmRequestType传输方向-主机到设备传输
#define USB_SETUP_DEVICE_TO_HOST                0x80    // Device Request bmRequestType transfer direction - device to host transfer//设备请求bmRequestType传输方向-设备到主机传输
#define USB_SETUP_TYPE_STANDARD                 0x00    // Device Request bmRequestType type - standard//设备请求bmRequestType类型-标准
#define USB_SETUP_TYPE_CLASS                    0x20    // Device Request bmRequestType type - class//设备请求bmRequestType类型-类
#define USB_SETUP_TYPE_VENDOR                   0x40    // Device Request bmRequestType type - vendor//设备请求bmRequestType类型-供应商
#define USB_SETUP_RECIPIENT_DEVICE              0x00    // Device Request bmRequestType recipient - device//设备请求bmRequestType收件人-设备
#define USB_SETUP_RECIPIENT_INTERFACE           0x01    // Device Request bmRequestType recipient - interface//设备请求bmRequestType收件人-接口
#define USB_SETUP_RECIPIENT_ENDPOINT            0x02    // Device Request bmRequestType recipient - endpoint//设备请求bmRequestType收件人-终结点
#define USB_SETUP_RECIPIENT_OTHER               0x03    // Device Request bmRequestType recipient - other//设备请求bmRequestType收件人-其他

/* USB descriptors  */

#define USB_DESCRIPTOR_DEVICE                   0x01    // bDescriptorType for a Device Descriptor.//设备描述符的bDescriptorType。
#define USB_DESCRIPTOR_CONFIGURATION            0x02    // bDescriptorType for a Configuration Descriptor.//配置描述符的bDescriptorType。
#define USB_DESCRIPTOR_STRING                   0x03    // bDescriptorType for a String Descriptor.//b字符串描述符的脚本类型。
#define USB_DESCRIPTOR_INTERFACE                0x04    // bDescriptorType for an Interface Descriptor.//接口描述符的bDescriptorType。
#define USB_DESCRIPTOR_ENDPOINT                 0x05    // bDescriptorType for an Endpoint Descriptor.//终结点描述符的bDescriptorType。
#define USB_DESCRIPTOR_DEVICE_QUALIFIER         0x06    // bDescriptorType for a Device Qualifier.//b设备限定符的脚本类型。
#define USB_DESCRIPTOR_OTHER_SPEED              0x07    // bDescriptorType for a Other Speed Configuration.//b其他速度配置的脚本类型。
#define USB_DESCRIPTOR_INTERFACE_POWER          0x08    // bDescriptorType for Interface Power.//b接口电源的脚本类型。
#define USB_DESCRIPTOR_OTG                      0x09    // bDescriptorType for an OTG Descriptor.//b OTG描述符的脚本类型。

#define HID_DESCRIPTOR_HID                      0x21


/* OTG SET FEATURE Constants    */
#define OTG_FEATURE_B_HNP_ENABLE                3       // SET FEATURE OTG - Enable B device to perform HNP//设置功能OTG-启用B设备执行HNP
#define OTG_FEATURE_A_HNP_SUPPORT               4       // SET FEATURE OTG - A device supports HNP//设置功能OTG-设备支持HNP
#define OTG_FEATURE_A_ALT_HNP_SUPPORT           5       // SET FEATURE OTG - Another port on the A device supports HNP//设置功能OTG-设备上的另一个端口支持HNP

/* USB Endpoint Transfer Types  */
#define USB_TRANSFER_TYPE_CONTROL               0x00    // Endpoint is a control endpoint.//端点是控制端点。
#define USB_TRANSFER_TYPE_ISOCHRONOUS           0x01    // Endpoint is an isochronous endpoint.//端点是等时端点。
#define USB_TRANSFER_TYPE_BULK                  0x02    // Endpoint is a bulk endpoint.//端点是批量端点。
#define USB_TRANSFER_TYPE_INTERRUPT             0x03    // Endpoint is an interrupt endpoint.//端点是一个中断端点。
#define bmUSB_TRANSFER_TYPE                     0x03    // bit mask to separate transfer type from ISO attributes//将传输类型与ISO属性分开的位掩码


/* Standard Feature Selectors for CLEAR_FEATURE Requests    */
#define USB_FEATURE_ENDPOINT_STALL              0       // Endpoint recipient//端点接收者
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP        1       // Device recipient//设备收件人
#define USB_FEATURE_TEST_MODE                   2       // Device recipient//设备收件人

/* descriptor data structures */

/* Device descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.//此描述符的长度。
        uint8_t bDescriptorType; // DEVICE descriptor type (USB_DESCRIPTOR_DEVICE).//设备描述符类型（USB\U描述符\U设备）。
        uint16_t bcdUSB; // USB Spec Release Number (BCD).//USB规范发行号（BCD）。
        uint8_t bDeviceClass; // Class code (assigned by the USB-IF). 0xFF-Vendor specific.//类别代码（由USB-IF分配）。0xFF特定于供应商。
        uint8_t bDeviceSubClass; // Subclass code (assigned by the USB-IF).//子类代码（由USB-IF分配）。
        uint8_t bDeviceProtocol; // Protocol code (assigned by the USB-IF). 0xFF-Vendor specific.//协议代码（由USB-IF分配）。0xFF特定于供应商。
        uint8_t bMaxPacketSize0; // Maximum packet size for endpoint 0.//终结点0的最大数据包大小。
        uint16_t idVendor; // Vendor ID (assigned by the USB-IF).//供应商ID（由USB-IF分配）。
        uint16_t idProduct; // Product ID (assigned by the manufacturer).//产品ID（由制造商指定）。
        uint16_t bcdDevice; // Device release number (BCD).//设备发行号（BCD）。
        uint8_t iManufacturer; // Index of String Descriptor describing the manufacturer.//描述制造商的字符串描述符的索引。
        uint8_t iProduct; // Index of String Descriptor describing the product.//描述产品的字符串描述符的索引。
        uint8_t iSerialNumber; // Index of String Descriptor with the device's serial number.//带有设备序列号的字符串描述符索引。
        uint8_t bNumConfigurations; // Number of possible configurations.//可能的配置数量。
} __attribute__((packed)) USB_FD_DEVICE_DESCRIPTOR;

/* Configuration descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.//此描述符的长度。
        uint8_t bDescriptorType; // CONFIGURATION descriptor type (USB_DESCRIPTOR_CONFIGURATION).//配置描述符类型（USB\U描述符\U配置）。
        uint16_t wTotalLength; // Total length of all descriptors for this configuration.//此配置的所有描述符的总长度。
        uint8_t bNumInterfaces; // Number of interfaces in this configuration.//此配置中的接口数。
        uint8_t bConfigurationValue; // Value of this configuration (1 based).//此配置的值（基于1）。
        uint8_t iConfiguration; // Index of String Descriptor describing the configuration.//描述配置的字符串描述符的索引。
        uint8_t bmAttributes; // Configuration characteristics.//配置特征。
        uint8_t bMaxPower; // Maximum power consumed by this configuration.//此配置消耗的最大功率。
} __attribute__((packed)) USB_FD_CONFIGURATION_DESCRIPTOR;

/* Interface descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.//此描述符的长度。
        uint8_t bDescriptorType; // INTERFACE descriptor type (USB_DESCRIPTOR_INTERFACE).//接口描述符类型（USB\U描述符\U接口）。
        uint8_t bInterfaceNumber; // Number of this interface (0 based).//此接口的编号（基于0）。
        uint8_t bAlternateSetting; // Value of this alternate interface setting.//此备用接口设置的值。
        uint8_t bNumEndpoints; // Number of endpoints in this interface.//此接口中的终结点数。
        uint8_t bInterfaceClass; // Class code (assigned by the USB-IF).  0xFF-Vendor specific.//类别代码（由USB-IF分配）。0xFF特定于供应商。
        uint8_t bInterfaceSubClass; // Subclass code (assigned by the USB-IF).//子类代码（由USB-IF分配）。
        uint8_t bInterfaceProtocol; // Protocol code (assigned by the USB-IF).  0xFF-Vendor specific.//协议代码（由USB-IF分配）。0xFF特定于供应商。
        uint8_t iInterface; // Index of String Descriptor describing the interface.//描述接口的字符串描述符的索引。
} __attribute__((packed)) USB_FD_INTERFACE_DESCRIPTOR;

/* Endpoint descriptor structure */
typedef struct {
        uint8_t bLength; // Length of this descriptor.//此描述符的长度。
        uint8_t bDescriptorType; // ENDPOINT descriptor type (USB_DESCRIPTOR_ENDPOINT).//端点描述符类型（USB_描述符_端点）。
        uint8_t bEndpointAddress; // Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).//端点地址。第7位表示方向（0=输出，1=输入）。
        uint8_t bmAttributes; // Endpoint transfer type.//端点传输类型。
        uint16_t wMaxPacketSize; // Maximum packet size.//最大数据包大小。
        uint8_t bInterval; // Polling interval in frames.//以帧为单位的轮询间隔。
} __attribute__((packed)) USB_FD_ENDPOINT_DESCRIPTOR;

/* HID descriptor */
typedef struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint16_t bcdHID; // HID class specification release//HID类规范发布
        uint8_t bCountryCode;
        uint8_t bNumDescriptors; // Number of additional class specific descriptors//附加类特定描述符的数量
        uint8_t bDescrType; // Type of class descriptor//类描述符的类型
        uint16_t wDescriptorLength; // Total size of the Report descriptor//报表描述符的总大小
} __attribute__((packed)) USB_HID_DESCRIPTOR;

typedef struct {
        uint8_t bDescrType; // Type of class descriptor//类描述符的类型
        uint16_t wDescriptorLength; // Total size of the Report descriptor//报表描述符的总大小
} __attribute__((packed)) HID_CLASS_DESCRIPTOR_LEN_AND_TYPE;
