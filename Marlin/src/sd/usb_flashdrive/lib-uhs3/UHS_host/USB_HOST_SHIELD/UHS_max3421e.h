/** translatione by yx */
/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Circuits At Home, LTD
Web      :  https://www.circuitsathome.com//www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */
#if !defined(USB_HOST_SHIELD_H) || defined(_max3421e_h_)
#error "Never include UHS_max3421e.h directly; include USB_HOST_SHIELD.h instead"
#else

#define _max3421e_h_

/* MAX3421E register/bit names and bitmasks */

#define SE0     0
#define SE1     1
#define FSHOST  2
#define LSHOST  3

/* MAX3421E command byte format: rrrrr0wa where 'r' is register number  */

////
// MAX3421E Registers in HOST mode.//MAX3421E寄存器处于主机模式。
////
#define        rRCVFIFO 0x08            // Receive FIFO Register//接收FIFO寄存器
#define        rSNDFIFO 0x10            // Send FIFO Register//发送FIFO寄存器
#define        rSUDFIFO 0x20            // Set Up Data FIFO Register//设置数据FIFO寄存器
#define          rRCVBC 0x30            // Receive FIFO Byte Count Register//接收FIFO字节计数寄存器
#define          rSNDBC 0x38            // Send FIFO Byte Count Register//发送FIFO字节计数寄存器

// USB Interrupt Request Status (USBIRQ)//USB中断请求状态（USBIRQ）
#define         rUSBIRQ 0x68            // USB Interrupt Request Register//USB中断请求寄存器
#define       bmVBUSIRQ 0x40            // Vbus Present Interrupt Request//Vbus当前中断请求
#define     bmNOVBUSIRQ 0x20            // Vbus Absent Interrupt Request//Vbus缺席中断请求
#define      bmOSCOKIRQ 0x01            // Oscillator OK Interrupt Request//振荡器正常中断请求

// USB Interrupt Request Control (USBIEN)//USB中断请求控制（USBIEN）
#define         rUSBIEN 0x70            // USB Interrupt Request Enable Register//USB中断请求启用寄存器
#define        bmVBUSIE bmVBUSIRQ       // Vbus Present Interrupt Request Enable//Vbus当前中断请求启用
#define      bmNOVBUSIE bmNOVBUSIRQ     // Vbus Absent Interrupt Request Enable//Vbus缺席中断请求启用
#define       bmOSCOKIE bmOSCOKIRQ      // Oscillator OK Interrupt Request Enable//振荡器正常中断请求启用

// (USBCTL)//（USBCTL）
#define         rUSBCTL 0x78            //15<<3//15<<3
#define       bmCHIPRES 0x20            //b5//b5
#define       bmPWRDOWN 0x10            //b4//b4

// (CPUCTL)//（CPUCTL）
#define         rCPUCTL 0x80            //16<<3//16<<3
#define     bmPUSLEWID1 0x80            //b7//b7
#define     bmPULSEWID0 0x40            //b6//b6
#define            bmIE 0x01            //b0//b0

// bmPUSLEWID1 bmPULSEWID0 Pulse width//bmPUSLEWID1 bmPULSEWID0脉冲宽度
// 0           0           10.6uS//0.10.6uS
// 0           1            5.3uS//0 1 5.3uS
// 1           0            2.6uS//102.6uS
// 1           1            1.3uS//1.3美国
#define  PUSLEWIDTH10_6 (0)
#define   PUSLEWIDTH5_3 (bmPULSEWID0)
#define   PUSLEWIDTH2_6 (bmPUSLEWID1)
#define   PUSLEWIDTH1_3 (bmPULSEWID0 | bmPUSLEWID1)

// (PINCTL)//（PINCTL）
#define         rPINCTL 0x88            //17<<3//17<<3
#define       bmFDUPSPI 0x10            //b4//b4
#define      bmINTLEVEL 0x08            //b3//b3
#define        bmPOSINT 0x04            //b2//b2
#define          bmGPXB 0x02            //b1//b1
#define          bmGPXA 0x01            //b0//b0

// GPX pin selections//GPX引脚选择
#define     GPX_OPERATE 0x00            ////
#define       GPX_VBDET 0x01            ////
#define      GPX_BUSACT 0x02            ////
#define         GPX_SOF 0x03            ////

#define       rREVISION 0x90            //18<<3//18<<3

// (IOPINS1)//（IOPINS1）
#define        rIOPINS1 0xA0            //20<<3//20<<3
#define        bmGPOUT0 0x01            ////
#define        bmGPOUT1 0x02            ////
#define        bmGPOUT2 0x04            ////
#define        bmGPOUT3 0x08            ////
#define         bmGPIN0 0x10            ////
#define         bmGPIN1 0x20            ////
#define         bmGPIN2 0x40            ////
#define         bmGPIN3 0x80            ////

// (IOPINS2)//（IOPINS2）
#define        rIOPINS2 0xA8            //21<<3//21<<3
#define        bmGPOUT4 0x01            ////
#define        bmGPOUT5 0x02            ////
#define        bmGPOUT6 0x04            ////
#define        bmGPOUT7 0x08            ////
#define         bmGPIN4 0x10            ////
#define         bmGPIN5 0x20            ////
#define         bmGPIN6 0x40            ////
#define         bmGPIN7 0x80            ////

// (GPINIRQ)//（GPINIRQ）
#define        rGPINIRQ 0xB0            //22<<3//22<<3
#define      bmGPINIRQ0 0x01            ////
#define      bmGPINIRQ1 0x02            ////
#define      bmGPINIRQ2 0x04            ////
#define      bmGPINIRQ3 0x08            ////
#define      bmGPINIRQ4 0x10            ////
#define      bmGPINIRQ5 0x20            ////
#define      bmGPINIRQ6 0x40            ////
#define      bmGPINIRQ7 0x80            ////

// (GPINIEN)//（菲律宾）
#define        rGPINIEN 0xB8            //23<<3//23<<3
#define      bmGPINIEN0 0x01            ////
#define      bmGPINIEN1 0x02            ////
#define      bmGPINIEN2 0x04            ////
#define      bmGPINIEN3 0x08            ////
#define      bmGPINIEN4 0x10            ////
#define      bmGPINIEN5 0x20            ////
#define      bmGPINIEN6 0x40            ////
#define      bmGPINIEN7 0x80            ////

// (GPINPOL)//（GPINPOL）
#define        rGPINPOL 0xC0            //24<<3//24<<3
#define      bmGPINPOL0 0x01            ////
#define      bmGPINPOL1 0x02            ////
#define      bmGPINPOL2 0x04            ////
#define      bmGPINPOL3 0x08            ////
#define      bmGPINPOL4 0x10            ////
#define      bmGPINPOL5 0x20            ////
#define      bmGPINPOL6 0x40            ////
#define      bmGPINPOL7 0x80            ////

////
// If any data transfer errors occur, the HXFRDNIRQ asserts, while the RCVDAVIRQ does not.//如果发生任何数据传输错误，HXFRDNIRQ会断言，而RCVDAVIRQ不会。
////
// The CPU clears the SNDBAVIRQ by writing the SNDBC register.//CPU通过写入SNDBC寄存器来清除SNDBAVIRQ。
// The CPU should never directly clear the SNDBAVIRQ bit.//CPU不应直接清除SNDBAVIRQ位。

// Host Interrupt Request Status (HIRQ)//主机中断请求状态（HIRQ）
#define           rHIRQ 0xC8            // Host Interrupt Request Register//主机中断请求寄存器
#define   bmBUSEVENTIRQ 0x01            // BUS Reset Done or BUS Resume Interrupt Request//总线复位完成或总线恢复中断请求
#define        bmRWUIRQ 0x02            // Remote Wakeup Interrupt Request//远程唤醒中断请求
#define     bmRCVDAVIRQ 0x04            // Receive FIFO Data Available Interrupt Request//接收FIFO数据可用中断请求
#define     bmSNDBAVIRQ 0x08            // Send Buffer Available Interrupt Request//发送缓冲区可用中断请求
#define      bmSUSDNIRQ 0x10            // Suspend operation Done Interrupt Request//暂停操作完成中断请求
#define     bmCONDETIRQ 0x20            // Peripheral Connect/Disconnect Interrupt Request//外围设备连接/断开中断请求
#define      bmFRAMEIRQ 0x40            // Frame Generator Interrupt Request//帧生成器中断请求
#define     bmHXFRDNIRQ 0x80            // Host Transfer Done Interrupt Request//主机传输完成中断请求

// IRQs that are OK for the CPU to clear//CPU可以清除的IRQ
#define     ICLRALLBITS (bmBUSEVENTIRQ | bmRWUIRQ | bmRCVDAVIRQ | bmSUSDNIRQ | bmCONDETIRQ | bmFRAMEIRQ | bmHXFRDNIRQ)

// Host Interrupt Request Control (HIEN)//主机中断请求控制（HIEN）
#define           rHIEN 0xD0            ////
#define    bmBUSEVENTIE bmBUSEVENTIRQ   // BUS Reset Done or BUS Resume Interrupt Request Enable//总线复位完成或总线恢复中断请求启用
#define         bmRWUIE bmRWUIRQ        // Remote Wakeup Interrupt Request Enable//远程唤醒中断请求启用
#define      bmRCVDAVIE bmRCVDAVIRQ     // Receive FIFO Data Available Interrupt Request Enable//接收FIFO数据可用中断请求启用
#define      bmSNDBAVIE bmSNDBAVIRQ     // Send Buffer Available Interrupt Request Enable//发送缓冲区可用中断请求启用
#define       bmSUSDNIE bmSUSDNIRQ      // Suspend operation Done Interrupt Request Enable//暂停操作完成中断请求启用
#define      bmCONDETIE bmCONDETIRQ     // Peripheral Connect/Disconnect Interrupt Request Enable//外围设备连接/断开中断请求启用
#define       bmFRAMEIE bmFRAMEIRQ      // Frame Generator Interrupt Request Enable//帧生成器中断请求启用
#define      bmHXFRDNIE bmHXFRDNIRQ     // Host Transfer Done Interrupt Request Enable//主机传输完成中断请求启用

// (MODE))//（模式）
#define           rMODE 0xD8            //27<<3//27<<3
#define          bmHOST 0x01            ////
#define      bmLOWSPEED 0x02            ////
#define        bmHUBPRE 0x04            ////
#define     bmSOFKAENAB 0x08            ////
#define        bmSEPIRQ 0x10            ////
#define      bmDELAYISO 0x20            ////
#define      bmDMPULLDN 0x40            ////
#define      bmDPPULLDN 0x80            ////

#define        rPERADDR 0xE0            //28<<3//28<<3

// (HCTL)//（HCTL）
#define           rHCTL 0xE8            //29<<3//29<<3
#define        bmBUSRST 0x01            ////
#define        bmFRMRST 0x02            ////
#define     bmSAMPLEBUS 0x04            ////
#define        bmSIGRSM 0x08            ////
#define       bmRCVTOG0 0x10            ////
#define       bmRCVTOG1 0x20            ////
#define       bmSNDTOG0 0x40            ////
#define       bmSNDTOG1 0x80            ////

// Host transfer (HXFR)//主机传输（HXFR）
#define           rHXFR 0xF0            //30<<3//30<<3
/* Host transfer token values for writing the HXFR register (R30)   */
/* OR this bit field with the endpoint number in bits 3:0               */
#define       MAX3421E_tokSETUP 0x10    // HS=0, ISO=0, OUTNIN=0, SETUP=1//HS=0，ISO=0，OUTNIN=0，SETUP=1
#define          MAX3421E_tokIN 0x00    // HS=0, ISO=0, OUTNIN=0, SETUP=0//HS=0，ISO=0，OUTNIN=0，SETUP=0
#define         MAX3421E_tokOUT 0x20    // HS=0, ISO=0, OUTNIN=1, SETUP=0//HS=0，ISO=0，OUTNIN=1，SETUP=0
#define        MAX3421E_tokINHS 0x80    // HS=1, ISO=0, OUTNIN=0, SETUP=0//HS=1，ISO=0，OUTNIN=0，SETUP=0
#define       MAX3421E_tokOUTHS 0xA0    // HS=1, ISO=0, OUTNIN=1, SETUP=0//HS=1，ISO=0，OUTNIN=1，SETUP=0
#define       MAX3421E_tokISOIN 0x40    // HS=0, ISO=1, OUTNIN=0, SETUP=0//HS=0，ISO=1，OUTNIN=0，SETUP=0
#define      MAX3421E_tokISOOUT 0x60    // HS=0, ISO=1, OUTNIN=1, SETUP=0//HS=0，ISO=1，OUTNIN=1，SETUP=0

// (HRSL)//（HRSL）
#define           rHRSL 0xF8            //31<<3//31<<3
#define      bmRCVTOGRD 0x10            ////
#define      bmSNDTOGRD 0x20            ////
#define       bmKSTATUS 0x40            ////
#define       bmJSTATUS 0x80            ////
#define           bmSE0 0x00            //SE0 - disconnect state//SE0-断开状态
#define           bmSE1 0xC0            //SE1 - illegal state//SE1-非法状态

#define    MODE_FS_HOST (bmDPPULLDN|bmDMPULLDN|bmHOST|bmSOFKAENAB)
#define    MODE_LS_HOST (bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB)

#endif //_max3421e_h_//_max3421e_h_
