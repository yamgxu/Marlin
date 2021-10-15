/** translatione by yx */
/* Copyright (C) 2015-2016 Andrew J. Kroll
   and
Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as publishe7d by the Free Software
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

#if defined(USB_HOST_SHIELD_H) && !defined(USB_HOST_SHIELD_LOADED)
#define USB_HOST_SHIELD_LOADED
#include <Arduino.h>

#ifndef digitalPinToInterrupt
#error digitalPinToInterrupt not defined, complain to your board maintainer.
#endif


#if USB_HOST_SHIELD_USE_ISR

// allow two slots. this makes the maximum allowed shield count TWO//允许两个插槽。这使得允许的最大屏蔽计数为2
// for AVRs this is limited to pins 2 and 3 ONLY//对于AVRs，这仅限于引脚2和3
// for all other boards, one odd and one even pin number is allowed.//对于所有其他电路板，允许使用一个奇数和一个偶数管脚编号。
static MAX3421E_HOST *ISReven;
static MAX3421E_HOST *ISRodd;

static void UHS_NI call_ISReven() {
        ISReven->ISRTask();
}

static void UHS_NI call_ISRodd() {
        UHS_PIN_WRITE(LED_BUILTIN, HIGH);
        ISRodd->ISRTask();
}
#endif


void UHS_NI MAX3421E_HOST::resume_host() {
                // Used on MCU that lack control of IRQ priority (AVR).//用于缺少IRQ优先级（AVR）控制的MCU。
                // Resumes ISRs.//恢复ISRs。
                // NOTE: you must track the state yourself!//注意：您必须自己跟踪状态！
#ifdef __AVR__
                noInterrupts();
                if(irq_pin & 1) {
                        ISRodd = this;
                        attachInterrupt(UHS_GET_DPI(irq_pin), call_ISRodd, IRQ_SENSE);
                } else {
                        ISReven = this;
                        attachInterrupt(UHS_GET_DPI(irq_pin), call_ISReven, IRQ_SENSE);
                }
                interrupts();
#endif

}
/* write single byte into MAX3421e register */
void UHS_NI MAX3421E_HOST::regWr(uint8_t reg, uint8_t data) {
        SPIclass.beginTransaction(MAX3421E_SPI_Settings);
        MARLIN_UHS_WRITE_SS(LOW);
        SPIclass.transfer(reg | 0x02);
        SPIclass.transfer(data);
        MARLIN_UHS_WRITE_SS(HIGH);
        SPIclass.endTransaction();
}


/* multiple-byte write                            */

/* returns a pointer to memory position after last written */
uint8_t* UHS_NI MAX3421E_HOST::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
        SPIclass.beginTransaction(MAX3421E_SPI_Settings);
        MARLIN_UHS_WRITE_SS(LOW);
        SPIclass.transfer(reg | 0x02);
        //printf("%2.2x :", reg);//printf（“%2.2x:”，reg）；

        while(nbytes) {
                SPIclass.transfer(*data_p);
                //printf("%2.2x ", *data_p);//printf（“%2.2x”，*数据）；
                nbytes--;
                data_p++; // advance data pointer//高级数据指针
        }
        MARLIN_UHS_WRITE_SS(HIGH);
        SPIclass.endTransaction();
        //printf("\r\n");//printf（“\r\n”）；
        return (data_p);
}
/* GPIO write                                           */
/*GPIO byte is split between 2 registers, so two writes are needed to write one byte */

/* GPOUT bits are in the low nybble. 0-3 in IOPINS1, 4-7 in IOPINS2 */
void UHS_NI MAX3421E_HOST::gpioWr(uint8_t data) {
        regWr(rIOPINS1, data);
        data >>= 4;
        regWr(rIOPINS2, data);
        return;
}

/* single host register read    */
uint8_t UHS_NI MAX3421E_HOST::regRd(uint8_t reg) {
        SPIclass.beginTransaction(MAX3421E_SPI_Settings);
        MARLIN_UHS_WRITE_SS(LOW);
        SPIclass.transfer(reg);
        uint8_t rv = SPIclass.transfer(0);
        MARLIN_UHS_WRITE_SS(HIGH);
        SPIclass.endTransaction();
        return (rv);
}
/* multiple-byte register read  */

/* returns a pointer to a memory position after last read   */
uint8_t* UHS_NI MAX3421E_HOST::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p) {
        SPIclass.beginTransaction(MAX3421E_SPI_Settings);
        MARLIN_UHS_WRITE_SS(LOW);
        SPIclass.transfer(reg);
        while(nbytes) {
                *data_p++ = SPIclass.transfer(0);
                nbytes--;
        }
        MARLIN_UHS_WRITE_SS(HIGH);
        SPIclass.endTransaction();
        return ( data_p);
}

/* GPIO read. See gpioWr for explanation */

/* GPIN pins are in high nybbles of IOPINS1, IOPINS2    */
uint8_t UHS_NI MAX3421E_HOST::gpioRd() {
        uint8_t gpin = 0;
        gpin = regRd(rIOPINS2); //pins 4-7//引脚4-7
        gpin &= 0xF0; //clean lower nybble//清洁下Nyble
        gpin |= (regRd(rIOPINS1) >> 4); //shift low bits and OR with upper from previous operation.//将低位和或与上一操作的高位相移。
        return ( gpin);
}

/* reset MAX3421E. Returns number of microseconds it took for PLL to stabilize after reset
  or zero if PLL haven't stabilized in 65535 cycles */
uint16_t UHS_NI MAX3421E_HOST::reset() {
        uint16_t i = 0;

        // Initiate chip reset//启动芯片复位
        regWr(rUSBCTL, bmCHIPRES);
        regWr(rUSBCTL, 0x00);

        int32_t now;
        uint32_t expires = micros() + 65535;

        // Enable full-duplex SPI so we can read rUSBIRQ//启用全双工SPI以便我们可以读取rUSBIRQ
        regWr(rPINCTL, bmFDUPSPI);
        while((int32_t)(micros() - expires) < 0L) {
                if((regRd(rUSBIRQ) & bmOSCOKIRQ)) {
                        break;
                }
        }
        now = (int32_t)(micros() - expires);
        if(now < 0L) {
                i = 65535 + now; // Note this subtracts, as now is negative//请注意，这是减法，因为现在是负数
        }
        return (i);
}

void UHS_NI MAX3421E_HOST::VBUS_changed() {
        /* modify USB task state because Vbus changed or unknown */
        uint8_t speed = 1;
        //printf("\r\n\r\n\r\n\r\nSTATE %2.2x -> ", usb_task_state);//printf（“\r\n\r\n\r\n\r\n状态%2.2x->”，usb任务状态）；
        switch(vbusState) {
                case LSHOST: // Low speed//低速

                        speed = 0;
                        // Intentional fall-through//故意落空
                case FSHOST: // Full speed//全速
                        // Start device initialization if we are not initializing//如果没有初始化，请启动设备初始化
                        // Resets to the device cause an IRQ//重置到设备会导致IRQ
                        // usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;//usb_任务_状态==UHS_usb_主机_状态_重置_未完成；
                        //if((usb_task_state & UHS_USB_HOST_STATE_MASK) != UHS_USB_HOST_STATE_DETACHED) {//如果（（usb_任务_状态和UHS_usb_主机_状态_掩码）！=UHS_usb_主机_状态_分离）{
                        ReleaseChildren();
                        if(!doingreset) {
                                if(usb_task_state == UHS_USB_HOST_STATE_RESET_NOT_COMPLETE) {
                                        usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
                                } else if(usb_task_state != UHS_USB_HOST_STATE_WAIT_BUS_READY) {
                                        usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE;
                                }
                        }
                        sof_countdown = 0;
                        break;
                case SE1: //illegal state//非法国家
                        sof_countdown = 0;
                        doingreset = false;
                        ReleaseChildren();
                        usb_task_state = UHS_USB_HOST_STATE_ILLEGAL;
                        break;
                case SE0: //disconnected//断开
                default:
                        sof_countdown = 0;
                        doingreset = false;
                        ReleaseChildren();
                        usb_task_state = UHS_USB_HOST_STATE_IDLE;
                        break;
        }
        usb_host_speed = speed;
        //printf("0x%2.2x\r\n\r\n\r\n\r\n", usb_task_state);//printf（“0x%2.2x\r\n\r\n\r\n\r\n”，usb\u任务\u状态）；
        return;
};

/**
 *  Probe bus to determine device presence and speed,
 *  then switch host to detected speed.
 */
void UHS_NI MAX3421E_HOST::busprobe() {
        uint8_t bus_sample;
        uint8_t tmpdata;
        bus_sample = regRd(rHRSL); //Get J,K status//获得J，K状态
        bus_sample &= (bmJSTATUS | bmKSTATUS); //zero the rest of the byte//将剩余字节归零
        switch(bus_sample) { //start full-speed or low-speed host//启动全速或低速主机
                case(bmJSTATUS):
                        // Serial.println("J");//序列号：println（“J”）；
                        if((regRd(rMODE) & bmLOWSPEED) == 0) {
                                regWr(rMODE, MODE_FS_HOST); // start full-speed host//启动全速主机
                                vbusState = FSHOST;
                        } else {
                                regWr(rMODE, MODE_LS_HOST); // start low-speed host//启动低速主机
                                vbusState = LSHOST;
                        }
                        #ifdef USB_HOST_MANUAL_POLL
                                enable_frame_irq(true);
                        #endif
                        tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation//开始SOF生成
                        regWr(rHIRQ, bmFRAMEIRQ); // see data sheet.//见数据表。
                        regWr(rMODE, tmpdata);
                        break;
                case(bmKSTATUS):
                        // Serial.println("K");//序列号。println（“K”）；
                        if((regRd(rMODE) & bmLOWSPEED) == 0) {
                                regWr(rMODE, MODE_LS_HOST); // start low-speed host//启动低速主机
                                vbusState = LSHOST;
                        } else {
                                regWr(rMODE, MODE_FS_HOST); // start full-speed host//启动全速主机
                                vbusState = FSHOST;
                        }
                        #ifdef USB_HOST_MANUAL_POLL
                                enable_frame_irq(true);
                        #endif
                        tmpdata = regRd(rMODE) | bmSOFKAENAB; // start SOF generation//开始SOF生成
                        regWr(rHIRQ, bmFRAMEIRQ); // see data sheet.//见数据表。
                        regWr(rMODE, tmpdata);
                        break;
                case(bmSE1): //illegal state//非法国家
                        // Serial.println("I");//序列号。打印号（“I”）；
                        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
                        vbusState = SE1;
                        // sofevent = false;//sofant=false；
                        break;
                case(bmSE0): //disconnected state//断开状态
                        // Serial.println("D");//序列号。打印号（“D”）；
                        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);
                        vbusState = SE0;
                        // sofevent = false;//sofant=false；
                        break;
        }//end switch( bus_sample )//终端开关（总线_示例）
}

/**
 * Initialize USB hardware, turn on VBUS
 *
 * @param mseconds Delay energizing VBUS after mseconds, A value of INT16_MIN means no delay.
 * @return 0 on success, -1 on error
 */
int16_t UHS_NI MAX3421E_HOST::Init(int16_t mseconds) {
        usb_task_state = UHS_USB_HOST_STATE_INITIALIZE; //set up state machine//设置状态机
        //        Serial.print("MAX3421E 'this' USB Host @ 0x");//Serial.print（“MAX3421E”this“USB主机@0x”）；
        //        Serial.println((uint32_t)this, HEX);//Serial.println（（uint32_t）this，HEX）；
        //        Serial.print("MAX3421E 'this' USB Host Address Pool @ 0x");//Serial.print（“MAX3421E'this'USB主机地址池@0x”）；
        //        Serial.println((uint32_t)GetAddressPool(), HEX);//Serial.println（（uint32_t）GetAddressPool（），十六进制）；
        Init_dyn_SWI();
        UHS_printf_HELPER_init();
        noInterrupts();
#ifdef ARDUINO_AVR_ADK
        // For Mega ADK, which has a Max3421e on-board,//对于搭载Max3421e的Mega ADK，
        // set MAX_RESET to output mode, and then set it to HIGH//将MAX_RESET设置为输出模式，然后将其设置为高
        // PORTJ bit 2//端口J第2位
        if(irq_pin == 54) {
                DDRJ |= 0x04; // output//输出
                PORTJ |= 0x04; // HIGH//高
        }
#endif
        SPIclass.begin();
#ifdef ARDUINO_AVR_ADK
        if(irq_pin == 54) {
                DDRE &= ~0x20; // input//输入
                PORTE |= 0x20; // pullup//拉起
        } else
#endif
                pinMode(irq_pin, INPUT_PULLUP);
        //UHS_PIN_WRITE(irq_pin, HIGH);//UHS_引脚_写入（irq_引脚，高）；
        pinMode(ss_pin, OUTPUT);
        MARLIN_UHS_WRITE_SS(HIGH);

#ifdef USB_HOST_SHIELD_TIMING_PIN
        pinMode(USB_HOST_SHIELD_TIMING_PIN, OUTPUT);
        // My counter/timer can't work on an inverted gate signal//我的计数器/计时器不能在反转的门信号上工作
        // so we gate using a high pulse -- AJK//所以我们用一个高脉冲——AJK进行选通
        UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, LOW);
#endif
        interrupts();

#if USB_HOST_SHIELD_USE_ISR
        int intr = digitalPinToInterrupt(irq_pin);
        if(intr == NOT_AN_INTERRUPT) {
#ifdef ARDUINO_AVR_ADK
                if(irq_pin == 54)
                        intr = 6;
                else
#endif
                        return (-2);
        }
        SPIclass.usingInterrupt(intr);
#else
        SPIclass.usingInterrupt(255);
#endif
#ifndef NO_AUTO_SPEED
        // test to get to reset acceptance.//测试以获得重置验收。
        uint32_t spd = UHS_MAX3421E_SPD;
again:
        MAX3421E_SPI_Settings = SPISettings(spd, MSBFIRST, SPI_MODE0);
        if(reset() == 0) {
                MAX_HOST_DEBUG(PSTR("Fail SPI speed %lu\r\n"), spd);
                if(spd > 1999999) {
                        spd -= 1000000;
                        goto again;
                }
                return (-1);
        } else {
                // reset passes, does 64k?//重置通行证，是64k吗？
                uint8_t sample_wr = 0;
                uint8_t sample_rd = 0;
                uint8_t gpinpol_copy = regRd(rGPINPOL);
                for(uint16_t j = 0; j < 65535; j++) {
                        regWr(rGPINPOL, sample_wr);
                        sample_rd = regRd(rGPINPOL);
                        if(sample_rd != sample_wr) {
                                MAX_HOST_DEBUG(PSTR("Fail SPI speed %lu\r\n"), spd);
                                if(spd > 1999999) {
                                        spd -= 1000000;
                                        goto again;
                                }
                                return (-1);
                        }
                        sample_wr++;
                }
                regWr(rGPINPOL, gpinpol_copy);
        }

        MAX_HOST_DEBUG(PSTR("Pass SPI speed %lu\r\n"), spd);
#endif

        if(reset() == 0) { //OSCOKIRQ hasn't asserted in time//OSCOKIRQ没有及时声明
                MAX_HOST_DEBUG(PSTR("OSCOKIRQ hasn't asserted in time"));
                return ( -1);
        }

        /* MAX3421E - full-duplex SPI, interrupt kind, vbus off */
        regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE | GPX_VBDET));

        // Delay a minimum of 1 second to ensure any capacitors are drained.//延迟至少1秒，以确保电容器耗尽。
        // 1 second is required to make sure we do not smoke a Microdrive!//需要1秒来确保我们不吸Microdrive！
        if(mseconds != INT16_MIN) {
                if(mseconds < 1000) mseconds = 1000;
                delay(mseconds); // We can't depend on SOF timer here.//在这里我们不能依赖软件定时器。
        }

        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host//设置下拉菜单，主机

        // Enable interrupts on the MAX3421e//在MAX3421e上启用中断
        regWr(rHIEN, IRQ_CHECK_MASK);
        // Enable interrupt pin on the MAX3421e, set pulse width for edge//启用MAX3421e上的中断引脚，设置边缘的脉冲宽度
        regWr(rCPUCTL, (bmIE | bmPULSEWIDTH));

        /* check if device is connected */
        regWr(rHCTL, bmSAMPLEBUS); // sample USB bus//USB总线示例
        while(!(regRd(rHCTL) & bmSAMPLEBUS)); //wait for sample operation to finish//等待样本操作完成

        busprobe(); //check if anything is connected//检查是否有任何连接
        VBUS_changed();

        // GPX pin on. This is done here so that a change is detected if we have a switch connected.//GPX引脚打开。这是在此处完成的，以便在连接开关时检测到更改。
        /* MAX3421E - full-duplex SPI, interrupt kind, vbus on */
        regWr(rPINCTL, (bmFDUPSPI | bmIRQ_SENSE));
        regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.//见数据表。
        regWr(rHCTL, bmBUSRST); // issue bus reset to force generate yet another possible IRQ//发布总线重置以强制生成另一个可能的IRQ


#if USB_HOST_SHIELD_USE_ISR
        // Attach ISR to service IRQ from MAX3421e//从MAX3421e将ISR连接到服务IRQ
        noInterrupts();
        if(irq_pin & 1) {
                ISRodd = this;
                attachInterrupt(UHS_GET_DPI(irq_pin), call_ISRodd, IRQ_SENSE);
        } else {
                ISReven = this;
                attachInterrupt(UHS_GET_DPI(irq_pin), call_ISReven, IRQ_SENSE);
        }
        interrupts();
#endif
        //printf("\r\nrPINCTL 0x%2.2X\r\n", rPINCTL);//printf（“\r\nrPINCTL 0x%2.2X\r\n”，rPINCTL）；
        //printf("rCPUCTL 0x%2.2X\r\n", rCPUCTL);//printf（“rCPUCTL 0x%2.2X\r\n”，rCPUCTL）；
        //printf("rHIEN 0x%2.2X\r\n", rHIEN);//printf（“rHIEN 0x%2.2X\r\n”，rHIEN）；
        //printf("irq_pin %i\r\n", irq_pin);//printf（“irq_pin%i\r\n”，irq_pin）；
        return 0;
}

/**
 * Setup UHS_EpInfo structure
 *
 * @param addr USB device address
 * @param ep Endpoint
 * @param ppep pointer to the pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::SetAddress(uint8_t addr, uint8_t ep, UHS_EpInfo **ppep, uint16_t &nak_limit) {
        UHS_Device *p = addrPool.GetUsbDevicePtr(addr);

        if(!p)
                return UHS_HOST_ERROR_NO_ADDRESS_IN_POOL;

        if(!p->epinfo)
                return UHS_HOST_ERROR_NULL_EPINFO;

        *ppep = getEpInfoEntry(addr, ep);

        if(!*ppep)
                return UHS_HOST_ERROR_NO_ENDPOINT_IN_TABLE;

        nak_limit = (0x0001UL << (((*ppep)->bmNakPower > UHS_USB_NAK_MAX_POWER) ? UHS_USB_NAK_MAX_POWER : (*ppep)->bmNakPower));
        nak_limit--;
        /*
          USBTRACE2("\r\nAddress: ", addr);
          USBTRACE2(" EP: ", ep);
          USBTRACE2(" NAK Power: ",(*ppep)->bmNakPower);
          USBTRACE2(" NAK Limit: ", nak_limit);
          USBTRACE("\r\n");
         */
        regWr(rPERADDR, addr); //set peripheral address//设置外围地址

        uint8_t mode = regRd(rMODE);

        //Serial.print("\r\nMode: ");//Serial.print（“\r\n模式：”）；
        //Serial.println( mode, HEX);//串行打印LN（模式，十六进制）；
        //Serial.print("\r\nLS: ");//Serial.print（“\r\nLS:”）；
        //Serial.println(p->speed, HEX);//串行打印LN（p->速度，十六进制）；

        // Set bmLOWSPEED and bmHUBPRE in case of low-speed device, reset them otherwise//如果是低速设备，则设置bmLOWSPEED和bmHUBPRE，否则复位
        regWr(rMODE, (p->speed) ? mode & ~(bmHUBPRE | bmLOWSPEED) : mode | bmLOWSPEED | hub_present);

        return 0;
}

/**
 * Receive a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytesptr pointer to maximum number of bytes of data to receive
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::InTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t *nbytesptr, uint8_t *data) {
        uint8_t rcode = 0;
        uint8_t pktsize;

        uint16_t nbytes = *nbytesptr;
        MAX_HOST_DEBUG(PSTR("Requesting %i bytes "), nbytes);
        uint8_t maxpktsize = pep->maxPktSize;

        *nbytesptr = 0;
        regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); //set toggle value//设置切换值

        // use a 'break' to exit this loop//使用“中断”退出此循环
        while(1) {
                rcode = dispatchPkt(MAX3421E_tokIN, pep->epAddr, nak_limit); //IN packet to EP-'endpoint'. Function takes care of NAKS.//在数据包中发送到EP-“端点”。函数负责NAK。
#if 0
                // This issue should be resolved now.//这个问题现在应该解决。
                if(rcode == UHS_HOST_ERROR_TOGERR) {
                        //MAX_HOST_DEBUG(PSTR("toggle wrong\r\n"));//最大主机调试（PSTR（“切换错误\r\n”）；
                        // yes, we flip it wrong here so that next time it is actually correct!//是的，我们在这里把它翻错了，这样下次它实际上是正确的！
                        pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                        regWr(rHCTL, (pep->bmRcvToggle) ? bmRCVTOG1 : bmRCVTOG0); //set toggle value//设置切换值
                        continue;
                }
#endif
                if(rcode) {
                        //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! dispatchPkt %2.2x\r\n"), rcode);//最大主机调试（PSTR（“>>>>>>问题！dispatchPkt%2.2x\r\n”），rcode）；
                        break; //should be 0, indicating ACK. Else return error code.//应为0，表示确认。否则返回错误代码。
                }
                /* check for RCVDAVIRQ and generate error if not present */
                /* the only case when absence of RCVDAVIRQ makes sense is when toggle error occurred. Need to add handling for that */
                if((regRd(rHIRQ) & bmRCVDAVIRQ) == 0) {
                        //MAX_HOST_DEBUG(PSTR(">>>>>>>> Problem! NO RCVDAVIRQ!\r\n"));//最大主机调试（PSTR（“>>>>>>>问题！无RCVDAVIRQ！\r\n”）；
                        rcode = 0xF0; //receive error//接收错误
                        break;
                }
                pktsize = regRd(rRCVBC); //number of received bytes//接收的字节数
                MAX_HOST_DEBUG(PSTR("Got %i bytes \r\n"), pktsize);

                if(pktsize > nbytes) { //certain devices send more than asked//某些设备发送的信息超过要求
                        //MAX_HOST_DEBUG(PSTR(">>>>>>>> Warning: wanted %i bytes but got %i.\r\n"), nbytes, pktsize);//最大主机调试（PSTR（“>>>>>>警告：需要%i字节，但得到%i。\r\n”）、n字节、pktsize）；
                        pktsize = nbytes;
                }

                int16_t mem_left = (int16_t)nbytes - *((int16_t*)nbytesptr);

                if(mem_left < 0)
                        mem_left = 0;

                data = bytesRd(rRCVFIFO, ((pktsize > mem_left) ? mem_left : pktsize), data);

                regWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer//清除IRQ并释放缓冲区
                *nbytesptr += pktsize; // add this packet's byte count to total transfer length//将此数据包的字节计数添加到总传输长度

                /* The transfer is complete under two conditions:           */
                /* 1. The device sent a short packet (L.T. maxPacketSize)   */
                /* 2. 'nbytes' have been transferred.                       */
                if((pktsize < maxpktsize) || (*nbytesptr >= nbytes)) // have we transferred 'nbytes' bytes?//我们是否传输了“N字节”字节？
                {
                        // Save toggle value//保存切换值
                        pep->bmRcvToggle = ((regRd(rHRSL) & bmRCVTOGRD)) ? 1 : 0;
                        //MAX_HOST_DEBUG(PSTR("\r\n"));//最大主机调试（PSTR（“\r\n”）；
                        rcode = 0;
                        break;
                } // if//如果
        } //while( 1 )//而(1)
        return ( rcode);
}

/**
 * Transmit a packet
 *
 * @param pep pointer to a valid UHS_EpInfo structure
 * @param nak_limit how many NAKs before aborting
 * @param nbytes number of bytes of data to send
 * @param data pointer to data buffer
 * @return 0 on success
 */
uint8_t UHS_NI MAX3421E_HOST::OutTransfer(UHS_EpInfo *pep, uint16_t nak_limit, uint16_t nbytes, uint8_t *data) {
        uint8_t rcode = UHS_HOST_ERROR_NONE;
        uint8_t retry_count;
        uint8_t *data_p = data; //local copy of the data pointer//数据指针的本地副本
        uint16_t bytes_tosend;
        uint16_t nak_count;
        uint16_t bytes_left = nbytes;

        uint8_t maxpktsize = pep->maxPktSize;

        if(maxpktsize < 1 || maxpktsize > 64)
                return UHS_HOST_ERROR_BAD_MAX_PACKET_SIZE;

        unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;

        regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value//设置切换值

        while(bytes_left) {
                SYSTEM_OR_SPECIAL_YIELD();
                retry_count = 0;
                nak_count = 0;
                bytes_tosend = (bytes_left >= maxpktsize) ? maxpktsize : bytes_left;
                bytesWr(rSNDFIFO, bytes_tosend, data_p); //filling output FIFO//填充输出FIFO
                regWr(rSNDBC, bytes_tosend); //set number of bytes//设置字节数
                regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); //dispatch packet//发送包
                while(!(regRd(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ//等待完成IRQ
                regWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ//清除IRQ
                rcode = (regRd(rHRSL) & 0x0F);

                while(rcode && ((long)(millis() - timeout) < 0L)) {
                        switch(rcode) {
                                case UHS_HOST_ERROR_NAK:
                                        nak_count++;
                                        if(nak_limit && (nak_count == nak_limit))
                                                goto breakout;
                                        break;
                                case UHS_HOST_ERROR_TIMEOUT:
                                        retry_count++;
                                        if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                                                goto breakout;
                                        break;
                                case UHS_HOST_ERROR_TOGERR:
                                        // yes, we flip it wrong here so that next time it is actually correct!//是的，我们在这里把它翻错了，这样下次它实际上是正确的！
                                        pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                                        regWr(rHCTL, (pep->bmSndToggle) ? bmSNDTOG1 : bmSNDTOG0); //set toggle value//设置切换值
                                        break;
                                default:
                                        goto breakout;
                        }//switch( rcode//开关（rcode

                        /* process NAK according to Host out NAK bug */
                        regWr(rSNDBC, 0);
                        regWr(rSNDFIFO, *data_p);
                        regWr(rSNDBC, bytes_tosend);
                        regWr(rHXFR, (MAX3421E_tokOUT | pep->epAddr)); //dispatch packet//发送包
                        while(!(regRd(rHIRQ) & bmHXFRDNIRQ)); //wait for the completion IRQ//等待完成IRQ
                        regWr(rHIRQ, bmHXFRDNIRQ); //clear IRQ//清除IRQ
                        rcode = (regRd(rHRSL) & 0x0F);
                        SYSTEM_OR_SPECIAL_YIELD();
                }//while( rcode && ....//while（rcode&&…）。。。。
                bytes_left -= bytes_tosend;
                data_p += bytes_tosend;
        }//while( bytes_left...//当你离开的时候。。。
breakout:

        pep->bmSndToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 1 : 0; //bmSNDTOG1 : bmSNDTOG0;  //update toggle//bmSNDTOG1:bmSNDTOG0；//更新切换
        return ( rcode); //should be 0 in all cases//在所有情况下都应为0
}

/**
 * Send the actual packet.
 *
 * @param token
 * @param ep Endpoint
 * @param nak_limit how many NAKs before aborting, 0 == exit after timeout
 * @return 0 on success, 0xFF indicates NAK timeout. @see
 */
/* Assumes peripheral address is set and relevant buffer is loaded/empty       */
/* If NAK, tries to re-send up to nak_limit times                                                   */
/* If nak_limit == 0, do not count NAKs, exit after timeout                                         */
/* If bus timeout, re-sends up to USB_RETRY_LIMIT times                                             */

/* return codes 0x00-0x0F are HRSLT( 0x00 being success ), 0xFF means timeout                       */
uint8_t UHS_NI MAX3421E_HOST::dispatchPkt(uint8_t token, uint8_t ep, uint16_t nak_limit) {
        unsigned long timeout = millis() + UHS_HOST_TRANSFER_MAX_MS;
        uint8_t tmpdata;
        uint8_t rcode = UHS_HOST_ERROR_NONE;
        uint8_t retry_count = 0;
        uint16_t nak_count = 0;

        for(;;) {
                regWr(rHXFR, (token | ep)); //launch the transfer//启动转移
                while((long)(millis() - timeout) < 0L) //wait for transfer completion//等待转移完成
                {
                        SYSTEM_OR_SPECIAL_YIELD();
                        tmpdata = regRd(rHIRQ);

                        if(tmpdata & bmHXFRDNIRQ) {
                                regWr(rHIRQ, bmHXFRDNIRQ); //clear the interrupt//清除中断
                                //rcode = 0x00;//rcode=0x00；
                                break;
                        }//if( tmpdata & bmHXFRDNIRQ//如果（tmpdata和bmHXFRDNIRQ

                }//while ( millis() < timeout//while（毫秒（）<超时

                rcode = (regRd(rHRSL) & 0x0F); //analyze transfer result//分析传输结果

                switch(rcode) {
                        case UHS_HOST_ERROR_NAK:
                                nak_count++;
                                if(nak_limit && (nak_count == nak_limit))
                                        return (rcode);
                                delayMicroseconds(200);
                                break;
                        case UHS_HOST_ERROR_TIMEOUT:
                                retry_count++;
                                if(retry_count == UHS_HOST_TRANSFER_RETRY_MAXIMUM)
                                        return (rcode);
                                break;
                        default:
                                return (rcode);
                }//switch( rcode//开关（rcode
        }
}

////
// NULL is error, we don't need to know the reason.//NULL是错误，我们不需要知道原因。
////

UHS_EpInfo * UHS_NI MAX3421E_HOST::ctrlReqOpen(uint8_t addr, uint64_t Request, uint8_t *dataptr) {
        uint8_t rcode;
        UHS_EpInfo *pep = NULL;
        uint16_t nak_limit = 0;
        rcode = SetAddress(addr, 0, &pep, nak_limit);

        if(!rcode) {

                bytesWr(rSUDFIFO, 8, (uint8_t*)(&Request)); //transfer to setup packet FIFO//传输到设置包FIFO

                rcode = dispatchPkt(MAX3421E_tokSETUP, 0, nak_limit); //dispatch packet//发送包
                if(!rcode) {
                        if(dataptr != NULL) {
                                if(((Request)/* bmReqType*/ & 0x80) == 0x80) {
                                        pep->bmRcvToggle = 1; //bmRCVTOG1;//bmRCVTOG1；
                                } else {
                                        pep->bmSndToggle = 1; //bmSNDTOG1;//bmSNDTOG1；
                                }
                        }
                } else {
                        pep = NULL;
                }
        }
        return pep;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqRead(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint16_t nbytes, uint8_t *dataptr) {
        *read = 0;
        uint16_t nak_limit = 0;
        MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i\r\n"), *left);
        if(*left) {
again:
                *read = nbytes;
                uint8_t rcode = InTransfer(pep, nak_limit, read, dataptr);
                if(rcode == UHS_HOST_ERROR_TOGERR) {
                        // yes, we flip it wrong here so that next time it is actually correct!//是的，我们在这里把它翻错了，这样下次它实际上是正确的！
                        pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                        goto again;
                }

                if(rcode) {
                        MAX_HOST_DEBUG(PSTR("ctrlReqRead ERROR: %2.2x, left: %i, read %i\r\n"), rcode, *left, *read);
                        return rcode;
                }
                *left -= *read;
                MAX_HOST_DEBUG(PSTR("ctrlReqRead left: %i, read %i\r\n"), *left, *read);
        }
        return 0;
}

uint8_t UHS_NI MAX3421E_HOST::ctrlReqClose(UHS_EpInfo *pep, uint8_t bmReqType, uint16_t left, uint16_t nbytes, uint8_t *dataptr) {
        uint8_t rcode = 0;

        //MAX_HOST_DEBUG(PSTR("Closing"));//最大主机调试（PSTR（“关闭”）；
        if(((bmReqType & 0x80) == 0x80) && pep && left && dataptr) {
                MAX_HOST_DEBUG(PSTR("ctrlReqRead Sinking %i\r\n"), left);
                // If reading, sink the rest of the data.//如果正在读取，则接收其余数据。
                while(left) {
                        uint16_t read = nbytes;
                        rcode = InTransfer(pep, 0, &read, dataptr);
                        if(rcode == UHS_HOST_ERROR_TOGERR) {
                                // yes, we flip it wrong here so that next time it is actually correct!//是的，我们在这里把它翻错了，这样下次它实际上是正确的！
                                pep->bmRcvToggle = (regRd(rHRSL) & bmSNDTOGRD) ? 0 : 1;
                                continue;
                        }
                        if(rcode) break;
                        left -= read;
                        if(read < nbytes) break;
                }
        }
        if(!rcode) {
                //               Serial.println("Dispatching");//Serial.println（“发送”）；
                rcode = dispatchPkt(((bmReqType & 0x80) == 0x80) ? MAX3421E_tokOUTHS : MAX3421E_tokINHS, 0, 0); //GET if direction//得到如果的方向
                //        } else {//}其他{
                //                Serial.println("Bypassed Dispatch");//Serial.println（“旁路调度”）；
        }
        return rcode;
}

/**
 * Bottom half of the ISR task
 */
void UHS_NI MAX3421E_HOST::ISRbottom() {
        uint8_t x;
        //        Serial.print("Enter ");//序列号。打印（“输入”）；
        //        Serial.print((uint32_t)this,HEX);//串行打印（（uint32_t）this，十六进制）；
        //        Serial.print(" ");//连续打印（“”）；
        //        Serial.println(usb_task_state, HEX);//Serial.println（usb_任务_状态，十六进制）；

        DDSB();
        if(condet) {
                VBUS_changed();
#if USB_HOST_SHIELD_USE_ISR
                noInterrupts();
#endif
                condet = false;
#if USB_HOST_SHIELD_USE_ISR
                interrupts();
#endif
        }
        switch(usb_task_state) {
                case UHS_USB_HOST_STATE_INITIALIZE:
                        // should never happen...//不应该发生。。。
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_INITIALIZE\r\n"));
                        busprobe();
                        VBUS_changed();
                        break;
                case UHS_USB_HOST_STATE_DEBOUNCE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE\r\n"));
                        // This seems to not be needed. The host controller has debounce built in.//这似乎是不必要的。主机控制器内置了去抖动功能。
                        sof_countdown = UHS_HOST_DEBOUNCE_DELAY_MS;
                        usb_task_state = UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE;
                        break;
                case UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_DEBOUNCE_NOT_COMPLETE\r\n"));
                        if(!sof_countdown) usb_task_state = UHS_USB_HOST_STATE_RESET_DEVICE;
                        break;
                case UHS_USB_HOST_STATE_RESET_DEVICE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_DEVICE\r\n"));
                        busevent = true;
                        usb_task_state = UHS_USB_HOST_STATE_RESET_NOT_COMPLETE;
                        regWr(rHIRQ, bmBUSEVENTIRQ); // see data sheet.//见数据表。
                        regWr(rHCTL, bmBUSRST); // issue bus reset//发布总线重置
                        break;
                case UHS_USB_HOST_STATE_RESET_NOT_COMPLETE:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_RESET_NOT_COMPLETE\r\n"));
                        if(!busevent) usb_task_state = UHS_USB_HOST_STATE_WAIT_BUS_READY;
                        break;
                case UHS_USB_HOST_STATE_WAIT_BUS_READY:
                        MAX_HOST_DEBUG(PSTR("UHS_USB_HOST_STATE_WAIT_BUS_READY\r\n"));
                        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING;
                        break; // don't fall through//不要失败

                case UHS_USB_HOST_STATE_CONFIGURING:
                        usb_task_state = UHS_USB_HOST_STATE_CHECK;
                        x = Configuring(0, 1, usb_host_speed);
                        usb_error = x;
                        if(usb_task_state == UHS_USB_HOST_STATE_CHECK) {
                                if(x) {
                                        MAX_HOST_DEBUG(PSTR("Error 0x%2.2x"), x);
                                        if(x == UHS_HOST_ERROR_JERR) {
                                                usb_task_state = UHS_USB_HOST_STATE_IDLE;
                                        } else if(x != UHS_HOST_ERROR_DEVICE_INIT_INCOMPLETE) {
                                                usb_error = x;
                                                usb_task_state = UHS_USB_HOST_STATE_ERROR;
                                        }
                                } else
                                        usb_task_state = UHS_USB_HOST_STATE_CONFIGURING_DONE;
                        }
                        break;

                case UHS_USB_HOST_STATE_CHECK:
                        // Serial.println((uint32_t)__builtin_return_address(0), HEX);//Serial.println（（uint32 t）\内置\返回\地址（0），十六进制）；
                        break;
                case UHS_USB_HOST_STATE_CONFIGURING_DONE:
                        usb_task_state = UHS_USB_HOST_STATE_RUNNING;
                        break;
                #ifdef USB_HOST_MANUAL_POLL
                case UHS_USB_HOST_STATE_RUNNING:
                case UHS_USB_HOST_STATE_ERROR:
                case UHS_USB_HOST_STATE_IDLE:
                case UHS_USB_HOST_STATE_ILLEGAL:
                        enable_frame_irq(false);
                        break;
                #else
                case UHS_USB_HOST_STATE_RUNNING:
                        Poll_Others();
                        for(x = 0; (usb_task_state == UHS_USB_HOST_STATE_RUNNING) && (x < UHS_HOST_MAX_INTERFACE_DRIVERS); x++) {
                                if(devConfig[x]) {
                                        if(devConfig[x]->bPollEnable) devConfig[x]->Poll();
                                }
                        }
                        // fall thru//跌破
                #endif
                default:
                        // Do nothing//无所事事
                        break;
        } // switch( usb_task_state )//开关（usb_任务_状态）
        DDSB();
#if USB_HOST_SHIELD_USE_ISR
        if(condet) {
                VBUS_changed();
                noInterrupts();
                condet = false;
                interrupts();
        }
#endif
#ifdef USB_HOST_SHIELD_TIMING_PIN
        // My counter/timer can't work on an inverted gate signal//我的计数器/计时器不能在反转的门信号上工作
        // so we gate using a high pulse -- AJK//所以我们用一个高脉冲——AJK进行选通
        UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, LOW);
#endif
        //usb_task_polling_disabled--;//usb_任务_轮询_已禁用--；
        EnablePoll();
        DDSB();
}


/* USB main task. Services the MAX3421e */
#if !USB_HOST_SHIELD_USE_ISR

void UHS_NI MAX3421E_HOST::ISRTask() {
}
void UHS_NI MAX3421E_HOST::Task()
#else

void UHS_NI MAX3421E_HOST::Task() {
#ifdef USB_HOST_MANUAL_POLL
        if(usb_task_state == UHS_USB_HOST_STATE_RUNNING) {
                noInterrupts();
                for(uint8_t x = 0; x < UHS_HOST_MAX_INTERFACE_DRIVERS; x++)
                        if(devConfig[x] && devConfig[x]->bPollEnable)
                                devConfig[x]->Poll();
                interrupts();
        }
#endif
}

void UHS_NI MAX3421E_HOST::ISRTask()
#endif
{
        DDSB();

#ifndef SWI_IRQ_NUM
        suspend_host();
#if USB_HOST_SHIELD_USE_ISR
        // Enable interrupts//启用中断
        interrupts();
#endif
#endif

        counted = false;
        if(!MARLIN_UHS_READ_IRQ()) {
                uint8_t HIRQALL = regRd(rHIRQ); //determine interrupt source//确定中断源
                uint8_t HIRQ = HIRQALL & IRQ_CHECK_MASK;
                uint8_t HIRQ_sendback = 0x00;

                if((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
                        MAX_HOST_DEBUG
                                (PSTR("\r\nBEFORE CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
                                (HIRQ & bmCONDETIRQ) ? "T" : "F",
                                (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
                                doingreset ? "T" : "F",
                                usb_task_state
                                );
                }
                // ALWAYS happens BEFORE or WITH CONDETIRQ//总是发生在Condentirq之前或与Condentirq一起
                if(HIRQ & bmBUSEVENTIRQ) {
                        HIRQ_sendback |= bmBUSEVENTIRQ;
                        if(!doingreset) condet = true;
                        busprobe();
                        busevent = false;
                }

                if(HIRQ & bmCONDETIRQ) {
                        HIRQ_sendback |= bmCONDETIRQ;
                        if(!doingreset) condet = true;
                        busprobe();
                }

#if 1
                if((HIRQ & bmCONDETIRQ) || (HIRQ & bmBUSEVENTIRQ)) {
                        MAX_HOST_DEBUG
                                (PSTR("\r\nAFTER CDIRQ %s BEIRQ %s resetting %s state 0x%2.2x\r\n"),
                                (HIRQ & bmCONDETIRQ) ? "T" : "F",
                                (HIRQ & bmBUSEVENTIRQ) ? "T" : "F",
                                doingreset ? "T" : "F",
                                usb_task_state
                                );
                }
#endif

                if(HIRQ & bmFRAMEIRQ) {
                        HIRQ_sendback |= bmFRAMEIRQ;
                        if(sof_countdown) {
                                sof_countdown--;
                                counted = true;
                        }
                        sofevent = false;
                }

                //MAX_HOST_DEBUG(PSTR("\r\n%s%s%s\r\n"),//最大主机调试（PSTR（“\r\n%s%s%s\r\n”），
                //        sof_countdown ? "T" : "F",//倒数计时？“T”：“F”，
                //        counted ? "T" : "F",//计数为“T”：“F”，
                //        usb_task_polling_disabled? "T" : "F");//usb_任务_轮询_禁用？“T”：“F”）；
                DDSB();
                regWr(rHIRQ, HIRQ_sendback);
#ifndef SWI_IRQ_NUM
        resume_host();
#if USB_HOST_SHIELD_USE_ISR
        // Disable interrupts//禁用中断
        noInterrupts();
#endif
#endif
                if(!sof_countdown && !counted && !usb_task_polling_disabled) {
                        DisablePoll();
                        //usb_task_polling_disabled++;//usb_任务_轮询_禁用++；
#ifdef USB_HOST_SHIELD_TIMING_PIN
                        // My counter/timer can't work on an inverted gate signal//我的计数器/计时器不能在反转的门信号上工作
                        // so we gate using a high pulse -- AJK//所以我们用一个高脉冲——AJK进行选通
                        UHS_PIN_WRITE(USB_HOST_SHIELD_TIMING_PIN, HIGH);
#endif

#ifdef SWI_IRQ_NUM
                        // MAX_HOST_DEBUG(PSTR("--------------- Doing SWI ----------------"));//最大主机调试（PSTR（“--------------正在进行SWI-----------------”）；
                        exec_SWI(this);
#else
#if USB_HOST_SHIELD_USE_ISR
                        // Enable interrupts//启用中断
                        interrupts();
#endif /* USB_HOST_SHIELD_USE_ISR */
                        ISRbottom();
#endif /* SWI_IRQ_NUM */
                }
        }
}

#if 0
DDSB();
#endif
#else
#error "Never include USB_HOST_SHIELD_INLINE.h, include UHS_host.h instead"
#endif
