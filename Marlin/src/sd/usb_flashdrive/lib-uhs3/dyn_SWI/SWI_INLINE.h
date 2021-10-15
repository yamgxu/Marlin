/** translatione by yx */
/*
 * File:   SWI_INLINE.h
 * Author: xxxajk@gmail.com
 *
 * Created on December 5, 2014, 9:40 AM
 *
 * This is the actual library.
 * There are no 'c' or 'cpp' files.
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

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifdef DYN_SWI_H
#ifndef SWI_INLINE_H
#define SWI_INLINE_H

#ifndef SWI_MAXIMUM_ALLOWED
#define SWI_MAXIMUM_ALLOWED 4
#endif



#if defined(__arm__) || defined(ARDUINO_ARCH_PIC32)
static char dyn_SWI_initied = 0;
static dyn_SWI* dyn_SWI_LIST[SWI_MAXIMUM_ALLOWED];
static dyn_SWI* dyn_SWI_EXEC[SWI_MAXIMUM_ALLOWED];
#ifdef __arm__
#ifdef __USE_CMSIS_VECTORS__
extern "C" {
        void (*_VectorsRam[VECTORTABLE_SIZE])(void)__attribute__((aligned(VECTORTABLE_ALIGNMENT)));
}
#else

__attribute__((always_inline)) static inline void __DSB() {
        __asm__ volatile ("dsb");
}
#endif // defined(__USE_CMSIS_VECTORS__)//已定义（使用向量）
#else // defined(__arm__)//已定义（手臂）
__attribute__((always_inline)) static inline void __DSB() {
        __asm__ volatile ("sync" : : : "memory");
}
#endif // defined(__arm__)//已定义（手臂）

/**
 * Execute queued class ISR routines.
 */
#ifdef ARDUINO_ARCH_PIC32
static p32_regset *ifs = ((p32_regset *) & IFS0) + (SWI_IRQ_NUM / 32); //interrupt flag register set//中断标志寄存器集
static p32_regset *iec = ((p32_regset *) & IEC0) + (SWI_IRQ_NUM / 32); //interrupt enable control reg set//中断启用控制寄存器集
static uint32_t swibit = 1 << (SWI_IRQ_NUM % 32);

void
#ifdef __PIC32MZXX__
        __attribute__((nomips16,at_vector(SWI_VECTOR),interrupt(SWI_IPL)))
#else
        __attribute__((interrupt(),nomips16))
#endif
        softISR() {
#else
#ifdef ARDUINO_spresense_ast
unsigned int softISR() {
#else
void softISR() {
#endif
#endif

        ////
        // TO-DO: Perhaps limit to 8, and inline this?//待办事项：可能限制为8，并将其内联？
        ////


        // Make a working copy, while clearing the queue.//在清除队列的同时制作工作副本。
        noInterrupts();
#ifdef ARDUINO_ARCH_PIC32
        //ifs->clr = swibit;//ifs->clr=swibit；
#endif
        for(int i = 0; i < SWI_MAXIMUM_ALLOWED; i++) {
                dyn_SWI_EXEC[i] = dyn_SWI_LIST[i];
                dyn_SWI_LIST[i] = NULL;
        }
        __DSB();
        interrupts();

        // Execute each class SWI//执行每个类SWI
        for(int i = 0; i < SWI_MAXIMUM_ALLOWED; i++) {
                if(dyn_SWI_EXEC[i]) {
#ifdef __DYN_SWI_DEBUG_LED__
                        digitalWrite(__DYN_SWI_DEBUG_LED__, HIGH);
#endif
                        dyn_SWI_EXEC[i]->dyn_SWISR();
#ifdef __DYN_SWI_DEBUG_LED__
                        digitalWrite(__DYN_SWI_DEBUG_LED__, LOW);
#endif
                }
        }
#ifdef ARDUINO_ARCH_PIC32
        noInterrupts();
        if(!dyn_SWI_EXEC[0]) ifs->clr = swibit;
        interrupts();
#endif
#ifdef ARDUINO_spresense_ast
        return 0;
#endif
}

#define DDSB() __DSB()
#endif


#ifdef __arm__
#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus() __attribute__((always_inline, unused));

static inline unsigned char __interruptsStatus() {
        unsigned int primask;
        asm volatile ("mrs %0, primask" : "=r" (primask));
        if(primask) return 0;
        return 1;
}
#endif

/**
 * Initialize the Dynamic (class) Software Interrupt
 */
static void Init_dyn_SWI() {
        if(!dyn_SWI_initied) {
#ifdef __USE_CMSIS_VECTORS__
                uint32_t *X_Vectors = (uint32_t*)SCB->VTOR;
                for(int i = 0; i < VECTORTABLE_SIZE; i++) {
                        _VectorsRam[i] = reinterpret_cast<void (*)()>(X_Vectors[i]); /* copy vector table to RAM */
                }
                /* relocate vector table */
                noInterrupts();
                SCB->VTOR = reinterpret_cast<uint32_t>(&_VectorsRam);
                DDSB();
                interrupts();
#endif
#ifndef ARDUINO_spresense_ast
                for(int i = 0; i < SWI_MAXIMUM_ALLOWED; i++) dyn_SWI_LIST[i] = NULL;
                noInterrupts();
                _VectorsRam[SWI_IRQ_NUM + 16] = reinterpret_cast<void (*)()>(softISR);
                DDSB();
                interrupts();
                NVIC_SET_PRIORITY(SWI_IRQ_NUM, 255);
                NVIC_ENABLE_IRQ(SWI_IRQ_NUM);
#endif
#ifdef __DYN_SWI_DEBUG_LED__
                pinMode(__DYN_SWI_DEBUG_LED__, OUTPUT);
                digitalWrite(__DYN_SWI_DEBUG_LED__, LOW);
#endif
                dyn_SWI_initied = 1;
        }
}

/**
 * @param klass class that extends dyn_SWI
 * @return 0 on queue full, else returns queue position (ones based)
 */
int exec_SWI(const dyn_SWI* klass) {
        int rc = 0;

        uint8_t irestore = interruptsStatus();
        // Allow use from inside a critical section...//允许从关键部分内部使用。。。
        // ... and prevent races if also used inside an ISR// ... 如果在ISR中也使用，则可防止种族冲突
        noInterrupts();
        for(int i = 0; i < SWI_MAXIMUM_ALLOWED; i++) {
                if(!dyn_SWI_LIST[i]) {
                        rc = 1 + i; // Success!//成功！
                        dyn_SWI_LIST[i] = (dyn_SWI*)klass;
#ifndef ARDUINO_spresense_ast
                        if(!NVIC_GET_PENDING(SWI_IRQ_NUM)) NVIC_SET_PENDING(SWI_IRQ_NUM);
#else
                        // Launch 1-shot timer as an emulated SWI//作为模拟SWI启动单次定时器
                        // Hopefully the value of Zero is legal.//希望零的值是合法的。
                        // 1 microsecond latency would suck!//1微秒的延迟太糟糕了！
                        attachTimerInterrupt(softISR, 100);
#endif
                        DDSB();
                        break;
                }
        }
        // Restore interrupts, if they were on.//恢复中断，如果它们处于打开状态。
        if(irestore) interrupts();
        return rc;
}
#elif defined(ARDUINO_ARCH_PIC32)

/**
 * Initialize the Dynamic (class) Software Interrupt
 */
static void Init_dyn_SWI() {
        if(!dyn_SWI_initied) {
                uint32_t sreg = disableInterrupts();

                setIntVector(SWI_VECTOR, softISR);
                setIntPriority(SWI_VECTOR, 1, 1); // Lowest priority, ever.//最低优先级，永远。
                ifs->clr = swibit;
                iec->clr = swibit;
                iec->set = swibit;
                restoreInterrupts(sreg);
#ifdef __DYN_SWI_DEBUG_LED__
                pinMode(__DYN_SWI_DEBUG_LED__, OUTPUT);
                UHS_PIN_WRITE(__DYN_SWI_DEBUG_LED__, LOW);
#endif
        }
}

/**
 * @param klass class that extends dyn_SWI
 * @return 0 on queue full, else returns queue position (ones based)
 */
int exec_SWI(const dyn_SWI* klass) {
        int rc = 0;
        uint32_t sreg = disableInterrupts();
        for(int i = 0; i < SWI_MAXIMUM_ALLOWED; i++) {
                if(!dyn_SWI_LIST[i]) {
                        rc = 1 + i; // Success!//成功！
                        dyn_SWI_LIST[i] = (dyn_SWI*)klass;
                        if(!(ifs->reg & swibit)) ifs->set = swibit;
                        ;
                        break;
                }
        }
        restoreInterrupts(sreg);
        return rc;
}

#endif /* defined(__arm__) */
#endif  /* SWI_INLINE_H */
#else
#error "Never include SWI_INLINE.h directly, include dyn_SWI.h instead"
#endif
