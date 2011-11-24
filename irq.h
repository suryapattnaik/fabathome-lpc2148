/******************************************************************************
 *   irq.h:  Interrupt related Header file for Philips LPC214x Family 
 *   Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *   2006.4.20	ver 1.01  Changed Keil __asm (inline assembly) to be compatible 
 *   with GCC compiler - Evan Malone, Cornell University
 *
******************************************************************************/
/*License Notification
Fab@Home operates under the BSD Open Source License

Copyright (c) 2006, Hod Lipson and Evan Malone (evan.malone@cornell.edu) All rights reserved. 

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met: 

Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer. 
Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or 
other materials provided with the distribution. 
Neither the name of the Fab@Home Project nor the names of its contributors may be 
used to endorse or promote products derived from this software without specific 
prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __IRQ_H 
#define __IRQ_H

#include "types.h"

/* if nested interrupt is used, NEST_INTERRUPT needs to be set to 1, otherwise 0 */
#define NESTED_INTERRUPT	1//1 5-4-2006 Evan Malone - testing purposes

#define I_Bit			0x80
#define F_Bit			0x40

#define SYS32Mode		0x1F
#define IRQ32Mode		0x12
#define FIQ32Mode		0x11

#define	WDT_INT			0
#define SWI_INT			1
#define ARM_CORE0_INT	2
#define	ARM_CORE1_INT	3
#define	TIMER0_INT		4
#define TIMER1_INT		5
#define UART0_INT		6
#define	UART1_INT		7
#define	PWM0_INT		8
#define I2C0_INT		9
#define SPI0_INT		10
#define	SPI1_INT		11
#define	PLL_INT			12
#define RTC_INT			13
#define EINT0_INT		14
#define EINT1_INT		15
#define EINT2_INT		16
#define EINT3_INT		17
#define	ADC0_INT		18
#define I2C1_INT		19
#define BOD_INT			20
#define ADC1_INT		21
#define USB_INT			22

#define IRQ_SLOT_EN		0x20	/* bit 5 in Vector control register */
#define VIC_SIZE		16
#define VIC_BASE_ADDR	0xFFFFF000

#define VECT_ADDR_INDEX	0x100
#define VECT_CNTL_INDEX 0x200

/* Be aware that, from compiler to compiler, nested interrupt will have to
be handled differently. More details can be found in Philips LPC2000
family app-note AN10381 */
 
/* unlike Keil Compiler, don't save and restore registers into the stack
in RVD as the compiler does that for you. See RVD ARM compiler Inline and
embedded assemblers, "Rules for using __asm and asm keywords. */
// static DWORD sysreg;		/* used as LR register */

/* Here is the original Keil irq code
 Evan Malone, Cornell University - 4.22.2006 
 
#if NESTED_INTERRUPT 
#define IENABLE __asm { MRS LR, SPSR } \
				__asm { STMFD SP!, {LR} } \
				__asm { MSR CPSR_c, #SYS32Mode } \
				__asm { STMFD SP!, {LR} }
#else
#define IENABLE ;	// do nothing
#endif

#if NESTED_INTERRUPT
#define IDISABLE	__asm { LDMFD SP!, {LR} } \ 
					__asm { MSR CPSR_c, #(IRQ32Mode|I_Bit) } \
					__asm { LDMFD SP!, {LR} } \
					__asm { MSR SPSR_cxsf, LR }
#else
#define IDISABLE ;	// do nothing 
#endif*/
	  
//Here is the code rewritten for GNU CC inline assembly
//The GCC will not substitute inside of the strings, so the #defines above
//had to be substituted by literals.  There may be a workaround for this.
//Evan Malone, Cornell University - 4.22.2006
#if NESTED_INTERRUPT 
#define IENABLE asm volatile( "MRS LR, SPSR\n\tSTMFD SP!, {LR}\n\tMSR CPSR_c, #0x1F\n\tSTMFD SP!, {LR}\n\t" );
#else
#define IENABLE ;	/* do nothing */
#endif

#if NESTED_INTERRUPT
#define IDISABLE asm volatile( "LDMFD SP!, {LR}\n\tMSR CPSR_c, #(0x12|0x80)\n\tLDMFD SP!, {LR}\n\tMSR SPSR_cxsf, LR\n\t" );
#else
#define IDISABLE ;	/* do nothing */
#endif

void init_VIC( void );
DWORD install_irq( DWORD IntNumber, void *HandlerAddr );
DWORD uninstall_irq( DWORD IntNumber );

#endif /* end __IRQ_H */

/******************************************************************************
**                            End Of File
******************************************************************************/
