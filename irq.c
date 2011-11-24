/*****************************************************************************
 *   irq.c: Interrupt handler C file for Philips LPC214x Family Microprocessors
 *
 *   Copyright(C) 2006, Philips Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2005.10.01  ver 1.00    Prelimnary version, first Release
 *   2006.4.20	ver 1.01 Modified by Evan Malone, Cornell University, 
 *			 Changed Keil __irq, __fiq directives to be GNU C style
 *			 __attribute__ ((interrupt ("IRQ"))), etc.  Requires
 *			adding function prototypes to be modified by directive.
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

#include "lpc21xx.h"			/* LPC214X Peripheral Registers	*/
#include "types.h"
#include "irq.h"

/******************************************************************************
** Function name:		DefaultVICHandler
**
** Descriptions:		Default VIC interrupt handler.
**				This handler is set to deal with spurious 
**				interrupt.
**				If the IRQ service routine reads the VIC
**				address register, and no IRQ slot responses
**				as described above, this address is returned.
** parameters:			None
** Returned value:		None
** 
******************************************************************************/

void DefaultVICHandler (void)__attribute__ ((interrupt ("IRQ")));
void DefaultVICHandler (void)
{
    /* if the IRQ is not installed into the VIC, and interrupt occurs, the
    default interrupt VIC address will be used. This could happen in a race 
    condition. For debugging, use this endless loop to trace back. */
    /* For more details, see Philips appnote AN10414 */
    VICVectAddr = 0;		/* Acknowledge Interrupt */ 
    while ( 1 );
}

/* Initialize the interrupt controller */
/******************************************************************************
** Function name:		init_VIC
**
** Descriptions:		Initialize VIC interrupt controller.
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void init_VIC(void) 
{
    DWORD i = 0;
    DWORD *vect_addr, *vect_cntl;
   	
    /* initialize VIC*/
    VICIntEnClear = 0xffffffff;
    VICVectAddr = 0;
    VICIntSelect = 0;

    /* set all the vector and vector control register to 0 */
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	vect_addr = (DWORD *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (DWORD *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	*vect_addr = 0;	
	*vect_cntl = 0;
    }

    /* Install the default VIC handler here */
    VICDefVectAddr = (DWORD)DefaultVICHandler;   
    return;
}

/******************************************************************************
** Function name:		install_irq
**
** Descriptions:		Install interrupt handler
**				The max VIC size is 16, but, there are 32 interrupt
**				request inputs. Not all of them can be installed into
**				VIC table at the same time.
**				The order of the interrupt request installation is
**				first come first serve.
** parameters:			Interrupt number and interrupt handler address
** Returned value:		true or false, when the table is full, return false
** 
******************************************************************************/
DWORD install_irq( DWORD IntNumber, void *HandlerAddr )
{
    DWORD i;
    DWORD *vect_addr;
    DWORD *vect_cntl;
      
    VICIntEnClear = 1 << IntNumber;	/* Disable Interrupt */
    
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	/* find first un-assigned VIC address for the handler */

	vect_addr = (DWORD *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (DWORD *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	if ( *vect_addr == (DWORD)NULL )
	{
	    *vect_addr = (DWORD)HandlerAddr;	/* set interrupt vector */
	    *vect_cntl = (DWORD)(IRQ_SLOT_EN | IntNumber);
	    break;
	}
    }
    if ( i == VIC_SIZE )
    {
	return( FALSE );		/* fatal error, can't find empty vector slot */
    }
    VICIntEnable = 1 << IntNumber;	/* Enable Interrupt */
    return( TRUE );
}

/******************************************************************************
** Function name:		uninstall_irq
**
** Descriptions:		Uninstall interrupt handler
**				Find the interrupt handler installed in the VIC
**				based on the interrupt number, set the location
**				back to NULL to uninstall it.
** parameters:			Interrupt number
** Returned value:		true or false, when the interrupt number is not found, 
**				return false
** 
******************************************************************************/
DWORD uninstall_irq( DWORD IntNumber )
{
    DWORD i;
    DWORD *vect_addr;
    DWORD *vect_cntl;
      
    VICIntEnClear = 1 << IntNumber;	/* Disable Interrupt */
    
    for ( i = 0; i < VIC_SIZE; i++ )
    {
	/* find first un-assigned VIC address for the handler */
	vect_addr = (DWORD *)(VIC_BASE_ADDR + VECT_ADDR_INDEX + i*4);
	vect_cntl = (DWORD *)(VIC_BASE_ADDR + VECT_CNTL_INDEX + i*4);
	if ( (*vect_cntl & ~IRQ_SLOT_EN ) == IntNumber )
	{
	    *vect_addr = (DWORD)NULL;	/* clear the VIC entry in the VIC table */
	    *vect_cntl &= ~IRQ_SLOT_EN;	/* disable SLOT_EN bit */	
	    break;
	}
    }
    if ( i == VIC_SIZE )
    {
	return( FALSE );		/* fatal error, can't find interrupt number 
					in vector slot */
    }
    VICIntEnable = 1 << IntNumber;	/* Enable Interrupt */
    return( TRUE );
}

/******************************************************************************
**                            End Of File
******************************************************************************/
