/*****************************************************************************
 *   timer.c:  Timer C file for Philips LPC214x Family Microprocessors
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
 *   2006.5.4	ver 1.02 Changed InitTimer to accept an interval parameter
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

#include "lpc21xx.h"		/* LPC214X Peripheral Registers	*/
#include "types.h"
#include "irq.h"
#include "timer.h"
#include "main.h"
#include "config.h"
#include "global.h"
#include "MotionControl.h"
#include "processor.h"

extern STATUS_SIGNAL_ENUM globalStatus;
extern STATUS_SIGNAL_STRUCT statusSignal[];
extern DWORD gTick; //from MotionControl.c
unsigned char onCnt = 0;
unsigned char offCnt = 0;
/******************************************************************************
** Function name:		Timer0Handler
**
** Descriptions:		Timer/Counter 0 interrupt handler
**				flash the status LED according to statusSignal
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void Timer0Handler (void) __attribute__ ((interrupt ("IRQ"))); 
void Timer0Handler (void)
{  
    T0IR = 1;			/* clear interrupt flag */
    IENABLE;			/* handles nested interrupt */
    #ifdef __DEBUG_PRINTF
      debug_printf("Timer0 ISR\n");
    #endif  
    //if onCnt is nonzero, start alternating on and off
    if( onCnt > 0 )
    {
      if( IO1PIN & LEDE )
      {
    	IO1CLR |= LEDE;
      }
      else
      {
    	IO1SET |= LEDE;
        onCnt--; //turning off marks one complete blink
      }
    } 
    else if( offCnt > 0 )
    {
      IO1SET |= LEDE;
      offCnt--;
    }
    else
    {
      //if both counters are zero, then reload them
      //update the period from the global value and 
      //turn on the LED
      T0MR0 = statusSignal[globalStatus].us_period;
      onCnt = statusSignal[globalStatus].on_blinks;
      offCnt = 2*statusSignal[globalStatus].off_blinks; 
      IO1CLR |= LEDE;  //clearing turns on LEDE on LPC-H2148 board
    }    
    IDISABLE;
    VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/******************************************************************************
** Function name:		Timer1Handler
**
** Descriptions:		Timer/Counter 1 interrupt handler
**				handles the motion control functions associated
**				with stepper motor control by calling step();
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void Timer1Handler (void) __attribute__ ((interrupt ("IRQ"))); 
void Timer1Handler (void)
{
  T1IR = 1;			/* clear interrupt flag */
  #ifdef __DEBUG_PRINTF
  debug_printf("Timer1 ISR\n");
  #endif
  //in principle, this should be running at 2*MAX_STEP_FREQ
  //increment a tick counter
  //conversion to ms occurs in Step fn in MotionControl.c
  gTick++;
  //step the motors that need stepping - see MotionControl.c
  step();
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/******************************************************************************
** Function name:		enable_timer
**
** Descriptions:		Enable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void enable_timer( BYTE timer_num )
{
    if ( timer_num == TIMER0 )
    {
	T0TCR = 1;
    }
    else
    {
	T1TCR = 1;
    }
    return;
}

/******************************************************************************
** Function name:		disable_timer
**
** Descriptions:		Disable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void disable_timer( BYTE timer_num )
{
    if ( timer_num == TIMER0 )
    {
	T0TCR = 0;
    }
    else
    {
	T1TCR = 0;
    }
    return;
}

/******************************************************************************
** Function name:		reset_timer
**
** Descriptions:		Reset timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void reset_timer( BYTE timer_num )
{
    DWORD regVal;

    if ( timer_num == TIMER0 )
    {
	regVal = T0TCR;
	regVal |= 0x02;
	T0TCR = regVal;
    }
    else
    {
	regVal = T1TCR;
	regVal |= 0x02;
	T1TCR = regVal;
    }
    return;
}

/******************************************************************************
** Function name:		init_timer
**
** Descriptions:		Initialize timer, set timer interval, reset timer,
**				install timer interrupt handler
**
** parameters:			timer_num: 0 or 1, specify which timer/counter
**				us_interval: microseconds before IRQ occurs
** Returned value:		true or false, if the interrupt handler can't be
**				installed, return false.
** 
******************************************************************************/
BOOL init_timer (BYTE timer_num, unsigned int us_interval) 
{
  switch( timer_num )
  {
    case TIMER0:
      T0PR = (PCLK/TIMER0_MAX_FREQ)-1; //set the timer0 prescaler register
      T0MR0 = us_interval; // assumes PCLK = CCLK = 60MHz
      T0MCR = 3;	//Interrupt and Reset on match 0 register
      if ( install_irq( TIMER0_INT, (void *)Timer0Handler ) == FALSE )
      {
	  return (FALSE);
      }
      else
      {
	  return (TRUE);
      }
      break;
    case TIMER1:
      T1PR = (PCLK/TIMER1_MAX_FREQ)-1; //set the timer1 prescaler register
      T1MR0 = us_interval; // assumes PCLK = CCLK = 60MHz
      T1MCR = 3;	// Interrupt and Reset on match 0 register
      if ( install_irq( TIMER1_INT, (void *)Timer1Handler ) == FALSE )
      {
	  return (FALSE);
      }
      else
      {
	  return (TRUE);
      }
      break;
    }
    return FALSE;
}

