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

//main.c

#include "processor.h"
#include "lpc21xx.h"            /* LPC21xx definitions */

#include "usb.h"
#include "main.h"
#include "timer.h"
#include "irq.h"
#include "MotionControl.h"

const STATUS_SIGNAL_STRUCT statusSignal[NUM_SIGNALS] = 
	      {HEALTHY_SIGNAL, USB_INIT_ERR_SIGNAL, 
              MOTION_INIT_ERR_SIGNAL, MALF_SIGNAL, 
	      ABORT_SIGNAL, PAUSED_SIGNAL};
STATUS_SIGNAL_ENUM globalStatus = HEALTHY;

/* Main Program */
int main (void) {
  int j=0;  
  processorInit();
  init_VIC(); //necessary 5-17-2006 EDM
  //initialize the motion control subsystem
  if( !init_motion_control() )
    status_led(MOTION_INIT_ERR);
  init_status_led();
  for(j=0; j<1000000; j++); //short delay here 
  //initialize USB
  usbSetup();
  //if we made it this far, blink to show health
  status_led(HEALTHY);
  while (1)                                /* Loop forever */
  {
	  //Allow USB to send information if any needs to be send
	  SendNextBulkIn(BULK_IN_EP);
  }										   
}
	  
void status_led( STATUS_SIGNAL_ENUM status )
{
  //flash signal pattern on the status LED
  //statusSignal[status].us_period determines the base period
  //statusSignal[status].on_blinks determines # blinks on
  //statusSignal[status].off_blinks determines # blinks skipped
  globalStatus = status;
}
void init_status_led( void )
{
  IO1DIR |= LEDE; //set LEDE to be an output.
  IO1DIR |= DEBUGPIN;
  init_timer( TIMER0, statusSignal[globalStatus].us_period);
  reset_timer(0);
  enable_timer(0);
}
