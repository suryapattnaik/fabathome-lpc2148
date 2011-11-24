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

//main.h

/* Firmware Version */
#define VERSION 4

/* LED Definitions */
#define LEDE   0x01000000  /* P1.24 */
#define DEBUGPIN   0x02000000  /* P1.25 */

typedef struct __attribute__((__packed__))
{
  unsigned int us_period; //the period of the blinking, in microseconds
  unsigned char on_blinks; //the number of blinks on in the pattern
  unsigned char off_blinks; //the number of blinks off in the pattern
} STATUS_SIGNAL_STRUCT;

#define NUM_SIGNALS 6 //the number of defined signals
#define HEALTHY_SIGNAL {500000, 1, 0} 
#define USB_INIT_ERR_SIGNAL {250000, 2, 1}
#define MOTION_INIT_ERR_SIGNAL {250000, 3, 1}
#define ABORT_SIGNAL  {125000, 4, 4}
#define MALF_SIGNAL   {125000, 5, 5}
#define PAUSED_SIGNAL {1000000, 1, 0}

typedef enum
{
  HEALTHY,
  USB_INIT_ERR,
  MOTION_INIT_ERR,
  ABORT_SIG,
  MALF_SIG,
  PAUSED_SIG
} STATUS_SIGNAL_ENUM;

extern void status_led(STATUS_SIGNAL_ENUM status);
void init_status_led(void);
