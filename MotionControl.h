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

// MotionControl.h

#include "types.h"
#include "timer.h"

//set the (max) number of axes in the system
#define AXES_CNT 6
//define the maximum data payload of a command packet in bytes
//#define MAX_PAYLOAD 59
#define LIM_SMPL_PER 5 //period of limit sampling (ms)
#define LIM_SAMPLES 10 //number of samples for voting limit switch readings
#define LIM_THRSHLD 7 //threshhold above which to consider limit active
//set the name of the timer to use for stepping control
#define MOTION_TIMER TIMER1
#define MOTION_TIMER_MAX_FREQ TIMER1_MAX_FREQ
//Max step rate for any of the controlled axes
//assumes timer1 prescale is set so that timer1 counts at 1MHz.
#define MAX_STEP_FREQ 50000
//the motion timer should run at 2x the max step freq to allow for rising
//and falling edges, and good dynamic performance
#define MOTION_TIMER_FREQ (2 * MAX_STEP_FREQ)
#define MOTION_TIMER_MATCH (MOTION_TIMER_MAX_FREQ / MOTION_TIMER_FREQ)-1
//Name the max and min tolerable sizes for the command queue
//in units of COMMAND_STRUCT
//#define MAX_CMD_Q 1000  //this is too big, it won't fit in ram
//#define MIN_CMD_Q 100
#define NOMINAL_CMD_Q 600  //this is the actual size of the command queue, aribitrary at this point
//#define CMD_Q_MALLOC_STEP 2 //if malloc for queue fails, reduce attempted size
//#define MIN_FREE_HEAP 1024 //leave this much RAM (in bytes)
			  //in the heap for other functions

//PINSEL0 controls fn of P0.0->P0.15
//P0.3 -> Clear bits 7:6 for GPIO     Z_FWD limit switch input
//P0.4 -> Clear bits 9:8 for GPIO     X_BKD limit switch input
//P0.5 -> Clear bits 11:10 for GPIO   Y_FWD limit switch input
//P0.6 -> Clear bits 13:12 for GPIO   Y_BKD limit switch input
//P0.7 -> Clear bits 15:14 for GPIO   X_FWD limit switch input
//P0.8 -> Clear bits 17:16 for GPIO   Z_BKD limit switch input
//P0.9 -> Clear bits 19:18 for GPIO   XYZ_EN stepper amp board enable output
//P0.15 -> Clear bits 31:30 for GPIO  X_DIR output
#define PINSEL0_VAL 0xC00FFFC0  //0b11000000000011111111111111000000, use for clearing, not setting

//PINSEL1 Controls fn of P0.16->P0.31
//ONLY P0.16->P0.22, P0.25, P028->P0.30should be used
//The others are used for USB bus, and for LED indicators
//P0.16 -> Clear bits 1:0 for GPIO    X_STEP output
//P0.18 -> Clear bits 5:4 for GPIO    Y_STEP output
//P0.20 -> Clear bits 9:8 for GPIO    Y_DIR output
//P0.22 -> Clear bits 13:12 for GPIO  Z_STEP output
//P0.25 -> Clear bits 19:18 for GPIO  Z_DIR output
//P0.28 -> Clear bits 25:24 for GPIO  U_BKD limit switch input
//P0.29 -> Clear bits 27:26 for GPIO  UVW_EN stepper amp board enable output
//P0.30 -> Clear bits 29:28 for GPIO  V_BKD limit switch input
#define PINSEL1_VAL 0x3F0C3333 //0b111111000011000011001100110011, use for clearing, not setting

//note that P0.31 and P1.16-P1.31 have on-chip pullup resistors to 3.3V
//P1.16 - P1.25 are controlled in a block by PINSEL2 bit 3 (0-> GPIO)
//caution must be used not to write zeros to bits 31:4, 1:0 
//P1.24 is being used as the LEDE output pin, so it is fine as a GPIO output.
//P1.16 -> W_BKD limit switch input
//P1.17 -> V_DIR output
//P1.18 -> U_STEP output
//P1.19 -> U_DIR output
//P1.21 -> V_STEP output
//P1.22 -> W_DIR output
//P1.23 -> W_STEP output
#define PINSEL2_VAL 0x00000008 //0b1000, use for clearing, not setting

//now set the directions for the GPIO pins, write ones for outputs, clear for inputs
//iodir0 and iodir1 are bitmapped 1:1 with P0 and P1 pins
#define IODIR0_OUT_VAL	0x22558200  //0b00100010010101011000001000000000
#define	IODIR0_IN_VAL	0x500001F8  //0b01010000000000000000000111111000
#define IODIR1_OUT_VAL	0x00EE0000  //0b00000000111011100000000000000000
#define IODIR1_IN_VAL	0x00010000  //0b00000000000000010000000000000000
#define IOPIN0_OUT_MASK IODIR0_OUT_VAL
#define IOPIN0_IN_MASK	IODIR0_IN_VAL
#define IOPIN1_OUT_MASK IODIR1_OUT_VAL
#define IOPIN1_IN_MASK	IODIR1_IN_VAL

//specify the pin control registers and bit masks
//for the step and direction pins
//for the forward and the backward limit
#define AXISX_CTRL {__IOPIN0,0x00010000,__IOPIN0,0x00008000}//
#define AXISX_LIM  {__IOPIN0,0x00000080,__IOPIN0,0x00000010}//
#define AXISY_CTRL {__IOPIN0,0x00040000,__IOPIN0,0x00100000}//
#define AXISY_LIM  {__IOPIN0,0x00000020,__IOPIN0,0x00000040}//
#define AXISZ_CTRL {__IOPIN0,0x00400000,__IOPIN0,0x02000000}//
#define AXISZ_LIM  {__IOPIN0,0x00000008,__IOPIN0,0x00000100}//
#define AXISU_CTRL {__IOPIN1,0x00040000,__IOPIN1,0x00080000}//
#define AXISU_LIM  {__NOTUSE,0x00000000,__IOPIN0,0x10000000}//
#define AXISV_CTRL {__IOPIN1,0x00200000,__IOPIN1,0x00020000}//
#define AXISV_LIM  {__NOTUSE,0x00000000,__IOPIN0,0x40000000}//
#define AXISW_CTRL {__IOPIN1,0x00800000,__IOPIN1,0x00400000}//
#define AXISW_LIM  {__NOTUSE,0x00000000,__IOPIN1,0x00010000}//
#define XYZUVW_EN  {__IOPIN0,0x00000200,__IOPIN0,0x20000000}

typedef enum 
{
  AXIS_X,
  AXIS_Y,
  AXIS_Z,
  AXIS_U,
  AXIS_V,
  AXIS_W
} AXIS_ENUM;
  
typedef enum
{
  __NOTUSE,
  __IOPIN0,
  __IOPIN1
} IOPINREG_ENUM;

typedef struct
{
  IOPINREG_ENUM S_REG;
  DWORD STEP;
  IOPINREG_ENUM D_REG;
  DWORD DIR;
} CTRL_STRUCT;

typedef enum
{
  UNDEFINED   = -1, //undefined, should not receive
  ABORT	      = 0,  //abort current motion, flush the queue, toggles
  ABORT_ACK   = 1,
  PAUSE	      = 2,  //pause motion; hold the queue index
  PAUSE_ACK   = 3,
  RESUME      = 4,  //resume motion at next queue index
  RESUME_ACK  = 5,
  BEGIN	      = 6,  //start motion at front of queue
  BEGIN_ACK   = 7,
  CONFIG      = 8,  //send configuration information to the device, i.e. clear error
  CONFIG_ACK  = 9,  //send back configuration and version info to application
  STATUS      = 10, //request that status information be returned
  STATUS_ACK  = 11, //the status information that was returned
  SEGMENT     = 12, //enqueue a path segment
  SEGMENT_ACK = 13,
  HOME	      = 14, //home the axes
  HOME_ACK    = 15, //signal when homed
  RESET	      = 16, //(re)initialize the motion control system
  RESET_ACK   = 17, //signal when complete
  IMMSEG      = 18, //send segment for immediate (non-queued) execution
  IMMSEG_ACK  = 19  //acknowledgment of immediate segment
} CMD_ENUM;

//allow byte access to DWORD type
typedef struct  __attribute__((__packed__))
{
	BYTE B00;
	BYTE B01;
	BYTE B10;
	BYTE B11;
}DW_B;

typedef union __attribute__((__packed__))
{
	DW_B B;
	DWORD DW;
} BYTE_DWORD;

//allow byte access to long type
typedef struct __attribute__((__packed__))
{
	BYTE B00;
	BYTE B01;
	BYTE B10;
	BYTE B11;
}L_B;

typedef union __attribute__((__packed__))
{
	L_B B;
	long L;
}BYTE_LONG;

//allow byte access to WORD type
typedef struct __attribute__((__packed__))
{
	BYTE B0;
	BYTE B1;
}W_B;

typedef union __attribute__((__packed__))
{
	W_B B;
	WORD W;
}BYTE_WORD;

//allow byte access to short type
typedef struct __attribute__((__packed__))
{
	BYTE B0;
	BYTE B1;
}S_B;

typedef union __attribute__((__packed__))
{
	S_B B;
	short S;
}BYTE_SHORT;

typedef struct __attribute__((__packed__))
{
	WORD X_FWD : 1;
	WORD X_BKD : 1;
	WORD Y_FWD : 1;
	WORD Y_BKD : 1;
	WORD Z_FWD : 1;
	WORD Z_BKD : 1;
	WORD U_FWD : 1;
	WORD U_BKD : 1;
	WORD V_FWD : 1;
	WORD V_BKD : 1;
	WORD W_FWD : 1;
	WORD W_BKD : 1;
}LIMIT_FLAGS;

typedef union  __attribute__((__packed__))
{
	LIMIT_FLAGS flags;
	BYTE_WORD data;
}LIMIT_STATE;


typedef struct __attribute__((__packed__))
{
	WORD ABORTED : 1;
	WORD PAUSED : 1;
	WORD STARTED : 1;
	WORD CONFIGURED : 1;
	WORD HOMED : 1;
	WORD MOVING : 1;
	WORD MALF : 1;
	WORD QEMPTY : 1;
} GEN_STATE_FLAGS;

typedef union  __attribute__((__packed__))
{
	GEN_STATE_FLAGS flags;
	BYTE_WORD data;
} GENERAL_STATE;

//the command data definitions - should be <=32bytes

typedef struct __attribute__((__packed__))
{
  //TBD
} ABORT_DATA;

typedef struct __attribute__((__packed__))
{
  //TBD
} PAUSE_DATA;

typedef struct __attribute__((__packed__))
{
  //TBD
} RESUME_DATA;

typedef struct __attribute__((__packed__))
{
  //TBD
} START_DATA;

typedef struct __attribute__((__packed__))
{
  LIMIT_STATE hwLimits;
} CONFIG_DATA;

typedef struct __attribute__((__packed__))
{
  //TBD
} STATUS_DATA;

typedef struct __attribute__((__packed__))
{
  BYTE_LONG d[AXES_CNT];  //proportional to directional cosines; signed
  BYTE_DWORD s;    // inversely proportional to speed, unsigned  
  BYTE_DWORD L;    // length (~ dist/s), unsigned
} SEGMENT_DATA;

typedef struct __attribute__((__packed__))
{
  //TBD
} HOME_DATA;
	
typedef union __attribute__((__packed__))
{
  ABORT_DATA abort;  //abort current motion, flush the queue
  PAUSE_DATA pause;  //pause motion; hold the queue index
  RESUME_DATA resume; //resume motion at next queue index
  START_DATA begin;   //start motion at front of queue
  CONFIG_DATA config; //send configuration information to the device
  STATUS_DATA status; //request that status information be returned
  SEGMENT_DATA segment; //enqueue a path segment
  HOME_DATA home;    //home the axes, signal when homed, or report in status?
} PAYLOAD_UNION;

typedef struct __attribute__((__packed__))
{
  BYTE cmdSize;	      //the size of the command in bytes
  CMD_ENUM cmdCode;   //the code of the command
  BYTE_DWORD cmdIndex;     //unique index to allow execution restarting/tracking
  PAYLOAD_UNION data;
} CMD_STRUCT;

//for immediate (non-queued) motion commands
typedef struct __attribute__((__packed__))
{
  BOOL	valid;	//flag to indicate whether seg data has already been executed
  CMD_STRUCT cmd;
} IMM_CMD_STRUCT;

typedef struct
{
  WORD head;
  WORD tail;
  WORD length;
  WORD maxLength;
  CMD_STRUCT cmdBuf[NOMINAL_CMD_Q];
} CMD_Q_STRUCT;

extern BYTE gTxLength;
extern BYTE gRxLength;
extern BYTE gUSBRxBuf[];
extern BYTE gUSBTxBuf[];

BOOL init_cmd_Q( void );
BOOL is_cmdQ_empty( void );
BOOL is_cmdQ_full( void );
WORD get_cmdQ_length( void );
WORD get_cmdQ_space( void );
void flush_cmdQ( void );
void dispose_cmdQ( void );
int next_cmdQ_pos(int pos);
BOOL en_cmdQ(CMD_STRUCT *newcmdptr);
CMD_STRUCT *head_cmdQ( void );
BOOL de_cmdQ( void );
CMD_STRUCT *head_de_cmdQ( void );

BOOL init_motion_control( void );
void step( void );
BOOL startstepping( CMD_STRUCT *cmdPtr);
BOOL abort_motion( CMD_STRUCT *cmdPtr );
BOOL pause( CMD_STRUCT *cmdPtr );
BOOL resume( CMD_STRUCT *cmdPtr );
BOOL home( CMD_STRUCT *cmdPtr );
BOOL reset( CMD_STRUCT *cmdPtr );
BOOL configure( CMD_STRUCT *cmdPtr );
BOOL send_config( CMD_STRUCT *cmdPtr );
BOOL send_status( CMD_STRUCT *cmdPtr );
BOOL queue_segment( CMD_STRUCT *cmdPtr );
BOOL immediate_segment( CMD_STRUCT *cmdPtr );
BOOL parse_command( BYTE *cmdBuf, DWORD cnty );
