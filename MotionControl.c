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

//MotionControl.c
#include "stdlib.h"

#include "types.h"
#include "MotionControl.h"
#include "lpc21xx.h"
#include "timer.h"
#include "main.h"
#include "usb.h"

//currently the command queue should hold only SEGMENT_CMD commands
//all other commands are processed immediately

static const CTRL_STRUCT AXISCTRL[AXES_CNT] = 
{ AXISX_CTRL, AXISY_CTRL, AXISZ_CTRL,
  AXISU_CTRL, AXISV_CTRL, AXISW_CTRL };
static const CTRL_STRUCT AXIS_LIM[AXES_CNT] = 
{ AXISX_LIM , AXISY_LIM, AXISZ_LIM,
  AXISU_LIM, AXISV_LIM, AXISW_LIM };
static const CTRL_STRUCT AMP_EN = XYZUVW_EN;

//Some global variables for the motion control code:
CMD_Q_STRUCT cmdQ;
DWORD c[AXES_CNT]; // temporary counters
DWORD dir; //hold the direction bits, 1=positive direction
short gFWDLimitVote[AXES_CNT]; //accumulator for voting on fwd limits
short gBKDLimitVote[AXES_CNT]; //accumulator for voting on bkd limits
SEGMENT_DATA curSeg; //hold the currently executing segment
BYTE gCurAxis = 0; //iterator over axes
BOOL gSegDone = TRUE; //track whether the current command has been completed
BOOL gRisingEdge = TRUE; //track parity of calls to step() for rising/falling edges
DWORD ioPin0Bits, ioPin1Bits; //to write output registers synchronously

//globals to hold the system state
BYTE_LONG gPos[AXES_CNT]; //global displacement of axes from their home position
LIMIT_STATE gLimitReg; //status flags for limit switches
LIMIT_STATE gHWLimits;	//which axes/directions have HW limit switches (1=>yes)
LIMIT_STATE gDirReg;	 //direction flags for the axes; 1 is fwd, 0 is bkwd
GENERAL_STATE gStateReg; //status flags for the motion subsystem
BYTE_DWORD gCurCmdIndex;   //track the index of the command being executed
BYTE_DWORD gLastCmdIndex; //track the index of the last command finished
BYTE_DWORD gElapsedTime_ms;  //track the elapsed time since the timer was reset
DWORD gTick; //count ticks of the step timer for elapsed time tracking
IMM_CMD_STRUCT gIMMCmd; //hold an immediate command- not queued!



/******************************************************************************
** Function name:		is_cmdQ_empty
**
** Descriptions:		check whether queue is empty
**				update the state register accordingly
** parameters:			None
** Returned value:		BOOL, empty == TRUE
** 
******************************************************************************/
__inline__ BOOL is_cmdQ_empty( void )
{
  return(cmdQ.length == 0);
}

/******************************************************************************
** Function name:		is_cmdQ_full
**
** Descriptions:		check whether queue is full
**				update the state register accordingly
** parameters:			None
** Returned value:		BOOL, full == TRUE
** 
******************************************************************************/
__inline__ BOOL is_cmdQ_full( void )
{
  return(cmdQ.length == cmdQ.maxLength);
}

/******************************************************************************
** Function name:		get_cmdQ_length
**
** Descriptions:		return how many items in queue
**				
** parameters:			None
** Returned value:		WORD, number of items in queue
** 
******************************************************************************/
__inline__ WORD get_cmdQ_length( void )
{
  return cmdQ.length;
}

/******************************************************************************
** Function name:		get_cmdQ_space
**
** Descriptions:		return space in queue
**				
** parameters:			None
** Returned value:		WORD spaces remaining in queue
** 
******************************************************************************/
__inline__ WORD get_cmdQ_space( void )
{
  return( cmdQ.maxLength - cmdQ.length );
}

/******************************************************************************
** Function name:		flush_cmdQ
**
** Descriptions:		set the queue to zero length
**				
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
__inline__ void flush_cmdQ( void )
{
  cmdQ.length = 0;
  cmdQ.head = 1;
  cmdQ.tail = 0;
}

/******************************************************************************
** Function name:		dispose_cmdQ
**
** Descriptions:		free the memory of the queue
**				
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
__inline__ void dispose_cmdQ( void )
{
  //since command queue is a fixed size, it is not possible to free
  /*
  if (cmdQ.cmdBuf != NULL)
  {
    free(cmdQ.cmdBuf);
  }
  */
}

/******************************************************************************
** Function name:		next_cmdQ_pos
**
** Descriptions:		get next index for enqueing/dequeueing
**				treat the memory block as circular without
**				overwriting
**				
** parameters:			int pos, the current index
** Returned value:		int, the next insertion index into the queue
** 
******************************************************************************/
__inline__ int next_cmdQ_pos( int pos )
{
  if (++pos == cmdQ.maxLength)
  {
    pos = 0;
  }
  return pos;
}

/******************************************************************************
** Function name:		en_cmdQ
**
** Descriptions:		insert a command into the queue
**				
** parameters:			CMD_STRUCT *newcmdptr - ptr to a new command 
** Returned value:		BOOL, sucess == TRUE
** 
******************************************************************************/
__inline__ BOOL en_cmdQ( CMD_STRUCT *newcmdptr )
{
  if (is_cmdQ_full())
    return FALSE;
  //else
  cmdQ.length++;
  cmdQ.tail = next_cmdQ_pos(cmdQ.tail);
  cmdQ.cmdBuf[cmdQ.tail] = *newcmdptr;
  return TRUE;
}

/******************************************************************************
** Function name:		head_cmdQ
**
** Descriptions:		return the first item in the queue
**				
** parameters:			None
** Returned value:		CMD_STRUCT *, ptr to the first cmd in queue
** 
******************************************************************************/
__inline__ CMD_STRUCT *head_cmdQ( void )
{
  if (!is_cmdQ_empty())
  {
    return &(cmdQ.cmdBuf[cmdQ.head]);
  }
  /* Return value to avoid warnings from the compiler */
  return NULL;
}

/******************************************************************************
** Function name:		de_cmdQ
**
** Descriptions:		remove first cmd from the queue
**				
** parameters:			None
** Returned value:		BOOL, success==TRUE
** 
******************************************************************************/
__inline__ BOOL de_cmdQ( void )
{
  if (!is_cmdQ_empty())
  {
    cmdQ.length--;
    cmdQ.head = next_cmdQ_pos(cmdQ.head);
    return TRUE;
  }
  return FALSE;
}

/******************************************************************************
** Function name:		head_de_cmdQ
**
** Descriptions:		return first cmd in queue and dequeue it
**				
** parameters:			None
** Returned value:		CMD_STRUCT *, ptr to the first cmd in queue
** 
******************************************************************************/
__inline__ CMD_STRUCT *head_de_cmdQ( void )
{
  CMD_STRUCT *cs = NULL;
  if (!is_cmdQ_empty())
  {
    cmdQ.length--;
    cs = &(cmdQ.cmdBuf[cmdQ.head]);
    cmdQ.head = next_cmdQ_pos(cmdQ.head);
  }
  return cs;
}

/******************************************************************************
** Function name:		init_cmd_Q
**
** Descriptions:		set up queue for commands
**				
** parameters:			None
** Returned value:		BOOL = sucess or failure
** 
******************************************************************************/
BOOL init_cmd_Q()
{
  cmdQ.maxLength = NOMINAL_CMD_Q; //since pre-allocated
  flush_cmdQ();
  // This section is the dynamic memory allocation routine:
  //   it is not necessary, and the queue has be fixed to a size
  
  /*
  //start by allocating memory for the command queue
  //this will take most of the heap; be sure to leave some for other fns
  cmdQ.maxLength = MAX_CMD_Q;
  CMD_STRUCT *qBufTmp = NULL;
  void *freeHeap = NULL;

  //make sure there is enough heap space for other functions
  //by grabbing some space
  if( NULL == (freeHeap = malloc( MIN_FREE_HEAP*sizeof(BYTE) )) )
  {
    //not enough memory for this, so fail
    return FALSE;
  }
  while( (NULL ==(qBufTmp = calloc( cmdQ.maxLength, sizeof(CMD_STRUCT) )))
	  && (cmdQ.maxLength >= MIN_CMD_Q) )
  {
    cmdQ.maxLength-= CMD_Q_MALLOC_STEP;
  }
  //if we couldn't allocate the minimum required, then free memory and fail
  if( cmdQ.maxLength < MIN_CMD_Q )
  {
    free( freeHeap );
    free( qBufTmp );
    return FALSE;
  }
  //we succeeded, so free the courtesy block of heap space
  free( freeHeap );
  //initialize the queue variables
  //queue starts out empty
  cmdQ.cmdBuf = qBufTmp;
  flush_cmdQ();
  */
  
  return TRUE;
}
  
/******************************************************************************
** Function name:		init_motion_control
**
** Descriptions:		set up queue for commands and configure GPIO
**				outputs for control of axes
** parameters:			None
** Returned value:		BOOL = sucess or failure
** 
******************************************************************************/
BOOL init_motion_control( void )
{
  //DWORD tmp;
  //See MotionControl.h for the definitions and pin assignments
  //Configure necessary pins for GPIO function - typically clear bits for GPIO
  PINSEL0 &= ~PINSEL0_VAL;
  PINSEL1 &= ~PINSEL1_VAL;
  PINSEL2 &= ~PINSEL2_VAL;

  //set bits for output function
  IO0DIR |= IODIR0_OUT_VAL;
  IO1DIR |= IODIR1_OUT_VAL;
  
  //clear bits for input function
  IO0DIR &= ~IODIR0_IN_VAL;
  IO1DIR &= ~IODIR1_IN_VAL;

  //clear all the output pins (step and direction) under our control
  IO0PIN &= ~IOPIN0_OUT_MASK;
  IO1PIN &= ~IOPIN1_OUT_MASK;

  //but disable the amplifiers
  if(AMP_EN.S_REG == __IOPIN0) //for the XYZ
    IO0PIN |= AMP_EN.STEP; //set the bit high
  else
    IO1PIN |= AMP_EN.STEP;

  if(AMP_EN.D_REG == __IOPIN0) //for the UVW
    IO0PIN |= AMP_EN.DIR; //set the bit high
  else
    IO1PIN |= AMP_EN.DIR;
  
  //initialize timer1 to interrupt at 2x the MAX_STEP_FREQ
  //since timer1 will need to control rising and falling edges of pulses
  //return false if the IRQ for timer1 fails to be installed.
  //the timer will run continuously
  if( !init_timer(MOTION_TIMER, MOTION_TIMER_MATCH) )
      return FALSE;
  //reset the timer
  reset_timer(MOTION_TIMER);  //not done automatically
  //start the timer
  enable_timer(MOTION_TIMER); //not done automatically
  //initialize the position counters
  //and the limit switch accumulators
  int i;
  for(i = 0; i < AXES_CNT; i++)
  {
    gPos[i].L = 0;
    gFWDLimitVote[i] = 0;
    gBKDLimitVote[i] = 0;
  }
  //initialize the limit state flags
  //for now just clear all
  gLimitReg.data.W = 0;
  //initialize the general state flags
  gStateReg.flags.ABORTED = FALSE;
  gStateReg.flags.PAUSED = TRUE;  //start with the queue paused
  gStateReg.flags.STARTED = FALSE;
  gStateReg.flags.CONFIGURED = FALSE;
  gStateReg.flags.HOMED = FALSE;
  gStateReg.flags.MOVING = FALSE;
  gStateReg.flags.MALF = FALSE;
  //initialize the command counters
  gCurCmdIndex.DW = 0; 
  gLastCmdIndex.DW = 0;
  //initialize the elapsed timer
  gElapsedTime_ms.DW = 0; 
  gTick = 0;
  //initialize var to track whether ISR call is odd or even
  //even calls start clock pulses, odd calls finish them
  gRisingEdge = TRUE;
  gSegDone = TRUE; //haven't started yet
  gIMMCmd.valid = FALSE; //no valid immediate commands yet
  gCurAxis = 0;  //haven't started motion yet
  //allocate and setup the command queue
  //init_cmd_Q should be last called in this function so that
  //memory is deallocated (or not allocated) if something else fails.
  //waiting is not really necessary any more, since the command queue is preallocated
  if( !init_cmd_Q() )
    return FALSE;

  return TRUE;
}


/******************************************************************************
** Function name:		step
**
** Descriptions:		set the output pins for step and direction
**				control of the stepper axes based on the 
**				current command in the buffer
**				step runs inside of a high-frequency ISR
**				for timer1, so minimize context switching by
**				inlining functions
**				output pin bits are accumulated into temp vars
**				then written all at once for synchronous update
**				when gRisingEdge, perform rising edges of steps
**				when !gRisingEdge, perform falling edges
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void step( void )
{
  //gTick the elapsed time clock as necessary
  //and update the limit switch states
  //TODO: Adjust this so that it occurs every LIM_SMPL_PER
  //        currently, it is slightly off, due to the length of this interrupt 
  // 		Originally, multiplier was set to 1000, which had about 10% error
  //		  1120 decreases the error to ~<1%
  if(1000*gTick >= LIM_SMPL_PER*MOTION_TIMER_FREQ) //every LIMIT_SAMPLE_PER ms
  {
    //reset gTick to prevent overflow
    gTick=0;
    //increment the millisecond counter
    gElapsedTime_ms.DW+=LIM_SMPL_PER;
    //if any axes use HW limit switches
    //update the limit switch states (active low)
    if(gHWLimits.data.W > 0)
    { 
      //first clear the limit states
      gLimitReg.data.W = 0;
      BYTE i;
      for(i = 0; i<AXES_CNT; i++)
      {
	switch(AXIS_LIM[i].S_REG) //for the forward direction limits
	{
	  case __IOPIN0:
	    if(IO0PIN & AXIS_LIM[i].STEP)
	    {
	      gFWDLimitVote[i]+= LIM_SAMPLES-LIM_THRSHLD; //increment if active
              if(gFWDLimitVote[i]>LIM_SAMPLES)
		gFWDLimitVote[i] = LIM_SAMPLES;//truncate to mimic windowed vote
	    }
	    else
	    {
	      gFWDLimitVote[i] -= LIM_THRSHLD; //decrement if inactive
	      if(gFWDLimitVote[i]<0)
		gFWDLimitVote[i] = 0; //truncate to mimic windowed vote
	    }
	    break;
	  case __IOPIN1:
	    if(IO1PIN & AXIS_LIM[i].STEP)
	    {
	      gFWDLimitVote[i]+= LIM_SAMPLES-LIM_THRSHLD; //increment if active
              if(gFWDLimitVote[i]>LIM_SAMPLES)
		gFWDLimitVote[i] = LIM_SAMPLES;//truncate to mimic windowed vote
	    }
	    else
	    {
	      gFWDLimitVote[i] -= LIM_THRSHLD; //decrement if inactive
	      if(gFWDLimitVote[i]<0)
		gFWDLimitVote[i] = 0; //truncate to mimic windowed vote
	    }
	    break;
	  default: //make sure unimplemented bits are set high (limits are active low)
	    gFWDLimitVote[i] = LIM_SAMPLES;
	}
        switch(AXIS_LIM[i].D_REG) //for the backward direction limits
	{
	  case __IOPIN0:
	    if(IO0PIN & AXIS_LIM[i].DIR)
	    {
	      gBKDLimitVote[i]+= LIM_SAMPLES-LIM_THRSHLD; //increment if active
              if(gBKDLimitVote[i]>LIM_SAMPLES)
		gBKDLimitVote[i] = LIM_SAMPLES;//truncate to mimic windowed vote
	    }
	    else
	    {
	      gBKDLimitVote[i] -= LIM_THRSHLD; //decrement if inactive
	      if(gBKDLimitVote[i]<0)
		gBKDLimitVote[i] = 0; //truncate to mimic windowed vote
	    }
	    break;
	  case __IOPIN1:
	    if(IO1PIN & AXIS_LIM[i].DIR)
	    {
	      gBKDLimitVote[i]+= LIM_SAMPLES-LIM_THRSHLD; //increment if active
              if(gBKDLimitVote[i]>LIM_SAMPLES)
		gBKDLimitVote[i] = LIM_SAMPLES;//truncate to mimic windowed vote
	    }
	    else
	    {
	      gBKDLimitVote[i] -= LIM_THRSHLD; //decrement if inactive
	      if(gBKDLimitVote[i]<0)
		gBKDLimitVote[i] = 0; //truncate to mimic windowed vote
	    }
	    break;
	  default: //make sure unimplemented bits are set high (limits are active low)
	    gBKDLimitVote[i] = LIM_SAMPLES;
	}
        //if fwd accumulator has pegged, indicate forward switch hit 
	if(gFWDLimitVote[i] == LIM_SAMPLES) gLimitReg.data.W |= 1<<(2*i);
        //if bkd accumulator has pegged, indicate backward switch hit
	if(gBKDLimitVote[i] == LIM_SAMPLES) gLimitReg.data.W |= 1<<((2*i)+1);
      } //end for
      //now mask with configuration data on which limits to obey
      gLimitReg.data.W |= ~gHWLimits.data.W;
    }
    else //no limits used
    {
      gLimitReg.data.W = 0xFFFF;	//all high implies none active
    }
    
    //set the indicator led on occasion
    
    if(gStateReg.flags.ABORTED) status_led(ABORT_SIG); else
    if(gStateReg.flags.MALF) status_led(MALF_SIG); else
    if(gStateReg.flags.PAUSED) status_led(PAUSED_SIG); else
    status_led(HEALTHY);
  }
  //return immediately if the state is invalid any motion:
  //aborted, !started, !configured 
  if( gStateReg.flags.ABORTED || !gStateReg.flags.STARTED 
      || !gStateReg.flags.CONFIGURED ) return;

  //init the bit accumulators with current state of resp registers
  ioPin0Bits = IO0PIN;
  ioPin1Bits = IO1PIN;
  
  //initialize only if the prior segment is complete
  //indicating that the next segment is beginning
  if( gSegDone )
  {
    //if there is a valid immediate segment, 
    //and the queue is paused or empty
    //then we can execute the immediate segment
    if( gIMMCmd.valid && (gStateReg.flags.PAUSED || is_cmdQ_empty()) )
    {
      //invalidate the immediate segment to indicate execution
      gIMMCmd.valid =FALSE;
      //load the immediate segment data for execution
      curSeg = gIMMCmd.cmd.data.segment;
      //record the index of the command
      gCurCmdIndex.DW = gIMMCmd.cmd.cmdIndex.DW;
    }
    //if the queue is not paused and not empty, and the system is homed
    else if ( !gStateReg.flags.PAUSED && !is_cmdQ_empty()
	      && gStateReg.flags.HOMED )
    {
      //time to grab a new command from the head of the queue
      if( (head_cmdQ() == NULL) || (head_cmdQ()->cmdCode != SEGMENT) )
      {
	//signal an error state
	gStateReg.flags.MALF = TRUE;
	return;
      } 
      //record the index of the command
      gCurCmdIndex.DW = head_cmdQ()->cmdIndex.DW;
      //write the segment data into a global to clear queue space
      curSeg = head_de_cmdQ()->data.segment;
    }
    else //state not valid for motion
    {
      gStateReg.flags.MOVING = FALSE; //signal not moving
      return; 
    }
      
    //return immediately if s or L is zero; clear moving flag
    //these are unsigned so comparisons are not reliable
    if( (curSeg.s.DW == 0) || (curSeg.L.DW == 0) )
    {
      gSegDone = TRUE;
      //record that the current command has been executed
      gLastCmdIndex.DW = gCurCmdIndex.DW;
      //if this was an immediate command, or the queue is paused
      //then set the MOVING status flag to FALSE here (HW should be stopping)
      if(is_cmdQ_empty() || gStateReg.flags.PAUSED)
	gStateReg.flags.MOVING = FALSE;
      return;
    }
  
    //PROCEED TO EXECUTE SOME MOTION
    //init some flags
    //we are now moving
    gStateReg.flags.MOVING = TRUE;
    gDirReg.data.W = 0; //don't know directions yet
    gCurAxis = 0; //start with first axis
    gRisingEdge = TRUE; //start with rising edges
    BYTE i;
    for (i=0; i<AXES_CNT; i++)
    {
      //reset the individual step counters
      c[i]=0;  
      //update the direction bit for each axis from the sign of the "d" value
      if( curSeg.d[i].L > 0 )
      {
	//store the current direction bits
	//make sure that only one of the dir flags is set for each axis
	//e.g. only fwd or bkd
	gDirReg.data.W |= (1 << (2*i)); //set the fwd flag
	gDirReg.data.W &= ~(1 << ((2*i)+1));//clear the bkwd flag
	
	//accumulate appropriate bits, set where direction is positive
	//clear when direction is negative
	switch( AXISCTRL[i].D_REG )
	{
	  case __IOPIN0:
	    ioPin0Bits |= AXISCTRL[i].DIR;
	    break;
	  case __IOPIN1:
	    ioPin1Bits |= AXISCTRL[i].DIR;
	    break;
	  default:
	    //report an error
	    gStateReg.flags.MALF = TRUE;
	    break;
	} //switch
      } //if
      else //d[i]<=0
      {
	//store the current direction bits
	//make sure that only one of the dir flags is set for each axis
	//e.g. only fwd or bkd
	gDirReg.data.W &= ~(1 << (2*i)); //clear the fwd flag
	gDirReg.data.W |= (1 << ((2*i)+1));//set the bkwd flag
	
	//accumulate appropriate bits, clear when direction is negative
	switch( AXISCTRL[i].D_REG )
	{
	  case __IOPIN0:
	    ioPin0Bits &= ~AXISCTRL[i].DIR;
	    break;
	  case __IOPIN1:
	    ioPin1Bits &= ~AXISCTRL[i].DIR;
	    break;
	  default:
	    //report an error
	    gStateReg.flags.MALF = TRUE;
	    break;
	} //switch
	 //now that we have the direction from the sign of d[i]
	 //take its absolute value
	 curSeg.d[i].L = -curSeg.d[i].L;
      } //else d[i]<0
    } //for all axes
    //change state to "running the next command"
    gSegDone = FALSE;
  }//if segDone == TRUE
  
  if( c[gCurAxis] < curSeg.s.DW )
  {
    c[gCurAxis] += curSeg.d[gCurAxis].L;
    gCurAxis++;  //increment the current axis
      
    if( gCurAxis >= AXES_CNT )
    {
      gCurAxis = 0; //keep gCurAxis in bounds
      if( curSeg.L.DW > 0 )
      {
	(curSeg.L.DW)--;  //decrement L since we have gone through all axes
      }
      else  //curSeg.L == 0, so segment is done
      {
	gSegDone = TRUE;
        //record that the current command has been executed
	gLastCmdIndex.DW = gCurCmdIndex.DW;
	//if this was an immediate command, or the queue is paused
	//then set the MOVING status flag to FALSE here (HW should be stopping)
	if(is_cmdQ_empty() || gStateReg.flags.PAUSED)
	  gStateReg.flags.MOVING = FALSE;
      }
    }
  }
  else //c[gCurAxis] >= curSeg.s.DW )
  {
    if( gRisingEdge ) //if rising edge time, set bits
    {
      //if there are any limits active
      //inhibit motion in that direction
      //if either of the direction flags is set and 
      //the associated limit flag is cleared (active low)
      //for the current axis, then prevent motion of that axis in that direction
      if(! ((gDirReg.data.W & ~gLimitReg.data.W) & (0x03 << (2*gCurAxis))) )
      {
	//set the step bit for the current axis
	//in the appropriate bit accumulator
	switch( AXISCTRL[gCurAxis].S_REG )
	{
	  case __IOPIN0:
	    ioPin0Bits |= AXISCTRL[gCurAxis].STEP; 
	    break;
	  case __IOPIN1:
	    ioPin1Bits |= AXISCTRL[gCurAxis].STEP;
	    break;
	  default:
	    //report an error
	    gStateReg.flags.MALF = TRUE;
	    break;
	}
	//update the global position counters, RISING EDGES (EVEN CALL) ONLY
	//increment if fwd direction step
	if( gDirReg.data.W & (1 << (2*gCurAxis)) )
	    gPos[gCurAxis].L++;
	//decrement if bkd direction step
	else if ( gDirReg.data.W & (1 << ((2*gCurAxis)+1)) )
	    gPos[gCurAxis].L--;
	else //report an error
	  gStateReg.flags.MALF = TRUE;
      }
      //decrement the axis counter, even if against a limit
      c[gCurAxis] -= curSeg.s.DW;
      //the next call will be falling edge
      gRisingEdge = FALSE;
    } 
    else //gRisingEdge == FALSE
    {
      //if an odd call, regardless of whether the command is complete
      //clear the step bit for the current axis to complete the step pulse
      //if there are any limits active inhibit motion in that direction
      //if either of the direction flags is set and 
      //the associated limit flag is cleared (active low)
      //for the current axis, then prevent motion of that axis in that direction
      if(! ((gDirReg.data.W & ~gLimitReg.data.W) & (0x03 << (2*gCurAxis))) )
      {
	switch( AXISCTRL[gCurAxis].S_REG )
	{
	  //clear the appropriate bit in the bit accumulator
	  case __IOPIN0:
	    ioPin0Bits &= ~(AXISCTRL[gCurAxis].STEP);
	    break;
	  case __IOPIN1:
	    ioPin1Bits &= ~(AXISCTRL[gCurAxis].STEP);
	    break;
	  default:
	    //report an error
	    gStateReg.flags.MALF = TRUE;
	    break;
	} //switch
      } //if
      //next call will be rising edge
      gRisingEdge = TRUE;
    } //else gRisingEdge == FALSE;
  } //c[gCurAxis] >= gCurSeg.s.DW
      
  //now change the pin states synchronously
  //by writing to the pin registers
  IO0PIN = ioPin0Bits;
  IO1PIN = ioPin1Bits;
}

/******************************************************************************
** Function name:		startstepping
**
** Descriptions:		set flags to start stepping and
**				drawing segment data from the queue
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL startstepping( CMD_STRUCT *cmdPtr )
{
  //if conditions are right, toggle start; otherwise stop
  //if not aborted or error and if configured
  //starting also means setting the motor amplifier enable bits
  if( gStateReg.flags.CONFIGURED && !gStateReg.flags.ABORTED 
      && !gStateReg.flags.MALF && !gStateReg.flags.STARTED )
  {
    //update the state register
    gStateReg.flags.STARTED = TRUE;
    //enable the amplifiers
    //Xylotex amps have active low enables
    if(AMP_EN.S_REG == __IOPIN0) //for the XYZ
      IO0PIN &= ~AMP_EN.STEP; //clear the bit
    else
      IO1PIN &= ~AMP_EN.STEP;
  
    if(AMP_EN.D_REG == __IOPIN0) //for the UVW
      IO0PIN &= ~AMP_EN.DIR; //clear the bit
    else
      IO1PIN &= ~AMP_EN.DIR;

    //init some flags for the motion code
    gSegDone = TRUE; //haven't started yet
    gCurAxis = 0;   //ditto
    gRisingEdge = TRUE; //ditto
    
    return TRUE;
  }

  //can always stop
  gStateReg.flags.STARTED = FALSE;
  //disable the amps
  if(AMP_EN.S_REG == __IOPIN0) //for the XYZ
    IO0PIN |= AMP_EN.STEP; //set the bit high
  else
    IO1PIN |= AMP_EN.STEP;

  if(AMP_EN.D_REG == __IOPIN0) //for the UVW
    IO0PIN |= AMP_EN.DIR; //set the bit high
  else
    IO1PIN |= AMP_EN.DIR;

  return FALSE;
}

/******************************************************************************
** Function name:		abort
**
** Descriptions:		set flags to stop all motion
**				flush the command queue
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL abort_motion( CMD_STRUCT *cmdPtr )
{
  //can abort from any state;
  //abort toggles, and clears errors
  if( gStateReg.flags.ABORTED )
  {
    gStateReg.flags.ABORTED = FALSE; //toggle
  }
  else
  {
    //flush the queue
    flush_cmdQ();
    //invalidate any immediate command
    gIMMCmd.valid = FALSE;
    //and update the state register
    //so we only need to set aborted, clear started, clear paused
    //and clear malf
    gStateReg.flags.ABORTED = TRUE;
    gStateReg.flags.STARTED = FALSE;
    gStateReg.flags.PAUSED = TRUE;  //pause the queue
    gStateReg.flags.MALF = FALSE;
  }
  return TRUE;
}

/******************************************************************************
** Function name:		pause
**
** Descriptions:		set flags to stop pulling data from the queue;
**				immediate segments can still execute
**				The pause data should
**				probably include a suckback for the tool and
**				possibly deceleration for the robot
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL pause( CMD_STRUCT *cmdPtr )
{
  //if not aborted or paused or error, and if configured
  if( gStateReg.flags.CONFIGURED && !gStateReg.flags.ABORTED 
      && !gStateReg.flags.PAUSED && !gStateReg.flags.MALF )
  {
    //update the state register
    gStateReg.flags.PAUSED = TRUE;
    return TRUE;
  }
  return FALSE;
}

/******************************************************************************
** Function name:		resume
**
** Descriptions:		set flags
**				restart pulling data from the queue and
**				motion.  The resume packet should
**				probably include a pushout for the tool and
**				possibly some reacceleration for the robot
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL resume( CMD_STRUCT *cmdPtr )
{
  //if not aborted or error or q_empty, and if configured and homed
    //and started and paused
  if( gStateReg.flags.CONFIGURED && gStateReg.flags.HOMED
      && gStateReg.flags.STARTED && !gStateReg.flags.ABORTED 
      && gStateReg.flags.PAUSED  && !gStateReg.flags.MALF 
      && !is_cmdQ_empty())
  {
    //update the state register
    gStateReg.flags.PAUSED = FALSE;
    return TRUE;
  }
  return FALSE;
}
  
/******************************************************************************
** Function name:		home
**
** Descriptions:		set flags, clear the position counters
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL home( CMD_STRUCT *cmdPtr )
{
  //if not aborted or error
  if( !gStateReg.flags.ABORTED && !gStateReg.flags.MALF )
  {
    //clear the position counters
    BYTE i;
    for(i = 0; i < AXES_CNT; i++)
    {
      gPos[i].L = 0;
    }
    //update the state register
    gStateReg.flags.HOMED = TRUE;
    return TRUE;
  }
  return FALSE;
}
  
/******************************************************************************
** Function name:		reset
**
** Descriptions:		reinitialize the motion control subsystem
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL reset( CMD_STRUCT *cmdPtr )
{
  //allow reset from any state
  //start by flushing and releasing the queue
  dispose_cmdQ();
  //then call init_motion_control() to reset flags, allocate new queue, etc.
  return init_motion_control();
}

/******************************************************************************
** Function name:		configure
**
** Descriptions:		set flag. change some settings
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL configure( CMD_STRUCT *cmdPtr )
{
  //fill in which axes use HW limits
  gHWLimits.data.W = cmdPtr->data.config.hwLimits.data.W;
  //update the state register
  gStateReg.flags.CONFIGURED = TRUE;
  return TRUE;
}

/******************************************************************************
** Function name:		send_config
**
** Descriptions:		send configuration info + version number
** parameters:			CMD_STRUCT *cmdPtr
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL send_config( CMD_STRUCT *cmdPtr )
{
  //fill in which axes use HW limits
  gHWLimits.data.W = cmdPtr->data.config.hwLimits.data.W;
  //update the state register
  gStateReg.flags.CONFIGURED = TRUE;

  gTxLength = 1; //hold open a spot for the size, inserted at 0 offset
		//but not known until end of function
  
  //increment gTxLength, and push bytes into the buffer
  gUSBTxBuf[gTxLength++] = (BYTE)CONFIG_ACK;

  //bounce back the index of the requesting packet
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B00;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B01;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B10;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B11;

  //send the firmware version number
  gUSBTxBuf[gTxLength++] = (BYTE)VERSION;

  //send the HW limit configuration
  gUSBTxBuf[gTxLength++] = gHWLimits.data.B.B0;
  gUSBTxBuf[gTxLength++] = gHWLimits.data.B.B1;

  //now put the size in as the very first byte
  gUSBTxBuf[0] = gTxLength;

  //finally, tell the USB to send the data
  SendData2Host(gUSBTxBuf,gTxLength);
  return TRUE;
}
/******************************************************************************
** Function name:		send_status
**
** Descriptions:		send the remaining queue space,
**				the global positions of the axes,
**				the limit switch states
**				the general state
**				the index of the last executed command
**				and the elapsed time since reset
** parameters:			none
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL send_status( CMD_STRUCT *cmdPtr )
{
  gTxLength = 1; //hold open a spot for the size, inserted at 0 offset
		  //but not known until end of function

  //put the command queue length into an easy to send form
  BYTE_WORD qSpace;
  qSpace.W = get_cmdQ_space();
  //update the queue empty flag
  gStateReg.flags.QEMPTY = is_cmdQ_empty();
  
  //increment gTxLength, and push bytes into the buffer
  gUSBTxBuf[gTxLength++] = (BYTE)STATUS_ACK;

  //bounce back the index of the requesting packet
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B00;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B01;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B10;
  gUSBTxBuf[gTxLength++] = cmdPtr->cmdIndex.B.B11;

  //how much queue space is remaining
  gUSBTxBuf[gTxLength++] = qSpace.B.B0;
  gUSBTxBuf[gTxLength++] = qSpace.B.B1;

  //the real-time positions of the axes
  BYTE i;
  for(i =0; i < AXES_CNT; i++)
  {
	  gUSBTxBuf[gTxLength++] = gPos[i].B.B00;
	  gUSBTxBuf[gTxLength++] = gPos[i].B.B01;
	  gUSBTxBuf[gTxLength++] = gPos[i].B.B10;
	  gUSBTxBuf[gTxLength++] = gPos[i].B.B11;
  }

  //the real-time states of the limit switches
  gUSBTxBuf[gTxLength++] = gLimitReg.data.B.B0;
  gUSBTxBuf[gTxLength++] = gLimitReg.data.B.B1;
  //the real-time state of the status register
  gUSBTxBuf[gTxLength++] = gStateReg.data.B.B0;
  gUSBTxBuf[gTxLength++] = gStateReg.data.B.B1;

  //the index of the command being executed
  gUSBTxBuf[gTxLength++] = gCurCmdIndex.B.B00;
  gUSBTxBuf[gTxLength++] = gCurCmdIndex.B.B01;
  gUSBTxBuf[gTxLength++] = gCurCmdIndex.B.B10;
  gUSBTxBuf[gTxLength++] = gCurCmdIndex.B.B11;

  //the index of the last command executed
  gUSBTxBuf[gTxLength++] = gLastCmdIndex.B.B00;
  gUSBTxBuf[gTxLength++] = gLastCmdIndex.B.B01;
  gUSBTxBuf[gTxLength++] = gLastCmdIndex.B.B10;
  gUSBTxBuf[gTxLength++] = gLastCmdIndex.B.B11;

  //the elapsed time
  gUSBTxBuf[gTxLength++] = gElapsedTime_ms.B.B00;
  gUSBTxBuf[gTxLength++] = gElapsedTime_ms.B.B01;
  gUSBTxBuf[gTxLength++] = gElapsedTime_ms.B.B10;
  gUSBTxBuf[gTxLength++] = gElapsedTime_ms.B.B11;

  //now put the size in as the very first byte
  gUSBTxBuf[0] = gTxLength;

  //finally, tell the USB to send the data
  SendData2Host(gUSBTxBuf,gTxLength);
  return TRUE;
}

/******************************************************************************
** Function name:		queue_segment
**
** Descriptions:		add the incoming segment to the queue
** parameters:			CMD_STRUCT *pCmd
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL queue_segment( CMD_STRUCT *cmdPtr )
{
  //transfer the segment command into the command queue
  //if this fails, the queue is full  
  return en_cmdQ(cmdPtr);
}

/******************************************************************************
** Function name:		immediate_segment
**
** Descriptions:		immediately execute the segment, if state allows
**				pause the queue
** parameters:			CMD_STRUCT *pCmd
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL immediate_segment( CMD_STRUCT *cmdPtr )
{
  //transfer the segment command into the immediate
  //segment buffer; immediate commands (if valid) are executed
  //ahead of queued commands and interrupt any immediate command underway
  gIMMCmd.valid = TRUE;	//flag to indicate whether cmd has already been executed
  gIMMCmd.cmd = *cmdPtr;  //copy the cmd data
  gSegDone = TRUE;  //terminate the current command
  return TRUE;
}
      
/******************************************************************************
** Function name:		parse_command
**
** Descriptions:		look at command code in command packet
**				execute necessary function
**				called by HostData2Device fn in vcomuser.c
** parameters:			BYTE *cmdBuf holds newly received cmd
**				cnt holds the amount of data received
**				BYTE *sendBuf should be filled with data to TX
** Returned value:		BOOL success == TRUE
** 
******************************************************************************/
BOOL parse_command( BYTE *cmdBuf, DWORD cnt )
{
  if( (cmdBuf == NULL) || !(cnt > 0) ) return FALSE;
  //construct a command struct from the buffer data
  CMD_STRUCT newCmd;
  DWORD index = 0;
  //first byte should be the size
  newCmd.cmdSize = cmdBuf[index++];
  //the next is the command code
  newCmd.cmdCode = cmdBuf[index++];
  //next 4 bytes should be the index
  newCmd.cmdIndex.B.B00 = cmdBuf[index++];
  newCmd.cmdIndex.B.B01 = cmdBuf[index++];
  newCmd.cmdIndex.B.B10 = cmdBuf[index++];
  newCmd.cmdIndex.B.B11 = cmdBuf[index++];
  //remaining bytes should be the data payload, handled below
  DWORD i;
  switch( newCmd.cmdCode )
  {
    case ABORT:
      return abort_motion( &newCmd );
      break;
    case PAUSE:
      return pause( &newCmd );
      break;
    case RESUME:
      return resume( &newCmd );
      break;
    case BEGIN:
      return startstepping( &newCmd );
      break;
    case CONFIG:
      //read in the data payload
      newCmd.data.config.hwLimits.data.B.B0 = cmdBuf[index++];
      newCmd.data.config.hwLimits.data.B.B1 = cmdBuf[index++];
      //configure the firmware
      configure( &newCmd );
      //send configuration confirmation and firmware version
      return send_config( &newCmd );
      break;
    case STATUS:
      return send_status( &newCmd );
      break;
    case SEGMENT:
      //read in the data payload
      for(i = 0; i < AXES_CNT; i++)
      {
	newCmd.data.segment.d[i].B.B00 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B01 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B10 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B11 = cmdBuf[index++];
      }
      newCmd.data.segment.s.B.B00 = cmdBuf[index++];
      newCmd.data.segment.s.B.B01 = cmdBuf[index++];
      newCmd.data.segment.s.B.B10 = cmdBuf[index++];
      newCmd.data.segment.s.B.B11 = cmdBuf[index++];
      newCmd.data.segment.L.B.B00 = cmdBuf[index++];
      newCmd.data.segment.L.B.B01 = cmdBuf[index++];
      newCmd.data.segment.L.B.B10 = cmdBuf[index++];
      newCmd.data.segment.L.B.B11 = cmdBuf[index++];
      return queue_segment( &newCmd );
      break;
    case HOME:
      return home( &newCmd );
      break;
    case RESET:
      return reset( &newCmd );
    case IMMSEG:
      //read in the data payload
      for(i = 0; i < AXES_CNT; i++)
      {
	newCmd.data.segment.d[i].B.B00 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B01 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B10 = cmdBuf[index++];
	newCmd.data.segment.d[i].B.B11 = cmdBuf[index++];
      }
      newCmd.data.segment.s.B.B00 = cmdBuf[index++];
      newCmd.data.segment.s.B.B01 = cmdBuf[index++];
      newCmd.data.segment.s.B.B10 = cmdBuf[index++];
      newCmd.data.segment.s.B.B11 = cmdBuf[index++];
      newCmd.data.segment.L.B.B00 = cmdBuf[index++];
      newCmd.data.segment.L.B.B01 = cmdBuf[index++];
      newCmd.data.segment.L.B.B10 = cmdBuf[index++];
      newCmd.data.segment.L.B.B11 = cmdBuf[index++];
      return immediate_segment( &newCmd );
    default:
      break;
  }
  return TRUE;
}
