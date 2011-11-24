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

//config.h
//Configuration parameters for LPC 2148

/*****************************************************************************
 *                           Preprocessor Definitions
 *                           ------------------------
 *
 * VECTORED_IRQ_INTERRUPTS
 *
 *   Enable vectored IRQ interrupts. If defined, the PC register will be loaded
 *   with the contents of the VICVectAddr register on an IRQ exception.
 *
 * USE_PLL
 *
 *   If defined, connect PLL as processor clock source. If undefined, the 
 *   oscillator clock will be used.
 *
 * PLLCFG_VAL
 *
 *   Override the default PLL configuration (multiplier = 5, divider = 2)
 *   by defining PLLCFG_VAL.
 *
 * USE_MAM
 *
 *   If defined then the memory accelerator module (MAM) will be enabled.
 *
 * MAMCR_VAL & MAMTIM_VAL
 * 
 *   Override the default MAM configuration (fully enabled, 3 fetch cycles)
 *   by defining MAMCR_VAL and MAMTIM_VAL.
 *
 * VPBDIV_VAL
 *
 *   If defined then this value will be used to configure the VPB divider.
 *
 * SRAM_EXCEPTIONS
 *
 *   If defined, enable copying and re-mapping of interrupt vectors from User 
 *   FLASH to SRAM. If undefined, interrupt vectors will be mapped in User 
 *   FLASH.
 *
 *****************************************************************************/

  //Enable the vectored interrupt controller for IRQs
  #define VECTORED_IRQ_INTERRUPTS
  
  //Have the system use the phase-lock-loop (PLL) as the clock source
  #define USE_PLL
 
  //The main oscillator crystal frequency, typically 12MHz
  /*
   * This configuration is now done in global.h
   */
  //#define FOSC	12000000
  //CAUTION - THE PLL MUST BE CONFIGURED CORRECTLY FOR THE BOOTLOADER
  //TO FUNCTION - BOOTLOADER MALFUNCTION PREVENTS REPROGRAMMING
  //AND MICROCONTROLLER WILL BE UNUSABLE
  //Configure PLL0 (the main clock source) for stable operation
  //This requires that the frequency of the current-controlled oscillator (Fcco)
  //be within 156MHz and 320MHz

  /*
   * This configuration is now done in global.h
   */
  //#define PLL_MUL 5
  //#define PLLCFG_VAL 0x24  //0x24 = 0100100, P=2, M=5 for 60MHz CCLK
  //The core clock frequency
  //this depends on the crystal and PLLCFG_VAL used to set up the PLL!!
  //#define CCLK	60000000

  //Set up the Memory Accelerator Module, which speeds up access to the
  //flash memory by predictive prefetching of code
  #define USE_MAM
  #define MAMCR_VAL 0x02
  #define MAMTIM_VAL 0x03
 
  //set the ratio between the CCLK and the peripheral bus clock
  //0 : PCLK = 1/4 CCLK; 1: PCLK  = CCLK; 2: PCLK = 1/2 CCLK
  /*
   * This configuration is now done in global.h
   */
  //#define PBSD 1
  #define VPBDIV_VAL 0x01
  //the peripheral bus frequency
  //this depends on the CCLK and the VPBDIV_VAL register
  /*
   * This configuration is now done in global.h
   */
  //#define PCLK  CCLK
 
  //If defined, SRAM_EXCEPTIONS will cause the interrupt vectors to be
  //mapped into the SRAM, so that their contents can be changed at run time.
  //This allows code to change the interrupt behavior at run time.
  /*#define SRAM_EXCEPTIONS*/
