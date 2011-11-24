# Makefile for Fab@Home Firmware
# William Westrick (based on version by Phelps Williams (based on version by Pascal Stang))

########### change this lines according to your project ##################
#put the name of the target mcu architecture here (arm7tdmi, etc)
	CPU = arm7tdmi

#put the name of the target file here (without extension)
	TRG	= main

#put your C sourcefiles here 
	
	# This should be defined as an environment varible
	ARMLIB = armlib
	ARCH = lpc2000

	ARMLIB_ARCH_SRC =  processor.c
	#uart.c
	ARMLIB_SRC = 
	#rprintf.c

	SRC = $(TRG).c irq.c MotionControl.c usb.c timer.c buffer.c

#put additional assembler source file here
	ASRC = $(ARMLIB)/arch/$(ARCH)/boot/boot.s

#additional libraries and object files to link
	LIB	=  usbstack.a

#additional includes to compile
	INC	= 

#assembler flags
	ASFLAGS = -Wa, -gstabs

#compiler flags
	CPFLAGS	= -I./ -I./inc -I./conf -Os -Wall -Wstrict-prototypes -I$(ARMLIB) -I$(ARMLIB)/arch/$(ARCH) -I$(ARMLIB)/arch/$(ARCH)/include -I$(ARM)/arm-elf/include -Wa,-ahlms=$(<:.c=.lst)

#linker flags
	LDFLAGS = -T$(ARMLIB)/arch/$(ARCH)/boot/lpc2148-rom-iap.ld -lm -nostartfiles -Wl,-Map=$(TRG).map,--cref,-nostdlib

########### you should not need to change the following line #############
include $(ARMLIB)/make/armproj_make
	  
###### dependecies, add any dependencies you need here ###################
$(TRG).o		: $(TRG).c makefile global.h
