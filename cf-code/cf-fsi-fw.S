#include "cf-fsi-fw.h"

	.equ	SRAM_BASE_BE,	0x320000
	.equ	SRAM_BASE_LE,	0x720000
	.equ	GPIO_BASE,	0x780000
	.equ	CVIC_BASE,	0x6c2000

	.equ	STACK_SIZE,	0x100		/* 256 bytes of stack is enough */

	.equ	CVIC_STATUS,		0x00
	.equ	CVIC_SW_IRQ_CLR,	0x1c
	.equ	CVIC_SW_IRQ,		0x2

	.equ	FSI_PRE_BREAK_CLOCKS,	50	/* Number clocks to prep for break */
	.equ	FSI_BREAK_CLOCKS,	256	/* Number of clocks to issue break */
	.equ	FSI_POST_BREAK_CLOCKS,	16000	/* Number clocks to set up cfam */

	/* Register usage
	 *
	 * A0 : sratch/temp
	 * A1 : SRAM base (BE)
	 * A2:  CVIC address.
	 * A3:  TRACEBUF
	 * A4 : Data GPIO address
	 * A5 : Clock GPIO address
	 * A6 : CMD pointer
	 * A7 : Stack pointer
	 * D7 : clock/data/trans GPIO cache
	 * D6 : data/trans GPIO cache
	 * D5 : clock bit number
	 * D4 : data value
	 * D3 : loop counter
	 * D2 : command register
	 * D1 : scratch/temp
	 * D0 : scratch/temp
	 */

	/*
	 * Define clock/data GPIO cache registers and which
	 * register to use for trans GPIO in order to deal with
	 * potential overlaps
	 */
#define DCLK d7

#if CLOCK_GPIO_VREG == DATA_GPIO_VREG
#define DDAT d7
#else
#define DDAT d6
#endif

#if TRANS_GPIO_VREG == CLOCK_GPIO_VREG
#define DTRA d7
#elif TRANS_GPIO_VREG == DATA_GPIO_VREG
#define DTRA d6
#elif CLOCK_GPIO_VREG == DATA_GPIO_VREG
#define DTRA d6
#else
#define DTRA d1	/* Temp, not a cache */
#define DTRA_UNCACHED
#endif

	/* Tracing macro */

#ifdef ENABLE_TRACE
	.macro trace op:req
	move.b	\op,%a3@+
	.endm
#else
	.macro trace op:req
	.endm
#endif

	/*
	 * Macros for clocking, sending and receiving
	 */

	/* clock_toggle: toggle the clock down and back up */
	.macro clock_toggle
	bclr.l	%d5,%DCLK	/* clock low */
	move.l	%DCLK,%a5@(0)
	bset.l	%d5,%DCLK	/* clock high */
	move.l	%DCLK,%a5@(0)
	.endm

	/* clock_out_bit reg: Clock out bit 31 of reg */
	.macro clock_out_bit reg:req
	btst.l	#31,\reg
	beq	98f
	bset.l	#DATA_GPIO_BIT,%DDAT
	trace	#TR_CLKOBIT1
	bra	99f
98:	bclr.l	#DATA_GPIO_BIT,%DDAT
	trace	#TR_CLKOBIT0
99:
	/* If data and clock GPIO share the same register, such as on
	 * Romulus, the write done by clock_toggle will set the new data
	 * value along with the low clock edge. Thus we don't need to
	 * set it here, thus saving a PCLK
	*/
#if DATA_GPIO_VREG != CLOCK_GPIO_VREG
	move.l	%DDAT,%a4@(0)
#endif
	clock_toggle
	.endm

	/* clock_zeros reg: Clock out zeros (GPIO set to 1), assume at least 1 */
	.macro clock_out_zeros reg:req
	trace	#TR_CLKZ
	trace	\reg
	bset.l	#DATA_GPIO_BIT,%DDAT
#if DATA_GPIO_VREG != CLOCK_GPIO_VREG
	move.l	%DDAT,%a4@(0)
#endif
99:	clock_toggle
	subq.l	#1,\reg
	bne	99b
	.endm

	/* clock_in_bit reg: Clocks in bit into bit 0 of reg, the rest is 0
	 * note: bit 0 of reg must already be cleared
	 */
	.macro clock_in_bit reg:req tmp:req tmp2:req
	bclr.l	%d5,%DCLK	/* clock low */
	move.l	%DCLK,%a5@(0)
	move.l	%a4@(0),\tmp		/* dummy read */
	move.l	%a4@(0),\tmp		/* actual read */
	bset.l	%d5,%DCLK		/* clock high */
	move.l	%DCLK,%a5@(0)
	moveq.l	#DATA_GPIO_BIT,\tmp2
	lsr.l	\tmp2,\tmp
	moveq.l	#1,\tmp2
	and.l	\tmp2,\tmp
	or.l	\tmp,\reg
#ifdef ENABLE_TRACE
	move.l	#TR_CLKIBIT0,\tmp2
	or.l	\tmp,\tmp2
	trace	\tmp2
#endif
	.endm

	/*
	 * Beginning of code
	*/

	.text
	.org	0

	/*
	 * m68k exception Vectors
	 */
_vecs:
	/* Boot vector */
	.long	_stack_top		/* Stack below 1M */
	.long	_start			/* Start code */

	/* Remaining 254 vectors point to corresponding stubs
	 * starting at 0x10000, 0x10 bytes each
	 */
	.rept	254
0:	.long	_bad_exceptions + (0b - _vecs)
	.endr

	/*
	 * Header info
	 */
	.org	0x400
_header_info:
	.short	SYS_SIG			/* 0x00 */
	.short	FW_VERSION		/* 0x02 */
	.byte	API_VERSION_MAJ		/* 0x04 */
	.byte	API_VERSION_MIN		/* 0x05 */
	.byte	0,0			/* 0x06 pad */
#ifdef ENABLE_TRACE
	.long	FW_OPTION_TRACE_EN	/* 0x08 */
#else
	.long	0
#endif

	/*
	 * Config area
	*/
	.org	0x400 + HDR_CMD_STAT_AREA
_cmd_stat_base:
	.long	SRAM_BASE_BE

	/*
	 * Main entry point
	 */
	.org	0x500
	.global	_start
_start:
	/* Get base addresses */
	lea	%pc@(_cmd_stat_base),%a0
	movea.l	%a0@(0),%a1
	movea.l	#GPIO_BASE,%a4
	movea.l	%a4,%a5
	add.l	#CLOCK_GPIO_VREG,%a5
	add.l	#DATA_GPIO_VREG,%a4
	movea.l	#CVIC_BASE,%a2

	/* Cache clock bit number */
	moveq.l	#CLOCK_GPIO_BIT,%d5

	/*
	 * Check arbitration register early. There's a case where an arbitration request
	 * might have occurred right as we were starting the coprocessor. In that case,
	 * wait until we get back ownership
	 */
	move.b	%a1@(ARB_REG),%d2
	beq	1f

	/* Ack it */
	move.b	#ARB_ARM_ACK,%a1@(ARB_REG)

	/* Wait until it's cleared by the host */
0:	move.b	%a1@(ARB_REG),%d1
	bne	0b

1:	/*
	 * Load GPIO values into caches and set initial values
	 *
	 * Note: We load from the "Data Read" register which
	 *       contains the value of the write latch, and not
	 *       the "Value" register which, when read, returns
	 *       the value sampled on the line. The reason is that
	 *       the value can be missing recent changes due to
	 *       being behind synchronizers.
	 */
	move.l	%a5@(CLOCK_GPIO_DREG-CLOCK_GPIO_VREG),%DCLK
	move.l	%a4@(DATA_GPIO_DREG-DATA_GPIO_VREG),%DDAT
	move.l	%a5@(TRANS_GPIO_DREG-CLOCK_GPIO_VREG),%DTRA
	bset.l	#CLOCK_GPIO_BIT,%DCLK
	bset.l	#DATA_GPIO_BIT,%DDAT
	bset.l	#TRANS_GPIO_BIT,%DTRA
	move.l	%DCLK,%a5@(0)
	move.l	%DDAT,%a4@(0)
	move.l	%DTRA,%a5@(TRANS_GPIO_VREG-CLOCK_GPIO_VREG)

	/* Configure all GPIOs as output */
	move.l	%a5@(4),%d0
	bset.l	#CLOCK_GPIO_BIT,%d0
	move.l	%d0,%a5@(4)
	move.l	%a4@(4),%d0
	bset.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)
	move.l	%a5@(TRANS_GPIO_VREG-CLOCK_GPIO_VREG+4),%d0
	bset.l	#TRANS_GPIO_BIT,%d0
	move.l	%d0,%a5@(TRANS_GPIO_VREG-CLOCK_GPIO_VREG+4)

	/* Initialize A6 to point to command area */
	lea	%a1@(CMD_DATA),%a6

	/* Clear interrupt count */
	moveq.l	#0,%d0
	move.l	%d0,%a1@(INT_CNT)

	/* Install external interrupt vector */
	lea	%pc@(_int),%a0
	move.l	%a0,(0x46*4)

	/* Mask interrupts */
	move.w	#0x2000,%sr

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Indicate that we are started */
	mov3q.l	#0xffffffff,%a1@(CF_STARTED)

	/*
	 * Main command loop
	 */
main_loop:
	lea	%a1@(TRACEBUF),%a3

	/* Wait arbitration request or command
	 *
	 * Note: This can run with interrupts enabled as a result of
	 *       the stop op, so %d0 can be clobbered at any time.
	 */
1:	move.b	%a1@(ARB_REG),%d2
	bne	arbitration_request
	move.l	%a1@(CMD_STAT_REG),%d2
	tst.b	%d2
	bne	command_request
	stop	#0x2000
	bra	1b

arbitration_request:
	/* Mask interrupts */
	move.w	#0x2007,%sr

	/* Ack request */
	move.b	#ARB_ARM_ACK,%a1@(ARB_REG)

	/* Wait until it's cleared by the host */
0:	move.b	%a1@(ARB_REG),%d1
	bne	1f

	/* Got it, re-load the GPIO caches */
	move.l	%a5@(0),%DCLK
	move.l	%a4@(0),%DDAT
#ifndef DTRA_UNCACHED
	move.l	%a5@(TRANS_GPIO_DREG-CLOCK_GPIO_VREG),%DTRA
#endif
	/* Reconfigure data as output just in case ... */
	bsr	config_gpio_out

	/* Back to main loop */
	bra	main_loop

	/* Wait, we'll get an interrupt when the host clears it */
1:	stop	#0x2000
	bra	0b

command_request:
	/* Mask interrupts */
	move.w	#0x2007,%sr

	/* Clear the command/status register */
	moveq.l	#0,%d0
	move.l	%d0,%a1@(CMD_STAT_REG)

	/* Start command ? */
	cmpi.b	#CMD_COMMAND,%d2
	beq	start_command

	/* Break command ? */
	cmpi.b	#CMD_BREAK,%d2
	beq	start_break

	/* Idle clocks command ? */
	cmpi.b	#CMD_IDLE_CLOCKS,%d2
	beq	start_idle_clocks

	/* Error */
	trace	#TR_END
	move.b	#STAT_ERR_INVAL_CMD,%a1@(CMD_STAT_REG)
	bra	main_loop

	/*
	 * Process a command
	 */
start_command:
	/* Start bit */
	moveq.l	#0,%d0
	clock_out_bit %d0
	trace	#TR_CLKOSTART

	/* Load first lword and invert it */
	move.l	%a6@(0),%d4
	not.l	%d4

	/* Shift command right to get bit count at bottom */
	lsr.l	#8,%d2

	trace	#TR_OLEN
	trace	%d2

	/* More than 32 ? If not go to tail
	 *
	 * Note: This assumes we have at least 1 bit to clock
	 */
	moveq	#32,%d3
	cmp.b	%d3,%d2
	blt	1f

	/* Clock out 32 bits */
	sub.l	%d3,%d2
0:	clock_out_bit %d4
	lsl.l	#1,%d4
	subq.l	#1,%d3
	bne	0b

	/* Get remaining bits */
	move.l	%a6@(4),%d4
	not.l	%d4

	/* Clock out what's left */
1:	moveq.l	#0,%d3
	move.b	%d2,%d3
	beq	2f
	trace	#TR_OLEN
	trace	%d3
0:	clock_out_bit %d4
	lsl.l	#1,%d4
	subq.l	#1,%d3
	bne	0b

2:	/* Done sending, ready to receive, first echo delay */
	move.b	%a1@(ECHO_DLY_REG),%d3 /* d3 is already 0 */
	clock_out_zeros %d3

	/* Set GPIO and transceivers to input */
	bsr	config_gpio_in

	/* Wait for start bit */
	move.l	#1000,%d3
	trace	#TR_CLKWSTART
0:	moveq	#0,%d4
	clock_in_bit %d4,%d0,%d1
	/* We read inverted value, so wait for a "0" */
	btst	#0,%d4
	beq	1f
	subq.l	#1,%d3
	bne	0b
	trace	#TR_END
	move.b	#STAT_ERR_MTOE,%a1@(CMD_STAT_REG)
	bra	send_delay

1:	/* Got start bit, clock in slave ID and response tag */
	trace	#TR_CLKTAG
	moveq	#4,%d3
	moveq	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d3
	bne	0b

	/* Invert data & trace*/
	not.l	%d4
	trace	%d4

	/* (not strictly needed: clean up top bits) */
	moveq	#0xf,%d0
	and.l	%d0,%d4

	/* Store into STAT_RTAG for host */
	move.b	%d4,%a1@(STAT_RTAG)

	/* Extract tag part */
	moveq	#0x3,%d0
	and.l	%d0,%d4

	/* If non-0, no data, go get CRC */
	bne	1f

	/* Do we expect data ? */
	lsr.l	#8,%d2
	beq	1f

	/* Let's get data. Assume no more than 32-bits */
	trace	#TR_CLKDATA
	trace	%d2
	move.l	%d2,%d3
	moveq.l	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d3
	bne	0b

	/* Invert data and store it */
	not.l	%d4
	move.l	%d4,%a1@(RSP_DATA)

1:	/* Grab CRC */
	trace	#TR_CLKCRC
	moveq.l	#4,%d3
	moveq.l	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d3
	bne	0b

	/* Invert it, extract 4 bits, and store it */
	not.l	%d4
	trace	%d4
	moveq.l	#0xf,%d0
	and.l	%d0,%d4
	move.b	%d4,%a1@(STAT_RCRC)

	/* End trace */
	trace	#TR_END

	/* Mark command complete */
	moveq.l	#STAT_COMPLETE,%d0
	byterev	%d0
	move.l	%d0,%a1@(CMD_STAT_REG)

send_delay:
	/* Send delay after every command */
	moveq.l	#0,%d3
	move.b	%a1@(SEND_DLY_REG),%d3
	clock_out_zeros %d3

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Next command */
	bra	main_loop

start_break:
	/* Clock some 1's to pace and flush out whatever's going on */
	move.l	#FSI_PRE_BREAK_CLOCKS,%d3
	clock_out_zeros %d3

	/* Clock out the break */
	moveq.l	#0,%d0
	clock_out_bit %d0
	move.l	#(FSI_BREAK_CLOCKS-1),%d3
0:	clock_toggle
	subq.l	#1,%d3
	bne	0b

	/* Clock some more 1's to resync (includes the send delay) */
	move.l	#FSI_POST_BREAK_CLOCKS,%d3
	clock_out_zeros %d3

	/* End trace */
	trace	#TR_END

	/* Mark command complete */
	moveq.l	#STAT_COMPLETE,%d0
	byterev	%d0
	move.l	%d0,%a1@(CMD_STAT_REG)

	bra	main_loop

start_idle_clocks:
	/* Shift command right to get bit count at bottom */
	lsr.l	#8,%d2

	/* Clock them out */
	moveq	#0,%d3
	move.b	%d2,%d3
	clock_out_zeros %d3

	/* End trace */
	trace	#TR_END

	/* Mark command complete */
	moveq.l	#STAT_COMPLETE,%d0
	byterev	%d0
	move.l	%d0,%a1@(CMD_STAT_REG)

	bra	main_loop

config_gpio_out:
	/* Configure data GPIO as output, value 1 (idle) */
	bset.l	#DATA_GPIO_BIT,%DDAT
	move.l	%DDAT,%a4@(0)
	move.l	%a4@(4),%d0
	bset.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)

	/* Set transceivers to output */
#ifdef DTRA_UNCACHED
	move.l	%a5@(TRANS_GPIO_DREG-CLOCK_GPIO_VREG),%DTRA
#endif
	bset.l	#TRANS_GPIO_BIT,%DTRA
	move.l	%DTRA,%a5@(TRANS_GPIO_VREG-CLOCK_GPIO_VREG)
	rts

config_gpio_in:
	/* Set transceiver to input */
#ifdef DTRA_UNCACHED
	move.l	%a5@(TRANS_GPIO_DREG-CLOCK_GPIO_VREG),%DTRA
#endif
	bclr.l	#TRANS_GPIO_BIT,%DTRA
	move.l	%DTRA,%a5@(TRANS_GPIO_VREG-CLOCK_GPIO_VREG)

	/* Configure data GPIO as input */
	move.l	%a4@(4),%d0
	bclr.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)
	rts

	/* Interrupt handler
	 *
	 * Note: Must only clobber %d0
	 */
_int:
	addq.l	#1,%a1@(INT_CNT)
	moveq.l	#CVIC_SW_IRQ,%d0
	move.l	%d0,%a2@(CVIC_SW_IRQ_CLR)
	rte

	/* Bad exception handler */
bad_exception:
	move.l	%a7@+,%d0
	move.l	_bad_exceptions + 4,%d1
	sub.l	%d1,%d0
	lsr.l	#2,%d0
	move.b	%d0,%a1@(BAD_INT_VEC)
	move.b	#STAT_ERR_INVAL_IRQ,%a1@(CMD_STAT_REG)
	halt

	/* Bad exception stubs */
_bad_exceptions:
	.rept	256
	bsr.w	bad_exception
	.endr

	/* Stack */
	.balign	0x10
	.space	STACK_SIZE
_stack_top:
