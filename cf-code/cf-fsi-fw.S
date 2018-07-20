// SPDX-License-Identifier: GPL-2.0+

#include "cf-fsi-fw.h"

	.equ	SRAM_BASE_BE,	0x320000
	.equ	SRAM_BASE_LE,	0x720000
	.equ	GPIO_BASE,	0x780000
	.equ	CVIC_BASE,	0x6c2000
	.equ	IFLUSH_BASE,	0x008000
	.equ	DFLUSH_BASE,	0x008004

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
	 * A2:  GPIO block base
	 * A3:  TRACEBUF
	 * A4 : Data GPIO address
	 * A5 : Clock GPIO address
	 * A6 : CMD pointer
	 * A7 : Stack pointer
	 * D7 : clock/data/trans GPIO cache
	 * D6 : data/trans GPIO cache
	 * D5 : clock bit number
	 * D4 : data value
	 * D3 : loop counter or scratch
	 * D2 : command register
	 * D1 : data bit number or scratch/temp
	 * D0 : FW control bits or scratch/temp
	 */

	/*
	 * Define clock/data GPIO cache registers and which
	 * register to use for trans GPIO in order to deal with
	 * potential overlaps
	 */
#define DCLK d7

#ifdef DCLK_DDAT_SHARED
#define DDAT d7
#define DTRA d6
#else
#define DDAT d6
#define DTRA d0
#endif
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
#define _FW_OPTION_TRACE_EN	FW_OPTION_TRACE_EN
#else
#define _FW_OPTION_TRACE_EN	0
#endif
	.long	_FW_OPTION_TRACE_EN | FW_OPTION_CONT_CLOCK	/* 0x08 FW options */
	.long	0						/* 0x0c pad */
	.long	_end - _vecs					/* 0x10 FW size */

	/*
	 * Config area
	*/
	.org	0x400 + HDR_CMD_STAT_AREA
_cmd_stat_base:
	.long	SRAM_BASE_BE
_fw_ctrl:
	.long	0

	.org	0x400 + HDR_CLOCK_GPIO_VADDR
_clk_gpio_vreg:
	.short	0x1e0
_clk_gpio_dreg:
	.short	0x0d8
_dat_gpio_vreg:
	.short	0x1e0
_dat_gpio_dreg:
	.short	0x0d8
_tra_gpio_vreg:
	.short	0x080
_tra_gpio_dreg:
	.short	0x0d0
_clk_gpio_bit:
	.byte	16
_dat_gpio_bit:
	.byte	18
_tra_gpio_bit:
	.byte	10

	/*
	 * Tracing macro
	 */

#ifdef ENABLE_TRACE
	.macro trace op:req
	move.b	\op,%a3@+
	.endm
	.macro count_clock
	addq.l	#1,%a1@(CLK_CNT)
	.endm
	.macro count_stop
	addq.l	#1,%a1@(STOP_CNT)
	.endm
	.macro count_ints
	addq.l	#1,%a1@(INT_CNT)
	.endm
#else
	.macro trace op:req
	.endm
	.macro count_clock
	.endm
	.macro count_stop
	.endm
	.macro count_ints
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
	count_clock
	.endm

	/* clock_out_bit reg: Clock out bit 31 of reg */
	.macro clock_out_bit reg:req dbit:req
	btst.l	#31,\reg
	beq	98f
	bset.l	\dbit,%DDAT
	trace	#TR_CLKOBIT1
	bra	99f
98:	bclr.l	\dbit,%DDAT
	trace	#TR_CLKOBIT0
99:	/* If data and clock GPIO share the same register, such as on
	 * Romulus, the write done by clock_toggle will set the new data
	 * value along with the low clock edge. Thus we don't need to
	 * set it here, thus saving a PCLK
	*/
#ifndef DCLK_DDAT_SHARED
	move.l	%DDAT,%a4@(0)
#endif
	clock_toggle
	.endm

	/* clock_zeros reg: Clock out zeros (GPIO set to 1), assume at least 1 */
	.macro clock_out_zeros reg:req tmp:req
	trace	#TR_CLKZ
	trace	\reg
	mvz.b	%pc@(_dat_gpio_bit),\tmp
	bset.l	\tmp,%DDAT
#ifndef DCLK_DDAT_SHARED
	move.l	%DDAT,%a4@(0)
#endif
99:	clock_toggle
	subq.l	#1,\reg
	bne	99b
	.endm

	/* clock_in_bit reg: Clocks in bit into bit 0 of reg, the rest is 0
	 * note: bit 0 of reg must already be cleared
	 */
	.macro clock_in_bit reg:req tmp:req dbit:req idx:req
	bclr.l	%d5,%DCLK	/* clock low */
	move.l	%DCLK,%a5@(0)
_clk_in_patch\idx:
	move.l	%a4@(0),\tmp		/* dummy read */
	move.l	%a4@(0),\tmp		/* dummy read */
	move.l	%a4@(0),\tmp		/* actual read */
	bset.l	%d5,%DCLK		/* clock high */
	move.l	%DCLK,%a5@(0)
	count_clock
	lsr.l	\dbit,\tmp
	andi.l	#1,\tmp
	or.l	\tmp,\reg
#ifdef ENABLE_TRACE
	ori.l	#TR_CLKIBIT0,\tmp
	trace	\tmp
#endif
	.endm

	/*
	 * Macro used when there's no STOP or when doing contiguous
	 * clocking insert some NOPs to relieve the bus.
	 */
	.macro pause_nops reg:req
	moveq.l	#32,\reg
0:	nop
	subq.l	#1,\reg
	bne	0b
	.endm

	/*
	 * Main entry point
	 */
	.org	0x500
	.global	_start
_start:
	/* Get base addresses */
	lea	%pc@(_cmd_stat_base),%a0
	movea.l	%a0@(0),%a1
	movea.l	#GPIO_BASE,%a2
	movea.l	%a2,%a5
	mvz.w	%pc@(_clk_gpio_vreg),%d0
	add.l	%d0,%a5
	movea.l	%a2,%a4
	mvz.w	%pc@(_dat_gpio_vreg),%d0
	add.l	%d0,%a4

	/* Cache clock bit number */
	mvz.b	%pc@(_clk_gpio_bit),%d5

	/* Coldfire sets D0 and D1 to special configuration values at reset,
	 * we capture them here for diagnostics purposes
	 */
	move.l	%d0,%a1@(CF_RESET_D0)
	move.l	%d1,%a1@(CF_RESET_D1)

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
	 * Code patching tricks for performance reasons
	 */

	/* If trans GPIO doesn't share a register with either data
	 * nor clock, we can remove some cache reloads
	*/
	mvs.w	%pc@(_tra_gpio_vreg),%d0
	mvs.w	%pc@(_clk_gpio_vreg),%d1
	cmp.l	%d0,%d1
	beq	1f
	mvs.w	%pc@(_dat_gpio_vreg),%d1
	cmp.l	%d0,%d1
	beq	1f
	lea	%pc@(_config_out_load_ret),%a0
	move.w	#0x4e75,%a0@(0)
	bsr	cache_flush
	lea	%pc@(_config_in_load_ret),%a0
	move.w	#0x4e75,%a0@(0)
	bsr	cache_flush

	/* Check if extra dummy read required */
1:	move.l	%pc@(_fw_ctrl),%d0
	moveq.l	#FW_CONTROL_DUMMY_RD,%d1
	and.l	%d0,%d1
	bne	1f
	lea	%pc@(_clk_in_patch0),%a0
	move.w	#0x4e71,%a0@(0)
	bsr	cache_flush
	lea	%pc@(_clk_in_patch1),%a0
	move.w	#0x4e71,%a0@(0)
	bsr	cache_flush
	lea	%pc@(_clk_in_patch2),%a0
	move.w	#0x4e71,%a0@(0)
	bsr	cache_flush
	lea	%pc@(_clk_in_patch3),%a0
	move.w	#0x4e71,%a0@(0)
	bsr	cache_flush

1:	/*
	 * Load GPIO values into caches and set initial values
	 *
	 * Note: We load from the "Data Read" register which
	 *       contains the value of the write latch, and not
	 *       the "Value" register which, when read, returns
	 *       the value sampled on the line. The reason is that
	 *       the value can be missing recent changes due to
	 *       being behind synchronizers.
	 *
	 * Since the trans GPIO may overlap the data or clock one,
	 * and we don't always keep a cache of it, we set it up first before
	 * we load the caches.
	*/

	/* Setup TRANS GPIO */
	mvs.w	%pc@(_tra_gpio_dreg),%d3
	mvs.b	%pc@(_tra_gpio_bit),%d1
	move.l	%a2@(%d3),%d0
	mvs.w	%pc@(_tra_gpio_vreg),%d3
	bset.l	%d1,%d0
	move.l	%d0,%a2@(%d3)
	addq.l	#4,%d3
	move.l	%a2@(%d3),%d0
	bset.l	%d1,%d0
	move.l	%d0,%a2@(%d3)

	/* Load CLK and DAT GPIO caches */
	bsr	load_gpio_caches

	/* Set initial CLK and DAT GPIOs */
	bset.l	%d5,%DCLK
	mvs.b	%pc@(_dat_gpio_bit),%d1
	bset.l	%d1,%DDAT
	move.l	%DCLK,%a5@(0)
	move.l	%DDAT,%a4@(0)

	/* Configure CLK and DAT as output */
	move.l	%a5@(4),%d0
	bset.l	%d5,%d0
	move.l	%d0,%a5@(4)
	move.l	%a4@(4),%d0
	bset.l	%d1,%d0
	move.l	%d0,%a4@(4)

	/* Initialize A6 to point to command area */
	lea	%a1@(CMD_DATA),%a6

	/* Install external interrupt vector */
	lea	%pc@(_int),%a0
	move.l	%a0,(0x46*4)

	/* Mask interrupts */
	move.w	#0x2700,%sr

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Indicate that we are started */
	mov3q.l	#0xffffffff,%a1@(CF_STARTED)

	/*
	 * Main command loop
	 */
main_loop:
	/* Load control bits */
	move.l	%pc@(_fw_ctrl),%d0

wait_cmd:
	/* Reset trace pointer */
#ifdef ENABLE_TRACE
	lea	%a1@(TRACEBUF),%a3
#endif
	/* Wait arbitration request or command */
	move.b	%a1@(ARB_REG),%d2
	bne	arbitration_request
	move.l	%a1@(CMD_STAT_REG),%d2
	tst.b	%d2
	bne	command_request

	/* Are we doing continuous clocking ? */
	moveq.l	#FW_CONTROL_CONT_CLOCK,%d1
	and.l	%d0,%d1
	beq	1f
	clock_toggle
	bra	wait_cmd

	/* Can we use STOP instructions ? */
1:	moveq.l	#FW_CONTROL_USE_STOP,%d1
	and.l	%d0,%d1
	beq	1f
	count_stop
	stop	#0x2000
	move.w	#0x2700,%sr
	bra	wait_cmd

	/* Neither continuous clocks nor STOP, use some nops
	 * and go back
	 */
1:	pause_nops %d1
	bra	wait_cmd

arbitration_request:
	/* Ack request */
	move.b	#ARB_ARM_ACK,%a1@(ARB_REG)

	/* Wait until it's cleared by the host */
arb_wait:
	move.b	%a1@(ARB_REG),%d1
	beq	arb_done

	/* Control bits still in %d0 */
	moveq.l	#FW_CONTROL_USE_STOP,%d1
	and.l	%d0,%d1
	beq	1f

	/* Wait, we'll get an interrupt when the host clears it */
	count_stop
	stop	#0x2000
	move.w	#0x2007,%sr
	bra	arb_wait

1:	/* In absence of SW irq, just loop with some NOPs to avoid
	 * hammering the bus too hard
	*/
	pause_nops %d1
	bra	arb_wait

arb_done:
	/* Got it, re-load the GPIO caches */
	bsr	load_gpio_caches

	/* Reconfigure data as output just in case ... */
	bsr	config_gpio_out

	/* Back to main loop */
	bra	main_loop

command_request:
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

	/* This seems to help performance, probably cache alignemnt/aliasing
	 * of some loops. Rather random but heh...
	 */
	.balign	0x10
start_command:
	/* Start bit */
	mvz.b	%pc@(_dat_gpio_bit),%d1
	moveq.l	#0,%d0
	clock_out_bit %d0,%d1
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
0:	clock_out_bit %d4,%d1
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
0:	clock_out_bit %d4,%d1
	lsl.l	#1,%d4
	subq.l	#1,%d3
	bne	0b

2:	/* Done sending, ready to receive, first echo delay */
	move.b	%a1@(ECHO_DLY_REG),%d3 /* d3 is already 0 */
	clock_out_zeros %d3,%d0

	/* Set GPIO and transceivers to input */
	bsr	config_gpio_in

	/* Wait for start bit */
	move.l	#1000,%d3
	trace	#TR_CLKWSTART
	mvz.b	%pc@(_dat_gpio_bit),%d1
0:	moveq	#0,%d4
	clock_in_bit %d4,%d0,%d1,0
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
	clock_in_bit %d4,%d0,%d1,1
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
	clock_in_bit %d4,%d0,%d1,2
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
	clock_in_bit %d4,%d0,%d1,3
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
	clock_out_zeros %d3,%d0

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Next command */
	bra	main_loop

start_break:
	/* Clock some 1's to pace and flush out whatever's going on */
	move.l	#FSI_PRE_BREAK_CLOCKS,%d3
	clock_out_zeros %d3,%d0

	/* Clock out the break */
	mvz.b	%pc@(_dat_gpio_bit),%d1
	moveq.l	#0,%d0
	clock_out_bit %d0,%d1
	move.l	#(FSI_BREAK_CLOCKS-1),%d3
0:	clock_toggle
	subq.l	#1,%d3
	bne	0b

	/* Clock some more 1's to resync (includes the send delay) */
	move.l	#FSI_POST_BREAK_CLOCKS,%d3
	clock_out_zeros %d3,%d0

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
	clock_out_zeros %d3,%d0

	/* End trace */
	trace	#TR_END

	/* Mark command complete */
	moveq.l	#STAT_COMPLETE,%d0
	byterev	%d0
	move.l	%d0,%a1@(CMD_STAT_REG)

	bra	main_loop

config_gpio_out:
	/* Configure data GPIO as output, value 1 (idle) */
	mvs.b	%pc@(_dat_gpio_bit),%d1
	bset.l	%d1,%DDAT
	move.l	%DDAT,%a4@(0)
	move.l	%a4@(4),%d0
	bset.l	%d1,%d0
	move.l	%d0,%a4@(4)

	/* Set transceivers to output */
	mvs.b	%pc@(_tra_gpio_bit),%d1
#ifndef DCLK_DDAT_SHARED
	mvs.w	%pc@(_tra_gpio_dreg),%d3
	move.l	%a2@(%d3),%DTRA
#endif
	mvs.w	%pc@(_tra_gpio_vreg),%d3
	bset.l	%d1,%DTRA
	move.l	%DTRA,%a2@(%d3)

	/* Reload caches in case of collision & return
	 *
	 * This can be patched out if unnecessary
	 */
_config_out_load_ret:
	bra	load_gpio_caches

config_gpio_in:
	/* Configure data GPIO as input */
	mvs.b	%pc@(_dat_gpio_bit),%d1
	move.l	%a4@(4),%d0
	bclr.l	%d1,%d0
	move.l	%d0,%a4@(4)

	/* Set transceiver to input */
	mvs.b	%pc@(_tra_gpio_bit),%d1
#ifndef DCLK_DDAT_SHARED
	mvs.w	%pc@(_tra_gpio_dreg),%d3
	move.l	%a2@(%d3),%DTRA
#endif
	mvs.w	%pc@(_tra_gpio_vreg),%d3
	bclr.l	%d1,%DTRA
	move.l	%DTRA,%a2@(%d3)

	/* Reload caches in case of collision & return */
_config_in_load_ret:
	bra	load_gpio_caches

load_gpio_caches:
	mvs.w	%pc@(_clk_gpio_dreg),%d0
	move.l	%a2@(%d0),%DCLK
#ifdef DCLK_DDAT_SHARED
	mvs.w	%pc@(_tra_gpio_dreg),%d0
	move.l	%a2@(%d0),%DTRA
#else
	mvs.w	%pc@(_dat_gpio_dreg),%d0
	move.l	%a2@(%d0),%DDAT
#endif
	rts

	/* Flush D and I cache for address %a0 */
cache_flush:
	move.l	%a0,DFLUSH_BASE
	nop
	move.l	%a0,IFLUSH_BASE
	nop
	rts

	/* Interrupt handler
	 *
	 * Note: Must only clobber %d0
	 */
_int:
	move.l	%d0,%a7@-
	move.l	%a0,%a7@-
	count_ints
	moveq.l	#CVIC_SW_IRQ,%d0
	movea.l	#CVIC_BASE,%a0
	move.l	%d0,%a0@(CVIC_SW_IRQ_CLR)
	move.l	%a0@(CVIC_SW_IRQ_CLR),%d0
	move.l	%a7@+,%d0
	movea.l	%a7@+,%a0
	rte

	/* Bad exception handler */
bad_exception:
	move.l	%a7@+,%d0
	move.l	#(_bad_exceptions + 4),%d1
	sub.l	%d1,%d0
	lsr.l	#2,%d0
	move.l	%d0,%a1@(BAD_INT_VEC)
	move.l	%a7@+,%d0
	move.l	%d0,%a1@(BAD_INT_S0)
	move.l	%a7@+,%d0
	move.l	%d0,%a1@(BAD_INT_S1)
	move.b	#STAT_ERR_INVAL_IRQ,%a1@(CMD_STAT_REG)
	bra	.

	/* Bad exception stubs */
_bad_exceptions:
	.rept	256
	bsr.w	bad_exception
	.endr

	/* Stack */
	.balign	0x10
	.space	STACK_SIZE
_stack_top:
	/* Dummy padding */
	.space	0x10
_end:
