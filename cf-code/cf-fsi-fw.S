	.equ	SRAM_BASE_BE,	0x320000
	.equ	SRAM_BASE_LE,	0x720000
	.equ	GPIO_BASE,	0x780000
	.equ	CVIC_BASE,	0x6c2000

	.equ	CVIC_STATUS,		0x00
	.equ	CVIC_SW_IRQ_CLR,	0x1c
	.equ	CVIC_SW_IRQ,		0x2

	/****  SRAM layout ****/

	/* Command register:
	 *
	 * +---------------------------+
	 * | rsvd | RLEN | CLEN | CMD  |
	 * |   8  |   8  |   8  |   8  |
	 * +---------------------------+
	 *            |      |      |
	 * Response len      |      |
	 * (in bits)         |      |
	 *                   |      |
	 *         Command len      |
	 *         (in bits)        |
	 *                          |
	 *               Command code
	 */
	.equ	CMD_REG,		0x00
	.equ	 CMD_NONE,		0x00
	.equ	 CMD_COMMAND,		0x01
	.equ	 CMD_BREAK,		0x02

	/* Status register
	 *
	 */
	.equ	STAT_REG,		0x04 /* Status */
	.equ	 STAT_STOPPED,		0x00
	.equ	 STAT_SENDING,		0x01
	.equ	 STAT_COMPLETE,		0x02
	.equ	 STAT_ERR_INVAL_CMD,	0x80
	.equ	 STAT_ERR_INVAL_IRQ,	0x81
	.equ	 STAT_ERR_MTOE,		0x82

	/* Response tag */
	.equ	STAT_RTAG,		0x05

	/* Response CRC */
	.equ	STAT_RCRC,		0x06

	/* Command data area
	 *
	 * Last byte of message must be left aligned
	 */
	.equ	CMD_DATA,		0x10 /* 64 bit of data */

	/* Response data area, right aligned, unused top bits are 1 */
	.equ	RSP_DATA,		0x20 /* 32 bit of data */

	/* Misc */
	.equ	INT_CNT,		0x30 /* 32-bit interrupt count */
	.equ	BAD_INT_VEC,		0x34
	.equ	TRACEBUF,		0x40
	.equ	  TR_CLKOSTART,		0x00
	.equ	  TR_OLEN,		0x01/* + len */
	.equ	  TR_CLKOBIT0,		0x02
	.equ	  TR_CLKOBIT1,		0x03
	.equ	  TR_CLKZ,		0x04 /* + count */
	.equ	  TR_CLKWSTART,		0x05
	.equ	  TR_CLKTAG,		0x06
	.equ	  TR_CLKDATA,		0x07 /* + len */
	.equ	  TR_CLKCRC,		0x08 /* + raw crc */
	.equ	  TR_CLKIBIT0,		0x80
	.equ	  TR_CLKIBIT1,		0x81

	/* Register usage
	 *
	 * A0 : sratch/temp
	 * A1 : SRAM base (BE)
	 * A2:  CVIC address.
	 * A3:  TRACEBUF
	 * A4 : Data GPIO address
	 * A5 : Clock GPIO address
	 * A6 : CMD/RESP pointer
	 * A7 : Stack pointer
	 * D7 : clock GPIO cache (and data on Romulus)
	 * D6 : data GPIO cache (when need
	 * D5 : unused
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
#if CLOCK_GPIO_REG == DATA_GPIO_REG
#define DDAT d7
#else
#define DDAT d6
#endif
#if TRANS_GPIO_REG == CLOCK_GPIO_REG
#define DTRA d7
#elif TRANS_GPIO_REG == DATA_GPIO_REG
#define DTRA d6
#else
#define DTRA d0
#endif


#ifdef ENABLE_TRACE
	.macro trace op:req
	move.b	\op,%a3@+
	.endm
#else
	.macro trace op:req
	.endm
#endif

	/* clock_toggle: toggle the clock down and back up */
	.macro clock_toggle
	bclr.l	#CLOCK_GPIO_BIT,%DCLK	/* clock low */
	move.l	%DCLK,%a5@(0)
	bset.l	#CLOCK_GPIO_BIT,%DCLK	/* clock high */
	move.l	%DCLK,%a5@(0)
	.endm

	/* clock_out_bit reg: Clock out bit 31 of reg */
	/* XXX TODO: only write to GPIO if value changes */
	/* XXX TODO: can probably optimize further using shifts & logical ops rather than branches */
	.macro clock_out_bit reg:req
	btst.l	#31,\reg
	beq	98f
	bset.l	#DATA_GPIO_BIT,%DDAT
	trace	#TR_CLKOBIT1
	bra	99f
98:	bclr.l	#DATA_GPIO_BIT,%DDAT
	trace	#TR_CLKOBIT0
99:	move.l	%DDAT,%a4@(0)
	clock_toggle
	.endm

	/* clock_zeros reg: Clock out zeros (GPIO set to 1), assume at least 1 */
	.macro clock_out_zeros reg:req
	trace	#TR_CLKZ
	trace	\reg
	bset.l	#DATA_GPIO_BIT,%DDAT
	move.l	%DDAT,%a4@(0)
99:	clock_toggle
	subq.l	#1,\reg
	bne	99b
	.endm

	/* clock_in_bit reg: Clocks in bit into bit 0 of reg, the rest is 0
	 * note: bit 0 of reg must already be cleared
	 */
	.macro clock_in_bit reg:req tmp:req tmp2:req
	bclr.l	#CLOCK_GPIO_BIT,%DCLK	/* clock low */
	move.l	%DCLK,%a5@(0)
	move.l	%a4@(0),\tmp		/* dummy read */
	move.l	%a4@(0),\tmp		/* actual read */
	bset.l	#CLOCK_GPIO_BIT,%DCLK	/* clock high */
	move.l	%DCLK,%a5@(0)
	moveq.l	#DATA_GPIO_BIT,\tmp2
	lsr.l	\tmp2,\tmp
	moveq.l	#1,\tmp2
	and.l	\tmp2,\tmp
	or.l	\tmp,\reg
	.if	TRACE == 1
	move.l	#TR_CLKIBIT0,\tmp2
	or.l	\tmp,\tmp2
	trace	\tmp2
	.endif
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
	.long	0x0ffff0	/* Stack below 1M */
	.long	0x400		/* Start code */

	/* Remaining 254 vectors point to corresponding stubs
	 * starting at 0x10000, 0x10 bytes each
	 */
	.rept	254
0:	.long	0x10000 + (0b - _vecs) * 4
	.endr

	/*
	 * Main entry point
	 */
	.org	0x400
	.global	_start
_start:
	/* Get base addresses */
	movea.l	#SRAM_BASE_BE,%a1
	movea.l	#GPIO_BASE,%a4
	movea.l	%a4,%a5
	add.l	#CLOCK_GPIO_REG,%a5
	add.l	#DATA_GPIO_REG,%a4
	movea.l	#CVIC_BASE,%a2

	/* Load GPIO value and Configure clock & data GPIO as output */
	move.l	%a5@(0),%DCLK
	move.l	%a4@(0),%DDAT
	bset.l	#CLOCK_GPIO_BIT,%DCLK
	bset.l	#DATA_GPIO_BIT,%DDAT
	move.l	%DCLK, %a5@(0)
	move.l	%DDAT, %a4@(0)
	move.l	%a5@(4),%d0
	bset.l	#CLOCK_GPIO_BIT,%d0
	move.l	%d0,%a5@(4)
	move.l	%a4@(4),%d0
	bset.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)

	/* Initialize A6 to point to command area */
	lea	%a1@(CMD_DATA),%a6

	/* Clear interrupt count */
	moveq.l	#0,%d0
	move.l	%d0,%a1@(INT_CNT)

	/* Install external interrupt vector */
	lea	_int,%a0
	move.l	%a0,(0x46*4)

	/* Mask interrupts */
	move.w	#0x2000,%sr

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/*
	 * Main command loop
	 */
main_loop:
	lea	%a1@(TRACEBUF),%a3

	/* Wait for command */
1:	move.l	%a1@(CMD_REG),%d2
	tst.b	%d2
	bne	1f
	stop	#0x2000
	bra	1b

	/* Mask interrupts */
1:	move.w	#0x2007,%sr

	/* Mark ourselves as sending a command */
	move.b	#STAT_SENDING,%a1@(STAT_REG)

	/* Clear command register */
	move.b	#CMD_NONE,%a1@(CMD_REG + 3)

	/* Start command ? */
	cmpi.b	#CMD_COMMAND,%d2
	beq	start_command

	/* Break command ? */
	cmpi.b	#CMD_BREAK,%d2
	beq	start_break

	/* Error */
	move.b	#STAT_ERR_INVAL_CMD,%a1@(STAT_REG)
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
	btst.b	#5,%d2
	beq	1f

	/* Clock out 32 bits */
	moveq	#32,%d3
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
	moveq	#16,%d3
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
	move.b	#STAT_ERR_MTOE,%a1@(STAT_REG)
	bra	send_delay

1:	/* Got start bit, clock in slave ID and response tag */
	trace	#TR_CLKTAG
	moveq	#4,%d3
	moveq	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d3
	bne	0b

	/* Invert data */
	not.l	%d4

	/* (not strictly needed: clean up top bits) */
	moveq	#0xf,%d0
	and.l	%d0,%d4

	/* Store into STAT_RTAG for host */
	move.b	%d4,%a1@(STAT_RTAG)

	/* Extract tag part */
	moveq	#0x7,%d0
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
	trace	%d4

	/* Invert it, extract 4 bits, and store it */
	not.l	%d4
	moveq.l	#0xf,%d0
	and.l	%d0,%d4
	move.b	%d4,%a1@(STAT_RCRC)

	/* Mark command complete */
	move.b	#STAT_COMPLETE,%a1@(STAT_REG)

send_delay:
	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Send delay after every command */
	moveq	#16,%d3
	clock_out_zeros %d3
	bra	main_loop

start_break:
	move.b	#STAT_COMPLETE,%a1@(STAT_REG)
	bra	main_loop

config_gpio_out:
	/* Configure data GPIO as output, value 1 (idle) */
	bset.l	#DATA_GPIO_BIT,%DDAT
	move.l	%DDAT,%a4@(0)
	move.l	%a4@(4),%d0
	bset.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)

	/* Set transceivers to output */
	move.l	%a5@(TRANS_GPIO_REG-CLOCK_GPIO_REG),%DTRA
	bset.l	#TRANS_GPIO_BIT,%DTRA
	move.l	%DTRA,%a5@(TRANS_GPIO_REG-CLOCK_GPIO_REG)
	rts

config_gpio_in:
	/* Set transceiver to input */
	move.l	%a5@(TRANS_GPIO_REG-CLOCK_GPIO_REG),%DTRA
	bclr.l	#TRANS_GPIO_BIT,%DTRA
	move.l	%DTRA,%a5@(TRANS_GPIO_REG-CLOCK_GPIO_REG)

	/* Configure data GPIO as input */
	move.l	%a4@(4),%d0
	bclr.l	#DATA_GPIO_BIT,%d0
	move.l	%d0,%a4@(4)
	rts

	/* Interrupt handler */
_int:
	addq.l	#1,%a1@(INT_CNT)
	moveq.l	#CVIC_SW_IRQ, %d0
	move.l	%d0,%a2@(CVIC_SW_IRQ_CLR)
	rte

	/* Bad exception stubs */
	.org	0x10000
_bad_exceptions:
	.rept	256
	.balign	0x10
0:	move.b	#(0b - _bad_exceptions) / 0x10,%d0
	move.b	%d0,%a1@(BAD_INT_VEC)
	move.b	#STAT_ERR_INVAL_IRQ,%a1@(STAT_REG)
	halt
	.endr
