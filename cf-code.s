	.text

	.equ	TRACE,	0
_vecs:
	/* Vectors */
	.org	0

	/* Boot vector */
	.long	0x0ffff0	/* Stack below 1M */
	.long	0x400		/* Start code */

	/*
	 * Remaining 254 vectors point to corresponding stubs
	 * starting at 0x10000, 0x10 bytes each
	 */
	.rept	254
0:	.long	0x10000 + (0b - _vecs) * 4
	.endr

	/* Main entry */
	.org	0x400
	.global	_start
_start:
	.equ	SRAM_BASE_BE,	0x320000
	.equ	SRAM_BASE_LE,	0x720000
	.equ	GPIO_BASE,	0x780000
	.equ	CVIC_BASE,	0x6c2000

	.equ	CVIC_STATUS,		0x00
	.equ	CVIC_SW_IRQ_CLR,	0x1c
	.equ	CVIC_SW_IRQ,		0x2

	/* XXX Romulus specific definitions */
	.equ	GPIO_YZAAAB_DATA,	0x1e0
	.equ	GPIO_YZAAAB_DIR,	0x1e4
	.equ	GPIO_CLOCK_BIT,		16
	.equ	GPIO_DATA_BIT,		18
	.equ	GPIO_QRST_DATA,		0x80
	.equ	GPIO_QRST_TRANS_BIT,	10

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

	/* Useful macros */

	.if	TRACE == 1
	.macro trace op:req
	move.b	\op,%a3@+
	.endm
	.else
	.macro trace op:req
	.endm
	.endif

	/* clock_toggle: toggle the clock down and back up */
	.macro clock_toggle
	bclr.l	#GPIO_CLOCK_BIT,%d7	/* clock low */
	move.l	%d7,%a1@(0)
	bset.l	#GPIO_CLOCK_BIT,%d7	/* clock high */
	move.l	%d7,%a1@(0)
	.endm

	/* clock_out_bit reg: Clock out bit 31 of reg */
	/* XXX TODO: only write to GPIO if value changes */
	/* XXX TODO: can probably optimize further using shifts & logical ops rather than branches */
	.macro clock_out_bit reg:req
	btst.l	#31,\reg
	beq	98f
	bset.l	#GPIO_DATA_BIT,%d7
	trace	#TR_CLKOBIT1
	bra	99f
98:	bclr.l	#GPIO_DATA_BIT,%d7
	trace	#TR_CLKOBIT0
99:	move.l	%d7,%a1@(0)
	clock_toggle
	.endm

	/* clock_zeros reg: Clock out zeros (GPIO set to 1), assume at least 1 */
	.macro clock_out_zeros reg:req
	trace	#TR_CLKZ
	trace	\reg
	bset.l	#GPIO_DATA_BIT,%d7
	move.l	%d7,%a1@(0)
99:	clock_toggle
	subq.l	#1,\reg
	bne	99b
	.endm

	/* clock_in_bit reg: Clocks in bit into bit 0 of reg, the rest is 0
	 * note: bit 0 of reg must already be cleared
	 */
	.macro clock_in_bit reg:req tmp:req tmp2:req
	bclr.l	#GPIO_CLOCK_BIT,%d7	/* clock low */
	move.l	%d7,%a1@(0)
	move.l	%a1@(0),\tmp		/* dummy read */
	move.l	%a1@(0),\tmp		/* actual read */
	bset.l	#GPIO_CLOCK_BIT,%d7	/* clock high */
	move.l	%d7,%a1@(0)
	moveq.l	#GPIO_DATA_BIT,\tmp2
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

	/* Register usage
	 *
	 * A0 : SRAM base (BE)
	 * A1 : GPIO address. This is he data register, we assume the direction
	 *      register is at this +4
	 * A2:  CVIC address.
	 * A3:  TRACEBUF
	 * A6:  CMD/RESP pointer
	 * D7 : clock GPIO value (and data on Romulus)
	 * D6 : loop counter
	 * D5 : command register
	 * D4 : data value
	 */

	/* Get base addresses */
	movea.l	#SRAM_BASE_BE,%a0
	movea.l	#GPIO_BASE,%a1
	add.l	#GPIO_YZAAAB_DATA,%a1
	movea.l	#CVIC_BASE,%a2

	/* Load GPIO value and Configure clock & data GPIO as output */
	move.l	%a1@(0),%d7
	bset.l	#GPIO_CLOCK_BIT,%d7
	bset.l	#GPIO_DATA_BIT,%d7
	move.l	%d7, %a1@(0)
	move.l	%a1@(4),%d0
	bset.l	#GPIO_CLOCK_BIT,%d0
	bset.l	#GPIO_DATA_BIT,%d0
	move.l	%d0,%a1@(4)

	/* Cache GPIO value */
	move.l	%a1@(0),%d7

	/* Clear interrupt count */
	moveq.l	#0,%d0
	move.l	%d0,%a0@(INT_CNT)

	/* Install external interrupt vector */
	lea	_int,%a6
	move.l	%a6,(0x46*4)

	/* Mask interrupts */
	move.w	#0x2000,%sr

	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/**** Main loop ****/
main_loop:
	/* Initialize A6 to point to command area */
	lea	%a0@(CMD_DATA),%a6

	lea	%a0@(TRACEBUF),%a3

	/* Wait for command */
1:	move.l	%a0@(CMD_REG),%d5
	tst.b	%d5
	bne	1f
	stop	#0x2000
	bra	1b

	/* Mask interrupts */
1:	move.w	#0x2007,%sr

	/* Mark ourselves as sending a command */
	move.b	#STAT_SENDING,%a0@(STAT_REG)

	/* Clear command register */
	move.b	#CMD_NONE,%a0@(CMD_REG + 3)

	/* Start command ? */
	cmpi.b	#CMD_COMMAND,%d5
	beq	start_command

	/* Break command ? */
	cmpi.b	#CMD_BREAK,%d5
	beq	start_break

	/* Error */
	move.b	#STAT_ERR_INVAL_CMD,%a0@(STAT_REG)
	bra	main_loop

start_command:
	/* Start bit */
	moveq.l	#0,%d0
	clock_out_bit %d0
	trace	#TR_CLKOSTART

	/* Load first lword and invert it */
	move.l	%a6@(0),%d4
	not.l	%d4

	/* Shift command right to get bit count at bottom */
	lsr.l	#8,%d5

	trace	#TR_OLEN
	trace	%d5

	/* More than 32 ? If not go to tail
	 *
	 * Note: This assumes we have at least 1 bit to clock
	 */
	btst.b	#5,%d5
	beq	1f

	/* Clock out 32 bits */
	moveq	#32,%d6
	sub.l	%d6,%d5
0:	clock_out_bit %d4
	lsl.l	#1,%d4
	subq.l	#1,%d6
	bne	0b

	/* Get remaining bits */
	move.l	%a6@(4),%d4
	not.l	%d4

	/* Clock out what's left */
1:	moveq.l	#0,%d6
	move.b	%d5,%d6
	beq	2f
	trace	#TR_OLEN
	trace	%d6
0:	clock_out_bit %d4
	lsl.l	#1,%d4
	subq.l	#1,%d6
	bne	0b

2:	/* Done sending, ready to receive, first echo delay */
	moveq	#16,%d6
	clock_out_zeros %d6

	/* Set GPIO and transceivers to input */
	bsr	config_gpio_in

	/* Wait for start bit */
	move.l	#1000,%d6
	trace	#TR_CLKWSTART
0:	moveq	#0,%d4
	clock_in_bit %d4,%d0,%d1
	/* We read inverted value, so wait for a "0" */
	btst	#0,%d4
	beq	1f
	subq.l	#1,%d6
	bne	0b
	move.b	#STAT_ERR_MTOE,%a0@(STAT_REG)
	bra	send_delay

1:	/* Got start bit, clock in slave ID and response tag */
	trace	#TR_CLKTAG
	moveq	#4,%d6
	moveq	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d6
	bne	0b

	/* Invert data */
	not.l	%d4

	/* (not strictly needed: clean up top bits) */
	moveq	#0xf,%d0
	and.l	%d0,%d4

	/* Store into STAT_RTAG for host */
	move.b	%d4,%a0@(STAT_RTAG)

	/* Extract tag part */
	moveq	#0x7,%d0
	and.l	%d0,%d4

	/* If non-0, no data, go get CRC */
	bne	1f

	/* Do we expect data ? */
	lsr.l	#8,%d5
	beq	1f

	/* Let's get data. Assume no more than 32-bits */
	trace	#TR_CLKDATA
	trace	%d5
	move.l	%d5,%d6
	moveq.l	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d6
	bne	0b

	/* Invert data and store it */
	not.l	%d4
	move.l	%d4,%a0@(RSP_DATA)

1:	/* Grab CRC */
	trace	#TR_CLKCRC
	moveq.l	#4,%d6
	moveq.l	#0,%d4
0:	lsl.l	#1,%d4
	clock_in_bit %d4,%d0,%d1
	subq.l	#1,%d6
	bne	0b
	trace	%d4

	/* Invert it, extract 4 bits, and store it */
	not.l	%d4
	moveq.l	#0xf,%d0
	and.l	%d0,%d4
	move.b	%d4,%a0@(STAT_RCRC)

	/* Mark command complete */
	move.b	#STAT_COMPLETE,%a0@(STAT_REG)

send_delay:
	/* Configure GPIOs to output */
	bsr	config_gpio_out

	/* Send delay after every command */
	moveq	#16,%d6
	clock_out_zeros %d6
	bra	main_loop

start_break:
	move.b	#STAT_COMPLETE,%a0@(STAT_REG)
	bra	main_loop

config_gpio_out:
	/* Configure data GPIO as output, value 1 (idle) */
	bset.l	#GPIO_DATA_BIT,%d7
	move.l	%d7,%a1@(0)
	move.l	%a1@(4),%d0
	bset.l	#GPIO_DATA_BIT,%d0
	move.l	%d0,%a1@(4)

	/* Set transceivers to output */
	move.l	%a1@(GPIO_QRST_DATA-GPIO_YZAAAB_DATA),%d0
	bset.l	#GPIO_QRST_TRANS_BIT,%d0
	move.l	%d0,%a1@(GPIO_QRST_DATA-GPIO_YZAAAB_DATA)
	rts

config_gpio_in:
	/* Set transceiver to input */
	move.l	%a1@(GPIO_QRST_DATA-GPIO_YZAAAB_DATA),%d0
	bclr.l	#GPIO_QRST_TRANS_BIT,%d0
	move.l	%d0,%a1@(GPIO_QRST_DATA-GPIO_YZAAAB_DATA)

	/* Configure data GPIO as input */
	move.l	%a1@(4),%d0
	bclr.l	#GPIO_DATA_BIT,%d0
	move.l	%d0,%a1@(4)
	rts

	/* Interrupt handler */
_int:
	addq.l	#1,%a0@(INT_CNT)
	moveq.l	#CVIC_SW_IRQ, %d0
	move.l	%d0,%a2@(CVIC_SW_IRQ_CLR)
	rte

	/* Bad exception stubs */
	.org	0x10000
_bad_exceptions:
	.rept	256
	.balign	0x10
0:	move.b	#(0b - _bad_exceptions) / 0x10,%d0
	move.b	%d0,%a0@(BAD_INT_VEC)
	move.b	#STAT_ERR_INVAL_IRQ,%a0@(STAT_REG)
	halt
	.endr
