#ifndef __CF_FSI_FW_H
#define __CF_FSI_FW_H

/*
 * uCode file layout
 *
 * 0000...03ff : m68k exception vectors
 * 0400...04ff : Header info & boot config block
 * 0500....... : Code & stack
 */

/*
 * Header info & boot config area
 *
 * The Header info is built into the ucode and provide version and
 * platform information.
 *
 * the Boot config needs to be adjusted by the ARM prior to starting
 * the ucode if the Command/Status area isn't at 0x320000 in CF space
 * (ie. beginning of SRAM).
 */

#define HDR_OFFSET	        0x400

/* Info: Signature & version */
#define HDR_SYS_SIG		0x00	/* 2 bytes system signature */
#define  SYS_SIG_ROMULUS	0x526d		/* 'Rm' */
#define  SYS_SIG_WITHERSPOON	0x5773		/* 'Ws' */
#define HDR_FW_VERS		0x02	/* 2 bytes Major.Minor */
#define HDR_API_VERS		0x04	/* 2 bytes Major.Minor */
#define  API_VERSION_MAJ	1	/* Current version */
#define  API_VERSION_MIN	1
#define HDR_FW_OPTIONS		0x08	/* 4 bytes option flags */
#define   FW_OPTION_TRACE_EN	0x00000001	/* FW tracing enabled */

/* Boot Config: Address of Command/Status area */
#define HDR_CMD_STAT_AREA	0x80	/* 4 bytes CF address */

/*
 *  Command/Status area layout: Main part
 */

/* Command/Status register:
 *
 * +---------------------------+
 * | STAT | RLEN | CLEN | CMD  |
 * |   8  |   8  |   8  |   8  |
 * +---------------------------+
 *    |       |      |      |
 *    status  |      |      |
 * Response len      |      |
 * (in bits)         |      |
 *                   |      |
 *         Command len      |
 *         (in bits)        |
 *                          |
 *               Command code
 *
 * Due to the big endian layout, that means that a byte read will
 * return the status byte
 */
#define	CMD_STAT_REG	        0x00
#define  CMD_REG_CMD_MASK	0x000000ff
#define  CMD_REG_CMD_SHIFT	0
#define	  CMD_NONE		0x00
#define	  CMD_COMMAND		0x01
#define	  CMD_BREAK		0x02
#define	  CMD_IDLE_CLOCKS	0x03 /* clen = #clocks */
#define   CMD_INVALID		0xff
#define  CMD_REG_CLEN_MASK	0x0000ff00
#define  CMD_REG_CLEN_SHIFT	8
#define  CMD_REG_RLEN_MASK	0x00ff0000
#define  CMD_REG_RLEN_SHIFT	16
#define  CMD_REG_STAT_MASK	0xff000000
#define  CMD_REG_STAT_SHIFT	24
#define	  STAT_WORKING		0x00
#define	  STAT_COMPLETE		0x01
#define	  STAT_ERR_INVAL_CMD	0x80
#define	  STAT_ERR_INVAL_IRQ	0x81
#define	  STAT_ERR_MTOE		0x82

/* Response tag & CRC */
#define	STAT_RTAG		0x04

/* Response CRC */
#define	STAT_RCRC		0x05

/* Echo and Send delay */
#define	ECHO_DLY_REG		0x08
#define	SEND_DLY_REG		0x09

/* Command data area
 *
 * Last byte of message must be left aligned
 */
#define	CMD_DATA		0x10 /* 64 bit of data */

/* Response data area, right aligned, unused top bits are 1 */
#define	RSP_DATA		0x20 /* 32 bit of data */

/* Misc */
#define	INT_CNT			0x30 /* 32-bit interrupt count */
#define	BAD_INT_VEC		0x34
#define	CF_STARTED		0x38 /* byte, set to -1 when copro started */

/*
 *  SRAM layout: GPIO arbitration part
 */
#define ARB_REG			0x40
#define  ARB_ARM_REQ		0x01
#define  ARB_ARM_ACK		0x02

/* Misc2 */
#define CF_RESET_D0		0x50
#define CF_RESET_D1		0x54

/*
 * SRAM layout: Trace buffer (debug builds only)
 */
#define	TRACEBUF		0x100
#define	  TR_CLKOBIT0		0xc0
#define	  TR_CLKOBIT1		0xc1
#define	  TR_CLKOSTART		0x82
#define	  TR_OLEN		0x83 /* + len */
#define	  TR_CLKZ		0x84 /* + count */
#define	  TR_CLKWSTART		0x85
#define	  TR_CLKTAG		0x86 /* + tag */
#define	  TR_CLKDATA		0x87 /* + len */
#define	  TR_CLKCRC		0x88 /* + raw crc */
#define	  TR_CLKIBIT0		0x90
#define	  TR_CLKIBIT1		0x91
#define	  TR_END		0xff

#endif /* __CF_FSI_FW_H */

