#ifndef __CF_FSI_FW_H
#define __CF_FSI_FW_H

/*
 *  SRAM layout: Main part
 */

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
#define	CMD_REG			0x00
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

/* Status register
 *
 */
#define	STAT_REG		0x04 /* Status */
#define	 STAT_STOPPED		0x00
#define	 STAT_SENDING		0x01
#define	 STAT_COMPLETE		0x02
#define	 STAT_ERR_INVAL_CMD	0x80
#define	 STAT_ERR_INVAL_IRQ	0x81
#define	 STAT_ERR_MTOE		0x82

/* Response tag */
#define	STAT_RTAG		0x05

/* Response CRC */
#define	STAT_RCRC		0x06

/* Echo and Send delay */
#define	ECHO_DLY_REG		0x08
#define	SEND_DLY_REG		0x09

/* Signature & version */
#define SYS_SIG_REG		0x0c /* 2 bytes system signature */
#define FW_VERS_REG		0x0e
#define API_VERS_REG		0x0f

/* Current API version */
#define API_VERSION		1

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
#define	TRACEBUF		0x100
#define	  TR_CLKOSTART		0x00
#define	  TR_OLEN		0x01/* + len */
#define	  TR_CLKOBIT0		0x02
#define	  TR_CLKOBIT1		0x03
#define	  TR_CLKZ		0x04 /* + count */
#define	  TR_CLKWSTART		0x05
#define	  TR_CLKTAG		0x06 /* + tag */
#define	  TR_CLKDATA		0x07 /* + len */
#define	  TR_CLKCRC		0x08 /* + raw crc */
#define	  TR_CLKIBIT0		0x80
#define	  TR_CLKIBIT1		0x81

/*
 *  SRAM layout: GPIO arbitration part
 */
#define ARB_REG			0x40
#define  ARB_ARM_REQ		0x01
#define  ARB_ARM_ACK		0x02

#endif /* __CF_FSI_FW_H */

