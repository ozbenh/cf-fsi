#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <byteswap.h>
#include <stdint.h>
#include <stdbool.h>
#include <getopt.h>
#include <limits.h>
#include <assert.h>
#include <arpa/inet.h>
#include <errno.h>
#include <time.h>

#include "cf-fsi-fw.h"

#define dsb() __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" \
				    : : "r" (0) : "memory")

static inline uint8_t readb(void *addr)
{
	dsb();
	return *(volatile uint8_t *)addr;
}

static inline uint16_t readw(void *addr)
{
	dsb();
	return *(volatile uint16_t *)addr;
}

static inline uint32_t readl(void *addr)
{
	dsb();
	return *(volatile uint32_t *)addr;
}

static inline void writeb(uint8_t val, void *addr)
{
	dsb();
	*(volatile uint8_t *)addr = val;
}

static inline void writew(uint16_t val, void *addr)
{
	dsb();
	*(volatile uint16_t *)addr = val;
}

static inline void writel(uint32_t val, void *addr)
{
	dsb();
	*(volatile uint32_t *)addr = val;
}

static inline void writeq(uint64_t val, void *addr)
{
	dsb();
	*(volatile uint64_t *)addr = val;
}

#define SCU_REGS		0x000e2000	/* 1e6e2000 */
#define SCU_COPRO_CTRL	(SCU_REGS + 0x100)
#define SCU_COPRO_RESET		0x00000002
#define SCU_COPRO_CLK_EN		0x00000001
#define SCU_COPRO_SEG0	(SCU_REGS + 0x104) /* 1M */
#define SCU_COPRO_SEG1	(SCU_REGS + 0x108) /* 1M */
#define SCU_COPRO_SEG2	(SCU_REGS + 0x10c) /* 1M */
#define SCU_COPRO_SEG3	(SCU_REGS + 0x110) /* 1M */
#define SCU_COPRO_SEG4	(SCU_REGS + 0x114) /* 1M */
#define SCU_COPRO_SEG5	(SCU_REGS + 0x118) /* 1M */
#define SCU_COPRO_SEG6	(SCU_REGS + 0x11c) /* 1M */
#define SCU_COPRO_SEG7	(SCU_REGS + 0x120) /* 1M */
#define SCU_COPRO_SEG8	(SCU_REGS + 0x124) /* 8M */
#define SCU_COPRO_SEG_SWAP		0x00000001
#define SCU_COPRO_CACHE_CTL	(SCU_REGS + 0x128)
#define SCU_COPRO_CACHE_EN		0x00000001
#define SCU_COPRO_SEG0_CACHE_EN	0x00000002
#define SCU_COPRO_SEG1_CACHE_EN	0x00000004
#define SCU_COPRO_SEG2_CACHE_EN	0x00000008
#define SCU_COPRO_SEG3_CACHE_EN	0x00000010
#define SCU_COPRO_SEG4_CACHE_EN	0x00000020
#define SCU_COPRO_SEG5_CACHE_EN	0x00000040
#define SCU_COPRO_SEG6_CACHE_EN	0x00000080
#define SCU_COPRO_SEG7_CACHE_EN	0x00000100
#define SCU_COPRO_SEG8_CACHE_EN	0x00000200

#define COPRO_ICACHE_FLUSH_REG	0x00008000
#define COPRO_DCACHE_FLUSH_REG	0x00008004

#define SRAM_BASE		0x00120000	/* 1e720000 - actually 36K */
#define SRAM_SIZE		0x00008000

#define GPIO_REGS		0x00180000 /* 1e780000 */
#define GPIO_YZAAAB_CMDSRC0	(GPIO_REGS + 0x170)
#define GPIO_YZAAAB_CMDSRC1	(GPIO_REGS + 0x174)
#define GPIO_QRST_CMDSRC0	(GPIO_REGS + 0x110)
#define GPIO_QRST_CMDSRC1	(GPIO_REGS + 0x114)

#define GPIO_AA_SRC_BIT		0x00010000
#define GPIO_R_SRC_BIT		0x00000100

#define CVIC_BASE		0x000c2000	/* 1e6c2000 */
#define CVIC_EN_REG		0x10
#define CVIC_TRIG_REG		0x18

static void *sysreg;
#define SYSREG_BASE	0x1e600000	/* System registers */
#define SYSREG_SIZE	0x00200000	/* 2M*/

static void *cfmem;
#define CFMEM_BASE	0x9ef00000	/* Reserved memory */
#define CFMEM_SIZE	0x00100000	/* 1M */

#define	FSI_GPIO_CMD_DPOLL      0x2
#define	FSI_GPIO_CMD_EPOLL      0x3
#define	FSI_GPIO_CMD_TERM	0x3f
#define FSI_GPIO_CMD_ABS_AR	0x4
#define FSI_GPIO_CMD_REL_AR	0x5
#define FSI_GPIO_CMD_SAME_AR	0x3	/* but only a 2-bit opcode... */

#define FSI_SLAVE_BASE			0x800
#define FSI_SMODE		0x0	/* R/W: Mode register */
#define FSI_SMODE_WSC		0x80000000	/* Warm start done */
#define FSI_SMODE_ECRC		0x20000000	/* Hw CRC check */
#define FSI_SMODE_SID_SHIFT	24		/* ID shift */
#define FSI_SMODE_SID_MASK	3		/* ID Mask */
#define FSI_SMODE_ED_SHIFT	20		/* Echo delay shift */
#define FSI_SMODE_ED_MASK	0xf		/* Echo delay mask */
#define FSI_SMODE_SD_SHIFT	16		/* Send delay shift */
#define FSI_SMODE_SD_MASK	0xf		/* Send delay mask */
#define FSI_SMODE_LBCRR_SHIFT	8		/* Clk ratio shift */
#define FSI_SMODE_LBCRR_MASK	0xf		/* Clk ratio mask */

#define LAST_ADDR_INVALID		0x1

uint32_t g_last_addr;
bool trace_enabled;

static void open_mem(void)
{
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("can't open /dev/mem");
		exit(1);
	}

	sysreg = mmap(0, SYSREG_SIZE, PROT_READ | PROT_WRITE,
		      MAP_SHARED, fd, SYSREG_BASE);
	if (sysreg == MAP_FAILED) {
		perror("can't map system registers via /dev/mem");
		exit(1);
	}

	cfmem = mmap(0, CFMEM_SIZE, PROT_READ | PROT_WRITE,
		     MAP_SHARED, fd, CFMEM_BASE);
	if (cfmem == MAP_FAILED) {
		perror("can't map CF memory via /dev/mem");
		exit(1);
	}
}

static void setup_cf_maps(void)
{
	/*
	 * Note about byteswap setting: the bus is wired backwards,
	 * so setting the byteswap bit actually makes the ColdFire
	 * work "normally" for a BE processor, ie, put the MSB in
	 * the lowest address byte.
	 *
	 * We thus need to set the bit for our main memory which
	 * contains our program code. We create two mappings for
	 * the register, one with each setting.
	 *
	 * Segments 2 and 3 has a "swapped" mapping (BE)
	 * and 6 and 7 have a non-swapped mapping (LE) which allows
	 * us to avoid byteswapping register accesses since the
	 * registers are all LE.
	 */

	/* Setup segment 0 to our memory region */
	writel(CFMEM_BASE | SCU_COPRO_SEG_SWAP, sysreg + SCU_COPRO_SEG0);

	/* Segments 2 and 3 to sysregs with byteswap (SRAM) */
	writel(SYSREG_BASE | SCU_COPRO_SEG_SWAP, sysreg + SCU_COPRO_SEG2);
	writel(SYSREG_BASE | 0x100000 | SCU_COPRO_SEG_SWAP, sysreg + SCU_COPRO_SEG3);

	/* And segment 6 and 7 to our registers */
	writel(SYSREG_BASE, sysreg + SCU_COPRO_SEG6);
	writel(SYSREG_BASE | 0x100000, sysreg + SCU_COPRO_SEG7);

	/* Memory cachable, regs and SRAM not cachable */
	writel(SCU_COPRO_SEG0_CACHE_EN | SCU_COPRO_CACHE_EN,
	       sysreg + SCU_COPRO_CACHE_CTL);
}

static void reset_cf(void)
{
	writel(SCU_COPRO_RESET, sysreg + SCU_COPRO_CTRL);
	usleep(10);
	writel(0, sysreg + SCU_COPRO_CTRL);
}

static void start_cf(void)
{
	writel(SCU_COPRO_CLK_EN, sysreg + SCU_COPRO_CTRL);
}

static void load_cf_code(void)
{
	extern uint8_t cf_code_start, cf_code_end;

	uint8_t *code = &cf_code_start;
	uint8_t *mem = cfmem;

	while(code < &cf_code_end)
		writeb(*(code++), mem++);
}

static void gpio_source_arm(void)
{
	uint32_t val;

	/* ARM = 00 */
	val = readl(sysreg + GPIO_YZAAAB_CMDSRC0);
	val &= ~GPIO_AA_SRC_BIT;
	writel(val, sysreg + GPIO_YZAAAB_CMDSRC0);
	val = readl(sysreg + GPIO_YZAAAB_CMDSRC1);
	val &= ~GPIO_AA_SRC_BIT;
	writel(val, sysreg + GPIO_YZAAAB_CMDSRC1);

	val = readl(sysreg + GPIO_QRST_CMDSRC0);
	val &= ~GPIO_R_SRC_BIT;
	writel(val, sysreg + GPIO_QRST_CMDSRC0);
	val = readl(sysreg + GPIO_QRST_CMDSRC1);
	val &= ~GPIO_R_SRC_BIT;
	writel(val, sysreg + GPIO_QRST_CMDSRC1);
}

static void gpio_source_cf(void)
{
	uint32_t val;

	/* CF = 10 */
	val = readl(sysreg + GPIO_YZAAAB_CMDSRC0);
	val &= ~GPIO_AA_SRC_BIT;
	writel(val, sysreg + GPIO_YZAAAB_CMDSRC0);
	val = readl(sysreg + GPIO_YZAAAB_CMDSRC1);
	val |= GPIO_AA_SRC_BIT;
	writel(val, sysreg + GPIO_YZAAAB_CMDSRC1);

	val = readl(sysreg + GPIO_QRST_CMDSRC0);
	val &= ~GPIO_R_SRC_BIT;
	writel(val, sysreg + GPIO_QRST_CMDSRC0);
	val = readl(sysreg + GPIO_QRST_CMDSRC1);
	val |= GPIO_R_SRC_BIT;
	writel(val, sysreg + GPIO_QRST_CMDSRC1);
}

static const uint8_t crc4_tab[] = {
	0x0, 0x7, 0xe, 0x9, 0xb, 0xc, 0x5, 0x2,
	0x1, 0x6, 0xf, 0x8, 0xa, 0xd, 0x4, 0x3,
};

/**
 * crc4 - calculate the 4-bit crc of a value.
 * @crc:  starting crc4
 * @x:    value to checksum
 * @bits: number of bits in @x to checksum
 *
 * Returns the crc4 value of @x, using polynomial 0b10111.
 *
 * The @x value is treated as left-aligned, and bits above @bits are ignored
 * in the crc calculations.
 */
static uint8_t crc4(uint8_t c, uint64_t x, int bits)
{
	int i;

	/* mask off anything above the top bit */
	x &= (1ull << bits) - 1;

	/* Align to 4-bits */
	bits = (bits + 3) & ~0x3;

	/* Calculate crc4 over four-bit nibbles, starting at the MSbit */
	for (i = bits - 4; i >= 0; i -= 4)
		c = crc4_tab[c ^ ((x >> i) & 0xf)];

	return c;
}

struct fsi_gpio_msg {
	uint64_t	msg;
	uint8_t		bits;
};

static void msg_push_bits(struct fsi_gpio_msg *msg, uint64_t data, int bits)
{
	msg->msg <<= bits;
	msg->msg |= data & ((1ull << bits) - 1);
	msg->bits += bits;
}

static void msg_push_crc(struct fsi_gpio_msg *msg)
{
	uint8_t crc;
	int top;

	top = msg->bits & 0x3;

	/* start bit, and any non-aligned top bits */
	crc = crc4(0, 1 << top | msg->msg >> (msg->bits - top), top + 1);

	/* aligned bits */
	crc = crc4(crc, msg->msg, msg->bits - top);

	msg_push_bits(msg, crc, 4);
}

static bool check_same_address(int id, uint32_t addr)
{
	/* this will also handle LAST_ADDR_INVALID */
	return g_last_addr == (((id & 0x3) << 21) | (addr & ~0x3));
}

static bool check_relative_address(int id, uint32_t addr, uint32_t *rel_addrp)
{
	uint32_t last_addr = g_last_addr;
	int32_t rel_addr;

	if (last_addr == LAST_ADDR_INVALID)
		return false;

	/* We may be in 23-bit addressing mode, which uses the id as the
	 * top two address bits. So, if we're referencing a different ID,
	 * use absolute addresses.
	 */
	if (((last_addr >> 21) & 0x3) != id)
		return false;

	/* remove the top two bits from any 23-bit addressing */
	last_addr &= (1 << 21) - 1;

	/* We know that the addresses are limited to 21 bits, so this won't
	 * overflow the signed rel_addr */
	rel_addr = addr - last_addr;
	if (rel_addr > 255 || rel_addr < -256)
		return false;

	*rel_addrp = (uint32_t)rel_addr;

	return true;
}

static void last_address_update(int id, bool valid, uint32_t addr)
{
	if (!valid)
		g_last_addr = LAST_ADDR_INVALID;
	else
		g_last_addr = ((id & 0x3) << 21) | (addr & ~0x3);
}

static void build_ar_command(struct fsi_gpio_msg *cmd, uint8_t id,
			     uint32_t addr, size_t size, const void *data)
{
	int i, addr_bits, opcode_bits;
	bool write = !!data;
	uint8_t ds, opcode;
	uint32_t rel_addr;

	cmd->bits = 0;
	cmd->msg = 0;

	/* we have 21 bits of address max */
	addr &= ((1 << 21) - 1);

	/* cmd opcodes are variable length - SAME_AR is only two bits */
	opcode_bits = 3;

	if (check_same_address(id, addr)) {
		/* we still address the byte offset within the word */
		addr_bits = 2;
		opcode_bits = 2;
		opcode = FSI_GPIO_CMD_SAME_AR;

	} else if (check_relative_address(id, addr, &rel_addr)) {
		/* 8 bits plus sign */
		addr_bits = 9;
		addr = rel_addr;
		opcode = FSI_GPIO_CMD_REL_AR;

	} else {
		addr_bits = 21;
		opcode = FSI_GPIO_CMD_ABS_AR;
	}

	/*
	 * The read/write size is encoded in the lower bits of the address
	 * (as it must be naturally-aligned), and the following ds bit.
	 *
	 *	size	addr:1	addr:0	ds
	 *	1	x	x	0
	 *	2	x	0	1
	 *	4	0	1	1
	 *
	 */
	ds = size > 1 ? 1 : 0;
	addr &= ~(size - 1);
	if (size == 4)
		addr |= 1;

	msg_push_bits(cmd, id, 2);
	msg_push_bits(cmd, opcode, opcode_bits);
	msg_push_bits(cmd, write ? 0 : 1, 1);
	msg_push_bits(cmd, addr, addr_bits);
	msg_push_bits(cmd, ds, 1);
	for (i = 0; write && i < size; i++)
		msg_push_bits(cmd, ((uint8_t *)data)[i], 8);

	msg_push_crc(cmd);
}

static void dump_stuff(void)
{
	int i;

	printf("CMD:%08x STAT:%02x RTAG=%02x RCRC=%02x RDATA=%02x #INT=%08x\n",
	       ntohl(readl(sysreg + SRAM_BASE + CMD_REG)),
	       readb(sysreg + SRAM_BASE + STAT_REG),
	       readb(sysreg + SRAM_BASE + STAT_RTAG),
	       readb(sysreg + SRAM_BASE + STAT_RCRC),
	       ntohl(readl(sysreg + SRAM_BASE + RSP_DATA)),
	       ntohl(readl(sysreg + SRAM_BASE + INT_CNT)));

	for (i = 0; trace_enabled && i < 128; i++) {
		printf("%02x ", readb(sysreg + SRAM_BASE + TRACEBUF + i));
		if ((i % 16) == 15)
			printf("\n");
	}
}

static int do_command(uint32_t op)
{
	uint32_t timeout = 100000;
	uint8_t stat;

	/* Clear status reg */
	writeb(0, sysreg + SRAM_BASE + STAT_REG);

	/* Send command */
	writel(htonl(op), sysreg + SRAM_BASE + CMD_REG);

	/* Ring doorbell */
	writel(0x2, sysreg + CVIC_BASE + CVIC_TRIG_REG);

	/* Wait for status to indicate completion (or error) */
	do {
		if (timeout-- == 0) {
			printf("Timeout !\n");

			dump_stuff();
			return -ETIMEDOUT;
		}
		stat = readb(sysreg + SRAM_BASE + STAT_REG);
	} while(stat < STAT_COMPLETE || stat == 0xff);

	if (stat == STAT_COMPLETE)
		return 0;
	switch(stat) {
	case STAT_ERR_INVAL_CMD:
		return -EINVAL;
	case STAT_ERR_INVAL_IRQ:
		return -EIO;
	case STAT_ERR_MTOE:
		return -ETIMEDOUT;
	}
	return -ENXIO;
}

int test_break(void)
{
	printf("Sending break..\n");
	return do_command(CMD_BREAK);
}

int test_rw(uint32_t addr, bool is_write, uint32_t *data)
{
	struct fsi_gpio_msg cmd;
	uint32_t op, resp = 0, crc;
	uint8_t rtag, rcrc, ack;
	uint32_t be_data;
	int rc;

	if (is_write) {
		be_data = htonl(*data);
		build_ar_command(&cmd, 0, addr, 4, &be_data);
	} else
		build_ar_command(&cmd, 0, addr, 4, NULL);

	/* Left align message */
	cmd.msg <<= (64 - cmd.bits);

	/* Store message into SRAM */
	writel(htonl(cmd.msg >> 32), sysreg + SRAM_BASE + CMD_DATA);
	writel(htonl(cmd.msg & 0xffffffff), sysreg + SRAM_BASE + CMD_DATA + 4);

	op = CMD_COMMAND;
	op |= cmd.bits  << CMD_REG_CLEN_SHIFT;
	if (!is_write)
		op |= 32 << CMD_REG_RLEN_SHIFT;

	rc = do_command(op);
	if (rc) {
		printf("Error %d sending command\n", rc);
		return rc;
	}

	if (!is_write)
		resp = ntohl(readl(sysreg + SRAM_BASE + RSP_DATA));
	rtag = readb(sysreg + SRAM_BASE + STAT_RTAG);
	rcrc = readb(sysreg + SRAM_BASE + STAT_RCRC);
	ack = rtag & 3;

	/* we have a whole message now; check CRC */
	crc = crc4(0, 1, 1);
	crc = crc4(crc, rtag, 4);
	if (ack == 0 && !is_write)
		crc = crc4(crc, resp, 32);
	crc = crc4(crc, rcrc, 4);
	if (crc) {
		last_address_update(0, false, 0);
		printf("BAD CRC !!!\n");
		dump_stuff();
		return -ETIMEDOUT;
	}
	if (ack) {
		printf("FSI error 0x%x\n", rtag & 3);
		last_address_update(0, false, 0);
		dump_stuff();
		return -EIO;
	}
	last_address_update(0, true, addr);
	if (data && !is_write)
		*data = resp;
	else
		dump_stuff();
	return 0;
}

/* Encode slave local bus echo delay */
static inline uint32_t fsi_smode_echodly(int x)
{
	return (x & FSI_SMODE_ED_MASK) << FSI_SMODE_ED_SHIFT;
}

/* Encode slave local bus send delay */
static inline uint32_t fsi_smode_senddly(int x)
{
	return (x & FSI_SMODE_SD_MASK) << FSI_SMODE_SD_SHIFT;
}

/* Encode slave local bus clock rate ratio */
static inline uint32_t fsi_smode_lbcrr(int x)
{
	return (x & FSI_SMODE_LBCRR_MASK) << FSI_SMODE_LBCRR_SHIFT;
}


void bench(void)
{
	struct timespec t0, t1;
	uint32_t val, orig;
	uint64_t tns0, tns1;
	int i, rc;

	printf("Bench...\n");
	rc = test_rw(0, false, &orig);
	if (rc)
		return;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	for (i = 0; i < (0x100000 / 4); i++) {
		rc = test_rw(0, false, &val);
		if (rc) {
			printf("Failed after %d iterations\n", i);
			break;
		}
		if (val != orig) {
			printf("mismatch ! %08x vs. %08x\n", val, orig);
			break;
		}
	}
	printf("\n");
	clock_gettime(CLOCK_MONOTONIC, &t1);
	tns0 = t0.tv_sec * 1000000000ull + t0.tv_nsec;
	tns1 = t1.tv_sec * 1000000000ull + t1.tv_nsec;
	fprintf(stderr, "Spent: %lld ms\n", (tns1 - tns0) / 1000000);
}


int main(int argc, char *argv[])
{
	uint32_t val;

	open_mem();

	printf("Resetting ColdFire...\n");
	reset_cf();

	printf("Setting up and starting ColdFire...\n");

	setup_cf_maps();

	load_cf_code();

	gpio_source_cf();

	/* Clear SRAM */
	memset(sysreg + SRAM_BASE, 0x00, 0x1000);
	dsb();

	/* Start ColdFire */
	start_cf();

	/* Wait for ack API version register*/
	do {
		val = readb(sysreg + SRAM_BASE + API_VERS_REG);
	} while (val == 0x00);

	trace_enabled = !!(val & API_VERSION_TRACE_EN);

	printf("SYS_SIG=%.4x FW_VERSION=%d API_VERSION=%d (trace %s)\n",
	       ntohs(readw(sysreg + SRAM_BASE + SYS_SIG_REG)),
	       readb(sysreg + SRAM_BASE + FW_VERS_REG),
	       val & API_VERSION_MASK,
	       trace_enabled ? "enabled" : "disabled");

	/* Configure echo & send delay */
	writeb(16, sysreg + SRAM_BASE + ECHO_DLY_REG);
	writeb(16, sysreg + SRAM_BASE + SEND_DLY_REG);

	/* Enable interrupt */
	writel(0x2, sysreg + CVIC_BASE + CVIC_EN_REG);

	last_address_update(0, false, 0);

	/* Send break */
	test_break();

	/* Test read */
	test_rw(0, false, NULL);
	test_rw(4, false, NULL);

	/* Read smode */
	test_rw(FSI_SLAVE_BASE + FSI_SMODE, false, &val);
	dump_stuff();
	printf("old smode: %08x\n", val);

	/* 6 seems to be the "sweet spot" for performance */
#define ECHO_SEND_DELAY		6

	/* Change it to 2,2 */
	val = FSI_SMODE_WSC | FSI_SMODE_ECRC
		| fsi_smode_echodly(ECHO_SEND_DELAY - 1)
		| fsi_smode_senddly(ECHO_SEND_DELAY - 1)
		| fsi_smode_lbcrr(0x8);
	printf("writing smode 0x%08x..\n", val);
	test_rw(FSI_SLAVE_BASE + FSI_SMODE, true, &val);
	do_command(CMD_IDLE_CLOCKS | (16 << CMD_REG_CLEN_SHIFT));
	writeb(ECHO_SEND_DELAY, sysreg + SRAM_BASE + ECHO_DLY_REG);
	writeb(ECHO_SEND_DELAY, sysreg + SRAM_BASE + SEND_DLY_REG);

	test_rw(FSI_SLAVE_BASE + FSI_SMODE, false, &val);
	printf("new smode: %08x\n", val);

	bench();

	gpio_source_arm();

	return 0;
}


