// SPDX-License-Identifier: GPL-2.0+

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

#undef FORCE_SYNC

#ifdef ROMULUS
#define dsb() __asm__ __volatile__ ("mcr p15, 0, %0, c7, c10, 4" \
				    : : "r" (0) : "memory")
#else
#define dsb()
#endif

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
	dsb();
}

static inline void writew(uint16_t val, void *addr)
{
	dsb();
	*(volatile uint16_t *)addr = val;
	dsb();
}

static inline void writel(uint32_t val, void *addr)
{
	dsb();
	*(volatile uint32_t *)addr = val;
	dsb();
}

static inline void writeq(uint64_t val, void *addr)
{
	dsb();
	*(volatile uint64_t *)addr = val;
	dsb();
}

#define SCU_REGS			0x000e2000	/* 1e6e2000 */
#define SCU_COPRO_CTRL			(SCU_REGS + 0x100)
#define SCU_COPRO_RESET			0x00000002
#define SCU_COPRO_CLK_EN		0x00000001

#define SCU_2500_COPRO_SEG0		(SCU_REGS + 0x104) /* 1M */
#define SCU_2500_COPRO_SEG1		(SCU_REGS + 0x108) /* 1M */
#define SCU_2500_COPRO_SEG2		(SCU_REGS + 0x10c) /* 1M */
#define SCU_2500_COPRO_SEG3		(SCU_REGS + 0x110) /* 1M */
#define SCU_2500_COPRO_SEG4		(SCU_REGS + 0x114) /* 1M */
#define SCU_2500_COPRO_SEG5		(SCU_REGS + 0x118) /* 1M */
#define SCU_2500_COPRO_SEG6		(SCU_REGS + 0x11c) /* 1M */
#define SCU_2500_COPRO_SEG7		(SCU_REGS + 0x120) /* 1M */
#define SCU_2500_COPRO_SEG8		(SCU_REGS + 0x124) /* 8M */
#define SCU_2500_COPRO_SEG_SWAP		0x00000001
#define SCU_2500_COPRO_CACHE_CTL	(SCU_REGS + 0x128)
#define SCU_2500_COPRO_CACHE_EN		0x00000001
#define SCU_2500_COPRO_SEG0_CACHE_EN	0x00000002
#define SCU_2500_COPRO_SEG1_CACHE_EN	0x00000004
#define SCU_2500_COPRO_SEG2_CACHE_EN	0x00000008
#define SCU_2500_COPRO_SEG3_CACHE_EN	0x00000010
#define SCU_2500_COPRO_SEG4_CACHE_EN	0x00000020
#define SCU_2500_COPRO_SEG5_CACHE_EN	0x00000040
#define SCU_2500_COPRO_SEG6_CACHE_EN	0x00000080
#define SCU_2500_COPRO_SEG7_CACHE_EN	0x00000100
#define SCU_2500_COPRO_SEG8_CACHE_EN	0x00000200

#define SCU_2400_COPRO_SEG0		(SCU_REGS + 0x104)
#define SCU_2400_COPRO_SEG2		(SCU_REGS + 0x108)
#define SCU_2400_COPRO_SEG4		(SCU_REGS + 0x10c)
#define SCU_2400_COPRO_SEG6		(SCU_REGS + 0x110)
#define SCU_2400_COPRO_SEG8		(SCU_REGS + 0x114)
#define SCU_2400_COPRO_SEG_SWAP		0x80000000
#define SCU_2400_COPRO_CACHE_CTL	(SCU_REGS + 0x118)
#define SCU_2400_COPRO_CACHE_EN		0x00000001
#define SCU_2400_COPRO_SEG0_CACHE_EN	0x00000002
#define SCU_2400_COPRO_SEG2_CACHE_EN	0x00000004
#define SCU_2400_COPRO_SEG4_CACHE_EN	0x00000008
#define SCU_2400_COPRO_SEG6_CACHE_EN	0x00000010
#define SCU_2400_COPRO_SEG8_CACHE_EN	0x00000020

#define COPRO_ICACHE_FLUSH_REG		0x00008000
#define COPRO_DCACHE_FLUSH_REG		0x00008004

#define SRAM_BASE			0x00120000	/* 1e720000 - actually 36K */
#define SRAM_SIZE			0x00008000

#define GPIO_REGS		0x00180000 /* 1e780000 */
#ifdef ROMULUS
#define GPIO_YZAAAB_CMDSRC0	(GPIO_REGS + 0x170)
#define GPIO_YZAAAB_CMDSRC1	(GPIO_REGS + 0x174)
#define GPIO_QRST_CMDSRC0	(GPIO_REGS + 0x110)
#define GPIO_QRST_CMDSRC1	(GPIO_REGS + 0x114)
#define GPIO_QRST_DATA		(GPIO_REGS + 0x080)
#define GPIO_QRST_DIR		(GPIO_REGS + 0x084)
#define GPIO_QRST_DATARD	(GPIO_REGS + 0x0d0)
#define GPIO_AA_SRC_BIT		0x00010000
#define GPIO_R_SRC_BIT		0x00000100
#endif

#ifdef PALMETTO
#define GPIO_ABCD_CMDSRC0	(GPIO_REGS + 0x060)
#define GPIO_ABCD_CMDSRC1	(GPIO_REGS + 0x064)
#define GPIO_EFGH_CMDSRC0	(GPIO_REGS + 0x068)
#define GPIO_EFGH_CMDSRC1	(GPIO_REGS + 0x06c)
#define GPIO_A_SRC_BIT		0x00000001
#define GPIO_H_SRC_BIT		0x01000000
#endif

#define CVIC_BASE		0x000c2000	/* 1e6c2000 */
#define CVIC_EN_REG		0x10
#define CVIC_TRIG_REG		0x18

static void *sysreg;
#define SYSREG_BASE	0x1e600000	/* System registers */
#define SYSREG_SIZE	0x00200000	/* 2M*/

static void *cfmem;
#ifdef ROMULUS
#define CFMEM_BASE	0x9ef00000	/* Reserved memory */
#define CFMEM_SIZE	0x00100000	/* 1M */
#endif
#ifdef PALMETTO
#define CFMEM_BASE	0x5ee00000	/* Reserved memory */
#define CFMEM_SIZE	0x00200000	/* 2M */
#endif

#define	FSI_GPIO_CMD_DPOLL      0x2
#define	FSI_GPIO_CMD_EPOLL      0x3
#define	FSI_GPIO_CMD_TERM	0x3f
#define FSI_GPIO_CMD_ABS_AR	0x4
#define FSI_GPIO_CMD_REL_AR	0x5
#define FSI_GPIO_CMD_SAME_AR	0x3	/* but only a 2-bit opcode... */

#define FSI_SLAVE_BASE		0x800
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

#define FSI_LLMODE		0x100	/* R/W: Link layer mode register */
#define FSI_LLMODE_ASYNC	0x1


#define LAST_ADDR_INVALID		0x1

uint32_t g_last_addr;
bool trace_enabled;
int slave_id;
int busy_count;

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

#ifdef ROMULUS
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
	writel(CFMEM_BASE | SCU_2500_COPRO_SEG_SWAP, sysreg + SCU_2500_COPRO_SEG0);

	/* Segments 2 and 3 to sysregs with byteswap (SRAM) */
	writel(SYSREG_BASE | SCU_2500_COPRO_SEG_SWAP, sysreg + SCU_2500_COPRO_SEG2);
	writel(SYSREG_BASE | 0x100000 | SCU_2500_COPRO_SEG_SWAP, sysreg + SCU_2500_COPRO_SEG3);

	/* And segment 6 and 7 to our registers */
	writel(SYSREG_BASE, sysreg + SCU_2500_COPRO_SEG6);
	writel(SYSREG_BASE | 0x100000, sysreg + SCU_2500_COPRO_SEG7);

	/* Memory cachable, regs and SRAM not cachable */
	writel(SCU_2500_COPRO_SEG0_CACHE_EN | SCU_2500_COPRO_CACHE_EN,
	       sysreg + SCU_2500_COPRO_CACHE_CTL);
}
#endif

#ifdef PALMETTO
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
	writel(CFMEM_BASE | SCU_2400_COPRO_SEG_SWAP, sysreg + SCU_2400_COPRO_SEG0);

	/* Segments 2 to sysregs with byteswap (SRAM) */
	writel(SYSREG_BASE | SCU_2400_COPRO_SEG_SWAP, sysreg + SCU_2400_COPRO_SEG2);

	/* And segment 6to our registers */
	writel(SYSREG_BASE, sysreg + SCU_2400_COPRO_SEG6);

	/* Memory cachable, regs and SRAM not cachable */
	writel(SCU_2400_COPRO_SEG0_CACHE_EN | SCU_2400_COPRO_CACHE_EN,
	       sysreg + SCU_2400_COPRO_CACHE_CTL);
}
#endif

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

#ifdef ROMULUS
#define WANTED_SIG SYS_SIG_SHARED
static void setup_cf_config(void)
{
	void *base = cfmem + HDR_OFFSET;

	writew(htons(0x01e0), base + HDR_CLOCK_GPIO_VADDR);
	writew(htons(0x00d8), base + HDR_CLOCK_GPIO_DADDR);
	writew(htons(0x01e0), base + HDR_DATA_GPIO_VADDR);
	writew(htons(0x00d8), base + HDR_DATA_GPIO_DADDR);
	writew(htons(0x0080), base + HDR_TRANS_GPIO_VADDR);
	writew(htons(0x00d0), base + HDR_TRANS_GPIO_DADDR);
	writeb(16, base + HDR_CLOCK_GPIO_BIT);
	writeb(18, base + HDR_DATA_GPIO_BIT);
	writeb(10, base + HDR_TRANS_GPIO_BIT);
#ifdef FORCE_SYNC
	writel(htonl(FW_CONTROL_CONT_CLOCK), base + HDR_FW_CONTROL);
#else
	writel(htonl(FW_CONTROL_USE_STOP), base + HDR_FW_CONTROL);
#endif
}
#endif

#ifdef PALMETTO
#define WANTED_SIG SYS_SIG_SHARED
static void setup_cf_config(void)
{
	void *base = cfmem + HDR_OFFSET;

	writew(htons(0x0000), base + HDR_CLOCK_GPIO_VADDR);
	writew(htons(0x00c0), base + HDR_CLOCK_GPIO_DADDR);
	writew(htons(0x0000), base + HDR_DATA_GPIO_VADDR);
	writew(htons(0x00c0), base + HDR_DATA_GPIO_DADDR);
	writew(htons(0x0020), base + HDR_TRANS_GPIO_VADDR);
	writew(htons(0x00c4), base + HDR_TRANS_GPIO_DADDR);
	writeb(4, base + HDR_CLOCK_GPIO_BIT);
	writeb(5, base + HDR_DATA_GPIO_BIT);
	writeb(30, base + HDR_TRANS_GPIO_BIT);
	writel(htonl(FW_CONTROL_CONT_CLOCK|FW_CONTROL_DUMMY_RD), base + HDR_FW_CONTROL);
}
#endif

static uint8_t *find_cf_code(uint16_t want_sig, size_t *out_size)
{
	extern uint8_t cf_code_start, cf_code_end;
	uint8_t *start = &cf_code_start;
	uint8_t *end = &cf_code_end;
	size_t size;
	uint16_t sig;

	while(start < end) {
		sig = ntohs(*(uint16_t *)(start + HDR_OFFSET + HDR_SYS_SIG));
		size = ntohl(*(uint32_t *)(start + HDR_OFFSET + HDR_FW_SIZE));
		if (sig == want_sig) {
			*out_size = size;
			return start;
		}
		start += size;
	}
	return NULL;
}

static void load_cf_code(void)
{
	uint16_t sig, fw_vers, api_vers;
	uint32_t fw_options;
	uint8_t *code, *end;
	size_t size;
	uint8_t *mem = cfmem;

	code = find_cf_code(WANTED_SIG, &size);
	if (!code) {
		printf("Can't find code signature %04x\n", WANTED_SIG);
		exit(1);
	}
	end = code + size;
	while(code < end)
		writeb(*(code++), mem++);

	sig = ntohs(readw(cfmem + HDR_OFFSET + HDR_SYS_SIG));
	fw_vers = ntohs(readw(cfmem + HDR_OFFSET + HDR_FW_VERS));
	api_vers = ntohs(readw(cfmem + HDR_OFFSET + HDR_API_VERS));
	fw_options = ntohl(readl(cfmem + HDR_OFFSET + HDR_FW_OPTIONS));

	trace_enabled = !!(fw_options & FW_OPTION_TRACE_EN);

	printf("SYS_SIG=%.4x FW_VERSION=%d API_VERSION=%d.%d (trace %s)\n",
	       sig, fw_vers, api_vers >> 8, api_vers & 0xff,
	       trace_enabled ? "enabled" : "disabled");

	setup_cf_config();
}

#ifdef ROMULUS
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
#endif

#ifdef PALMETTO
#if 0
static void gpio_source_arm(void)
{
	uint32_t val;

	/* ARM = 00 */
	val = readl(sysreg + GPIO_ABCD_CMDSRC0);
	val &= ~GPIO_A_SRC_BIT;
	writel(val, sysreg + GPIO_ABCD_CMDSRC0);
	val = readl(sysreg + GPIO_ABCD_CMDSRC1);
	val &= ~GPIO_A_SRC_BIT;
	writel(val, sysreg + GPIO_ABCD_CMDSRC1);

	val = readl(sysreg + GPIO_EFGH_CMDSRC0);
	val &= ~GPIO_H_SRC_BIT;
	writel(val, sysreg + GPIO_EFGH_CMDSRC0);
	val = readl(sysreg + GPIO_EFGH_CMDSRC1);
	val &= ~GPIO_H_SRC_BIT;
	writel(val, sysreg + GPIO_EFGH_CMDSRC1);
}
#endif
static void gpio_source_cf(void)
{
	uint32_t val;

	/* CF = 10 */
	val = readl(sysreg + GPIO_ABCD_CMDSRC0);
	val &= ~GPIO_A_SRC_BIT;
	writel(val, sysreg + GPIO_ABCD_CMDSRC0);
	val = readl(sysreg + GPIO_ABCD_CMDSRC1);
	val |= GPIO_A_SRC_BIT;
	writel(val, sysreg + GPIO_ABCD_CMDSRC1);

	val = readl(sysreg + GPIO_EFGH_CMDSRC0);
	val &= ~GPIO_H_SRC_BIT;
	writel(val, sysreg + GPIO_EFGH_CMDSRC0);
	val = readl(sysreg + GPIO_EFGH_CMDSRC1);
	val |= GPIO_H_SRC_BIT;
	writel(val, sysreg + GPIO_EFGH_CMDSRC1);
}
#endif

#ifdef TEST_GPIO
#ifndef ROMULUS
#error
#endif

static void copro_gpio_request(void)
{
	int timeout;
	uint32_t val;

	/* Write reqest */
	writeb(ARB_ARM_REQ, sysreg + SRAM_BASE + ARB_REG);

	/* Ring doorbell */
	writel(0x2, sysreg + CVIC_BASE + CVIC_TRIG_REG);

	for (timeout = 0; timeout < 1000000; timeout++) {
		val = readb(sysreg + SRAM_BASE + ARB_REG);
		if (val != ARB_ARM_REQ)
			break;
	}

	/* If it failed, override anyway */
	if (val != ARB_ARM_ACK)
		printf("GPIO request arbitration timeout\n");
}

static void copro_gpio_release(void)
{
	/* Write release */
	writeb(0, sysreg + SRAM_BASE + ARB_REG);

	/* Ring doorbell */
	writel(0x2, sysreg + CVIC_BASE + CVIC_TRIG_REG);
}

static bool no_release = false;
static bool no_switch_own = false;

#define GPIO_TEST_BIT	0x2000
static void test_gpio_stuff(void)
{
	uint32_t cache = readl(sysreg + GPIO_QRST_DATA);
	uint32_t val;
	bool good, first = true;

	if (no_release)
		copro_gpio_request();

	printf("t0: dir=%08x\n", readl(sysreg + GPIO_QRST_DIR));

	for (;;) {
		val = readl(sysreg + GPIO_QRST_DATARD);
		good = ((val ^ cache) & GPIO_TEST_BIT) == 0;
		printf("t1: cache=%08x reg=%08x %c\n", cache, val, good ? ' ' : '$');

		if (!no_release)
			copro_gpio_request();

		writel(0, sysreg + SRAM_BASE + 0x3c);

		if (!no_switch_own || first) {
			first = false;
			val = readl(sysreg + GPIO_QRST_CMDSRC1);
			val &= ~GPIO_R_SRC_BIT;
			writel(val, sysreg + GPIO_QRST_CMDSRC1);
			val = readl(sysreg + GPIO_QRST_CMDSRC0);
			val &= ~GPIO_R_SRC_BIT;
			writel(val, sysreg + GPIO_QRST_CMDSRC0);
		}

		cache ^= GPIO_TEST_BIT;
		writel(cache, sysreg + GPIO_QRST_DATA);

		do {
			val = readl(sysreg + GPIO_QRST_DATARD);
			good = ((val ^ cache) & GPIO_TEST_BIT) == 0;
			printf("t2: cache=%08x reg=%08x %c\n", cache, val, good ? '*' : '!');
		} while(!good);

		if (!no_switch_own) {
			val = readl(sysreg + GPIO_QRST_CMDSRC1);
			val |= GPIO_R_SRC_BIT;
			writel(val, sysreg + GPIO_QRST_CMDSRC1);
			val = readl(sysreg + GPIO_QRST_CMDSRC0);
			val &= ~GPIO_R_SRC_BIT;
			writel(val, sysreg + GPIO_QRST_CMDSRC0);
		}

		if (!no_release)
			copro_gpio_release();

		usleep(1);
		printf("t3: cache=%08x reg=%08x\n---\n",
		       cache, readl(sysreg + GPIO_QRST_DATARD));

		sleep(1);
	}
}

#endif

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

static void msg_finish_cmd(struct fsi_gpio_msg *cmd)
{
	/* Left align message */
	cmd->msg <<= (64 - cmd->bits);
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
	msg_finish_cmd(cmd);
}

static void build_dpoll_command(struct fsi_gpio_msg *cmd, uint8_t slave_id)
{
	cmd->bits = 0;
	cmd->msg = 0;

	msg_push_bits(cmd, slave_id, 2);
	msg_push_bits(cmd, FSI_GPIO_CMD_DPOLL, 3);
	msg_push_crc(cmd);
	msg_finish_cmd(cmd);
}

static void dump_stuff(void)
{
	int i;

	printf("CMD:%08x RTAG=%02x RCRC=%02x RDATA=%02x BAD_INT=%08x (%08x %08x)\n",
	       ntohl(readl(sysreg + SRAM_BASE + CMD_STAT_REG)),
	       readb(sysreg + SRAM_BASE + STAT_RTAG),
	       readb(sysreg + SRAM_BASE + STAT_RCRC),
	       ntohl(readl(sysreg + SRAM_BASE + RSP_DATA)),
	       ntohl(readl(sysreg + SRAM_BASE + BAD_INT_VEC)),
	       ntohl(readl(sysreg + SRAM_BASE + BAD_INT_S0)),
	       ntohl(readl(sysreg + SRAM_BASE + BAD_INT_S1)));
	if (trace_enabled) {
		printf("#INT=%08x #CLK=%08x #STOP=%08x\n",
		       ntohl(readl(sysreg + SRAM_BASE + INT_CNT)),
		       ntohl(readl(sysreg + SRAM_BASE + CLK_CNT)),
		       ntohl(readl(sysreg + SRAM_BASE + STOP_CNT)));
	}

	for (i = 0; trace_enabled && i < 128; i++) {
		uint8_t v = readb(sysreg + SRAM_BASE + TRACEBUF + i);
		printf("%02x ", v);
		if ((i % 16) == 15)
			printf("\n");
		if (v == TR_END)
			break;
	}
	if (i % 16)
		printf("\n");
}

static int do_command(uint32_t op)
{
	uint32_t timeout = 1000000;
	uint8_t stat;

	/* Clear trace */
	if (trace_enabled) {
		memset(sysreg + SRAM_BASE + TRACEBUF, 0x00, 128);
	}

	/* Send command */
	writel(htonl(op), sysreg + SRAM_BASE + CMD_STAT_REG);

	/* Ring doorbell */
	writel(0x2, sysreg + CVIC_BASE + CVIC_TRIG_REG);

	/* Wait for status to indicate completion (or error) */
	do {
		if (timeout-- == 0) {
			printf("Timeout !\n");

			dump_stuff();
			return -ETIMEDOUT;
		}
		stat = readb(sysreg + SRAM_BASE + CMD_STAT_REG);
	} while(stat < STAT_COMPLETE || stat == 0xff);

	if (stat == STAT_COMPLETE)
		return 0;
	dump_stuff();
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

int dummy_clocks(int count)
{
	int rc;

	while(count > 0) {
		rc = do_command(CMD_IDLE_CLOCKS | (100 << 8));
		if (rc)
			return rc;
		count -= 100;
	}
	return 0;
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
		build_ar_command(&cmd, slave_id, addr, 4, &be_data);
	} else
		build_ar_command(&cmd, slave_id, addr, 4, NULL);

 try_again:
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
	rtag = readb(sysreg + SRAM_BASE + STAT_RTAG) & 0xf;
	rcrc = readb(sysreg + SRAM_BASE + STAT_RCRC) & 0xf;
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
	if (ack == 1) {
		//printf("BUSY ... DPOLL'ing\n");
		busy_count++;
		//dump_stuff();
		do_command(CMD_IDLE_CLOCKS | (50 << 8));
		build_dpoll_command(&cmd, slave_id);
		goto try_again;
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
	uint32_t val, timeout;

	open_mem();

	printf("Resetting ColdFire...\n");
	reset_cf();

	printf("Setting up and starting ColdFire...\n");

	setup_cf_maps();

	printf("Loading CF code...\n");
	load_cf_code();

	printf("Switching GPIOs...\n");
	gpio_source_cf();

	/* Clear SRAM */
	printf("Clearing SRAM...\n");
	memset(sysreg + SRAM_BASE, 0x00, 0x1000);
	dsb();

	/* Start ColdFire */
	printf("Starting CF...\n");
	start_cf();

	/* Wait for status register to say command complete */
	timeout = 10000;
	do {
		if (!--timeout) {
			printf("Startup failed !\n");
			dump_stuff();
		}
		val = readl(sysreg + SRAM_BASE + CF_STARTED);
		usleep(10);
	} while (val == 0x00);

	/* Configure echo & send delay */
	writeb(16, sysreg + SRAM_BASE + ECHO_DLY_REG);
	writeb(16, sysreg + SRAM_BASE + SEND_DLY_REG);

	/* Enable interrupt */
	writel(0x2, sysreg + CVIC_BASE + CVIC_EN_REG);

	last_address_update(0, false, 0);
	slave_id = 3;

#ifdef PALMETTO
	/* Let it run for a bit */
	sleep(1);
#endif
	dump_stuff();

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
#define ECHO_SEND_DELAY		16
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
	slave_id = 0;

	test_rw(FSI_SLAVE_BASE + FSI_SMODE, false, &val);
	printf("new smode: %08x\n", val);

#ifdef ROMULUS
	test_rw(FSI_SLAVE_BASE + FSI_LLMODE, false, &val);
	printf("llmode: %08x\n", val);
#ifdef FORCE_SYNC
	val = 0;
	test_rw(FSI_SLAVE_BASE + FSI_LLMODE, true, &val);
	printf("new llmode: %08x\n", val);
#endif
#endif

#ifdef PALMETTO
	/* Boot the host */
	printf("ATTNA...\n");
	val = 0x20000000;
	test_rw(0x870, true, &val);
	printf("ATTNB...\n");
	val = 0x40000000;
	test_rw(0x1034, true, &val);
	printf("ATTNC...\n");
	val = 0xffffffff;
	test_rw(0x102c, true, &val);
	printf("Primary select...\n");
	val = 0x30000000;
	test_rw(0x2870, true, &val);
	printf("Go select...\n");
	val = 0xB0000000;
	test_rw(0x2870, true, &val);
#endif
#ifdef TEST_GPIO
	test_gpio_stuff();
#else
	//bench();
	printf("Busy count: %d\n", busy_count);
#endif
	printf("Press return...\n");
	getchar();
#ifndef PALMETTO
	gpio_source_arm();
#endif
	return 0;
}


