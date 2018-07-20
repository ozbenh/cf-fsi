# Main debug switch
DEBUG ?= 0

# FW version
FW_VERSION = 4

# ARM compiler for userspace test code
CROSS_COMPILE ?= arm-linux-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Os -Werror

# M68K toolchain for CF firmware
M68KCROSS ?= /opt/cross/binutils-coldfire/bin/m68k-unknown-elf-
M68KAS=$(M68KCROSS)as
M68KLD=$(M68KCROSS)ld
M68KOC=$(M68KCROSS)objcopy

M68KCPPFLAGS = -DFW_VERSION=$(FW_VERSION)

ifeq ($(DEBUG),1)
M68KCPPFLAGS += -DENABLE_TRACE
endif
M68KAFLAGS = -march=isac --pcrel -k
M68KLDFLAGS = -Ttext 0

# FW code files
TARGET_DEFS = $(wildcard cf-code/*.h)
TARGETS_bin = $(patsubst %.h,%.bin,$(TARGET_DEFS))

FW_SOURCE = cf-code/cf-fsi-fw.S
FW_DEPS = $(FW_SOURCE) cf-fsi-fw.h

all: cf-fsi-fw.bin cf-fsi-test-rom cf-fsi-test-palm

cf-code/%.s : cf-code/%.h $(FW_DEPS)
	$(CC) -E $(M68KCPPFLAGS) -I. -include $< $(FW_SOURCE) -o $@

cf-code/%.o : cf-code/%.s
	$(M68KAS) $(M68KAFLAGS) -march=isac $^ -o $@

cf-code/%.elf : cf-code/%.o
	$(M68KLD) $(M68KLDFLAGS) $^ -o $@

cf-code/%.bin : cf-code/%.elf
	$(M68KOC) -O binary $^ $@

cf-fsi-fw.bin : $(TARGETS_bin)
	cat $^ >$@

cf-wrapper.o : cf-fsi-fw.bin cf-wrapper.S
	$(CC) $(CFLAGS) -DCF_FILE=$< -c cf-wrapper.S -o $@

cf-fsi-test-rom : cf-fsi-test.c cf-wrapper.o
	$(CC) $(CFLAGS) -DROMULUS $^ -o $@

cf-fsi-test-palm : cf-fsi-test.c cf-wrapper.o
	$(CC) $(CFLAGS) -O0 -mcpu=arm926ej-s -DPALMETTO $^ -o $@

# Keep the ELF for debugging
.PRECIOUS : cf-code/%.elf

clean:
	rm -rf cf-fsi-test-* *.o *.s *.bin
	rm -rf cf-code/*.elf cf-code/*.bin cf-code/*.s cf-code/*.o
