# Main debug switch
DEBUG ?= 0

# ARM compiler for userspace test code
CROSS_COMPILE ?= arm-linux-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Os -Werror

# M68K toolchain for CF firmware
M68KCROSS ?= /opt/cross/binutils-coldfire/bin/m68k-unknown-elf-
M68KAS=$(M68KCROSS)as
M68KLD=$(M68KCROSS)ld
M68KOC=$(M68KCROSS)objcopy

ifeq ($(DEBUG),1)
M68KCPPFLAGS = -DENABLE_TRACE
endif
M68KAFLAGS = -march=isac
M68KLDFLAGS = -Ttext 0

# FW code files
TARGET_DEFS = $(wildcard cf-code/*.h)
TARGETS_s = $(patsubst %.h,%.s,$(TARGET_DEFS))
TARGETS_o = $(patsubst %.h,%.o,$(TARGET_DEFS))
TARGETS_elf = $(patsubst %.elf,%.o,$(TARGET_DEFS))
TARGETS_bin = $(patsubst %.bin,%.o,$(TARGET_DEFS))

all: $(TARGETS_bin) cf-fsi-test

cf-code/%.s : cf-code/%.h cf-code/cf-fsi-fw.S
	$(CC) -E $(M68KCPPFLAGS) -include $^ -o $@

cf-code/%.o : cf-code/%.s
	$(M68KAS) $(M68KAFLAGS) -march=isac $^ -o $@

cf-code/%.elf : cf-code/%.o
	$(M68KLD) $(M68KLDFLAGS) $^ -o $@

cf-code/%.bin : cf-code/%.elf
	$(M68KOC) -O binary $^ $@

cf-wrapper.o : cf-wrapper.S cf-code/cf-fsi-romulus.bin
	$(CC) $(CFLAGS) -c cf-wrapper.S -o $@

cf-fsi-test : cf-fsi-test.o cf-wrapper.o
	$(CC) $(CFLAGS) $^ -o $@

# Keep the ELF for debugging
.PRECIOUS : cf-code/%.elf

clean:
	rm -rf cf-fsi-test *.o
	rm -rf cf-code/*.elf cf-code/*.bin cf-code/*.s cf-code/*.o
