M68KCROSS ?= /opt/cross/binutils-coldfire/bin/m68k-unknown-elf-
M68KAS=$(M68KCROSS)as
M68KLD=$(M68KCROSS)ld
M68KOC=$(M68KCROSS)objcopy

CC = $(CROSS_COMPILE)gcc
CFLAGS = -Wall -Os

all: cf-fsi-test

cf-code.o : cf-code.s
	$(M68KAS) -march=isac $^ -o $@

cf-code.elf : cf-code.o
	$(M68KLD) -Ttext 0 $^ -o $@

cf-code.bin : cf-code.elf
	$(M68KOC) -O binary $^ $@

cf-wrapper.o : cf-wrapper.S cf-code.bin
	$(CC) $(CFLAGS) -c cf-wrapper.S -o $@

cf-fsi-test : cf-fsi-test.o cf-wrapper.o
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf cf-fsi-test *.o *.elf *.bin
