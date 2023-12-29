.PHONY: all
all: upcase.bin hello.bin add.bin

.SUFFIXES: .bin .asm
.asm.bin:
	z80asm -o $@ $<
