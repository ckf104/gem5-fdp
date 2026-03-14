CC = riscv64-unknown-linux-gnu-gcc
CFLAGS ?= -O2 -Wall -Wextra -static

OUTDIR ?= ../bin/riscv/linux
TARGET := $(OUTDIR)/renameat2

.PHONY: all clean

all: $(TARGET)

$(TARGET): renameat2.c
	mkdir -p $(OUTDIR)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)
