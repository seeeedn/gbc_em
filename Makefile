CC := gcc

TARGET := gbc

SRC :=  src/main.c \
		src/cpu.c \
	 	src/mmu.c \
		src/ppu.c \
		src/input.c \
		src/timer.c \
		src/interrupt.c \
		src/debug.c \
		src/mbc1.c \
		src/mbc3.c \
		src/cartridge.c

all:
	$(CC) -g -Wall -Wextra -o $(TARGET) $(SRC) -lSDL3
