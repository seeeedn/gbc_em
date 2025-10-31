CC := gcc

TARGET := gbc

SRC :=  src/main.c \
		src/cpu.c \
	 	src/mmu.c \
		src/ppu.c \
		src/input.c \
		src/timer.c

all:
	$(CC) -Wall -Wextra -o $(TARGET) $(SRC) -lSDL2main -lSDL2
