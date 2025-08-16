CC := gcc

TARGET := gbc

SRC :=  src/main.c \
		src/cpu.c \
	 	src/mmu.c

all:
	$(CC) -Iinclude -Llib -o $(TARGET) $(SRC) -IC:/SDL2/include -LC:/SDL2/lib -lmingw32 -lSDL2main -lSDL2
