#ifndef MBC1_H
#define MBC1_H

#include "utils.h"

#define SIMPLE 0
#define ADVANCED 1

#define ROM_BANK_SIZE 0x4000
#define RAM_BANK_SIZE 0x2000

bool load_rom(const char *path);
u8 read_mbc1(u16 address);
void write_mbc1(u16 address, u8 value);

#endif
