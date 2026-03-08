#ifndef MBC3_H
#define MBC3_H

#include <time.h>
#include "utils.h"

#define ROM_BANK_SIZE 0x4000
#define RAM_BANK_SIZE 0x2000

bool load_rom(const char *path);
u8 read_mbc3(u16 address);
void write_mbc3(u16 address, u8 value);

#endif
