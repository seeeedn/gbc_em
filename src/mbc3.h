#ifndef MBC3_H
#define MBC3_H

#include <time.h>
#include "utils.h"

u8 read_mbc3(u16 address);
void write_mbc3(u16 address, u8 value);

#endif
