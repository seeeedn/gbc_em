#ifndef MBC3_H
#define MBC3_H

#include "utils.h"
#include "cartridge.h"

u8 read_mbc3(Cartridge *cart, u16 address);
void write_mbc3(Cartridge *cart, u16 address, u8 value);

#endif
