#ifndef MBC1_H
#define MBC1_H

#include "utils.h"
#include "cartridge.h"

#define SIMPLE      0
#define ADVANCED    1

u8 read_mbc1(Cartridge *cart, u16 address);
void write_mbc1(Cartridge *cart, u16 address, u8 value);

#endif
