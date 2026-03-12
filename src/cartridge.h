#ifndef CARTRIDGE_H
#define CARTRIDGE_H

#include "utils.h"
#include <time.h>

#define ROM_BANK_SIZE 0x4000
#define RAM_BANK_SIZE 0x2000

#define NO_MBC  0
#define MBC1    1
#define MBC3    3

typedef struct {
    u8 seconds;
    u8 minutes;
    u8 hours;
    u8 day_counter_low;
    u8 day_counter_high;
    time_t last_update;
} RTC;

typedef struct {
    u8 *rom;
    u8 *ram;
    RTC *rtc;
    size_t rom_size;
    bool ram_enable;
    bool mode;
    bool battery;
    bool timer;
    u8 mbc_type;
    u8 rom_bank;
    u8 ram_bank;
    u8 num_rom_banks;
    u8 num_ram_banks;
} Cartridge;

bool load_rom(const char *path);
bool load_ram(const char *path);
bool save_ram(const char *path);

u8 read_cart(u16 address);
void write_cart(u16 address, u8 value);

#endif
