#include "mbc3.h"

//u8 rom[0x4000 * 0x80] = {0};
u8 *rom = NULL;
u8 ram_bank_n[0x2000 * 8] = {0};        // RAM Bank 00-07
u8 rtc_register[5];                     // RTC register

bool ram_rtc_enable = false;
u8 rom_idx = 1;                         // Current ROM Bank
u8 ram_idx = 0;                         // Current RAM Bank or RTC register
u8 last_latch_write = 0;

int rom_size;
u16 num_rom_banks = 0;
u16 num_ram_banks = 3;

struct tm *ptime;
time_t t;


bool load_rom(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) 
        return false;

    fseek(f, 0, SEEK_END);
    rom_size = ftell(f);
    rewind(f);

    rom = malloc(rom_size);
    if (!rom) {
        fclose(f);
        return false;
    }

    fread(rom, 1, rom_size, f);
    fclose(f);

    num_rom_banks = rom_size / ROM_BANK_SIZE;

    return true;
}

void write_mbc3(u16 address, u8 value) {
    if (address <= 0x1FFF) {
        ram_rtc_enable = (value == 0x0A) ? true : false; 
        return;
    }

    if (address <= 0x3FFF) {
        value &= 0x7F;
        if (value == 0x00) {
            value = 1;
        }
        rom_idx = value;
        return;
    }

    if (address <= 0x5FFF) {
        ram_idx = value;
        return;
    }

    if (address <= 0x7FFF) {
        if (last_latch_write == 0x00 && value == 0x01) {
            t = time(NULL);
            ptime = localtime(&t);

            rtc_register[0] = ptime->tm_sec;
            rtc_register[1] = ptime->tm_min;
            rtc_register[2] = ptime->tm_hour;

            int days = ptime->tm_yday;
            rtc_register[3] = days & 0xFF;
            rtc_register[4] = (days >> 8) & 0x01;

            if (days & 0x100)
                rtc_register[4] |= 0x01;

            if (days > 511)
                rtc_register[4] |= 0x80;
        }

        last_latch_write = value;
        return;
    }

    if (address <= 0xBFFF && ram_rtc_enable) {
        if (ram_idx <= num_ram_banks) {
            ram_bank_n[RAM_BANK_SIZE * ram_idx + (address - 0xA000)] = value;
            return;
        }
        if (ram_idx >= 0x08 && ram_idx <= 0x0C) {
            rtc_register[ram_idx - 0x08] = value;
            return;
        }
    }
}

u8 read_mbc3(u16 address) {
    if (address <= 0x3FFF) {
        return rom[address];
    }

    if (address <= 0x7FFF) {
        u8 bank = rom_idx % num_rom_banks;
        return rom[ROM_BANK_SIZE * bank + (address - 0x4000)];
    }

    if (address <= 0xBFFF && ram_rtc_enable) {
        if (ram_idx <= num_ram_banks) {
            return ram_bank_n[RAM_BANK_SIZE * ram_idx + (address - 0xA000)];
        }
        if (ram_idx >= 0x08 && ram_idx <= 0x0C) {
            return rtc_register[ram_idx - 0x08];
        }
    }

    return 0xFF;
}
