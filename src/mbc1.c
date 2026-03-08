#include "mbc1.h"

u8 *rom;
u8 ram[RAM_BANK_SIZE * 4] = {0};

u8 ram_bank = 1;
u8 rom_bank = 1;
bool ram_enable = false;
bool mode = SIMPLE;

u8 num_rom_banks = 0;
size_t rom_size;

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

u8 read_mbc1(u16 address) {
    if (address <= 0x3FFF) {
        u8 bank = (mode == SIMPLE) ? 0 : (ram_bank << 5);
        return rom[ROM_BANK_SIZE * bank + address];
    }

    if (address <= 0x7FFF) {
        u8 bank = rom_bank % num_rom_banks;
        if ((bank & 0x1F) == 0) {
            bank++;
        }
        if (num_rom_banks > 32) {
            bank = (ram_bank << 5) | bank;
        }
        return rom[ROM_BANK_SIZE * bank + (address - 0x4000)];    
    }

    if (address <= 0xBFFF) {
        if (ram_enable) {
            u8 bank = (mode == SIMPLE) ? 0 : ram_bank;
            return ram[RAM_BANK_SIZE * bank + (address - 0xA000)];
        }
    }

    return 0xFF;
}

void write_mbc1(u16 address, u8 value) {
    if (address <= 0x1FFF) {
        if ((value & 0x0F) == 0x0A) {
            ram_enable = true;
        } else {
            ram_enable = false;
        }
        return;
    }

    if (address <= 0x3FFF) {
        value &= 0x1F;
        if (value == 0) {
            value = 1;
        }
        rom_bank = value;
        return;
    }

    if (address <= 0x5FFF) {
        ram_bank = value & 0x03; 
        return;
    }

    if (address <= 0x7FFF) {
        mode = value & 0x01;
        return;
    }

    if (address <= 0xBFFF && ram_enable) {
        u8 bank = (mode == SIMPLE) ? 0 : ram_bank;
        ram[RAM_BANK_SIZE * bank + (address - 0xA000)] = value;
    }
}
