#include "mbc1.h"

u8 read_mbc1(Cartridge *cart, u16 address) {
    if (address <= 0x3FFF) {
        u8 bank = (cart->mode == SIMPLE) ? 0 : (cart->ram_bank << 5);
        return cart->rom[ROM_BANK_SIZE * bank + address];
    }

    if (address <= 0x7FFF) {
        u8 bank = cart->rom_bank % cart->num_rom_banks;
        if ((bank & 0x1F) == 0) {
            bank++;
        }
        if (cart->num_rom_banks > 32) {
            bank = (cart->ram_bank << 5) | bank;
        }
        return cart->rom[ROM_BANK_SIZE * bank + (address - 0x4000)];    
    }

    if (address <= 0xBFFF) {
        if (cart->ram_enable) {
            u8 bank = (cart->mode == SIMPLE) ? 0 : cart->ram_bank;
            return cart->ram[RAM_BANK_SIZE * bank + (address - 0xA000)];
        }
    }

    return 0xFF;
}

void write_mbc1(Cartridge *cart, u16 address, u8 value) {
    if (address <= 0x1FFF) {
        cart->ram_enable = ((value & 0x0F) == 0x0A) ? true : false;
        return;
    }

    if (address <= 0x3FFF) {
        value &= 0x1F;
        if (value == 0) {
            value = 1;
        }
        cart->rom_bank = value;
        return;
    }

    if (address <= 0x5FFF) {
        cart->ram_bank = value & 0x03; 
        return;
    }

    if (address <= 0x7FFF) {
        cart->mode = value & 0x01;
        return;
    }

    if (address <= 0xBFFF && cart->ram_enable) {
        u8 bank = (cart->mode == SIMPLE) ? 0 : cart->ram_bank;
        cart->ram[RAM_BANK_SIZE * bank + (address - 0xA000)] = value;
    }
}
