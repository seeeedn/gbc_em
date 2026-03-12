#include "mbc3.h"

u8 last_latch_write;

void write_mbc3(Cartridge *cart, u16 address, u8 value) {
    if (address <= 0x1FFF) {
        cart->ram_enable = (value == 0x0A) ? true : false; 
        return;
    }

    if (address <= 0x3FFF) {
        value &= 0x7F;
        if (value == 0x00) {
            value = 1;
        }
        cart->rom_bank = value;
        return;
    }

    if (address <= 0x5FFF) {
        cart->ram_bank = value;
        return;
    }

    if (address <= 0x7FFF && cart->timer) {
        if (last_latch_write == 0x00 && value == 0x01) {
            time_t t = time(NULL);
            time_t delta_t = t - cart->rtc->last_update;

            cart->rtc->seconds = delta_t % 60;

            time_t min = t / 60;
            cart->rtc->minutes = min % 60;

            time_t hr = min / 60;
            cart->rtc->hours = hr % 24;

            time_t day = hr / 24;
            cart->rtc->day_counter_low = day % 31;

            if (day >= 256) {
                cart->rtc->day_counter_high |= 0x01; 
            }
            if (day > 511) {
                cart->rtc->day_counter_high |= 0x80;
            }
            
            cart->rtc->last_update = t;
        }
        last_latch_write = value;
        return;
    }

    if (address <= 0xBFFF && cart->ram_enable) {
        if (cart->ram_bank < cart->num_ram_banks) {
            cart->ram[RAM_BANK_SIZE * cart->ram_bank + (address - 0xA000)] = value;
            return;
        }
        switch (cart->ram_bank) {
            case 0x08:
                cart->rtc->seconds = value;
                break;

            case 0x09:
                cart->rtc->minutes = value;
                break;

            case 0x0A:
                cart->rtc->hours = value;
                break;

            case 0x0B:
                cart->rtc->day_counter_low = value;
                break;

            case 0x0C:
                cart->rtc->day_counter_high = value;
                break;

            default:
        }
    }
}

u8 read_mbc3(Cartridge *cart, u16 address) {
    if (address <= 0x3FFF) {
        return cart->rom[address];
    }

    if (address <= 0x7FFF) {
        u8 bank = cart->rom_bank % cart->num_rom_banks;
        return cart->rom[ROM_BANK_SIZE * bank + (address - 0x4000)];
    }

    if (address <= 0xBFFF && cart->ram_enable) {
        if (cart->ram_bank < cart->num_ram_banks) {
            return cart->ram[RAM_BANK_SIZE * cart->ram_bank + (address - 0xA000)];
        }
        switch (cart->ram_bank) {
            case 0x08:
                return cart->rtc->seconds;

            case 0x09:
                return cart->rtc->minutes;

            case 0x0A:
                return cart->rtc->hours;

            case 0x0B:
                return cart->rtc->day_counter_low;

            case 0x0C:
                return cart->rtc->day_counter_high;

            default:
        }
    }

    return 0xFF;
}
