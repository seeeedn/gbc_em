#include <string.h>
#include "cartridge.h"
#include "mbc1.h"
#include "mbc3.h"

RTC rtc;
Cartridge cart;
char rom_title[17] = {0};

u8 ram_size[6] = { 0, 1, 1, 4, 16, 8 };     // Lookup table for number of RAM banks
bool battery = false;                       // Determines if Cartridge has battery-backed SRAM
bool timer = false;

bool load_rom(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }

    fseek(f, 0, SEEK_END);
    cart.rom_size = ftell(f);
    rewind(f);

    cart.rom = malloc(cart.rom_size);
    if (!cart.rom) {
        fclose(f);
        return false;
    }

    fread(cart.rom, 1, cart.rom_size, f);
    fclose(f);

    cart.num_rom_banks = cart.rom_size / ROM_BANK_SIZE;
    cart.num_ram_banks = ram_size[cart.rom[0x0149]];

    size_t ram_bytes = cart.num_ram_banks * RAM_BANK_SIZE;
    if (ram_bytes) {
        cart.ram = calloc(1, ram_bytes);
    } else {
        cart.ram = NULL;
    }

    memcpy(rom_title, &cart.rom[0x0134], 16);
    rom_title[16] = '\0';

    u8 type = cart.rom[0x0147];
    switch (type) {
        case 0x00:
            cart.mbc_type = NO_MBC;
            break;

        case 0x01 ... 0x03:
            cart.mbc_type = MBC1;
            if (type == 0x03) {
                cart.battery = true;
            }
            break;

        case 0x0F ... 0x13:
            cart.mbc_type = MBC3;
            if (type == 0x0F || type == 0x10) {
                cart.timer = true;
            }
            if (type == 0x0F || type == 0x10 || type == 0x13) {
                cart.battery = true;
            }
            break;

        default:
            printf("Unsupported MBC type %02X\n", type);
    }

    if (cart.timer) {
        memset(&rtc, 0, sizeof(RTC));
        rtc.last_update = time(NULL); 
        cart.rtc = &rtc;
    } else {
        cart.rtc = NULL;
    }

    if (cart.battery) {
        load_ram(rom_title);
    }

    return true;
}

char* make_save_name(const char *rom_path) {
    const char *savedir = "../savefiles/";
    char *savefile = malloc(strlen(savedir) + strlen(rom_path) + 5);

    strcpy(savefile, savedir);
    strcpy(savefile + strlen(savedir), rom_path);

    char *dot = strrchr(savefile, '.');
    if (dot) {
        strcpy(dot, ".sav");
    } else {
        strcat(savefile, ".sav");
    }

    return savefile;
}

bool load_ram(const char *path) {
    if (!cart.battery) {
        return false;
    }
    
    char *savefile = make_save_name(path);
    FILE *save = fopen(savefile, "rb");
    free(savefile);

    if (!save) {
        return false;
    }

    size_t ram_size = cart.num_ram_banks * RAM_BANK_SIZE;

    fread(cart.ram, 1, ram_size, save);
    if (timer && cart.rtc) {
        fread(cart.rtc, sizeof(RTC), 1, save);
    }

    fclose(save);
    return true;
}

bool save_ram(const char *path) {
    if (!cart.battery) {
        return false;
    }

    char *savefile = make_save_name(path);
    FILE *save = fopen(savefile, "wb");
    free(savefile);
    
    if (!save) {
        return false;
    }

    size_t ram_size = cart.num_ram_banks * RAM_BANK_SIZE;
    fwrite(cart.ram, 1, ram_size, save);

    if (timer && cart.rtc) {
        fwrite(cart.rtc, sizeof(RTC), 1, save);
    }

    fclose(save);
    return true;
}

u8 read_cart(u16 address) {
    switch (cart.mbc_type) {
        case NO_MBC:
            return cart.rom[address];

        case MBC1:
            return read_mbc1(&cart, address);

        case MBC3:
            return read_mbc3(&cart, address);
        
        default:
            printf("Err: Unknown MBC type\n");
            return 0xFF;
    }
}

void write_cart(u16 address, u8 value) {
    switch (cart.mbc_type) {
        case NO_MBC:
            return;

        case MBC1:
            write_mbc1(&cart, address, value);
            if (cart.battery) {
                save_ram(rom_title);
            }
            return;

        case MBC3:
            write_mbc3(&cart, address, value);
            if (cart.battery) {
                save_ram(rom_title);
            }
            return;

        default:
            printf("Err: Unknown MBC type\n");
            return;
    }
}

