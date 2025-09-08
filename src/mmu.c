#include "mmu.h"
#include <stdio.h>

u8 *rom_banks = 0;
u32 rom_size = 0;
u8 current_rom_bank = 0;
u8 current_ram_bank = 0;
u8 banking_mode = 0;

u8 current_vram_bank = 0;
u8 current_wram_bank = 0;

u8 vram[2][0x2000] = {0};
u8 ext_ram[0x2000] = {0};

u8 wram0[0x1000] = {0};
u8 wram_switchable[7][0x1000] = {0};

u8 oam[0xA0] = {0};
u8 io_regs[0x80] = {0};
u8 hram[0x7F] = {0};
u8 interrupts_enabled = 0;

void init_mmu() {
    io_regs[IF_ADDRESS] = 0xE1;
    interrupts_enabled = 0x00;

    io_regs[LCDC] = 0x91;
    io_regs[STAT] = 0x85;
    io_regs[SCY] = 0x00;
    io_regs[SCX] = 0x00;
    io_regs[LY] = 0x00;
    io_regs[LYC] = 0x00;
    io_regs[BGP] = 0xFC;
    io_regs[OBP0] = 0xFF;
    io_regs[OBP1] = 0xFF;
    io_regs[WY] = 0x00;
    io_regs[WX] = 0x00;

    current_rom_bank = 1;   // Bank 1 by default
    current_ram_bank = 0;
    current_vram_bank = 0;
    current_wram_bank = 1;  // Bank 1 by default
}

void mmu_write_byte(u16 address, u8 value) {

    if (address == 0xFF02 && value == 0x81) {
        printf("%c", io_regs[01]);
        fflush(stdout);  // ensures output appears immediately
    }

    if (address == 0xFF46)
        printf("DMA\n");

    current_vram_bank = io_regs[VBK_INDEX] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX] & 0x7;

    u8 current_ppu_mode = io_regs[STAT] & 0x03;

    if (current_wram_bank == 0)
        current_wram_bank = 1;
    if (current_rom_bank == 0)
        current_rom_bank = 1;

    current_rom_bank = 1;

    if (address >= BANK_0_START && address <= BANK_0_END) {
        // TODO: ROM
    }
    else if (address >= BANK_N_START && address <= BANK_N_END) {
        // TODO: ROM
    }
    else if (address >= VRAM_START && address <= VRAM_END) {
        if (current_ppu_mode == DRAWING) {
            return;             // writes are disabled
        } else {
            vram[0][address - VRAM_START] = value;
        }
    }
    else if (address >= EXT_RAM_START && address <= EXT_RAM_END) {
        ext_ram[address - EXT_RAM_START] = value;
    }
    else if (address >= WRAM_0_START && address <= WRAM_0_END) {
        wram0[address - WRAM_0_START] = value;
    }
    else if (address >= WRAM_S_START && address <= WRAM_S_END) {
        wram_switchable[1][address - WRAM_S_START] = value;
    }
    else if (address >= ECHO_START && address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        mmu_write_byte(wram_address, value);
    }
    else if (address >= OAM_START && address <= OAM_END) {
        if (current_ppu_mode == OAM_SCAN || current_ppu_mode == DRAWING) {
            return;
        } else {
            oam[address - OAM_START] = value;
        }
    }
    else if (address >= UNUSED_START && address <= UNUSED_END) {
        return;                 // writes are disabled
    }
    else if (address >= IO_REGS_START && address <= IO_REGS_END) {
        io_regs[address - IO_REGS_START] = value;
        if (address == 0xFF46) {
            oam_dma(value);
        }
    }
    else if (address >= HRAM_START && address <= HRAM_END) {
        hram[address - HRAM_START] = value;
    }
    else if (address == IE_ADDRESS) {
        interrupts_enabled = value;
    }
    else {
        return;     // Error handling
    }
}

u8 mmu_read_byte(u16 address) {
    current_vram_bank = io_regs[VBK_INDEX] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX] & 0x7;

    u8 current_ppu_mode = io_regs[STAT] & 0x03;

    if (current_wram_bank == 0)
        current_wram_bank = 1;
    if (current_rom_bank == 0)
        current_rom_bank = 1;

    current_rom_bank = 1;

    u8 value = 0;

    if (address >= BANK_0_START && address <= BANK_N_END) {
        value = rom_banks[address];
    }
   /*  else if (address >= BANK_N_START && address <= BANK_N_END) {
        value = rom_banks[address];
        }*/
    else if (address >= VRAM_START && address <= VRAM_END) {
        if (current_ppu_mode == DRAWING) {
            value = 0xFF;       // VRAM inaccessible in this mode (read return garbage value)
        } else {
            value = vram[0][address - VRAM_START];
        }
    }
    else if (address >= EXT_RAM_START && address <= EXT_RAM_END) {
        value = ext_ram[address - EXT_RAM_START];
    }
    else if (address >= WRAM_0_START && address <= WRAM_0_END) {
        value = wram0[address - WRAM_0_START];
    }
    else if (address >= WRAM_S_START && address <= WRAM_S_END) {
        value = wram_switchable[1][address - WRAM_S_START];
    }
    else if (address >= ECHO_START && address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        value = mmu_read_byte(wram_address);
    }
    else if (address >= OAM_START && address <= OAM_END) {
        if (current_ppu_mode == OAM_SCAN || current_ppu_mode == DRAWING) {
            value = 0xFF;       // OAM inaccessible in these modes (read return garbage value)
        } else {
            value = oam[address - OAM_START];
        }
    }
    else if (address >= UNUSED_START && address <= UNUSED_END) {
        u8 addr_nibble = address & 0x00F0;
        value = addr_nibble + (addr_nibble >> 4);       // reads here return upper nibble of lower address-byte twice (e.g 0xAA for addr. 0xFFAx)
    }
    else if (address >= IO_REGS_START && address <= IO_REGS_END) {
        value = io_regs[address - IO_REGS_START];
    }
    else if (address >= HRAM_START && address <= HRAM_END) {
        value = hram[address - HRAM_START];
    }
    else if (address == IE_ADDRESS) {
        value = interrupts_enabled;
    }
    else {
        // Error handling
    }
    return value;
}

void mmu_write_word(u16 address, u16 value) {
    mmu_write_byte(address, value & 0x00FF);
    mmu_write_byte(address + 1, (value >> 8) & 0x00FF);
}

u16 mmu_read_word(u16 address) {
    u16 value = 0;
    value |= mmu_read_byte(address);
    value |= (mmu_read_byte(address + 1) << 8);
    return value;
}

void oam_dma(u8 address) {
    u16 base_addr = (address << 8);
    for (int i = 0; i < 0xA0; i++) {
        oam[i] = mmu_read_byte(base_addr);
    }
}

void request_interrupt(u8 intr) {
    io_regs[IF_ADDRESS] |= intr;
}

void enable_interrupt(u8 intr) {
    interrupts_enabled |= intr;
}

bool is_bit_set(u8 src, u8 bit) {
    bool ret = (src & (1 << bit)) ? true : false;
    return ret;
}
