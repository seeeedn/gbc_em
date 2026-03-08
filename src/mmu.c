#include "mmu.h"
#include "input.h"
#include "timer.h"
#include "mbc3.h"
//#include "mbc1.h"

/*u8 *rom_banks;
u32 rom_size;
u8 current_rom_bank;
u8 current_ram_bank;
u8 banking_mode;
*/
u8 current_vram_bank;
u8 current_wram_bank;

u8 vram[2][0x2000];
u8 ext_ram[0x2000] = {0xFF};

u8 wram0[0x1000];
u8 wram_switchable[7][0x1000];

u8 oam[0xA0];
u8 io_regs[0x80];
u8 hram[0x7F];
u8 interrupts_enabled;

bool was_dma = false;

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
    io_regs[TAC] = 0xF8;
    io_regs[DMA] = 0xFF;

    current_vram_bank = 0;
    current_wram_bank = 1;  // Bank 1 by default
}

static bool valid_access(u16 address) {
    u8 current_ppu_mode = read_mmio(STAT) & 0x03;

    if (address >= VRAM_START && address <= VRAM_END) {
        if (current_ppu_mode == DRAWING) {
            return false;
        }
    }

    if (address >= OAM_START && address <= OAM_END) {
        if (current_ppu_mode == OAM_SCAN || current_ppu_mode == DRAWING) {
            return false;
        }
    }

    if (address >= UNUSED_START && address <= UNUSED_END) {
        return false;
    }

    return true;
}

void mmu_write_byte(u16 address, u8 value) {
    current_vram_bank = io_regs[VBK_INDEX] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX] & 0x7;

    if (!valid_access(address)) {
        return;
    }

    if (current_wram_bank == 0) {
        current_wram_bank = 1;
    }

    if (address <= BANK_N_END) {
        write_mbc3(address, value);
    }

    else if (address <= VRAM_END) {
        vram[0][address - VRAM_START] = value;
    }

    else if (address <= EXT_RAM_END) {
        ext_ram[address - EXT_RAM_START] = value;
    }

    else if (address <= WRAM_0_END) {
        wram0[address - WRAM_0_START] = value;
    }

    else if (address <= WRAM_S_END) {
        wram_switchable[1][address - WRAM_S_START] = value;
    }

    else if (address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        mmu_write_byte(wram_address, value);
    }

    else if (address <= OAM_END) {
        oam[address - OAM_START] = value;
    }

    else if (address <= IO_REGS_END) {
        io_regs[address - IO_REGS_START] = value;

        if (address == 0xFF00) {
            io_regs[JOYPAD] = (value & 0x30) | (io_regs[JOYPAD] & 0xCF);
        }

        if (address == 0xFF02 && value == 0x81) {
            //printf("%c", io_regs[SB]);
            //fflush(stdout);
        }

        if (address == 0xFF04) {    // writes to DIV reset the register to 0x00
            div_counter = 0;
            io_regs[DIV] = 0x00;
        }

        if (address == 0xFF07) {
            io_regs[TIMA]++;
        }

        if (address == 0xFF46) {
            oam_dma(value);
        }
    }

    else if (address <= HRAM_END) {
        hram[address - HRAM_START] = value;
    }

    else {
        interrupts_enabled = value;
    }
}

u8 mmu_read_byte(u16 address) {
    current_vram_bank = io_regs[VBK_INDEX] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX] & 0x7;

    if (current_wram_bank == 0)
        current_wram_bank = 1;

    if (!valid_access(address)) {
        return 0xFF;
    }

    u8 value = 0;

    if (address <= BANK_N_END) {
        value = read_mbc3(address);
    }

    else if (address <= VRAM_END) {
        value = vram[0][address - VRAM_START];
    }

    else if (address <= EXT_RAM_END) {
        value = ext_ram[address - EXT_RAM_START];
    }

    else if (address <= WRAM_0_END) {
        value = wram0[address - WRAM_0_START];
    }

    else if (address <= WRAM_S_END) {
        value = wram_switchable[1][address - WRAM_S_START];
    }

    else if (address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        value = mmu_read_byte(wram_address);
    }

    else if (address <= OAM_END) {
        value = oam[address - OAM_START];
    }

    else if (address <= UNUSED_END) {
        u8 addr_nibble = address & 0x00F0;
        value = addr_nibble + (addr_nibble >> 4);       // reads here return upper nibble of lower address-byte twice (e.g 0xAA for addr. 0xFFAx)
    }

    else if (address <= IO_REGS_END) {
        value = io_regs[address - IO_REGS_START];

        if (address == 0xFF00) {
            value = handle_input();
        }
    }

    else if (address <= HRAM_END) {
        value = hram[address - HRAM_START];
    }

    else {
        value = interrupts_enabled;
    }

    return value;
}

void write_mmio(u8 address, u8 val) {
    u16 abs_addr = 0xFF00 | address;

    if (abs_addr <= IO_REGS_END) {
        io_regs[address] = val;
    } else if (abs_addr == IE_ADDRESS) {
        interrupts_enabled = val;
    } else {
        fprintf(stderr, "Error: Invalid memory write access at address: 0x%x04\n", abs_addr);
        exit(EXIT_FAILURE);
    }
}

u8 read_mmio(u8 address) {
    u8 val = 0;
    u16 abs_addr = 0xFF00 | address;

    if (abs_addr <= IO_REGS_END) {
        val = io_regs[address];
    } else if (abs_addr == IE_ADDRESS) {
        val = interrupts_enabled;
    } else {
        fprintf(stderr, "Error: Invalid memory read access at address: 0x%x04\n", abs_addr);
        exit(EXIT_FAILURE);
    }

    return val;
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
    for (int i = 0; i < OAM_SIZE; i++) {
        oam[i] = mmu_read_byte(base_addr + i);
    }
    was_dma = true;
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
