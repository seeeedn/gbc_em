#include "mmu.h"

u8* rom_banks = 0;
u32 rom_size = 0;
u8 current_rom_bank = 0;

u8 current_vram_bank = 0;
u8 current_wram_bank = 0;

u8 vram[2][0x2000] = {0};
u8 ext_ram[0x2000] = {0};

u8 wram0[0x1000] = {0};
u8 wram_switchable[7][0x1000] = {0};

u8 oam[0xA0] = {0};
u8 io_regs[0x80] = {0};
u8 hram[0x7F] = {0};
u8 interrupt_enable = 0;

void mmu_write_byte(u16 address, u8 value) {
    current_vram_bank = io_regs[VBK_INDEX - IO_REGS_START] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX - IO_REGS_START] & 0x7;
    if (current_wram_bank == 0)
        current_wram_bank = 1;

    if (address >= BANK_0_START && address <= BANK_0_END) {
        // TODO: ROM
    }
    else if (address >= BANK_N_START && address <= BANK_N_END) {
        // TODO: ROM
    }
    else if (address >= VRAM_START && address <= VRAM_END) {
        vram[current_vram_bank][address - VRAM_START] = value;
    }
    else if (address >= EXT_RAM_START && address <= EXT_RAM_END) {
        ext_ram[address - EXT_RAM_START] = value;
    }
    else if (address >= WRAM_0_START && address <= WRAM_0_END) {
        wram0[address - WRAM_0_START] = value;
    }
    else if (address >= WRAM_S_START && address <= WRAM_S_END) {
        wram_switchable[current_wram_bank][address - WRAM_S_START] = value;
    }
    else if (address >= ECHO_START && address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        mmu_write_byte(wram_address, value);
    }
    else if (address >= OAM_START && address <= OAM_END) {
        oam[address - OAM_START] = value;
    }
    else if (address >= UNUSED_START && address <= UNUSED_END) {
        return;         // ignore writes to this area
    }
    else if (address >= IO_REGS_START && address <= IO_REGS_END) {
        io_regs[address - IO_REGS_START] = value;
    }
    else if (address >= HRAM_START && address <= HRAM_END) {
        hram[address - HRAM_START] = value;
    }
    else if (address == IE_ADDRESS) {
        interrupt_enable = value;
    }
    else {
        // Error handling
        return;
    }
}

u8 mmu_read_byte(u16 address) {
    current_vram_bank = io_regs[VBK_INDEX - IO_REGS_START] & 0x1;
    current_wram_bank = io_regs[WBK_INDEX - IO_REGS_START] & 0x7;
    if (current_wram_bank == 0)
        current_wram_bank = 1;
    u8 value = 0;

    if (address >= BANK_0_START && address <= BANK_0_END) {
        // TODO: ROM
    }
    else if (address >= BANK_N_START && address <= BANK_N_END) {
        // TODO: ROM
    }
    else if (address >= VRAM_START && address <= VRAM_END) {
        value = vram[current_vram_bank][address - VRAM_START];
    }
    else if (address >= EXT_RAM_START && address <= EXT_RAM_END) {
        value = ext_ram[address - EXT_RAM_START];
    }
    else if (address >= WRAM_0_START && address <= WRAM_0_END) {
        value = wram0[address - WRAM_0_START];
    }
    else if (address >= WRAM_S_START && address <= WRAM_S_END) {
        value = wram_switchable[current_wram_bank][address - WRAM_S_START];
    }
    else if (address >= ECHO_START && address <= ECHO_END) {
        u16 wram_address = address - 0x2000;
        value = mmu_read_byte(wram_address);
    }
    else if (address >= OAM_START && address <= OAM_END) {
        value = oam[address - OAM_START];
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
        value = interrupt_enable;
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
