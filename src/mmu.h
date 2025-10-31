#ifndef MMU_H
#define MMU_H

#include "utils.h"

// MEMORY START/END LOCATIONS
#define BANK_0_START    0x0000
#define BANK_0_END      0x3FFF
#define BANK_N_START    0x4000
#define BANK_N_END      0x7FFF
#define VRAM_START      0x8000
#define VRAM_END        0x9FFF
#define EXT_RAM_START   0xA000
#define EXT_RAM_END     0xBFFF
#define WRAM_0_START    0xC000
#define WRAM_0_END      0xCFFF
#define WRAM_S_START    0xD000
#define WRAM_S_END      0xDFFF
#define ECHO_START      0xE000
#define ECHO_END        0xFDFF
#define OAM_START       0xFE00
#define OAM_END         0xFE9F
#define UNUSED_START    0xFEA0
#define UNUSED_END      0xFEFF
#define IO_REGS_START   0xFF00
#define IO_REGS_END     0xFF7F
#define HRAM_START      0xFF80
#define HRAM_END        0xFFFE

// MEMORY SIZES
#define VRAM_SIZE       0x2000
#define EXT_RAM_SIZE    0x2000
#define WRAM_SIZE       0x1000
#define OAM_SIZE        0xA0
#define IO_SIZE         0x80
#define HRAM_SIZE       0x7F

// FLAG/REG LOCATIONS
#define IE_ADDRESS      0xFFFF              // IE

extern u8 *rom_banks;                       // Full ROM, dynamically loaded
extern u32 rom_size;
extern u8 current_rom_bank;
extern u8 current_ram_bank;
extern u8 banking_mode;

extern u8 current_vram_bank;                // 0 or 1
extern u8 current_wram_bank;                // 1–7 (bank 1 by default)
extern u8 vram[2][VRAM_SIZE];               // 8 KB VRAM (GBC mode)
extern u8 ext_ram[EXT_RAM_SIZE];            // 8 KB external RAM (cart RAM)

extern u8 wram0[WRAM_SIZE];                 // 4 KB WRAM bank 0
extern u8 wram_switchable[7][WRAM_SIZE];    // 4 KB WRAM switchable banks (GBC mode)

extern u8 oam[OAM_SIZE];                    // Sprite attribute table
extern u8 io_regs[IO_SIZE];                 // IO registers (0xFF00-0xFF7F)
extern u8 hram[HRAM_SIZE];                  // High RAM (0xFF80-0xFFFE)
extern u8 interrupts_enabled;               // IE register at 0xFFFF

extern bool was_dma;

void init_mmu();

void mmu_write_byte(u16 address, u8 value);
u8 mmu_read_byte(u16 address);

void mmu_write_word(u16 address, u16 value);
u16 mmu_read_word(u16 address);

void oam_dma(u8 address);

void request_interrupt(u8 intr);
void enable_interrupt(u8 intr);

bool is_bit_set(u8 src, u8 bit);

#endif
