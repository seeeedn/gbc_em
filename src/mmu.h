#ifndef MMU_H
#define MMU_H

#include "utils.h"

// PPU MODES
#define HBLANK          0x0
#define VBLANK          0x1
#define OAM_SCAN        0x2
#define DRAWING         0x3

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
#define UNUSED_END      0xFEFE

#define IO_REGS_START   0xFF00
#define IO_REGS_END     0xFF7F

#define HRAM_START      0xFF80
#define HRAM_END        0xFFFE

// FLAG/REG LOCATIONS
#define IE_ADDRESS      0xFFFF         // IE

// FLAG/REG LOCATIONS (relative to io_regs[])
#define IF_ADDRESS      0x000F         // IF
#define LCDC            0x0040         // LCD-Control
#define STAT            0x0041
#define SCY             0x0042
#define SCX             0x0043         // Scroll registers
#define LY              0x0044         // current scanline
#define LYC             0x0045
#define BGP             0x0047         // Background palette
#define OBP0            0x0048
#define OBP1            0x0049
#define WY              0x004A
#define WX              0x004B
#define VBK_INDEX       0x004F
#define WBK_INDEX       0x0070
#define SB              0x0001
#define SC              0x0002

// INTERRUPTS
#define INT_VBLANK      0x01
#define INT_STAT        0x02
#define INT_TIMER       0x04
#define INT_SERIAL      0x08
#define INT_JOYPAD      0x10

extern u8 *rom_banks;                  // Full ROM, dynamically loaded
extern u32 rom_size;
extern u8 current_rom_bank;
extern u8 current_ram_bank;
extern u8 banking_mode;

extern u8 current_vram_bank;           // 0 or 1
extern u8 current_wram_bank;           // 1–7 (bank 1 by default)
extern u8 vram[2][0x2000];             // 8 KB VRAM (GBC mode)
extern u8 ext_ram[0x2000];             // 8 KB external RAM (cart RAM)

extern u8 wram0[0x1000];               // 4 KB WRAM bank 0
extern u8 wram_switchable[7][0x1000];  // 4 KB WRAM switchable banks (GBC mode)

extern u8 oam[0xA0];                   // Sprite attribute table
extern u8 io_regs[0x80];               // IO registers (0xFF00-0xFF7F)
extern u8 hram[0x7F];                  // High RAM (0xFF80-0xFFFE)
extern u8 interrupts_enabled;          // IE register at 0xFFFF

void init_mmu();

void mmu_write_byte(u16 address, u8 value);
u8 mmu_read_byte(u16 address);

void mmu_write_word(u16 address, u16 value);
u16 mmu_read_word(u16 address);

void request_interrupt(u8 intr);
void enable_interrupt(u8 intr);

bool is_bit_set(u8 src, u8 bit);

#endif
