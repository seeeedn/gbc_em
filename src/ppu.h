#ifndef PPU_H
#define PPU_H

#include "utils.h"
#include "mmu.h"

#define OAM_CYCLES 80
#define VRAM_CYCLES 172
#define HBLANK_CYCLES 204
#define SCANLINE_CYCLES 456
#define SCREEN_VBLANK_HEIGHT 153

extern u32 framebuffer[160 * 144];
extern bool ppu_frame_ready;

void ppu_step(u8 cycles);

#endif
