#include "ppu.h"
#include "mmu.h"

#define SCREEN_HEIGHT   144
#define SCREEN_WIDTH    160

u32 framebuffer[SCREEN_HEIGHT * SCREEN_WIDTH] = {0};
u16 ppu_cycles = 0;
bool ppu_frame_ready = false;

void ppu_step(u8 cycles) {
    ppu_cycles += cycles;
    u8 ppu_mode = (io_regs[STAT] & 0x3);

    if (!(io_regs[LCDC] & 0x80)) {          // LCD is disabled



    }

    switch (ppu_mode) {
        case OAM_SCAN: {


            break;
        }

        case DRAWING: {


            break;
        }

        case HBLANK: {


            break;
        }

        case VBLANK: {


            break;
        }
    }

    if (ppu_cycles >= 456) {
        ppu_cycles -= 456;
        io_regs[LY] += 1;

        if (io_regs[LY] == 144) {
            io_regs[STAT] = (io_regs[STAT] & ~0x3) | VBLANK;
            ppu_frame_ready = true;
        } else if (io_regs[LY] > 153) {
            io_regs[LY] = 0;
        }
    }

    if (io_regs[LY] < 144) {
        ppu_draw_line();
    }

    // Set STAT mode based on LY and cycles
    if (io_regs[LY] >= 144) {
        io_regs[STAT] = (io_regs[STAT] & ~0x3) | VBLANK;
    } else {
        if (ppu_cycles < 80) {
            io_regs[STAT] = (io_regs[STAT] & ~0x3) | OAM_SCAN;
        } else if (ppu_cycles < 252) {
            io_regs[STAT] = (io_regs[STAT] & ~0x3) | DRAWING;
        } else {
            io_regs[STAT] = (io_regs[STAT] & ~0x3) | HBLANK;
        }
    }
}

void ppu_draw_line() {
    int y = io_regs[LY];
    if (y >= SCREEN_HEIGHT) return;

    u8 scy = io_regs[SCY];
    u8 scx = io_regs[SCX];
    u8 lcdc = io_regs[LCDC];

    if (!(lcdc & 0x01)) {
        memset(&framebuffer[y * SCREEN_WIDTH], 0xFF, SCREEN_WIDTH * sizeof(u32));
        return;
    }

    u16 tile_data_base = (lcdc & 0x10) ? 0x8000 : 0x8800;
    bool signed_index = !(lcdc & 0x10);
    u16 tilemap_base = (lcdc & 0x08) ? 0x9C00 : 0x9800;

    int bg_y = (y + scy) & 0xFF;
    int tile_row = (bg_y / 8) * 32;
    int line_in_tile = bg_y % 8;

    u32 colors[4] = {0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000};
    u8 bgp = io_regs[BGP];  // Background palette register (0xFF47)

    u8 *v = vram[0];

    for (int tile_x = 0; tile_x < 20; tile_x++) {
        int bg_x = tile_x * 8 + scx;
        int tile_col = (bg_x / 8) & 0x1F;

        u8 tile_num = v[tilemap_base - VRAM_START + tile_row + tile_col];
        if (signed_index) tile_num = (i8)tile_num + 128;

        u16 tile_addr = tile_data_base - 0x8000 + tile_num * 16;
        u8 low  = v[tile_addr + line_in_tile * 2];
        u8 high = v[tile_addr + line_in_tile * 2 + 1];

        // Decode the 8 pixels in this tile row
        for (int bit = 0; bit < 8; bit++) {
            int x = (tile_x * 8 + bit - scx) & 0xFF;
            if (x >= 0 && x < SCREEN_WIDTH) {
                int color_id = (((high >> (7 - bit)) & 1) << 1) | ((low >> (7 - bit)) & 1);
                int shade = (bgp >> (color_id * 2)) & 0x03; // Remap pixel value 0–3
                framebuffer[y * SCREEN_WIDTH + x] = colors[shade];
            }
        }
    }
}
