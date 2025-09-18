#include "ppu.h"
#include "mmu.h"

#define SCREEN_HEIGHT   144
#define SCREEN_WIDTH    160
#define SCANLINE_MAX_SPRITES      10

const u32 colors[4] = {0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000};       // grey-scale
u32 framebuffer[SCREEN_HEIGHT * SCREEN_WIDTH] = {0};
u16 ppu_cycles = 0;
bool ppu_frame_ready = false;

static void ppu_draw_scanline();
static void draw_obj();
static void draw_bg();

void ppu_step(u8 cycles) {
    ppu_cycles += cycles;
    u8 ppu_mode = (io_regs[STAT] & 0x3);

    if (!is_bit_set(io_regs[LCDC], 7)) {    // LCD is disabled
        ppu_cycles = 0;
        io_regs[LY] = 0;
        io_regs[STAT] &= ~0x3;              // mode = HBLANK
        return;
    }

    switch (ppu_mode) {
        case OAM_SCAN: {
            if (ppu_cycles >= OAM_CYCLES) {
                io_regs[STAT] = (io_regs[STAT] & ~0x3) | DRAWING;
                ppu_cycles -= OAM_CYCLES;
            }
            break;
        }

        case DRAWING: {
            if (ppu_cycles >= VRAM_CYCLES) {
                io_regs[STAT] = (io_regs[STAT] & ~0x3) | HBLANK;
                ppu_draw_scanline();
                ppu_cycles -= VRAM_CYCLES;

                if (is_bit_set(io_regs[STAT], 3))
                    request_interrupt(INT_STAT);
            }
            break;
        }

        case HBLANK: {
            if (ppu_cycles >= HBLANK_CYCLES) {
                io_regs[LY]++;
                ppu_cycles -= HBLANK_CYCLES;

                if (io_regs[LY] == SCREEN_HEIGHT) {
                    io_regs[STAT] = (io_regs[STAT] & ~0x3) | VBLANK;
                    request_interrupt(INT_VBLANK);
                    if (is_bit_set(io_regs[STAT], 4))
                        request_interrupt(INT_STAT);

                    ppu_frame_ready = true;
                } else {
                    io_regs[STAT] = (io_regs[STAT] & ~0x3) | OAM_SCAN;

                    if (is_bit_set(io_regs[STAT], 5))
                        request_interrupt(INT_STAT);
                }
            }
            break;
        }

        case VBLANK: {
            if (ppu_cycles >= SCANLINE_CYCLES) {
                io_regs[LY]++;
                ppu_cycles -= SCANLINE_CYCLES;

                if (io_regs[LY] > SCREEN_VBLANK_HEIGHT) {
                    io_regs[STAT] = (io_regs[STAT] & ~0x3) | OAM_SCAN;
                    io_regs[LY] = 0;

                    if (is_bit_set(io_regs[STAT], 5))
                        request_interrupt(INT_STAT);
                }
            }
            break;
        }
    }

    if (io_regs[LY] == io_regs[LYC]) {
        io_regs[STAT] |= 0x4;
        if (is_bit_set(io_regs[STAT], 6))
            request_interrupt(INT_STAT);
    } else {
        io_regs[STAT] &= ~0x4;
    }
}

static void ppu_draw_scanline() {
    if (is_bit_set(io_regs[LCDC], 0)) {     // render Background
        draw_bg();
    }
    if (is_bit_set(io_regs[LCDC], 1)) {     // render Object
        //draw_obj();
    }
    if (is_bit_set(io_regs[LCDC], 5)) {     // render Windows
        //draw_win();
    }
}

static void draw_obj() {
    u8 lcdc = io_regs[LCDC];
    u8 ly = io_regs[LY];
    u8 obp0 = io_regs[OBP0];
    u8 obp1 = io_regs[OBP1];
    u8 bgp = io_regs[BGP];

    for (int i = 0x9C; i >= 0; i -= 4) {
        u8 sprite_size = (is_bit_set(lcdc, 2)) ? 16 : 8;   // false = 8x8, true = 8x16

        int y = oam[i] - 16;
        int x = oam[i + 1] - 8;
        u8 tile = oam[i + 2];
        u8 attr = oam[i + 3];

        if ((ly >= y) && (ly < (y + sprite_size))) {
            u8 palette = is_bit_set(attr, 4) ? obp1 : obp0;

            int tile_row = is_bit_set(attr, 6) ? sprite_size - 1 - (ly - y) : (ly - y);

            u16 tile_addr = (tile * 16) + (tile_row * 2);
            u8 lo = vram[0][tile_addr];
            u8 hi = vram[0][tile_addr + 1];

            for (int p = 0; p < 8; p++) {
                int id_pos = is_bit_set(attr, 5) ? p : 7 - p;

                int high = (hi >> id_pos) & 0x1;
                int low = (lo >> id_pos) & 0x1;

                int color_id =(high << 1 | low);
                int color_id_pal = (palette >> color_id * 2) & 0x3;

                u8 id = bgp & 0x3;
                bool is_bg_white = framebuffer[(x + p) + SCREEN_WIDTH * ly] == colors[id];

                if ((x + p) >= 0 && (x + p) < SCREEN_WIDTH) {
                    if ((color_id != 0) && (((attr & 0x80) != 0) || (is_bg_white))) {
                        framebuffer[(x + p) + SCREEN_WIDTH * ly] = colors[color_id_pal];
                    }
                }
            }
        }
    }
}

static void draw_bg() {
    u8 lcdc = io_regs[LCDC];
    u8 bgp = io_regs[BGP];
    u8 ly = io_regs[LY];
    u8 scx = io_regs[SCX];
    u8 scy = io_regs[SCY];

    if (ly >= SCREEN_HEIGHT) {
        return;
    }

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        u16 bg_y = (ly + scy) & 0xFF;
        u16 bg_x = (x + scx) & 0xFF;
        u16 tile_col = bg_x / 8;
        u16 tile_row = bg_y / 8;

        u16 bg_tile_base = (is_bit_set(lcdc, 3)) ? 0x9C00 : 0x9800;
        u16 bg_tile_data = (is_bit_set(lcdc, 4)) ? 0x8000 : 0x8800;
        bool signed_idx = (!is_bit_set(lcdc, 4));

        u16 tile_idx_addr = bg_tile_base | (tile_row << 5) | tile_col;
        u8 tile_idx = vram[0][tile_idx_addr - VRAM_START];

        if (signed_idx) {
            tile_idx = (i8)(tile_idx + 128);
        }

        u16 tile_addr = bg_tile_data + (tile_idx << 4) - VRAM_START;

        u8 line = bg_y % 8;

        u8 lo = vram[0][tile_addr | (line << 1)];
        u8 hi = vram[0][tile_addr | (line << 1) | 1];

        int bit = 7 - (bg_x % 8);
        u8 color_id = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

        u8 palette_id = (bgp >> (color_id * 2)) & 0x3;
        framebuffer[ly * SCREEN_WIDTH + x] = colors[palette_id];
    }
}
