#include "ppu.h"
#include "mmu.h"
#include "debug.h"
#include <stdlib.h>

#define SCREEN_HEIGHT           144
#define SCREEN_WIDTH            160
#define SCANLINE_MAX_SPRITES     10

u16 ppu_cycles = 0;
const u32 colors[4] = { 0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000 };       // grey-scale

u32 framebuffer[SCREEN_HEIGHT * SCREEN_WIDTH] = {0};
RenderInfo render_util;

//enum PpuMode { HBLANK = 0, VBLANK = 1, OAM = 2, DRAWING = 3 };
//static enum PpuMode mode;

void init_sdl(char *win_name, int win_width, int win_height) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);
    
    SDL_Window *window = SDL_CreateWindow(
        win_name,
        win_width,
        win_height,
        0
    );
    if (window == nullptr) {
        fprintf(stderr, "Failed to initialize SDL-Window\n");
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);
    if (renderer == nullptr) {
        fprintf(stderr, "Failed to initialize SDL-Renderer\n");
        SDL_DestroyWindow(window);
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    SDL_Texture *texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        win_width,
        win_height
    );
    if (texture == nullptr) {
        fprintf(stderr, "Failed to initialized SDL-Texture\n");
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    SDL_SetRenderLogicalPresentation(
        renderer, 
        win_width,
        win_height,
        SDL_LOGICAL_PRESENTATION_INTEGER_SCALE
    );
    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);
    SDL_SetWindowSize(window, win_width * 4, win_height * 4);

    render_util.window = window;
    render_util.renderer = renderer;
    render_util.texture = texture;
}

void render() {
    SDL_UpdateTexture(render_util.texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(u32));

    SDL_RenderClear(render_util.renderer);
    SDL_RenderTexture(render_util.renderer, render_util.texture, NULL, NULL);
    SDL_RenderPresent(render_util.renderer);
}

static void ppu_draw_scanline();
static void draw_obj();
static void draw_bg();
static void draw_win();

void ppu_step(u16 cycles) {
    ppu_cycles += cycles;

    // LCD off
    if (!is_bit_set(io_regs[LCDC], 7)) {
        ppu_cycles = 0;
        io_regs[LY] = 0;
        io_regs[STAT] = (io_regs[STAT] & ~0x3) | HBLANK;
        return;
    }

    while (ppu_cycles >= 456) {
        ppu_cycles -= 456;
        io_regs[LY]++;

        // Enter VBlank
        if (io_regs[LY] == 144) {
            io_regs[STAT] = (io_regs[STAT] & ~0x3) | VBLANK;
            request_interrupt(INT_VBLANK);
            render();
        }
        // End of VBlank
        else if (io_regs[LY] > 153) {
            io_regs[LY] = 0;
        }
    }

    u8 new_mode;
    if (io_regs[LY] >= 144) {
        new_mode = VBLANK;
    } else if (ppu_cycles < 80) {
        new_mode = OAM_SCAN;
    } else if (ppu_cycles < 252) {
        new_mode = DRAWING;
    } else {
        new_mode = HBLANK;
    }

    u8 old_mode = io_regs[STAT] & 0x3;
    io_regs[STAT] = (io_regs[STAT] & ~0x3) | new_mode;

    // STAT interrupt
    if (new_mode != old_mode) {
        if ((new_mode == HBLANK && is_bit_set(io_regs[STAT], 3)) ||
            (new_mode == VBLANK && is_bit_set(io_regs[STAT], 4)) ||
            (new_mode == OAM_SCAN && is_bit_set(io_regs[STAT], 5))) {
            request_interrupt(INT_STAT);
        }

        if (new_mode == HBLANK) {
            ppu_draw_scanline();
        }
    }

    // LY == LYC
    if (io_regs[LY] == io_regs[LYC]) {
        io_regs[STAT] |= 0x04;
        if (is_bit_set(io_regs[STAT], 6)) {
            request_interrupt(INT_STAT);
        }
    } else {
        io_regs[STAT] &= ~0x04;
    }
}

static void ppu_draw_scanline() {
    if (is_bit_set(io_regs[LCDC], 0)) {     // render Background
        draw_bg();
    }
    if (is_bit_set(io_regs[LCDC], 1)) {     // render Object
        draw_obj();
    }
    if (is_bit_set(io_regs[LCDC], 5)) {     // render Windows
        draw_win();
    }
}

static void draw_obj() {
    u8 lcdc = io_regs[LCDC];
    u8 ly   = io_regs[LY];
    u8 obp0 = io_regs[OBP0];
    u8 obp1 = io_regs[OBP1];
    u8 bgp  = io_regs[BGP];
    int obj_in_scanline = 0;

    for (int i = 0; i < 0xA0; i += 4) {
        u8 sprite_size = (is_bit_set(lcdc, 2)) ? 16 : 8;   // false = 8x8, true = 8x16

        int y = oam[i] - 16;
        int x = oam[i + 1] - 8;
        u8 tile = oam[i + 2];
        u8 attr = oam[i + 3];

        if (sprite_size == 16) {
            tile &= 0xFE;
        }

        //printf("OAM[%d] rawY=%d calcY=%d ly=%d size=%d\n", i/4, oam[i], oam[i] - 16, ly, sprite_size);

        if ((ly >= y) && (ly < (y + sprite_size))) {
            u8 palette = is_bit_set(attr, 4) ? obp1 : obp0;

            int tile_row = is_bit_set(attr, 6) ? sprite_size - 1 - (ly - y) : (ly - y);

            u16 tile_addr = (tile << 4) + (tile_row << 1);
            u8 lo = vram[0][tile_addr];
            u8 hi = vram[0][tile_addr + 1];

            for (int p = 0; p < 8; p++) {
                int id_pos = is_bit_set(attr, 5) ? p : 7 - p;

                int high = (hi >> id_pos) & 0x1;
                int low = (lo >> id_pos) & 0x1;

                int color_id = (high << 1 | low);
                int color_id_pal = (palette >> color_id * 2) & 0x3;

                u8 id = bgp & 0x3;
                bool is_bg_white = framebuffer[(x + p) + SCREEN_WIDTH * ly] == colors[id];

                if ((x + p) >= 0 && (x + p) < SCREEN_WIDTH) {
                    if ((color_id != 0) && (((attr & 0x80) == 0) || (is_bg_white))) {
                        framebuffer[(x + p) + SCREEN_WIDTH * ly] = colors[color_id_pal];
                    }
                }
            }

            obj_in_scanline++;
            if (obj_in_scanline >= 10) {
                return;
            }
        }
    }
}

static void draw_bg() {
    u8 lcdc = io_regs[LCDC];
    u8 bgp  = io_regs[BGP];
    u8 ly   = io_regs[LY];
    u8 scx  = io_regs[SCX];
    u8 scy  = io_regs[SCY];

    if (ly >= SCREEN_HEIGHT) {
        return;
    }

    for (int x = 0; x < SCREEN_WIDTH; x++) {
        u16 bg_y = (ly + scy) & 0xFF;
        u16 bg_x = (x + scx) & 0xFF;

        u16 tile_col = bg_x / 8;
        u16 tile_row = bg_y / 8;

        u8 line = bg_y % 8;

        u16 tilemap     = (is_bit_set(lcdc, 3)) ? 0x9C00 : 0x9800;
        u16 tiledata    = (is_bit_set(lcdc, 4)) ? 0x8000 : 0x8800;
        bool is_signed  = (!is_bit_set(lcdc, 4));

        u16 tile_id_addr = tilemap + (tile_row << 5) + tile_col;
        u8 tile_id = vram[0][tile_id_addr - VRAM_START];

        if (is_signed) {
            tile_id = (i8)(tile_id + 128);
        }

        u16 tile_addr = tiledata + (tile_id << 4) - VRAM_START;

        u8 lo = vram[0][tile_addr + (line << 1)];
        u8 hi = vram[0][tile_addr + (line << 1) + 1];

        int bit = 7 - (bg_x % 8);
        u8 color_id = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

        u8 palette_id = (bgp >> (color_id * 2)) & 0x3;
        framebuffer[ly * SCREEN_WIDTH + x] = colors[palette_id];
    }
}

static void draw_win() {
    u8 lcdc = io_regs[LCDC];
    u8 bgp  = io_regs[BGP];
    u8 ly   = io_regs[LY];
    u8 scy  = io_regs[SCY];
    u8 scx  = io_regs[SCX];
    u8 wy   = io_regs[WY];
    int wx   = io_regs[WX] - 7;

    if (ly < wy || ly >= SCREEN_HEIGHT) {
        return;
    }

    if (wx < 0) {
        wx = 0;
    }

    for (int x = 0; x < 160 - wx; x++) {

        u16 tilemap     = (is_bit_set(lcdc, 6)) ? 0x9C00 : 0x9800;
        u16 tiledata    = (is_bit_set(lcdc, 4)) ? 0x8000 : 0x8800;
        bool is_signed  = (!is_bit_set(lcdc, 4));

        int win_y = ly - io_regs[WY];
        int win_x = x - wx;

        u8 line = win_y % 8;

        u16 tile_row = win_y / 8;
        u16 tile_col = win_x / 8;
        u16 tile_id_addr = tilemap + (tile_row << 5) + tile_col;

        u8 tile_id = vram[0][tile_id_addr - VRAM_START];

        if (is_signed) {
            tile_id = (i8)(tile_id + 128);
        }

        u16 tile_addr = tiledata + (tile_id << 4) - VRAM_START;

        u8 lo = vram[0][tile_addr + (line << 1)];
        u8 hi = vram[0][tile_addr + (line << 1) + 1];

        int bit = 7 - (win_x % 8);
        u8 color_id = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

        u8 palette_id = (bgp >> (color_id * 2)) & 0x3;
        framebuffer[ly * SCREEN_WIDTH + x] = colors[palette_id];

    }
}
