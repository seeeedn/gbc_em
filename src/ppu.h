#ifndef PPU_H
#define PPU_H

#include <SDL3/SDL.h>
#include "utils.h"

#define OAM_CYCLES 80
#define VRAM_CYCLES 172
#define HBLANK_CYCLES 204
#define SCANLINE_CYCLES 456
#define SCREEN_VBLANK_HEIGHT 153

typedef struct {
    SDL_Window *window;
    SDL_Renderer *renderer;
    SDL_Texture *texture;
} RenderInfo;

extern RenderInfo render_util;

extern u32 framebuffer[160 * 144];
extern bool ppu_frame_ready;

void init_sdl(char *win_name, int win_width, int win_height);
void ppu_step(u16 cycles);

#endif
