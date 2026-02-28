#include <SDL3/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"
#include "input.h"
#include "timer.h"
#include "interrupt.h"
#include "debug.h"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144

void load_rom(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        printf("ROM not found!\n");
        exit(EXIT_FAILURE);
    }

    fseek(file, 0, SEEK_END);
    size_t rom_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    rom_banks = malloc(rom_size);
    fread(rom_banks, 1, rom_size, file);
    fclose(file);
}

int main(int argc, char *argv[]) {
    if (argc < 1) {
        return EXIT_FAILURE;
    }

    init_sdl("GBC Emu", SCREEN_WIDTH, SCREEN_HEIGHT);

    load_rom(argv[1]);

    init_cpu();
    init_mmu();

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }

            if (event.type == SDL_EVENT_KEY_DOWN) {
                switch (event.key.key) {
                    case SDLK_RIGHT:  joypad.right  = true; break;
                    case SDLK_LEFT:   joypad.left   = true; break;
                    case SDLK_UP:     joypad.up     = true; break;
                    case SDLK_DOWN:   joypad.down   = true; break;

                    case SDLK_X:      joypad.a      = true; break;
                    case SDLK_Z:      joypad.b      = true; break;
                    case SDLK_SPACE:  joypad.select = true; break;
                    case SDLK_RETURN: joypad.start  = true; break;
                }
            }

            if (event.type == SDL_EVENT_KEY_UP) {
                switch (event.key.key) {
                    case SDLK_RIGHT:  joypad.right  = false; break;
                    case SDLK_LEFT:   joypad.left   = false; break;
                    case SDLK_UP:     joypad.up     = false; break;
                    case SDLK_DOWN:   joypad.down   = false; break;

                    case SDLK_X:      joypad.a      = false; break;
                    case SDLK_Z:      joypad.b      = false; break;
                    case SDLK_SPACE:  joypad.select = false; break;
                    case SDLK_RETURN: joypad.start  = false; break;
                }
            }
        }

        u16 cycles = cpu_step();
        update_timer(cycles, cpu.stopped);
        ppu_step(cycles);
    }

    SDL_DestroyTexture(render_util.texture);
    SDL_DestroyRenderer(render_util.renderer);
    SDL_DestroyWindow(render_util.window);
    SDL_Quit();

    return EXIT_SUCCESS;
}
