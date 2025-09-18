#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144

void load_rom(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        printf("ROM not found!\n");
        exit(EXIT_FAILURE);
    }

    fseek(file, 0, SEEK_END);
    rom_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    rom_banks = malloc(rom_size);
    fread(rom_banks, 1, rom_size, file);
    fclose(file);
}

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);

    SDL_Window* window = SDL_CreateWindow(
        "GBC Emu",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_WIDTH, SCREEN_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture* texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH,
        SCREEN_HEIGHT
    );

    SDL_RenderSetLogicalSize(renderer, 160, 144);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);
    SDL_SetWindowSize(window, 640, 576);

    load_rom(argv[1]);

    CPU cpu;
    init_cpu(&cpu);
    init_mmu();

    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        u8 opcode = mmu_read_byte(cpu.PC);
        u8 cycles = execute_instruction(&cpu, opcode);

        if (cpu.ime_delay > 0) {
            cpu.ime_delay--;
            if (cpu.ime_delay == 0) {
                cpu.ime = true;
            }
        }

        cycles += handle_interrupt(&cpu);

        ppu_step(cycles);

        if (ppu_frame_ready) {
            SDL_UpdateTexture(texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(u32));

            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);

            ppu_frame_ready = false;
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
