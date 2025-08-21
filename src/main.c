#include <SDL2/SDL.h>
#include "cpu.h"
#include "mmu.h"
#include "ppu.h"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 144

void load_rom(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) { printf("ROM not found!\n"); exit(1); }
    fseek(file, 0, SEEK_END);
    rom_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    rom_banks = malloc(rom_size);
    fread(rom_banks, 1, rom_size, file);
    fclose(file);
}


void print_registers(CPU *cpu) {
    printf("AF: %04X  BC: %04X  DE: %04X  HL: %04X  SP: %04X  PC: %04X\n",
           cpu->AF, cpu->BC, cpu->DE, cpu->HL, cpu->SP, cpu->PC);
    printf("Flags: Z=%d N=%d H=%d C=%d\n",
           IS_FLAG_SET(cpu->F, FLAG_Z),
           IS_FLAG_SET(cpu->F, FLAG_N),
           IS_FLAG_SET(cpu->F, FLAG_H),
           IS_FLAG_SET(cpu->F, FLAG_C));
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

    load_rom("./tet.gb");

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
        // Step one CPU instruction
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

        //print_registers(&cpu);


        if (ppu_frame_ready) {
            SDL_UpdateTexture(texture, NULL, framebuffer, SCREEN_WIDTH * sizeof(uint32_t));
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
