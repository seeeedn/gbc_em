#include <SDL2/SDL.h>
#include <stdio.h>
#include "cpu.h"

int main(int argc, char* argv[]) {

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);
    SDL_Window* window;

    window = SDL_CreateWindow(
        "GBC Emu",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        160, 144, SDL_WINDOW_SHOWN
    );

    SDL_SetRelativeMouseMode(SDL_TRUE);

    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }
        SDL_GL_SwapWindow(window);
    }

    return 0;
}
