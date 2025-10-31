#include "input.h"
#include "mmu.h"

u8 joypad_input = 0x0F;

void handle_input(SDL_Event *event) {
    u8 joypad_upper = io_regs[JOYPAD] & 0xF0;
    u8 joypad_input = 0x0F;

    if (event->type == SDL_KEYDOWN) {
        if (event->key.keysym.sym == SDLK_RIGHT || event->key.keysym.sym == SDLK_x) {
            joypad_input &= ~RIGHT_A;
        }
        if (event->key.keysym.sym == SDLK_LEFT || event->key.keysym.sym == SDLK_z) {
            joypad_input &= ~LEFT_B;
        }
        if (event->key.keysym.sym == SDLK_UP || event->key.keysym.sym == SDLK_SPACE) {
            joypad_input &= ~UP_SEL;
        }
        if (event->key.keysym.sym == SDLK_DOWN || event->key.keysym.sym == SDLK_RETURN) {
            joypad_input &= ~DOWN_ST;
        }
    }

    if ((joypad_input & 0x0F) != 0x0F) {
        request_interrupt(INT_JOYPAD);
        //printf("joyp intr\n");
    }
}
