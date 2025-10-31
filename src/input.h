#ifndef INPUT_H
#define INPUT_H

#include <SDL2/SDL.h>
#include "utils.h"

#define RIGHT_A     0x01
#define LEFT_B      0x02
#define UP_SEL      0x04
#define DOWN_ST     0x08
#define SEL_DPAD    0x10
#define SEL_BUTTON  0x20

extern u8 joypad_input;

void handle_input(SDL_Event *event);

#endif
