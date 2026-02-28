#ifndef INPUT_H
#define INPUT_H

#include <SDL3/SDL.h>
#include "utils.h"

#define RIGHT_A     0x01
#define LEFT_B      0x02
#define UP_SEL      0x04
#define DOWN_ST     0x08
#define DPAD        0x10
#define BUTTON      0x20

typedef struct {
    bool right, left, up, down;
    bool a, b, select, start;
} Joypad;

extern Joypad joypad;

u8 handle_input();

#endif
