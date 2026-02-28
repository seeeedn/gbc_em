#include "input.h"
#include "mmu.h"

Joypad joypad;

u8 handle_input() {
    u8 joyp = 0xCF;

    u8 select = read_mmio(JOYPAD) & 0x30;
    u8 lower = joyp & 0x0F;

    if (!(select & DPAD)) {
        if (joypad.right)
            lower &= ~RIGHT_A;
        if (joypad.left)
            lower &= ~LEFT_B;
        if (joypad.up)
            lower &= ~UP_SEL;
        if (joypad.down)
            lower &= ~DOWN_ST;
    }

    if (!(select & BUTTON)) {
        if (joypad.a)
            lower &= ~RIGHT_A;
        if (joypad.b)
            lower &= ~LEFT_B;
        if (joypad.select)
            lower &= ~UP_SEL;
        if (joypad.start)
            lower &= ~DOWN_ST;
    }

    u8 new_joyp = (joyp & 0xF0) | lower;

    if (new_joyp != joyp) {
        request_interrupt(INT_JOYPAD);
    }

    return lower;
}
