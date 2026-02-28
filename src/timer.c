#include "timer.h"
#include "mmu.h"

u64 tima_counter = 0;
u64 div_counter = 0;
const int timer_update_lut[] = { 1024, 16, 64, 256 };       // TIMA update rates in CPU cycles

void update_timer(u64 cycles, bool stopped) {
    const u8 tma = io_regs[TMA];
    const u8 tac = io_regs[TAC];
    const u16 update_rate = timer_update_lut[tac & 0x03];

    if (!stopped) {
        div_counter += cycles;
        while (div_counter >= 64) {
            div_counter -= 64;
            io_regs[DIV]++;
        }
    }

    tima_counter += cycles;

    if (IS_BIT_SET(tac, 0x04)) {
        while (tima_counter >= update_rate) {
            tima_counter -= update_rate;

            if (io_regs[TIMA] == 0xFF) {
                io_regs[TIMA] = tma;
                request_interrupt(INT_TIMER);
            } else {
                io_regs[TIMA]++;
            }
        }
    }
}
