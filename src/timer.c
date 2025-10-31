#include "timer.h"
#include "mmu.h"

u8 passed_cycles = 0;
const int timer_update_lut[] = { 256, 4, 16, 64 };

void update_timer(u64 cycles, bool stopped) {
    const u8 tma = io_regs[TMA];
    const u8 tac = io_regs[TAC];
    u16 tima = (u16)io_regs[TIMA];
    const int update_rate = timer_update_lut[tac & 0x03];

    bool update_div = false;
    bool update_tima = false;

    if (cycles / 64 > passed_cycles / 64) {
        update_div = true;
    }

    if (cycles / update_rate > passed_cycles / update_rate) {
        update_tima = true;
    }

    // DIV updates at a constant rate of every 64 Machine cycles (or 32 for CGB-mode)
    if (!stopped && update_div) {
        io_regs[DIV]++;
    }

    if (IS_BIT_SET(tac, 2) && update_tima) {
        tima++;
    }

    if (tima > 0xFF) {
        io_regs[TIMA] = tma;
        request_interrupt(INT_TIMER);
    }

    passed_cycles = cycles;
}
