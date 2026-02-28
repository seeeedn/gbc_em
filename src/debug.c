#include "debug.h"
#include "cpu.h"
#include "mmu.h"

RingEntry entries[RING_SIZE];
unsigned int ring_pos = 0;

void ring_log() {
    RingEntry e = {
        .pc = cpu.PC,
        .opcode = mmu_read_byte(cpu.PC),
        .ly = read_mmio(LY),
        .stat = read_mmio(STAT),
        .IE = read_mmio(0xFF),
        .IF = read_mmio(IF_ADDRESS),
        .IME = cpu.ime,
        .halted = cpu.halted
    };

    entries[ring_pos] = e;
    ring_pos = (ring_pos + 1) % RING_SIZE;
}

void ring_dump() {
    for (int i = 0; i < RING_SIZE; i++) {
        RingEntry *e = &entries[(ring_pos + i) % RING_SIZE];
        printf(
            "PC=%04X OP=%02X LY=%03d STAT=%02X IF=%02X IE=%02X IME=%d HALT=%d\n",
            e->pc, e->opcode, e->ly, e->stat,
            e->IF, e->IE, e->IME, e->halted
        );
    }
}
