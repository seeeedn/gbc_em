#include "cpu.h"

void init_cpu(CPU *cpu) {
    cpu->AF = 0x01B0;
    cpu->BC = 0x0013;
    cpu->DE = 0x00D8;
    cpu->HL = 0x014D;
    cpu->SP = 0xFFFE;
    cpu->PC = 0x0100;
    cpu->ime = 1;
    cpu->halted = 0;
    cpu->stopped = 0;
    cpu->cycles = 0;
}