#include "cpu.h"
#include <stdio.h>

u8 memory[MEM_SIZE] = {0};

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

static u8 read_byte(u16 address) {
    return memory[address];
}

static u16 read_word(u16 address) {
    return (read_byte(address + 1) << 8) | read_byte(address);
}

static void execute_instruction(CPU *cpu, u8 opcode) {
    switch (opcode) {
        case 0x00:              // NOP
            cpu->cycles += 4;
            break;

        case 0x01:              // LD BC, d16
            cpu->cycles += 12;


            break;

        
        
        default:                // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X\n", opcode, cpu->PC - 1);
            break;
    }
}

void cpu_step(CPU *cpu) {
    u8 opcode = memory[cpu->PC++];
    execute_instruction(cpu, opcode);
}