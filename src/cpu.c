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
    cpu->ime = true;
    cpu->halted = false;
    cpu->stopped = false;
    cpu->cycles = 0;
}

// ============================================= //
//             Read & Write Functions            //
// ============================================= //
static u8 read_byte(u16 address) {
    return memory[address];
}

static u16 read_word(u16 address) {
    return (read_byte(address + 1) << 8) | read_byte(address);
}

static void write_byte(u16 address, u8 value) {
    memory[address] = value;
}

static void write_word(u16 address, u16 value) {
    write_byte(address, (u8)(value & 0xFF));
    write_byte(address + 1, (u8)(value >> 8));
}

// executes the instruction of the given opcode (with cb prefix)
static void execute_cb_instruction(CPU *cpu, u8 opcode) {
    switch (opcode) {
        case 0x00:
            // TODO
            break;
        
        default:                // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X (CB-prefixed)\n", opcode, cpu->PC - 1);
            break;
    }
}

// executes the instruction of the given opcode (without cb prefix)
static void execute_instruction(CPU *cpu, u8 opcode) {
    switch (opcode) {
        // ============================================= //
        //           Miscellaneous instructions          //
        // ============================================= //

        case 0x00: {            // NOP (No operation)
            cpu->cycles += 4;
            break;
        }

        case 0x10: {            // STOP 0 (Stop CPU)
            cpu->cycles += 4;
            cpu->stopped = true;
            cpu->PC += 1;
            break;
        }

        case 0xCB: {            // PREFIX CB
            cpu->cycles += 4;
            u8 cb_opcode = read_byte(cpu->PC + 1);
            execute_cb_instruction(cpu, cb_opcode);
            cpu->PC += 1;
            break;
        }



        
        // ============================================= //
        //            LD -> Load Instructions            //
        // ============================================= //

        case 0x01: {            // LD BC, d16
            cpu->cycles += 12;
            u16 d16 = read_word(cpu->PC + 1);
            cpu->BC = d16;
            cpu->PC += 3;
            break;
        }

        case 0x02: {            // LD [BC], A
            cpu->cycles += 8;
            write_byte(cpu->BC, cpu->A);
            cpu->PC += 1;
            break;
        }

        case 0x06: {            // LD B, d8
            cpu->cycles += 8;
            u8 d8 = read_byte(cpu->PC + 1);
            cpu->B = d8;
            cpu->PC += 2;
            break;
        }

        case 0x08: {            // LD [a16], SP
            cpu->cycles += 20;
            u16 address = read_word(cpu->PC + 1);
            write_word(address, cpu->SP);
            cpu->PC += 3;
            break;
        }

        case 0x0A: {            // LD A, [BC]
            cpu->cycles += 8;
            u8 value = read_byte(cpu->BC);
            cpu->A = value;
            cpu->PC += 1;
            break;
        }

        case 0x11: {            // LD DE, d16
            cpu->cycles += 12;
            u16 value = read_word(cpu->PC + 1);
            cpu->DE = value;
            cpu->PC += 3;
            break;
        }

        case 0x12: {            // LD [DE], A
            cpu->cycles += 8;
            u16 address = cpu->DE;
            write_byte(address, cpu->A);
            cpu->PC += 1;
            break;
        }

        case 0x0E: {            // LD C, d8
            cpu->cycles += 8;
            u8 value = read_byte(cpu->PC + 1);
            cpu->C = value;
            cpu->PC += 2;
            break;
        }

        case 0x16: {            // LD D, d8
            cpu->cycles += 8;
            u8 value = read_byte(cpu->PC + 1);
            cpu->D = value;
            cpu->PC += 2;
            break;
        }




        // ============================================= //
        //         8-bit arithmetic instructions         //
        // ============================================= //

        case 0x04: {            // INC B
            cpu->cycles += 4;
            u8 result = cpu->B + 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->B & 0x0F) == 0x0F) SET_FLAG(cpu->F, FLAG_H);

            cpu->B = result;
            cpu->PC += 1;
            break;
        }

        case 0x05: {            // DEC B
            cpu->cycles += 4;
            u8 result = cpu->B - 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H);
            SET_FLAG(cpu->F, FLAG_N);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->B & 0x0F) == 0x10) SET_FLAG(cpu->F, FLAG_H);

            cpu->B = result;
            cpu->PC += 1;
            break;
        }

        case 0x0C: {            // INC C
            cpu->cycles += 4;
            u8 result = cpu->C + 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->C & 0x0F) == 0x0F) SET_FLAG(cpu->F, FLAG_H);

            cpu->C = result;
            cpu->PC += 1;
            break;
        }

        case 0x0D: {            // DEC C
            cpu->cycles += 4;
            u8 result = cpu->C - 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H);
            SET_FLAG(cpu->F, FLAG_N);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->C & 0x0F) == 0x10) SET_FLAG(cpu->F, FLAG_H); 

            cpu->C = result;
            cpu->PC += 1;
            break;
        }


        case 0x14: {            // INC D
            cpu->cycles += 4;
            u8 result = cpu->D + 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->D & 0x0F) == 0x0F) SET_FLAG(cpu->F, FLAG_H);

            cpu->PC += 1;
            break;
        }

        case 0x15: {            // DEC D
            cpu->cycles += 4;
            u8 result = cpu->D - 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H);
            SET_FLAG(cpu->F, FLAG_N);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->D & 0x0F) == 0x10) SET_FLAG(cpu->F, FLAG_H); 

            cpu->PC += 1;
            break;
        }




        // ============================================= //
        //         16-bit arithmetic instructions        //
        // ============================================= //

        case 0x03: {            // INC BC
            cpu->cycles += 8;
            cpu->BC += 1;
            cpu->PC += 1;
            break;
        }

        case 0x09: {            // ADD HL, BC
            cpu->cycles += 8;
            u32 result = cpu->HL + cpu->BC;

            CLEAR_FLAG(cpu->F, FLAG_N);
            if ((cpu->HL & 0xFFF) + (cpu->BC & 0xFFF) > 0xFFF)      // Half Carry
                SET_FLAG(cpu->F, FLAG_H);
            else
                CLEAR_FLAG(cpu->F, FLAG_H);
        
            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);
            else
                CLEAR_FLAG(cpu->F, FLAG_C);

            cpu->HL = (u16)result;
            cpu->PC += 1;
            break;
        }

        case 0x0B: {            // DEC BC
            cpu->cycles += 4;
            cpu->BC -= 1;
            cpu->PC += 1;
            break;
        }

        case 0x13: {            // INC DE
            cpu->cycles += 8;
            cpu->DE += 1;
            cpu->PC += 1;
            break;
        }

        


        // ============================================= //
        //             Bit shift instructions            //
        // ============================================= //

        case 0x07: {            // RLCA (Rotate Left Circular Accumulator)
            cpu->cycles += 4;
            u8 bit7 = (cpu->A & 0x80) >> 7;
            cpu->A = (cpu->A << 1) | bit7;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H | FLAG_C);
            if (bit7) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }

        case 0x0F: {            // RRCA (Rotate Right Circular Accumulator)
            cpu->cycles += 4;
            u8 bit0 = (cpu->A & 0x01) << 7;
            cpu->A = (cpu->A >> 1) | bit0;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H | FLAG_C);
            if (bit0) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }

        case 0x17: {            // RLA (Rotate Left Accumulator (through Carry flag))
            cpu->cycles += 4;
            u8 old_carry = (IS_FLAG_SET(cpu->F, FLAG_C) ? 1 : 0);
            u8 bit7 = (cpu->A & 0x80) >> 7;
            cpu->A = (cpu->A << 1) | old_carry;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H | FLAG_C);
            if (bit7) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }



        
        default:                // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X\n", opcode, cpu->PC - 1);
            break;
    }
}

void cpu_step(CPU *cpu) {
    u8 opcode = memory[cpu->PC++];
    execute_instruction(cpu, opcode);
}