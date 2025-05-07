#include "cpu.h"
#include <stdbool.h>
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
    cpu->ime_delay = 0;
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

static i8 read_byte_signed(u16 address) {
    return (i8)memory[address];
}

static u16 read_word(u16 address) {
    return (read_byte(address + 1) << 8) | read_byte(address);
}

static i16 read_word_signed(u16 address) {
    return (i16)((read_byte(address + 1) << 8) | read_byte(address));
}

static void write_byte(u16 address, u8 value) {
    memory[address] = value;
}

static void write_word(u16 address, u16 value) {
    write_byte(address, (u8)(value & 0xFF));
    write_byte(address + 1, (u8)(value >> 8));
}

static u8* get_register(CPU *cpu, u8 reg) {
    switch (reg) {
        case 0b000: return &cpu->B;
        case 0b001: return &cpu->C;
        case 0b010: return &cpu->D;
        case 0b011: return &cpu->E;
        case 0b100: return &cpu->H;
        case 0b101: return &cpu->L;
        // Left out 0b110 because its handled separately
        case 0b111: return &cpu->A;
        default: return NULL;
    }
}

static u16* get_u16_register(CPU *cpu, u8 reg) {
    switch (reg) {
        case 0b00: return &cpu->BC;
        case 0b01: return &cpu->DE;
        case 0b10: return &cpu->HL;
        case 0b11: return &cpu->SP;
        default: return NULL;
    }
}

// ============================================= //
//                 Load Functions                //
// ============================================= //
static void load_to_r8(u8 *dest_reg, u8 source_reg) {
    *dest_reg = source_reg;
}

static void load_to_r16(u16 *dest_reg, u16 value) {
    *dest_reg = value;
}

static void load_hl(u8 *dest_reg, u16 address) {
    u8 value = read_byte(address);
    *dest_reg = value;
}

static void load_to_addr(u16 address, u8 source_reg) {
    u8 value = source_reg;
    write_byte(address, value);
}

// ============================================= //
//                Logic Functions                //
// ============================================= //
static void and_r8(CPU *cpu, u8 reg) {
    u8 result = cpu->A & reg;

    CLEAR_ALL_FLAGS(cpu->F);
    SET_FLAG(cpu->F, FLAG_H);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

static void and_hl(CPU *cpu, u16 address) {
    u8 value = read_byte(address);
    u8 result = cpu->A & value;

    CLEAR_ALL_FLAGS(cpu->F);
    SET_FLAG(cpu->F, FLAG_H);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

static void xor_r8(CPU *cpu, u8 reg) {
    u8 result = cpu->A ^ reg;

    CLEAR_ALL_FLAGS(cpu->F);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

static void xor_hl(CPU *cpu, u16 address) {
    u8 value = read_byte(address);
    u8 result = cpu->A ^ value;

    CLEAR_ALL_FLAGS(cpu->F);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

static void or_r8(CPU *cpu, u8 reg) {
    u8 result = cpu->A | reg;

    CLEAR_ALL_FLAGS(cpu->F);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

static void or_hl(CPU *cpu, u16 address) {
    u8 value = read_byte(address);
    u8 result = cpu->A | value;

    CLEAR_ALL_FLAGS(cpu->F);
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);

    cpu->A = result;
}

// executes the instruction of the given opcode (with cb prefix)
static void execute_cb_instruction(CPU *cpu, u8 cb_opcode) {
    switch (cb_opcode) {

        case 0x40 ... 0x7F: {       // BIT x, r8

            break;
        }

        case 0x80 ... 0xBF: {       // RES x, r8

            break;
        }

        case 0xC0 ... 0xFF: {       // SET x, r8

            break;
        }

        default:                // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X (CB-prefixed)\n", cb_opcode, cpu->PC - 1);
            break;
    }
}

// executes the instruction of the given opcode (without cb prefix)
static void execute_instruction(CPU *cpu, u8 opcode) {
    switch (opcode) {
        // ============================================= //
        //           Misc/Control instructions           //
        // ============================================= //

        case 0x00: {            // NOP (No operation)
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x10: {            // STOP 0 (Stop CPU)
            cpu->stopped = true;
            cpu->cycles += 4;
            cpu->PC += 2;
            break;
        }

        case 0x76: {            // HALT
            cpu->halted = true;
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xCB: {            // PREFIX CB
            u8 cb_opcode = read_byte(cpu->PC + 1);
            execute_cb_instruction(cpu, cb_opcode);
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xF3: {            // DI (Disable Interrupts)
            cpu->ime = false;
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xFB: {            // EI (Enable Interrupts)
            cpu->ime_delay = 1;
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }




        // ============================================= //
        //            8-bit Load Instructions            //
        // ============================================= //

        case 0x02:              // LD [BC], A
        case 0x0A:              // LD A, [BC]
        case 0x12:              // LD [DE], A
        case 0x1A:              // LD A, [DE]
        case 0x22:              // LD [HL+], A
        case 0x2A:              // LD A, [HL+]
        case 0x32:              // LD [HL-], A
        case 0x3A: {            // LD A, [HL-]
            u8 register_bit_mask = (opcode & (3 << 4)) >> 4;
            u8 mode_bit_mask = opcode & 0b1111;
            u16 *dest_reg = get_u16_register(cpu, register_bit_mask);
            u16 address = *dest_reg;

            bool is_load_from = (mode_bit_mask == 0x0A);        // LD A, [rr]
            bool inc_hl = (opcode == 0x2A);                     // LD A, [HL+]
            bool dec_hl = (opcode == 0x3A);                     // LD A, [HL-]
            bool inc_hl_store = (opcode == 0x22);               // LD [HL+], A
            bool dec_hl_store = (opcode == 0x32);               // LD [HL-], A

            if (is_load_from || opcode == 0x2A || opcode == 0x3A) {
                u8 value = read_byte(address);
                load_to_r8(&cpu->A, value);
            } else {
                write_byte(address, cpu->A);
            }

            if (inc_hl || inc_hl_store) *dest_reg += 1;
            if (dec_hl || dec_hl_store) *dest_reg -= 1;

            cpu->cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0x06:              // LD B, d8
        case 0x0E:              // LD C, d8
        case 0x16:              // LD D, d8
        case 0x1E:              // LD E, d8
        case 0x26:              // LD H, d8
        case 0x2E:              // LD L, d8
        case 0x36:              // LD [HL], d8
        case 0x3E: {            // LD A, d8
            u8 register_bit_mask = (opcode & DEST_REG_BIT) >> 3;
            u8 *dest_reg = get_register(cpu, register_bit_mask);

            u8 value = read_byte(cpu->PC + 1);

            if (opcode == 0x36) {   // LD [HL], d8
                write_byte(cpu->HL, value);
                cpu->cycles += 12;
                cpu->PC += 2;
                break;
            }

            load_to_r8(dest_reg, value);        // LD r8, d8
            cpu->cycles += 8;
            cpu->PC += 2;
            break;
        }

        case 0x40 ... 0x75: {               // LD r8, (r8 / HL)
            u8 high_bit = (opcode & DEST_REG_BIT) >> 3;
            u8 low_bit = opcode & SOURCE_REG_BIT;
            u8 *dest_reg = get_register(cpu, high_bit);
            u8 *source_reg = get_register(cpu, low_bit);

            if (high_bit == 0b110) {        // Is true if dest_reg is HL
                load_to_addr(cpu->HL, *source_reg);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            if (low_bit == 0b110) {         // Is true if source_reg is HL
                load_hl(dest_reg, cpu->HL);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            load_to_r8(dest_reg, *source_reg);
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x77 ... 0x7F: {               // Same as above
            u8 high_bit = (opcode & DEST_REG_BIT) >> 3;
            u8 low_bit = opcode & SOURCE_REG_BIT;
            u8 *dest_reg = get_register(cpu, high_bit);
            u8 *source_reg = get_register(cpu, low_bit);

            if (high_bit == 0b110) {        // Is true if dest_reg is HL
                load_to_addr(cpu->HL, *source_reg);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            if (low_bit == 0b110) {         // Is true if source_reg is HL
                load_hl(dest_reg, cpu->HL);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            load_to_r8(dest_reg, *source_reg);
            cpu->cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xE0:
        case 0xF0: {
            u8 offset = read_byte(cpu->PC + 1);
            u16 address = 0xFF00 + offset;

            if (opcode == 0xE0) {               // LDH [0xFF00 + a8], A
                write_byte(address, cpu->A);
            } else {                            // LDH A, [0xFF00 + a8]
                u8 value = read_byte(address);
                load_to_r8(&cpu->A, value);
            }

            cpu->cycles += 12;
            cpu->PC += 2;
            break;
        }

        case 0xE2:
        case 0xF2: {
            u16 address = 0xFF00 + cpu->C;

            if (opcode == 0xE2) {               // LD [0xFF00 + C], A
                write_byte(address, cpu->A);
            } else {                            // LD A, [0xFF00 + C]
                u8 value = read_byte(address);
                load_to_r8(&cpu->A, value);
            }

            cpu->cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0xEA:
        case 0xFA: {
            u16 address = read_word(cpu->PC);

            if (opcode == 0xEA) {               // LDH [a16], A
                write_byte(address, cpu->A);
            } else {                            // LDH A, [a16]
                u8 value = read_byte(address);
                load_to_r8(&cpu->A, value);
            }

            cpu->cycles += 16;
            cpu->PC += 3;
            break;
        }



        // ============================================= //
        //            16-bit Load Instructions           //
        // ============================================= //

        case 0x01:              // LD BC, d16
        case 0x08:              // LD [a16], SP
        case 0x11:              // LD DE, d16
        case 0x21:              // LD HL, d16
        case 0x31: {            // LD SP, d16
            u16 *dest_reg = NULL;

            switch (opcode) {
                case 0x01: dest_reg = &cpu->BC; break;
                case 0x11: dest_reg = &cpu->DE; break;
                case 0x21: dest_reg = &cpu->HL; break;
                case 0x31: dest_reg = &cpu->SP; break;
            }

            u16 value_u16 = read_word(cpu->PC + 1);

            if (opcode == 0x08) {           // LD [a16], SP
                write_word(value_u16, cpu->SP);
                cpu->cycles += 20;
                cpu->PC += 3;
                break;
            }

            load_to_r16(dest_reg, value_u16);
            cpu->cycles += 12;
            cpu->PC += 3;
            break;
        }

        case 0xC1:              // POP BC
        case 0xD1:              // POP DE
        case 0xE1:              // POP HL
        case 0xF1: {            // POP AF
            u16 *dest_reg = NULL;

            switch (opcode) {
                case 0xC1: dest_reg = &cpu->BC; break;
                case 0xD1: dest_reg = &cpu->DE; break;
                case 0xE1: dest_reg = &cpu->HL; break;
                case 0xF1: dest_reg = &cpu->AF; break;
            }

            u16 value = read_word(cpu->SP);
            load_to_r16(dest_reg, value);

            cpu->SP += 2;           // Increment Stackpointer
            cpu->cycles += 12;
            cpu->PC += 1;
            break;
        }

        case 0xC5:              // PUSH BC
        case 0xD5:              // PUSH DE
        case 0xE5:              // PUSH HL
        case 0xF5: {            // PUSH AF
            u16 source_reg;

            switch (opcode) {
                case 0xC5: source_reg = cpu->BC; break;
                case 0xD5: source_reg = cpu->DE; break;
                case 0xE5: source_reg = cpu->HL; break;
                case 0xF5: source_reg = cpu->AF; break;
            }

            cpu->SP -= 2;           // Decrement Stackpointer
            u16 address = cpu->SP;
            write_word(address, source_reg);

            cpu->cycles += 16;
            cpu->PC += 1;
            break;
        }

        case 0xF8: {            // LD HL, SP + r8
            cpu->cycles += 12;

            i8 offset = read_byte_signed(cpu->PC + 1);
            u16 value = cpu->SP + offset;
            cpu->HL = value;

            CLEAR_ALL_FLAGS(cpu->F);
            if (value > 0xFFFF)
                SET_FLAG(cpu->F, FLAG_C);

            if (((cpu->SP & 0x0F) + (offset & 0x0F)) > 0x0F)
                SET_FLAG(cpu->F, FLAG_H);

            cpu->PC += 2;
            break;
        }

        case 0xF9: {            // LD SP, HL
            load_to_r16(&cpu->SP, cpu->HL);
            cpu->cycles += 8;
            cpu->PC += 1;
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

        case 0x1C: {            // INC E
            cpu->cycles += 4;
            u8 result = cpu->E + 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->E & 0x0F) == 0x0F) SET_FLAG(cpu->F, FLAG_H);

            cpu->PC += 1;
            break;
        }

        case 0x1D: {            // DEC E
            cpu->cycles += 4;
            u8 result = cpu->E - 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H);
            SET_FLAG(cpu->F, FLAG_N);

            if ((result & 0xFF) == 0) SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->E & 0x0F) == 0x10) SET_FLAG(cpu->F, FLAG_H);

            cpu->PC += 1;
            break;
        }

        // TODO: decoding of opcodes 0x80 - 0xBF
        case 0xA0 ... 0xA7: {   // AND r8
            u8 low_bit = opcode & SOURCE_REG_BIT;
            u8 *source_reg = get_register(cpu, low_bit);

            if (low_bit == 0b110) {
                and_hl(cpu, cpu->HL);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            and_r8(cpu, *source_reg);
            cpu->cycles += 4;
            cpu->PC += 1;
        }

        case 0xA8 ... 0xAF: {   // XOR r8
            u8 low_bit = opcode & SOURCE_REG_BIT;
            u8 *source_reg = get_register(cpu, low_bit);

            if (low_bit == 0b110) {
                xor_hl(cpu, cpu->HL);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            xor_r8(cpu, *source_reg);
            cpu->cycles += 4;
            cpu->PC += 1;
        }

        case 0xB0 ... 0xB7: {   // OR r8
            u8 low_bit = opcode & SOURCE_REG_BIT;
            u8 *source_reg = get_register(cpu, low_bit);

            if (low_bit == 0b110) {
                or_hl(cpu, cpu->HL);
                cpu->cycles += 8;
                cpu->PC += 1;
                break;
            }

            or_r8(cpu, *source_reg);
            cpu->cycles += 4;
            cpu->PC += 1;
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

            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H | FLAG_C);
            if ((cpu->HL & 0xFFF) + (cpu->BC & 0xFFF) > 0xFFF)      // Half Carry
                SET_FLAG(cpu->F, FLAG_H);

            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);

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

        case 0x19: {            // ADD HL, DE
            cpu->cycles += 8;
            u32 result = cpu->HL + cpu->DE;

            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H | FLAG_C);
            if ((cpu->HL & 0xFFF) + (cpu->DE & 0xFFF) > 0xFFF)      // Half Carry
                SET_FLAG(cpu->F, FLAG_H);

            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->HL = (u16)result;
            cpu->PC += 1;
            break;
        }

        case 0x1B: {            // DEC DE
            cpu->cycles += 8;
            cpu->DE -= 1;
            cpu->PC += 1;
            break;
        }

        case 0x23: {             // INC HL
            cpu->cycles += 8;
            cpu->HL += 1;
            cpu->PC += 1;
            break;
        }

        case 0x29: {            // ADD HL, HL
            cpu->cycles += 8;
            u32 result = cpu->HL + cpu->HL;

            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H | FLAG_C);
            if ((cpu->HL & 0xFFF) + (cpu->HL & 0xFFF) > 0xFFF)      // Half Carry
                SET_FLAG(cpu->F, FLAG_H);

            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->HL = (u16)result;
            cpu->PC += 1;
            break;
        }

        case 0x2B: {            // DEC HL
            cpu->cycles += 8;
            cpu->HL -= 1;
            cpu->PC += 1;
            break;
        }

        case 0x33: {            // INC SP
            cpu->cycles += 8;
            cpu->SP += 1;
            cpu->PC += 1;
            break;
        }

        case 0x39: {            // ADD HL, SP
            cpu->cycles += 8;
            u32 result = cpu->HL + cpu->SP;

            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H | FLAG_C);
            if ((cpu->HL & 0xFFF) + (cpu->SP & 0xFFF) > 0xFFF)      // Half Carry
                SET_FLAG(cpu->F, FLAG_H);

            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->HL = (u16)result;
            cpu->PC += 1;
            break;
        }

        case 0x3B: {            // DEC SP
            cpu->cycles += 8;
            cpu->SP -= 1;
            cpu->PC += 1;
            break;
        }

        case 0xE8: {            // ADD SP, r8
            cpu->cycles += 16;

            i8 value = read_byte_signed(cpu->PC + 1);
            u32 result = cpu->SP + value;

            CLEAR_ALL_FLAGS(cpu->F);
            if ((cpu->SP & 0x0F) + (cpu->SP & 0x0F) > 0x0F)        // Half Carry
                SET_FLAG(cpu->F, FLAG_H);

            if (((cpu->SP & 0xFF) + (value & 0xFF)) > 0xFF)        // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->SP = (u16)result;
            cpu->PC += 2;
            break;
        }




        // ============================================= //
        //             Bit shift instructions            //
        // ============================================= //

        case 0x07: {            // RLCA (Rotate Left Circular Accumulator)
            cpu->cycles += 4;
            u8 bit7 = (cpu->A & 0x80) >> 7;
            cpu->A = (cpu->A << 1) | bit7;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit7) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }

        case 0x0F: {            // RRCA (Rotate Right Circular Accumulator)
            cpu->cycles += 4;
            u8 bit0 = (cpu->A & 0x01) << 7;
            cpu->A = (cpu->A >> 1) | bit0;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit0) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }

        case 0x17: {            // RLA (Rotate Left Accumulator (through Carry flag))
            cpu->cycles += 4;
            u8 old_carry = (IS_FLAG_SET(cpu->F, FLAG_C) ? 1 : 0);
            u8 bit7 = (cpu->A & 0x80) >> 7;
            cpu->A = (cpu->A << 1) | old_carry;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit7) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }

        case 0x1F: {            // RRA (Rotate Right Accumulator (through Carry flag))
            cpu->cycles += 4;
            u8 old_carry = (IS_FLAG_SET(cpu->F, FLAG_C) ? 1 : 0);
            u8 bit0 = (cpu->A & 0x01) << 7;
            cpu->A = (cpu->A >> 1) | old_carry;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit0) SET_FLAG(cpu->F, FLAG_C);

            cpu->PC += 1;
            break;
        }




        // ============================================= //
        //       Jumps and subroutine instructions       //
        // ============================================= //

        case 0x18: {            // JR r8
            cpu->cycles += 12;
            i8 offset = read_byte_signed(cpu->PC + 1);
            cpu->PC += 2;               // Advance PC past the instuction
            cpu->PC += offset;          // Apply the offset
            break;
        }

        case 0x20: {            // JR NZ, r8
            if (IS_FLAG_CLEAR(cpu->F, FLAG_Z)) {        // if Z-flag is clear move to the offset
                cpu->cycles += 12;
                i8 offset = read_byte_signed(cpu->PC + 1);
                cpu->PC += 2;
                cpu->PC += offset;
            } else {                                    // else just increment cycles and PC
                cpu->cycles += 8;
                cpu->PC += 2;
            }
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

    if (cpu->ime_delay > 0) {
        cpu->ime_delay--;
        if (cpu->ime_delay == 0) {
            cpu->ime = true;
        }
    }
}
