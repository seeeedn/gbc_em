#include "cpu.h"
#include "mmu.h"
#include <stdio.h>
#include <stdlib.h>

static const u8 cycles_table[256] = {
      //0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
        4, 12,  8,  8,  4,  4,  8,  4, 20,  8,  8,  8,  4,  4,  8,  4, //0
        4, 12,  8,  8,  4,  4,  8,  4, 12,  8,  8,  8,  4,  4,  8,  4, //1
        8, 12,  8,  8,  4,  4,  8,  4,  8,  8,  8,  8,  4,  4,  8,  4, //2
        8, 12,  8,  8, 12, 12, 12,  4,  8,  8,  8,  8,  4,  4,  8,  4, //3

        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //4
        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //5
        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //6
        8,  8,  8,  8,  8,  8,  4,  8,  4,  4,  4,  4,  4,  4,  8,  4, //7

        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //8
        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //9
        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //A
        4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4, //B

        8,  12, 12, 16, 12, 16, 8, 16,  8, 16, 12,  0, 12, 24,  8, 16, //C
        8,  12, 12, 00, 12, 16, 8, 16,  8, 16, 12, 00, 12, 00,  8, 16, //D
        12, 12,  8, 00, 00, 16, 8, 16, 16,  4, 16, 00, 00, 00,  8, 16, //E
        12, 12,  8,  4, 00, 16, 8, 16, 12,  8, 16,  4, 00, 00,  8, 16  //F
};

static const u8 cb_cycles_table[256] = {
      //0 1 2 3 4 5  6 7 8 9 A B C D  E F
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //0
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //1
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //2
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //3

        8,8,8,8,8,8,12,8,8,8,8,8,8,8,12,8, //4
        8,8,8,8,8,8,12,8,8,8,8,8,8,8,12,8, //5
        8,8,8,8,8,8,12,8,8,8,8,8,8,8,12,8, //6
        8,8,8,8,8,8,12,8,8,8,8,8,8,8,12,8, //7

        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //8
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //9
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //A
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //B

        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //C
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //D
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8, //E
        8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8  //F
    };

void init_cpu(CPU *cpu) {
    cpu->AF = 0x01B0;
    cpu->BC = 0x0013;
    cpu->DE = 0x00D8;
    cpu->HL = 0x014D;
    cpu->SP = 0xFFFE;
    cpu->PC = 0x0100;
    cpu->ime_delay = 0;
    cpu->ime = 0;
    cpu->halted = false;
    cpu->stopped = false;
    cpu->total_cycles = 0;
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

static void xor_r8(CPU *cpu, u8 reg) {
    u8 result = cpu->A ^ reg;

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

static void cp_r8(CPU *cpu, u8 reg) {
    u8 result = cpu->A - reg;

    CLEAR_ALL_FLAGS(cpu->F);
    SET_FLAG(cpu->F, FLAG_N);

    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);
    if ((cpu->A & 0x0F) < (reg & 0x0F))
        SET_FLAG(cpu->F, FLAG_H);
    if (cpu->A < reg)
        SET_FLAG(cpu->F, FLAG_C);
}

// ============================================= //
//                 Other Helpers                 //
// ============================================= //
static inline void update_flags_zc(CPU *cpu, u8 result, u8 carry) {
    if (result == 0)
        SET_FLAG(cpu->F, FLAG_Z);
    if (carry)
        SET_FLAG(cpu->F, FLAG_C);
}

static inline void write_result(CPU *cpu, u8 *dest_reg, u8 value) {
    if (dest_reg == NULL) {
        mmu_write_byte(cpu->HL, value);
    } else {
        *dest_reg = value;
    }
}

// executes the instruction of the given opcode (with cb prefix)
static u8 execute_cb_instruction(CPU *cpu, u8 cb_opcode) {
    u8 *r8_lookup[] = { &cpu->B, &cpu->C, &cpu->D, &cpu->E, &cpu->H, &cpu->L, NULL, &cpu->A };
    u16 *r16_lookup[] = { &cpu->BC, &cpu->DE, &cpu->HL, &cpu->SP };

    // pre calculate variables
    u8 target_bit_nr = (cb_opcode & DEST_REG_BIT) >> 3;
    u8 target_bit = (1 << target_bit_nr);
    u8 dest_reg_bit = cb_opcode & SOURCE_REG_BIT;
    u8 *dest_reg = r8_lookup[dest_reg_bit];

    u8 carry_flag = IS_FLAG_SET(cpu->F, FLAG_C);

    u8 value = (dest_reg == NULL) ? mmu_read_byte(cpu->HL) : *dest_reg;
    u8 bit_0 = value & 1;
    u8 bit_7 = value & (1 << 7);

    switch (cb_opcode) {

        case 0x00 ... 0x07: {       // RLC r8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 bit_7_set = (bit_7 ? 1 : 0);
            u8 mod_value = (value << 1) | bit_7_set;

            update_flags_zc(cpu, mod_value, bit_7);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x08 ... 0x0F: {       // RRC r8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 bit_0_set = bit_0 << 7;
            u8 mod_value = (value >> 1) | bit_0_set;

            update_flags_zc(cpu, mod_value, bit_0);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x10 ... 0x17: {       // RL r8
            u8 carry_set = IS_FLAG_SET(cpu->F, FLAG_C) ? 0x01 : 0x00;
            CLEAR_ALL_FLAGS(cpu->F);

            u8 mod_value = (value << 1) | carry_set;

            update_flags_zc(cpu, mod_value, bit_7);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x18 ... 0x1F: {       // RR r8
            u8 carry_set = IS_FLAG_SET(cpu->F, FLAG_C) ? 0x80 : 0x00;
            CLEAR_ALL_FLAGS(cpu->F);

            u8 mod_value = (value >> 1) | carry_set;

            update_flags_zc(cpu, mod_value, bit_0);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x20 ... 0x27: {       // SLA r8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 mod_value = value << 1;

            update_flags_zc(cpu, mod_value, bit_7);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x28 ... 0x2F: {       // SRA r8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 mod_value = (value >> 1) | bit_7;

            update_flags_zc(cpu, mod_value, bit_0);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x30 ... 0x37: {       // SWAP r8/m8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 new_upper_nibble = (value & 0x0F) << 4;
            u8 new_lower_nibble = (value & 0xF0) >> 4;
            u8 mod_value = new_lower_nibble | new_upper_nibble;

            if (mod_value == 0)
                SET_FLAG(cpu->F, FLAG_Z);

            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x38 ... 0x3F: {       // SRL r8/m8
            CLEAR_ALL_FLAGS(cpu->F);

            u8 mod_value = value >> 1;

            update_flags_zc(cpu, mod_value, bit_0);
            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0x40 ... 0x7F: {       // BIT Bit, r8/m8
            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N);
            SET_FLAG(cpu->F, FLAG_H);

            if (!IS_BIT_SET(value, target_bit))
                SET_FLAG(cpu->F, FLAG_Z);

            cpu->total_cycles += (dest_reg == NULL) ? 12 : 8;
            break;
        }

        case 0x80 ... 0xBF: {       // RES Bit, r8/m8
            u8 mod_value = value & ~target_bit;

            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        case 0xC0 ... 0xFF: {       // SET x, r8
            u8 mod_value = value | target_bit;

            write_result(cpu, dest_reg, mod_value);

            cpu->total_cycles += (dest_reg == NULL) ? 16 : 8;
            break;
        }

        default: {                  // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X (CB-prefixed)\n", cb_opcode, cpu->PC);
            exit(1);
        }
    }
    cpu->PC += 2;
    return cb_cycles_table[cb_opcode];
}

// executes the instruction of the given opcode (without cb prefix)
u8 execute_instruction(CPU *cpu, u8 opcode) {
    u8 *r8_lookup[] = { &cpu->B, &cpu->C, &cpu->D, &cpu->E, &cpu->H, &cpu->L, NULL, &cpu->A };
    u16 *r16_lookup[] = { &cpu->BC, &cpu->DE, &cpu->HL, &cpu->SP };

    u8 opt_cyc = 0;
    u8 cb_cycles = 0;

    switch (opcode) {
        // ============================================= //
        //           Misc/Control instructions           //
        // ============================================= //

        case 0x00: {            // NOP (No operation)
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x10: {            // STOP 0 (Stop CPU)
            cpu->stopped = true;
            cpu->total_cycles += 4;
            cpu->PC += 2;
            break;
        }

        case 0x76: {            // HALT
            cpu->halted = true;
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xCB: {            // PREFIX CB
            u8 cb_opcode = mmu_read_byte(cpu->PC + 1);
            cb_cycles = execute_cb_instruction(cpu, cb_opcode);
            cpu->total_cycles += 4;
            break;
        }

        case 0xF3: {            // DI (Disable Interrupts)
            cpu->ime = false;
            cpu->ime_delay = 0;
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0xFB: {            // EI (Enable Interrupts)
            cpu->ime_delay = 2;
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }




        // ============================================= //
        //            8-bit Load Instructions            //
        // ============================================= //

        case 0x02:              // LD [BC], A
        case 0x0A:              // LD A, [BC]
        case 0x12:              // LD [DE], A
        case 0x1A: {            // LD A, [DE]
            u8 register_bit_mask = (opcode & (3 << 4)) >> 4;
            u8 mode_bit_mask = opcode & 0x0F;

            u16 *dest_reg = r16_lookup[register_bit_mask];
            u16 address = *dest_reg;

            bool is_load_from = (mode_bit_mask == 0x0A);

            if (is_load_from) {
                u8 value = mmu_read_byte(address);
                cpu->A = value;
            } else {
                mmu_write_byte(address, cpu->A);
            }

            cpu->total_cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0x22:              // LD [HL+], A
        case 0x2A:              // LD A, [HL+]
        case 0x32:              // LD [HL-], A
        case 0x3A: {            // LD A, [HL-]
            i8 mode = IS_BIT_SET(opcode, 0x10) ? -1 : 1;

            if (IS_BIT_SET(opcode, 0x08)) {        // dest_reg = A
                cpu->A = mmu_read_byte(cpu->HL);
            } else {                               // dest_reg = HL
                mmu_write_byte(cpu->HL, cpu->A);
            }
            cpu->HL += mode;

            cpu->total_cycles += 8;
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
            u8 *dest_reg = r8_lookup[register_bit_mask];

            u8 value = mmu_read_byte(cpu->PC + 1);
            write_result(cpu, dest_reg, value);

            cpu->total_cycles += (dest_reg == NULL) ? 12 : 8;
            cpu->PC += 2;
            break;
        }

        case 0x40 ... 0x75:
        case 0x77 ... 0x7F: {   // LD r8, (r8 / HL)
            u8 dest_reg_bit = (opcode & DEST_REG_BIT) >> 3;
            u8 src_reg_bit = opcode & SOURCE_REG_BIT;
            u8 *dest_reg = r8_lookup[dest_reg_bit];
            u8 *src_reg = r8_lookup[src_reg_bit];

            if (dest_reg == NULL) {
                mmu_write_byte(cpu->HL, *src_reg);
            }
            else if (src_reg == NULL) {
                *dest_reg = mmu_read_byte(cpu->HL);
            } else {
                *dest_reg = *src_reg;
            }

            cpu->total_cycles += (dest_reg == NULL || src_reg == NULL) ? 8 : 4;
            cpu->PC += 1;
            break;
        }

        case 0xE0:              // LDH (0xFF00 + a8), A
        case 0xF0: {            // LDH A, (0xFF00 + a8)
            u8 offset = mmu_read_byte(cpu->PC + 1);
            u16 address = 0xFF00 + offset;

            if (opcode == 0xE0) {
                mmu_write_byte(address, cpu->A);
            } else {
                u8 value = mmu_read_byte(address);
                cpu->A = value;
            }

            cpu->total_cycles += 12;
            cpu->PC += 2;
            break;
        }

        case 0xE2:              // LD (0xFF00 + C), A
        case 0xF2: {            // LD A, (0xFF00 + C)
            u16 address = 0xFF00 + cpu->C;

            if (opcode == 0xE2) {
                mmu_write_byte(address, cpu->A);
            } else {
                u8 value = mmu_read_byte(address);
                cpu->A = value;
            }

            cpu->total_cycles += 8;
            cpu->PC += 2;
            break;
        }

        case 0xEA:              // LDH (a16), A
        case 0xFA: {            // LDH A, (a16)
            u16 address = mmu_read_word(cpu->PC + 1);

            if (opcode == 0xEA) {
                mmu_write_byte(address, cpu->A);
            } else {
                u8 value = mmu_read_byte(address);
                cpu->A = value;
            }

            cpu->total_cycles += 16;
            cpu->PC += 3;
            break;
        }



        // ============================================= //
        //            16-bit Load Instructions           //
        // ============================================= //

        case 0x01:              // LD BC, d16
        case 0x11:              // LD DE, d16
        case 0x21:              // LD HL, d16
        case 0x31: {            // LD SP, d16
            u8 reg_index = (opcode & 0x30) >> 4;
            u16 *dest_reg = r16_lookup[reg_index];

            u16 value_u16 = mmu_read_word(cpu->PC + 1);
            *dest_reg = value_u16;

            cpu->total_cycles += 12;
            cpu->PC += 3;
            break;
        }

        case 0x08: {            // LD (a16), SP
            u16 value_u16 = mmu_read_word(cpu->PC + 1);
            mmu_write_word(value_u16, cpu->SP);
            cpu->total_cycles += 20;
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

            u16 value_u16 = mmu_read_word(cpu->SP);
            cpu->SP += 2;           // Increment Stackpointer

            *dest_reg = value_u16;

            if (opcode == 0xF1)
                *dest_reg &= 0xFFF0;    // enforce that lower nibble of F == 0

            cpu->total_cycles += 12;
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
            mmu_write_word(address, source_reg);

            cpu->total_cycles += 16;
            cpu->PC += 1;
            break;
        }

        case 0xF8: {            // LD HL, SP + r8
            i8 offset = (i8)mmu_read_byte(cpu->PC + 1);
            u16 value = cpu->SP + offset;

            cpu->HL = value;

            CLEAR_ALL_FLAGS(cpu->F);
            if (((cpu->SP & 0xFF) + (offset & 0xFF)) > 0xFF)
                SET_FLAG(cpu->F, FLAG_C);
            if (((cpu->SP & 0x0F) + (offset & 0x0F)) > 0x0F)
                SET_FLAG(cpu->F, FLAG_H);

            cpu->total_cycles += 12;
            cpu->PC += 2;
            break;
        }

        case 0xF9: {            // LD SP, HL
            cpu->SP = cpu->HL;
            cpu->total_cycles += 8;
            cpu->PC += 1;
            break;
        }



        // ============================================= //
        //         8-bit arithmetic instructions         //
        // ============================================= //

        case 0x04:
        case 0x14:
        case 0x24:
        case 0x34:              // INC (HL)
        case 0x0C:
        case 0x1C:
        case 0x2C:
        case 0x3C: {            // INC r8
            u8 reg_index = (opcode & DEST_REG_BIT) >> 3;
            u8 *dest_reg = r8_lookup[reg_index];

            u8 old_value = (dest_reg == NULL) ? mmu_read_byte(cpu->HL) : *dest_reg;
            u8 new_value = old_value + 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_N | FLAG_H);
            if (new_value == 0)
                SET_FLAG(cpu->F, FLAG_Z);
            if ((old_value & 0x0F) + 1 > 0x0F)
                SET_FLAG(cpu->F, FLAG_H);

            write_result(cpu, dest_reg, new_value);

            cpu->total_cycles += (dest_reg == NULL) ? 12 : 4;
            cpu->PC += 1;
            break;
        }

        case 0x05:
        case 0x15:
        case 0x25:
        case 0x35:              // DEC (HL)
        case 0x0D:
        case 0x1D:
        case 0x2D:
        case 0x3D: {            // DEC r8
            u8 reg_index = (opcode & DEST_REG_BIT) >> 3;
            u8 *dest_reg = r8_lookup[reg_index];

            u8 old_value = (dest_reg == NULL) ? mmu_read_byte(cpu->HL) : *dest_reg;
            u8 new_value = old_value - 1;

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H);
            SET_FLAG(cpu->F, FLAG_N);

            if (new_value == 0)
                SET_FLAG(cpu->F, FLAG_Z);
            if ((old_value & 0x0F) == 0)
                SET_FLAG(cpu->F, FLAG_H);

            write_result(cpu, dest_reg, new_value);

            cpu->total_cycles += (dest_reg == NULL) ? 12 : 4;
            cpu->PC += 1;
            break;
        }

        case 0x27: {            // DAA "Decimal adjust accumulator"
            // Adjust Accumulator for BCD-Usage
            // Refer to z80 Documentation for further explanation

            bool was_add = IS_FLAG_CLEAR(cpu->F, FLAG_N);
            bool carry_set = IS_FLAG_SET(cpu->F, FLAG_C);
            bool halfcarry_set = IS_FLAG_SET(cpu->F, FLAG_H);

            u8 correction = 0;
            bool set_carry = false;

            if (was_add) {
                if (halfcarry_set || (cpu->A & 0x0F) > 9) {
                    correction |= 0x06;
                }
                if (carry_set || cpu->A > 0x99) {
                    correction |= 0x60;
                    set_carry = true;
                }
                cpu->A += correction;
            } else {
                if (halfcarry_set) {
                    correction |= 0x06;
                }
                if (carry_set) {
                    correction |= 0x60;
                }
                cpu->A -= correction;
                if (carry_set) set_carry = true;
            }

            CLEAR_FLAG(cpu->F, FLAG_Z | FLAG_H | FLAG_C);

            if (cpu->A == 0)
                SET_FLAG(cpu->F, FLAG_Z);
            if (set_carry)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x2F: {            // CPL "Complement accumulator"
            SET_FLAG(cpu->F, FLAG_N | FLAG_H);
            cpu->A = ~cpu->A;
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x37: {            // SCF "Set carry flag"
            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H);
            SET_FLAG(cpu->F, FLAG_C);
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x3F: {            // CCF "Complement carry flag"
            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H);
            cpu->F ^= FLAG_C;
            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x80 ... 0x8F: {   // ADD r8, r8 / ADC r8, r8
            u8 src_reg_index = opcode & SOURCE_REG_BIT;
            u8 *src_reg = r8_lookup[src_reg_index];
            u8 value = (src_reg == NULL) ? mmu_read_byte(cpu->HL) : *src_reg;

            bool is_adc = opcode > 0x87;
            u8 carry = is_adc & IS_FLAG_SET(cpu->F, FLAG_C);

            u16 result = cpu->A + value + carry;

            CLEAR_ALL_FLAGS(cpu->F);
            if ((u8)result == 0)
                SET_FLAG(cpu->F, FLAG_Z);
            if ((cpu->A & 0x0F) + (value & 0x0F) + (carry & 0x0F) > 0x0F)
                SET_FLAG(cpu->F, FLAG_H);
            if (result > 0xFF)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->A = (u8)result;
            cpu->total_cycles += (src_reg == NULL) ? 8 : 4;
            cpu->PC += 1;
            break;
        }

        case 0xC6:              // ADD A, d8
        case 0xCE: {            // ADC A, d8
            u8 value = mmu_read_byte(cpu->PC + 1);

            bool is_adc = opcode == 0xCE;
            u8 carry = is_adc & IS_FLAG_SET(cpu->F, FLAG_C);

            u16 result = cpu->A + value + carry;

            CLEAR_ALL_FLAGS(cpu->F);
            if ((u8)result == 0)
                SET_FLAG(cpu->F, FLAG_Z);
            if (((cpu->A & 0x0F) + (value & 0x0F) + carry) > 0x0F)
                SET_FLAG(cpu->F, FLAG_H);
            if (result > 0xFF)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->A = (u8)result;
            cpu->total_cycles += 8;
            cpu->PC += 2;
            break;
        }

        case 0x90 ... 0x9F: {   // SUB A, r8 / SBC A, r8
            u8 src_reg_index = opcode & SOURCE_REG_BIT;
            u8 *src_reg = r8_lookup[src_reg_index];
            u8 value = (src_reg == NULL) ? mmu_read_byte(cpu->HL) : *src_reg;

            bool is_sbc = opcode > 0x97;
            u8 carry = is_sbc & IS_FLAG_SET(cpu->F, FLAG_C);

            u16 result = cpu->A - (value + carry);

            CLEAR_ALL_FLAGS(cpu->F);
            SET_FLAG(cpu->F, FLAG_N);

            if ((u8)result == 0)
                SET_FLAG(cpu->F, FLAG_Z);

            u8 halfcarry = ((cpu->A & 0x0F) - (value & 0x0F) - (carry & 0x0F)) & 0x10;
            if (halfcarry)
                SET_FLAG(cpu->F, FLAG_H);

            if (cpu->A < (value + carry))
                SET_FLAG(cpu->F, FLAG_C);

            cpu->A = (u8)result;
            cpu->total_cycles += (src_reg == NULL) ? 8 : 4;
            cpu->PC += 1;
            break;
        }

        case 0xD6:              // SUB A, r8
        case 0xDE: {            // SBC A, r8
            u8 value = mmu_read_byte(cpu->PC + 1);
            bool is_sbc = opcode == 0xDE;
            u8 carry = (is_sbc && IS_FLAG_SET(cpu->F, FLAG_C)) ? 1 : 0;

            u8 result = cpu->A - (value + carry);

            CLEAR_ALL_FLAGS(cpu->F);
            SET_FLAG(cpu->F, FLAG_N);

            if (result == 0)
                SET_FLAG(cpu->F, FLAG_Z);;

            u8 halfcarry = ((cpu->A & 0x0F) - (value & 0x0F) - (carry & 0x0F)) & 0x10;
            if (halfcarry)
                SET_FLAG(cpu->F, FLAG_H);

            if (cpu->A < (value + carry))
                SET_FLAG(cpu->F, FLAG_C);

            cpu->A = result;
            cpu->total_cycles += 8;
            cpu->PC += 2;
            break;
        }

        case 0xA0 ... 0xBF: {   // AND r8 / XOR r8 / OR r8 / CP r8
            u8 src_reg_index = opcode & SOURCE_REG_BIT;
            u8 *src_reg = r8_lookup[src_reg_index];
            u8 value = (src_reg == NULL) ? mmu_read_byte(cpu->HL) : *src_reg;
            u8 opcode_index = (opcode & 0x18) >> 3;

            switch (opcode_index) {
                case 0: {
                    and_r8(cpu, value);
                    break;
                }
                case 1: {
                    xor_r8(cpu, value);
                    break;
                }
                case 2: {
                    or_r8(cpu, value);
                    break;
                }
                case 3: {
                    cp_r8(cpu, value);
                    break;
                }
            }

            cpu->total_cycles += (src_reg == NULL) ? 8 : 4;
            cpu->PC += 1;
            break;
        }

        case 0xE6:              // AND d8
        case 0xEE:              // XOR d8
        case 0xF6:              // OR d8
        case 0xFE: {            // CP d8
            u8 value = mmu_read_byte(cpu->PC + 1);
            u8 opcode_index = (opcode & 0x18) >> 3;

            switch (opcode_index) {
                case 0: {
                    and_r8(cpu, value);
                    break;
                }
                case 1: {
                    xor_r8(cpu, value);
                    break;
                }
                case 2: {
                    or_r8(cpu, value);
                    break;
                }
                case 3: {
                    cp_r8(cpu, value);
                    break;
                }
            }

            cpu->total_cycles += 8;
            cpu->PC += 2;
            break;
        }


        // ============================================= //
        //         16-bit arithmetic instructions        //
        // ============================================= //

        case 0x03:
        case 0x13:
        case 0x23:
        case 0x33: {            // INC r16
            u8 reg_index = (opcode & 0x30) >> 4;
            *r16_lookup[reg_index] += 1;
            cpu->total_cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0x0B:
        case 0x1B:
        case 0x2B:
        case 0x3B: {            // DEC r16
            u8 reg_index = (opcode & 0x30) >> 4;
            *r16_lookup[reg_index] -= 1;
            cpu->total_cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0x09:
        case 0x19:
        case 0x29:
        case 0x39: {            // ADD HL, r16
            u8 reg_index = (opcode & 0x30) >> 4;
            u16 dest_reg = *r16_lookup[reg_index];

            u32 result = cpu->HL + dest_reg;

            CLEAR_FLAG(cpu->F, FLAG_N | FLAG_H | FLAG_C);
            if ((cpu->HL & 0xFFF) + (dest_reg & 0xFFF) > 0xFFF)     // Half Carry
                SET_FLAG(cpu->F, FLAG_H);
            if (result > 0xFFFF)                                    // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->HL = (u16)result;
            cpu->total_cycles += 8;
            cpu->PC += 1;
            break;
        }

        case 0xE8: {            // ADD SP, r8
            i8 value = (i8)mmu_read_byte(cpu->PC + 1);
            u16 result = cpu->SP + value;

            CLEAR_ALL_FLAGS(cpu->F);
            if (((cpu->SP & 0x0F) + (value & 0x0F)) > 0x0F)        // Half Carry
                SET_FLAG(cpu->F, FLAG_H);
            if (((cpu->SP & 0xFF) + (value & 0xFF)) > 0xFF)        // Carry
                SET_FLAG(cpu->F, FLAG_C);

            cpu->SP = result;
            cpu->total_cycles += 16;
            cpu->PC += 2;
            break;
        }




        // ============================================= //
        //             Bit shift instructions            //
        // ============================================= //

        case 0x07: {            // RLCA (Rotate Left Circular Accumulator)
            u8 bit_7 = (cpu->A & 0x80) >> 7;
            cpu->A = (cpu->A << 1) | bit_7;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit_7)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x0F: {            // RRCA (Rotate Right Circular Accumulator)
            u8 bit_0 = (cpu->A & 0x01) << 7;
            cpu->A = (cpu->A >> 1) | bit_0;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit_0)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x17: {            // RLA (Rotate Left Accumulator (through Carry flag))
            u8 old_carry = (IS_FLAG_SET(cpu->F, FLAG_C) ? 1 : 0);
            u8 bit_7 = cpu->A & 0x80;
            cpu->A = (cpu->A << 1) | old_carry;

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit_7)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }

        case 0x1F: {            // RRA (Rotate Right Accumulator (through Carry flag))
            u8 old_carry = (IS_FLAG_SET(cpu->F, FLAG_C) ? 1 : 0);
            u8 bit_0 = cpu->A & 0x01;
            cpu->A = (cpu->A >> 1) | (old_carry << 7);

            CLEAR_ALL_FLAGS(cpu->F);
            if (bit_0)
                SET_FLAG(cpu->F, FLAG_C);

            cpu->total_cycles += 4;
            cpu->PC += 1;
            break;
        }




        // ============================================= //
        //            Jump and Call instructions         //
        // ============================================= //

        case 0x18: {            // JR i8
            i8 offset = (i8)mmu_read_byte(cpu->PC + 1);
            cpu->total_cycles += 12;
            cpu->PC += 2 + offset;               // Advance PC past the instuction and apply the offset
            break;
        }

        case 0x20:
        case 0x28:
        case 0x30:
        case 0x38: {            // JR cond, i8
            u8 cond = (opcode & 0x18) >> 3;
            i8 offset = (i8)mmu_read_byte(cpu->PC + 1);

            if ((cond == 0 && IS_FLAG_CLEAR(cpu->F, FLAG_Z)) ||
                (cond == 1 && IS_FLAG_SET(cpu->F, FLAG_Z)) ||
                (cond == 2 && IS_FLAG_CLEAR(cpu->F, FLAG_C)) ||
                (cond == 3 && IS_FLAG_SET(cpu->F, FLAG_C)))
            {
                opt_cyc += 4;
                cpu->total_cycles += 12;
                cpu->PC += 2 + offset;
            } else {
                cpu->total_cycles += 8;
                cpu->PC += 2;
            }
            break;
        }

        case 0xC3:              // JP a16
        case 0xE9: {            // JP (HL)
            u16 address = (opcode == 0xC3) ? mmu_read_word(cpu->PC + 1) : cpu->HL;
            cpu->PC = address;
            cpu->total_cycles += (opcode == 0xC3) ? 16 : 4;
            break;
        }

        case 0xC2:
        case 0xCA:
        case 0xD2:
        case 0xDA: {            // JP cond, a16
            u8 cond = (opcode & 0x18) >> 3;
            u16 address = mmu_read_word(cpu->PC + 1);

            if ((cond == 0 && IS_FLAG_CLEAR(cpu->F, FLAG_Z)) ||
                (cond == 1 && IS_FLAG_SET(cpu->F, FLAG_Z)) ||
                (cond == 2 && IS_FLAG_CLEAR(cpu->F, FLAG_C)) ||
                (cond == 3 && IS_FLAG_SET(cpu->F, FLAG_C)))
            {
                opt_cyc += 4;
                cpu->PC = address;
                cpu->total_cycles += 16;
            } else {
                cpu->total_cycles += 12;
                cpu->PC += 3;
            }
            break;
        }

        case 0xCD: {            // CALL a16
            u16 ret_addr = cpu->PC + 3;
            u16 call_addr = mmu_read_word(cpu->PC + 1);

            cpu->SP -= 2;
            mmu_write_word(cpu->SP, ret_addr);

            cpu->PC = call_addr;
            cpu->total_cycles += 24;
            break;
        }

        case 0xC4:
        case 0xCC:
        case 0xD4:
        case 0xDC: {            // CALL cond, a16
            u8 cond = (opcode & 0x18) >> 3;
            u16 ret_addr = cpu->PC + 3;
            u16 call_addr = mmu_read_word(cpu->PC + 1);

            if ((cond == 0 && IS_FLAG_CLEAR(cpu->F, FLAG_Z)) ||
                (cond == 1 && IS_FLAG_SET(cpu->F, FLAG_Z)) ||
                (cond == 2 && IS_FLAG_CLEAR(cpu->F, FLAG_C)) ||
                (cond == 3 && IS_FLAG_SET(cpu->F, FLAG_C)))
            {
                cpu->SP -= 2;
                mmu_write_word(cpu->SP, ret_addr);
                cpu->PC = call_addr;
                opt_cyc += 12;
                cpu->total_cycles += 24;
            } else {
                cpu->total_cycles += 12;
                cpu->PC += 3;
            }
            break;
        }

        case 0xC9:              // RET
        case 0xD9: {            // RETI
            u16 ret_addr = mmu_read_word(cpu->SP);
            cpu->SP += 2;
            cpu->PC = ret_addr;

            if (opcode == 0xD9) {
                cpu->ime = true;
            }

            cpu->total_cycles += 16;
            break;
        }

        case 0xC0:
        case 0xC8:
        case 0xD0:
        case 0xD8: {            // RET cond
            u8 cond = (opcode & 0x18) >> 3;

            if ((cond == 0 && IS_FLAG_CLEAR(cpu->F, FLAG_Z)) ||
                (cond == 1 && IS_FLAG_SET(cpu->F, FLAG_Z)) ||
                (cond == 2 && IS_FLAG_CLEAR(cpu->F, FLAG_C)) ||
                (cond == 3 && IS_FLAG_SET(cpu->F, FLAG_C)))
            {
                u16 ret_addr = mmu_read_word(cpu->SP);
                cpu->SP += 2;
                cpu->PC = ret_addr;
                opt_cyc += 12;
                cpu->total_cycles += 20;
            } else {
                cpu->total_cycles += 8;
                cpu->PC += 1;
            }
            break;
        }

        case 0xC7:
        case 0xCF:
        case 0xD7:
        case 0xDF:
        case 0xE7:
        case 0xEF:
        case 0xF7:
        case 0xFF: {            // RST 00H-30H / 08H-38H
            u16 rst_addr = opcode & DEST_REG_BIT;

            cpu->SP -= 2;
            mmu_write_word(cpu->SP, cpu->PC + 1);

            cpu->PC = rst_addr;
            cpu->total_cycles += 16;
            break;
        }

        default: {              // Error Case
            printf("Unknown opcode: 0x%02X at PC=0x%04X\n", opcode, cpu->PC);
            exit(1);
        }
    }
    return cycles_table[opcode] + opt_cyc + cb_cycles;
}

u8 handle_interrupt(CPU *cpu) {
    if (!cpu->ime)
        return 0;

    u8 interrupts = io_regs[IF_ADDRESS] & interrupts_enabled;
    if (interrupts == 0)
        return 0;

    static const u8 call_vec[5] = { 0x40, 0x48, 0x50, 0x58, 0x60 };

    for (int i = 0; i < 5; i++) {
        if (interrupts & (1 << i)) {
            cpu->ime = false;
            cpu->halted = false;

            io_regs[IF_ADDRESS] &= ~(1 << i);

            cpu->SP -= 2;
            mmu_write_word(cpu->SP, cpu->PC);

            cpu->PC = call_vec[i];

            cpu->total_cycles += 20;
            return 20;
        }
    }
    return 0;
}
