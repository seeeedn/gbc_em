#ifndef CPU_H
#define CPU_H

#include "utils.h"
#include "mmu.h"

#define FLAG_Z (1 << 7)     // 7th bit of the F register
#define FLAG_N (1 << 6)     // 6th bit of the F register
#define FLAG_H (1 << 5)     // 5th bit of the F register
#define FLAG_C (1 << 4)     // 4th bit of the F register

#define SET_FLAG(reg, flag)         ((reg) |= (flag))
#define CLEAR_FLAG(reg, flag)       ((reg) &= ~(flag))
#define CLEAR_ALL_FLAGS(reg)        ((reg) = 0)
#define IS_FLAG_SET(reg, flag)      (((reg) & (flag)) != 0)
#define IS_FLAG_CLEAR(reg, flag)    (((reg) & (flag)) == 0)

#define IS_BIT_SET(reg, bit)        (((reg) & (bit)) != 0)
#define IS_BIT_CLEAR(reg, bit)      (((reg) & (bit)) == 0)

#define DEST_REG_BIT    0x38    // 0b00111000
#define SOURCE_REG_BIT  0x07    // 0b00000111

typedef struct {

    union {                 // 8-bit registers have to be swapped, because of little endian
        struct {
            u8 F;           // Flag register
            u8 A;           // 8-bit Accumulator register
        };
        u16 AF;             // Special 16-bit register comprised out of the Flag register and the Accumulator register
    };

    union {
        struct {
            u8 C;           // 8-bit register
            u8 B;
        };
        u16 BC;             // 16-bit register comprised out of two 8-bit registers
    };

    union {
        struct {
            u8 E;
            u8 D;
        };
        u16 DE;
    };

    union {
        struct {
            u8 L;
            u8 H;
        };
        u16 HL;
    };

    u16 SP;                 // Stackpointer register
    u16 PC;                 // Program Counter

    u64 total_cycles;       // CPU cycles passed since power on

    u8 ime_delay;           // CPU-steps until IME is enabled after EI-Instruction is being executed
    bool ime;

    bool halted;
    bool stopped;
} CPU;

void init_cpu(CPU *cpu);

u8 execute_instruction(CPU *cpu, u8 opcode);
u8 execute_cb_instruction(CPU *cpu, u8 cb_opcode);
u8 handle_interrupt(CPU *cpu);

#endif
