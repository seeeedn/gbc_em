#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

#define MEM_SIZE 0x10000
extern u8 memory[MEM_SIZE];

#define FLAG_Z (1 << 7)     // 7th bit of the F register
#define FLAG_N (1 << 6)     // 6th bit of the F register
#define FLAG_H (1 << 5)     // 5th bit of the F register
#define FLAG_C (1 << 4)     // 4th bit of the F register

#define SET_FLAG(reg, flag)         ((reg) |= (flag))
#define CLEAR_FLAG(reg, flag)       ((reg) &= ~(flag))
#define CLEAR_ALL_FLAGS(reg)        (reg = 0)
#define IS_FLAG_SET(reg, flag)      (((reg) & (flag)) != 0)
#define IS_FLAG_CLEAR(reg, flag)    (((reg) & (flag)) == 0)

#define DEST_REG_BIT (7 << 3)   // 0b111000
#define SOURCE_REG_BIT 7        // 0b000111

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
            u8 C;           // 8-bit General Purpose Register
            u8 B;
        };
        u16 BC;             // 16-bit register comprised out of two 8-bit registers
    };

    union {
        struct {
            u8 E;
            u8 D;
        };
        u16 DE;             // 16-bit register comprised out of two 8-bit registers
    };

    union {
        struct {
            u8 L;
            u8 H;
        };
        u16 HL;             // 16-bit register comprised out of two 8-bit registers
    };

    u16 SP;                 // Stackpointer register

    u16 PC;                 // Program Counter
    u64 cycles;             // CPU cycles passed

    bool ime;               // "Interrupt Master Enable"
    u64 ime_delay;          // CPU-steps until IME is enabled after EI-Instruction is being executed

    bool halted;
    bool stopped;
} CPU;

void init_cpu(CPU *cpu);
void cpu_step(CPU *cpu);

#endif
