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
#define IS_FLAG_SET(reg, flag)      (((reg) & (flag)) != 0)
#define IS_FLAG_CLEAR(reg, flag)    (((reg) & (flag)) == 0)

typedef struct {

    union {
        struct {
            u8 A;           // 8-bit Accumulator register
            u8 F;
        };
        u16 AF;             // special 16-bit register for storing flags of arithmetic/logic operations
    };

    union {
        struct {
            u8 B;           // 8-bit General Purpose Register
            u8 C;
        };
        u16 BC;             // 16-bit register comprised out of 2 * 8-bit registers
    };

    union {
        struct {
            u8 D;
            u8 E;
        };
        u16 DE;             // 16-bit register comprised out of 2 * 8-bit registers
    };

    union {
        struct {
            u8 H;
            u8 L;
        };
        u16 HL;             // 16-bit register comprised out of 2 * 8-bit registers
    };

    u16 SP;                 // Stackpointer register
    u16 PC;                 // Program Counter
    u64 cycles;             // CPU cycles passed

    bool ime;               // "interrupt master enable"
    bool ime_enable_scheduled;
    bool halted;
    bool stopped;
} CPU;

void init_cpu(CPU *cpu);
int load_rom(const char *filename);
void cpu_step(CPU *cpu);

#endif
