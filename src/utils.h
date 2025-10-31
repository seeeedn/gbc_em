#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

// PPU MODES
#define HBLANK          0
#define VBLANK          1
#define OAM_SCAN        2
#define DRAWING         3

// FLAG/REG LOCATIONS (relative to io_regs[])
#define JOYPAD          0x0000
#define SB              0x0001
#define SC              0x0002
#define DIV             0x0004
#define TIMA            0x0005
#define TMA             0x0006
#define TAC             0x0007
#define IF_ADDRESS      0x000F              // IF
#define LCDC            0x0040              // LCD-Control
#define STAT            0x0041
#define SCY             0x0042
#define SCX             0x0043              // Scroll registers
#define LY              0x0044              // current scanline
#define LYC             0x0045
#define BGP             0x0047              // Background palette
#define OBP0            0x0048
#define OBP1            0x0049
#define WY              0x004A
#define WX              0x004B
#define VBK_INDEX       0x004F
#define WBK_INDEX       0x0070

// INTERRUPTS
#define INT_VBLANK      0x01
#define INT_STAT        0x02
#define INT_TIMER       0x04
#define INT_SERIAL      0x08
#define INT_JOYPAD      0x10

#define IS_BIT_SET(reg, bit)        (((reg) & (bit)) != 0)
#define IS_BIT_CLEAR(reg, bit)      (((reg) & (bit)) == 0)

#define SET_BIT(dst, bit)           ((dst) |= (1 << bit))
#define CLEAR_BIT(dst, bit)         ((dst) ~= (1 << bit))

#endif
