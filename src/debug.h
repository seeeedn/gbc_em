#ifndef DEBUG_H
#define DEBUG_H

#include "utils.h"

#define RING_SIZE 512

typedef struct {
    u16 pc;
    u8 opcode;
    u8 ly;
    u8 stat;
    u8 IF;
    u8 IE;
    u8 IME;
    u8 halted;
} RingEntry;

extern RingEntry entries[RING_SIZE];
//extern unsigned int ring_pos;

void ring_log();
void ring_dump();

#endif
