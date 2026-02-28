#ifndef TIMER_H
#define TIMER_H

#include "utils.h"

extern u64 div_counter;

void update_timer(u64 cycles, bool stopped);

#endif
