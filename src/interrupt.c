#include "interrupt.h"
#include "mmu.h"
#include "cpu.h"
#include "debug.h"

u8 service_interrupt() {
    u8 IE = read_mmio(0xFF);
    u8 IF = read_mmio(IF_ADDRESS);
    u8 interrupts = IE & IF;

    if (cpu.halted && interrupts) {
        cpu.halted = false;
    }

    if (!cpu.ime || !interrupts) {
        return 0;
    }

    static const u8 call_vec[5] = { 0x40, 0x48, 0x50, 0x58, 0x60 };
    
    for (int i = 0; i < 5; i++) {
        if (interrupts & (1 << i)) {
            cpu.ime = false;
            IF &= ~(1 << i);

            write_mmio(IF_ADDRESS, IF);

            cpu.SP -= 2;
            mmu_write_word(cpu.SP, cpu.PC);

            cpu.PC = call_vec[i]; 

            break;
        } 
    }

    cpu.total_cycles += 20;
    return 20;
}

