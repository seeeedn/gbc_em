#include "cpu.h"
#include <stdio.h>

void run_test_rom(const char* rom_filename) {
    if (load_rom(rom_filename) != 0) {
        return; // Exit if ROM couldn't be loaded
    }

    CPU cpu;
    init_cpu(&cpu);

    // Run the CPU until the ROM test completes
    while (!cpu.halted) {  // You can also add more stopping conditions based on the test
        cpu_step(&cpu);
        printf("PC: 0x%04X, A: 0x%02X, B: 0x%02X, C: 0x%02X, D: 0x%02X, E: 0x%02X\n", 
                cpu.PC, cpu.A, cpu.B, cpu.C, cpu.D, cpu.E);
        // Add checks here to verify if the CPU state matches expected values.
    }

    printf("Test complete\n");
}

int main() {
    const char* test_rom = "src/cpu_instrs.gb";  // Path to your Blargg test ROM file
    run_test_rom(test_rom);
    while (1);
    return 0;
}