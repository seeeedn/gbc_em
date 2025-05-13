#include "cpu.h"
#include <stdio.h>

int load_rom(const char* filename) {
    FILE *rom_file = fopen(filename, "rb");
    if (!rom_file) {
        printf("Failed to open ROM file: %s\n", filename);
        return -1;
    }

    // Read ROM into memory starting at address 0x0100
    fseek(rom_file, 0, SEEK_END);
    long file_size = ftell(rom_file);
    fseek(rom_file, 0, SEEK_SET);

    if (file_size > MEM_SIZE) {
        printf("ROM file too large to fit in memory.\n");
        fclose(rom_file);
        return -1;
    }

    fread(&memory[0x0100], 1, file_size, rom_file);
    fclose(rom_file);

    return 0;
}

void run_test_rom(const char* rom_filename) {
    if (load_rom(rom_filename) != 0) {
        return; // Exit if ROM couldn't be loaded
    }

    CPU cpu;
    init_cpu(&cpu);

    // Run the CPU until the ROM test completes
    while (!cpu.halted) {
        cpu_step(&cpu);
        printf("PC: 0x%04X, A: 0x%02X, B: 0x%02X, C: 0x%02X, D: 0x%02X, E: 0x%02X\n",
                cpu.PC, cpu.A, cpu.B, cpu.C, cpu.D, cpu.E);
    }

    printf("Test complete\n");
}

int main() {
    const char *test_rom = "src/cpu_instrs.gb";
    run_test_rom(test_rom);
    while (1);
    return 0;
}
