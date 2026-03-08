# Game Boy Color Emulator (Work in Progress)

Overview
A simple emulator of the Game Boy Color, written in C.
The goal is to replicate the functionality of the original hardware as closely as possible.
This project is part of my effort to deepen my understanding of low-level systems and how different components of a system-on-chip (SoC) interact.

## Features so far:
  - CPU: fully implemented
  - PPU: rendering functional (not fully accurate at the moment)
  - MMU: mostly complete
  - Runs and passes all Blargg test ROMs
  - I/O fully functional
  - Basic Timer implemented
  - Basic MBC1, MBC3 implemented

## Next steps:
  - Implement Cartridge Memory Chips (MBC2)
  - Add Gameboy Color features
  - Finalize PPU implementation
  - Implement APU (Audio Processing Unit)
