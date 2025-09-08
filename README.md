# Game Boy Color Emulator (Work in Progress)

Overview
A simple emulator of the Game Boy Color, written in C.
The goal is to replicate the functionality of the original hardware as closely as possible.
This project is part of my effort to deepen my understanding of low-level systems and how different components of a system-on-chip (SoC) interact.

## Features so far:
  - CPU: fully implemented
  - PPU: basic rendering working
  - MMU: mostly complete
  - Runs and passes all Blargg test ROMs except the interrupt test (Joypad and Timer interrupts not yet implemented)

## Next steps:
  - Finalize PPU implementation
  - Implement ROM load/store
  - Add I/O functionality
  - Add Timer
  - Add Sound
