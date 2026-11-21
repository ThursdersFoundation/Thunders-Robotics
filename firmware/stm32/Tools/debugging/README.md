# Debugging the master board

1. Build: `make` (produces `build/thunders_master.elf`).
2. Connect ST-Link (SWD: PA13 SWDIO, PA14 SWCLK, GND, 3V3).
3. Flash + run: `make flash`.
4. Debug session: `make debug` (starts OpenOCD, attaches GDB, breaks at `main`).

Useful GDB one-liners while a task is running:
- `monitor reset halt`      - stop the CPU
- `p MotorControl_IsEnabled`- inspect module state
- `x/16wx 0x20000000`       - dump SRAM
- `bt`                      - call stack of the halted context
