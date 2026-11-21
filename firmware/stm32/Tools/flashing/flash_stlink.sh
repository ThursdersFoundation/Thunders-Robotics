#!/usr/bin/env bash
# Flash the master board via ST-Link + OpenOCD.
# Usage: ./flash_stlink.sh [path/to/thunders_master.elf]
set -euo pipefail
ELF="${1:-build/thunders_master.elf}"
if [ ! -f "$ELF" ]; then
    echo "error: $ELF not found - build first (make)" >&2
    exit 1
fi
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program $ELF verify reset exit"
