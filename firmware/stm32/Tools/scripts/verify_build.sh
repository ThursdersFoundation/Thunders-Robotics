#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# verify_build.sh - Thunders Robotic STM32 firmware host verification.
#
# 1. Syntax-check every C source in the STM32 tree with the project include
#    paths (gcc -fsyntax-only). This catches typos and API misuse without an
#    ARM toolchain.
# 2. (called separately) builds and runs the host unit tests (see Tests/).
# ---------------------------------------------------------------------------
set -u

STM32_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [ ! -d "$STM32_DIR/Core" ]; then
    # fallback when run from the repo root (script copied elsewhere)
    STM32_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/firmware/stm32"
fi

INC="-I$STM32_DIR/Core/Inc"
INC="$INC -I$STM32_DIR/Drivers/CMSIS/Include"
INC="$INC -I$STM32_DIR/Drivers/CMSIS/Device"
INC="$INC -I$STM32_DIR/Drivers/STM32F4xx_HAL_Driver/Inc"
INC="$INC -I$STM32_DIR/Drivers/Custom_Drivers/Inc"
INC="$INC -I$STM32_DIR/Middlewares/FreeRTOS/Inc"
INC="$INC -I$STM32_DIR/Middlewares/Communication/Inc"
INC="$INC -I$STM32_DIR/Middlewares/Utilities/Inc -I$STM32_DIR/Tests/mocks"

FAIL=0
COUNT=0
for f in $(find "$STM32_DIR" -name '*.c' | sort); do
    COUNT=$((COUNT + 1))
    if ! gcc -fsyntax-only -std=c11 -Wall -Wextra -Wno-unused-parameter $INC "$f" 2> /tmp/thunders_syntax_err.txt; then
        echo "FAIL: $f"
        sed -n '1,15p' /tmp/thunders_syntax_err.txt
        FAIL=$((FAIL + 1))
    fi
done

echo "-------------------------------------------"
echo "Syntax-checked $COUNT C files, $FAIL failing"
exit $FAIL
