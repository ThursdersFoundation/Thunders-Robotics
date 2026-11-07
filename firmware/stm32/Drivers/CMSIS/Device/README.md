# Drivers/CMSIS/Device/

This directory is the project's integration root for the **STMicro STM32F4xx
CMSIS-Device** headers.  The upstream tree is *not* committed to this
repository - it is pulled in via a git submodule and referenced by the
top-level `CMakeLists.txt`.

## Submodule placement

The expected layout is:

```
Drivers/CMSIS/Device/ST/STM32F4xx/
+- Include/
|  +- stm32f407xx.h            # part-specific register definitions
|  +- stm32f4xx.h               # family dispatch header
+- Source/
|  +- system_stm32f4xx.c       # SystemInit(), SystemCoreClock variable
|  +- startup_stm32f407xx.s    # vector table + Reset_Handler (Cortex-M4F)
+- LICENSE.txt
```

## Files normally provided by upstream

These files live in the STM32CubeF4 repository
(<https://github.com/STMicroelectronics/STM32CubeF4>) under
`Drivers/CMSIS/Device/ST/STM32F4xx/`:

| File | Purpose |
|------|---------|
| `Include/stm32f4xx.h`                | Top family include |
| `Include/stm32f407xx.h`              | STM32F407VG register definitions (IRQ numbers, base addresses) |
| `Include/system_stm32f4xx.h`         | `SystemInit`, `SystemCoreClockUpdate`, `SystemCoreClock` |
| `Source/Templates/system_stm32f4xx.c` | `SystemInit()` implementation (clock tree defaults) |
| `Source/Templates/gcc/startup_stm32f407xx.s` | GNU-assembler startup file with the Cortex-M4F vector table and `Reset_Handler` |

## How to set up the submodule

```bash
git submodule add https://github.com/STMicroelectronics/STM32CubeF4 \
  Drivers/CMSIS/Device/upstream/STM32CubeF4
# Symlink or copy the relevant subtree:
ln -s upstream/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx \
  Drivers/CMSIS/Device/ST/STM32F4xx
```

The CMake build adds `Drivers/CMSIS/Device/ST/STM32F4xx/Include` to the
target include path via the `CMSIS_INCLUDE_DIRS` variable defined in
the top-level `CMakeLists.txt`.

## Why this directory only ships a README

CMSIS-Device files are *part-specific*, *unchanged upstream*, and ~80 KB
combined.  Re-committing them here would make the project's diff noisy and
break updates.  The build will fail with "cannot find stm32f407xx.h" until
the submodule is checked out.
