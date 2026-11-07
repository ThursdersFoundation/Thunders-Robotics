# Drivers/CMSIS/Include/

This directory is the project's integration root for the **ARM CMSIS-Core**
headers (the ARMv7E-M / Cortex-M4F core register definitions).  As with the
CMSIS-Device tree, the upstream files are *not* committed here - they are
pulled in via a git submodule and added to the CMake include path by the
top-level `CMakeLists.txt`.

## Files normally provided by upstream

These come from the ARM-software/CMSIS_5 repository
(<https://github.com/ARM-software/CMSIS_5>) under `CMSIS/Core/Include/`:

| File | Purpose |
|------|---------|
| `core_cm4.h`             | Cortex-M4F core definitions, NVIC, SysTick, SCB |
| `core_cmFunc.h`          | Inline functions for core register access (`__enable_irq`, `__WFI`, etc.) |
| `core_cmInstr.h`         | CMSIS inline instruction intrinsics |
| `cmsis_compiler.h`       | Compiler-agnostic dispatch (GCC, IAR, AC6) |
| `cmsis_gcc.h`            | GCC-specific CMSIS intrinsics |
| `mpu_armv7.h`            | ARMv7-M MPU definitions (used for stack-overflow detection) |
| `mpu_armv7.h`            | MPU helper functions |
| `cmsis_version.h`        | CMSIS version macros (e.g. `__CM4_CMSIS_VERSION_MAIN`) |

## Submodule placement

The expected layout is:

```
Drivers/CMSIS/Include/CMSIS/Core/Include/
+- core_cm4.h
+- core_cmFunc.h
+- core_cmInstr.h
+- cmsis_compiler.h
+- cmsis_gcc.h
+- mpu_armv7.h
+- cmsis_version.h
```

## How to set up the submodule

```bash
git submodule add https://github.com/ARM-software/CMSIS_5 \
  Drivers/CMSIS/Include/upstream/CMSIS_5
ln -s upstream/CMSIS_5/CMSIS/Core/Include \
  Drivers/CMSIS/Include/CMSIS
```

The CMake build adds `Drivers/CMSIS/Include/CMSIS` to the
`CMSIS_INCLUDE_DIRS` variable.  Building without the submodule checked out
will fail with "core_cm4.h: No such file or directory" at the very first
preprocessed translation unit (typically `main.c`).

## License note

CMSIS 5 is Apache-2.0 licensed.  A copy of the upstream LICENSE.txt is
preserved inside the submodule checkout at
`Drivers/CMSIS/Include/upstream/CMSIS_5/LICENSE.txt`.
