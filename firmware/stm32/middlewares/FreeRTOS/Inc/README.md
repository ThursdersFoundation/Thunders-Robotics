# Middlewares/FreeRTOS/Inc/

This directory is the project's integration root for the **FreeRTOS-Kernel**
public headers.  The upstream tree is *not* committed to this repository -
it is pulled in via a git submodule and referenced by the top-level
`CMakeLists.txt`.

## Files normally provided by upstream

These come from the FreeRTOS-Kernel repository
(<https://github.com/FreeRTOS/FreeRTOS-Kernel>) under `include/`:

| File | Purpose |
|------|---------|
| `FreeRTOS.h`               | Master header, defines configKERNEL_INTERRUPT_PRIORITY etc. |
| `task.h`                   | Task API (xTaskCreate, vTaskDelay, etc.) |
| `queue.h`                  | Queue / semaphore primitives |
| `semphr.h`                 | Semaphore macros (binary, counting, mutex) |
| `timers.h`                 | Software timer API |
| `event_groups.h`           | Event-group API |
| `stream_buffer.h`          | Stream-buffer / message-buffer API |
| `message_buffer.h`         | Thin wrapper on top of stream_buffer.h |
| `list.h`                   | Intrusive doubly-linked list (used by tasks/queues internally) |
| `FreeRTOSConfig.h`         | **Project-specific** config; lives in this directory (NOT upstream) |

## FreeRTOSConfig.h location

The Thunders Robotic project keeps its own `FreeRTOSConfig.h` in this
directory (next to the upstream headers).  It defines:

* `configCPU_CLOCK_HZ`           = 168000000UL (matches `SystemClock_Config`)
* `configTICK_RATE_HZ`           = 1000
* `configMAX_PRIORITIES`         = 7
* `configMINIMAL_STACK_SIZE`     = 128 words
* `configTOTAL_HEAP_SIZE`        = 32 * 1024 bytes
* `configCHECK_FOR_STACK_OVERFLOW` = 2 (calls `vApplicationStackOverflowHook`)
* `configUSE_TRACE_FACILITY`     = 1
* `configUSE_16_BIT_TICKS`       = 0
* `configUSE_MUTEXES`           = 1
* `configUSE_RECURSIVE_MUTEXES` = 1
* `configUSE_TIMERS`            = 1
* `configPRIO_BITS`             = 4 (NVIC_PRIORITYGROUP_4)
* `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` = 15
* `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` = 5

## Submodule placement

```bash
git submodule add https://github.com/FreeRTOS/FreeRTOS-Kernel \
  Middlewares/FreeRTOS/upstream
```

The CMake build adds
`Middlewares/FreeRTOS/upstream/include` and
`Middlewares/FreeRTOS/upstream/portable/GCC/ARM_CM4F` to the
`FREERTOS_INCLUDE_DIRS` variable.  Building without the submodule
checked out will fail with "FreeRTOS.h: No such file or directory".
