# =============================================================================
# Thunders Robotic - STM32F407VG Firmware
# Modern CMake build (>= 3.20) for arm-none-eabi-gcc
# =============================================================================
cmake_minimum_required(VERSION 3.20)

# -----------------------------------------------------------------------------
# Toolchain selection.  The project supports two workflows:
#   1. External toolchain file passed via -DCMAKE_TOOLCHAIN_FILE=...
#      (recommended, e.g. the stm32-cmake gcc-arm-none-eabi.cmake)
#   2. In-tree detection of arm-none-eabi-gcc on PATH.
# We enable option (2) as a fallback so a plain `cmake -B build` works on a
# developer laptop that has the bare-metal toolchain installed.
# -----------------------------------------------------------------------------
if(NOT CMAKE_TOOLCHAIN_FILE)
    if(NOT DEFINED CMAKE_C_COMPILER)
        set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    endif()
    if(NOT DEFINED CMAKE_CXX_COMPILER)
        set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    endif()
endif()

project(ThundersRobotSTM32
    VERSION     0.4.0
    LANGUAGES   C CXX
    DESCRIPTION "STM32F407VG firmware for the Thunders Robotic differential-drive platform")

# -----------------------------------------------------------------------------
# MCU / board selection
# -----------------------------------------------------------------------------
set(STM32_MCU       STM32F407VG  CACHE STRING "Target part number")
set(STM32_FAMILY    STM32F4     CACHE STRING "STM32 family")
set(CPU_TYPE        "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# -----------------------------------------------------------------------------
# External dependency roots.  These point at vendor trees pulled in via git
# submodules (see README.md).  Override with -D<VAR>=... on the cmake command
# line when using a non-standard checkout layout.
# -----------------------------------------------------------------------------
set(STM32_HAL_DRIVER_DIR  "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/upstream"
    CACHE PATH    "Path to the ST STM32CubeF4 HAL driver tree")
set(CMSIS_DIR             "${CMAKE_SOURCE_DIR}/Drivers/CMSIS"
    CACHE PATH    "Path to ARM CMSIS + ST CMSIS-Device")
set(FREERTOS_DIR          "${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS/upstream"
    CACHE PATH    "Path to the FreeRTOS-Kernel checkout")

# -----------------------------------------------------------------------------
# Build configuration options
# -----------------------------------------------------------------------------
option(BUILD_UNIT_TESTS  "Build host-side unit tests (no cross compile)" OFF)
option(ENABLE_RTT        "Enable SEGGER RTT printf retargeting"           ON)
option(LTO               "Enable link-time optimization"                  OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# -----------------------------------------------------------------------------
# Common flags
# -----------------------------------------------------------------------------
set(MCU_COMMON_FLAGS
    "${CPU_TYPE}"
    "-fdata-sections"
    "-ffunction-sections"
    "-fsingle-precision-constant"
    "-fno-common"
    "-Wall -Wextra -Wno-unused-parameter"
    "-Wno-missing-field-initializers"
    "-g3"
    "-DUSE_HAL_DRIVER"
    "-DSTM32F407xx"
    "-DARM_MATH_CM4"
    "-D__FPU_PRESENT=1"
    "-DTHUNDERS_ROBOTIC_VERSION=\"${PROJECT_VERSION}\"")

if(LTO)
    list(APPEND MCU_COMMON_FLAGS "-flto")
endif()

set(CMAKE_C_FLAGS_INIT      "${MCU_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT    "${MCU_COMMON_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT    "${MCU_COMMON_FLAGS}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# -----------------------------------------------------------------------------
# Include directories
# -----------------------------------------------------------------------------
set(HAL_INCLUDE_DIRS
    ${STM32_HAL_DRIVER_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc)

set(CMSIS_INCLUDE_DIRS
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32F4xx/Include)

set(FREERTOS_INCLUDE_DIRS
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS/Inc)

# -----------------------------------------------------------------------------
# Linker script
# -----------------------------------------------------------------------------
set(LINKER_SCRIPT
    "${CMAKE_SOURCE_DIR}/Core/ld/stm32f407vg.ld"
    CACHE FILEPATH "GNU linker script for the target board")

# -----------------------------------------------------------------------------
# Firmware executable
# -----------------------------------------------------------------------------
if(NOT BUILD_UNIT_TESTS)
    add_executable(firmware.elf)

    target_include_directories(firmware.elf PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/Custom_Drivers/Inc
        ${CMAKE_SOURCE_DIR}/Middlewares/Communication/Inc
        ${CMAKE_SOURCE_DIR}/Middlewares/Utilities/Inc
        ${HAL_INCLUDE_DIRS}
        ${CMSIS_INCLUDE_DIRS}
        ${FREERTOS_INCLUDE_DIRS})

    # Source globs per subsystem
    file(GLOB CORE_SOURCES       CONFIGURE_DEPENDS "Core/Src/*.c")
    file(GLOB HAL_SOURCES        CONFIGURE_DEPENDS "Drivers/STM32F4xx_HAL_Driver/Src/*.c")
    file(GLOB CUSTOM_DRV_SOURCES CONFIGURE_DEPENDS "Drivers/Custom_Drivers/Src/*.c")
    file(GLOB COMM_SOURCES       CONFIGURE_DEPENDS "Middlewares/Communication/Src/*.c")
    file(GLOB UTIL_SOURCES       CONFIGURE_DEPENDS "Middlewares/Utilities/Src/*.c")
    file(GLOB FREERTOS_SOURCES   CONFIGURE_DEPENDS
        "${FREERTOS_DIR}/tasks.c"
        "${FREERTOS_DIR}/queue.c"
        "${FREERTOS_DIR}/timers.c"
        "${FREERTOS_DIR}/event_groups.c"
        "${FREERTOS_DIR}/list.c"
        "${FREERTOS_DIR}/stream_buffer.c"
        "${FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c"
        "${FREERTOS_DIR}/portable/MemMang/heap_4.c")

    target_sources(firmware.elf PRIVATE
        ${CORE_SOURCES}
        ${HAL_SOURCES}
        ${CUSTOM_DRV_SOURCES}
        ${COMM_SOURCES}
        ${UTIL_SOURCES}
        ${FREERTOS_SOURCES})

    target_link_options(firmware.elf PRIVATE
        -T${LINKER_SCRIPT}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map
        --specs=nano.specs
        --specs=nosys.specs
        -u _printf_float
        -lc -lm -lnosys)

    add_custom_command(TARGET firmware.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware.elf> ${CMAKE_BINARY_DIR}/firmware.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:firmware.elf> ${CMAKE_BINARY_DIR}/firmware.hex
        COMMAND ${CMAKE_SIZE}    $<TARGET_FILE:firmware.elf>
        COMMENT "Generating binary/hex images and printing size")
endif()

# -----------------------------------------------------------------------------
# Host-side unit tests
# -----------------------------------------------------------------------------
if(BUILD_UNIT_TESTS)
    enable_testing()
    enable_language(CXX)
    add_subdirectory(Tests)
endif()
