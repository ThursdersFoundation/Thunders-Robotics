# Thunders Robotic - STM32F407VG Firmware

Firmware for the **Thunders Robotic** differential-drive mobile robot, targeting
the **STM32F407VG** (Cortex-M4F @ 168 MHz, 1 MB flash, 192 KB SRAM, CAN, USB OTG,
3x ADC, 12x timers).  The firmware is structured as a FreeRTOS-based,
module-oriented codebase that talks to a host PC over UART/CAN, drives two
brushless DC motors with closed-loop velocity control, and reads an IMU,
ultrasonic range sensor, and an RPLIDAR A1.

## Architecture

```
                          +---------------------------+
   Host PC / ROS 2 ------ |  UART3 (115200 8N1)       | ---------+
   (cmd_vel, telemetry)   |  CAN1 (500 kbps)          |          |
                          +---------------------------+          |
                                      |                         |
                                      v                         v
   +-----------------+    +-----------------------+    +-----------------------+
   | Safety Manager  |-->|   Communication Mgr    |    |   Motor Controller    |
   | (ESTOP, faults) |    |  (protocol dispatch)  |    |  (PID, kinematics)    |
   +-----------------+    +-----------------------+    +-----------------------+
                                      |                          |
                                      v                          v
   +-----------------+    +-----------------------+    +-----------------------+
   | Sensor Manager  |    |  Utilities            |    |  Custom Drivers       |
   |  IMU / US / Batt|    |  ring / avg / WDog    |    |  drv_motor / drv_enc  |
   +-----------------+    +-----------------------+    |  drv_imu / drv_lidar  |
                                                       |  drv_batt / drv_us    |
                                                       +-----------------------+
                                      |
                                      v
                          +---------------------------+
                          |  STM32 HAL + FreeRTOS     |
                          |  CMSIS / ARM Cortex-M4F   |
                          +---------------------------+
```

## Directory layout

```
firmware/stm32/
+- Core/                       - Application layer: main, peripheral init, manager modules
|  +- Inc/                     - Public headers for Core modules
|  +- Src/                     - main.c, stm32f4xx_it.c, stm32f4xx_hal_msp.c, *_init.c
+- Drivers/
|  +- CMSIS/Device/README.md   - ST STM32F4xx CMSIS-Device headers (git submodule)
|  +- CMSIS/Include/README.md  - ARM CMSIS-Core headers (git submodule)
|  +- STM32F4xx_HAL_Driver/    - ST HAL glue + upstream re-export headers
|  +- Custom_Drivers/          - Project-specific peripheral drivers (motor, IMU, LIDAR...)
+- Middlewares/
|  +- Communication/           - Host protocol: packet parser, UART/CAN framing, CRC
|  +- Utilities/               - ring buffer, moving average, software watchdog
|  +- FreeRTOS/                - FreeRTOS-Kernel pulled in via git submodule
+- Tests/
|  +- unit/                    - Unity-style C++ host-side unit tests
|  +- integration/             - Catch2-style hardware-in-loop test plans
+- Tools/
|  +- flashing/                - ST-Link / J-Link / OpenOCD flashing notes
|  +- debugging/               - SWO ITM, RTT, GDB setup notes
|  +- scripts/                 - flash_firmware.sh, verify_build.sh
+- CMakeLists.txt               - Top-level CMake build
+- Makefile                    - `make build / flash / clean / test / menuconfig`
```

## Hardware target

* MCU: **STM32F407VG** (LQFP100, revision A)
* Board reference: STM32F4 DISCOVERY (MB997B) with motor driver breakout
* HSE crystal: 8 MHz
* SYSCLK: 168 MHz (PLL: M=8, N=336, P=2, Q=7)
* APB1: 42 MHz, APB2: 84 MHz
* Motor PWM: TIM1 CH1-4 @ 20 kHz (PA8 / PA9 / PA10 / PA11)
* Encoder: TIM2 (left, PA15/PB3) + TIM5 (right, PA0/PA1) in TI1+TI2 mode
* IMU: BNO055 on I2C1 (0x28 / 0x29), PB6=SCL, PB7=SDA
* LIDAR: RPLIDAR A1 on UART4 (PC10/PC11) @ 460800 baud
* Host comms: UART3 (PD8/PD9) @ 115200 or CAN1 (PD0/PD1) @ 500 kbps
* Battery monitor: ADC1 channel 0 (PA0) via 11:1 voltage divider
* ESTOP button: PC13 (active low, EXTI13 falling edge)

## FreeRTOS task list

| Task name       | Priority | Stack | Period | Responsibility                          |
|-----------------|----------|-------|--------|-----------------------------------------|
| MotorTask       | 5        | 512   | 10 ms  | PID closed-loop wheel velocity control  |
| SensorTask      | 4        | 768   | 20 ms  | Sample IMU / ultrasonic / battery       |
| CommTask        | 4        | 1024  | event | Parse inbound packets, send telemetry  |
| SafetyTask      | 6        | 256   | 5 ms   | ESTOP / fault monitoring, watchdog feed |
| LidarTask       | 3        | 1024  | event | RPLIDAR frame parse, distance table     |
| TelemetryTask   | 2        | 512   | 100 ms | Push RobotState_t to host               |
| IDLE            | 0 (tsk)  | 128   | n/a    | FreeRTOS idle hook feeds IWDG            |

## Build instructions

### Prerequisites
* arm-none-eabi-gcc toolchain (10.3+), OpenOCD 0.12+, cmake 3.20+, make
* git submodules initialised (Drivers/STM32F4xx_HAL_Driver/upstream,
  Drivers/CMSIS/Device/ST, Middlewares/FreeRTOS/upstream)

### Build the firmware
```bash
cd firmware/stm32
make            # configures + builds build/firmware.elf, .bin, .hex
make size       # prints flash/RAM usage
```

### Build & run host unit tests
```bash
make test       # builds Tests/ with the host compiler, runs ctest
```

### Flash via ST-Link / OpenOCD
```bash
make flash      # builds, then calls Tools/scripts/flash_firmware.sh
# or directly:
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg \
        -c "program build/firmware.elf verify reset exit"
```

### Debug with GDB
```bash
arm-none-eabi-gdb build/firmware.elf -ex "target remote :3333"
# see Tools/debugging/README.md for SWO ITM / RTT setup
```

## Coding conventions
* C99 for production code, C++14 for host tests
* 4-space indentation, no tabs
* Header guards `#ifndef FOO_H / #define FOO_H ... #endif`
* Public identifiers prefixed per module (`MOTOR_`, `ENC_`, `PROTO_`, `COMM_`)
* Doxygen `@brief` / `@param` / `@return` on every public function
* Each `.c` includes its own `.h` first, then system, then project headers

## Repository links (submodules)
* HAL: <https://github.com/STMicroelectronics/STM32CubeF4> -> `Drivers/STM32F4xx_HAL_Driver/upstream/`
* CMSIS: <https://github.com/ARM-software/CMSIS_5> -> `Drivers/CMSIS/Include/CMSIS/`
* FreeRTOS: <https://github.com/FreeRTOS/FreeRTOS-Kernel> -> `Middlewares/FreeRTOS/upstream/`
