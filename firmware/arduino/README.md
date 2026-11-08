# firmware/arduino

Arduino / PlatformIO firmware for the Thunders Robotic platform.

This directory holds five independent firmware sub-projects, one per node
on the robot. Every sub-project is a self-contained PlatformIO project
with its own `platformio.ini`, `include/`, `src/` and (where applicable)
`test/` directory. The top-level `platformio.ini` is a convenience
aggregator that exposes each sub-project as its own environment.

## Sub-projects

| Sub-project          | Board              | Role                                  |
|----------------------|--------------------|---------------------------------------|
| `motor_controller`   | ESP32-WROOM-32     | Closed-loop diff-drive motor controller |
| `sensor_node`        | ESP32-S3-DevKitC   | IMU / ultrasonic / temp / battery aggregation |
| `communication`      | ESP32-WROOM-32     | USB-CDC <-> CAN gateway, shared protocol |
| `actuator_controller`| ESP32-C3-DevKitM-1 | Servo + stepper driver, dispatches SET_MODE |
| `diagnostics`        | ESP32-WROOM-32     | Fault registry, 1 Hz JSON health stream |

## Hardware targets

- **ESP32-WROOM-32 DevKit** for `motor_controller`, `communication` and
  `diagnostics` (most GPIO-rich, PCNT + TWAI + LEDC all available).
- **ESP32-S3-DevKitC** for `sensor_node` (USB-CDC + I2C for the BNO055 IMU).
- **ESP32-C3-DevKitM-1** for `actuator_controller` (small form factor, LEDC
  servo + step/dir GPIO is more than enough).

All boards use the Espressif `arduino-esp32` core (`espressif32@^6.5.0`
platform). USB-CDC is enabled so every node presents as a single serial
device without a separate UART bridge.

## Shared wire protocol

`communication/` owns the framed serial protocol used by every node:

```
STX(0xAA) LEN TYPE PAYLOAD[LEN] CRC_LO CRC_HI ETX(0x55)
```

- LEN is the unescaped payload length (0..255).
- TYPE is one of `MsgType` (`communication/include/message_types.h`).
- CRC-16/CCITT (init 0xFFFF, poly 0x1021) covers LEN + TYPE + PAYLOAD.
- Bytes equal to STX/ETX/ESC(0x1B) inside the framed fields are escaped as
  `ESC (byte XOR 0x20)` so delimiters only appear as real frame markers.

`actuator_controller` links the `communication` project as a library
(`lib_deps = communication`) and reuses `SerialProtocol` directly.
`COMM_AS_LIBRARY` suppresses the communication node's own `setup()/loop()`
so there is no duplicate entry point at link time.

## Build commands

Build all sub-projects from this directory:

```bash
cd firmware/arduino
pio run
```

Build a single sub-project, either via the aggregator env or its own ini:

```bash
pio run -e motor_controller            # top-level env name
cd motor_controller && pio run -e esp32dev   # per-project env name
```

Flash and monitor:

```bash
pio run -e <env> -t upload
pio device monitor -b 115200
```

`diagnostics` has no per-project `platformio.ini`; build it only through the
top-level aggregator (`pio run -e diagnostics`).

## Tests

Host-side unit tests run on the developer's machine (no hardware needed):

```bash
cd firmware/arduino/<sub-project>
pio test -e native
```

| Sub-project      | Test file                          |
|------------------|------------------------------------|
| motor_controller | `test/test_pid_controller.cpp`     |
| sensor_node      | `test/test_sensor_manager.cpp`     |
| communication    | `test/test_packet_parser.cpp`      |

Integration tests under `tests/integration/` require multiple physical
boards; see `tests/integration/README.md` for the wiring and helper scripts.

## Shared libraries

`libraries/` is the home for project-internal libraries shared across two or
more sub-projects. Each sub-project pulls it in via `lib_extra_dirs`. See
`libraries/README.md` for the PlatformIO LDF conventions used here.

## Code style

- C++17, Arduino `setup()`/`loop()` pattern, business logic in classes.
- Header guards `#ifndef FOO_H` / `#define FOO_H` (no `#pragma once`).
- Doxygen `@brief` / `@param` / `@return` on every public function.
- 4-space indent, no tabs.
- Each translation unit is focused and unit-testable in isolation; hardware
  access is confined to the `*Driver` / `*Sensor` / `*Controller` classes.

## License

Proprietary - Thunders Robotic project. Do not redistribute without
written permission.
