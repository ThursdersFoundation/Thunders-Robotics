# Thunders Robotic — ESP32 Firmware

PlatformIO project for the ESP32 family of micro-controllers. The firmware
follows a layered architecture (App → Control → Drivers, with Safety and
Diagnostics cross-cutting) so each concern can be tested and evolved in
isolation.

## Prerequisites

| Tool           | Version   | Purpose                                   |
| -------------- | --------- | ------------------------------------------ |
| PlatformIO Core| ≥ 6.1     | Build / flash / monitor                    |
| Python         | ≥ 3.8     | Required by PlatformIO                     |
| USB driver     | —         | CP210x / CH340 depending on the board      |

Install PlatformIO:

```bash
python3 -m pip install -U platformio
```

Or use the [PlatformIO IDE extension for VS Code](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide).

## Directory layout

```
esp32/
├── platformio.ini          ← build / environment definitions
├── partitions.csv          ← flash partition map
├── include/               ← compile-time configuration headers
├── src/
│   ├── main.cpp            ← entry point: init & spawn FreeRTOS tasks
│   ├── app/                ← robot-level state machine
│   ├── control/            ← PID / velocity / motor controllers
│   ├── drivers/            ← hardware abstraction layer
│   ├── communication/       ← Wi-Fi / MQTT / serial / ROS bridge
│   ├── safety/             ← watchdog & e-stop supervision
│   └── diagnostics/        ← logger & health monitor
├── lib/                    ← private internal libraries
├── test/                   ← Unity unit tests (native platform)
└── docs/                   ← wiring diagrams, protocol specs
```

## Building & flashing

```bash
# List available environments
pio run --list-targets

# Build the default environment
pio run                       # = pio run -e esp32dev

# Build an alternate target
pio run -e esp32s3

# Upload firmware to the board
pio run -t upload

# Live serial monitor (Ctrl+] to exit)
pio device monitor -b 115200

# Combined: build + flash + monitor
pio run -t upload -t monitor
```

## Wiring guide (esp32dev reference)

| Peripheral          | ESP32 pin(s)            | Notes                                |
| ------------------- | ----------------------- | ------------------------------------ |
| Left motor driver   | PWM: GPIO 25/26, DIR: 27 | 20 kHz PWM, 10-bit resolution       |
| Right motor driver  | PWM: GPIO 14/15, DIR: 12 | 20 kHz PWM, 10-bit resolution       |
| Left encoder        | A: GPIO 34, B: GPIO 35  | Interrupt-capable, PCNT-friendly     |
| Right encoder       | A: GPIO 32, B: GPIO 33  | Interrupt-capable, PCNT-friendly     |
| IMU (BNO055)        | SCL: GPIO 22, SDA: 21   | I²C @ 100 kHz                        |
| Ultrasonic (HC-SR04)| TRIG: GPIO 4, ECHO: 16  | 5 V tolerant, use level shifter      |
| Status LED          | GPIO 2                  | On-board, used for heartbeat         |
| E-stop input        | GPIO 17                 | Active-low, pull-up enabled          |

Pin assignments are centralised in [`include/pin_config.hpp`](include/pin_config.hpp).

## Configuration

All tunable parameters live in `include/*_config.hpp` as `constexpr` values.
This keeps them compile-time checkable and lets the optimiser eliminate dead
branches. Override any value by either editing the header or passing
`-D` flags through `platformio.ini`'s `build_flags`.

## Testing

Unit tests run on the host via the `native` platform:

```bash
pio test -e native
```

Tests live under `test/` and stub out the ESP32 HAL via mocks under
`test/mock/`. Each module should have at least one test file covering its
public API and edge cases.

## Contributing

1. Branch off `develop`.
2. Keep changes within one architectural layer when possible.
3. Add or update unit tests for any new public function.
4. Ensure `pio test -e native` passes before requesting review.
5. Document new public APIs in `docs/`.
