# Architecture

This document describes the firmware architecture at a level useful for
maintainers adding new sub-systems.

## Layered view

```
        ┌──────────────────────────────────────────────┐
        │                  RobotApp                     │  ← wires everything
        │           (src/app/robot_app.cpp)             │
        └───┬──────────────────────────────────────┬────┘
            │                                      │
   ┌────────▼───────────┐                  ┌─────────▼──────────┐
   │     App / Tasks    │                  │   Communication    │
   │  (robot_state,      │                  │  (wifi, mqtt,      │
   │   task_manager)     │                  │   serial, ros)     │
   └────────┬───────────┘                  └────────┬───────────┘
            │                                      │
   ┌────────▼─────────────┐               ┌────────▼───────────┐
   │      Control         │               │      Safety        │
   │  (pid, motor_ctrl,   │               │  (watchdog,        │
   │   velocity_ctrl)     │               │   emergency_stop)  │
   └────────┬─────────────┘               └────────┬───────────┘
            │                                      │
   ┌────────▼──────────────────────────────────────▼────────────┐
   │                       Drivers                                │
   │     (motor, encoder, imu, ultrasonic)                        │
   └───────────────────────────┬─────────────────────────────────┘
                                │
                  ┌─────────────▼─────────────┐
                  │       Board / Pins         │  (board_config.hpp,
                  │   (HAL: ESP32 SDK +        │   pin_config.hpp)
                  │    Arduino-ESP32 core)     │
                  └───────────────────────────┘
```

### Dependency rules

* **App → Control → Drivers**: never the reverse. Drivers must not call
  controllers; controllers must not call the app layer.
* **Communication may read RobotState but writes only via well-defined
  callbacks** — it never mutates the state machine directly.
* **Safety is a free-roaming supervisor**: it can call any layer to force
  e-stop (typically `motor_driver.disableAll()` + `velocity_controller.halt()`).
* **Diagnostics is read-only** — it samples state and publishes logs/MQTT,
  it does not affect behaviour.

## FreeRTOS task tree

| Task        | Core | Prio | Period | Stack | Responsibility                       |
| ----------- | ---- | ---- | ------ | ----- | ------------------------------------ |
| `ctrl`      | 1    | 20   | 10 ms  | 2 KB  | PID + velocity control loop          |
| `estop`     | 1    | 24   | 10 ms  | 1 KB  | Poll e-stop button + software triggers |
| `wdog`      | 0    | 24   | 1 s    | 1 KB  | Pet the ESP32 TWDT                   |
| `tele`      | 0    | 8    | 100 ms | 4 KB  | Publish telemetry JSON over MQTT/serial |
| `health`    | 0    | 4    | 1 s    | 2 KB  | Battery, heap, uptime, tip-over check |
| `mqtt`      | 0    | 12   | 100 ms | 6 KB  | PubSubClient loop + reconnect        |
| `serial`    | 0    | 12   | 10 ms  | 4 KB  | Binary protocol RX state machine      |
| `imu`       | 0    | 16   | 20 ms  | 2 KB  | Sample BNO055 (optional)              |
| `sonar`     | 0    | 16   | 100 ms | 2 KB  | Sample HC-SR04 (optional)              |
| `batt`      | 0    | 16   | 1 s    | 2 KB  | Sample battery ADC                    |

Real-time tasks (`ctrl`, `estop`) are pinned to core 1 to keep them away
from the Wi-Fi stack's interrupts, which run on core 0.

## State machine

```
        begin()         start()           command
   BOOT ──────────► IDLE ──────────► TELEOP ──────────► AUTONOMOUS
                       │                   ▲
                       │                   │
                       │             e-stop│
                       ▼                   │
                     HALT ◄────────────────┘
                       │
                       │  e-stop
                       ▼
                   EMERGENCY ◄────────── (any other state)
                       │
                       │  manual reset (kRequireManualResetOnEstop=true)
                       ▼
                     IDLE
```

The state machine lives in `RobotState` (mutex-protected). The app layer is
the only thing that calls `setMode()`. E-stop can override any state to
`kEmergency` directly via `RobotState::setEstop(true)`.
