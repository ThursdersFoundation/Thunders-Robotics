# MQTT topic namespace & payload schemas

## Topic hierarchy

All topics follow the pattern:

```
thunders/<robot_id>/<category>[/...]
```

`<robot_id>` defaults to `thunders-001` and is set in
`include/app_config.hpp` (`kRobotName`).

### Subscribed topics

| Topic                              | Direction | Payload                                                  |
| ---------------------------------- | --------- | ------------------------------------------------------- |
| `thunders/<id>/command`            | Host→Bot  | JSON command, see **Command payload** below             |

### Published topics

| Topic                              | Direction | QoS | Retain | Payload                              |
| ---------------------------------- | --------- | --- | ------ | ------------------------------------ |
| `thunders/<id>/telemetry`          | Bot→Host  | 0   | false  | JSON snapshot (10 Hz)                |
| `thunders/<id>/status`             | Bot→Host  | 1   | true   | `"online"` / `"offline"` (LWT)       |
| `thunders/<id>/health`             | Bot→Host  | 1   | false  | JSON health (1 Hz)                   |
| `thunders/<id>/heartbeat`          | Bot→Host  | 0   | false  | `{"t": 12345}` (1 Hz)                |

## Command payload (`command`)

A single JSON object with one of the following `op` values:

```json
// Set wheel velocities
{"op": "cmd_vel", "linear": 0.2, "angular": -0.05}

// Change operating mode
{"op": "set_mode", "mode": "autonomous"}

// Trigger software e-stop (requires manual reset)
{"op": "estop"}

// Reset the e-stop latch (only valid if kRequireManualResetOnEstop=false)
{"op": "reset"}

// Ping
{"op": "ping"}
```

The firmware replies with a JSON `ack` on the `command` topic when applicable.

## Telemetry payload (`telemetry`)

Published at 10 Hz. See `docs/serial_protocol.md` for the JSON schema —
the MQTT and serial payloads share the same JSON shape so that downstream
consumers (dashboards, loggers) can ingest both without translation.

## Last Will & Testament

The firmware connects with an LWT publishing `offline` to the `status`
topic. If the firmware disconnects ungracefully (power loss, network drop),
the broker retains `offline` so subscribers can detect the loss immediately.

## QoS notes

* `telemetry` at QoS 0 — high rate, occasional drops are acceptable.
* `status` at QoS 1, retained — must not be lost. Retention lets new
  subscribers know the current state without waiting for the next publish.
* `command` at QoS 1 — commands must arrive exactly once, no duplicates.
