# Serial binary protocol

A compact, frame-based protocol for low-latency control from a host PC.
Transported over USB-CDC or UART0.

## Frame layout

```
┌─────┬─────┬──────┬─────────┬────────┬───────┐
│ STX │ LEN │ TYPE │ PAYLOAD │ CRC16  │ ETX   │
│ 1 B │ 1 B │ 1 B  │ 0..250 B│ 2 B    │ 1 B   │
└─────┴─────┴──────┴─────────┴────────┴───────┘
```

| Field    | Bytes | Notes                                           |
| -------- | ----- | ------------------------------------------------- |
| `STX`    | 1     | Constant `0xAA` — start-of-frame sentinel         |
| `LEN`    | 1     | Payload byte count, ≤ 250                         |
| `TYPE`   | 1     | Opcode, see `MsgType` in `serial_protocol.hpp`     |
| `PAYLOAD`| 0..250| Opaque bytes; host & firmware agree per opcode    |
| `CRC16`  | 2     | CCITT-FALSE polynomial (0x1021) over (TYPE|PAYLOAD) |
| `ETX`    | 1     | Constant `0x55` — end-of-frame sentinel            |

All multi-byte integer fields are **little-endian**. Floats are IEEE-754
single-precision, also little-endian (ESP32 native).

## Opcodes

| Hex  | Name        | Direction | Payload                                       |
| ---- | ----------- | --------- | --------------------------------------------- |
| 0x01 | `kPing`     | Host→Bot  | (none)                                        |
| 0x02 | `kPong`     | Bot→Host  | (none) — auto-replied to `kPing`               |
| 0x10 | `kCmdVel`   | Host→Bot  | `float linear_mps`, `float angular_rps` (8 B) |
| 0x11 | `kSetMode`  | Host→Bot  | `uint8 mode` (see `RobotMode`)                 |
| 0x12 | `kEstop`    | Host→Bot  | (none) — triggers e-stop, requires manual reset |
| 0x80 | `kTelemetry`| Bot→Host  | JSON snapshot (UTF-8)                          |
| 0x81 | `kHealth`   | Bot→Host  | JSON health report                              |
| 0xFF | `kAck`      | Bot→Host  | `uint8 ack_type` echoing the op being acked   |

## Example frames

### Ping

```
AA 00 01 7E 81 55
└─┘ ─┘ ─┘ ──────┘ ─┘
STX LEN TYPE CRC16   ETX
       (CRC of [0x01])
```

### CmdVel (linear=0.2 m/s, angular=0.0 rad/s)

```
AA 08 10 CD CC 4C 3E 00 00 00 00 [crc_lo crc_hi] 55
└─┘ ─┘ ─┘ └─────────────────────────────┘ ─────────┘ ─┘
STX LEN TYPE linear (0.2f, little-endian)     CRC16     ETX
              angular (0.0f, little-endian)
```

## Telemetry JSON schema

```json
{
  "t": 12345,
  "mode": "TELEOP",
  "v":  {"lin": 0.21, "ang": -0.05},
  "tgt":{"lin": 0.20, "ang":  0.00},
  "wheels": {"l_rpm": 58.3, "r_rpm": 62.1,
             "l_count": 12345, "r_count": 12987},
  "imu": {"h": 1.570, "p": 0.01, "r": -0.02, "cal": true},
  "sonar": 0.42,
  "batt": 7.92,
  "heap": 215000,
  "errors": 0, "warns": 3
}
```

## CRC16-CCITT implementation reference

```c
uint16_t crc16_ccitt(const uint8_t* p, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}
```
