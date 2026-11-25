# motor_controller (CAN node 1)

Node aktuator roda pada Arduino Mega 2560: menerima kecepatan roda dari
master via CAN (`0x100`), menjalankan PID kecepatan 100 Hz per roda, dan
menggerakkan dua TB6612FNG (4 motor).

## Wiring (lihat docs/pinmaps.md)
- MCP2515: CS D10, INT D2, SPI D11-D13, crystal 8 MHz
- TB6612: PWM D5/D6/D46/D9, IN D4/D7/D8/D12/D44/D45/D22/D23, STBY D24
- Encoder: D18/D19 (ENC1), D20/D21 (ENC2), D2/D3+D14/D15 (ENC3/4, poling)

## Telemetry yang dikirim
| CAN ID | Isi | Frekuensi |
| --- | --- | --- |
| 0x200 / 0x201 | encoder tick (2x int32) | 50 Hz |
| 0x210 | duty 4 motor (int16 x1e-3) | 50 Hz |
| 0x211 | arus (mA) | 50 Hz |
| 0x0A1 | heartbeat (state, uptime, faults) | 10 Hz |

## Catatan desain
- Kecepatan encoder dihitung dari delta 16/32-bit pada 100 Hz; `updateSpeeds`
  dipanggil setelah PID agar PID memakai data baru.
- Timeout perintah 500 ms: master berhenti mengirim → target dinolkan dan
  fault `COMM_TIMEOUT` dipublish.
- Dua encoder pertama dekode 4x di interrupt; ENC3/4 poling (Mega punya 6
  eksternal interrupt, sisanya dipakai CAN INT).
