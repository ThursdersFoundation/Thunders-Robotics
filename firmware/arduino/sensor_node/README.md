# sensor_node (CAN node 2)

Node sensor pada Mega 2560: 2x HC-SR04 (trigger/echo D22-D25) dan MPU6050
(I2C pin 20/21, 400 kHz).

Telemetri: `0x220` jarak 4 ch (2 valid + 2 filler 0xFFFF) tiap 20 ms,
`0x221` yaw + gyro tiap 10 ms, heartbeat `0x0A2` tiap 100 ms.

Catatan: pengukuran echo poling dengan timeout 25 ms; sensor di-trigger
bergantian tiap 20 ms untuk menghindari cross-talk akustik.
