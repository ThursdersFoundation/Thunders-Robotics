# actuator_controller (CAN node 3)

Node aktuator: 6 servo (D6/D7/D8/D44/D45/D46) dan 2 stepper A4988
(step D26/D27, dir D24/D25, enable D28).

Perintah: `0x110` servo (id + sudut x0.1°), `0x111` stepper (axis, posisi
int32 langkah, kecepatan int16 langkah/s). Status sudut 6 servo dikirim
setiap 100 ms pada `0x2E0`.
