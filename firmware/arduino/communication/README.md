# communication (CAN node 4)

Jembatan CAN ↔ USB console. Serial1 (pin 18/19, 921600 baud) membawa
protokol frame yang sama dengan master; setiap frame CAN di bus dimirror
ke PC sebagai `MSG_CAN_BRIDGE_RX` (11 B: id u16, dlc, 8 B data), dan
`MSG_CAN_BRIDGE_TX` dari PC dikirim ke bus.

Sketsa ini juga berguna sebagai CAN sniffer saat debugging node lain.
