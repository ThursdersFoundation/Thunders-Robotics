/**
 * @file serial_protocol.hpp
 * @brief Compact binary command/telemetry protocol over USB-CDC / UART0.
 *
 * Frame format (big-endian):
 * ┌─────┬─────┬──────┬─────────┬────────┬───────┐
 * │ STX │ LEN │ TYPE │ PAYLOAD │ CRC16  │ ETX   │
 * │ 1B  │ 1B  │ 1B   │ 0..n B  │ 2B     │ 1B    │
 * └─────┴─────┴──────┴─────────┴────────┴───────┘
 *   STX = 0xAA
 *   LEN = payload byte count (max serial::kMaxFrameBytes - 5)
 *   TYPE = command/telemetry opcode (see `MsgType` below)
 *   PAYLOAD = opaque bytes (escaped if equal to STX/ETX/ESC)
 *   CRC16 = CCITT-FALSE over (LEN | TYPE | PAYLOAD)
 *   ETX = 0x55
 *
 * @copyright Copyright (c) 2025 Thunders Robotic. All rights reserved.
 */
#ifndef THUNDERS_SERIAL_PROTOCOL_HPP
#define THUNDERS_SERIAL_PROTOCOL_HPP

#include <cstdint>

namespace thunders::communication {

class SerialProtocol {
public:
    enum class MsgType : std::uint8_t {
        kPing          = 0x01,
        kPong          = 0x02,
        kCmdVel        = 0x10,  // payload: float linear_mps, float angular_rps
        kSetMode       = 0x11,  // payload: uint8 mode
        kEstop         = 0x12,
        kTelemetry     = 0x80,  // payload: JSON snapshot
        kHealth        = 0x81,
        kAck           = 0xFF,
    };

    SerialProtocol();
    ~SerialProtocol();

    /// Start the serial port at the configured baud.
    void begin();

    /// Pump the RX state machine and dispatch any completed frames.
    void loop();

    /// Send a framed message to the host.
    void send(MsgType type, const std::uint8_t* payload, std::uint16_t len);

    /// Convenience: send a preformatted JSON telemetry snapshot.
    void sendTelemetry(const char* json);

private:
    bool              initialised_ = false;
    std::uint8_t      rxState_ = 0;       // 0=STX, 1=LEN, 2=TYPE, 3=PAYLOAD, 4=CRC, 5=ETX
    std::uint8_t      rxLen_   = 0;
    std::uint8_t      rxType_  = 0;
    std::uint8_t      rxIdx_   = 0;
    std::uint8_t      rxBuf_[256];
    std::uint16_t     rxCrc_   = 0;

    void dispatch_();
    void handleFrame_(MsgType type, const std::uint8_t* p, std::uint8_t len);
    std::uint16_t crc16Ccitt_(const std::uint8_t* p, std::uint16_t n) const;
};

}  // namespace thunders::communication

#endif  // THUNDERS_SERIAL_PROTOCOL_HPP
