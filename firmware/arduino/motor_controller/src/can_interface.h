/**
 * @file    can_interface.h
 * @brief   MCP2515 CAN wrapper at 500 kbit/s (8 MHz crystal).
 *
 * Wraps the common autowp/arduino-mcp2515 driver; the sketch ships with
 * MCP_CAN_lib. RX frames land in a small ring handled by poll(); TX goes
 * out immediately (send-and-forget with retry on FAIL).
 */
#ifndef MOTOR_NODE_CAN_INTERFACE_H
#define MOTOR_NODE_CAN_INTERFACE_H

#include <stdint.h>
#include "RobotCommon.h"

class CommandHandler;   /* fwd: avoids a circular include */

class CanInterface
{
public:
    void begin(CanNodeId nodeId);
    /** Drain RX FIFO, dispatch frames, retry queued TX. Call every loop. */
    void poll(uint32_t nowMs);
    void sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults);
    void sendMotorTelemetry(const volatile int32_t counts[4], const float duty[4],
                            int16_t currentMa);
    void sendFault(uint16_t code, uint16_t argument);
    void setHandler(CommandHandler *handler) { m_handler = handler; }

private:
    CommandHandler *m_handler = 0;
    CanNodeId m_nodeId = CAN_NODE_MOTOR;
    uint32_t m_lastBeatMs = 0;
    uint8_t m_txRetry = 0;
};
#endif /* MOTOR_NODE_CAN_INTERFACE_H */
