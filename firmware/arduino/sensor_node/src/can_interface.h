/**
 * @file    can_interface.h
 * @brief   Shared MCP2515 wrapper (same layout as motor node, generic send).
 */
#ifndef SENSOR_NODE_CAN_INTERFACE_H
#define SENSOR_NODE_CAN_INTERFACE_H
#include <stdint.h>
#include "RobotCommon.h"

class CanInterface
{
public:
    void begin(CanNodeId nodeId);
    void poll();
    void send(uint32_t id, const uint8_t *data, uint8_t len);
    void sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults);
private:
    CanNodeId m_nodeId = CAN_NODE_SENSOR;
};
#endif /* SENSOR_NODE_CAN_INTERFACE_H */
