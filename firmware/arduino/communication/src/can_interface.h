/**
 * @file    can_interface.h
 * @brief   CAN wrapper for the bridge node with RX forwarding.
 */
#ifndef COMM_NODE_CAN_INTERFACE_H
#define COMM_NODE_CAN_INTERFACE_H
#include <stdint.h>
#include "RobotCommon.h"

class UsbBridge;

class CanInterface
{
public:
    void begin(CanNodeId nodeId);
    void poll(uint32_t nowMs);
    void send(uint32_t id, const uint8_t *data, uint8_t len);
    void sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults);
    void setBridge(UsbBridge *bridge) { m_bridge = bridge; }
private:
    UsbBridge *m_bridge = 0;
    CanNodeId m_nodeId = CAN_NODE_COMM;
};
#endif /* COMM_NODE_CAN_INTERFACE_H */
