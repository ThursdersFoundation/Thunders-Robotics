/**
 * @file    usb_bridge.h
 * @brief   Serial1 <-> CAN bridge: frame protocol on Serial1, CAN on the bus.
 */
#ifndef COMM_NODE_USB_BRIDGE_H
#define COMM_NODE_USB_BRIDGE_H
#include <stdint.h>
#include "RobotCommon.h"

class CanInterface;

class UsbBridge
{
public:
    void begin(CanInterface *can);
    void onByte(uint8_t byte);
    void onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc);
    void tick(uint32_t nowMs);
private:
    void sendFrame(uint8_t type, const uint8_t *payload, uint8_t len);
    CanInterface *m_can = 0;
    PacketParser  m_parser;
    uint32_t      m_lastBeatMs = 0;
    uint32_t      m_txCount = 0;
};
#endif /* COMM_NODE_USB_BRIDGE_H */
