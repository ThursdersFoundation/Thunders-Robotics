/** @file sensor_node can_interface.cpp */
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "can_interface.h"

static MCP_CAN s_mcpCan(10);
static const uint8_t kCanIntPin = 2U;

void CanInterface::begin(CanNodeId nodeId)
{
    m_nodeId = nodeId;
    while (s_mcpCan.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
    {
        delay(200);
    }
    s_mcpCan.setMode(MCP_NORMAL);
    pinMode(kCanIntPin, INPUT);
}

void CanInterface::poll()
{
    /* RX not needed on this node; keep FIFO drained to avoid overflow */
    uint32_t id; uint8_t len; uint8_t buf[8];
    while (!digitalRead(kCanIntPin))
    {
        if (s_mcpCan.readMsgBuf(&id, &len, buf) != CAN_OK) { break; }
    }
}

void CanInterface::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    s_mcpCan.sendMsgBuf(id, 0U, len, (uint8_t *)data);
}

void CanInterface::sendHeartbeat(uint8_t state, uint16_t uptimeS, uint16_t faults)
{
    uint8_t payload[5];
    (void)Can_EncodeHeartbeat(payload, state, uptimeS, faults);
    send((uint32_t)(CAN_ID_HEARTBEAT_BASE | m_nodeId), payload, 5U);
}
