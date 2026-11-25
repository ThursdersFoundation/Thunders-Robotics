/** @file diag_can.cpp */
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "diag_can.h"
#include "heartbeat_monitor.h"

static MCP_CAN s_mcpCan(10);
static const uint8_t kCanIntPin = 2U;

void DiagCan::begin(CanNodeId nodeId, HeartbeatMonitor *monitor)
{
    m_nodeId = nodeId;
    m_monitor = monitor;
    while (s_mcpCan.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(200); }
    s_mcpCan.setMode(MCP_NORMAL);
    pinMode(kCanIntPin, INPUT);
}

void DiagCan::poll(uint32_t nowMs)
{
    (void)nowMs;
    uint32_t id;
    uint8_t len = 0U;
    uint8_t buf[8];
    while (!digitalRead(kCanIntPin))
    {
        if (s_mcpCan.readMsgBuf(&id, &len, buf) != CAN_OK) { break; }
        if (m_monitor != NULL)
        {
            m_monitor->onCanFrame(id, buf, len);
        }
    }
}
