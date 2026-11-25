/**
 * @file    diag_can.h
 * @brief   CAN wrapper for the diagnostics node (heartbeat reception).
 */
#ifndef DIAG_NODE_CAN_H
#define DIAG_NODE_CAN_H
#include <stdint.h>
#include "RobotCommon.h"

class HeartbeatMonitor;

class DiagCan
{
public:
    void begin(CanNodeId nodeId, HeartbeatMonitor *monitor);
    void poll(uint32_t nowMs);
private:
    HeartbeatMonitor *m_monitor = 0;
    CanNodeId m_nodeId = CAN_NODE_DIAGNOSTICS;
};
#endif /* DIAG_NODE_CAN_H */
