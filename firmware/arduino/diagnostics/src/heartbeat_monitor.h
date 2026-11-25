/**
 * @file    heartbeat_monitor.h
 * @brief   Freshness tracking for the 5 robot CAN heartbeats.
 */
#ifndef DIAG_NODE_HEARTBEAT_H
#define DIAG_NODE_HEARTBEAT_H
#include <stdint.h>
#include "RobotCommon.h"

class HeartbeatMonitor
{
public:
    typedef enum { HEALTH_ALL_OK = 0, HEALTH_DEGRADED, HEALTH_CRITICAL } Health;
    void begin();
    void onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc);
    Health evaluate(uint32_t nowMs);
    uint8_t aliveCount() const;
    uint8_t staleCount() const;
private:
    uint32_t m_lastSeen[6];
    bool     m_everSeen[6];
    uint16_t m_faults[6];
};
#endif /* DIAG_NODE_HEARTBEAT_H */
