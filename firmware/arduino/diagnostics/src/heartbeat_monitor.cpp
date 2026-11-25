/** @file heartbeat_monitor.cpp */
#include <Arduino.h>
#include "heartbeat_monitor.h"

static const uint32_t kStaleMs = 500U;    /* yellow */
static const uint32_t kCriticalMs = 2000U;/* red + buzzer */

void HeartbeatMonitor::begin()
{
    for (uint8_t i = 0U; i < 6U; i++)
    {
        m_lastSeen[i] = 0U;
        m_everSeen[i] = false;
        m_faults[i] = 0U;
    }
}

void HeartbeatMonitor::onCanFrame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if ((id & 0xFF0U) != CAN_ID_HEARTBEAT_BASE) { return; }
    uint8_t node = (uint8_t)(id & 0x0FU);
    if (node >= 6U) { return; }
    m_lastSeen[node] = millis();
    m_everSeen[node] = true;
    /* payload: state u8, uptime u16, faults u16 */
    if (dlc >= 5U)
    {
        m_faults[node] = can_get_u16(&data[3]);
    }
}

HeartbeatMonitor::Health HeartbeatMonitor::evaluate(uint32_t nowMs)
{
    uint8_t stale = 0U;
    bool critical = false;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (!m_everSeen[i]) { continue; }   /* not yet on the bus */
        uint32_t age = nowMs - m_lastSeen[i];
        if (age > kCriticalMs) { critical = true; }
        else if (age > kStaleMs) { stale++; }
    }
    if (critical)        { return HEALTH_CRITICAL; }
    if (stale > 0U)      { return HEALTH_DEGRADED; }
    return HEALTH_ALL_OK;
}

uint8_t HeartbeatMonitor::aliveCount() const
{
    uint8_t n = 0U;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (m_everSeen[i]) { n++; }
    }
    return n;
}

uint8_t HeartbeatMonitor::staleCount() const
{
    uint8_t n = 0U;
    for (uint8_t i = 0U; i < 6U; i++)
    {
        if (m_everSeen[i] && (millis() - m_lastSeen[i]) > kStaleMs) { n++; }
    }
    return n;
}
