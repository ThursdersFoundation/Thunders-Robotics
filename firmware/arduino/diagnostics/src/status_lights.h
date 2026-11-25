/**
 * @file    status_lights.h
 * @brief   RGB LED + buzzer patterns for the diagnostics panel.
 */
#ifndef DIAG_NODE_STATUS_LIGHTS_H
#define DIAG_NODE_STATUS_LIGHTS_H
#include <stdint.h>
#include "heartbeat_monitor.h"

class StatusLights
{
public:
    void begin(uint8_t r, uint8_t g, uint8_t b, uint8_t buzzer);
    void show(HeartbeatMonitor::Health h);
private:
    uint8_t m_r, m_g, m_b, m_buzzer;
    uint32_t m_lastToggle = 0;
    bool m_blinkState = false;
};
#endif /* DIAG_NODE_STATUS_LIGHTS_H */
