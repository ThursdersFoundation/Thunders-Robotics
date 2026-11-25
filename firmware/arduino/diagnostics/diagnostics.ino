/**
 * @file    diagnostics.ino
 * @brief   Thunders Robotic diagnostics node (CAN node 5).
 *
 * Watches heartbeats of every node (master + 4 nodes); lights the RGB LED
 * green when all are fresh, yellow when one is stale, red + buzzer when a
 * required node is missing for > 2 s.
 */
#include "RobotCommon.h"
#include "src/diag_can.h"
#include "src/heartbeat_monitor.h"
#include "src/status_lights.h"

static DiagCan s_can;
static HeartbeatMonitor s_monitor;
static StatusLights s_lights;

static uint32_t s_bootMs = 0;

void setup()
{
    Serial.begin(115200UL);
    s_can.begin(CAN_NODE_DIAGNOSTICS, &s_monitor);
    s_lights.begin(5, 6, 7, 8);
    s_monitor.begin();
    s_bootMs = millis();
}

void loop()
{
    uint32_t now = millis();
    s_can.poll(now);

    HeartbeatMonitor::Health h = s_monitor.evaluate(now);
    s_lights.show(h);

    static uint32_t lastReport = 0U;
    if ((now - lastReport) >= 5000U)
    {
        lastReport = now;
        Serial.print(F("[diag] nodes ok="));
        Serial.print(s_monitor.aliveCount());
        Serial.print(F(" stale="));
        Serial.println(s_monitor.staleCount());
    }
}
