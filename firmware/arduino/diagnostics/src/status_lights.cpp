/** @file status_lights.cpp */
#include <Arduino.h>
#include "status_lights.h"

void StatusLights::begin(uint8_t r, uint8_t g, uint8_t b, uint8_t buzzer)
{
    m_r = r; m_g = g; m_b = b; m_buzzer = buzzer;
    pinMode(r, OUTPUT); pinMode(g, OUTPUT); pinMode(b, OUTPUT);
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, LOW);
}

void StatusLights::show(HeartbeatMonitor::Health h)
{
    uint32_t now = millis();
    if ((now - m_lastToggle) >= 250U)
    {
        m_lastToggle = now;
        m_blinkState = !m_blinkState;
    }

    switch (h)
    {
    case HeartbeatMonitor::HEALTH_ALL_OK:
        digitalWrite(m_r, LOW);
        digitalWrite(m_g, HIGH);
        digitalWrite(m_b, LOW);
        digitalWrite(m_buzzer, LOW);
        break;
    case HeartbeatMonitor::HEALTH_DEGRADED:
        /* yellow blink: red + green alternating */
        digitalWrite(m_r, m_blinkState ? HIGH : LOW);
        digitalWrite(m_g, m_blinkState ? LOW : HIGH);
        digitalWrite(m_b, LOW);
        digitalWrite(m_buzzer, LOW);
        break;
    case HeartbeatMonitor::HEALTH_CRITICAL:
        digitalWrite(m_r, HIGH);
        digitalWrite(m_g, LOW);
        digitalWrite(m_b, LOW);
        digitalWrite(m_buzzer, m_blinkState ? HIGH : LOW);   /* 2 Hz beep */
        break;
    }
}
