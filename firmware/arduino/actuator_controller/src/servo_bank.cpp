/** @file servo_bank.cpp */
#include <Arduino.h>
#include "servo_bank.h"

const uint8_t ServoBank::kPins[6] = { 6, 7, 8, 44, 45, 46 };

void ServoBank::begin()
{
    for (uint8_t i = 0U; i < 6U; i++)
    {
        m_servo[i].attach(kPins[i]);
        m_angle[i] = 90.0f;
        m_servo[i].write(90U);
    }
}

void ServoBank::setAngle(uint8_t idx, float angleDeg)
{
    if (idx >= 6U) { return; }
    if (angleDeg > 180.0f) { angleDeg = 180.0f; }
    if (angleDeg < 0.0f)   { angleDeg = 0.0f; }
    m_angle[idx] = angleDeg;
    m_servo[idx].write((uint8_t)(angleDeg + 0.5f));
}

void ServoBank::fillStatus(uint8_t out[8])
{
    for (uint8_t i = 0U; i < 6U; i++) { out[i] = (uint8_t)(m_angle[i] + 0.5f); }
    out[6] = 0U;
    out[7] = 0U;
}
